// SPDX-License-Identifier: GPL-2.0
/*
 * Synopsys DesignWare PCIe host controller driver
 *
 * Copyright (C) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Jingoo Han <jg1.han@samsung.com>
 *
 * Customizations for the NXP S32V PCIE driver
 * Copyright 2017-2019 NXP
 */

#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>

#include "../../pci.h"
#include "pcie-designware.h"

static struct pci_ops dw_pcie_ops;

static int dw_pcie_rd_own_conf(struct pcie_port *pp, int where, int size,
			       u32 *val)
{
	struct dw_pcie *pci;

	if (pp->ops->rd_own_conf)
		return pp->ops->rd_own_conf(pp, where, size, val);

	pci = to_dw_pcie_from_pp(pp);
	return dw_pcie_read(pci->dbi_base + where, size, val);
}

static int dw_pcie_wr_own_conf(struct pcie_port *pp, int where, int size,
			       u32 val)
{
	struct dw_pcie *pci;

	if (pp->ops->wr_own_conf)
		return pp->ops->wr_own_conf(pp, where, size, val);

	pci = to_dw_pcie_from_pp(pp);
	return dw_pcie_write(pci->dbi_base + where, size, val);
}

#ifdef CONFIG_PCI_DW_DMA
/* TODO: dw_pcie_*_rc functions should be removed,
 * and the new API used instead
 * TODO: dma related code should be moved to new files,
 * and pcie-designware*.* code cleaned up
 */
static inline void dw_pcie_readl_rc(struct pcie_port *pp, u32 reg, u32 *val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	*val = dw_pcie_readl_dbi(pci, PCI_INTERRUPT_LINE);
}
static inline void dw_pcie_writel_rc(struct pcie_port *pp, u32 val, u32 reg)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_dbi(pci, reg, val);
}
int dw_pcie_dma_write_en(struct pcie_port *pp)
{
	dw_pcie_writel_rc(pp, 0x1, PCIE_DMA_WRITE_ENGINE_EN);
	return 0;
}
int dw_pcie_dma_write_soft_reset(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, 0x0, PCIE_DMA_WRITE_ENGINE_EN);
	while (readl(pci->dbi_base + PCIE_DMA_WRITE_ENGINE_EN) == 1)
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
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, 0x0, PCIE_DMA_READ_ENGINE_EN);
	while (readl(pci->dbi_base + PCIE_DMA_READ_ENGINE_EN) == 1)
		;
	pp->rd_ch.status = DMA_CH_STOPPED;
	dw_pcie_writel_rc(pp, 0x1, PCIE_DMA_READ_ENGINE_EN);
	return 0;
}
void dw_pcie_dma_set_wr_remote_done_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, lower_32_bits(val),
		PCIE_DMA_WRITE_DONE_IMWR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val),
		PCIE_DMA_WRITE_DONE_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_CH_CONTROL1) | 0x10,
		PCIE_DMA_CH_CONTROL1);
}
void dw_pcie_dma_set_wr_remote_abort_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, lower_32_bits(val),
		PCIE_DMA_WRITE_ABORT_IMWR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val),
		PCIE_DMA_WRITE_ABORT_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_CH_CONTROL1) | 0x10,
		PCIE_DMA_CH_CONTROL1);
}
void dw_pcie_dma_set_rd_remote_done_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, lower_32_bits(val),
		PCIE_DMA_READ_DONE_IMWR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val),
		PCIE_DMA_READ_DONE_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_CH_CONTROL1) | 0x10,
		PCIE_DMA_CH_CONTROL1);
}
void dw_pcie_dma_set_rd_remote_abort_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, lower_32_bits(val),
		PCIE_DMA_READ_ABORT_IMWR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val),
		PCIE_DMA_READ_ABORT_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_CH_CONTROL1) | 0x10,
		PCIE_DMA_CH_CONTROL1);
}
void dw_pcie_dma_en_local_int(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_CH_CONTROL1) | 0x8,
		PCIE_DMA_CH_CONTROL1);
}
/* Interrupts mask and clear functions */
int dw_pcie_dma_clear_wr_done_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) & (~val),
		PCIE_DMA_WRITE_INT_MASK);
	return 0;
}
int dw_pcie_dma_clear_wr_abort_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) & ((~val)<<16),
		PCIE_DMA_WRITE_INT_MASK);
	return 0;
}
int dw_pcie_dma_clear_rd_done_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_READ_INT_MASK) & (~val),
		PCIE_DMA_READ_INT_MASK);
	return 0;
}
int dw_pcie_dma_clear_rd_abort_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_READ_INT_MASK) & ((~val)<<16),
		PCIE_DMA_READ_INT_MASK);
	return 0;
}

int dw_pcie_dma_set_wr_done_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) | val, PCIE_DMA_WRITE_INT_MASK);
	return 0;
}
int dw_pcie_dma_set_wr_abort_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) | val<<16, PCIE_DMA_WRITE_INT_MASK);
	return 0;
}

int dw_pcie_dma_set_rd_done_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) | val, PCIE_DMA_READ_INT_MASK);
	return 0;
}
int dw_pcie_dma_set_rd_abort_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) | val<<16, PCIE_DMA_READ_INT_MASK);
	return 0;
}

int dw_pcie_dma_clear_wr_done_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_CLEAR) | val, PCIE_DMA_WRITE_INT_CLEAR);
	return 0;
}
int dw_pcie_dma_clear_wr_abort_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_CLEAR) | val<<16, PCIE_DMA_WRITE_INT_CLEAR);
	return 0;
}
int dw_pcie_dma_clear_rd_done_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_READ_INT_CLEAR) | val, PCIE_DMA_READ_INT_CLEAR);
	return 0;
}
int dw_pcie_dma_clear_rd_abort_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
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
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	pp->wr_ch.status = DMA_CH_STOPPED;
	pp->rd_ch.status = DMA_CH_STOPPED;
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_INT_MASK);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_INT_CLEAR);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_DONE_IMWR_LOW);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_DONE_IMWR_HIGH);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_ABORT_IMWR_LOW);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_ABORT_IMWR_HIGH);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_CH01_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_CH23_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_CH45_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_CH67_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_READ_INT_MASK);
	writel(0, pci->dbi_base + PCIE_DMA_READ_INT_CLEAR);
	writel(0, pci->dbi_base + PCIE_DMA_READ_DONE_IMWR_LOW);
	writel(0, pci->dbi_base + PCIE_DMA_READ_DONE_IMWR_HIGH);
	writel(0, pci->dbi_base + PCIE_DMA_READ_ABORT_IMWR_LOW);
	writel(0, pci->dbi_base + PCIE_DMA_READ_ABORT_IMWR_HIGH);
	writel(0, pci->dbi_base + PCIE_DMA_READ_CH01_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_READ_CH23_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_READ_CH45_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_READ_CH67_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_CH_CONTROL1);
	writel(0, pci->dbi_base + PCIE_DMA_CH_CONTROL2);
	writel(0, pci->dbi_base + PCIE_DMA_TRANSFER_SIZE);
	writel(0, pci->dbi_base + PCIE_DMA_SAR_LOW);
	writel(0, pci->dbi_base + PCIE_DMA_SAR_HIGH);
	writel(0, pci->dbi_base + PCIE_DMA_DAR_LOW);
	writel(0, pci->dbi_base + PCIE_DMA_DAR_HIGH);
	writel(0, pci->dbi_base + PCIE_DMA_LLP_LOW);
	writel(0, pci->dbi_base + PCIE_DMA_LLP_HIGH);

}
void dma_set_list_ptr(struct pcie_port *pp, u8 direction,
	u32 phy_list_addr)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	writel(lower_32_bits(phy_list_addr), pci->dbi_base + PCIE_DMA_LLP_LOW);
	writel(upper_32_bits(phy_list_addr), pci->dbi_base + PCIE_DMA_LLP_HIGH);
}
static void dma_set_link_elem(u32 *ptr_list_base,
	u8 arr_sz, u32 phy_list_addr)
{
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
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	/* Program DMA regs for LL mode */
	writel(0x0, pci->dbi_base + PCIE_DMA_CH_CONTROL1);
	writel(0x1, pci->dbi_base + PCIE_DMA_WRITE_ENGINE_EN);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_INT_MASK);
	writel(0x10000, pci->dbi_base +
		PCIE_DMA_WRITE_LINKED_LIST_ERR_EN);
	writel(0, pci->dbi_base + PCIE_DMA_VIEWPORT_SEL);
	writel(0x04000300, pci->dbi_base + PCIE_DMA_CH_CONTROL1);

	/* Set pointer to start of first list */
	dma_set_list_ptr(pp, DMA_CH_WRITE, phy_list_addr);
	/* Ring doorbell */
	pp->wr_ch.status = DMA_CH_RUNNING;
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_DOORBELL);
}
EXPORT_SYMBOL(dw_start_dma_llw);

int dw_pcie_dma_start_linked_list(struct pcie_port *pp,
	u32 phy_list_addr,
	u8 direction)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

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
			writel(0x0, pci->dbi_base + PCIE_DMA_CH_CONTROL1);
			pp->wr_ch.phy_list_addr = phy_list_addr;
			writel(0x1, pci->dbi_base + PCIE_DMA_WRITE_ENGINE_EN);
			writel(0, pci->dbi_base + PCIE_DMA_WRITE_INT_MASK);
			writel(0x10000, pci->dbi_base +
			PCIE_DMA_WRITE_LINKED_LIST_ERR_EN);
			writel(0, pci->dbi_base + PCIE_DMA_VIEWPORT_SEL);
			writel(0x04000300, pci->dbi_base +
					PCIE_DMA_CH_CONTROL1);

			/* Set pointer to start of first list */
			dma_set_list_ptr(pp, direction, phy_list_addr);
			/* Ring doorbell */
			writel(0, pci->dbi_base + PCIE_DMA_WRITE_DOORBELL);
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
			writel(0x0, pci->dbi_base + PCIE_DMA_CH_CONTROL1);

			pp->rd_ch.phy_list_addr = phy_list_addr;
			writel(0x1, pci->dbi_base + PCIE_DMA_READ_ENGINE_EN);
			writel(0, pci->dbi_base + PCIE_DMA_READ_INT_MASK);
			writel(0x10000, pci->dbi_base +
				PCIE_DMA_READ_LINKED_LIST_ERR_EN);
			writel(0x80000000, pci->dbi_base +
				PCIE_DMA_VIEWPORT_SEL);
			writel(0x04000300, pci->dbi_base +
				PCIE_DMA_CH_CONTROL1);

			/* Set pointer to start of first list */
			dma_set_list_ptr(pp, direction, phy_list_addr);
			/* Ring doorbell */
			writel(0, pci->dbi_base + PCIE_DMA_READ_DOORBELL);
		} else {
			return -EBUSY;
		}
	}
	return 0;
}

int dw_pcie_dma_load_linked_list(struct pcie_port *pp,
	struct dma_list arr_ll[], u8 arr_sz, u32 phy_list_addr,
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
			arr_ll[i].sar, arr_ll[i].dar,
			arr_ll[i].size, 0);
		ptr_ch->current_sar = arr_ll[i].sar;
		ptr_ch->current_dar = arr_ll[i].dar;
		ptr_ch->current_size = arr_ll[i].size;
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
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	/* Invalid channel number */
	if (dma_single_rw->ch_num > PCIE_DMA_NR_CH - 1)
		return -EINVAL;

	/* Invalid transfer size */
	if (dma_single_rw->size > PCIE_DMA_MAX_SIZE)
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
	writel(0x0, pci->dbi_base + PCIE_DMA_CH_CONTROL1);

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
		writel(0, pci->dbi_base + PCIE_DMA_WRITE_DOORBELL);
	else
		writel(0, pci->dbi_base + PCIE_DMA_READ_DOORBELL);

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
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_readl_rc(pp, PCIE_DMA_WRITE_INT_STATUS, &val_write);
	dw_pcie_readl_rc(pp, PCIE_DMA_READ_INT_STATUS, &val_read);

	if (val_write) {
		if (pp->wr_ch.status == DMA_CH_RUNNING) {
			if (val_write & 0x10000) { /* Abort interrupt */
				/* Get error type */
				dw_pcie_dma_check_errors(pp,
					DMA_FLAG_WRITE_ELEM,
					&pp->wr_ch.errors);
					writel(0x00FF0000, pci->dbi_base +
						PCIE_DMA_WRITE_INT_CLEAR);
				err_type = pp->wr_ch.errors;
			} else { /* Done interrupt */
				writel(0x000000FF, pci->dbi_base +
					PCIE_DMA_WRITE_INT_CLEAR);
				/* Check channel list mode */
			}
			pp->wr_ch.status = DMA_CH_STOPPED;
			#ifdef CONFIG_PCI_S32V234
			if (pci->ops->send_signal_to_user)
				pci->ops->send_signal_to_user(pp);
			#endif
		} else
			writel(0x00FF00FF, pci->dbi_base +
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
				writel(0x00FF0000, pci->dbi_base +
						PCIE_DMA_READ_INT_CLEAR);
				err_type = pp->rd_ch.errors;
			} else { /* Done interrupt */
				writel(0x000000FF, pci->dbi_base +
					PCIE_DMA_READ_INT_CLEAR);
				/* Check channel list mode */
			}
			pp->rd_ch.status = DMA_CH_STOPPED;
			#ifdef CONFIG_PCI_S32V234
			if (pci->ops->send_signal_to_user)
				pci->ops->send_signal_to_user(pp);
			#endif
		} else
			writel(0x00FF00FF, pci->dbi_base +
				PCIE_DMA_READ_INT_CLEAR);
	}
	if (pp->ptr_func)
		pp->ptr_func(err_type);

	return ret;
}
#endif /* CONFIG_PCI_DW_DMA */

static void dw_msi_ack_irq(struct irq_data *d)
{
	irq_chip_ack_parent(d);
}

static void dw_msi_mask_irq(struct irq_data *d)
{
	pci_msi_mask_irq(d);
	irq_chip_mask_parent(d);
}

static void dw_msi_unmask_irq(struct irq_data *d)
{
	pci_msi_unmask_irq(d);
	irq_chip_unmask_parent(d);
}

static struct irq_chip dw_pcie_msi_irq_chip = {
	.name = "PCI-MSI",
	.irq_ack = dw_msi_ack_irq,
	.irq_mask = dw_msi_mask_irq,
	.irq_unmask = dw_msi_unmask_irq,
};

static struct msi_domain_info dw_pcie_msi_domain_info = {
	.flags	= (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		   MSI_FLAG_PCI_MSIX | MSI_FLAG_MULTI_PCI_MSI),
	.chip	= &dw_pcie_msi_irq_chip,
};

/* MSI int handler */
irqreturn_t dw_handle_msi_irq(struct pcie_port *pp)
{
	int i, pos, irq;
	u32 val, num_ctrls;
	irqreturn_t ret = IRQ_NONE;

	num_ctrls = pp->num_vectors / MAX_MSI_IRQS_PER_CTRL;

	for (i = 0; i < num_ctrls; i++) {
		dw_pcie_rd_own_conf(pp, PCIE_MSI_INTR0_STATUS +
					(i * MSI_REG_CTRL_BLOCK_SIZE),
				    4, &val);
		if (!val)
			continue;

		ret = IRQ_HANDLED;
		pos = 0;
		while ((pos = find_next_bit((unsigned long *) &val,
					    MAX_MSI_IRQS_PER_CTRL,
					    pos)) != MAX_MSI_IRQS_PER_CTRL) {
			irq = irq_find_mapping(pp->irq_domain,
					       (i * MAX_MSI_IRQS_PER_CTRL) +
					       pos);
			generic_handle_irq(irq);
			pos++;
		}
	}

	return ret;
}

/* Chained MSI interrupt service routine */
static void dw_chained_msi_isr(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct pcie_port *pp;

	chained_irq_enter(chip, desc);

	pp = irq_desc_get_handler_data(desc);
	dw_handle_msi_irq(pp);

	chained_irq_exit(chip, desc);
}

static void dw_pci_setup_msi_msg(struct irq_data *data, struct msi_msg *msg)
{
	struct pcie_port *pp = irq_data_get_irq_chip_data(data);
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	u64 msi_target;

	if (pp->ops->get_msi_addr)
		msi_target = pp->ops->get_msi_addr(pp);
	else
		msi_target = (u64)pp->msi_data;

	msg->address_lo = lower_32_bits(msi_target);
	msg->address_hi = upper_32_bits(msi_target);

	if (pp->ops->get_msi_data)
		msg->data = pp->ops->get_msi_data(pp, data->hwirq);
	else
		msg->data = data->hwirq;

	dev_dbg(pci->dev, "msi#%d address_hi %#x address_lo %#x\n",
		(int)data->hwirq, msg->address_hi, msg->address_lo);
}

static int dw_pci_msi_set_affinity(struct irq_data *irq_data,
				   const struct cpumask *mask, bool force)
{
	return -EINVAL;
}

static void dw_pci_bottom_mask(struct irq_data *data)
{
	struct pcie_port *pp = irq_data_get_irq_chip_data(data);
	unsigned int res, bit, ctrl;
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	if (pp->ops->msi_clear_irq) {
		pp->ops->msi_clear_irq(pp, data->hwirq);
	} else {
		ctrl = data->hwirq / MAX_MSI_IRQS_PER_CTRL;
		res = ctrl * MSI_REG_CTRL_BLOCK_SIZE;
		bit = data->hwirq % MAX_MSI_IRQS_PER_CTRL;

		pp->irq_status[ctrl] &= ~(1 << bit);
		dw_pcie_wr_own_conf(pp, PCIE_MSI_INTR0_MASK + res, 4,
				    ~pp->irq_status[ctrl]);
	}

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static void dw_pci_bottom_unmask(struct irq_data *data)
{
	struct pcie_port *pp = irq_data_get_irq_chip_data(data);
	unsigned int res, bit, ctrl;
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	if (pp->ops->msi_set_irq) {
		pp->ops->msi_set_irq(pp, data->hwirq);
	} else {
		ctrl = data->hwirq / MAX_MSI_IRQS_PER_CTRL;
		res = ctrl * MSI_REG_CTRL_BLOCK_SIZE;
		bit = data->hwirq % MAX_MSI_IRQS_PER_CTRL;

		pp->irq_status[ctrl] |= 1 << bit;
		dw_pcie_wr_own_conf(pp, PCIE_MSI_INTR0_MASK + res, 4,
				    ~pp->irq_status[ctrl]);
	}

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static void dw_pci_bottom_ack(struct irq_data *d)
{
	struct pcie_port *pp  = irq_data_get_irq_chip_data(d);
	unsigned int res, bit, ctrl;
	unsigned long flags;

	ctrl = d->hwirq / MAX_MSI_IRQS_PER_CTRL;
	res = ctrl * MSI_REG_CTRL_BLOCK_SIZE;
	bit = d->hwirq % MAX_MSI_IRQS_PER_CTRL;

	raw_spin_lock_irqsave(&pp->lock, flags);

	dw_pcie_wr_own_conf(pp, PCIE_MSI_INTR0_STATUS + res, 4, 1 << bit);

	if (pp->ops->msi_irq_ack)
		pp->ops->msi_irq_ack(d->hwirq, pp);

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static struct irq_chip dw_pci_msi_bottom_irq_chip = {
	.name = "DWPCI-MSI",
	.irq_ack = dw_pci_bottom_ack,
	.irq_compose_msi_msg = dw_pci_setup_msi_msg,
	.irq_set_affinity = dw_pci_msi_set_affinity,
	.irq_mask = dw_pci_bottom_mask,
	.irq_unmask = dw_pci_bottom_unmask,
};

static int dw_pcie_irq_domain_alloc(struct irq_domain *domain,
				    unsigned int virq, unsigned int nr_irqs,
				    void *args)
{
	struct pcie_port *pp = domain->host_data;
	unsigned long flags;
	u32 i;
	int bit;

	raw_spin_lock_irqsave(&pp->lock, flags);

	bit = bitmap_find_free_region(pp->msi_irq_in_use, pp->num_vectors,
				      order_base_2(nr_irqs));

	raw_spin_unlock_irqrestore(&pp->lock, flags);

	if (bit < 0)
		return -ENOSPC;

	for (i = 0; i < nr_irqs; i++)
		irq_domain_set_info(domain, virq + i, bit + i,
				    &dw_pci_msi_bottom_irq_chip,
				    pp, handle_edge_irq,
				    NULL, NULL);

	return 0;
}

static void dw_pcie_irq_domain_free(struct irq_domain *domain,
				    unsigned int virq, unsigned int nr_irqs)
{
	struct irq_data *data = irq_domain_get_irq_data(domain, virq);
	struct pcie_port *pp = irq_data_get_irq_chip_data(data);
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	bitmap_release_region(pp->msi_irq_in_use, data->hwirq,
			      order_base_2(nr_irqs));

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static const struct irq_domain_ops dw_pcie_msi_domain_ops = {
	.alloc	= dw_pcie_irq_domain_alloc,
	.free	= dw_pcie_irq_domain_free,
};

int dw_pcie_allocate_domains(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct fwnode_handle *fwnode = of_node_to_fwnode(pci->dev->of_node);

	pp->irq_domain = irq_domain_create_linear(fwnode, pp->num_vectors,
					       &dw_pcie_msi_domain_ops, pp);
	if (!pp->irq_domain) {
		dev_err(pci->dev, "Failed to create IRQ domain\n");
		return -ENOMEM;
	}

	pp->msi_domain = pci_msi_create_irq_domain(fwnode,
						   &dw_pcie_msi_domain_info,
						   pp->irq_domain);
	if (!pp->msi_domain) {
		dev_err(pci->dev, "Failed to create MSI domain\n");
		irq_domain_remove(pp->irq_domain);
		return -ENOMEM;
	}

	return 0;
}

void dw_pcie_free_msi(struct pcie_port *pp)
{
	irq_set_chained_handler(pp->msi_irq, NULL);
	irq_set_handler_data(pp->msi_irq, NULL);

	irq_domain_remove(pp->msi_domain);
	irq_domain_remove(pp->irq_domain);

	if (pp->msi_page)
		__free_page(pp->msi_page);
}

void dw_pcie_msi_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct device *dev = pci->dev;
	u64 msi_target;

	pp->msi_page = alloc_page(GFP_KERNEL);
	pp->msi_data = dma_map_page(dev, pp->msi_page, 0, PAGE_SIZE,
				    DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, pp->msi_data)) {
		dev_err(dev, "Failed to map MSI data\n");
		__free_page(pp->msi_page);
		pp->msi_page = NULL;
		return;
	}
	msi_target = (u64)pp->msi_data;

	/* Program the msi_data */
	dw_pcie_wr_own_conf(pp, PCIE_MSI_ADDR_LO, 4,
			    lower_32_bits(msi_target));
	dw_pcie_wr_own_conf(pp, PCIE_MSI_ADDR_HI, 4,
			    upper_32_bits(msi_target));
}

int dw_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct device *dev = pci->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *pdev = to_platform_device(dev);
	struct resource_entry *win, *tmp;
	struct pci_bus *bus, *child;
	struct pci_host_bridge *bridge;
	struct resource *cfg_res;
	int ret;

	raw_spin_lock_init(&pci->pp.lock);

	cfg_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "config");
	if (cfg_res) {
		pp->cfg0_size = resource_size(cfg_res) >> 1;
		pp->cfg1_size = resource_size(cfg_res) >> 1;
		pp->cfg0_base = cfg_res->start;
		pp->cfg1_base = cfg_res->start + pp->cfg0_size;
	} else if (!pp->va_cfg0_base) {
		dev_err(dev, "Missing *config* reg space\n");
	}

	bridge = pci_alloc_host_bridge(0);
	if (!bridge)
		return -ENOMEM;

	ret = devm_of_pci_get_host_bridge_resources(dev, 0, 0xff,
					&bridge->windows, &pp->io_base);
	if (ret)
		return ret;

	ret = devm_request_pci_bus_resources(dev, &bridge->windows);
	if (ret)
		goto error;

	/* Get the I/O and memory ranges from DT */
	resource_list_for_each_entry_safe(win, tmp, &bridge->windows) {
		switch (resource_type(win->res)) {
		case IORESOURCE_IO:
			ret = devm_pci_remap_iospace(dev, win->res,
						     pp->io_base);
			if (ret) {
				dev_warn(dev, "Error %d: failed to map resource %pR\n",
					 ret, win->res);
				resource_list_destroy_entry(win);
			} else {
				pp->io = win->res;
				pp->io->name = "I/O";
				pp->io_size = resource_size(pp->io);
				pp->io_bus_addr = pp->io->start - win->offset;
			}
			break;
		case IORESOURCE_MEM:
			pp->mem = win->res;
			pp->mem->name = "MEM";
			pp->mem_size = resource_size(pp->mem);
			pp->mem_bus_addr = pp->mem->start - win->offset;
			break;
		case 0:
			pp->cfg = win->res;
			pp->cfg0_size = resource_size(pp->cfg) >> 1;
			pp->cfg1_size = resource_size(pp->cfg) >> 1;
			pp->cfg0_base = pp->cfg->start;
			pp->cfg1_base = pp->cfg->start + pp->cfg0_size;
			break;
		case IORESOURCE_BUS:
			pp->busn = win->res;
			break;
		}
	}

	if (!pci->dbi_base) {
		pci->dbi_base = devm_pci_remap_cfgspace(dev,
						pp->cfg->start,
						resource_size(pp->cfg));
		if (!pci->dbi_base) {
			dev_err(dev, "Error with ioremap\n");
			ret = -ENOMEM;
			goto error;
		}
	}

	pp->mem_base = pp->mem->start;

	if (!pp->va_cfg0_base) {
		pp->va_cfg0_base = devm_pci_remap_cfgspace(dev,
					pp->cfg0_base, pp->cfg0_size);
		if (!pp->va_cfg0_base) {
			dev_err(dev, "Error with ioremap in function\n");
			ret = -ENOMEM;
			goto error;
		}
	}

	if (!pp->va_cfg1_base) {
		pp->va_cfg1_base = devm_pci_remap_cfgspace(dev,
						pp->cfg1_base,
						pp->cfg1_size);
		if (!pp->va_cfg1_base) {
			dev_err(dev, "Error with ioremap\n");
			ret = -ENOMEM;
			goto error;
		}
	}

	ret = of_property_read_u32(np, "num-viewport", &pci->num_viewport);
	if (ret)
		pci->num_viewport = 2;

	if (pci_msi_enabled()) {
		/*
		 * If a specific SoC driver needs to change the
		 * default number of vectors, it needs to implement
		 * the set_num_vectors callback.
		 */
		if (!pp->ops->set_num_vectors) {
			pp->num_vectors = MSI_DEF_NUM_VECTORS;
		} else {
			pp->ops->set_num_vectors(pp);

			if (pp->num_vectors > MAX_MSI_IRQS ||
			    pp->num_vectors == 0) {
				dev_err(dev,
					"Invalid number of vectors\n");
				goto error;
			}
		}

		if (!pp->ops->msi_host_init) {
			ret = dw_pcie_allocate_domains(pp);
			if (ret)
				goto error;

			if (pp->msi_irq)
				irq_set_chained_handler_and_data(pp->msi_irq,
							    dw_chained_msi_isr,
							    pp);
		} else {
			ret = pp->ops->msi_host_init(pp);
			if (ret < 0)
				goto error;
		}
	}

	if (pp->ops->host_init) {
		ret = pp->ops->host_init(pp);
		if (ret)
			goto err_free_msi;
	}

	pp->root_bus_nr = pp->busn->start;

	bridge->dev.parent = dev;
	bridge->sysdata = pp;
	bridge->busnr = pp->root_bus_nr;
	bridge->ops = &dw_pcie_ops;
	bridge->map_irq = of_irq_parse_and_map_pci;
	bridge->swizzle_irq = pci_common_swizzle;

	ret = pci_scan_root_bus_bridge(bridge);
	if (ret)
		goto err_free_msi;

	bus = bridge->bus;

	if (pp->ops->scan_bus)
		pp->ops->scan_bus(pp);

	pci_bus_size_bridges(bus);
	pci_bus_assign_resources(bus);

	list_for_each_entry(child, &bus->children, node)
		pcie_bus_configure_settings(child);

	pci_bus_add_devices(bus);
	return 0;

err_free_msi:
	if (pci_msi_enabled() && !pp->ops->msi_host_init)
		dw_pcie_free_msi(pp);
error:
	pci_free_host_bridge(bridge);
	return ret;
}

static int dw_pcie_rd_other_conf(struct pcie_port *pp, struct pci_bus *bus,
				 u32 devfn, int where, int size, u32 *val)
{
	int ret, type;
	u32 busdev, cfg_size;
	u64 cpu_addr;
	void __iomem *va_cfg_base;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	if (pp->ops->rd_other_conf)
		return pp->ops->rd_other_conf(pp, bus, devfn, where, size, val);

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 PCIE_ATU_FUNC(PCI_FUNC(devfn));

	if (bus->parent->number == pp->root_bus_nr) {
		type = PCIE_ATU_TYPE_CFG0;
		cpu_addr = pp->cfg0_base;
		cfg_size = pp->cfg0_size;
		va_cfg_base = pp->va_cfg0_base;
	} else {
		type = PCIE_ATU_TYPE_CFG1;
		cpu_addr = pp->cfg1_base;
		cfg_size = pp->cfg1_size;
		va_cfg_base = pp->va_cfg1_base;
	}

	dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX1,
				  type, cpu_addr,
				  busdev, cfg_size);
	ret = dw_pcie_read(va_cfg_base + where, size, val);
	if (pci->num_viewport <= 2)
		dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX1,
					  PCIE_ATU_TYPE_IO, pp->io_base,
					  pp->io_bus_addr, pp->io_size);

	return ret;
}

static int dw_pcie_wr_other_conf(struct pcie_port *pp, struct pci_bus *bus,
				 u32 devfn, int where, int size, u32 val)
{
	int ret, type;
	u32 busdev, cfg_size;
	u64 cpu_addr;
	void __iomem *va_cfg_base;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	if (pp->ops->wr_other_conf)
		return pp->ops->wr_other_conf(pp, bus, devfn, where, size, val);

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 PCIE_ATU_FUNC(PCI_FUNC(devfn));

	if (bus->parent->number == pp->root_bus_nr) {
		type = PCIE_ATU_TYPE_CFG0;
		cpu_addr = pp->cfg0_base;
		cfg_size = pp->cfg0_size;
		va_cfg_base = pp->va_cfg0_base;
	} else {
		type = PCIE_ATU_TYPE_CFG1;
		cpu_addr = pp->cfg1_base;
		cfg_size = pp->cfg1_size;
		va_cfg_base = pp->va_cfg1_base;
	}

	dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX1,
				  type, cpu_addr,
				  busdev, cfg_size);
	ret = dw_pcie_write(va_cfg_base + where, size, val);
	if (pci->num_viewport <= 2)
		dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX1,
					  PCIE_ATU_TYPE_IO, pp->io_base,
					  pp->io_bus_addr, pp->io_size);

	return ret;
}

static int dw_pcie_valid_device(struct pcie_port *pp, struct pci_bus *bus,
				int dev)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	/* If there is no link, then there is no device */
	if (bus->number != pp->root_bus_nr) {
		if (!dw_pcie_link_up(pci))
			return 0;
	}

	/* Access only one slot on each root port */
	if (bus->number == pp->root_bus_nr && dev > 0)
		return 0;

	return 1;
}

static int dw_pcie_rd_conf(struct pci_bus *bus, u32 devfn, int where,
			   int size, u32 *val)
{
	struct pcie_port *pp = bus->sysdata;

	if (!dw_pcie_valid_device(pp, bus, PCI_SLOT(devfn))) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (bus->number == pp->root_bus_nr)
		return dw_pcie_rd_own_conf(pp, where, size, val);

	return dw_pcie_rd_other_conf(pp, bus, devfn, where, size, val);
}

static int dw_pcie_wr_conf(struct pci_bus *bus, u32 devfn,
			   int where, int size, u32 val)
{
	struct pcie_port *pp = bus->sysdata;

	if (!dw_pcie_valid_device(pp, bus, PCI_SLOT(devfn)))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number == pp->root_bus_nr)
		return dw_pcie_wr_own_conf(pp, where, size, val);

	return dw_pcie_wr_other_conf(pp, bus, devfn, where, size, val);
}

static struct pci_ops dw_pcie_ops = {
	.read = dw_pcie_rd_conf,
	.write = dw_pcie_wr_conf,
};

static u8 dw_pcie_iatu_unroll_enabled(struct dw_pcie *pci)
{
	u32 val;

	val = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT);
	if (val == 0xffffffff)
		return 1;

	return 0;
}

void dw_pcie_setup_rc(struct pcie_port *pp)
{
	u32 val, ctrl, num_ctrls;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_setup(pci);

	num_ctrls = pp->num_vectors / MAX_MSI_IRQS_PER_CTRL;

	/* Initialize IRQ Status array */
	for (ctrl = 0; ctrl < num_ctrls; ctrl++) {
		dw_pcie_wr_own_conf(pp, PCIE_MSI_INTR0_MASK +
					(ctrl * MSI_REG_CTRL_BLOCK_SIZE),
				    4, ~0);
		dw_pcie_wr_own_conf(pp, PCIE_MSI_INTR0_ENABLE +
					(ctrl * MSI_REG_CTRL_BLOCK_SIZE),
				    4, ~0);
		pp->irq_status[ctrl] = 0;
	}

	/* Setup RC BARs */
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 0x00000004);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_1, 0x00000000);

	/* Setup interrupt pins */
	dw_pcie_dbi_ro_wr_en(pci);
	val = dw_pcie_readl_dbi(pci, PCI_INTERRUPT_LINE);
	val &= 0xffff00ff;
	val |= 0x00000100;
	dw_pcie_writel_dbi(pci, PCI_INTERRUPT_LINE, val);
	dw_pcie_dbi_ro_wr_dis(pci);

	/* Setup bus numbers */
	val = dw_pcie_readl_dbi(pci, PCI_PRIMARY_BUS);
	val &= 0xff000000;
	val |= 0x00ff0100;
	dw_pcie_writel_dbi(pci, PCI_PRIMARY_BUS, val);

	/* Setup command register */
	val = dw_pcie_readl_dbi(pci, PCI_COMMAND);
	val &= 0xffff0000;
	val |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
		PCI_COMMAND_MASTER | PCI_COMMAND_SERR;
	dw_pcie_writel_dbi(pci, PCI_COMMAND, val);

	/*
	 * If the platform provides ->rd_other_conf, it means the platform
	 * uses its own address translation component rather than ATU, so
	 * we should not program the ATU here.
	 */
	if (!pp->ops->rd_other_conf) {
		/* Get iATU unroll support */
		pci->iatu_unroll_enabled = dw_pcie_iatu_unroll_enabled(pci);
		dev_dbg(pci->dev, "iATU unroll: %s\n",
			pci->iatu_unroll_enabled ? "enabled" : "disabled");

		dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX0,
					  PCIE_ATU_TYPE_MEM, pp->mem_base,
					  pp->mem_bus_addr, pp->mem_size);
		if (pci->num_viewport > 2)
			dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX2,
						  PCIE_ATU_TYPE_IO, pp->io_base,
						  pp->io_bus_addr, pp->io_size);
	}

	dw_pcie_wr_own_conf(pp, PCI_BASE_ADDRESS_0, 4, 0);

	/* Enable write permission for the DBI read-only register */
	dw_pcie_dbi_ro_wr_en(pci);
	/* Program correct class for RC */
	dw_pcie_wr_own_conf(pp, PCI_CLASS_DEVICE, 2, PCI_CLASS_BRIDGE_PCI);
	/* Better disable write permission right after the update */
	dw_pcie_dbi_ro_wr_dis(pci);

	dw_pcie_rd_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, &val);
	val |= PORT_LOGIC_SPEED_CHANGE;
	dw_pcie_wr_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, val);
}
