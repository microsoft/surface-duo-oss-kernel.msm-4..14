/* SPDX-License-Identifier: GPL-2.0 */
/*
 * DMA support for the Synopsys DesignWare
 * PCIe host controller driver, customized
 * for the NXP S32V PCIE driver
 *
 * Copyright 2017-2020 NXP
 */

#ifndef _PCIE_DMA_S32V234_H
#define _PCIE_DMA_S32V234_H

#ifdef CONFIG_PCI_DW_DMA

#include "pcie-designware.h"

#define PCIE_DMA_BASE							0x970

/* Synopsys-specific PCIe configuration registers */
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

/* DW DMA Internal flags */
/* TODO: Number of channels and max size should come from a
 * kernel config node under CONFIG_PCI_DW_DMA
 */
#ifndef PCIE_DMA_NR_CH
/* Number of supported DMA channels */
#define PCIE_DMA_NR_CH		1
#endif
#ifndef PCIE_DMA_MAX_SIZE
/* Max DMA size, in KB */
#define PCIE_DMA_MAX_SIZE	(4 * 1024 * 1024)  /* 4GB */
#endif
#define DMA_FLAG_LIE         BIT(0)
#define DMA_FLAG_RIE         BIT(1)
#define DMA_FLAG_LLP         BIT(2)
#define DMA_FLAG_WRITE_ELEM			BIT(3)
#define DMA_FLAG_READ_ELEM			BIT(4)
#define DMA_FLAG_EN_DONE_INT		BIT(5)
#define DMA_FLAG_EN_ABORT_INT		BIT(6)
#define DMA_FLAG_EN_REMOTE_DONE_INT			BIT(7)
#define DMA_FLAG_EN_REMOTE_ABORT_INT		BIT(8)

enum DMA_CH_FLAGS {
	DMA_CH_STOPPED = 0,
	DMA_CH_RUNNING,
	DMA_CH_HALTED,
	DMA_CH_STOPPED_FATAL,
};
enum DMA_CH_DIR {
	DMA_CH_WRITE = 0,
	DMA_CH_READ
};
enum DMA_ERROR {
	DMA_ERR_NONE = 0,
	DMA_ERR_WR,
	DMA_ERR_RD,
	DMA_ERR_FETCH_LL,
	DMA_ERR_UNSUPPORTED_REQ,
	DMA_ERR_CPL_ABORT,
	DMA_ERR_CPL_TIMEOUT,
	DMA_ERR_DATA_POISIONING
};
/* Linked list mode struct */
struct dma_ll_info {
	u32 direction;
	u32 ch_num;
	u32 nr_elem;
	u32 phy_list_addr;
	u32 next_phy_list_addr;
};
/* Channel info struct */
struct dma_ch_info {
	u32 direction;
	u32	status;
	u32 errors;
	u32 phy_list_addr;
	u32 current_sar;
	u32 current_dar;
	u32 current_size;
	u32 *virt_addr;
	u8 current_elem_idx;
	u8 current_list_size;
};
/* Single block DMA transfer struct */
struct dma_data_elem {
	u64 sar;
	u64 dar;
	u64 imwr;
	u32 size;
	u32 flags;
	u32 ch_num;
};
/* Type of array of structures for passing linked list  */
struct dma_list {
	u64 sar;
	u64 dar;
	u32 size;
};

struct dma_info {
	struct dma_ch_info	wr_ch;
	struct dma_ch_info	rd_ch;
	struct dma_ll_info	ll_info;
	struct dma_list(*dma_linked_list)[];
#ifdef DMA_PTR_FUNC
	int (*ptr_func)(u32 arg);
#endif /* DMA_PTR_FUNC */
};


int dw_pcie_dma_write_en(struct dw_pcie *pci);
int dw_pcie_dma_read_en(struct dw_pcie *pci);
int dw_pcie_dma_write_soft_reset(struct dw_pcie *pci, struct dma_info *di);
int dw_pcie_dma_read_soft_reset(struct dw_pcie *pci, struct dma_info *di);
irqreturn_t dw_handle_dma_irq(struct dw_pcie *pci);
void dw_pcie_dma_set_wr_remote_done_int(struct dw_pcie *pci, u64 val);
void dw_pcie_dma_set_wr_remote_abort_int(struct dw_pcie *pci, u64 val);
void dw_pcie_dma_en_local_int(struct dw_pcie *pci/* , u64 val */);
void dw_pcie_dma_set_sar(struct dw_pcie *pci, u64 val);
void dw_pcie_dma_set_dar(struct dw_pcie *pci, u64 val);
void dw_pcie_dma_set_transfer_size(struct dw_pcie *pci, u32 val);
void dw_pcie_dma_set_rd_viewport(struct dw_pcie *pci, u8 ch_nr);
void dw_pcie_dma_set_wr_viewport(struct dw_pcie *pci, u8 ch_nr);
void dw_pcie_dma_clear_regs(struct dw_pcie *pci, struct dma_info *di);
int dw_pcie_dma_single_rw(struct dw_pcie *pci, struct dma_info *di,
	struct dma_data_elem *dma_single_rw);
int dw_pcie_dma_load_linked_list(struct dw_pcie *pci, struct dma_info *di,
	u8 arr_sz, u32 phy_list_addr,
	u32 next_phy_list_addr, u8 direction);
int dw_pcie_dma_start_linked_list(struct dw_pcie *pci, struct dma_info *di,
	u32 phy_list_addr,
	u8 direction);
void dw_pcie_dma_start_llw(struct dw_pcie *pci, struct dma_info *di, u64 phy_list_addr);
irqreturn_t dw_handle_dma_irq_write(struct dw_pcie *pci, struct dma_info *di, u32 val_write);
irqreturn_t dw_handle_dma_irq_read(struct dw_pcie *pci, struct dma_info *di, u32 val_read);


#endif /* CONFIG_PCI_DW_DMA */

#endif  /* _PCIE_DMA_S32V234_H */
