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

#ifndef _PCIE_DESIGNWARE_H
#define _PCIE_DESIGNWARE_H

/*
 * Maximum number of MSI IRQs can be 256 per controller. But keep
 * it 32 as of now. Probably we will never need more than 32. If needed,
 * then increment it in multiple of 32.
 */
#define MAX_MSI_IRQS			32
#define MAX_MSI_CTRLS			(MAX_MSI_IRQS / 32)

#ifdef CONFIG_PCI_DW_DMA
/* DW DMA Internal flags */
#define S32V_PCIE_DMA_NR_CH		1
#define S32V_PCIE_DMA_MAX_SIZE	(4 * 1024 * 1024)  /* 4G bytes */
#define DMA_FLAG_LIE         (1 << 0)
#define DMA_FLAG_RIE         (1 << 1)
#define DMA_FLAG_LLP         (1 << 2)
#define DMA_FLAG_WRITE_ELEM			(1 << 3)
#define DMA_FLAG_READ_ELEM			(1 << 4)
#define DMA_FLAG_EN_DONE_INT		(1 << 5)
#define DMA_FLAG_EN_ABORT_INT		(1 << 6)
#define DMA_FLAG_EN_REMOTE_DONE_INT			(1 << 7)
#define DMA_FLAG_EN_REMOTE_ABORT_INT		(1 << 8)
#endif

enum ATU_TYPE {
	ATU_TYPE_CFG0,
	ATU_TYPE_CFG1,
	ATU_TYPE_MEM,
	ATU_TYPE_IO,
	ATU_TYPE_MAX
};
#ifdef CONFIG_PCI_DW_DMA
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
#endif

struct pcie_port {
	struct device		*dev;
	u8			root_bus_nr;
	void __iomem		*dbi_base;
	u64			cfg0_base;
	u64			cfg0_mod_base;
	void __iomem		*va_cfg0_base;
	u32			cfg0_size;
	u64			cfg1_base;
	u64			cfg1_mod_base;
	void __iomem		*va_cfg1_base;
	u32			cfg1_size;
	u64			io_base;
	u64			io_mod_base;
	phys_addr_t		io_bus_addr;
	u32			io_size;
	u64			mem_base;
	u64			mem_mod_base;
	phys_addr_t		mem_bus_addr;
	u32			mem_size;
	struct resource		cfg;
	struct resource		io;
	struct resource		mem;
	struct resource		busn;
	int			irq;
	u32			lanes;
	struct pcie_host_ops	*ops;
	int			msi_irq;
	int			dma_irq;
	int			link_req_rst_not_irq;
	struct irq_domain	*irq_domain;
	unsigned long		msi_data;
	DECLARE_BITMAP(msi_irq_in_use, MAX_MSI_IRQS);
	u8			atu_idx[ATU_TYPE_MAX];
	#ifdef CONFIG_PCI_S32V234
	struct dentry		*dir;
	int			user_pid;
	struct siginfo	info;    /* signal information */
	void (*call_back)(u32);
	#endif
	#ifdef CONFIG_PCI_DW_DMA
	struct dma_ch_info	wr_ch;
	struct dma_ch_info	rd_ch;
	struct dma_ll_info	ll_info;
	struct dma_list(*dma_linked_list)[];
	int (*ptr_func)(u32 arg);
	#endif

};

struct pcie_host_ops {
	void (*readl_rc)(struct pcie_port *pp,
			void __iomem *dbi_base, u32 *val);
	void (*writel_rc)(struct pcie_port *pp,
			u32 val, void __iomem *dbi_base);
	int (*rd_own_conf)(struct pcie_port *pp, int where, int size, u32 *val);
	int (*wr_own_conf)(struct pcie_port *pp, int where, int size, u32 val);
	int (*rd_other_conf)(struct pcie_port *pp, struct pci_bus *bus,
			unsigned int devfn, int where, int size, u32 *val);
	int (*wr_other_conf)(struct pcie_port *pp, struct pci_bus *bus,
			unsigned int devfn, int where, int size, u32 val);
	int (*link_up)(struct pcie_port *pp);
	void (*host_init)(struct pcie_port *pp);
	void (*msi_set_irq)(struct pcie_port *pp, int irq);
	void (*msi_clear_irq)(struct pcie_port *pp, int irq);
	u32 (*get_msi_addr)(struct pcie_port *pp);
	u32 (*get_msi_data)(struct pcie_port *pp, int pos);
	void (*scan_bus)(struct pcie_port *pp);
	int (*msi_host_init)(struct pcie_port *pp, struct msi_controller *chip);
	#ifdef CONFIG_PCI_S32V234
	int (*send_signal_to_user)(struct pcie_port *pp);
	#endif
};

int dw_pcie_cfg_read(void __iomem *addr, int where, int size, u32 *val);
int dw_pcie_cfg_write(void __iomem *addr, int where, int size, u32 val);
irqreturn_t dw_handle_msi_irq(struct pcie_port *pp);
void dw_pcie_msi_init(struct pcie_port *pp);
int dw_pcie_link_up(struct pcie_port *pp);
void dw_pcie_setup_rc(struct pcie_port *pp);
int dw_pcie_host_init(struct pcie_port *pp);
#ifdef CONFIG_PCI_DW_DMA
int dw_pcie_dma_write_en(struct pcie_port *pp);
int dw_pcie_dma_read_en(struct pcie_port *pp);
int dw_pcie_dma_write_soft_reset(struct pcie_port *pp);
int dw_pcie_dma_read_soft_reset(struct pcie_port *pp);
irqreturn_t dw_handle_dma_irq(struct pcie_port *pp);
void dw_pcie_dma_set_wr_remote_done_int(struct pcie_port *pp, u64 val);
void dw_pcie_dma_set_wr_remote_abort_int(struct pcie_port *pp, u64 val);
void dw_pcie_dma_en_local_int(struct pcie_port *pp/* , u64 val */);
void dw_pcie_dma_set_sar(struct pcie_port *pp, u64 val);
void dw_pcie_dma_set_dar(struct pcie_port *pp, u64 val);
void dw_pcie_dma_set_transfer_size(struct pcie_port *pp, u32 val);
void dw_pcie_dma_set_rd_viewport(struct pcie_port *pp, u8 ch_nr);
void dw_pcie_dma_set_wr_viewport(struct pcie_port *pp, u8 ch_nr);
void dw_pcie_dma_clear_regs(struct pcie_port *pp);
int dw_pcie_dma_single_rw(struct pcie_port *pp,
	struct dma_data_elem *dma_single_rw);
int send_signal_to_user(struct pcie_port *pp);
int dw_pcie_dma_load_linked_list(struct pcie_port *pp,
	struct dma_list (*arr_ll)[], u8 arr_sz, u32 phy_list_addr,
	u32 next_phy_list_addr, u8 direction);
int dw_pcie_dma_start_linked_list(struct pcie_port *pp,
	u32 phy_list_addr,
	u8 direction);
void dw_start_dma_llw(struct pcie_port*, u64);
#endif
#endif /* _PCIE_DESIGNWARE_H */
