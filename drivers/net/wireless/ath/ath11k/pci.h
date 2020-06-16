/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Copyright (c) 2019-2020 The Linux Foundation. All rights reserved.
 */
#ifndef _ATH11K_PCI_H
#define _ATH11K_PCI_H

#include <linux/mhi.h>

#define PCI_BAR_NUM			0
#define PCI_DMA_MASK_64_BIT		64
#define PCI_DMA_MASK_32_BIT		32

#define MAX_UNWINDOWED_ADDRESS 0x80000
#define WINDOW_ENABLE_BIT 0x40000000
#define WINDOW_REG_ADDRESS 0x310C
#define WINDOW_SHIFT 19
#define WINDOW_VALUE_MASK 0x3F
#define WINDOW_START MAX_UNWINDOWED_ADDRESS
#define WINDOW_RANGE_MASK 0x7FFFF

/* FIXME_KVALO: this should be in hw_params */
/* #define ATH11K_IRQ_CE0_OFFSET 3 */

#define PCIE_SOC_GLOBAL_RESET (0x3008)
#define PCIE_SOC_GLOBAL_RESET_V 1

#define WLAON_WARM_SW_ENTRY (0x1f80504)
#define WLAON_SOC_RESET_CAUSE_REG   (0x01f8060c)

#define PCIE_Q6_COOKIE_ADDR         (0x01F80500)
#define PCIE_Q6_COOKIE_DATA         (0xC0000000)

/* Register to wake the UMAC from power collapse */
#define PCIE_SCRATCH_0_SOC_PCIE_REG 0x4040
/* Register used for handshake mechanism to validate UMAC is awake */
#define PCIE_SOC_WAKE_PCIE_LOCAL_REG 0x3004

#define TCSR_SOC_HW_VERSION                          (0x0224)
#define HW_MAJOR_VERSION_MASK                        (0xFF00)
#define HW_MAJOR_VERSION_SHIFT                       (0x08)
#define HW_MINOR_VERSION_MASK                        (0xFF)
#define HW_MINOR_VERSION_SHIFT                       (0x0)

struct ath11k_msi_user {
	char *name;
	int num_vectors;
	u32 base_vector;
};

struct ath11k_msi_config {
	int total_vectors;
	int total_users;
	struct ath11k_msi_user *users;
};

struct ath11k_pci {
	struct pci_dev *pdev;
	struct device *dev;
	struct ath11k_base *ab;
	void __iomem *mem;
	size_t mem_len;
	u16 dev_id;
	u32 chip_id;
	struct ath11k_msi_config *msi_config;
	u32 msi_ep_base_data;
	struct mhi_controller *mhi_ctrl;
	unsigned long mhi_state;
	u32 register_window;

	/* FIXME_KVALO: add comment */
	spinlock_t window_lock;
};

int ath11k_pci_get_user_msi_assignment(struct ath11k_pci *ar_pci, char *user_name,
				       int *num_vectors, u32 *user_base_data,
				       u32 *base_vector);

int ath11k_pci_get_msi_irq(struct device *dev, unsigned int vector);
void ath11k_pci_write32(struct ath11k_base *ab, u32 offset, u32 value);
u32 ath11k_pci_read32(struct ath11k_base *ab, u32 offset);
#endif
