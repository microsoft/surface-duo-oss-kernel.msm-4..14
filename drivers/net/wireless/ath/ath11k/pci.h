/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Copyright (c) 2019-2020 The Linux Foundation. All rights reserved.
 */

#include "core.h"

#define PCI_BAR_NUM			0
#define PCI_DMA_MASK_64_BIT		64
#define PCI_DMA_MASK_32_BIT		32

struct ath11k_pci {
	struct pci_dev *pdev;
	struct device *dev;
	struct ath11k_base *ab;
	void __iomem *mem;
	size_t mem_len;
	u16 dev_id;
	u32 chip_id;
};

static inline struct ath11k_pci *ath11k_pci_priv(struct ath11k_base *ab)
{
	return (struct ath11k_pci *)ab->drv_priv;
}
