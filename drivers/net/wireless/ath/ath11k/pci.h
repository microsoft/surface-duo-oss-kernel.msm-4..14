/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Copyright (c) 2019-2020 The Linux Foundation. All rights reserved.
 */

#define PCI_BAR_NUM			0
#define PCI_DMA_MASK_64_BIT		64
#define PCI_DMA_MASK_32_BIT		32

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
};
