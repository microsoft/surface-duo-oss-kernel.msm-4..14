/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Copyright (c) 2020 The Linux Foundation. All rights reserved.
 */
#ifndef _ATH11K_MHI_H
#define _ATH11K_MHI_H

#include "pci.h"
#define ATH11K_PCI_FW_FILE_NAME		"amss.bin"

#define PCIE_TXVECDB (0x360)
#define PCIE_TXVECSTATUS (0x368)
#define PCIE_RXVECDB (0x394)
#define PCIE_RXVECSTATUS (0x39C)

#define MHISTATUS (0x48)
#define MHICTRL (0x38)
#define MHICTRL_RESET_MASK (0x2)

enum ath11k_mhi_state {
	ATH11K_MHI_INIT,
	ATH11K_MHI_DEINIT,
	ATH11K_MHI_POWER_ON,
	ATH11K_MHI_POWER_OFF,
	ATH11K_MHI_FORCE_POWER_OFF,
	ATH11K_MHI_SUSPEND,
	ATH11K_MHI_RESUME,
	ATH11K_MHI_TRIGGER_RDDM,
	ATH11K_MHI_RDDM,
	ATH11K_MHI_RDDM_DONE,
};

int ath11k_pci_start_mhi(struct ath11k_pci *ar_pci);
void ath11k_pci_stop_mhi(struct ath11k_pci *ar_pci);
int ath11k_pci_register_mhi(struct ath11k_pci *ar_pci);
void ath11k_pci_unregister_mhi(struct ath11k_pci *ar_pci);
void ath11k_mhi_set_mhictrl_reset(struct ath11k_base *ab);
void ath11k_mhi_clear_vector(struct ath11k_base *ab);
#endif
