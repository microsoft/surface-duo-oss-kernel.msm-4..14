/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 *
 */
#ifndef _MHI_QCOM_
#define _MHI_QCOM_

#define QCOM_PCI_VENDOR_ID 0x17cb
#define QCOM_PCI_BAR_NUM 0

#define MHI_QCOM_AUTOSUSPEND_MS 1000

struct mhi_qcom_dev {
	struct pci_dev *pci_dev;
	struct mhi_controller *mhi_cntrl;
	int bars;
	bool powered_on;
};

void mhi_deinit_pci_dev(struct mhi_controller *mhi_cntrl);
int mhi_pci_probe(struct pci_dev *pci_dev,
		  const struct pci_device_id *device_id);

#endif /* _MHI_QCOM_ */
