/* Copyright (c) 2020, The Linux Foundation. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/
#ifndef __FSM_MHI_CNTL_H__
#define __FSM_MHI_CNTL_H__

#include <linux/platform_device.h>

void mhi_pci_cntl_register(void *priv);
int fsm_mhi_cntl_probe(struct platform_device *pdev);
int fsm_mhi_cntl_remove(struct platform_device *pdev);

int mhi_arch_pcie_ops_power_on(void *priv);
void mhi_arch_pcie_ops_power_off(void *priv);

#endif /*__FSM_MHI_CNTL_H__*/