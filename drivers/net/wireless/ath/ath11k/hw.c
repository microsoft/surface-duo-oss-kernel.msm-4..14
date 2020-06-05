// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * Copyright (c) 2018-2020 The Linux Foundation. All rights reserved.
 */

#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/bitfield.h>

#include "hw.h"
#include "core.h"

const char *ath11k_irq_name[ATH11K_IRQ_NUM_MAX] = {
	"misc-pulse1",
	"misc-latch",
	"sw-exception",
	"watchdog",
	"ce0",
	"ce1",
	"ce2",
	"ce3",
	"ce4",
	"ce5",
	"ce6",
	"ce7",
	"ce8",
	"ce9",
	"ce10",
	"ce11",
	"host2wbm-desc-feed",
	"host2reo-re-injection",
	"host2reo-command",
	"host2rxdma-monitor-ring3",
	"host2rxdma-monitor-ring2",
	"host2rxdma-monitor-ring1",
	"reo2ost-exception",
	"wbm2host-rx-release",
	"reo2host-status",
	"reo2host-destination-ring4",
	"reo2host-destination-ring3",
	"reo2host-destination-ring2",
	"reo2host-destination-ring1",
	"rxdma2host-monitor-destination-mac3",
	"rxdma2host-monitor-destination-mac2",
	"rxdma2host-monitor-destination-mac1",
	"ppdu-end-interrupts-mac3",
	"ppdu-end-interrupts-mac2",
	"ppdu-end-interrupts-mac1",
	"rxdma2host-monitor-status-ring-mac3",
	"rxdma2host-monitor-status-ring-mac2",
	"rxdma2host-monitor-status-ring-mac1",
	"host2rxdma-host-buf-ring-mac3",
	"host2rxdma-host-buf-ring-mac2",
	"host2rxdma-host-buf-ring-mac1",
	"rxdma2host-destination-ring-mac3",
	"rxdma2host-destination-ring-mac2",
	"rxdma2host-destination-ring-mac1",
	"host2tcl-input-ring4",
	"host2tcl-input-ring3",
	"host2tcl-input-ring2",
	"host2tcl-input-ring1",
	"wbm2host-tx-completions-ring3",
	"wbm2host-tx-completions-ring2",
	"wbm2host-tx-completions-ring1",
	"tcl2host-status-ring",
};
EXPORT_SYMBOL(ath11k_irq_name);

/* Map from pdev index to hw mac index */
static u8 ath11k_hw_ipq8074_mac_from_pdev_id(int pdev_idx)
{
	switch (pdev_idx) {
	case 0:
		return 0;
	case 1:
		return 2;
	case 2:
		return 1;
	default:
		return ATH11K_INVALID_HW_MAC_ID;
	}
}

static u8 ath11k_hw_ipq6018_mac_from_pdev_id(int pdev_idx)
{
	return pdev_idx;
}

const struct ath11k_hw_ops ipq8074_ops = {
	.get_hw_mac_from_pdev_id = ath11k_hw_ipq8074_mac_from_pdev_id,
};

const struct ath11k_hw_ops ipq6018_ops = {
	.get_hw_mac_from_pdev_id = ath11k_hw_ipq6018_mac_from_pdev_id,
};

const struct ath11k_hw_ops qca6390_ops = {
	.get_hw_mac_from_pdev_id = ath11k_hw_ipq8074_mac_from_pdev_id,
};

#define ATH11K_TX_RING_MASK_0 0x1
#define ATH11K_TX_RING_MASK_1 0x2
#define ATH11K_TX_RING_MASK_2 0x4

#define ATH11K_RX_RING_MASK_0 0x1
#define ATH11K_RX_RING_MASK_1 0x2
#define ATH11K_RX_RING_MASK_2 0x4
#define ATH11K_RX_RING_MASK_3 0x8

#define ATH11K_RX_ERR_RING_MASK_0 0x1

#define ATH11K_RX_WBM_REL_RING_MASK_0 0x1

#define ATH11K_REO_STATUS_RING_MASK_0 0x1

#define ATH11K_RXDMA2HOST_RING_MASK_0 0x1
#define ATH11K_RXDMA2HOST_RING_MASK_1 0x2
#define ATH11K_RXDMA2HOST_RING_MASK_2 0x4

#define ATH11K_HOST2RXDMA_RING_MASK_0 0x1
#define ATH11K_HOST2RXDMA_RING_MASK_1 0x2
#define ATH11K_HOST2RXDMA_RING_MASK_2 0x4

#define ATH11K_RX_MON_STATUS_RING_MASK_0 0x1
#define ATH11K_RX_MON_STATUS_RING_MASK_1 0x2
#define ATH11K_RX_MON_STATUS_RING_MASK_2 0x4

const u8 ath11k_tx_ring_mask[ATH11K_EXT_IRQ_GRP_NUM_MAX] = {
	ATH11K_TX_RING_MASK_0,
	ATH11K_TX_RING_MASK_1,
	ATH11K_TX_RING_MASK_2,
};

const u8 rx_mon_status_ring_mask[ATH11K_EXT_IRQ_GRP_NUM_MAX] = {
	0, 0, 0, 0,
	ATH11K_RX_MON_STATUS_RING_MASK_0,
	ATH11K_RX_MON_STATUS_RING_MASK_1,
	ATH11K_RX_MON_STATUS_RING_MASK_2,
};

const u8 ath11k_rx_ring_mask[ATH11K_EXT_IRQ_GRP_NUM_MAX] = {
	0, 0, 0, 0, 0, 0, 0,
	ATH11K_RX_RING_MASK_0,
	ATH11K_RX_RING_MASK_1,
	ATH11K_RX_RING_MASK_2,
	ATH11K_RX_RING_MASK_3,
};

const u8 ath11k_rx_err_ring_mask[ATH11K_EXT_IRQ_GRP_NUM_MAX] = {
	ATH11K_RX_ERR_RING_MASK_0,
};

const u8 ath11k_rx_wbm_rel_ring_mask[ATH11K_EXT_IRQ_GRP_NUM_MAX] = {
	ATH11K_RX_WBM_REL_RING_MASK_0,
};

const u8 ath11k_reo_status_ring_mask[ATH11K_EXT_IRQ_GRP_NUM_MAX] = {
	ATH11K_REO_STATUS_RING_MASK_0,
};

const u8 ath11k_rxdma2host_ring_mask[ATH11K_EXT_IRQ_GRP_NUM_MAX] = {
	ATH11K_RXDMA2HOST_RING_MASK_0,
	ATH11K_RXDMA2HOST_RING_MASK_1,
	ATH11K_RXDMA2HOST_RING_MASK_2,
};

const u8 ath11k_host2rxdma_ring_mask[ATH11K_EXT_IRQ_GRP_NUM_MAX] = {
	ATH11K_HOST2RXDMA_RING_MASK_0,
	ATH11K_HOST2RXDMA_RING_MASK_1,
	ATH11K_HOST2RXDMA_RING_MASK_2,
};

const struct ath11k_hw_regs ipq8074_regs = {
	/* SW2TCL(x) R0 ring configuration address */
	.hal_tcl1_ring_base_lsb = 0x00000510,
	.hal_tcl1_ring_base_msb = 0x00000514,
	.hal_tcl1_ring_id = 0x00000518,
	.hal_tcl1_ring_misc = 0x00000520,
	.hal_tcl1_ring_tp_addr_lsb = 0x0000052c,
	.hal_tcl1_ring_tp_addr_msb = 0x00000530,
	.hal_tcl1_ring_consumer_int_setup_ix0 = 0x00000540,
	.hal_tcl1_ring_consumer_int_setup_ix1 = 0x00000544,
	.hal_tcl1_ring_msi1_base_lsb = 0x00000558,
	.hal_tcl1_ring_msi1_base_msb = 0x0000055c,
	.hal_tcl1_ring_msi1_data = 0x00000560,
	.hal_tcl2_ring_base_lsb = 0x00000568,
	.hal_tcl_ring_base_lsb = 0x00000618,

	/* TCL STATUS ring address */
	.hal_tcl_status_ring_base_lsb = 0x00000720,

	/* REO2SW(x) R0 ring configuration address */
	.hal_reo1_ring_base_lsb = 0x0000029c,
	.hal_reo1_ring_base_msb = 0x000002a0,
	.hal_reo1_ring_id = 0x000002a4,
	.hal_reo1_ring_misc = 0x000002ac,
	.hal_reo1_ring_hp_addr_lsb = 0x000002b0,
	.hal_reo1_ring_hp_addr_msb = 0x000002b4,
	.hal_reo1_ring_producer_int_setup = 0x000002c0,
	.hal_reo1_ring_msi1_base_lsb = 0x000002e4,
	.hal_reo1_ring_msi1_base_msb = 0x000002e8,
	.hal_reo1_ring_msi1_data = 0x000002ec,
	.hal_reo2_ring_base_lsb = 0x000002f4,
	.hal_reo1_aging_thresh_ix_0 = 0x00000564,
	.hal_reo1_aging_thresh_ix_1 = 0x00000568,
	.hal_reo1_aging_thresh_ix_2 = 0x0000056c,
	.hal_reo1_aging_thresh_ix_3 = 0x00000570,

	/* REO2SW(x) R2 ring pointers (head/tail) address */
	.hal_reo1_ring_hp = 0x00003038,
	.hal_reo1_ring_tp = 0x0000303c,
	.hal_reo2_ring_hp = 0x00003040,

	/* REO2TCL R0 ring configuration address */
	.hal_reo_tcl_ring_base_lsb = 0x000003fc,
	.hal_reo_tcl_ring_hp = 0x00003058,

	/* REO status address */
	.hal_reo_status_ring_base_lsb = 0x00000504,
	.hal_reo_status_hp = 0x00003070,

};

const struct ath11k_hw_regs qca6x90_regs = {
	/* SW2TCL(x) R0 ring configuration address */
	.hal_tcl1_ring_base_lsb = 0x00000684,
	.hal_tcl1_ring_base_msb = 0x00000688,
	.hal_tcl1_ring_id = 0x0000068c,
	.hal_tcl1_ring_misc = 0x00000694,
	.hal_tcl1_ring_tp_addr_lsb = 0x000006a0,
	.hal_tcl1_ring_tp_addr_msb = 0x000006a4,
	.hal_tcl1_ring_consumer_int_setup_ix0 = 0x000006b4,
	.hal_tcl1_ring_consumer_int_setup_ix1 = 0x000006b8,
	.hal_tcl1_ring_msi1_base_lsb = 0x000006cc,
	.hal_tcl1_ring_msi1_base_msb = 0x000006d0,
	.hal_tcl1_ring_msi1_data = 0x000006d4,
	.hal_tcl2_ring_base_lsb = 0x000006dc,
	.hal_tcl_ring_base_lsb = 0x0000078c,

	/* TCL STATUS ring address */
	.hal_tcl_status_ring_base_lsb = 0x00000894,

	/* REO2SW(x) R0 ring configuration address */
	.hal_reo1_ring_base_lsb = 0x00000244,
	.hal_reo1_ring_base_msb = 0x00000248,
	.hal_reo1_ring_id = 0x0000024c,
	.hal_reo1_ring_misc = 0x00000254,
	.hal_reo1_ring_hp_addr_lsb = 0x00000258,
	.hal_reo1_ring_hp_addr_msb = 0x0000025c,
	.hal_reo1_ring_producer_int_setup = 0x00000268,
	.hal_reo1_ring_msi1_base_lsb = 0x0000028c,
	.hal_reo1_ring_msi1_base_msb = 0x00000290,
	.hal_reo1_ring_msi1_data = 0x00000294,
	.hal_reo2_ring_base_lsb = 0x0000029c,
	.hal_reo1_aging_thresh_ix_0 = 0x0000050c,
	.hal_reo1_aging_thresh_ix_1 = 0x00000510,
	.hal_reo1_aging_thresh_ix_2 = 0x00000514,
	.hal_reo1_aging_thresh_ix_3 = 0x00000518,

	/* REO2SW(x) R2 ring pointers (head/tail) address */
	.hal_reo1_ring_hp = 0x00003030,
	.hal_reo1_ring_tp = 0x00003034,
	.hal_reo2_ring_hp = 0x00003038,

	/* REO2TCL R0 ring configuration address */
	.hal_reo_tcl_ring_base_lsb = 0x000003a4,
	.hal_reo_tcl_ring_hp = 0x00003050,

	/* REO status address */
	.hal_reo_status_ring_base_lsb = 0x000004ac,
	.hal_reo_status_hp = 0x00003068,
};
