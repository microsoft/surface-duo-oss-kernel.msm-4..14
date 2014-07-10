/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (C) 2014 Red Hat
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

/* NOTE: originally based on msm_iommu non-DT driver for same hw
 * but as the structure of the driver changes considerably for DT
 * it seemed easier to not try to support old platforms with the
 * same driver.
 */

#ifndef QCOM_IOMMU_V0_H
#define QCOM_IOMMU_V0_H

#include <linux/interrupt.h>
#include <linux/clk.h>

/* Sharability attributes of QCOM IOMMU mappings */
#define QCOM_IOMMU_ATTR_NON_SH		0x0
#define QCOM_IOMMU_ATTR_SH		0x4

/* Cacheability attributes of QCOM IOMMU mappings */
#define QCOM_IOMMU_ATTR_NONCACHED	0x0
#define QCOM_IOMMU_ATTR_CACHED_WB_WA	0x1
#define QCOM_IOMMU_ATTR_CACHED_WB_NWA	0x2
#define QCOM_IOMMU_ATTR_CACHED_WT	0x3

/* Mask for the cache policy attribute */
#define QCOM_IOMMU_CP_MASK		0x03

/* Maximum number of Machine IDs that we are allowing to be mapped to the same
 * context bank. The number of MIDs mapped to the same CB does not affect
 * performance, but there is a practical limit on how many distinct MIDs may
 * be present. These mappings are typically determined at design time and are
 * not expected to change at run time.
 */
#define MAX_NUM_MIDS	32

/**
 * struct qcom_iommu - a single IOMMU hardware instance
 * @dev: IOMMU device
 * @base: IOMMU config port base address (VA)
 * @irq: Interrupt number
 * @ncb: Number of context banks present on this IOMMU HW instance
 * @ttbr_split: ttbr split
 * @clk: The bus clock for this IOMMU hardware instance
 * @pclk: The clock for the IOMMU bus interconnect
 * @ctx_list: list of 'struct qcom_iommu_ctx'
 * @dev_node: list head in qcom_iommu_devices list
 * @dom_node: list head in domain
 * @domain: attached domain.  Note that the relationship between domain and
 *     and iommu's is N:1, ie. an IOMMU can only be attached to one domain,
 *     but a domain can be attached to many IOMMUs
 */
struct qcom_iommu {
	struct device *dev;
	void __iomem *base;
	int irq;
	int ncb;
	int ttbr_split;
	struct clk *clk;
	struct clk *pclk;
	struct list_head ctx_list;
	struct list_head dev_node;
	struct list_head dom_node;
	struct iommu_domain *domain;
};

/**
 * struct qcom_iommu_ctx - an IOMMU context bank instance
 * @of_node: node ptr of client device
 * @num: Index of this context bank within the hardware
 * @mids: List of Machine IDs that are to be mapped into this context
 *     bank, terminated by -1. The MID is a set of signals on the
 *     AXI bus that identifies the function associated with a specific
 *     memory request. (See ARM spec).
 * @node: list head in ctx_list
 */
struct qcom_iommu_ctx {
	struct device_node *of_node;
	int num;
	int mids[MAX_NUM_MIDS];
	struct list_head node;
};

#endif
