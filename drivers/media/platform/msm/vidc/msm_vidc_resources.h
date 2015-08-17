/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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
 */

#ifndef __MSM_VIDC_RESOURCES_H__
#define __MSM_VIDC_RESOURCES_H__

#include <linux/platform_device.h>

struct load_freq_table {
	u32 load;
	unsigned long freq;
};

struct reg_value_pair {
	u32 reg;
	u32 value;
};

struct reg_set {
	const struct reg_value_pair *reg_tbl;
	int count;
};

struct addr_range {
	u32 start;
	u32 size;
};

struct context_bank_info {
	struct list_head list;
	const char *name;
	u32 buffer_type;
	bool is_secure;
	struct addr_range addr_range;
	struct device *dev;
	struct dma_iommu_mapping *mapping;
};

struct clock_info {
	const char *name;
	struct clk *clk;
	const struct load_freq_table *load_freq_tbl;
	u32 count; /* == has_scaling iff count != 0 */
};

struct clock_set {
	struct clock_info *clock_tbl;
	u32 count;
};

struct vidc_resources {
	void __iomem *base;
	unsigned int irq;
	const struct load_freq_table *load_freq_tbl;
	u32 load_freq_tbl_size;
	struct reg_set reg_set;
	u32 max_load;
	struct platform_device *pdev;
	struct clock_set clock_set;
	bool sw_power_collapsible;
	bool sys_idle_indicator;
	struct list_head context_banks;
	const char *hfi_version;
};

extern unsigned int vidc_pwr_collapse_delay;
struct vidc_core;

int get_platform_resources(struct vidc_core *);
void put_platform_resources(struct vidc_core *);

int enable_clocks(struct vidc_resources *);
void disable_clocks(struct vidc_resources *);

#endif
