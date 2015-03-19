/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __QCOM_GDSC_H__
#define __QCOM_GDSC_H__

#include <linux/pm_domain.h>

struct regmap;

/**
 * struct gdsc - Globally Distributed Switch Controller
 * pd: power domain
 * @regmap: regmap for MMIO accesses
 * @gdscr: gsdc control register
 */
struct gdsc {
	struct generic_pm_domain	pd;
	struct regmap			*regmap;
	unsigned int			gdscr;
};

#define domain_to_gdsc(domain) container_of(domain, struct gdsc, pd)

#ifdef CONFIG_QCOM_GDSC
int gdsc_init(struct generic_pm_domain *domain, struct regmap *regmap);
#else
int gdsc_init(struct generic_pm_domain *domain, struct regmap *regmap)
{
	return 0;
}
#endif /* CONFIG_QCOM_GDSC */
#endif /* __QCOM_GDSC_H__ */
