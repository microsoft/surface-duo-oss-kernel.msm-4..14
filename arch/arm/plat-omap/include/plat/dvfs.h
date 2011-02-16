/*
 * OMAP3/OMAP4 DVFS Management Routines
 *
 * Author: Vishwanath BS	<vishwanath.bs@ti.com>
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Vishwanath BS <vishwanath.bs@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_DVFS_H
#define __ARCH_ARM_MACH_OMAP2_DVFS_H
#include <plat/voltage.h>

#ifdef CONFIG_PM
int omap_dvfs_register_device(struct voltagedomain *voltdm, struct device *dev);
int omap_device_scale(struct device *req_dev, struct device *dev,
			unsigned long rate);
#else
static inline int omap_dvfs_register_device(struct voltagedomain *voltdm,
		struct device *dev)
{
	return -EINVAL;
}
static inline int omap_device_scale(struct device *req_dev, struct devices
			*target_dev, unsigned long rate);
{
	return -EINVAL;
}
#endif
#endif
