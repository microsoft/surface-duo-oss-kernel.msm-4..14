/*
 * devfreq_cooling: Thermal cooling device implementation for devices using
 *                  devfreq
 *
 * Copyright (C) 2014 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __DEVFREQ_COOLING_H__
#define __DEVFREQ_COOLING_H__

#include <linux/devfreq.h>
#include <linux/thermal.h>

#ifdef CONFIG_DEVFREQ_THERMAL

struct devfreq_cooling_device {
	struct thermal_cooling_device *cdev;
	struct devfreq *devfreq;
	unsigned long cooling_state;
};


unsigned long devfreq_cooling_get_level(struct devfreq *df, unsigned long freq);

struct devfreq_cooling_device *
of_devfreq_cooling_register(struct device_node *np, struct devfreq *df);
struct devfreq_cooling_device *devfreq_cooling_register(struct devfreq *df);
void devfreq_cooling_unregister(struct devfreq_cooling_device *dfc);

#else /* !CONFIG_DEVFREQ_THERMAL */

static inline struct devfreq_cooling_device *
of_devfreq_cooling_register(struct device_node *np, struct devfreq *df)
{
	return NULL;
}

static inline unsigned long
devfreq_cooling_get_level(struct devfreq *df, unsigned long freq)
{
	return THERMAL_CSTATE_INVALID;
}

static inline struct devfreq_cooling_device *
devfreq_cooling_register(struct devfreq *df)
{
	return NULL;
}

static inline void
devfreq_cooling_unregister(struct devfreq_cooling_device *dfc)
{
	return;
}

#endif /* CONFIG_DEVFREQ_THERMAL */
#endif /* __DEVFREQ_COOLING_H__ */
