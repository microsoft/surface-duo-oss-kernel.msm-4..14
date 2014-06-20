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

#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/pm_opp.h>
#include <linux/thermal.h>

unsigned long devfreq_cooling_get_level(struct devfreq *df, unsigned long freq)
{
	unsigned long state = THERMAL_CSTATE_INVALID;
	int i;

	for (i = 0; i < df->profile->max_state; i++) {
		if (df->profile->freq_table[i] == freq)
			state = (df->profile->max_state - 1) - i;
	}

	return state;
}
EXPORT_SYMBOL(devfreq_cooling_get_level);

static int devfreq_cooling_get_max_state(struct thermal_cooling_device *cdev,
		unsigned long *state)
{
	struct devfreq_cooling_device *dfc = cdev->devdata;
	struct devfreq *df = dfc->devfreq;

	*state = df->profile->max_state;

	return 0;
}

static int devfreq_cooling_get_cur_state(struct thermal_cooling_device *cdev,
		unsigned long *state)
{
	struct devfreq_cooling_device *dfc = cdev->devdata;

	*state = dfc->cooling_state;

	return 0;
}

static int devfreq_cooling_set_cur_state(struct thermal_cooling_device *cdev,
		unsigned long state)
{
	struct devfreq_cooling_device *dfc = cdev->devdata;
	struct devfreq *df = dfc->devfreq;
	struct device *dev = df->dev.parent;
	unsigned long freq;

	if (state == dfc->cooling_state)
		return 0;

	dev_dbg(dev, "Setting cooling state %lu\n", state);

	if (state == THERMAL_NO_LIMIT) {
		freq = 0;
	} else {
		unsigned long index;
		unsigned long max_state = df->profile->max_state;

		if (state >= max_state)
			return -EINVAL;

		index = (max_state - 1) - state;
		freq = df->profile->freq_table[index];
	}

	if (df->max_freq != freq)
		devfreq_qos_set_max(df, freq);

	dfc->cooling_state = state;

	return 0;
}

static struct thermal_cooling_device_ops const devfreq_cooling_ops = {
	.get_max_state = devfreq_cooling_get_max_state,
	.get_cur_state = devfreq_cooling_get_cur_state,
	.set_cur_state = devfreq_cooling_set_cur_state,
};

struct devfreq_cooling_device *
of_devfreq_cooling_register(struct device_node *np, struct devfreq *df)
{
	struct thermal_cooling_device *cdev;
	struct devfreq_cooling_device *dfc;

	/* freq_table is required to look map state index to frequency. */
	if (!df->profile->max_state && !df->profile->freq_table)
		return ERR_PTR(-EINVAL);

	dfc = kzalloc(sizeof(struct devfreq_cooling_device), GFP_KERNEL);
	if (!dfc)
		return ERR_PTR(-ENOMEM);

	dfc->devfreq = df;

	cdev = thermal_of_cooling_device_register(np, "devfreq", dfc,
			&devfreq_cooling_ops);
	if (!cdev) {
		dev_err(df->dev.parent,
			"Failed to register devfreq cooling device (%ld)\n",
			PTR_ERR(cdev));
		kfree(dfc);
		return ERR_CAST(cdev);
	}

	dfc->cdev = cdev;

	return dfc;
}
EXPORT_SYMBOL(of_devfreq_cooling_register);

struct devfreq_cooling_device *devfreq_cooling_register(struct devfreq *df)
{
	return of_devfreq_cooling_register(NULL, df);
}
EXPORT_SYMBOL(devfreq_cooling_register);

void devfreq_cooling_unregister(struct devfreq_cooling_device *dfc)
{
	if (!dfc)
		return;

	thermal_cooling_device_unregister(dfc->cdev);

	kfree(dfc);
}
EXPORT_SYMBOL(devfreq_cooling_unregister);
