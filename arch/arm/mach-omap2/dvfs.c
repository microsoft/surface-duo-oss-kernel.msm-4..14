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

#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/plist.h>
#include <linux/slab.h>
#include <linux/opp.h>
#include <plat/common.h>
#include <plat/voltage.h>
#include <plat/omap_device.h>

/**
 * struct omap_dev_user_list - Structure maitain userlist per device
 *
 * @dev:       The device requesting for a particular frequency
 * @node:      The list head entry
 * @freq:      frequency being requested
 *
 * Using this structure, user list (requesting dev * and frequency) for
 * each device is maintained. This is how we can have different devices
 * at different frequencies (to support frequency locking and throttling).
 * Even if one of the devices in a given vdd has locked it's frequency,
 * other's can still scale their frequency using this list.
 * If no one has placed a frequency request for a device, then device is
 * set to the frequency from it's opp table.
 */
struct omap_dev_user_list {
		struct device *dev;
		struct plist_node node;
		u32 freq;
};

/**
 * struct omap_vdd_dev_list - Device list per vdd
 *
 * @dev:	The device belonging to a particular vdd
 * @node:	The list head entry
 */
struct omap_vdd_dev_list {
	struct device *dev;
	struct list_head node;
	struct plist_head user_list;
	spinlock_t user_lock; /* spinlock for plist */
};

/**
 * struct omap_vdd_user_list - The per vdd user list
 *
 * @dev:	The device asking for the vdd to be set at a particular
 *			voltage
 * @node:	The list head entry
 * @volt:	The voltage requested by the device <dev>
 */
struct omap_vdd_user_list {
	struct device *dev;
	struct plist_node node;
	u32 volt;
};

/**
 * struct omap_vdd_dvfs_info - The per vdd dvfs info
 *
 * @user_lock:	spinlock for plist operations
 * @user_list:	The vdd user list
 * @scaling_mutex:	Mutex for protecting dvfs data structures for a vdd
 * @voltdm: Voltage domains for which dvfs info stored
 *
 * This is a fundamental structure used to store all the required
 * DVFS related information for a vdd.
 */
struct omap_vdd_dvfs_info {
	spinlock_t user_lock; /* spin lock */
	struct plist_head user_list;
	struct mutex scaling_mutex; /* dvfs mutex */
	struct voltagedomain *voltdm;
	struct list_head dev_list;
	struct device vdd_device;
};

static struct omap_vdd_dvfs_info *omap_dvfs_info_list;
static int omap_nr_vdd;

static struct voltagedomain omap3_vdd[] = {
	{
	.name = "mpu",
	},
	{
	.name = "core",
	},
};
static int omap_dvfs_voltage_scale(struct omap_vdd_dvfs_info *dvfs_info);

static int __init omap_dvfs_init(void);

static struct omap_vdd_dvfs_info *get_dvfs_info(struct voltagedomain *voltdm)
{
	int i;
	if (!voltdm || !omap_dvfs_info_list)
		return NULL;

	for (i = 0; i < omap_nr_vdd; i++)
		if (omap_dvfs_info_list[i].voltdm == voltdm)
			return &omap_dvfs_info_list[i];

	pr_warning("%s: unable find dvfs info for vdd %s\n",
			__func__, voltdm->name);
	return NULL;
}

/**
 * omap_dvfs_find_voltage() - search for given voltage
 * @dev:	device pointer associated with the opp type
 * @volt:	voltage to search for
 *
 * Searches for exact match in the opp list and returns handle to the matching
 * opp if found, else returns ERR_PTR in case of error and should be handled
 * using IS_ERR. If there are multiple opps with same voltage, it will return
 * the first available entry.
 */
static struct opp *omap_dvfs_find_voltage(struct device *dev,
		unsigned long volt)
{
	struct opp *opp = ERR_PTR(-ENODEV);
	unsigned long f = 0;

	do {
		opp = opp_find_freq_ceil(dev, &f);
		if (IS_ERR(opp))
			break;
		if (opp_get_voltage(opp) >= volt)
			break;
		f++;
	} while (1);

	return opp;
}

/**
 * omap_dvfs_add_vdd_user() - Add a voltage request
 * @dvfs_info: omap_vdd_dvfs_info pointer for the required vdd
 * @dev: device making the request
 * @volt: requesting voltage in uV
 *
 * Adds the given devices' voltage request into corresponding
 * vdd's omap_vdd_dvfs_info user list (plist). This list is used
 * to find the maximum voltage request for a given vdd.
 *
 * Returns 0 on success.
 */
static int omap_dvfs_add_vdd_user(struct omap_vdd_dvfs_info *dvfs_info,
		struct device *dev, unsigned long volt)
{
	struct omap_vdd_user_list *user = NULL, *temp_user;
	struct plist_node *node;

	if (!dvfs_info || IS_ERR(dvfs_info)) {
		dev_warn(dev, "%s: VDD specified does not exist!\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&dvfs_info->scaling_mutex);

	plist_for_each_entry(temp_user, &dvfs_info->user_list, node) {
		if (temp_user->dev == dev) {
			user = temp_user;
			break;
		}
	}

	if (!user) {
		user = kzalloc(sizeof(struct omap_vdd_user_list), GFP_KERNEL);
		if (!user) {
			dev_err(dev, "%s: Unable to creat a new user for vdd_%s\n",
				__func__, dvfs_info->voltdm->name);
			mutex_unlock(&dvfs_info->scaling_mutex);
			return -ENOMEM;
		}
		user->dev = dev;
	} else {
		plist_del(&user->node, &dvfs_info->user_list);
	}

	plist_node_init(&user->node, volt);
	plist_add(&user->node, &dvfs_info->user_list);
	node = plist_last(&dvfs_info->user_list);
	user->volt = node->prio;

	mutex_unlock(&dvfs_info->scaling_mutex);

	return 0;
}

/**
 * omap_dvfs_remove_vdd_user() - Remove a voltage request
 * @dvfs_info: omap_vdd_dvfs_info pointer for the required vdd
 * @dev: device making the request
 *
 * Removes the given devices' voltage request from corresponding
 * vdd's omap_vdd_dvfs_info user list (plist).
 *
 * Returns 0 on success.
 */
static int omap_dvfs_remove_vdd_user(struct omap_vdd_dvfs_info *dvfs_info,
		struct device *dev)
{
	struct omap_vdd_user_list *user = NULL, *temp_user;
	int ret = 0;

	if (!dvfs_info || IS_ERR(dvfs_info)) {
		dev_err(dev, "%s: VDD specified does not exist!\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&dvfs_info->scaling_mutex);

	plist_for_each_entry(temp_user, &dvfs_info->user_list, node) {
		if (temp_user->dev == dev) {
			user = temp_user;
			break;
		}
	}

	if (user)
		plist_del(&user->node, &dvfs_info->user_list);
	else {
		dev_err(dev, "%s: Unable to find the user for vdd_%s\n",
					__func__, dvfs_info->voltdm->name);
		ret = -ENOMEM;
	}
	mutex_unlock(&dvfs_info->scaling_mutex);

	return ret;
}

/**
 * omap_dvfs_register_device - Add a device into voltage domain
 * @voltdm:	voltage domain to which the device is to be added
 * @dev:	Device to be added
 *
 * This API will add a given device into user_list of corresponding
 * vdd's omap_vdd_dvfs_info strucure. This list is traversed to scale
 * frequencies of all the devices on a given vdd. This api is called
 * while hwmod db is built for an omap_device.
 *
 * Returns 0 on success.
 */
int omap_dvfs_register_device(struct voltagedomain *voltdm, struct device *dev)
{
	struct omap_vdd_dev_list *temp_dev;
	struct omap_vdd_dvfs_info *dvfs_info = get_dvfs_info(voltdm);

	if (!voltdm || IS_ERR(voltdm) || !dvfs_info) {
		dev_warn(dev, "%s: VDD specified does not exist!\n", __func__);
		return -EINVAL;
	}

	list_for_each_entry(temp_dev, &dvfs_info->dev_list, node) {
		if (temp_dev->dev == dev) {
			dev_warn(dev, "%s: Device already added to vdee_%s\n",
				__func__, dvfs_info->voltdm->name);
			return -EINVAL;
		}
	}

	temp_dev = kzalloc(sizeof(struct omap_vdd_dev_list), GFP_KERNEL);
	if (!temp_dev) {
		dev_err(dev, "%s: Unable to creat a new device for vdd_%s\n",
			__func__, dvfs_info->voltdm->name);
		return -ENOMEM;
	}

	/* Initialize priority ordered list */
	spin_lock_init(&temp_dev->user_lock);
	plist_head_init(&temp_dev->user_list, &temp_dev->user_lock);

	temp_dev->dev = dev;
	list_add(&temp_dev->node, &dvfs_info->dev_list);

	return 0;
}

/**
 * omap_dvfs_add_freq_request() - add a requested device frequency
 *
 *
 * @dvfs_info: omap_vdd_dvfs_info pointer for the required vdd
 * @req_dev: device making the request
 * @target_dev: target device for which frequency request is being made
 * @freq:	target device frequency
 *
 * This API adds a requested frequency into target's device frequency list.
 *
 * Returns 0 on success.
 */
static int omap_dvfs_add_freq_request(struct omap_vdd_dvfs_info *dvfs_info,
	struct device *req_dev, struct device *target_dev, unsigned long freq)
{
	struct omap_dev_user_list *dev_user = NULL, *tmp_user;
	struct omap_vdd_dev_list *temp_dev;

	if (!dvfs_info || IS_ERR(dvfs_info)) {
		dev_warn(target_dev, "%s: VDD specified does not exist!\n",
			__func__);
		return -EINVAL;
	}

	mutex_lock(&dvfs_info->scaling_mutex);

	list_for_each_entry(temp_dev, &dvfs_info->dev_list, node) {
		if (temp_dev->dev == target_dev)
			break;
	}

	if (temp_dev->dev != target_dev) {
		dev_warn(target_dev, "%s: target_dev does not exist!\n",
			__func__);
		mutex_unlock(&dvfs_info->scaling_mutex);
		return -EINVAL;
	}

	plist_for_each_entry(tmp_user, &temp_dev->user_list, node) {
		if (tmp_user->dev == req_dev) {
			dev_user = tmp_user;
			break;
		}
	}

	if (!dev_user) {
		dev_user = kzalloc(sizeof(struct omap_dev_user_list),
					GFP_KERNEL);
		if (!dev_user) {
			dev_err(target_dev, "%s: Unable to creat a new user for vdd_%s\n",
				__func__, dvfs_info->voltdm->name);
			mutex_unlock(&dvfs_info->scaling_mutex);
			return -ENOMEM;
		}
		dev_user->dev = req_dev;
	} else {
		plist_del(&dev_user->node, &temp_dev->user_list);
	}

	plist_node_init(&dev_user->node, freq);
	plist_add(&dev_user->node, &temp_dev->user_list);

	mutex_unlock(&dvfs_info->scaling_mutex);
	return 0;
}

/**
 * omap_dvfs_remove_freq_request() - Remove the requested device frequency
 *
 * @dvfs_info: omap_vdd_dvfs_info pointer for the required vdd
 * @req_dev: device removing the request
 * @target_dev: target device from which frequency request is being removed
 *
 * This API removes a requested frequency from target's device frequency list.
 *
 * Returns 0 on success.
 */

static int omap_dvfs_remove_freq_request(struct omap_vdd_dvfs_info *dvfs_info,
	struct device *req_dev, struct device *target_dev)
{
	struct omap_dev_user_list *dev_user = NULL, *tmp_user;
	int ret = 0;
	struct omap_vdd_dev_list *temp_dev;

	if (!dvfs_info || IS_ERR(dvfs_info)) {
		dev_warn(target_dev, "%s: VDD specified does not exist!\n",
			__func__);
		return -EINVAL;
	}

	mutex_lock(&dvfs_info->scaling_mutex);

	list_for_each_entry(temp_dev, &dvfs_info->dev_list, node) {
		if (temp_dev->dev == target_dev)
			break;
	}

	if (temp_dev->dev != target_dev) {
		dev_warn(target_dev, "%s: target_dev does not exist!\n",
			__func__);
		mutex_unlock(&dvfs_info->scaling_mutex);
		return -EINVAL;
	}

	plist_for_each_entry(tmp_user, &temp_dev->user_list, node) {
		if (tmp_user->dev == req_dev) {
			dev_user = tmp_user;
			break;
		}
	}

	if (dev_user)
		plist_del(&dev_user->node, &temp_dev->user_list);
	else {
		dev_err(target_dev, "%s: Unable to remove the user for vdd_%s\n",
				__func__, dvfs_info->voltdm->name);
			ret = -EINVAL;
		}

	return ret;
}

/* Calculate dependency vdd voltage for given vdd voltage */
static int calc_dep_vdd_volt(struct device *dev,
		struct omap_vdd_info *main_vdd, unsigned long main_volt)
{
	struct omap_vdd_dep_info *dep_vdds;
	int i, ret = 0;

	if (!main_vdd->dep_vdd_info) {
		pr_debug("%s: No dependent VDD's for vdd_%s\n",
			__func__, main_vdd->voltdm.name);
		return 0;
	}

	dep_vdds = main_vdd->dep_vdd_info;

	for (i = 0; i < main_vdd->nr_dep_vdd; i++) {
		struct omap_vdd_dep_volt *volt_table = dep_vdds[i].dep_table;
		int nr_volt = 0;
		unsigned long dep_volt = 0, act_volt = 0;

		while (volt_table[nr_volt].main_vdd_volt != 0) {
			if (volt_table[nr_volt].main_vdd_volt == main_volt) {
				dep_volt = volt_table[nr_volt].dep_vdd_volt;
				break;
			}
			nr_volt++;
		}
		if (!dep_volt) {
			pr_warning("%s: Not able to find a matching volt for"
				"vdd_%s corresponding to vdd_%s %ld volt\n",
				__func__, dep_vdds[i].name,
				main_vdd->voltdm.name, main_volt);
			ret = -EINVAL;
			continue;
		}

		if (!dep_vdds[i].voltdm)
			dep_vdds[i].voltdm =
				omap_voltage_domain_lookup(dep_vdds[i].name);

		act_volt = dep_volt;

		/* See if dep_volt is possible for the vdd*/
		ret = omap_dvfs_add_vdd_user(get_dvfs_info(dep_vdds[i].voltdm),
				dev, act_volt);
	}

	return ret;
}

/* Scale dependent VDD */
static int scale_dep_vdd(struct omap_vdd_dvfs_info *vdd_info)
{
	struct omap_vdd_dep_info *dep_vdds;
	int i;
	struct omap_vdd_info *main_vdd;
	struct voltagedomain *voltdm = vdd_info->voltdm;
	main_vdd = container_of(voltdm, struct omap_vdd_info, voltdm);

	if (!main_vdd->dep_vdd_info) {
		pr_debug("%s: No dependent VDD's for vdd_%s\n",
			__func__, main_vdd->voltdm.name);
		return 0;
	}

	dep_vdds = main_vdd->dep_vdd_info;

	for (i = 0; i < main_vdd->nr_dep_vdd; i++)
		omap_dvfs_voltage_scale(get_dvfs_info(dep_vdds[i].voltdm));

	return 0;
}

/**
 * omap_dvfs_voltage_scale() : API to scale the devices associated with a
 *						voltage domain vdd voltage.
 *
 * @dvfs_info: omap_vdd_dvfs_info pointer for the required vdd
 *
 * This API runs through the list of devices associated with the
 * voltage domain and scales the device rates to the one requested
 * by the user or those corresponding to the new voltage of the
 * voltage domain. Target voltage is the highest voltage in the vdd_user_list.
 *
 * Returns 0 on success
 * else the error value.
 */
static int omap_dvfs_voltage_scale(struct omap_vdd_dvfs_info *dvfs_info)
{
	unsigned long curr_volt;
	int is_volt_scaled = 0;
	struct omap_vdd_dev_list *temp_dev;
	struct plist_node *node;
	int ret = 0;
	struct voltagedomain *voltdm;
	unsigned long volt;
	struct omap_vdd_info *vdd;

	if (!dvfs_info || IS_ERR(dvfs_info)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return -EINVAL;
	}

	voltdm = dvfs_info->voltdm;
	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);

	mutex_lock(&dvfs_info->scaling_mutex);

	/* Find the highest voltage being requested */
	node = plist_last(&dvfs_info->user_list);
	volt = node->prio;

	curr_volt = omap_voltage_get_nom_volt(voltdm);

	if (curr_volt == volt) {
		is_volt_scaled = 1;
	} else if (curr_volt < volt) {
		ret = omap_voltage_scale_vdd(voltdm, volt);
		if (ret) {
			pr_warning("%s: Unable to scale the %s to %ld volt\n",
						__func__, voltdm->name, volt);
			mutex_unlock(&dvfs_info->scaling_mutex);
			return ret;
		}
		is_volt_scaled = 1;
	}

	list_for_each_entry(temp_dev, &dvfs_info->dev_list, node) {
		struct device *dev;
		struct opp *opp;
		unsigned long freq;

		dev = temp_dev->dev;
		if (!plist_head_empty(&temp_dev->user_list)) {
			node = plist_last(&temp_dev->user_list);
			freq = node->prio;
		} else {
			opp = omap_dvfs_find_voltage(dev, volt);
			if (IS_ERR(opp))
				continue;
			freq = opp_get_freq(opp);
		}

		if (freq == omap_device_get_rate(dev)) {
			dev_dbg(dev, "%s: Already at the requested"
				"rate %ld\n", __func__, freq);
			continue;
		}

		ret |= omap_device_set_rate(dev, freq);
	}

	if (!is_volt_scaled && !ret)
		omap_voltage_scale_vdd(voltdm, volt);

	mutex_unlock(&dvfs_info->scaling_mutex);

	/* calculate the voltages for dependent vdd's */
	if (calc_dep_vdd_volt(&dvfs_info->vdd_device, vdd, volt)) {
		pr_warning("%s: Error in calculating dependent vdd voltages"
			"for vdd_%s\n", __func__, voltdm->name);
		return -EINVAL;
	}

	/* Scale dependent vdds */
	scale_dep_vdd(dvfs_info);

	return 0;
}

/**
 * omap_dvfs_init() - Initialize omap dvfs layer
 *
 * Initalizes omap dvfs layer. It basically allocates memory for
 * omap_dvfs_info_list and  populates voltdm pointer inside
 * omap_vdd_dvfs_info structure for all the VDDs.
 *
 * Returns 0 on success.
 */
static int __init omap_dvfs_init()
{
	int i;
	struct voltagedomain *vdd_list;
	if (cpu_is_omap34xx()) {
		omap_nr_vdd = 2;
		vdd_list = omap3_vdd;
	}

	omap_dvfs_info_list = kzalloc(omap_nr_vdd *
			sizeof(struct omap_vdd_dvfs_info), GFP_KERNEL);
	if (!omap_dvfs_info_list) {
		pr_warning("%s: Unable to allocate memory for vdd_list",
			__func__);
		return -ENOMEM;
	}

	for (i = 0; i < omap_nr_vdd; i++) {
		omap_dvfs_info_list[i].voltdm =
			omap_voltage_domain_lookup(vdd_list[i].name);
		/* Init the plist */
		spin_lock_init(&omap_dvfs_info_list[i].user_lock);
		plist_head_init(&omap_dvfs_info_list[i].user_list,
					&omap_dvfs_info_list[i].user_lock);
		/* Init the DVFS mutex */
		mutex_init(&omap_dvfs_info_list[i].scaling_mutex);
		/* Init the device list */
		INIT_LIST_HEAD(&omap_dvfs_info_list[i].dev_list);
	}

	return 0;
}
core_initcall(omap_dvfs_init);
