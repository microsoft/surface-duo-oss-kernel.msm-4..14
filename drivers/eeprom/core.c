/*
 * EEPROM framework core.
 *
 * Copyright (C) 2015 Srinivas Kandagatla <srinivas.kandagatla@linaro.org>
 * Copyright (C) 2013 Maxime Ripard <maxime.ripard@free-electrons.com>
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

#include <linux/device.h>
#include <linux/eeprom-provider.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/init.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

struct eeprom_device {
	struct regmap		*regmap;
	int			stride;
	size_t			size;

	struct module		*owner;
	struct device		dev;
	int			id;
	int			users;
};

static DEFINE_MUTEX(eeprom_mutex);
static DEFINE_IDA(eeprom_ida);

#define to_eeprom(d) container_of(d, struct eeprom_device, dev)

static ssize_t bin_attr_eeprom_read(struct file *filp, struct kobject *kobj,
				    struct bin_attribute *attr,
				    char *buf, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct eeprom_device *eeprom = to_eeprom(dev);
	int rc;

	if (offset > eeprom->size)
		return -EINVAL;

	if (offset + count > eeprom->size)
		count = eeprom->size - offset;

	rc = regmap_bulk_read(eeprom->regmap, offset,
			      buf, count/eeprom->stride);

	if (IS_ERR_VALUE(rc))
		return rc;

	return count - count % eeprom->stride;
}

static ssize_t bin_attr_eeprom_write(struct file *filp, struct kobject *kobj,
				     struct bin_attribute *attr,
				     char *buf, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct eeprom_device *eeprom = to_eeprom(dev);
	int rc;

	if (offset > eeprom->size)
		return -EINVAL;

	if (offset + count > eeprom->size)
		count = eeprom->size - offset;

	rc = regmap_bulk_write(eeprom->regmap, offset,
			       buf, count/eeprom->stride);

	if (IS_ERR_VALUE(rc))
		return rc;

	return count - count % eeprom->stride;
}

static struct bin_attribute bin_attr_eeprom = {
	.attr	= {
		.name	= "eeprom",
		.mode	= S_IWUSR | S_IRUGO,
	},
	.read	= bin_attr_eeprom_read,
	.write	= bin_attr_eeprom_write,
};

static struct bin_attribute *eeprom_bin_attributes[] = {
	&bin_attr_eeprom,
	NULL,
};

static const struct attribute_group eeprom_bin_group = {
	.bin_attrs	= eeprom_bin_attributes,
};

static const struct attribute_group *eeprom_dev_groups[] = {
	&eeprom_bin_group,
	NULL,
};

static void eeprom_release(struct device *dev)
{
	struct eeprom_device *eeprom = to_eeprom(dev);

	ida_simple_remove(&eeprom_ida, eeprom->id);
	kfree(eeprom);
}

static struct class eeprom_class = {
	.name		= "eeprom",
	.dev_groups	= eeprom_dev_groups,
	.dev_release	= eeprom_release,
};

/**
 * eeprom_register(): Register a eeprom device for given eeprom.
 * Also creates an binary entry in /sys/class/eeprom/name-id/eeprom
 *
 * @eeprom: eeprom device that needs to be created
 *
 * The return value will be an error code on error or a zero on success.
 * The eeprom_device and sysfs entery will be freed by the eeprom_unregister().
 */

struct eeprom_device *eeprom_register(struct eeprom_config *config)
{
	struct eeprom_device *eeprom;
	struct regmap *rm;
	int rval;

	if (!config->dev)
		return ERR_PTR(-EINVAL);

	rm = dev_get_regmap(config->dev, NULL);
	if (!rm) {
		dev_err(config->dev, "Regmap not found\n");
		return ERR_PTR(-EINVAL);
	}

	eeprom = kzalloc(sizeof(*eeprom), GFP_KERNEL);
	if (!eeprom)
		return ERR_PTR(-ENOMEM);

	eeprom->id = ida_simple_get(&eeprom_ida, 0, 0, GFP_KERNEL);
	if (eeprom->id < 0) {
		kfree(eeprom);
		return ERR_PTR(eeprom->id);
	}

	eeprom->regmap = rm;
	eeprom->owner = config->owner;
	eeprom->stride = regmap_get_reg_stride(rm);
	eeprom->size = regmap_get_max_register(rm);
	eeprom->dev.class = &eeprom_class;
	eeprom->dev.parent = config->dev;
	eeprom->dev.of_node = config->dev ? config->dev->of_node : NULL;
	dev_set_name(&eeprom->dev, "%s%d",
		     config->name ? : "eeprom", config->id);

	device_initialize(&eeprom->dev);

	dev_dbg(&eeprom->dev, "Registering eeprom device %s\n",
		dev_name(&eeprom->dev));

	rval = device_add(&eeprom->dev);
	if (rval) {
		ida_simple_remove(&eeprom_ida, eeprom->id);
		kfree(eeprom);
		return ERR_PTR(rval);
	}

	return eeprom;
}
EXPORT_SYMBOL_GPL(eeprom_register);

/**
 * eeprom_unregister(): Unregister previously registered eeprom device
 *
 * @eeprom: Pointer to previously registered eeprom device.
 *
 * The return value will be an non zero on error or a zero on success.
 */
int eeprom_unregister(struct eeprom_device *eeprom)
{
	mutex_lock(&eeprom_mutex);
	if (eeprom->users) {
		mutex_unlock(&eeprom_mutex);
		return -EBUSY;
	}
	mutex_unlock(&eeprom_mutex);

	device_del(&eeprom->dev);

	return 0;
}
EXPORT_SYMBOL_GPL(eeprom_unregister);

static int eeprom_init(void)
{
	return class_register(&eeprom_class);
}

static void eeprom_exit(void)
{
	class_unregister(&eeprom_class);
}

subsys_initcall(eeprom_init);
module_exit(eeprom_exit);

MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com");
MODULE_DESCRIPTION("EEPROM Driver Core");
MODULE_LICENSE("GPL v2");
