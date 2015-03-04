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
#include <linux/eeprom-consumer.h>
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

struct eeprom_cell {
	struct eeprom_device	*eeprom;
	int			nblocks;
	int			size;
	struct eeprom_block	blocks[0];
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

static int of_eeprom_match(struct device *dev, const void *eeprom_np)
{
	return dev->of_node == eeprom_np;
}

static struct eeprom_device *of_eeprom_find(struct device_node *eeprom_np)
{
	struct device *d;

	if (!eeprom_np)
		return NULL;

	d = class_find_device(&eeprom_class, NULL, eeprom_np, of_eeprom_match);

	return d ? to_eeprom(d) : NULL;
}

static int eeprom_match(struct device *dev, const void *data)
{
	return !strcmp(dev_name(dev), (const char *)data);
}

static struct eeprom_device *eeprom_find(const char *name)
{
	struct device *d;

	d = class_find_device(&eeprom_class, NULL, (void *)name, eeprom_match);

	return d ? to_eeprom(d) : NULL;
}

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

static int eeprom_cell_sanity_check(struct eeprom_cell *cell)
{
	struct eeprom_device *eeprom = cell->eeprom;
	int i;

	/* byte aligned, no need to check for stride sanity */
	if (eeprom->stride == 1)
		return 0;

	for (i = 0; i < cell->nblocks; i++) {
		if (!IS_ALIGNED(cell->blocks[i].offset, eeprom->stride) ||
		    !IS_ALIGNED(cell->blocks[i].count, eeprom->stride)) {
			dev_err(&eeprom->dev,
				"cell unaligned to eeprom stride %d\n",
				eeprom->stride);
			return -EINVAL;
		}
	}

	return 0;
}

static struct eeprom_cell *__eeprom_cell_get(struct device_node *cell_np,
					     const char *ename,
					     struct eeprom_block *blocks,
					     int nblocks)
{
	struct eeprom_cell *cell;
	struct eeprom_device *eeprom = NULL;
	struct property *prop;
	const __be32 *vp;
	u32 pv;
	int i, rval;

	mutex_lock(&eeprom_mutex);

	eeprom = cell_np ? of_eeprom_find(cell_np->parent) : eeprom_find(ename);
	if (!eeprom) {
		mutex_unlock(&eeprom_mutex);
		return ERR_PTR(-EPROBE_DEFER);
	}

	eeprom->users++;
	mutex_unlock(&eeprom_mutex);

	if (!try_module_get(eeprom->owner)) {
		dev_err(&eeprom->dev,
			"could not increase module refcount for cell %s\n",
			ename);
		rval = -EINVAL;
		goto err_mod;
	}

	if (cell_np)
		nblocks = of_property_count_u32_elems(cell_np, "reg") / 2;

	cell = kzalloc(sizeof(*cell) + nblocks * sizeof(*blocks), GFP_KERNEL);
	if (!cell) {
		rval = -ENOMEM;
		goto err_mem;
	}

	cell->nblocks = nblocks;
	cell->eeprom = eeprom;
	cell->size = 0;
	i = 0;

	if (cell_np) {
		of_property_for_each_u32(cell_np, "reg", prop, vp, pv) {
			cell->blocks[i].offset = pv;
			vp = of_prop_next_u32(prop, vp, &pv);
			cell->blocks[i].count = pv;
			cell->size += pv;
			i++;
		}
	} else {
		memcpy(cell->blocks, blocks, nblocks * sizeof(*blocks));
		for (; i < nblocks; i++)
			cell->size += blocks[i].count;
	}

	if (IS_ERR_VALUE(eeprom_cell_sanity_check(cell))) {
		rval  = -EINVAL;
		goto err_sanity;
	}

	return cell;

err_sanity:
	kfree(cell);

err_mem:
	module_put(eeprom->owner);

err_mod:
	mutex_lock(&eeprom_mutex);
	eeprom->users--;
	mutex_unlock(&eeprom_mutex);

	return ERR_PTR(rval);

}

/**
 * eeprom_cell_get(): Get eeprom cell of device form a given eeprom name
 * and blocks.
 *
 * @ename: eeprom device name that needs to be looked-up.
 * @blocks: eeprom blocks containing offset and length information.
 * @nblocks: number of eeprom blocks.
 *
 * The return value will be an ERR_PTR() on error or a valid pointer
 * to a struct eeprom_cell.  The eeprom_cell will be freed by the
 * eeprom_cell_put().
 */
struct eeprom_cell *eeprom_cell_get(const char *ename,
				    struct eeprom_block *blocks, int nblocks)
{
	return __eeprom_cell_get(NULL, ename, blocks, nblocks);
}
EXPORT_SYMBOL_GPL(eeprom_cell_get);

/**
 * of_eeprom_cell_get(): Get eeprom cell of device form a given index
 *
 * @dev: Device that will be interacted with
 * @index: eeprom index in eeproms property.
 *
 * The return value will be an ERR_PTR() on error or a valid pointer
 * to a struct eeprom_cell.  The eeprom_cell will be freed by the
 * eeprom_cell_put().
 */
struct eeprom_cell *of_eeprom_cell_get(struct device *dev, int index)
{
	struct device_node *cell_np;

	if (!dev || !dev->of_node)
		return ERR_PTR(-EINVAL);

	cell_np = of_parse_phandle(dev->of_node, "eeproms", index);
	if (!cell_np)
		return ERR_PTR(-EPROBE_DEFER);

	return __eeprom_cell_get(cell_np, NULL, NULL, 0);
}
EXPORT_SYMBOL_GPL(of_eeprom_cell_get);

/**
 * of_eeprom_cell_get_byname(): Get eeprom cell of device form a given name
 *
 * @dev: Device that will be interacted with
 * @name: eeprom name in eeprom-names property.
 *
 * The return value will be an ERR_PTR() on error or a valid pointer
 * to a struct eeprom_cell.  The eeprom_cell will be freed by the
 * eeprom_cell_put().
 */
struct eeprom_cell *of_eeprom_cell_get_byname(struct device *dev,
					      const char *id)
{
	int index = 0;

	if (!dev || !dev->of_node)
		return ERR_PTR(-EINVAL);

	if (id)
		index = of_property_match_string(dev->of_node,
						 "eeprom-names",
						 id);
	return of_eeprom_cell_get(dev, index);

}
EXPORT_SYMBOL_GPL(of_eeprom_cell_get_byname);

/**
 * eeprom_cell_put(): Release previously allocated eeprom cell.
 *
 * @cell: Previously allocated eeprom cell by eeprom_cell_get()
 * or of_eeprom_cell_get() or of_eeprom_cell_get_byname().
 */
void eeprom_cell_put(struct eeprom_cell *cell)
{
	struct eeprom_device *eeprom = cell->eeprom;

	mutex_lock(&eeprom_mutex);
	eeprom->users--;
	mutex_unlock(&eeprom_mutex);
	module_put(eeprom->owner);
	kfree(cell);
}
EXPORT_SYMBOL_GPL(eeprom_cell_put);

/**
 * eeprom_cell_read(): Read a given eeprom cell
 *
 * @cell: eeprom cell to be read.
 * @len: pointer to length of cell which will be populated on successful read.
 *
 * The return value will be an ERR_PTR() on error or a valid pointer
 * to a char * bufffer.  The buffer should be freed by the consumer with a
 * kfree().
 */
char *eeprom_cell_read(struct eeprom_cell *cell, ssize_t *len)
{
	struct eeprom_device *eeprom = cell->eeprom;
	char *buf;
	int rc, i, offset = 0;

	if (!eeprom || !eeprom->regmap)
		return ERR_PTR(-EINVAL);

	buf = kzalloc(cell->size, GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < cell->nblocks; i++) {
		rc = regmap_bulk_read(eeprom->regmap, cell->blocks[i].offset,
				      buf + offset,
				      cell->blocks[i].count);

		if (IS_ERR_VALUE(rc)) {
			kfree(buf);
			return ERR_PTR(rc);
		}
		offset += cell->blocks[i].count;
	}

	*len = cell->size;

	return buf;
}
EXPORT_SYMBOL_GPL(eeprom_cell_read);

/**
 * eeprom_cell_write(): Write to a given eeprom cell
 *
 * @cell: eeprom cell to be written.
 * @buf: Buffer to be written.
 * @len: length of buffer to be written to eeprom cell.
 *
 * The return value will be an non zero on error or a zero on successful write.
 */
int eeprom_cell_write(struct eeprom_cell *cell, const char *buf, ssize_t len)
{
	struct eeprom_device *eeprom = cell->eeprom;
	int i, rc, offset = 0;

	if (!eeprom || !eeprom->regmap || len != cell->size)
		return -EINVAL;

	for (i = 0; i < cell->nblocks; i++) {
		rc = regmap_bulk_write(eeprom->regmap, cell->blocks[i].offset,
				 buf + offset,
				 cell->blocks[i].count);

		if (IS_ERR_VALUE(rc))
			return rc;

		offset += cell->blocks[i].count;
	}

	return len;
}
EXPORT_SYMBOL_GPL(eeprom_cell_write);

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
