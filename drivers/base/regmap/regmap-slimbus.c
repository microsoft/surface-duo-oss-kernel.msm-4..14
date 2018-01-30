/*
 * Register map access API - slimbus support
 *
 * Copyright 2017 Linaro Inc
 *
 * Author: Srinivas Kandagatla <srinivas.kandagatla@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/regmap.h>
#include <linux/slimbus/slimbus.h>
#include <linux/module.h>

#include "internal.h"
#define WCD9XXX_REGISTER_START_OFFSET 0x800

static int regmap_slimbus_byte_reg_read(void *context, unsigned int reg,
					unsigned int *val)
{
	struct slim_device *slim = context;
	struct slim_ele_access msg = {0,};

//	pr_err("DEBUG:: %s: %x \n", __func__, reg);
	msg.start_offset = WCD9XXX_REGISTER_START_OFFSET + reg;
	msg.num_bytes = 1;
	msg.comp = NULL;
//	msg.rbuf = (void *)val;

	return slim_request_val_element(slim, &msg, (void *)val, 1);
}

static int regmap_slimbus_byte_reg_write(void *context, unsigned int reg,
					 unsigned int val)
{
	struct slim_device *slim = context;
	struct slim_ele_access msg = {0,};

//	pr_err("DEBUG:: %s: %x-> %x \n", __func__, reg, val);

	msg.start_offset = WCD9XXX_REGISTER_START_OFFSET + reg;
	msg.num_bytes = 1;
	msg.comp = NULL;

	return slim_change_val_element(slim, &msg, (void *)&val, 1);
}
#define REG_BYTES 2
#define VAL_BYTES 1

static int regmap_bus_gather_write(void *context,
				const void *reg, size_t reg_size,
				const void *val, size_t val_size)
{
	struct slim_device *slim = context;
	unsigned short c_reg, rreg;
	struct slim_ele_access msg;
	int ret, i;

	//dev_err(&slim->dev, "%s: Codec write  (%d), reg:0x%x, size:%zd\n",
	//		__func__, ret, rreg, val_size);

	msg.start_offset = WCD9XXX_REGISTER_START_OFFSET + *(u16 *)reg;
	msg.num_bytes = val_size;
	msg.comp = NULL;

	ret = slim_change_val_element(slim,  &msg, val, val_size);
	if (ret)
		pr_err("%s: Error, Codec write failed (%d)\n", __func__, ret);

	return ret;
}

static int regmap_bus_write(void *context, const void *data, size_t count)
{
	WARN_ON(count < REG_BYTES);


	return regmap_bus_gather_write(context, data, REG_BYTES,
					data + REG_BYTES,
					count - REG_BYTES);
}


static int regmap_bus_read(void *context, const void *reg, size_t reg_size,
			    void *val, size_t val_size)
{
	struct slim_device *slim = context;
	unsigned short c_reg, rreg;
	struct slim_ele_access msg;
	int ret, i;

	//pr_err("DEBUG:: %s: %x \n", __func__, *(u32 *)reg);
	msg.start_offset = WCD9XXX_REGISTER_START_OFFSET + *(u16 *)reg;
	msg.num_bytes = val_size;
	msg.comp = NULL;

//	pr_err("DEBUG:: %s: %x start offset %x bytes %d \n", __func__, reg, msg.start_offset, val_size);

	ret = slim_request_val_element(slim,  &msg, val, val_size);
	if (ret)
		pr_err("%s: Error, Codec read failed (%d)\n", __func__, ret);

	return ret;
}


static struct regmap_bus regmap_slimbus_bus = {
	.read = regmap_bus_read,
	.write = regmap_bus_write,
//	.reg_write = regmap_slimbus_byte_reg_write,
//	.reg_read = regmap_slimbus_byte_reg_read,
	.reg_format_endian_default = REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default = REGMAP_ENDIAN_NATIVE,

};

static const struct regmap_bus *regmap_get_slimbus(struct slim_device *slim,
					const struct regmap_config *config)
{
	if (config->val_bits == 8)
		return &regmap_slimbus_bus;

	return ERR_PTR(-ENOTSUPP);
}

struct regmap *__regmap_init_slimbus(struct slim_device *slimbus,
				     const struct regmap_config *config,
				     struct lock_class_key *lock_key,
				     const char *lock_name)
{
	const struct regmap_bus *bus = regmap_get_slimbus(slimbus, config);

	if (IS_ERR(bus))
		return ERR_CAST(bus);

	return __regmap_init(&slimbus->dev, bus, &slimbus->dev, config,
			     lock_key, lock_name);
}
EXPORT_SYMBOL_GPL(__regmap_init_slimbus);

struct regmap *__devm_regmap_init_slimbus(struct slim_device *slimbus,
					  const struct regmap_config *config,
					  struct lock_class_key *lock_key,
					  const char *lock_name)
{
	const struct regmap_bus *bus = regmap_get_slimbus(slimbus, config);

	if (IS_ERR(bus))
		return ERR_CAST(bus);

	return __devm_regmap_init(&slimbus->dev, bus, &slimbus, config,
				  lock_key, lock_name);
}
EXPORT_SYMBOL_GPL(__devm_regmap_init_slimbus);

MODULE_LICENSE("GPL");
