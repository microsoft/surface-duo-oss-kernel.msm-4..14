/*
 * max344417.c
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#define MAX34417_DEVICE_ID 0x07 // Expected device id returned from did command

/* MAX34417 command codes */
#define MAX34417_CMD_UPDATE	0x00
#define MAX34417_CMD_CONTROL 0x01
#define MAX34417_CMD_ACC_COUNT	0x02
#define MAX34417_CMD_PWR_ACC(ch)	(0x02 + (ch))
#define MAX34417_CMD_VOLTAGE(ch) (0x06 + (ch))
#define MAX34417_CMD_DID	0x0F
#define MAX34417_CMD_BULK_UPDATE	0x00
#define MAX34417_CMD_BULK_PWR 0x10
#define MAX34417_CMD_BULK_VOLTAGE 0x11

/* MAX34417 control bits */
#define MAX34417_CTRL_MODE (0x1 << 7) // 0 == MAX34407, 1 == MAX34417
#define MAX34417_CTRL_CAM	(0x1 << 6) // Continuous Accumulate Mode enabled
#define MAX34417_CTRL_SMM	(0x1 << 5) // Single Measure Mode enabled
#define MAX34417_CTRL_PARK_EN	(0x1 << 4) // Channel park feature enabled
#define MAX34417_CTRL_PARK_CH(ch)	((ch) - 0x1) << 2) // Parked channel select
#define MAX34417_CTRL_SLOW	(0x1 << 1) // Slow low power accumulation mode
#define MAX34417_CTRL_OVF	(0x1 << 0) // Overflow condition flag

#define MAX34417_BROADCAST_ADDR 0x2C

#define MAX34417_VOLTAGE_BITS 14
#define MAX34417_VOLTAGE_FULL_SCALE_MV 24000
#define MAX34417_AVG_POWER_FULL_SCALE_UW 2400000000

#define MAX34417_DEFAULT_UPDATE_INTERVAL_MS 1000
#define MAX34417_UPDATE_DELAY_MS 3

#define MAX34417_POWER_UP_TIME_MS 4
#define MAX34417_MAX_LABEL_SIZE 20

#define CREATE_TRACE_POINTS
#include "max34417.h"

struct max34417_did {
	s8 revision : 3;
	s8 device_id : 5;
};

/*
 * Per device instance client data
 */
struct max34417_data {
	struct i2c_client *client;
	struct delayed_work update_work;
	struct delayed_work read_work;
	struct mutex telemetry_lock;
	u32 update_interval; // protected by telemetry_lock
	u64 power_uw[4]; // protected by telemetry_lock
	u16 voltage_mv[4]; // protected by telemetry_lock
	u8 ch_enabled[4]; // immutable
	u16 ch_sense_resistance_mo[4]; // immutable
	u32 ch_reg[4];
	char ch_label[4][MAX34417_MAX_LABEL_SIZE];
};

static s32 max34417_update(struct max34417_data *max)
{
	return i2c_smbus_write_byte(max->client, MAX34417_CMD_UPDATE);
}

static s32 max34417_control_read(struct max34417_data *max)
{
	return i2c_smbus_read_byte_data(max->client, MAX34417_CMD_CONTROL);
}

static s32 max34417_control_write(struct max34417_data *max, u8 flags)
{
	return i2c_smbus_write_byte_data(max->client, MAX34417_CMD_CONTROL, flags);
}

static s32 max34417_acc_count(struct max34417_data *max)
{
	s32 rv;
	u8 buf[32];

	rv = i2c_smbus_read_i2c_block_data(max->client, MAX34417_CMD_ACC_COUNT, 4, buf);

	if (rv < 0) return rv;

	return __be32_to_cpup((__be32 *)(buf + 1)) >> 8;
}

static s64 __attribute__((unused)) max34417_power_acc(struct max34417_data *max, int channel)
{
	s32 rv;
	u8 buf[32];

	if (channel < 1 || channel > 4) return -EINVAL;

	rv = i2c_smbus_read_i2c_block_data(max->client, MAX34417_CMD_PWR_ACC(channel), 8, buf);

	if (rv < 0) return rv;

	return __be64_to_cpup((__be64 *)(buf + 1)) >> 8;
}

static s16 __attribute__((unused)) max34417_voltage_read(struct max34417_data *max, int channel)
{
	s32 rv;
	u8 buf[32];

	if (channel < 1 || channel > 4) return -EINVAL;

	rv = i2c_smbus_read_i2c_block_data(max->client, MAX34417_CMD_VOLTAGE(channel), 3, buf);

	if (rv < 0) return rv;

	return __be16_to_cpup((__be16 *)buf + 1) >> 2;
}

static s32 max34417_device_id(struct max34417_data *max)
{
	s32 rv;

	rv = i2c_smbus_read_byte_data(max->client, MAX34417_CMD_DID);

	return rv;
}

static __attribute__((unused)) s32 max34417_bulk_update(struct i2c_adapter *adapter)
{
	return i2c_smbus_xfer(adapter, MAX34417_BROADCAST_ADDR, 0,
		I2C_SMBUS_WRITE, MAX34417_CMD_BULK_UPDATE, I2C_SMBUS_BYTE, NULL);
}

static s32 max34417_bulk_power_read(struct max34417_data *max, u64 *out)
{
	s32 rv;
	int i;
	u8 buf[32];

	rv = i2c_smbus_read_i2c_block_data(max->client, MAX34417_CMD_BULK_PWR, 29, buf);

	if (rv < 0) return rv;

	for (i = 0; i < 4; i++) {
		out[i] = __be64_to_cpup((__be64 *)&buf[(i * 7) + 1]) >> 8;
	}

	return rv;
}

static s32 max34417_bulk_voltage_read(struct max34417_data *max, u16 *out)
{
	s32 rv;
	u8 buf[32];
	int i;

	rv = i2c_smbus_read_i2c_block_data(max->client, MAX34417_CMD_BULK_VOLTAGE, 9, buf);

	if (rv < 0) return rv;

	for (i = 0; i < 4; i++) {
		out[i] = __be16_to_cpup((__be16 *)&buf[(i*2)+1]) >> 2;
	}

	return rv;
}

void max34417_update_handler(struct work_struct *work) {
	struct max34417_data *data = container_of(
		to_delayed_work(work), struct max34417_data, update_work);

	max34417_update(data);

	schedule_delayed_work(&data->read_work,
		msecs_to_jiffies(MAX34417_UPDATE_DELAY_MS));
}

void max34417_read_handler(struct work_struct *work) {
	u32 update_interval, acc_count;
	u64 power_acc[4];
	u16 voltage_acc[4];
	int i;
	struct max34417_data *data = container_of(
		to_delayed_work(work), struct max34417_data, read_work);

	acc_count = max34417_acc_count(data);
	max34417_bulk_power_read(data, power_acc);
	max34417_bulk_voltage_read(data, voltage_acc);

	mutex_lock(&data->telemetry_lock);
	for (i = 0; i < 4; i++) {
		u32 scaled_mv;
		u64 unscaled_avg_pwr = power_acc[i] / acc_count; // 56b - 24b = 32b
		u64 power_scale_correction_factor_uw = // 32b
				MAX34417_AVG_POWER_FULL_SCALE_UW / data->ch_sense_resistance_mo[i];
		data->power_uw[i] = // 32 bits + 32 bits - 30 bits = 34 bits
				(unscaled_avg_pwr * power_scale_correction_factor_uw) >> 30;

		// 16b + 14b = 30b
		scaled_mv = voltage_acc[i] * MAX34417_VOLTAGE_FULL_SCALE_MV;
		data->voltage_mv[i] = // 30b - 14b = 16b
				(long) (scaled_mv >> MAX34417_VOLTAGE_BITS);

		trace_max34417_read(data->ch_reg[i], data->ch_label[i],
			data->power_uw[i], data->voltage_mv[i]);
	}
	update_interval = data->update_interval;
	mutex_unlock(&data->telemetry_lock);

	schedule_delayed_work(&data->update_work, msecs_to_jiffies(
		update_interval - MAX34417_UPDATE_DELAY_MS));
}

static int max34417_read_power(struct device *dev, u32 attr, int channel,
			     long *val)
{
	struct max34417_data *max = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_power_average:
		mutex_lock(&max->telemetry_lock);
		*val = max->power_uw[channel];
		mutex_unlock(&max->telemetry_lock);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t max34417_power_is_visible(const struct max34417_data *max, u32 attr,
								int channel)
{
	switch (attr) {
	case hwmon_power_average:
		if (max->ch_enabled[channel])	return S_IRUGO;
	default:
		return 0;
	}
}

static int max34417_read_in(struct device *dev, u32 attr, int channel,
			     long *val)
{
	struct max34417_data *max = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_in_input:
		mutex_lock(&max->telemetry_lock);
		*val = max->voltage_mv[channel];
		mutex_unlock(&max->telemetry_lock);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t max34417_in_is_visible(const struct max34417_data *max, u32 attr, int channel)
{
	switch (attr) {
	case hwmon_in_input:
		if (max->ch_enabled[channel])	return S_IRUGO;
	default:
		return 0;
	}
}

static int max34417_read(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long *val)
{
	struct max34417_data *max = dev_get_drvdata(dev);
	switch (type) {
	case hwmon_chip:
		mutex_lock(&max->telemetry_lock);
		*val = max->update_interval;
		mutex_unlock(&max->telemetry_lock);
		return 0;
	case hwmon_power:
		return max34417_read_power(dev, attr, channel, val);
	case hwmon_in:
		return max34417_read_in(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int max34417_write(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long val)
{
	struct max34417_data *max = dev_get_drvdata(dev);
	switch (type) {
	case hwmon_chip:
		mutex_lock(&max->telemetry_lock);
		max->update_interval = val;
		mutex_unlock(&max->telemetry_lock);
		return 0;
	case hwmon_power:
	case hwmon_in:
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t max34417_is_visible(const void *data,
				   enum hwmon_sensor_types type,
				   u32 attr, int channel)
{
	switch (type) {
	case hwmon_chip:
		return S_IRUGO | S_IWUSR | S_IWGRP;
	case hwmon_power:
		return max34417_power_is_visible(data, attr, channel);
	case hwmon_in:
		return max34417_in_is_visible(data, attr, channel);
	default:
		return 0;
	}
}

static const u32 max34417_power_config[] = {
	HWMON_P_AVERAGE,
	HWMON_P_AVERAGE,
	HWMON_P_AVERAGE,
	HWMON_P_AVERAGE,
	0
};

static const struct hwmon_channel_info max34417_power = {
	.type = hwmon_power,
	.config = max34417_power_config,
};

static const u32 max34417_in_config[] = {
	HWMON_I_INPUT,
	HWMON_I_INPUT,
	HWMON_I_INPUT,
	HWMON_I_INPUT,
	0
};

static const struct hwmon_channel_info max34417_in = {
	.type = hwmon_in,
	.config = max34417_in_config,
};

static const u32 max34417_chip_config[] = {
	HWMON_C_UPDATE_INTERVAL,
	0
};

static const struct hwmon_channel_info max34417_chip = {
	.type = hwmon_chip,
	.config = max34417_chip_config,
};

static const struct hwmon_channel_info *max34417_info[] = {
	&max34417_power,
	&max34417_in,
	&max34417_chip,
	NULL
};

static const struct hwmon_ops max34417_hwmon_ops = {
	.is_visible = max34417_is_visible,
	.read = max34417_read,
	.write = max34417_write,
};

static const struct hwmon_chip_info max34417_chip_info = {
	.ops = &max34417_hwmon_ops,
	.info = max34417_info,
};

static int max34417_init_client(struct i2c_client *client,
				struct max34417_data *data)
{
	s32 ctrl, err;
	int i;
	struct regulator *regulator_vdd, *regulator_vio;
	struct device_node *channels_node, *channel_node = NULL;
	union {
		s32 raw;
		struct max34417_did did;
	} did;

	// Enable regulators to power the device and io bus
	regulator_vdd = regulator_get(&client->dev, "vdd");
	if (!IS_ERR(regulator_vdd)) {
		err = regulator_enable(regulator_vdd);
	}

	regulator_vio = regulator_get(&client->dev, "vio");
	if (!IS_ERR(regulator_vio)) {
		err = regulator_enable(regulator_vio);
	}

	usleep_range(MAX34417_POWER_UP_TIME_MS * 1000,
				(MAX34417_POWER_UP_TIME_MS + 2) * 1000);

	// Initialize all channels as disabled
	for (i = 0; i < 4; i++) data->ch_enabled[i] = 0;

	// For all channels defined in device tree if any, parse parameters and enable
	channels_node = of_get_child_by_name(client->dev.of_node, "channels");
	if (channels_node) {
		while ((channel_node = of_get_next_available_child(
			channels_node, channel_node))
		) {
			u32 reg, res;
			if (!of_property_read_u32(channel_node, "reg", &reg)) {
				if (reg >= 0 && reg < 4) {
					if (!of_property_read_u32(channel_node, "sense-resistor-milliohm",
						&res)) {
						const char *label = NULL;
						const char unknown_label[] = "unknown";

						data->ch_sense_resistance_mo[reg] = res;
						data->ch_enabled[reg] = 1;
						data->ch_reg[reg] = reg;

						if (!of_property_read_string(channel_node, "label", &label))
							strlcpy(data->ch_label[reg], label, sizeof(data->ch_label[reg]));
						else
							strlcpy(data->ch_label[reg], unknown_label, sizeof(unknown_label));
					} else {
						dev_err(&client->dev,
							"Missing sense-resistor-milliohm property in device tree node\n");
						return -ENODEV;
					}
				}
			}
		}
	}

	(void) err;
	did.raw = max34417_device_id(data);

	if (did.raw < 0) return did.raw;

	if (did.did.device_id != MAX34417_DEVICE_ID) {
		dev_err(&client->dev, "Probed device is not a max34417\n");
		return -ENODEV;
	}

	if ((err = ctrl = max34417_control_read(data)) < 0) {
		dev_err(&client->dev, "Error reading control register\n");
		return err;
	}

	if ((err = max34417_control_write(data, ctrl | MAX34417_CTRL_MODE))) {
		dev_err(&client->dev, "Error writing control register\n");
		return err;
	}

	data->update_interval = MAX34417_DEFAULT_UPDATE_INTERVAL_MS;
	for (i = 0; i < 4; i++) {
		data->power_uw[i] = 0;
		data->voltage_mv[i] = 0;
	}

	mutex_init(&data->telemetry_lock);

	INIT_DELAYED_WORK(&data->update_work, max34417_update_handler);
	INIT_DELAYED_WORK(&data->read_work, max34417_read_handler);

	schedule_delayed_work(&data->update_work, 0);

	return 0;
}

static int max34417_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct max34417_data *data;
	struct device *hwmon_dev;
	int err;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE |
		I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_READ_I2C_BLOCK
	)) {
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev, sizeof(struct max34417_data), GFP_KERNEL);
	if (!data) return -ENOMEM;

	data->client = client;

	err = max34417_init_client(client, data);
	if (err) {
		return err;
	}

	hwmon_dev = devm_hwmon_device_register_with_info(&client->dev, client->name,
								data,
								&max34417_chip_info,
								NULL);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct of_device_id max34417_of_match_table[] = {
	{ .compatible = "maxim,max34417", },
	{ }
};
MODULE_DEVICE_TABLE(of, max34417_of_match_table);

static const struct i2c_device_id max34417_id[] = {
	{ "max34417", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max34417_id);

static struct i2c_driver max34417_driver = {
	.class		= I2C_CLASS_HWMON,
	.probe		= max34417_probe,
	.driver = {
		.name	= "max34417",
		.of_match_table = max34417_of_match_table
	},
	.id_table	= max34417_id,
};

module_i2c_driver(max34417_driver);

MODULE_AUTHOR("Viktor Sannum <v-visann@microsoft.com>");
MODULE_DESCRIPTION("MAX34417 power monitor driver");
MODULE_LICENSE("GPL");
