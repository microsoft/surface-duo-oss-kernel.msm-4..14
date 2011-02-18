/*
 * drivers/power/virtual_battery.c
 *
 * Virtual battery driver
 *
 * Copyright (C) 2008 Pylone, Inc.
 * Author: Masashi YOKOTA <yokota@pylone.jp>
 * Modified by: Akihiro MAEDA <sola.1980.a@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>


/* module parameters */
static int ac_status            = 1; /* online */
static int battery_status       = POWER_SUPPLY_STATUS_CHARGING;
static int battery_health       = POWER_SUPPLY_HEALTH_GOOD;
static int battery_present      = 1; /* true */
static int battery_technology   = POWER_SUPPLY_TECHNOLOGY_LION;
static int battery_capacity     = 50;


static struct platform_device *bat_pdev;

static enum power_supply_property virtual_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property virtual_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static int virtual_ac_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	int ret = 0;

	dev_dbg(&bat_pdev->dev, "%s: psp=%d\n", __func__, psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = ac_status;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int virtual_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;

	dev_dbg(&bat_pdev->dev, "%s: psp=%d\n", __func__, psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = battery_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = battery_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = battery_present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = battery_technology;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = battery_capacity;
		break; default: ret = -EINVAL;
		break;
	}

	return ret;
}

static struct power_supply power_supply_ac = {
	.properties = virtual_ac_props,
	.num_properties = ARRAY_SIZE(virtual_ac_props),
	.get_property = virtual_ac_get_property,
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
};

static struct power_supply power_supply_bat = {
	.properties = virtual_battery_props,
	.num_properties = ARRAY_SIZE(virtual_battery_props),
	.get_property = virtual_battery_get_property,
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
};


struct battery_property_map {
	int value;
	char const * key;
};

static struct battery_property_map map_ac_online[] = {
	{ 0,  "on"  },
	{ 1,  "off" },
	{ -1, NULL  },
};

static struct battery_property_map map_status[] = {
	{ POWER_SUPPLY_STATUS_CHARGING,     "charging"     },
	{ POWER_SUPPLY_STATUS_DISCHARGING,  "discharging"  },
	{ POWER_SUPPLY_STATUS_NOT_CHARGING, "not-charging" },
	{ POWER_SUPPLY_STATUS_FULL,         "full"         },
	{ -1,                               NULL           },
};

static struct battery_property_map map_health[] = {
	{ POWER_SUPPLY_HEALTH_GOOD,           "good"        },
	{ POWER_SUPPLY_HEALTH_OVERHEAT,       "overheat"    },
	{ POWER_SUPPLY_HEALTH_DEAD,           "dead"        },
	{ POWER_SUPPLY_HEALTH_OVERVOLTAGE,    "overvoltage" },
	{ POWER_SUPPLY_HEALTH_UNSPEC_FAILURE, "failure"     },
	{ -1,                                 NULL          },
};

static struct battery_property_map map_present[] = {
	{ 0,  "false" },
	{ 1,  "true"  },
	{ -1, NULL    },
};

static struct battery_property_map map_technology[] = {
	{ POWER_SUPPLY_TECHNOLOGY_NiMH, "NiMH" },
	{ POWER_SUPPLY_TECHNOLOGY_LION, "LION" },
	{ POWER_SUPPLY_TECHNOLOGY_LIPO, "LIPO" },
	{ POWER_SUPPLY_TECHNOLOGY_LiFe, "LiFe" },
	{ POWER_SUPPLY_TECHNOLOGY_NiCd, "NiCd" },
	{ POWER_SUPPLY_TECHNOLOGY_LiMn, "LiMn" },
	{ -1,				NULL   },
};


static int map_get_value(struct battery_property_map * map, const char * key, int def_val)
{
	char buf[4096];
	int cr;

	strcpy(buf, key);
	cr = strlen(buf) - 1;
	if (buf[cr] == '\n')
		buf[cr] = '\0';

	while (map->key) {
		if (strcasecmp(map->key, buf) == 0)
			return map->value;
		map++;
	}

	return def_val;
}


static const char * map_get_key(struct battery_property_map * map, int value, const char * def_key)
{
	while (map->key) {
		if (map->value == value)
			return map->key;
		map++;
	}

	return def_key;
}

static int param_set_ac_status(const char *key, const struct kernel_param *kp)
{
	dev_dbg(&bat_pdev->dev, "%s: name=%s, key=%s\n", __func__, kp->name, key);
	ac_status = map_get_value( map_ac_online, key, ac_status);
	power_supply_changed(&power_supply_ac);
	return 0;
}

static int param_get_ac_status(char *buffer, const struct kernel_param *kp)
{
	dev_dbg(&bat_pdev->dev, "%s: name=%s\n", __func__, kp->name);
	strcpy(buffer, map_get_key( map_ac_online, ac_status, "unknown"));
	return strlen(buffer);
}

static int param_set_battery_status(const char *key, const struct kernel_param *kp)
{
	dev_dbg(&bat_pdev->dev, "%s: name=%s, key=%s.\n", __func__, kp->name, key);
	battery_status = map_get_value( map_status, key, battery_status);
	power_supply_changed(&power_supply_bat);
	return 0;
}

static int param_get_battery_status(char *buffer, const struct kernel_param *kp)
{
	dev_dbg(&bat_pdev->dev, "%s: name=%s\n", __func__, kp->name);
	strcpy(buffer, map_get_key( map_status, battery_status, "unknown"));
	return strlen(buffer);
}

static int param_set_battery_health(const char *key, const struct kernel_param *kp)
{
	dev_dbg(&bat_pdev->dev, "%s: name=%s, key=%s\n", __func__, kp->name, key);
	battery_health = map_get_value( map_health, key, battery_health);
	power_supply_changed(&power_supply_bat);
	return 0;
}

static int param_get_battery_health(char *buffer, const struct kernel_param *kp)
{
	dev_dbg(&bat_pdev->dev, "%s: name=%s\n", __func__, kp->name);
	strcpy(buffer, map_get_key( map_health, battery_health, "unknown"));
	return strlen(buffer);
}

static int param_set_battery_present(const char *key, const struct kernel_param *kp)
{
	dev_dbg(&bat_pdev->dev, "%s: name=%s, key=%s\n", __func__, kp->name, key);
	battery_present = map_get_value( map_present, key, battery_present);
	power_supply_changed(&power_supply_ac);
	return 0;
}

static int param_get_battery_present(char *buffer, const struct kernel_param *kp)
{
	dev_dbg(&bat_pdev->dev, "%s: name=%s\n", __func__, kp->name);
	strcpy(buffer, map_get_key( map_present, battery_present, "unknown"));
	return strlen(buffer);
}

static int param_set_battery_technology(const char *key, const struct kernel_param *kp)
{
	dev_dbg(&bat_pdev->dev, "%s: name=%s, key=%s\n", __func__, kp->name, key);
	battery_technology = map_get_value( map_technology, key, battery_technology);
	power_supply_changed(&power_supply_bat);
	return 0;
}

static int param_get_battery_technology(char *buffer, const struct kernel_param *kp)
{
	dev_dbg(&bat_pdev->dev, "%s: name=%s\n", __func__, kp->name);
	strcpy(buffer, map_get_key( map_technology, battery_technology, "unknown"));
	return strlen(buffer);
}

static int param_set_battery_capacity(const char *key, const struct kernel_param *kp)
{
	int tmp;

	dev_dbg(&bat_pdev->dev, "%s: name=%s, key=%s\n", __func__, kp->name, key);

	if (1 != sscanf(key, "%d", &tmp))
		return -EINVAL;

	battery_capacity = tmp;
	power_supply_changed(&power_supply_bat);
	return 0;
}

#define param_get_battery_capacity param_get_int

static int __init virtual_battery_init(void)
{
	int ret;

	bat_pdev = platform_device_register_simple(KBUILD_BASENAME, 0, NULL, 0);
	if (IS_ERR(bat_pdev))
		return PTR_ERR(bat_pdev);

	ret = power_supply_register(&bat_pdev->dev, &power_supply_ac);
	if (ret)
		goto err_battery_failed;

	ret = power_supply_register(&bat_pdev->dev, &power_supply_bat);
	if (ret)
		goto err_ac_failed;

	printk(KERN_INFO KBUILD_BASENAME": registered \n");
	return 0;

 err_battery_failed:
	power_supply_unregister(&power_supply_ac);
 err_ac_failed:
	return ret;
}

static void __exit virtual_battery_exit(void)
{
	power_supply_unregister(&power_supply_ac);
	power_supply_unregister(&power_supply_bat);
	platform_device_unregister(bat_pdev);
	printk(KERN_INFO KBUILD_BASENAME": unregistered \n");
}

static struct kernel_param_ops param_ops_ac_status = {
	.set = param_set_ac_status,
	.get = param_get_ac_status,
};

static struct kernel_param_ops param_ops_battery_status = {
	.set = param_set_battery_status,
	.get = param_get_battery_status,
};

static struct kernel_param_ops param_ops_battery_present = {
	.set = param_set_battery_present,
	.get = param_get_battery_present,
};

static struct kernel_param_ops param_ops_battery_technology = {
	.set = param_set_battery_technology,
	.get = param_get_battery_technology,
};

static struct kernel_param_ops param_ops_battery_health = {
	.set = param_set_battery_health,
	.get = param_get_battery_health,
};

static struct kernel_param_ops param_ops_battery_capacity = {
	.set = param_set_battery_capacity,
	.get = param_get_battery_capacity,
};

module_init(virtual_battery_init);
module_exit(virtual_battery_exit);

#define param_check_ac_status(name, p) __param_check(name, p, void);
#define param_check_battery_status(name, p) __param_check(name, p, void);
#define param_check_battery_present(name, p) __param_check(name, p, void);
#define param_check_battery_technology(name, p) __param_check(name, p, void);
#define param_check_battery_health(name, p) __param_check(name, p, void);
#define param_check_battery_capacity(name, p) __param_check(name, p, void);

module_param(ac_status, ac_status, 0644);
MODULE_PARM_DESC(ac_status, "AC charging state <on|off>");

module_param(battery_status, battery_status, 0644);
MODULE_PARM_DESC(battery_status, "battery status <charging|discharging|not-charging|full>");

module_param(battery_present, battery_present, 0644);
MODULE_PARM_DESC(battery_present, "battery presence state <good|overheat|dead|overvoltage|failure>");

module_param(battery_technology, battery_technology, 0644);
MODULE_PARM_DESC(battery_technology, "battery technology <NiMH|LION|LIPO|LiFe|NiCd|LiMn>");

module_param(battery_health, battery_health, 0644);
MODULE_PARM_DESC(battery_health, "battery health state <good|overheat|dead|overvoltage|failure>");

module_param(battery_capacity, battery_capacity, 0644);
MODULE_PARM_DESC(battery_capacity, "battery capacity (percentage)");


MODULE_AUTHOR("Masashi YOKOTA <yokota@pylone.jp>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Virtual battery driver");
MODULE_ALIAS("platform:"KBUILD_BASENAME);
