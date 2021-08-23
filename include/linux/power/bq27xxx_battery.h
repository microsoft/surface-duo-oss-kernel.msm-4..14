/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef __LINUX_BQ27X00_BATTERY_H__
#define __LINUX_BQ27X00_BATTERY_H__
// MSCHANGE end: Add battery discharge sensor for fg-pack thermal zones
#include "linux/thermal.h"

// MSCHANGE Manufacturer Info Block A sizes
#define MFG_NAME_SIZE                       0x3 // 3 bytes allotted for manufacturer name
#define DEVICE_NAME_SIZE                    0x8
#define CHEM_SIZE                           0x4

// MSCHANGE adding debugfs node to fg driver
#define MAX_ALLOWED_TEMPERATURE       2000 //deciC
#define MAX_ALLOWED_RSOC 			  100  // %

enum bq27xxx_chip {
	BQ27000 = 1, /* bq27000, bq27200 */
	BQ27010, /* bq27010, bq27210 */
	BQ2750X, /* bq27500 deprecated alias */
	BQ2751X, /* bq27510, bq27520 deprecated alias */
	BQ2752X,
	BQ27500, /* bq27500/1 */
	BQ27510G1, /* bq27510G1 */
	BQ27510G2, /* bq27510G2 */
	BQ27510G3, /* bq27510G3 */
	BQ27520G1, /* bq27520G1 */
	BQ27520G2, /* bq27520G2 */
	BQ27520G3, /* bq27520G3 */
	BQ27520G4, /* bq27520G4 */
	BQ27530, /* bq27530, bq27531 */
	BQ27531,
	BQ27541, /* bq27541, bq27542, bq27546, bq27742 */
	BQ27542,
	BQ27546,
	BQ27742,
	BQ27545, /* bq27545 */
	BQ27421, /* bq27421, bq27425, bq27441, bq27621 */
	BQ27425,
	BQ27441,
	BQ27621,
};

// MSCHANGE Manufacturer Info Block A
#pragma pack(push, 1)
typedef struct BQ27742_MANUF_INFO_TYPE
{
    uint16_t batt_manufacture_date;                         // BatteryManufactureDate
    uint32_t batt_serial_num;                               // BatterySerialNumber
    char batt_manufacture_name[MFG_NAME_SIZE];              // BatteryManufactureName
    char batt_device_name[DEVICE_NAME_SIZE];                // BatteryDeviceName
    char batt_chemistry[CHEM_SIZE];                         // Chemistry
}*PBQ27742_MANUF_INFO_TYPE;
#pragma pack(pop)

struct bq27xxx_device_info;
struct bq27xxx_access_methods {
	int (*read)(struct bq27xxx_device_info *di, u8 reg, bool single);
	int (*write)(struct bq27xxx_device_info *di, u8 reg, int value, bool single);
	int (*read_bulk)(struct bq27xxx_device_info *di, u8 reg, u8 *data, int len);
	int (*write_bulk)(struct bq27xxx_device_info *di, u8 reg, u8 *data, int len);
};

struct bq27xxx_reg_cache {
	int temperature;
	int time_to_empty;
	int time_to_empty_avg;
	int time_to_full;
	int charge_full;
	int cycle_count;
	int capacity;
	int energy;
	int flags;
	int power_avg;
	int health;
};

struct bq27xxx_device_info {
	struct device *dev;
	int id;
	enum bq27xxx_chip chip;
	u32 opts;
	const char *name;
	struct bq27xxx_dm_reg *dm_regs;
	u32 unseal_key;
	struct bq27xxx_access_methods bus;
	struct bq27xxx_reg_cache cache;
	int charge_design_full;
	unsigned long last_update;
	struct delayed_work work;
	struct power_supply *bat;
	// MSCHANGE end: Add battery discharge sensor for fg-pack thermal zones
	struct thermal_zone_device *tzd;
	struct list_head list;
	struct mutex lock;
	u8 *regs;
	struct dentry *dfs_root;  // MSCHANGE adding debugfs node to fg driver
	int override_temperature; // MSCHANGE adding debugfs node to fg driver
	int override_capacity; 	  // MSCHANGE adding debugfs node to fg driver
};

void bq27xxx_battery_update(struct bq27xxx_device_info *di);
int bq27xxx_battery_setup(struct bq27xxx_device_info *di);
void bq27xxx_battery_teardown(struct bq27xxx_device_info *di);

#endif
