/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __WCD9335_H__
#define __WCD9335_H__

#include <linux/slimbus.h>
#include <linux/regulator/consumer.h>

#define WCD9335_VERSION_2_0     2
#define WCD9335_MAX_SUPPLY	5

enum wcd_interface_type {
	WCD9335_INTERFACE_TYPE_SLIMBUS = 1,
	WCD9335_INTERFACE_TYPE_I2C,
};

struct wcd9335 {
	int version;
	int intr1;
	int reset_gpio;
	enum wcd_interface_type intf_type;
	struct device *dev;
	struct clk *mclk;
	struct clk *native_clk;
	struct slim_device *slim;
	struct slim_device *slim_ifc_dev;
	struct regmap *regmap;
	struct regmap *ifc_dev_regmap;
	struct regulator_bulk_data supplies[WCD9335_MAX_SUPPLY];
};

#endif /* __WCD9335_H__ */
