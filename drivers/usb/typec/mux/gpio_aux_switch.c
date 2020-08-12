// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for GPIO controlled aux switch
 *
 * Copyright (C) 2020 The Linux Foundation
 */

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/pd.h>

struct gpio_aux_switch {
	struct typec_mux *typec_mux;
	struct gpio_desc *gpio_en, *gpio_cc;
};

static int gpio_typec_mux_set(struct typec_mux *mux, struct typec_mux_state *state)
{
	struct gpio_aux_switch *gas = typec_mux_get_drvdata(mux);
	bool enable = false, reverse = false;

	if (!state->alt)
		return 0;

	if (typec_altmode_get_orientation(state->alt) == TYPEC_ORIENTATION_REVERSE)
		reverse = true;

	if (state->alt->svid == USB_TYPEC_DP_SID && state->alt->active)
		enable = true;

	gpiod_set_value(gas->gpio_en, !enable);
	gpiod_set_value(gas->gpio_cc, reverse);

	return 0;
}

static int gpio_aux_switch_probe(struct platform_device *pdev)
{
	struct typec_mux_desc mux_desc = { };
	struct device *dev = &pdev->dev;
	struct gpio_aux_switch *gas;

	gas = devm_kzalloc(dev, sizeof(*gas), GFP_KERNEL);
	if (!gas)
		return -ENOMEM;

	gas->gpio_en = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	gas->gpio_cc = devm_gpiod_get(dev, "cc", GPIOD_OUT_LOW);

	mux_desc.fwnode = dev->fwnode;
	mux_desc.drvdata = gas;
	mux_desc.set = gpio_typec_mux_set;
	gas->typec_mux = typec_mux_register(dev, &mux_desc);
	if (IS_ERR(gas->typec_mux))
		return PTR_ERR(gas->typec_mux);

	platform_set_drvdata(pdev, gas);

	return 0;
}

static int gpio_aux_switch_remove(struct platform_device *pdev)
{
	struct gpio_aux_switch *gas = platform_get_drvdata(pdev);

	typec_mux_unregister(gas->typec_mux);

	return 0;
}

static const struct of_device_id gpio_aux_switch_dt_match[] = {
	{ .compatible = "gpio-aux-switch" },
	{ }
};

static struct platform_driver gpio_aux_switch_driver = {
	.driver = {
		.name = "gpio-aux-switch",
		.of_match_table = gpio_aux_switch_dt_match,
	},
	.probe = gpio_aux_switch_probe,
	.remove = gpio_aux_switch_remove,
};

module_platform_driver(gpio_aux_switch_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("GPIO AUX Switch driver");
