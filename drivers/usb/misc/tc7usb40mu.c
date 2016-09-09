/**
 * Copyright (C) 2016 Linaro Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/extcon.h>
#include <linux/gpio/consumer.h>
#include <linux/notifier.h>

struct tc7usb40mu_drv {
	struct gpio_desc *gpio;
	struct extcon_dev *edev;
	struct notifier_block notify;
};

static int tc7usb40mu_notify(struct notifier_block *nb, unsigned long event,
			     void *ptr)
{
	struct tc7usb40mu_drv *drv;

	drv = container_of(nb, struct tc7usb40mu_drv, notify);
	if (event)
		gpiod_set_value_cansleep(drv->gpio, 1); /* USB HUB */
	else
		gpiod_set_value_cansleep(drv->gpio, 0); /* device connector */

	return NOTIFY_OK;
}

static int tc7usb40mu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tc7usb40mu_drv *drv;
	int state, ret;
	enum gpiod_flags flags;

	drv = devm_kzalloc(dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	drv->edev = extcon_get_edev_by_phandle(dev, 0);
	if (IS_ERR(drv->edev))
		return PTR_ERR(drv->edev);

	/*
	 * TODO: This can race with extcon changing state before we request the
	 * gpio or the extcon changing state before we register the notifier
	 */
	state = extcon_get_cable_state_(drv->edev, EXTCON_USB_HOST);
	if (state)
		flags = GPIOD_OUT_HIGH;
	else
		flags = GPIOD_OUT_LOW;

	drv->gpio = devm_gpiod_get(dev, "switch", flags);
	if (IS_ERR(drv->gpio))
		return PTR_ERR(drv->gpio);

	drv->notify.notifier_call = tc7usb40mu_notify;
	ret = extcon_register_notifier(drv->edev, EXTCON_USB_HOST, &drv->notify);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, drv);

	return 0;
}

static int tc7usb40mu_remove(struct platform_device *pdev)
{
	struct tc7usb40mu_drv *drv;

	drv = platform_get_drvdata(pdev);
	extcon_unregister_notifier(drv->edev, EXTCON_USB_HOST, &drv->notify);

	return 0;
}

static const struct of_device_id tc7usb40mu_dt_match[] = {
	{ .compatible = "toshiba,tc7usb40mu", },
	{ }
};
MODULE_DEVICE_TABLE(of, tc7usb40mu_dt_match);

static struct platform_driver tc7usb40mu_driver = {
	.probe		= tc7usb40mu_probe,
	.remove		= tc7usb40mu_remove,
	.driver		= {
		.name	= "tc7usb40mu",
		.of_match_table = tc7usb40mu_dt_match,
	},
};
module_platform_driver(tc7usb40mu_driver);

MODULE_AUTHOR("Stephen Boyd <stephen.boyd@linaro.org>");
MODULE_DESCRIPTION("TC7USB40MU USB multiplexer driver");
MODULE_LICENSE("GPL");
