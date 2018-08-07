/*
 * userspace-consumer-wrapper.c -- Userspace Consumer Device Tree Wrapper
 *
 * Copyright 2016 CompuLab, Ltd.
 *
 * Author: Uri Mashiach <uri.mashiach@compulab.co.il>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/regulator/userspace-consumer.h>
#include <linux/regulator/consumer.h>

static struct regulator_bulk_data reg_consumer_supply = {
	.supply = NULL
};

static struct regulator_userspace_consumer_data reg_consumer_data = {
	.name = NULL,
	.num_supplies = 1,
	.supplies = &reg_consumer_supply,
	.init_on = 0,
};

static struct platform_device_info pdevinfo = {
	.name = "reg-userspace-consumer",
	.id = 0,
	.data = &reg_consumer_data,
	.size_data = sizeof(reg_consumer_data),
};

static int usc_wrapper_probe(struct platform_device *pdev)
{
	struct platform_device *reg_pdev;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;

	reg_consumer_data.name = of_get_property(np, "regulator-name", NULL);
	if (reg_consumer_data.name == NULL) {
		dev_err(dev, "Failed reading regulator-name\n");
		return -ENOENT;
	}
	reg_consumer_supply.supply = reg_consumer_data.name;
	ret = of_property_read_s32(np, "us-folder-num", &pdevinfo.id);
	if (ret) {
		dev_err(dev, "Failed reading us-folder-num\n");
		return -ENOENT;
	}
	reg_pdev = platform_device_register_full(&pdevinfo);
	if (IS_ERR(reg_pdev)) {
		dev_err(dev, "Failed to register user space regulator for bt_en");
		return 1;
	}
	platform_set_drvdata(pdev, reg_pdev);
	return 0;
}

static int usc_wrapper_remove(struct platform_device *pdev)
{
	struct platform_device *reg_pdev = platform_get_drvdata(pdev);

	platform_device_unregister(reg_pdev);
	return 0;
}

static const struct of_device_id userspace_consumer_wrapper_of_match[] = {
	{ .compatible = "userspace-consumer-wrapper", },
	{},
};

MODULE_DEVICE_TABLE(of, userspace_consumer_wrapper_of_match);

static struct platform_driver userspace_consumer_wrapper_driver = {
	.driver = {
		.name   = "userspace-consumer-wrapper",
		.of_match_table = userspace_consumer_wrapper_of_match,
		.owner  = THIS_MODULE,
	},
	.probe  = usc_wrapper_probe,
	.remove = usc_wrapper_remove,
};


module_platform_driver(userspace_consumer_wrapper_driver);

MODULE_AUTHOR("Uri Mashiach <uri.mashiach@compulab.co.il>");
MODULE_DESCRIPTION("Userspace consumer device tree wrapper");
MODULE_LICENSE("GPL v2");
