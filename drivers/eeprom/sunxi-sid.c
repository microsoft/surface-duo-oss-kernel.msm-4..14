/*
 * Allwinner sunXi SoCs Security ID support.
 *
 * Copyright (c) 2013 Oliver Schinagl <oliver@schinagl.nl>
 * Copyright (C) 2014 Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/eeprom-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

struct eeprom_sid {
	void __iomem	*membase;
	struct eeprom_device *eeprom;
};

static struct eeprom_config econfig = {
	.name = "sunix-sid",
};

/* We read the entire key, due to a 32 bit read alignment requirement. Since we
 * want to return the requested byte, this results in somewhat slower code and
 * uses 4 times more reads as needed but keeps code simpler. Since the SID is
 * only very rarely probed, this is not really an issue.
 */
static int sunxi_sid_reg_read(void *context,
			      unsigned int offset, unsigned int *val)
{
	struct eeprom_sid *sid  = context;
	u32 sid_key;

	sid_key = ioread32be(sid->membase + round_down(offset, 4));
	sid_key >>= (offset % 4) * 8;

	*val = sid_key;

	return 0;
}

static bool sunxi_sid_writeable_reg(struct device *dev, unsigned int reg)
{
	return false;
}

static const struct of_device_id sunxi_sid_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-sid", .data = (void *)16},
	{ .compatible = "allwinner,sun7i-a20-sid", .data = (void *)512},
	{/* sentinel */},
};
MODULE_DEVICE_TABLE(of, sunxi_sid_of_match);

static struct regmap_config sunxi_sid_regmap_config = {
	.reg_bits = 32,
	.val_bits = 8,
	.reg_stride = 1,
	.reg_read = sunxi_sid_reg_read,
	.writeable_reg = sunxi_sid_writeable_reg,
};

static int sunxi_sid_probe(struct platform_device *pdev)
{
	const struct of_device_id *device;
	struct eeprom_sid *sid;
	struct resource *res;
	struct eeprom_device *eeprom;
	struct device *dev = &pdev->dev;
	struct regmap *regmap;

	sid = devm_kzalloc(dev, sizeof(*sid), GFP_KERNEL);
	if (!sid)
		return -ENOMEM;


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sid->membase = devm_ioremap_resource(dev, res);
	if (IS_ERR(sid->membase))
		return PTR_ERR(sid->membase);

	device = of_match_device(sunxi_sid_of_match, dev);
	if (!device)
		return -ENODEV;

	sunxi_sid_regmap_config.max_register = (unsigned int)device->data - 1;

	regmap = devm_regmap_init(dev, NULL,
					  sid, &sunxi_sid_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	econfig.dev = dev;
	econfig.owner = THIS_MODULE;

	eeprom = eeprom_register(&econfig);
	if (IS_ERR(eeprom))
		return PTR_ERR(eeprom);

	platform_set_drvdata(pdev, eeprom);

	return 0;
}

static int sunxi_sid_remove(struct platform_device *pdev)
{
	struct eeprom_device *eeprom = platform_get_drvdata(pdev);

	return eeprom_unregister(eeprom);
}

static struct platform_driver sunxi_sid_driver = {
	.probe = sunxi_sid_probe,
	.remove = sunxi_sid_remove,
	.driver = {
		.name = "eeprom-sunxi-sid",
		.of_match_table = sunxi_sid_of_match,
	},
};
module_platform_driver(sunxi_sid_driver);

MODULE_AUTHOR("Oliver Schinagl <oliver@schinagl.nl>");
MODULE_DESCRIPTION("Allwinner sunxi security id driver");
MODULE_LICENSE("GPL");
