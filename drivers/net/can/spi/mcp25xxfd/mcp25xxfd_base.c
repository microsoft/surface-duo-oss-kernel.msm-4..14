// SPDX-License-Identifier: GPL-2.0

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "mcp25xxfd_base.h"
#include "mcp25xxfd_can.h"
#include "mcp25xxfd_cmd.h"
#include "mcp25xxfd_ecc.h"
#include "mcp25xxfd_int.h"
#include "mcp25xxfd_priv.h"

int mcp25xxfd_base_power_enable(struct regulator *reg, int enable)
{
	if (enable)
		return regulator_enable(reg);
	else
		return regulator_disable(reg);
}

static const struct of_device_id mcp25xxfd_of_match[] = {
	{
		.compatible	= "microchip,mcp2517fd",
		.data		= (void *)CAN_MCP2517FD,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, mcp25xxfd_of_match);

static int mcp25xxfd_base_probe(struct spi_device *spi)
{
	const struct of_device_id *of_id =
		of_match_device(mcp25xxfd_of_match, &spi->dev);
	struct mcp25xxfd_priv *priv;
	struct clk *clk;
	enum mcp25xxfd_model model;
	u32 freq;
	int ret;

	/* Check whether valid IRQ line is defined or not */
	if (spi->irq <= 0) {
		dev_err(&spi->dev, "no valid irq line defined: irq = %i\n",
			spi->irq);
		return -EINVAL;
	}

	priv = devm_kzalloc(&spi->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	spi_set_drvdata(spi, priv);
	priv->spi = spi;

	/* Assign device name */
	snprintf(priv->device_name, sizeof(priv->device_name),
		 DEVICE_NAME "-%s", dev_name(&priv->spi->dev));

	/* assign model from of or driver_data */
	if (of_id)
		model = (enum mcp25xxfd_model)of_id->data;
	else
		model = spi_get_device_id(spi)->driver_data;

	clk = devm_clk_get(&spi->dev, NULL);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto out_free;
	}

	freq = clk_get_rate(clk);
	if (!(freq == CLOCK_4_MHZ || freq == CLOCK_10_MHZ ||
	      freq == CLOCK_40_MHZ)) {
		ret = -ERANGE;
		goto out_free;
	}

	ret = clk_prepare_enable(clk);
	if (ret)
		goto out_free;

	priv->clk = clk;
	priv->clock_freq = freq;

	/* Configure the SPI bus */
	spi->bits_per_word = 8;

	/* The frequency of SCK has to be less than or equal to half the
	 * frequency of SYSCLK.
	 */
	spi->max_speed_hz = freq / 2;
	ret = spi_setup(spi);
	if (ret)
		goto out_clk;

	priv->power = devm_regulator_get(&spi->dev, "vdd");
	if (IS_ERR(priv->power)) {
		if (PTR_ERR(priv->power) != -EPROBE_DEFER)
			dev_err(&spi->dev, "failed to get vdd\n");
		ret = PTR_ERR(priv->power);
		goto out_clk;
	}

	ret = mcp25xxfd_base_power_enable(priv->power, 1);
	if (ret)
		goto out_clk;

	/* disable interrupts */
	ret = mcp25xxfd_int_enable(priv, false);
	if (ret)
		goto out_power;

	/* setup ECC for SRAM */
	ret = mcp25xxfd_ecc_enable(priv);
	if (ret)
		goto out_power;

	/* setting up CAN */
	ret = mcp25xxfd_can_setup(priv);
	if (ret)
		goto out_power;

	dev_info(&spi->dev,
		 "MCP%04x successfully initialized.\n", model);
	return 0;

out_power:
	mcp25xxfd_base_power_enable(priv->power, 0);
out_clk:
	clk_disable_unprepare(clk);
out_free:
	dev_err(&spi->dev, "Probe failed, err=%d\n", -ret);
	return ret;
}

static int mcp25xxfd_base_remove(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);

	mcp25xxfd_can_remove(priv);
	mcp25xxfd_base_power_enable(priv->power, 0);
	clk_disable_unprepare(priv->clk);

	return 0;
}

static const struct spi_device_id mcp25xxfd_id_table[] = {
	{
		.name		= "mcp2517fd",
		.driver_data	= (kernel_ulong_t)CAN_MCP2517FD,
	},
	{ }
};
MODULE_DEVICE_TABLE(spi, mcp25xxfd_id_table);

static struct spi_driver mcp25xxfd_can_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = mcp25xxfd_of_match,
	},
	.id_table = mcp25xxfd_id_table,
	.probe = mcp25xxfd_base_probe,
	.remove = mcp25xxfd_base_remove,
};
module_spi_driver(mcp25xxfd_can_driver);

MODULE_AUTHOR("Martin Sperl <kernel@martin.sperl.org>");
MODULE_DESCRIPTION("Microchip 25XXFD CAN driver");
MODULE_LICENSE("GPL v2");
