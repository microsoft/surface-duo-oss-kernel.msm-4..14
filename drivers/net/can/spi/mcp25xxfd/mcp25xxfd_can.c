// SPDX-License-Identifier: GPL-2.0

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>

#include "mcp25xxfd_can.h"
#include "mcp25xxfd_cmd.h"
#include "mcp25xxfd_priv.h"
#include "mcp25xxfd_regs.h"

int mcp25xxfd_can_get_mode(struct mcp25xxfd_priv *priv, u32 *reg)
{
	int ret;

	ret = mcp25xxfd_cmd_read(priv->spi, MCP25XXFD_CAN_CON, reg);
	if (ret)
		return ret;

	return FIELD_GET(MCP25XXFD_CAN_CON_OPMOD_MASK, *reg);
}

static int mcp25xxfd_can_switch_mode(struct mcp25xxfd_priv *priv,
				     u32 *reg, int mode)
{
	int ret, i;

	ret = mcp25xxfd_can_get_mode(priv, reg);
	if (ret < 0)
		return ret;

	/* Compute the effective mode in osc*/
	*reg &= ~(MCP25XXFD_CAN_CON_REQOP_MASK |
		  MCP25XXFD_CAN_CON_OPMOD_MASK);
	*reg |= FIELD_PREP(MCP25XXFD_CAN_CON_REQOP_MASK, mode) |
		FIELD_PREP(MCP25XXFD_CAN_CON_OPMOD_MASK, mode);

	/* Request the mode switch */
	ret = mcp25xxfd_cmd_write(priv->spi, MCP25XXFD_CAN_CON, *reg);
	if (ret)
		return ret;

	/* Wait for 256 rounds to stabilize. This is essentially > 12ms
	 * at 1MHz
	 */
	for (i = 0; i < 256; i++) {
		/* get the mode */
		ret = mcp25xxfd_can_get_mode(priv, reg);
		if (ret < 0)
			return ret;
		/* check that we have reached our mode */
		if (ret == mode)
			return 0;
	}

	dev_err(&priv->spi->dev, "Failed to switch to mode %u in time\n",
		mode);

	return -ETIMEDOUT;
}

static int mcp25xxfd_can_probe_modeswitch(struct mcp25xxfd_priv *priv)
{
	u32 mode_data;
	int ret;

	/* We should be in config mode now, so move to INT_LOOPBACK */
	ret = mcp25xxfd_can_switch_mode(priv, &mode_data,
					MCP25XXFD_CAN_CON_MODE_INT_LOOPBACK);
	if (ret) {
		dev_err(&priv->spi->dev,
			"Failed to switch into loopback mode\n");
		return ret;
	}

	/* Switch back into config mode */
	ret = mcp25xxfd_can_switch_mode(priv, &mode_data,
					MCP25XXFD_CAN_CON_MODE_CONFIG);
	if (ret) {
		dev_err(&priv->spi->dev,
			"Failed to switch back to config mode\n");
		return ret;
	}

	return 0;
}

int mcp25xxfd_can_probe(struct mcp25xxfd_priv *priv)
{
	struct spi_device *spi = priv->spi;
	u32 mode_data;
	int mode, ret;

	/* For sanity check read TXQCON register. The TXEN bit should always
	 * be read as 1.
	 */
	ret = mcp25xxfd_cmd_read(spi, MCP25XXFD_CAN_TXQCON, &mode_data);
	if (ret)
		return ret;

	if ((mode_data & MCP25XXFD_CAN_TXQCON_TXEN) == 0) {
		dev_err(&spi->dev, "TXQCON does not have TXEN bit set");
		return -EINVAL;
	}

	/* Try to get the current mode */
	mode = mcp25xxfd_can_get_mode(priv, &mode_data);
	if (mode < 0)
		return mode;

	/* SPI-reset should have moved us into config mode. But then the
	 * documentation says that SPI-reset may only work reliably when already
	 * in config mode. So if we are in config mode then everything is fine
	 * and we check that a mode switch works properly.
	 */
	if (mode == MCP25XXFD_CAN_CON_MODE_CONFIG)
		return mcp25xxfd_can_probe_modeswitch(priv);

	/* Any other mode is unexpected */
	dev_err(&spi->dev,
		"Found controller in unexpected mode: %d\n", mode);

	/* Once again try to move to config mode. If this fails, we'll
	 * bail out.
	 */
	ret = mcp25xxfd_can_switch_mode(priv, &mode_data,
					MCP25XXFD_CAN_CON_MODE_CONFIG);
	if (ret) {
		dev_err(&priv->spi->dev,
			"Unable to switch to config mode\n");
		return -EINVAL;
	}

	/* Finally check if modeswitch is really working */
	return mcp25xxfd_can_probe_modeswitch(priv);
}
