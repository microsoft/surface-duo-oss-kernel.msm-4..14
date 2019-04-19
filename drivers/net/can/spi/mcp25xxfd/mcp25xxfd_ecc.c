// SPDX-License-Identifier: GPL-2.0

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 */

#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include "mcp25xxfd_cmd.h"
#include "mcp25xxfd_ecc.h"
#include "mcp25xxfd_priv.h"
#include "mcp25xxfd_regs.h"

int mcp25xxfd_ecc_clear_int(struct mcp25xxfd_priv *priv)
{
	u32 val, addr;
	int ret;

	/* First report the error address */
	ret = mcp25xxfd_cmd_read(priv->spi, MCP25XXFD_ECCSTAT, &val);
	if (ret)
		return ret;

	/* If no flags are set then nothing to do */
	if (!(val & (MCP25XXFD_ECCSTAT_SECIF | MCP25XXFD_ECCSTAT_DEDIF)))
		return 0;

	addr = (val & MCP25XXFD_ECCSTAT_ERRADDR_MASK) >>
		MCP25XXFD_ECCSTAT_ERRADDR_SHIFT;

	dev_err_ratelimited(&priv->spi->dev, "ECC %s bit error at %03x\n",
			    (val & MCP25XXFD_ECCSTAT_DEDIF) ?
			    "double" : "single",
			    addr);

	/* Clear the error */
	return mcp25xxfd_cmd_write_mask(priv->spi, MCP25XXFD_ECCSTAT, 0,
					MCP25XXFD_ECCSTAT_SECIF |
					MCP25XXFD_ECCSTAT_DEDIF);
}

int mcp25xxfd_ecc_enable_int(struct mcp25xxfd_priv *priv, bool enable)
{
	u32 mask = MCP25XXFD_ECCCON_SECIE | MCP25XXFD_ECCCON_DEDIE;

	priv->regs.ecccon &= ~mask;
	priv->regs.ecccon |= MCP25XXFD_ECCCON_ECCEN | (enable ? mask : 0);

	return mcp25xxfd_cmd_write_mask(priv->spi, MCP25XXFD_ECCCON,
					priv->regs.ecccon,
					MCP25XXFD_ECCCON_ECCEN | mask);
}

int mcp25xxfd_ecc_enable(struct mcp25xxfd_priv *priv)
{
	u8 buffer[256];
	int i, ret;

	ret = mcp25xxfd_ecc_enable_int(priv, false);
	if (ret)
		return ret;

	memset(buffer, 0, sizeof(buffer));
	for (i = 0; i < MCP25XXFD_SRAM_SIZE; i += sizeof(buffer)) {
		ret = mcp25xxfd_cmd_write_multi(priv->spi,
						MCP25XXFD_SRAM_ADDR(i),
						buffer, sizeof(buffer));
		if (ret)
			return ret;
	}

	return 0;
}
