/* SPDX-License-Identifier: GPL-2.0 */

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 */

#ifndef __MCP25XXFD_PRIV_H
#define __MCP25XXFD_PRIV_H

#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include "mcp25xxfd_regs.h"

#define DEVICE_NAME "mcp25xxfd"
#define CLOCK_4_MHZ 4000000
#define CLOCK_10_MHZ 10000000
#define CLOCK_40_MHZ 40000000

enum mcp25xxfd_model {
	CAN_MCP2517FD	= 0x2517,
};

struct mcp25xxfd_can_priv;
struct mcp25xxfd_priv {
	struct spi_device *spi;
	struct clk *clk;
	struct mcp25xxfd_can_priv *cpriv;

	/* actual model of the mcp25xxfd */
	enum mcp25xxfd_model model;

	/* full device name used for interrupts */
	char device_name[32];

	int clock_freq;
	struct regulator *power;

	/* configuration registers */
	struct {
		u32 osc;
		u32 iocon;
		u32 crc;
		u32 ecccon;
	} regs;
};

#endif /* __MCP25XXFD_PRIV_H */
