// SPDX-License-Identifier: GPL-2.0

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 */

#include <linux/slab.h>
#include <linux/spi/spi.h>

#include "mcp25xxfd_cmd.h"
#include "mcp25xxfd_priv.h"

static int mcp25xxfd_cmd_sync_write(struct spi_device *spi,
				    const void *tx_buf,
				    unsigned int tx_len)
{
	struct spi_transfer xfer;

	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = tx_buf;
	xfer.len = tx_len;

	return spi_sync_transfer(spi, &xfer, 1);
}

static int mcp25xxfd_cmd_write_then_read(struct spi_device *spi,
					 const void *tx_buf,
					 unsigned int tx_len,
					 void *rx_buf,
					 unsigned int rx_len)
{
	struct spi_transfer xfer[2];
	u8 *spi_tx, *spi_rx;
	int xfers;
	int ret;

	spi_tx = kzalloc(tx_len + rx_len, GFP_KERNEL);
	if (!spi_tx)
		return -ENOMEM;

	spi_rx = spi_tx + tx_len;
	memset(xfer, 0, sizeof(xfer));

	/* Special handling for half-duplex */
	if (spi->master->flags & SPI_MASTER_HALF_DUPLEX) {
		xfers = 2;
		xfer[0].tx_buf = spi_tx;
		xfer[0].len = tx_len;
		/* Offset for rx_buf needs to get aligned */
		xfer[1].rx_buf = spi_rx + tx_len;
		xfer[1].len = rx_len;
	} else {
		xfers = 1;
		xfer[0].len = tx_len + rx_len;
		xfer[0].tx_buf = spi_tx;
		xfer[0].rx_buf = spi_rx;
	}

	memcpy(spi_tx, tx_buf, tx_len);
	ret = spi_sync_transfer(spi, xfer, xfers);
	if (ret)
		goto out;

	memcpy(rx_buf, xfer[0].rx_buf + tx_len, rx_len);

out:
	kfree(spi_tx);

	return ret;
}

static int mcp25xxfd_cmd_write_then_write(struct spi_device *spi,
					  const void *tx_buf,
					  unsigned int tx_len,
					  const void *tx2_buf,
					  unsigned int tx2_len)
{
	struct spi_transfer xfer;
	u8 *spi_tx;
	int ret;

	spi_tx = kzalloc(tx_len + tx2_len, GFP_KERNEL);
	if (!spi_tx)
		return -ENOMEM;

	memset(&xfer, 0, sizeof(xfer));
	xfer.len = tx_len + tx2_len;
	xfer.tx_buf = spi_tx;

	memcpy(spi_tx, tx_buf, tx_len);
	memcpy(spi_tx + tx_len, tx2_buf, tx2_len);

	ret = spi_sync_transfer(spi, &xfer, 1);
	kfree(spi_tx);

	return ret;
}

/* Read multiple bytes from registers */
int mcp25xxfd_cmd_read_multi(struct spi_device *spi, u32 reg,
			     void *data, int n)
{
	u8 cmd[2];
	int ret;

	mcp25xxfd_cmd_calc(MCP25XXFD_INSTRUCTION_READ, reg, cmd);

	ret = mcp25xxfd_cmd_write_then_read(spi, &cmd, 2, data, n);
	if (ret)
		return ret;

	return 0;
}

int mcp25xxfd_cmd_read_mask(struct spi_device *spi, u32 reg,
			    u32 *data, u32 mask)
{
	int first_byte, last_byte, len_byte;
	int ret;

	/* Make sure at least one bit is set */
	if (!mask)
		return -EINVAL;

	/* Calculate first and last byte used */
	first_byte = mcp25xxfd_cmd_first_byte(mask);
	last_byte = mcp25xxfd_cmd_last_byte(mask);
	len_byte = last_byte - first_byte + 1;

	*data = 0;
	ret = mcp25xxfd_cmd_read_multi(spi, reg + first_byte,
				       ((void *)data + first_byte), len_byte);
	if (ret)
		return ret;

	mcp25xxfd_cmd_convert_to_cpu(data, 1);

	return 0;
}

/* Write multiple bytes to registers */
int mcp25xxfd_cmd_write_multi(struct spi_device *spi, u32 reg,
			      void *data, int n)
{
	int ret;
	u8 cmd[2];

	mcp25xxfd_cmd_calc(MCP25XXFD_INSTRUCTION_WRITE, reg, cmd);

	ret = mcp25xxfd_cmd_write_then_write(spi, &cmd, 2, data, n);
	if (ret)
		return ret;

	return 0;
}

int mcp25xxfd_cmd_write_mask(struct spi_device *spi, u32 reg,
			     u32 data, u32 mask)
{
	int first_byte, last_byte, len_byte;
	u8 cmd[2];

	/* Make sure at least one bit is set */
	if (!mask)
		return -EINVAL;

	/* calculate first and last byte used */
	first_byte = mcp25xxfd_cmd_first_byte(mask);
	last_byte = mcp25xxfd_cmd_last_byte(mask);
	len_byte = last_byte - first_byte + 1;

	mcp25xxfd_cmd_calc(MCP25XXFD_INSTRUCTION_WRITE,
			   reg + first_byte, cmd);

	mcp25xxfd_cmd_convert_from_cpu(&data, 1);

	return mcp25xxfd_cmd_write_then_write(spi,
					      cmd, sizeof(cmd),
					      ((void *)&data + first_byte),
					      len_byte);
}

int mcp25xxfd_cmd_write_regs(struct spi_device *spi, u32 reg,
			     u32 *data, u32 bytes)
{
	int ret;

	mcp25xxfd_cmd_convert_from_cpu(data, bytes / sizeof(bytes));

	ret = mcp25xxfd_cmd_write_multi(spi, reg, data, bytes);

	mcp25xxfd_cmd_convert_to_cpu(data, bytes / sizeof(bytes));

	return ret;
}

int mcp25xxfd_cmd_read_regs(struct spi_device *spi, u32 reg,
			    u32 *data, u32 bytes)
{
	int ret;

	ret = mcp25xxfd_cmd_read_multi(spi, reg, data, bytes);

	mcp25xxfd_cmd_convert_to_cpu((u32 *)data, bytes / sizeof(bytes));

	return ret;
}

int mcp25xxfd_cmd_reset(struct spi_device *spi)
{
	u8 *cmd;
	int ret;

	cmd = kzalloc(2, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	mcp25xxfd_cmd_calc(MCP25XXFD_INSTRUCTION_RESET, 0, cmd);

	ret = mcp25xxfd_cmd_sync_write(spi, cmd, 2);

	kfree(cmd);

	return ret;
}
