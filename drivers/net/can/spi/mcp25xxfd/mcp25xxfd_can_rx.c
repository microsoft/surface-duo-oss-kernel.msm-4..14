// SPDX-License-Identifier: GPL-2.0

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 *
 * Based on Microchip MCP251x CAN controller driver written by
 * David Vrabel, Copyright 2006 Arcom Control Systems Ltd.
 */

#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include "mcp25xxfd_cmd.h"
#include "mcp25xxfd_can.h"
#include "mcp25xxfd_can_id.h"
#include "mcp25xxfd_can_priv.h"
#include "mcp25xxfd_can_rx.h"

static struct sk_buff *
mcp25xxfd_can_rx_submit_normal_frame(struct mcp25xxfd_can_priv *cpriv,
				     u32 id, u32 dlc, u8 **data)
{
	struct can_frame *frame;
	struct sk_buff *skb;

	/* allocate frame */
	skb = alloc_can_skb(cpriv->can.dev, &frame);
	if (!skb)
		return NULL;

	/* set id, dlc and flags */
	frame->can_id = id;
	frame->can_dlc = dlc;

	/* and set the pointer to data */
	*data = frame->data;

	return skb;
}

/* it is almost identical except for the type of the frame... */
static struct sk_buff *
mcp25xxfd_can_rx_submit_fd_frame(struct mcp25xxfd_can_priv *cpriv,
				 u32 id, u32 flags, u32 len, u8 **data)
{
	struct canfd_frame *frame;
	struct sk_buff *skb;

	/* allocate frame */
	skb = alloc_canfd_skb(cpriv->can.dev, &frame);
	if (!skb)
		return NULL;

	/* set id, dlc and flags */
	frame->can_id = id;
	frame->len = len;
	frame->flags |= flags;

	/* and set the pointer to data */
	*data = frame->data;

	return skb;
}

int mcp25xxfd_can_rx_submit_frame(struct mcp25xxfd_can_priv *cpriv, int fifo)
{
	struct net_device *net = cpriv->can.dev;
	int addr = cpriv->fifos.info[fifo].offset;
	struct mcp25xxfd_can_obj_rx *rx =
		(struct mcp25xxfd_can_obj_rx *)(cpriv->sram + addr);
	u8 *data = NULL;
	struct sk_buff *skb;
	u32 id, dlc, len, flags;

	/* compute the can_id */
	mcp25xxfd_can_id_from_mcp25xxfd(rx->id, rx->flags, &id);

	/* and dlc */
	dlc = (rx->flags & MCP25XXFD_CAN_OBJ_FLAGS_DLC_MASK) >>
		MCP25XXFD_CAN_OBJ_FLAGS_DLC_SHIFT;
	len = can_dlc2len(dlc);

	/* update stats */
	net->stats.rx_packets++;
	net->stats.rx_bytes += len;

	/* allocate the skb buffer */
	if (rx->flags & MCP25XXFD_CAN_OBJ_FLAGS_FDF) {
		flags = 0;
		flags |= (rx->flags & MCP25XXFD_CAN_OBJ_FLAGS_BRS) ?
			CANFD_BRS : 0;
		flags |= (rx->flags & MCP25XXFD_CAN_OBJ_FLAGS_ESI) ?
			CANFD_ESI : 0;
		skb = mcp25xxfd_can_rx_submit_fd_frame(cpriv, id, flags,
						       len, &data);
	} else {
		skb = mcp25xxfd_can_rx_submit_normal_frame(cpriv, id,
							   len, &data);
	}
	if (!skb) {
		netdev_err(net, "cannot allocate RX skb\n");
		net->stats.rx_dropped++;
		return -ENOMEM;
	}

	/* copy the payload data */
	memcpy(data, rx->data, len);

	/* and submit the frame */
	netif_rx_ni(skb);

	return 0;
}

static int mcp25xxfd_can_rx_read_frame(struct mcp25xxfd_can_priv *cpriv,
				       int fifo, int prefetch_bytes)
{
	struct spi_device *spi = cpriv->priv->spi;
	struct net_device *net = cpriv->can.dev;
	int addr = cpriv->fifos.info[fifo].offset;
	struct mcp25xxfd_can_obj_rx *rx =
		(struct mcp25xxfd_can_obj_rx *)(cpriv->sram + addr);
	int dlc;
	int len, ret;

	/* we read the header plus prefetch_bytes */
	ret = mcp25xxfd_cmd_read_multi(spi, MCP25XXFD_SRAM_ADDR(addr),
				       rx, sizeof(*rx) + prefetch_bytes);
	if (ret)
		return ret;

	/* transpose the headers to CPU format*/
	rx->id = le32_to_cpu(rx->id);
	rx->flags = le32_to_cpu(rx->flags);
	rx->ts = le32_to_cpu(rx->ts);

	/* compute len */
	dlc = (rx->flags & MCP25XXFD_CAN_OBJ_FLAGS_DLC_MASK) >>
		MCP25XXFD_CAN_OBJ_FLAGS_DLC_SHIFT;
	len = can_dlc2len(min_t(int, dlc, (net->mtu == CANFD_MTU) ? 15 : 8));

	/* read the remaining data for canfd frames */
	if (len > prefetch_bytes) {
		/* here the extra portion reading data after prefetch */
		ret = mcp25xxfd_cmd_read_multi(spi,
					       MCP25XXFD_SRAM_ADDR(addr) +
					       sizeof(*rx) + prefetch_bytes,
					       &rx->data[prefetch_bytes],
					       len - prefetch_bytes);
		if (ret)
			return ret;
	}

	/* clear the rest of the buffer - just to be safe */
	memset(rx->data + len, 0, ((net->mtu == CANFD_MTU) ? 64 : 8) - len);

	/* add the fifo to the process queues */
	mcp25xxfd_can_queue_frame(cpriv, fifo, rx->ts, true);

	/* and clear the interrupt flag for that fifo */
	return mcp25xxfd_cmd_write_mask(spi, MCP25XXFD_CAN_FIFOCON(fifo),
					MCP25XXFD_CAN_FIFOCON_FRESET,
					MCP25XXFD_CAN_FIFOCON_FRESET);
}

static int mcp25xxfd_can_rx_read_frames(struct mcp25xxfd_can_priv *cpriv)
{
	int i, f, prefetch;
	int ret;

	prefetch = 8;
	/* TODO: Optimize this */
	for (i = 0, f = cpriv->fifos.rx.start; i < cpriv->fifos.rx.count;
	     i++, f++) {
		if (cpriv->status.rxif & BIT(f)) {
			/* read the frame */
			ret = mcp25xxfd_can_rx_read_frame(cpriv, f, prefetch);
			if (ret)
				return ret;
		}
	}

	return 0;
}

int mcp25xxfd_can_rx_handle_int_rxif(struct mcp25xxfd_can_priv *cpriv)
{
	if (!cpriv->status.rxif)
		return 0;

	/* read all the fifos */
	return mcp25xxfd_can_rx_read_frames(cpriv);
}

int mcp25xxfd_can_rx_handle_int_rxovif(struct mcp25xxfd_can_priv *cpriv)
{
	u32 mask = MCP25XXFD_CAN_FIFOSTA_RXOVIF;
	int ret, i, reg;

	if (!cpriv->status.rxovif)
		return 0;

	/* clear all fifos that have an overflow bit set */
	for (i = 0; i < 32; i++) {
		if (cpriv->status.rxovif & BIT(i)) {
			/* clear fifo status */
			reg = MCP25XXFD_CAN_FIFOSTA(i);
			ret = mcp25xxfd_cmd_write_mask(cpriv->priv->spi,
						       reg, 0, mask);
			if (ret)
				return ret;

			/* update statistics */
			cpriv->can.dev->stats.rx_over_errors++;
			cpriv->can.dev->stats.rx_errors++;

			/* and prepare ERROR FRAME */
			cpriv->error_frame.id |= CAN_ERR_CRTL;
			cpriv->error_frame.data[1] |=
				CAN_ERR_CRTL_RX_OVERFLOW;
		}
	}

	return 0;
}
