// SPDX-License-Identifier: GPL-2.0

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 */

#include <linux/bitfield.h>
#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include "mcp25xxfd_base.h"
#include "mcp25xxfd_can_fifo.h"
#include "mcp25xxfd_can_int.h"
#include "mcp25xxfd_can_priv.h"
#include "mcp25xxfd_can_tx.h"
#include "mcp25xxfd_can.h"
#include "mcp25xxfd_cmd.h"
#include "mcp25xxfd_int.h"
#include "mcp25xxfd_priv.h"
#include "mcp25xxfd_regs.h"

#include <uapi/linux/can/netlink.h>

/* everything related to bit timing */
static
const struct can_bittiming_const mcp25xxfd_can_nominal_bittiming_const = {
	.name           = DEVICE_NAME,
	.tseg1_min      = 2,
	.tseg1_max      = BIT(MCP25XXFD_CAN_NBTCFG_TSEG1_BITS),
	.tseg2_min      = 1,
	.tseg2_max      = BIT(MCP25XXFD_CAN_NBTCFG_TSEG2_BITS),
	.sjw_max        = BIT(MCP25XXFD_CAN_NBTCFG_SJW_BITS),
	.brp_min        = 1,
	.brp_max        = BIT(MCP25XXFD_CAN_NBTCFG_BRP_BITS),
	.brp_inc        = 1,
};

static
const struct can_bittiming_const mcp25xxfd_can_data_bittiming_const = {
	.name           = DEVICE_NAME,
	.tseg1_min      = 1,
	.tseg1_max      = BIT(MCP25XXFD_CAN_DBTCFG_TSEG1_BITS),
	.tseg2_min      = 1,
	.tseg2_max      = BIT(MCP25XXFD_CAN_DBTCFG_TSEG2_BITS),
	.sjw_max        = BIT(MCP25XXFD_CAN_DBTCFG_SJW_BITS),
	.brp_min        = 1,
	.brp_max        = BIT(MCP25XXFD_CAN_DBTCFG_BRP_BITS),
	.brp_inc        = 1,
};

static int mcp25xxfd_can_do_set_nominal_bittiming(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct can_bittiming *bt = &cpriv->can.bittiming;
	int sjw = bt->sjw;
	int pseg2 = bt->phase_seg2;
	int pseg1 = bt->phase_seg1;
	int propseg = bt->prop_seg;
	int brp = bt->brp;
	int tseg1 = propseg + pseg1;
	int tseg2 = pseg2;

	/* calculate nominal bit timing */
	cpriv->regs.nbtcfg = ((sjw - 1) << MCP25XXFD_CAN_NBTCFG_SJW_SHIFT) |
		((tseg2 - 1) << MCP25XXFD_CAN_NBTCFG_TSEG2_SHIFT) |
		((tseg1 - 1) << MCP25XXFD_CAN_NBTCFG_TSEG1_SHIFT) |
		((brp - 1) << MCP25XXFD_CAN_NBTCFG_BRP_SHIFT);

	return mcp25xxfd_cmd_write(cpriv->priv->spi, MCP25XXFD_CAN_NBTCFG,
				   cpriv->regs.nbtcfg);
}

static int mcp25xxfd_can_do_set_data_bittiming(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	struct can_bittiming *bt = &cpriv->can.data_bittiming;
	struct spi_device *spi = priv->spi;
	int sjw = bt->sjw;
	int pseg2 = bt->phase_seg2;
	int pseg1 = bt->phase_seg1;
	int propseg = bt->prop_seg;
	int brp = bt->brp;
	int tseg1 = propseg + pseg1;
	int tseg2 = pseg2;
	int tdco;
	int ret;

	/* set up Transmitter delay compensation */
	cpriv->regs.tdc = FIELD_PREP(MCP25XXFD_CAN_TDC_TDCMOD_MASK,
				     MCP25XXFD_CAN_TDC_TDCMOD_AUTO);

	/* configure TDC offsets */
	tdco = clamp_t(int, bt->brp * tseg1, -64, 63);
	cpriv->regs.tdc &= ~MCP25XXFD_CAN_TDC_TDCO_MASK;
	cpriv->regs.tdc |= FIELD_PREP(MCP25XXFD_CAN_TDC_TDCO_MASK, tdco);

	/* set TDC */
	ret = mcp25xxfd_cmd_write(spi, MCP25XXFD_CAN_TDC, cpriv->regs.tdc);
	if (ret)
		return ret;

	/* calculate data bit timing */
	cpriv->regs.dbtcfg =
		FIELD_PREP(MCP25XXFD_CAN_DBTCFG_SJW_MASK, (sjw - 1)) |
		FIELD_PREP(MCP25XXFD_CAN_DBTCFG_TSEG2_MASK, (tseg2 - 1)) |
		FIELD_PREP(MCP25XXFD_CAN_DBTCFG_TSEG1_MASK, (tseg1 - 1)) |
		FIELD_PREP(MCP25XXFD_CAN_DBTCFG_BRP_MASK, (brp - 1));

	return mcp25xxfd_cmd_write(spi, MCP25XXFD_CAN_DBTCFG,
				   cpriv->regs.dbtcfg);
}

int mcp25xxfd_can_get_mode(struct mcp25xxfd_priv *priv, u32 *reg)
{
	int ret;

	ret = mcp25xxfd_cmd_read(priv->spi, MCP25XXFD_CAN_CON, reg);
	if (ret)
		return ret;

	return FIELD_GET(MCP25XXFD_CAN_CON_OPMOD_MASK, *reg);
}

int mcp25xxfd_can_switch_mode_no_wait(struct mcp25xxfd_priv *priv,
				      u32 *reg, int mode)
{
	int ret;

	ret = mcp25xxfd_can_get_mode(priv, reg);
	if (ret < 0)
		return ret;

	/* Compute the effective mode in osc*/
	*reg &= ~(MCP25XXFD_CAN_CON_REQOP_MASK |
		  MCP25XXFD_CAN_CON_OPMOD_MASK);
	*reg |= FIELD_PREP(MCP25XXFD_CAN_CON_REQOP_MASK, mode) |
		FIELD_PREP(MCP25XXFD_CAN_CON_OPMOD_MASK, mode);

	/* Request the mode switch */
	return mcp25xxfd_cmd_write(priv->spi, MCP25XXFD_CAN_CON, *reg);
}

int mcp25xxfd_can_switch_mode(struct mcp25xxfd_priv *priv, u32 *reg, int mode)
{
	int ret, i;

	/* trigger the mode switch itself */
	ret = mcp25xxfd_can_switch_mode_no_wait(priv, reg, mode);
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

static int mcp25xxfd_can_config(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	struct spi_device *spi = priv->spi;
	int ret;

	/* setup value of con_register */
	cpriv->regs.con = MCP25XXFD_CAN_CON_STEF; /* enable TEF, disable TXQ */

	/* non iso FD mode */
	if (!(cpriv->can.ctrlmode & CAN_CTRLMODE_FD_NON_ISO))
		cpriv->regs.con |= MCP25XXFD_CAN_CON_ISOCRCEN;

	/* one shot */
	if (cpriv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		cpriv->regs.con |= MCP25XXFD_CAN_CON_RTXAT;

	/* apply it now together with a mode switch */
	ret = mcp25xxfd_can_switch_mode(cpriv->priv, &cpriv->regs.con,
					MCP25XXFD_CAN_CON_MODE_CONFIG);
	if (ret)
		return 0;

	/* time stamp control register - 1ns resolution */
	cpriv->regs.tscon = 0;
	ret = mcp25xxfd_cmd_write(spi, MCP25XXFD_CAN_TBC, 0);
	if (ret)
		return ret;

	cpriv->regs.tscon = MCP25XXFD_CAN_TSCON_TBCEN |
			    FIELD_PREP(MCP25XXFD_CAN_TSCON_TBCPRE_MASK,
				       ((cpriv->can.clock.freq / 1000000)));
	ret = mcp25xxfd_cmd_write(spi, MCP25XXFD_CAN_TSCON, cpriv->regs.tscon);
	if (ret)
		return ret;

	/* setup fifos */
	ret = mcp25xxfd_can_fifo_setup(cpriv);
	if (ret)
		return ret;

	/* setup can bittiming now - the do_set_bittiming methods
	 * are not used as they get called before open
	 */
	ret = mcp25xxfd_can_do_set_nominal_bittiming(net);
	if (ret)
		return ret;

	ret = mcp25xxfd_can_do_set_data_bittiming(net);
	if (ret)
		return ret;

	return ret;
}

/* mode setting */
static int mcp25xxfd_can_do_set_mode(struct net_device *net,
				     enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

/* binary error counters */
static int mcp25xxfd_can_get_berr_counter(const struct net_device *net,
					  struct can_berr_counter *bec)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	bec->txerr = FIELD_PREP(MCP25XXFD_CAN_TREC_TEC_MASK,
				cpriv->status.trec);
	bec->rxerr = FIELD_PREP(MCP25XXFD_CAN_TREC_REC_MASK,
				cpriv->status.trec);

	return 0;
}

static int mcp25xxfd_can_open(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct spi_device *spi = cpriv->priv->spi;
	int ret, mode;

	ret = open_candev(net);
	if (ret) {
		netdev_err(net, "unable to set initial baudrate!\n");
		return ret;
	}

	/* request an IRQ but keep disabled for now */
	ret = request_threaded_irq(spi->irq, NULL,
				   mcp25xxfd_can_int,
				   IRQF_ONESHOT | IRQF_TRIGGER_LOW,
				   cpriv->priv->device_name, cpriv);
	if (ret) {
		dev_err(&spi->dev, "failed to acquire irq %d - %i\n",
			spi->irq, ret);
		goto out_candev;
	}

	disable_irq(spi->irq);
	cpriv->irq.allocated = true;
	cpriv->irq.enabled = false;

	/* enable power to the transceiver */
	ret = mcp25xxfd_base_power_enable(cpriv->transceiver, 1);
	if (ret)
		goto out_irq;

	/* configure controller for reception */
	ret = mcp25xxfd_can_config(net);
	if (ret)
		goto out_power;

	/* setting up state */
	cpriv->can.state = CAN_STATE_ERROR_ACTIVE;

	/* enable interrupts */
	ret = mcp25xxfd_int_enable(cpriv->priv, true);
	if (ret)
		goto out_canconfig;

	if (cpriv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		mode = MCP25XXFD_CAN_CON_MODE_EXT_LOOPBACK;
	else if (cpriv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		mode = MCP25XXFD_CAN_CON_MODE_LISTENONLY;
	else if (cpriv->can.ctrlmode & CAN_CTRLMODE_FD)
		mode = MCP25XXFD_CAN_CON_MODE_MIXED;
	else
		mode = MCP25XXFD_CAN_CON_MODE_CAN2_0;

	/* switch to active mode */
	ret = mcp25xxfd_can_switch_mode(cpriv->priv, &cpriv->regs.con, mode);
	if (ret)
		goto out_int;

	/* start the tx_queue */
	mcp25xxfd_can_tx_queue_manage(cpriv,
				      MCP25XXFD_CAN_TX_QUEUE_STATE_STARTED);

	return 0;

out_int:
	mcp25xxfd_int_enable(cpriv->priv, false);
out_canconfig:
	mcp25xxfd_can_fifo_release(cpriv);
out_power:
	mcp25xxfd_base_power_enable(cpriv->transceiver, 0);
out_irq:
	free_irq(spi->irq, cpriv);
	cpriv->irq.allocated = false;
	cpriv->irq.enabled = false;
out_candev:
	close_candev(net);
	return ret;
}

static void mcp25xxfd_can_shutdown(struct mcp25xxfd_can_priv *cpriv)
{
	/* switch us to CONFIG mode - this disables the controller */
	mcp25xxfd_can_switch_mode(cpriv->priv, &cpriv->regs.con,
				  MCP25XXFD_CAN_CON_MODE_CONFIG);
}

static int mcp25xxfd_can_stop(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	struct spi_device *spi = priv->spi;

	/* stop transmit queue */
	mcp25xxfd_can_tx_queue_manage(cpriv,
				      MCP25XXFD_CAN_TX_QUEUE_STATE_STOPPED);

	/* shutdown the can controller */
	mcp25xxfd_can_shutdown(cpriv);

	/* disable inerrupts on controller */
	mcp25xxfd_int_enable(cpriv->priv, false);

	/* disable the transceiver */
	mcp25xxfd_base_power_enable(cpriv->transceiver, 0);

	/* disable interrupt on host */
	free_irq(spi->irq, cpriv);
	cpriv->irq.allocated = false;
	cpriv->irq.enabled = false;

	/* close the can_decice */
	close_candev(net);

	return 0;
}

static const struct net_device_ops mcp25xxfd_netdev_ops = {
	.ndo_open = mcp25xxfd_can_open,
	.ndo_stop = mcp25xxfd_can_stop,
	.ndo_start_xmit = mcp25xxfd_can_tx_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

/* probe and remove */
int mcp25xxfd_can_setup(struct mcp25xxfd_priv *priv)
{
	struct spi_device *spi = priv->spi;
	struct mcp25xxfd_can_priv *cpriv;
	struct net_device *net;
	struct regulator *transceiver;
	int ret;

	/* get transceiver power regulator*/
	transceiver = devm_regulator_get(&spi->dev, "xceiver");
	if (PTR_ERR(transceiver) == -EPROBE_DEFER)
		return PTR_ERR(transceiver);

	/* allocate can device */
	net = alloc_candev(sizeof(*cpriv), TX_ECHO_SKB_MAX);
	if (!net)
		return -ENOMEM;

	cpriv = netdev_priv(net);
	cpriv->priv = priv;
	priv->cpriv = cpriv;

	/* setup network */
	SET_NETDEV_DEV(net, &spi->dev);
	net->netdev_ops = &mcp25xxfd_netdev_ops;
	net->flags |= IFF_ECHO;

	cpriv->transceiver = transceiver;

	cpriv->can.clock.freq = priv->clock_freq;
	cpriv->can.bittiming_const =
		&mcp25xxfd_can_nominal_bittiming_const;
	cpriv->can.data_bittiming_const =
		&mcp25xxfd_can_data_bittiming_const;

	/* we are not setting bit-timing methods here as they get called by
	 * the framework before open. So the controller would be still in sleep
	 * mode, which does not help as things are configured in open instead.
	 */
	cpriv->can.do_set_mode =
		mcp25xxfd_can_do_set_mode;
	cpriv->can.do_get_berr_counter =
		mcp25xxfd_can_get_berr_counter;
	cpriv->can.ctrlmode_supported =
		CAN_CTRLMODE_FD |
		CAN_CTRLMODE_FD_NON_ISO |
		CAN_CTRLMODE_LOOPBACK |
		CAN_CTRLMODE_LISTENONLY |
		CAN_CTRLMODE_BERR_REPORTING |
		CAN_CTRLMODE_ONE_SHOT;

	ret = register_candev(net);
	if (ret) {
		dev_err(&spi->dev, "Failed to register can device\n");
		goto out;
	}

	return 0;

out:
	free_candev(net);
	priv->cpriv = NULL;

	return ret;
}

void mcp25xxfd_can_remove(struct mcp25xxfd_priv *priv)
{
	if (priv->cpriv) {
		unregister_candev(priv->cpriv->can.dev);
		free_candev(priv->cpriv->can.dev);
		priv->cpriv = NULL;
	}
}
