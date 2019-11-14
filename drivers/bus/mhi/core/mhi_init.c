// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 *
 */

#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mhi.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include "mhi_internal.h"

static int parse_ev_cfg(struct mhi_controller *mhi_cntrl,
			struct mhi_controller_config *config)
{
	int i, num = 0;
	struct mhi_event *mhi_event;
	struct mhi_event_config *event_cfg;

	num = config->num_events;
	mhi_cntrl->total_ev_rings = num;
	mhi_cntrl->mhi_event = kcalloc(num, sizeof(*mhi_cntrl->mhi_event),
				       GFP_KERNEL);
	if (!mhi_cntrl->mhi_event)
		return -ENOMEM;

	/* populate ev ring */
	mhi_event = mhi_cntrl->mhi_event;
	for (i = 0; i < num; ++i) {
		event_cfg = &config->event_cfg[i];

		mhi_event->er_index = i++;
		mhi_event->ring.elements = event_cfg->num_elements;
		mhi_event->intmod = event_cfg->irq_moderation_ms;
		mhi_event->irq = event_cfg->irq;

		if (event_cfg->channel != U32_MAX) {
			/* This event ring has a dedicated channel */
			mhi_event->chan = event_cfg->channel;
			if (mhi_event->chan >= mhi_cntrl->max_chan) {
				dev_err(mhi_cntrl->dev,
					"Event Ring channel not available\n");
				goto error_ev_cfg;
			}

			mhi_event->mhi_chan =
				&mhi_cntrl->mhi_chan[mhi_event->chan];
		}

		/* Priority is fixed to 1 for now */
		mhi_event->priority = 1;

		mhi_event->db_cfg.brstmode = event_cfg->mode;
		if (MHI_INVALID_BRSTMODE(mhi_event->db_cfg.brstmode))
			goto error_ev_cfg;

		mhi_event->db_cfg.process_db =
			(mhi_event->db_cfg.brstmode == MHI_DB_BRST_ENABLE) ?
			mhi_db_brstmode : mhi_db_brstmode_disable;

		mhi_event->data_type = event_cfg->data_type;

		switch (mhi_event->data_type) {
		case MHI_ER_DATA:
			mhi_event->process_event = mhi_process_data_event_ring;
			break;
		case MHI_ER_CTRL:
			mhi_event->process_event = mhi_process_ctrl_ev_ring;
			break;
		default:
			dev_err(mhi_cntrl->dev,
				"Event Ring type not supported\n");
			goto error_ev_cfg;
		}

		mhi_event->hw_ring = event_cfg->hardware_event;
		if (mhi_event->hw_ring)
			mhi_cntrl->hw_ev_rings++;
		else
			mhi_cntrl->sw_ev_rings++;

		mhi_event->cl_manage = event_cfg->client_managed;
		mhi_event->offload_ev = event_cfg->offload_channel;
		mhi_event++;
	}

	/* we need irq for each event ring + additional one for BHI */
	mhi_cntrl->nr_irqs_req = mhi_cntrl->total_ev_rings + 1;

	return 0;

error_ev_cfg:

	kfree(mhi_cntrl->mhi_event);
	return -EINVAL;
}

static int parse_ch_cfg(struct mhi_controller *mhi_cntrl,
			struct mhi_controller_config *config)
{
	int i;
	u32 chan;
	struct mhi_channel_config *ch_cfg;

	mhi_cntrl->max_chan = config->max_channels;

	mhi_cntrl->mhi_chan = kcalloc(mhi_cntrl->max_chan,
				      sizeof(*mhi_cntrl->mhi_chan), GFP_KERNEL);
	if (!mhi_cntrl->mhi_chan)
		return -ENOMEM;

	INIT_LIST_HEAD(&mhi_cntrl->lpm_chans);

	/* populate channel configurations */
	for (i = 0; i < config->num_channels; ++i) {
		struct mhi_chan *mhi_chan;

		ch_cfg = &config->ch_cfg[i];

		chan = ch_cfg->num;
		if (chan >= mhi_cntrl->max_chan) {
			dev_err(mhi_cntrl->dev,
				"Channel %d not available\n", chan);
			goto error_chan_cfg;
		}

		mhi_chan = &mhi_cntrl->mhi_chan[chan];
		mhi_chan->name = ch_cfg->name;
		mhi_chan->chan = chan;

		mhi_chan->buf_ring.elements = ch_cfg->num_elements;
		if (!mhi_chan->buf_ring.elements)
			goto error_chan_cfg;

		mhi_chan->tre_ring.elements = mhi_chan->buf_ring.elements;
		mhi_chan->er_index = ch_cfg->event_ring;
		mhi_chan->dir = ch_cfg->dir;

		mhi_chan->ee = ch_cfg->ee;
		if (mhi_chan->ee >= MHI_EE_MAX_SUPPORTED)
			goto error_chan_cfg;

		mhi_chan->db_cfg.pollcfg = ch_cfg->pollcfg;
		mhi_chan->xfer_type = ch_cfg->data_type;

		switch (mhi_chan->xfer_type) {
		case MHI_BUF_RAW:
			mhi_chan->gen_tre = mhi_gen_tre;
			mhi_chan->queue_xfer = mhi_queue_buf;
			break;
		case MHI_BUF_SKB:
			mhi_chan->queue_xfer = mhi_queue_skb;
			break;
		case MHI_BUF_SCLIST:
			mhi_chan->gen_tre = mhi_gen_tre;
			mhi_chan->queue_xfer = mhi_queue_sclist;
			break;
		case MHI_BUF_NOP:
			mhi_chan->queue_xfer = mhi_queue_nop;
			break;
		default:
			dev_err(mhi_cntrl->dev,
				"Channel datatype not supported\n");
			goto error_chan_cfg;
		}

		mhi_chan->lpm_notify = ch_cfg->lpm_notify;
		mhi_chan->offload_ch = ch_cfg->offload_channel;
		mhi_chan->db_cfg.reset_req = ch_cfg->doorbell_mode_switch;
		mhi_chan->pre_alloc = ch_cfg->auto_queue;
		mhi_chan->auto_start = ch_cfg->auto_start;

		/*
		 * If MHI host allocates buffers, then the channel direction
		 * should be DMA_FROM_DEVICE and the buffer type should be
		 * MHI_BUF_RAW
		 */
		if (mhi_chan->pre_alloc && (mhi_chan->dir != DMA_FROM_DEVICE ||
				mhi_chan->xfer_type != MHI_BUF_RAW)) {
			dev_err(mhi_cntrl->dev,
				"Invalid channel configuration\n");
			goto error_chan_cfg;
		}

		/*
		 * Bi-directional and direction less channel must be an
		 * offload channel
		 */
		if ((mhi_chan->dir == DMA_BIDIRECTIONAL ||
		     mhi_chan->dir == DMA_NONE) && !mhi_chan->offload_ch) {
			dev_err(mhi_cntrl->dev,
				"Invalid channel configuration\n");
			goto error_chan_cfg;
		}

		/* If MHI host allocates buffers then client cannot queue */
		if (mhi_chan->pre_alloc)
			mhi_chan->queue_xfer = mhi_queue_nop;

		if (!mhi_chan->offload_ch) {
			mhi_chan->db_cfg.brstmode = ch_cfg->doorbell;
			if (MHI_INVALID_BRSTMODE(mhi_chan->db_cfg.brstmode)) {
				dev_err(mhi_cntrl->dev,
					"Invalid Door bell mode\n");
				goto error_chan_cfg;
			}

			mhi_chan->db_cfg.process_db =
				(mhi_chan->db_cfg.brstmode ==
				 MHI_DB_BRST_ENABLE) ?
				mhi_db_brstmode : mhi_db_brstmode_disable;
		}

		mhi_chan->configured = true;

		if (mhi_chan->lpm_notify)
			list_add_tail(&mhi_chan->node, &mhi_cntrl->lpm_chans);
	}

	return 0;

error_chan_cfg:
	kfree(mhi_cntrl->mhi_chan);

	return -EINVAL;
}

static int parse_config(struct mhi_controller *mhi_cntrl,
			struct mhi_controller_config *config)
{
	int ret;

	/* parse MHI channel configuration */
	ret = parse_ch_cfg(mhi_cntrl, config);
	if (ret)
		return ret;

	/* parse MHI event configuration */
	ret = parse_ev_cfg(mhi_cntrl, config);
	if (ret)
		goto error_ev_cfg;

	mhi_cntrl->timeout_ms = config->timeout_ms;
	if (!mhi_cntrl->timeout_ms)
		mhi_cntrl->timeout_ms = MHI_TIMEOUT_MS;

	mhi_cntrl->bounce_buf = config->use_bounce_buf;
	mhi_cntrl->buffer_len = config->buf_len;
	if (!mhi_cntrl->buffer_len)
		mhi_cntrl->buffer_len = MHI_MAX_MTU;

	return 0;

error_ev_cfg:
	kfree(mhi_cntrl->mhi_chan);

	return ret;
}

int mhi_register_controller(struct mhi_controller *mhi_cntrl,
			    struct mhi_controller_config *config)
{
	int ret;
	int i;
	struct mhi_event *mhi_event;
	struct mhi_chan *mhi_chan;
	struct mhi_cmd *mhi_cmd;
	struct mhi_device *mhi_dev;

	if (!mhi_cntrl->runtime_get || !mhi_cntrl->runtime_put)
		return -EINVAL;

	if (!mhi_cntrl->status_cb || !mhi_cntrl->link_status)
		return -EINVAL;

	ret = parse_config(mhi_cntrl, config);
	if (ret)
		return -EINVAL;

	mhi_cntrl->mhi_cmd = kcalloc(NR_OF_CMD_RINGS,
				     sizeof(*mhi_cntrl->mhi_cmd), GFP_KERNEL);
	if (!mhi_cntrl->mhi_cmd) {
		ret = -ENOMEM;
		goto error_alloc_cmd;
	}

	INIT_LIST_HEAD(&mhi_cntrl->transition_list);
	mutex_init(&mhi_cntrl->pm_mutex);
	rwlock_init(&mhi_cntrl->pm_lock);
	spin_lock_init(&mhi_cntrl->transition_lock);
	spin_lock_init(&mhi_cntrl->wlock);
	init_waitqueue_head(&mhi_cntrl->state_event);

	mhi_cmd = mhi_cntrl->mhi_cmd;
	for (i = 0; i < NR_OF_CMD_RINGS; i++, mhi_cmd++)
		spin_lock_init(&mhi_cmd->lock);

	mhi_event = mhi_cntrl->mhi_event;
	for (i = 0; i < mhi_cntrl->total_ev_rings; i++, mhi_event++) {
		/* Skip for offload events */
		if (mhi_event->offload_ev)
			continue;

		mhi_event->mhi_cntrl = mhi_cntrl;
		spin_lock_init(&mhi_event->lock);
		if (mhi_event->data_type == MHI_ER_CTRL)
			tasklet_init(&mhi_event->task, mhi_ctrl_ev_task,
				     (ulong)mhi_event);
		else
			tasklet_init(&mhi_event->task, mhi_ev_task,
				     (ulong)mhi_event);
	}

	mhi_chan = mhi_cntrl->mhi_chan;
	for (i = 0; i < mhi_cntrl->max_chan; i++, mhi_chan++) {
		mutex_init(&mhi_chan->mutex);
		init_completion(&mhi_chan->completion);
		rwlock_init(&mhi_chan->lock);
	}

	if (mhi_cntrl->bounce_buf) {
		mhi_cntrl->map_single = mhi_map_single_use_bb;
		mhi_cntrl->unmap_single = mhi_unmap_single_use_bb;
	} else {
		mhi_cntrl->map_single = mhi_map_single_no_bb;
		mhi_cntrl->unmap_single = mhi_unmap_single_no_bb;
	}

	/* Register controller with MHI bus */
	mhi_dev = mhi_alloc_device(mhi_cntrl);
	if (!mhi_dev) {
		ret = -ENOMEM;
		goto error_alloc_dev;
	}

	mhi_dev->dev_type = MHI_DEVICE_CONTROLLER;
	mhi_dev->mhi_cntrl = mhi_cntrl;
	dev_set_name(&mhi_dev->dev, "%s", mhi_cntrl->name);
	ret = device_add(&mhi_dev->dev);
	if (ret)
		goto error_add_dev;

	mhi_cntrl->mhi_dev = mhi_dev;

	return 0;

error_add_dev:
	mhi_dealloc_device(mhi_cntrl, mhi_dev);

error_alloc_dev:
	kfree(mhi_cntrl->mhi_cmd);

error_alloc_cmd:
	kfree(mhi_cntrl->mhi_chan);
	kfree(mhi_cntrl->mhi_event);

	return ret;
}
EXPORT_SYMBOL_GPL(mhi_register_controller);

void mhi_unregister_controller(struct mhi_controller *mhi_cntrl)
{
	struct mhi_device *mhi_dev = mhi_cntrl->mhi_dev;

	kfree(mhi_cntrl->mhi_cmd);
	kfree(mhi_cntrl->mhi_event);
	kfree(mhi_cntrl->mhi_chan);

	device_del(&mhi_dev->dev);
	put_device(&mhi_dev->dev);
}
EXPORT_SYMBOL_GPL(mhi_unregister_controller);

struct mhi_controller *mhi_alloc_controller(void)
{
	struct mhi_controller *mhi_cntrl;

	mhi_cntrl = kzalloc(sizeof(*mhi_cntrl), GFP_KERNEL);

	return mhi_cntrl;
}
EXPORT_SYMBOL_GPL(mhi_alloc_controller);

/* match dev to drv */
static int mhi_match(struct device *dev, struct device_driver *drv)
{
	struct mhi_device *mhi_dev = to_mhi_device(dev);
	struct mhi_driver *mhi_drv = to_mhi_driver(drv);
	const struct mhi_device_id *id;

	/* if controller type there is no client driver associated with it */
	if (mhi_dev->dev_type == MHI_DEVICE_CONTROLLER)
		return 0;

	for (id = mhi_drv->id_table; id->chan[0]; id++)
		if (!strcmp(mhi_dev->chan_name, id->chan)) {
			mhi_dev->id = id;
			return 1;
		}

	return 0;
};

static void mhi_release_device(struct device *dev)
{
	struct mhi_device *mhi_dev = to_mhi_device(dev);

	kfree(mhi_dev);
}

struct bus_type mhi_bus_type = {
	.name = "mhi",
	.dev_name = "mhi",
	.match = mhi_match,
};

static int mhi_driver_probe(struct device *dev)
{
	return -EINVAL;
}

static int mhi_driver_remove(struct device *dev)
{
	return -EINVAL;
}

int mhi_driver_register(struct mhi_driver *mhi_drv)
{
	struct device_driver *driver = &mhi_drv->driver;

	if (!mhi_drv->probe || !mhi_drv->remove)
		return -EINVAL;

	driver->bus = &mhi_bus_type;
	driver->probe = mhi_driver_probe;
	driver->remove = mhi_driver_remove;

	return driver_register(driver);
}
EXPORT_SYMBOL_GPL(mhi_driver_register);

void mhi_driver_unregister(struct mhi_driver *mhi_drv)
{
	driver_unregister(&mhi_drv->driver);
}
EXPORT_SYMBOL_GPL(mhi_driver_unregister);

struct mhi_device *mhi_alloc_device(struct mhi_controller *mhi_cntrl)
{
	struct mhi_device *mhi_dev = kzalloc(sizeof(*mhi_dev), GFP_KERNEL);
	struct device *dev;

	if (!mhi_dev)
		return NULL;

	dev = &mhi_dev->dev;
	device_initialize(dev);
	dev->bus = &mhi_bus_type;
	dev->release = mhi_release_device;
	dev->parent = mhi_cntrl->dev;
	mhi_dev->mhi_cntrl = mhi_cntrl;
	atomic_set(&mhi_dev->dev_wake, 0);

	return mhi_dev;
}

static int __init mhi_init(void)
{
	return bus_register(&mhi_bus_type);
}

static void __exit mhi_exit(void)
{
	bus_unregister(&mhi_bus_type);
}

postcore_initcall(mhi_init);
module_exit(mhi_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("MHI_CORE");
MODULE_DESCRIPTION("MHI Host Interface");
