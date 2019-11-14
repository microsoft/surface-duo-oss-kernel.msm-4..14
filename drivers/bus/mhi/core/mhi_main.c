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
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include "mhi_internal.h"

void mhi_db_brstmode(struct mhi_controller *mhi_cntrl,
		     struct db_cfg *db_cfg,
		     void __iomem *db_addr,
		     dma_addr_t db_val)
{
}

void mhi_db_brstmode_disable(struct mhi_controller *mhi_cntrl,
			     struct db_cfg *db_cfg,
			     void __iomem *db_addr,
			     dma_addr_t db_val)
{
}

int mhi_map_single_no_bb(struct mhi_controller *mhi_cntrl,
			 struct mhi_buf_info *buf_info)
{
	return -ENOMEM;
}

int mhi_map_single_use_bb(struct mhi_controller *mhi_cntrl,
			  struct mhi_buf_info *buf_info)
{
	return -ENOMEM;
}

void mhi_unmap_single_no_bb(struct mhi_controller *mhi_cntrl,
			    struct mhi_buf_info *buf_info)
{
}

void mhi_unmap_single_use_bb(struct mhi_controller *mhi_cntrl,
			     struct mhi_buf_info *buf_info)
{
}

int mhi_queue_sclist(struct mhi_device *mhi_dev, struct mhi_chan *mhi_chan,
		     void *buf, size_t len, enum mhi_flags mflags)
{
	return -EINVAL;
}

int mhi_queue_nop(struct mhi_device *mhi_dev, struct mhi_chan *mhi_chan,
		  void *buf, size_t len, enum mhi_flags mflags)
{
	return -EINVAL;
}

int mhi_queue_skb(struct mhi_device *mhi_dev, struct mhi_chan *mhi_chan,
		  void *buf, size_t len, enum mhi_flags mflags)
{
	return -EINVAL;
}

int mhi_gen_tre(struct mhi_controller *mhi_cntrl, struct mhi_chan *mhi_chan,
		void *buf, void *cb, size_t buf_len, enum mhi_flags flags)
{
	return -EINVAL;
}

int mhi_queue_transfer(struct mhi_device *mhi_dev,
		       enum dma_data_direction dir, void *buf, size_t len,
		       enum mhi_flags mflags)
{
	if (dir == DMA_TO_DEVICE)
		return mhi_dev->ul_chan->queue_xfer(mhi_dev, mhi_dev->ul_chan,
						    buf, len, mflags);
	else
		return mhi_dev->dl_chan->queue_xfer(mhi_dev, mhi_dev->dl_chan,
						    buf, len, mflags);
}
EXPORT_SYMBOL_GPL(mhi_queue_transfer);

int mhi_queue_buf(struct mhi_device *mhi_dev, struct mhi_chan *mhi_chan,
		  void *buf, size_t len, enum mhi_flags mflags)
{
	return -EINVAL;
}

int mhi_process_ctrl_ev_ring(struct mhi_controller *mhi_cntrl,
			     struct mhi_event *mhi_event,
			     u32 event_quota)
{
	return -EIO;
}

int mhi_process_data_event_ring(struct mhi_controller *mhi_cntrl,
				struct mhi_event *mhi_event,
				u32 event_quota)
{
	return -EIO;
}

void mhi_ev_task(unsigned long data)
{
}

void mhi_ctrl_ev_task(unsigned long data)
{
}
