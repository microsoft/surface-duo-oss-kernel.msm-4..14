/* Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __FSM_DP_MHI_H__
#define __FSM_DP_MHI_H__

#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/mhi.h>
#include <linux/skbuff.h>

#define FSM_DP_MHI_NAME	"fsm-l1rf-mhi"

struct fsm_dp_drv;

struct fsm_dp_mhi_stats {
	unsigned long tx_cnt;
	unsigned long tx_acked;
	unsigned long tx_err;
	unsigned long rx_cnt;
	unsigned long rx_err;
	unsigned long rx_out_of_buf;

	unsigned long rx_replenish;
	unsigned long rx_replenish_err;
	unsigned long rx_outofbuf_drop;
};

struct fsm_dp_mhi {
	struct mhi_device *mhi_dev;
	struct fsm_dp_mhi_stats stats;
	spinlock_t rx_lock;
	spinlock_t tx_lock;
	/*
	 * the following are for needed storage
	 * for mhi_queue_n_transfer.
	 */
	bool mhi_destroyed;
	void *ul_buf_array[FSM_DP_MAX_IOV_SIZE];
	size_t ul_size_array[FSM_DP_MAX_IOV_SIZE];
	enum MHI_FLAGS ul_flag_array[FSM_DP_MAX_IOV_SIZE];
	dma_addr_t ul_dma_addr_array[FSM_DP_MAX_IOV_SIZE];

	void *dl_buf_array[FSM_DP_MAX_IOV_SIZE];
	size_t dl_size_array[FSM_DP_MAX_IOV_SIZE];
	enum MHI_FLAGS dl_flag_array[FSM_DP_MAX_IOV_SIZE];
	dma_addr_t dl_dma_addr_array[FSM_DP_MAX_IOV_SIZE];
};

int fsm_dp_mhi_init(struct fsm_dp_drv *pdrv);
void fsm_dp_mhi_cleanup(struct fsm_dp_drv *pdrv);

int fsm_dp_mhi_rx_replenish(struct fsm_dp_drv *drv);

static inline int fsm_dp_mhi_skb_ul_xfer(
	struct fsm_dp_mhi *mhi, struct sk_buff *skb)
{
	return mhi_ul_skb_xfer(mhi->mhi_dev, skb);
}

static inline int fsm_dp_mhi_n_tx(struct fsm_dp_mhi *mhi,
				unsigned int num)
{
	int ret;

	if (mhi->mhi_destroyed)
		return -ENODEV;

	ret = mhi_queue_n_transfer(mhi->mhi_dev,
				DMA_TO_DEVICE,
				mhi->dl_buf_array,
				mhi->dl_size_array,
				mhi->dl_flag_array,
				mhi->dl_dma_addr_array,
				num);
	if (!ret)
		mhi->stats.tx_cnt += num;
	else
		mhi->stats.tx_err += num;
	return ret;
}

static inline bool fsm_dp_mhi_is_ready(struct fsm_dp_mhi *mhi)
{
	return (((mhi->mhi_dev) && !(mhi->mhi_destroyed)) ? true : false);
}

#endif /* __FSM_DP_MHI_H__ */
