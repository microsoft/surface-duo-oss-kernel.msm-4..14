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

int __must_check mhi_read_reg(struct mhi_controller *mhi_cntrl,
			      void __iomem *base, u32 offset, u32 *out)
{
	u32 tmp = le32_to_cpu(readl_relaxed(base + offset));

	/* If there is any unexpected value, query the link status */
	if (PCI_INVALID_READ(tmp) &&
	    mhi_cntrl->link_status(mhi_cntrl, mhi_cntrl->priv_data))
		return -EIO;

	*out = tmp;

	return 0;
}

int __must_check mhi_read_reg_field(struct mhi_controller *mhi_cntrl,
				    void __iomem *base, u32 offset,
				    u32 mask, u32 shift, u32 *out)
{
	u32 tmp;
	int ret;

	ret = mhi_read_reg(mhi_cntrl, base, offset, &tmp);
	if (ret)
		return ret;

	*out = (tmp & mask) >> shift;

	return 0;
}

void mhi_write_reg(struct mhi_controller *mhi_cntrl, void __iomem *base,
		   u32 offset, u32 val)
{
	writel_relaxed(val, base + offset);
}

void mhi_write_reg_field(struct mhi_controller *mhi_cntrl, void __iomem *base,
			 u32 offset, u32 mask, u32 shift, u32 val)
{
	int ret;
	u32 tmp;

	ret = mhi_read_reg(mhi_cntrl, base, offset, &tmp);
	if (ret)
		return;

	tmp &= ~mask;
	tmp |= (val << shift);
	mhi_write_reg(mhi_cntrl, base, offset, tmp);
}

void mhi_write_db(struct mhi_controller *mhi_cntrl, void __iomem *db_addr,
		  dma_addr_t db_val)
{
	mhi_write_reg(mhi_cntrl, db_addr, 4, upper_32_bits(db_val));
	mhi_write_reg(mhi_cntrl, db_addr, 0, lower_32_bits(db_val));
}

void mhi_db_brstmode(struct mhi_controller *mhi_cntrl,
		     struct db_cfg *db_cfg,
		     void __iomem *db_addr,
		     dma_addr_t db_val)
{
	if (db_cfg->db_mode) {
		db_cfg->db_val = db_val;
		mhi_write_db(mhi_cntrl, db_addr, db_val);
		db_cfg->db_mode = 0;
	}
}

void mhi_db_brstmode_disable(struct mhi_controller *mhi_cntrl,
			     struct db_cfg *db_cfg,
			     void __iomem *db_addr,
			     dma_addr_t db_val)
{
	db_cfg->db_val = db_val;
	mhi_write_db(mhi_cntrl, db_addr, db_val);
}

void mhi_ring_er_db(struct mhi_event *mhi_event)
{
	struct mhi_ring *ring = &mhi_event->ring;

	mhi_event->db_cfg.process_db(mhi_event->mhi_cntrl, &mhi_event->db_cfg,
				     ring->db_addr, *ring->ctxt_wp);
}

void mhi_ring_cmd_db(struct mhi_controller *mhi_cntrl, struct mhi_cmd *mhi_cmd)
{
	dma_addr_t db;
	struct mhi_ring *ring = &mhi_cmd->ring;

	db = ring->iommu_base + (ring->wp - ring->base);
	*ring->ctxt_wp = db;
	mhi_write_db(mhi_cntrl, ring->db_addr, db);
}

void mhi_ring_chan_db(struct mhi_controller *mhi_cntrl,
		      struct mhi_chan *mhi_chan)
{
	struct mhi_ring *ring = &mhi_chan->tre_ring;
	dma_addr_t db;

	db = ring->iommu_base + (ring->wp - ring->base);
	*ring->ctxt_wp = db;
	mhi_chan->db_cfg.process_db(mhi_cntrl, &mhi_chan->db_cfg,
				    ring->db_addr, db);
}

enum mhi_ee_type mhi_get_exec_env(struct mhi_controller *mhi_cntrl)
{
	u32 exec;
	int ret = mhi_read_reg(mhi_cntrl, mhi_cntrl->bhi, BHI_EXECENV, &exec);

	return (ret) ? MHI_EE_MAX : exec;
}

enum mhi_state mhi_get_m_state(struct mhi_controller *mhi_cntrl)
{
	u32 state;
	int ret = mhi_read_reg_field(mhi_cntrl, mhi_cntrl->regs, MHISTATUS,
				     MHISTATUS_MHISTATE_MASK,
				     MHISTATUS_MHISTATE_SHIFT, &state);
	return ret ? MHI_STATE_MAX : state;
}

int mhi_map_single_no_bb(struct mhi_controller *mhi_cntrl,
			 struct mhi_buf_info *buf_info)
{
	buf_info->p_addr = dma_map_single(mhi_cntrl->dev, buf_info->v_addr,
					  buf_info->len, buf_info->dir);
	if (dma_mapping_error(mhi_cntrl->dev, buf_info->p_addr))
		return -ENOMEM;

	return 0;
}

int mhi_map_single_use_bb(struct mhi_controller *mhi_cntrl,
			  struct mhi_buf_info *buf_info)
{
	void *buf = mhi_alloc_coherent(mhi_cntrl, buf_info->len,
				       &buf_info->p_addr, GFP_ATOMIC);

	if (!buf)
		return -ENOMEM;

	if (buf_info->dir == DMA_TO_DEVICE)
		memcpy(buf, buf_info->v_addr, buf_info->len);

	buf_info->bb_addr = buf;

	return 0;
}

void mhi_unmap_single_no_bb(struct mhi_controller *mhi_cntrl,
			    struct mhi_buf_info *buf_info)
{
	dma_unmap_single(mhi_cntrl->dev, buf_info->p_addr, buf_info->len,
			 buf_info->dir);
}

void mhi_unmap_single_use_bb(struct mhi_controller *mhi_cntrl,
			     struct mhi_buf_info *buf_info)
{
	if (buf_info->dir == DMA_FROM_DEVICE)
		memcpy(buf_info->v_addr, buf_info->bb_addr, buf_info->len);

	mhi_free_coherent(mhi_cntrl, buf_info->len, buf_info->bb_addr,
			  buf_info->p_addr);
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

static void mhi_add_ring_element(struct mhi_controller *mhi_cntrl,
				 struct mhi_ring *ring)
{
	ring->wp += ring->el_size;
	if (ring->wp >= (ring->base + ring->len))
		ring->wp = ring->base;
	/* smp update */
	smp_wmb();
}

static void mhi_del_ring_element(struct mhi_controller *mhi_cntrl,
				 struct mhi_ring *ring)
{
	ring->rp += ring->el_size;
	if (ring->rp >= (ring->base + ring->len))
		ring->rp = ring->base;
	/* smp update */
	smp_wmb();
}

static int get_nr_avail_ring_elements(struct mhi_controller *mhi_cntrl,
				      struct mhi_ring *ring)
{
	int nr_el;

	if (ring->wp < ring->rp) {
		nr_el = ((ring->rp - ring->wp) / ring->el_size) - 1;
	} else {
		nr_el = (ring->rp - ring->base) / ring->el_size;
		nr_el += ((ring->base + ring->len - ring->wp) /
			  ring->el_size) - 1;
	}

	return nr_el;
}

static void *mhi_to_virtual(struct mhi_ring *ring, dma_addr_t addr)
{
	return (addr - ring->iommu_base) + ring->base;
}

static void mhi_recycle_ev_ring_element(struct mhi_controller *mhi_cntrl,
					struct mhi_ring *ring)
{
	dma_addr_t ctxt_wp;

	/* Update the WP */
	ring->wp += ring->el_size;
	ctxt_wp = *ring->ctxt_wp + ring->el_size;

	if (ring->wp >= (ring->base + ring->len)) {
		ring->wp = ring->base;
		ctxt_wp = ring->iommu_base;
	}

	*ring->ctxt_wp = ctxt_wp;

	/* Update the RP */
	ring->rp += ring->el_size;
	if (ring->rp >= (ring->base + ring->len))
		ring->rp = ring->base;

	/* Update to all cores */
	smp_wmb();
}

static bool mhi_is_ring_full(struct mhi_controller *mhi_cntrl,
			     struct mhi_ring *ring)
{
	void *tmp = ring->wp + ring->el_size;

	if (tmp >= (ring->base + ring->len))
		tmp = ring->base;

	return (tmp == ring->rp);
}

int mhi_queue_skb(struct mhi_device *mhi_dev, struct mhi_chan *mhi_chan,
		  void *buf, size_t len, enum mhi_flags mflags)
{
	struct sk_buff *skb = buf;
	struct mhi_controller *mhi_cntrl = mhi_dev->mhi_cntrl;
	struct mhi_ring *tre_ring = &mhi_chan->tre_ring;
	struct mhi_ring *buf_ring = &mhi_chan->buf_ring;
	struct mhi_buf_info *buf_info;
	struct mhi_tre *mhi_tre;
	bool assert_wake = false;
	int ret;

	if (mhi_is_ring_full(mhi_cntrl, tre_ring))
		return -ENOMEM;

	read_lock_bh(&mhi_cntrl->pm_lock);
	if (unlikely(MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state))) {
		read_unlock_bh(&mhi_cntrl->pm_lock);
		return -EIO;
	}

	/* we're in M3 or transitioning to M3 */
	if (MHI_PM_IN_SUSPEND_STATE(mhi_cntrl->pm_state)) {
		mhi_cntrl->runtime_get(mhi_cntrl, mhi_cntrl->priv_data);
		mhi_cntrl->runtime_put(mhi_cntrl, mhi_cntrl->priv_data);
	}

	/*
	 * For UL channels always assert WAKE until work is done,
	 * For DL channels only assert if MHI is in a LPM
	 */
	if (mhi_chan->dir == DMA_TO_DEVICE ||
	    (mhi_chan->dir == DMA_FROM_DEVICE &&
	     mhi_cntrl->pm_state != MHI_PM_M0)) {
		assert_wake = true;
		mhi_cntrl->wake_get(mhi_cntrl, false);
	}

	/* generate the tre */
	buf_info = buf_ring->wp;
	buf_info->v_addr = skb->data;
	buf_info->cb_buf = skb;
	buf_info->wp = tre_ring->wp;
	buf_info->dir = mhi_chan->dir;
	buf_info->len = len;
	ret = mhi_cntrl->map_single(mhi_cntrl, buf_info);
	if (ret)
		goto map_error;

	mhi_tre = tre_ring->wp;
	mhi_tre->ptr = MHI_TRE_DATA_PTR(buf_info->p_addr);
	mhi_tre->dword[0] = MHI_TRE_DATA_DWORD0(buf_info->len);
	mhi_tre->dword[1] = MHI_TRE_DATA_DWORD1(1, 1, 0, 0);

	/* increment WP */
	mhi_add_ring_element(mhi_cntrl, tre_ring);
	mhi_add_ring_element(mhi_cntrl, buf_ring);

	if (likely(MHI_DB_ACCESS_VALID(mhi_cntrl->pm_state))) {
		read_lock_bh(&mhi_chan->lock);
		mhi_ring_chan_db(mhi_cntrl, mhi_chan);
		read_unlock_bh(&mhi_chan->lock);
	}

	if (mhi_chan->dir == DMA_FROM_DEVICE && assert_wake)
		mhi_cntrl->wake_put(mhi_cntrl, true);

	read_unlock_bh(&mhi_cntrl->pm_lock);

	return 0;

map_error:
	if (assert_wake)
		mhi_cntrl->wake_put(mhi_cntrl, false);
	read_unlock_bh(&mhi_cntrl->pm_lock);

	return ret;
}

int mhi_gen_tre(struct mhi_controller *mhi_cntrl, struct mhi_chan *mhi_chan,
		void *buf, void *cb, size_t buf_len, enum mhi_flags flags)
{
	struct mhi_ring *buf_ring, *tre_ring;
	struct mhi_tre *mhi_tre;
	struct mhi_buf_info *buf_info;
	int eot, eob, chain, bei;
	int ret;

	buf_ring = &mhi_chan->buf_ring;
	tre_ring = &mhi_chan->tre_ring;

	buf_info = buf_ring->wp;
	buf_info->v_addr = buf;
	buf_info->cb_buf = cb;
	buf_info->wp = tre_ring->wp;
	buf_info->dir = mhi_chan->dir;
	buf_info->len = buf_len;

	ret = mhi_cntrl->map_single(mhi_cntrl, buf_info);
	if (ret)
		return ret;

	eob = !!(flags & MHI_EOB);
	eot = !!(flags & MHI_EOT);
	chain = !!(flags & MHI_CHAIN);
	bei = !!(mhi_chan->intmod);

	mhi_tre = tre_ring->wp;
	mhi_tre->ptr = MHI_TRE_DATA_PTR(buf_info->p_addr);
	mhi_tre->dword[0] = MHI_TRE_DATA_DWORD0(buf_len);
	mhi_tre->dword[1] = MHI_TRE_DATA_DWORD1(bei, eot, eob, chain);

	/* increment WP */
	mhi_add_ring_element(mhi_cntrl, tre_ring);
	mhi_add_ring_element(mhi_cntrl, buf_ring);

	return 0;
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
	struct mhi_controller *mhi_cntrl = mhi_dev->mhi_cntrl;
	struct mhi_ring *tre_ring;
	unsigned long flags;
	bool assert_wake = false;
	int ret;

	/*
	 * this check here only as a guard, it's always
	 * possible mhi can enter error while executing rest of function,
	 * which is not fatal so we do not need to hold pm_lock
	 */
	if (unlikely(MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state)))
		return -EIO;

	/* we're in M3 or transitioning to M3 */
	if (MHI_PM_IN_SUSPEND_STATE(mhi_cntrl->pm_state)) {
		mhi_cntrl->runtime_get(mhi_cntrl, mhi_cntrl->priv_data);
		mhi_cntrl->runtime_put(mhi_cntrl, mhi_cntrl->priv_data);
	}

	tre_ring = &mhi_chan->tre_ring;
	if (mhi_is_ring_full(mhi_cntrl, tre_ring))
		return -ENOMEM;

	ret = mhi_chan->gen_tre(mhi_cntrl, mhi_chan, buf, buf, len, mflags);
	if (unlikely(ret))
		return ret;

	read_lock_irqsave(&mhi_cntrl->pm_lock, flags);

	/*
	 * For UL channels always assert WAKE until work is done,
	 * For DL channels only assert if MHI is in a LPM
	 */
	if (mhi_chan->dir == DMA_TO_DEVICE ||
	    (mhi_chan->dir == DMA_FROM_DEVICE &&
	     mhi_cntrl->pm_state != MHI_PM_M0)) {
		assert_wake = true;
		mhi_cntrl->wake_get(mhi_cntrl, false);
	}

	if (likely(MHI_DB_ACCESS_VALID(mhi_cntrl->pm_state))) {
		unsigned long flags;

		read_lock_irqsave(&mhi_chan->lock, flags);
		mhi_ring_chan_db(mhi_cntrl, mhi_chan);
		read_unlock_irqrestore(&mhi_chan->lock, flags);
	}

	if (mhi_chan->dir == DMA_FROM_DEVICE && assert_wake)
		mhi_cntrl->wake_put(mhi_cntrl, true);

	read_unlock_irqrestore(&mhi_cntrl->pm_lock, flags);

	return 0;
}

int mhi_destroy_device(struct device *dev, void *data)
{
	struct mhi_device *mhi_dev;
	struct mhi_controller *mhi_cntrl;

	if (dev->bus != &mhi_bus_type)
		return 0;

	mhi_dev = to_mhi_device(dev);
	mhi_cntrl = mhi_dev->mhi_cntrl;

	/* Only destroy virtual devices thats attached to bus */
	if (mhi_dev->dev_type ==  MHI_DEVICE_CONTROLLER)
		return 0;

	dev_info(mhi_cntrl->dev, "destroy device for chan:%s\n",
		 mhi_dev->chan_name);

	/* Notify the client and remove the device from mhi bus */
	device_del(dev);
	put_device(dev);

	return 0;
}

void mhi_notify(struct mhi_device *mhi_dev, enum mhi_callback cb_reason)
{
	struct mhi_driver *mhi_drv;

	if (!mhi_dev->dev.driver)
		return;

	mhi_drv = to_mhi_driver(mhi_dev->dev.driver);

	if (mhi_drv->status_cb)
		mhi_drv->status_cb(mhi_dev, cb_reason);
}

/* Bind mhi channels to mhi devices */
void mhi_create_devices(struct mhi_controller *mhi_cntrl)
{
	int i;
	struct mhi_chan *mhi_chan;
	struct mhi_device *mhi_dev;
	int ret;

	mhi_chan = mhi_cntrl->mhi_chan;
	for (i = 0; i < mhi_cntrl->max_chan; i++, mhi_chan++) {
		if (!mhi_chan->configured || mhi_chan->ee != mhi_cntrl->ee)
			continue;
		mhi_dev = mhi_alloc_device(mhi_cntrl);
		if (!mhi_dev)
			return;

		mhi_dev->dev_type = MHI_DEVICE_XFER;
		switch (mhi_chan->dir) {
		case DMA_TO_DEVICE:
			mhi_dev->ul_chan = mhi_chan;
			mhi_dev->ul_chan_id = mhi_chan->chan;
			break;
		case DMA_FROM_DEVICE:
			/* We use dl_chan as offload channels */
			mhi_dev->dl_chan = mhi_chan;
			mhi_dev->dl_chan_id = mhi_chan->chan;
			break;
		default:
			dev_err(mhi_cntrl->dev, "Direction not supported\n");
			mhi_dealloc_device(mhi_cntrl, mhi_dev);
			return;
		}

		mhi_chan->mhi_dev = mhi_dev;

		/* Check next channel if it matches */
		if ((i + 1) < mhi_cntrl->max_chan && mhi_chan[1].configured) {
			if (!strcmp(mhi_chan[1].name, mhi_chan->name)) {
				i++;
				mhi_chan++;
				if (mhi_chan->dir == DMA_TO_DEVICE) {
					mhi_dev->ul_chan = mhi_chan;
					mhi_dev->ul_chan_id = mhi_chan->chan;
				} else {
					mhi_dev->dl_chan = mhi_chan;
					mhi_dev->dl_chan_id = mhi_chan->chan;
				}
				mhi_chan->mhi_dev = mhi_dev;
			}
		}

		mhi_dev->chan_name = mhi_chan->name;
		dev_set_name(&mhi_dev->dev, "%04x_%s", mhi_chan->chan,
			     mhi_dev->chan_name);

		ret = device_add(&mhi_dev->dev);
		if (ret)
			mhi_dealloc_device(mhi_cntrl, mhi_dev);
	}
}

static int parse_xfer_event(struct mhi_controller *mhi_cntrl,
			    struct mhi_tre *event,
			    struct mhi_chan *mhi_chan)
{
	struct mhi_ring *buf_ring, *tre_ring;
	u32 ev_code;
	struct mhi_result result;
	unsigned long flags = 0;

	ev_code = MHI_TRE_GET_EV_CODE(event);
	buf_ring = &mhi_chan->buf_ring;
	tre_ring = &mhi_chan->tre_ring;

	result.transaction_status = (ev_code == MHI_EV_CC_OVERFLOW) ?
		-EOVERFLOW : 0;

	/*
	 * if it's a DB Event then we need to grab the lock
	 * with preemption disable and as a write because we
	 * have to update db register and another thread could
	 * be doing same.
	 */
	if (ev_code >= MHI_EV_CC_OOB)
		write_lock_irqsave(&mhi_chan->lock, flags);
	else
		read_lock_bh(&mhi_chan->lock);

	if (mhi_chan->ch_state != MHI_CH_STATE_ENABLED)
		goto end_process_tx_event;

	switch (ev_code) {
	case MHI_EV_CC_OVERFLOW:
	case MHI_EV_CC_EOB:
	case MHI_EV_CC_EOT:
	{
		dma_addr_t ptr = MHI_TRE_GET_EV_PTR(event);
		struct mhi_tre *local_rp, *ev_tre;
		void *dev_rp;
		struct mhi_buf_info *buf_info;
		u16 xfer_len;

		/* Get the TRB this event points to */
		ev_tre = mhi_to_virtual(tre_ring, ptr);

		/* device rp after servicing the TREs */
		dev_rp = ev_tre + 1;
		if (dev_rp >= (tre_ring->base + tre_ring->len))
			dev_rp = tre_ring->base;

		result.dir = mhi_chan->dir;

		/* local rp */
		local_rp = tre_ring->rp;
		while (local_rp != dev_rp) {
			buf_info = buf_ring->rp;
			/* if it's last tre get len from the event */
			if (local_rp == ev_tre)
				xfer_len = MHI_TRE_GET_EV_LEN(event);
			else
				xfer_len = buf_info->len;

			mhi_cntrl->unmap_single(mhi_cntrl, buf_info);

			result.buf_addr = buf_info->cb_buf;
			result.bytes_xferd = xfer_len;
			mhi_del_ring_element(mhi_cntrl, buf_ring);
			mhi_del_ring_element(mhi_cntrl, tre_ring);
			local_rp = tre_ring->rp;

			/* notify client */
			mhi_chan->xfer_cb(mhi_chan->mhi_dev, &result);

			if (mhi_chan->dir == DMA_TO_DEVICE) {
				read_lock_bh(&mhi_cntrl->pm_lock);
				mhi_cntrl->wake_put(mhi_cntrl, false);
				read_unlock_bh(&mhi_cntrl->pm_lock);
			}

			/*
			 * recycle the buffer if buffer is pre-allocated,
			 * if there is error, not much we can do apart from
			 * dropping the packet
			 */
			if (mhi_chan->pre_alloc) {
				if (mhi_queue_buf(mhi_chan->mhi_dev, mhi_chan,
						  buf_info->cb_buf,
						  buf_info->len, MHI_EOT)) {
					dev_err(mhi_cntrl->dev,
						"Error recycling buffer for chan:%d\n",
						mhi_chan->chan);
					kfree(buf_info->cb_buf);
				}
			}
		}
		break;
	} /* CC_EOT */
	case MHI_EV_CC_OOB:
	case MHI_EV_CC_DB_MODE:
	{
		unsigned long flags;

		mhi_chan->db_cfg.db_mode = 1;
		read_lock_irqsave(&mhi_cntrl->pm_lock, flags);
		if (tre_ring->wp != tre_ring->rp &&
		    MHI_DB_ACCESS_VALID(mhi_cntrl->pm_state)) {
			mhi_ring_chan_db(mhi_cntrl, mhi_chan);
		}
		read_unlock_irqrestore(&mhi_cntrl->pm_lock, flags);
		break;
	}
	case MHI_EV_CC_BAD_TRE:
	default:
		dev_err(mhi_cntrl->dev, "Unknown event 0x%x\n", ev_code);
		break;
	} /* switch(MHI_EV_READ_CODE(EV_TRB_CODE,event)) */

end_process_tx_event:
	if (ev_code >= MHI_EV_CC_OOB)
		write_unlock_irqrestore(&mhi_chan->lock, flags);
	else
		read_unlock_bh(&mhi_chan->lock);

	return 0;
}

static void mhi_process_cmd_completion(struct mhi_controller *mhi_cntrl,
				       struct mhi_tre *tre)
{
	dma_addr_t ptr = MHI_TRE_GET_EV_PTR(tre);
	struct mhi_cmd *cmd_ring = &mhi_cntrl->mhi_cmd[PRIMARY_CMD_RING];
	struct mhi_ring *mhi_ring = &cmd_ring->ring;
	struct mhi_tre *cmd_pkt;
	struct mhi_chan *mhi_chan;
	u32 chan;

	cmd_pkt = mhi_to_virtual(mhi_ring, ptr);

	chan = MHI_TRE_GET_CMD_CHID(cmd_pkt);
	mhi_chan = &mhi_cntrl->mhi_chan[chan];
	write_lock_bh(&mhi_chan->lock);
	mhi_chan->ccs = MHI_TRE_GET_EV_CODE(tre);
	complete(&mhi_chan->completion);
	write_unlock_bh(&mhi_chan->lock);

	mhi_del_ring_element(mhi_cntrl, mhi_ring);
}

int mhi_process_ctrl_ev_ring(struct mhi_controller *mhi_cntrl,
			     struct mhi_event *mhi_event,
			     u32 event_quota)
{
	struct mhi_tre *dev_rp, *local_rp;
	struct mhi_ring *ev_ring = &mhi_event->ring;
	struct mhi_event_ctxt *er_ctxt =
		&mhi_cntrl->mhi_ctxt->er_ctxt[mhi_event->er_index];
	int count = 0;
	u32 chan;
	struct mhi_chan *mhi_chan;

	/*
	 * This is a quick check to avoid unnecessary event processing
	 * in case MHI is already in error state, but it's still possible
	 * to transition to error state while processing events
	 */
	if (unlikely(MHI_EVENT_ACCESS_INVALID(mhi_cntrl->pm_state)))
		return -EIO;

	dev_rp = mhi_to_virtual(ev_ring, er_ctxt->rp);
	local_rp = ev_ring->rp;

	while (dev_rp != local_rp) {
		enum mhi_pkt_type type = MHI_TRE_GET_EV_TYPE(local_rp);

		switch (type) {
		case MHI_PKT_TYPE_STATE_CHANGE_EVENT:
		{
			enum mhi_state new_state;

			new_state = MHI_TRE_GET_EV_STATE(local_rp);

			dev_info(mhi_cntrl->dev,
				 "MHI state change event to state: %s\n",
				 TO_MHI_STATE_STR(new_state));

			switch (new_state) {
			case MHI_STATE_M0:
				mhi_pm_m0_transition(mhi_cntrl);
				break;
			case MHI_STATE_M1:
				mhi_pm_m1_transition(mhi_cntrl);
				break;
			case MHI_STATE_M3:
				mhi_pm_m3_transition(mhi_cntrl);
				break;
			case MHI_STATE_SYS_ERR:
			{
				enum mhi_pm_state new_state;

				dev_info(mhi_cntrl->dev,
					 "MHI system error detected\n");
				write_lock_irq(&mhi_cntrl->pm_lock);
				new_state = mhi_tryset_pm_state(mhi_cntrl,
							MHI_PM_SYS_ERR_DETECT);
				write_unlock_irq(&mhi_cntrl->pm_lock);
				if (new_state == MHI_PM_SYS_ERR_DETECT)
					schedule_work(&mhi_cntrl->syserr_worker);
				break;
			}
			default:
				dev_err(mhi_cntrl->dev, "Invalid state: %s\n",
					TO_MHI_STATE_STR(new_state));
			}

			break;
		}
		case MHI_PKT_TYPE_CMD_COMPLETION_EVENT:
			mhi_process_cmd_completion(mhi_cntrl, local_rp);
			break;
		case MHI_PKT_TYPE_EE_EVENT:
		{
			enum dev_st_transition st = DEV_ST_TRANSITION_MAX;
			enum mhi_ee_type event = MHI_TRE_GET_EV_EXECENV(local_rp);

			dev_info(mhi_cntrl->dev, "MHI EE received event:%s\n",
				 TO_MHI_EXEC_STR(event));
			switch (event) {
			case MHI_EE_SBL:
				st = DEV_ST_TRANSITION_SBL;
				break;
			case MHI_EE_AMSS:
				st = DEV_ST_TRANSITION_AMSS;
				break;
			case MHI_EE_RDDM:
				mhi_cntrl->status_cb(mhi_cntrl,
						     mhi_cntrl->priv_data,
						     MHI_CB_EE_RDDM);
				/* fallthrough */
			case MHI_EE_BHIE:
				write_lock_irq(&mhi_cntrl->pm_lock);
				mhi_cntrl->ee = event;
				write_unlock_irq(&mhi_cntrl->pm_lock);
				wake_up(&mhi_cntrl->state_event);
				break;
			default:
				dev_err(mhi_cntrl->dev, "Unhandled EE event\n");
			}
			if (st != DEV_ST_TRANSITION_MAX)
				mhi_queue_state_transition(mhi_cntrl, st);

			break;
		}
		case MHI_PKT_TYPE_TX_EVENT:
			chan = MHI_TRE_GET_EV_CHID(local_rp);
			mhi_chan = &mhi_cntrl->mhi_chan[chan];
			parse_xfer_event(mhi_cntrl, local_rp, mhi_chan);
			event_quota--;
			break;
		default:
			dev_err(mhi_cntrl->dev, "Unhandled event type %d\n",
				type);
			break;
		}

		mhi_recycle_ev_ring_element(mhi_cntrl, ev_ring);
		local_rp = ev_ring->rp;
		dev_rp = mhi_to_virtual(ev_ring, er_ctxt->rp);
		count++;
	}

	read_lock_bh(&mhi_cntrl->pm_lock);
	if (likely(MHI_DB_ACCESS_VALID(mhi_cntrl->pm_state)))
		mhi_ring_er_db(mhi_event);
	read_unlock_bh(&mhi_cntrl->pm_lock);

	return count;
}

int mhi_process_data_event_ring(struct mhi_controller *mhi_cntrl,
				struct mhi_event *mhi_event,
				u32 event_quota)
{
	struct mhi_tre *dev_rp, *local_rp;
	struct mhi_ring *ev_ring = &mhi_event->ring;
	struct mhi_event_ctxt *er_ctxt =
		&mhi_cntrl->mhi_ctxt->er_ctxt[mhi_event->er_index];
	int count = 0;
	u32 chan;
	struct mhi_chan *mhi_chan;

	if (unlikely(MHI_EVENT_ACCESS_INVALID(mhi_cntrl->pm_state)))
		return -EIO;

	dev_rp = mhi_to_virtual(ev_ring, er_ctxt->rp);
	local_rp = ev_ring->rp;

	while (dev_rp != local_rp && event_quota > 0) {
		enum mhi_pkt_type type = MHI_TRE_GET_EV_TYPE(local_rp);

		if (likely(type == MHI_PKT_TYPE_TX_EVENT)) {
			chan = MHI_TRE_GET_EV_CHID(local_rp);
			mhi_chan = &mhi_cntrl->mhi_chan[chan];
			parse_xfer_event(mhi_cntrl, local_rp, mhi_chan);
			event_quota--;
		}

		mhi_recycle_ev_ring_element(mhi_cntrl, ev_ring);
		local_rp = ev_ring->rp;
		dev_rp = mhi_to_virtual(ev_ring, er_ctxt->rp);
		count++;
	}
	read_lock_bh(&mhi_cntrl->pm_lock);
	if (likely(MHI_DB_ACCESS_VALID(mhi_cntrl->pm_state)))
		mhi_ring_er_db(mhi_event);
	read_unlock_bh(&mhi_cntrl->pm_lock);

	return count;
}

void mhi_ev_task(unsigned long data)
{
	struct mhi_event *mhi_event = (struct mhi_event *)data;
	struct mhi_controller *mhi_cntrl = mhi_event->mhi_cntrl;

	/* process all pending events */
	spin_lock_bh(&mhi_event->lock);
	mhi_event->process_event(mhi_cntrl, mhi_event, U32_MAX);
	spin_unlock_bh(&mhi_event->lock);
}

void mhi_ctrl_ev_task(unsigned long data)
{
	struct mhi_event *mhi_event = (struct mhi_event *)data;
	struct mhi_controller *mhi_cntrl = mhi_event->mhi_cntrl;
	enum mhi_state state = MHI_STATE_MAX;
	enum mhi_pm_state pm_state = 0;
	int ret;

	/* Process ctrl events events */
	ret = mhi_event->process_event(mhi_cntrl, mhi_event, U32_MAX);

	/*
	 * We received an IRQ but no events to process, maybe device went to
	 * SYS_ERR state? Check the state to confirm.
	 */
	if (!ret) {
		write_lock_irq(&mhi_cntrl->pm_lock);
		if (MHI_REG_ACCESS_VALID(mhi_cntrl->pm_state))
			state = mhi_get_m_state(mhi_cntrl);
		if (state == MHI_STATE_SYS_ERR) {
			dev_info(mhi_cntrl->dev, "MHI system error detected\n");
			pm_state = mhi_tryset_pm_state(mhi_cntrl,
						       MHI_PM_SYS_ERR_DETECT);
		}
		write_unlock_irq(&mhi_cntrl->pm_lock);
		if (pm_state == MHI_PM_SYS_ERR_DETECT)
			schedule_work(&mhi_cntrl->syserr_worker);
	}
}

irqreturn_t mhi_irq_handler(int irq_number, void *dev)
{
	struct mhi_event *mhi_event = dev;
	struct mhi_controller *mhi_cntrl = mhi_event->mhi_cntrl;
	struct mhi_event_ctxt *er_ctxt =
		&mhi_cntrl->mhi_ctxt->er_ctxt[mhi_event->er_index];
	struct mhi_ring *ev_ring = &mhi_event->ring;
	void *dev_rp = mhi_to_virtual(ev_ring, er_ctxt->rp);

	/* Only proceed if event ring has pending events */
	if (ev_ring->rp == dev_rp)
		return IRQ_HANDLED;

	/* For client managed event ring, notify pending data */
	if (mhi_event->cl_manage) {
		struct mhi_chan *mhi_chan = mhi_event->mhi_chan;
		struct mhi_device *mhi_dev = mhi_chan->mhi_dev;

		if (mhi_dev)
			mhi_notify(mhi_dev, MHI_CB_PENDING_DATA);
	} else {
		tasklet_schedule(&mhi_event->task);
	}

	return IRQ_HANDLED;
}

irqreturn_t mhi_intvec_threaded_handler(int irq_number, void *dev)
{
	struct mhi_controller *mhi_cntrl = dev;
	enum mhi_state state = MHI_STATE_MAX;
	enum mhi_pm_state pm_state = 0;

	write_lock_irq(&mhi_cntrl->pm_lock);
	if (MHI_REG_ACCESS_VALID(mhi_cntrl->pm_state))
		state = mhi_get_m_state(mhi_cntrl);
	if (state == MHI_STATE_SYS_ERR) {
		dev_info(mhi_cntrl->dev, "MHI system error detected\n");
		pm_state = mhi_tryset_pm_state(mhi_cntrl,
					       MHI_PM_SYS_ERR_DETECT);
	}
	write_unlock_irq(&mhi_cntrl->pm_lock);
	if (pm_state == MHI_PM_SYS_ERR_DETECT)
		schedule_work(&mhi_cntrl->syserr_worker);

	return IRQ_HANDLED;
}

irqreturn_t mhi_intvec_handler(int irq_number, void *dev)
{
	struct mhi_controller *mhi_cntrl = dev;

	/* Wake up events waiting for state change */
	wake_up(&mhi_cntrl->state_event);

	return IRQ_WAKE_THREAD;
}

int mhi_send_cmd(struct mhi_controller *mhi_cntrl,
		 struct mhi_chan *mhi_chan,
		 enum mhi_cmd_type cmd)
{
	struct mhi_tre *cmd_tre = NULL;
	struct mhi_cmd *mhi_cmd = &mhi_cntrl->mhi_cmd[PRIMARY_CMD_RING];
	struct mhi_ring *ring = &mhi_cmd->ring;
	int chan = 0;

	if (mhi_chan)
		chan = mhi_chan->chan;

	spin_lock_bh(&mhi_cmd->lock);
	if (!get_nr_avail_ring_elements(mhi_cntrl, ring)) {
		spin_unlock_bh(&mhi_cmd->lock);
		return -ENOMEM;
	}

	/* prepare the cmd tre */
	cmd_tre = ring->wp;
	switch (cmd) {
	case MHI_CMD_RESET_CHAN:
		cmd_tre->ptr = MHI_TRE_CMD_RESET_PTR;
		cmd_tre->dword[0] = MHI_TRE_CMD_RESET_DWORD0;
		cmd_tre->dword[1] = MHI_TRE_CMD_RESET_DWORD1(chan);
		break;
	case MHI_CMD_START_CHAN:
		cmd_tre->ptr = MHI_TRE_CMD_START_PTR;
		cmd_tre->dword[0] = MHI_TRE_CMD_START_DWORD0;
		cmd_tre->dword[1] = MHI_TRE_CMD_START_DWORD1(chan);
		break;
	default:
		dev_err(mhi_cntrl->dev, "Command not supported\n");
		break;
	}

	/* queue to hardware */
	mhi_add_ring_element(mhi_cntrl, ring);
	read_lock_bh(&mhi_cntrl->pm_lock);
	if (likely(MHI_DB_ACCESS_VALID(mhi_cntrl->pm_state)))
		mhi_ring_cmd_db(mhi_cntrl, mhi_cmd);
	read_unlock_bh(&mhi_cntrl->pm_lock);
	spin_unlock_bh(&mhi_cmd->lock);

	return 0;
}

static void __mhi_unprepare_channel(struct mhi_controller *mhi_cntrl,
				    struct mhi_chan *mhi_chan)
{
	int ret;

	dev_info(mhi_cntrl->dev, "Entered: unprepare channel:%d\n",
		 mhi_chan->chan);

	/* no more processing events for this channel */
	mutex_lock(&mhi_chan->mutex);
	write_lock_irq(&mhi_chan->lock);
	if (mhi_chan->ch_state != MHI_CH_STATE_ENABLED) {
		write_unlock_irq(&mhi_chan->lock);
		mutex_unlock(&mhi_chan->mutex);
		return;
	}

	mhi_chan->ch_state = MHI_CH_STATE_DISABLED;
	write_unlock_irq(&mhi_chan->lock);

	reinit_completion(&mhi_chan->completion);
	read_lock_bh(&mhi_cntrl->pm_lock);
	if (MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state)) {
		read_unlock_bh(&mhi_cntrl->pm_lock);
		goto error_invalid_state;
	}

	mhi_cntrl->wake_get(mhi_cntrl, false);
	read_unlock_bh(&mhi_cntrl->pm_lock);

	mhi_cntrl->runtime_get(mhi_cntrl, mhi_cntrl->priv_data);
	mhi_cntrl->runtime_put(mhi_cntrl, mhi_cntrl->priv_data);
	ret = mhi_send_cmd(mhi_cntrl, mhi_chan, MHI_CMD_RESET_CHAN);
	if (ret)
		goto error_completion;

	/* even if it fails we will still reset */
	ret = wait_for_completion_timeout(&mhi_chan->completion,
				msecs_to_jiffies(mhi_cntrl->timeout_ms));
	if (!ret || mhi_chan->ccs != MHI_EV_CC_SUCCESS)
		dev_err(mhi_cntrl->dev,
			"Failed to receive cmd completion, still resetting\n");

error_completion:
	read_lock_bh(&mhi_cntrl->pm_lock);
	mhi_cntrl->wake_put(mhi_cntrl, false);
	read_unlock_bh(&mhi_cntrl->pm_lock);

error_invalid_state:
	if (!mhi_chan->offload_ch) {
		mhi_reset_chan(mhi_cntrl, mhi_chan);
		mhi_deinit_chan_ctxt(mhi_cntrl, mhi_chan);
	}
	dev_info(mhi_cntrl->dev, "chan:%d successfully resetted\n",
		 mhi_chan->chan);
	mutex_unlock(&mhi_chan->mutex);
}

static int __mhi_prepare_channel(struct mhi_controller *mhi_cntrl,
				 struct mhi_chan *mhi_chan)
{
	int ret = 0;

	dev_info(mhi_cntrl->dev, "Entered: preparing channel:%d\n",
		 mhi_chan->chan);

	if (mhi_cntrl->ee != mhi_chan->ee)
		return -ENOTCONN;

	mutex_lock(&mhi_chan->mutex);
	/* client manages channel context for offload channels */
	if (!mhi_chan->offload_ch) {
		ret = mhi_init_chan_ctxt(mhi_cntrl, mhi_chan);
		if (ret)
			goto error_init_chan;
	}

	reinit_completion(&mhi_chan->completion);
	read_lock_bh(&mhi_cntrl->pm_lock);
	if (MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state)) {
		read_unlock_bh(&mhi_cntrl->pm_lock);
		ret = -EIO;
		goto error_pm_state;
	}

	mhi_cntrl->wake_get(mhi_cntrl, false);
	read_unlock_bh(&mhi_cntrl->pm_lock);
	mhi_cntrl->runtime_get(mhi_cntrl, mhi_cntrl->priv_data);
	mhi_cntrl->runtime_put(mhi_cntrl, mhi_cntrl->priv_data);

	ret = mhi_send_cmd(mhi_cntrl, mhi_chan, MHI_CMD_START_CHAN);
	if (ret)
		goto error_send_cmd;

	ret = wait_for_completion_timeout(&mhi_chan->completion,
				msecs_to_jiffies(mhi_cntrl->timeout_ms));
	if (!ret || mhi_chan->ccs != MHI_EV_CC_SUCCESS) {
		ret = -EIO;
		goto error_send_cmd;
	}

	write_lock_irq(&mhi_chan->lock);
	mhi_chan->ch_state = MHI_CH_STATE_ENABLED;
	write_unlock_irq(&mhi_chan->lock);

	/* pre allocate buffer for xfer ring */
	if (mhi_chan->pre_alloc) {
		int nr_el = get_nr_avail_ring_elements(mhi_cntrl,
						       &mhi_chan->tre_ring);
		size_t len = mhi_cntrl->buffer_len;

		while (nr_el--) {
			void *buf;

			buf = kmalloc(len, GFP_KERNEL);
			if (!buf) {
				ret = -ENOMEM;
				goto error_pre_alloc;
			}

			/* prepare transfer descriptors */
			ret = mhi_chan->gen_tre(mhi_cntrl, mhi_chan, buf, buf,
						len, MHI_EOT);
			if (ret) {
				kfree(buf);
				goto error_pre_alloc;
			}
		}

		read_lock_bh(&mhi_cntrl->pm_lock);
		if (MHI_DB_ACCESS_VALID(mhi_cntrl->pm_state)) {
			read_lock_irq(&mhi_chan->lock);
			mhi_ring_chan_db(mhi_cntrl, mhi_chan);
			read_unlock_irq(&mhi_chan->lock);
		}
		read_unlock_bh(&mhi_cntrl->pm_lock);
	}

	read_lock_bh(&mhi_cntrl->pm_lock);
	mhi_cntrl->wake_put(mhi_cntrl, false);
	read_unlock_bh(&mhi_cntrl->pm_lock);

	mutex_unlock(&mhi_chan->mutex);

	dev_info(mhi_cntrl->dev, "Chan:%d successfully moved to start state\n",
		 mhi_chan->chan);

	return 0;

error_send_cmd:
	read_lock_bh(&mhi_cntrl->pm_lock);
	mhi_cntrl->wake_put(mhi_cntrl, false);
	read_unlock_bh(&mhi_cntrl->pm_lock);

error_pm_state:
	if (!mhi_chan->offload_ch)
		mhi_deinit_chan_ctxt(mhi_cntrl, mhi_chan);

error_init_chan:
	mutex_unlock(&mhi_chan->mutex);

	return ret;

error_pre_alloc:

	read_lock_bh(&mhi_cntrl->pm_lock);
	mhi_cntrl->wake_put(mhi_cntrl, false);
	read_unlock_bh(&mhi_cntrl->pm_lock);

	mutex_unlock(&mhi_chan->mutex);
	__mhi_unprepare_channel(mhi_cntrl, mhi_chan);

	return ret;
}

void mhi_reset_chan(struct mhi_controller *mhi_cntrl, struct mhi_chan *mhi_chan)
{
	struct mhi_tre *dev_rp, *local_rp;
	struct mhi_event_ctxt *er_ctxt;
	struct mhi_event *mhi_event;
	struct mhi_ring *ev_ring, *buf_ring, *tre_ring;
	unsigned long flags;
	int chan = mhi_chan->chan;
	struct mhi_result result;

	/* Nothing to reset, client hasn't queued buffers */
	if (mhi_chan->offload_ch)
		return;

	read_lock_bh(&mhi_cntrl->pm_lock);
	mhi_event = &mhi_cntrl->mhi_event[mhi_chan->er_index];
	ev_ring = &mhi_event->ring;
	er_ctxt = &mhi_cntrl->mhi_ctxt->er_ctxt[mhi_chan->er_index];

	/* Mark all stale events related to channel as STALE event */
	spin_lock_irqsave(&mhi_event->lock, flags);
	dev_rp = mhi_to_virtual(ev_ring, er_ctxt->rp);
	if (!mhi_event->mhi_chan) {
		local_rp = ev_ring->rp;
		while (dev_rp != local_rp) {
			if (MHI_TRE_GET_EV_TYPE(local_rp) ==
			    MHI_PKT_TYPE_TX_EVENT &&
			    chan == MHI_TRE_GET_EV_CHID(local_rp))
				local_rp->dword[1] = MHI_TRE_EV_DWORD1(chan,
						MHI_PKT_TYPE_STALE_EVENT);
			local_rp++;
			if (local_rp == (ev_ring->base + ev_ring->len))
				local_rp = ev_ring->base;
		}
	} else {
		ev_ring->rp = dev_rp;
		ev_ring->wp = ev_ring->rp - ev_ring->el_size;
		if (ev_ring->wp < ev_ring->base)
			ev_ring->wp = ev_ring->base + ev_ring->len -
				ev_ring->el_size;
		if (likely(MHI_DB_ACCESS_VALID(mhi_cntrl->pm_state)))
			mhi_ring_er_db(mhi_event);
	}

	spin_unlock_irqrestore(&mhi_event->lock, flags);

	/* Reset any pending buffers */
	buf_ring = &mhi_chan->buf_ring;
	tre_ring = &mhi_chan->tre_ring;
	result.transaction_status = -ENOTCONN;
	result.bytes_xferd = 0;
	while (tre_ring->rp != tre_ring->wp) {
		struct mhi_buf_info *buf_info = buf_ring->rp;

		if (mhi_chan->dir == DMA_TO_DEVICE)
			mhi_cntrl->wake_put(mhi_cntrl, false);

		mhi_cntrl->unmap_single(mhi_cntrl, buf_info);
		mhi_del_ring_element(mhi_cntrl, buf_ring);
		mhi_del_ring_element(mhi_cntrl, tre_ring);

		if (mhi_chan->pre_alloc) {
			kfree(buf_info->cb_buf);
		} else {
			result.buf_addr = buf_info->cb_buf;
			mhi_chan->xfer_cb(mhi_chan->mhi_dev, &result);
		}
	}

	read_unlock_bh(&mhi_cntrl->pm_lock);
}

/* Move channel to start state */
int mhi_prepare_for_transfer(struct mhi_device *mhi_dev)
{
	int ret, dir;
	struct mhi_controller *mhi_cntrl = mhi_dev->mhi_cntrl;
	struct mhi_chan *mhi_chan;

	for (dir = 0; dir < 2; dir++) {
		mhi_chan = dir ? mhi_dev->dl_chan : mhi_dev->ul_chan;

		if (!mhi_chan)
			continue;

		ret = __mhi_prepare_channel(mhi_cntrl, mhi_chan);
		if (ret)
			goto error_open_chan;
	}

	return 0;

error_open_chan:
	for (--dir; dir >= 0; dir--) {
		mhi_chan = dir ? mhi_dev->dl_chan : mhi_dev->ul_chan;

		if (!mhi_chan)
			continue;

		__mhi_unprepare_channel(mhi_cntrl, mhi_chan);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(mhi_prepare_for_transfer);

void mhi_unprepare_from_transfer(struct mhi_device *mhi_dev)
{
	struct mhi_controller *mhi_cntrl = mhi_dev->mhi_cntrl;
	struct mhi_chan *mhi_chan;
	int dir;

	for (dir = 0; dir < 2; dir++) {
		mhi_chan = dir ? mhi_dev->ul_chan : mhi_dev->dl_chan;

		if (!mhi_chan)
			continue;

		__mhi_unprepare_channel(mhi_cntrl, mhi_chan);
	}
}
EXPORT_SYMBOL_GPL(mhi_unprepare_from_transfer);

int mhi_get_no_free_descriptors(struct mhi_device *mhi_dev,
				enum dma_data_direction dir)
{
	struct mhi_controller *mhi_cntrl = mhi_dev->mhi_cntrl;
	struct mhi_chan *mhi_chan = (dir == DMA_TO_DEVICE) ?
		mhi_dev->ul_chan : mhi_dev->dl_chan;
	struct mhi_ring *tre_ring = &mhi_chan->tre_ring;

	return get_nr_avail_ring_elements(mhi_cntrl, tre_ring);
}
EXPORT_SYMBOL_GPL(mhi_get_no_free_descriptors);

struct mhi_controller *mhi_get_controller(const char *dev_name)
{
	struct mhi_device *mhi_dev;
	struct device *dev;

	dev = bus_find_device_by_name(&mhi_bus_type, NULL, dev_name);
	if (!dev)
		return NULL;

	mhi_dev = to_mhi_device(dev);

	return mhi_dev->mhi_cntrl;
}
EXPORT_SYMBOL_GPL(mhi_get_controller);

int mhi_poll(struct mhi_device *mhi_dev, u32 budget)
{
	struct mhi_controller *mhi_cntrl = mhi_dev->mhi_cntrl;
	struct mhi_chan *mhi_chan = mhi_dev->dl_chan;
	struct mhi_event *mhi_event = &mhi_cntrl->mhi_event[mhi_chan->er_index];
	int ret;

	spin_lock_bh(&mhi_event->lock);
	ret = mhi_event->process_event(mhi_cntrl, mhi_event, budget);
	spin_unlock_bh(&mhi_event->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(mhi_poll);
