/* Copyright (c) 2011-2016, The Linux Foundation. All rights reserved.
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
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/slimbus.h>

/**
 * slim_msg_response: Deliver Message response received from a device to the
 *	framework.
 * @ctrl: Controller handle
 * @reply: Reply received from the device
 * @len: Length of the reply
 * @tid: Transaction ID received with which framework can associate reply.
 * Called by controller to inform framework about the response received.
 * This helps in making the API asynchronous, and controller-driver doesn't need
 * to manage 1 more table other than the one managed by framework mapping TID
 * with buffers
 */
void slim_msg_response(struct slim_controller *ctrl, u8 *reply, u8 tid, u8 len)
{
	struct slim_val_inf *msg;
	unsigned long flags;

	spin_lock_irqsave(&ctrl->txn_lock, flags);
	msg = ctrl->tid_tbl[tid];
	if (msg == NULL || msg->rbuf == NULL) {
		spin_unlock_irqrestore(&ctrl->txn_lock, flags);
		dev_err(&ctrl->dev, "Got response to invalid TID:%d, len:%d\n",
				tid, len);
		return;
	}
	ctrl->tid_tbl[tid] = NULL;
	spin_unlock_irqrestore(&ctrl->txn_lock, flags);

	memcpy(msg->rbuf, reply, len);
	if (msg->comp_cb)
		msg->comp_cb(msg->ctx, 0);
	/* Remove runtime-pm vote now that response was received for TID txn */
	pm_runtime_mark_last_busy(ctrl->dev.parent);
	pm_runtime_put_autosuspend(ctrl->dev.parent);
}
EXPORT_SYMBOL_GPL(slim_msg_response);

struct slim_cb_data {
	struct completion *comp;
	int ret;
};

static void slim_sync_default_cb(void *ctx, int err)
{
	struct slim_cb_data *cbd = ctx;

	cbd->ret = err;
	complete(cbd->comp);
}

int slim_processtxn(struct slim_controller *ctrl,
				struct slim_msg_txn *txn)
{
	int ret, i = 0;
	unsigned long flags;
	u8 *buf;
	bool async = false, clk_pause_msg = false;
	struct slim_cb_data cbd;
	DECLARE_COMPLETION_ONSTACK(done);
	bool need_tid = slim_tid_txn(txn->mt, txn->mc);

	/*
	 * do not vote for runtime-PM if the transactions are part of clock
	 * pause sequence
	 */
	if (ctrl->sched.clk_state == SLIM_CLK_ENTERING_PAUSE &&
		(txn->mt == SLIM_MSG_MT_CORE &&
		 txn->mc >= SLIM_MSG_MC_BEGIN_RECONFIGURATION &&
		 txn->mc <= SLIM_MSG_MC_RECONFIGURE_NOW))
		clk_pause_msg = true;

	if (!txn->msg->comp_cb) {
		txn->msg->comp_cb = slim_sync_default_cb;
		cbd.comp = &done;
		txn->msg->ctx = &cbd;
	} else {
		async = true;
	}

	buf = slim_get_tx(ctrl, txn, need_tid, clk_pause_msg);
	if (!buf)
		return -ENOMEM;

	if (need_tid) {
		spin_lock_irqsave(&ctrl->txn_lock, flags);
		for (i = 0; i < ctrl->last_tid; i++) {
			if (ctrl->tid_tbl[i] == NULL)
				break;
		}
		if (i >= ctrl->last_tid) {
			if (ctrl->last_tid == (SLIM_MAX_TIDS - 1)) {
				spin_unlock_irqrestore(&ctrl->txn_lock, flags);
				slim_return_tx(ctrl, -ENOMEM);
				ret = cbd.ret;
				return ret;
			}
			ctrl->last_tid++;
		}
		ctrl->tid_tbl[i] = txn->msg;
		txn->tid = i;
		spin_unlock_irqrestore(&ctrl->txn_lock, flags);
	}

	ret = ctrl->xfer_msg(ctrl, txn, buf);

	if (!ret && !async) { /* sync transaction */
		/* Fine-tune calculation after bandwidth management */
		unsigned long ms = txn->rl + 100;

		ret = wait_for_completion_timeout(&done,
						  msecs_to_jiffies(ms));
		if (!ret)
			slim_return_tx(ctrl, -ETIMEDOUT);

		ret = cbd.ret;
	}

	if (ret && need_tid) {
		spin_lock_irqsave(&ctrl->txn_lock, flags);
		/* Invalidate the transaction */
		ctrl->tid_tbl[txn->tid] = NULL;
		spin_unlock_irqrestore(&ctrl->txn_lock, flags);
	}
	if (ret)
		dev_err(&ctrl->dev, "Tx:MT:0x%x, MC:0x%x, LA:0x%x failed:%d\n",
			txn->mt, txn->mc, txn->la, ret);
	if (!async) {
		txn->msg->comp_cb = NULL;
		txn->msg->ctx = NULL;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(slim_processtxn);

static int slim_val_inf_sanity(struct slim_controller *ctrl,
			       struct slim_val_inf *msg, u8 mc)
{
	if (!msg || msg->num_bytes > 16 ||
	    (msg->start_offset + msg->num_bytes) > 0xC00)
		goto reterr;
	switch (mc) {
	case SLIM_MSG_MC_REQUEST_VALUE:
	case SLIM_MSG_MC_REQUEST_INFORMATION:
		if (msg->rbuf != NULL)
			return 0;
		break;
	case SLIM_MSG_MC_CHANGE_VALUE:
	case SLIM_MSG_MC_CLEAR_INFORMATION:
		if (msg->wbuf != NULL)
			return 0;
		break;
	case SLIM_MSG_MC_REQUEST_CHANGE_VALUE:
	case SLIM_MSG_MC_REQUEST_CLEAR_INFORMATION:
		if (msg->rbuf != NULL && msg->wbuf != NULL)
			return 0;
		break;
	default:
		break;
	}
reterr:
	dev_err(&ctrl->dev, "Sanity check failed:msg:offset:0x%x, mc:%d\n",
		msg->start_offset, mc);
	return -EINVAL;
}

static u16 slim_slicecodefromsize(u16 req)
{
	static const u8 codetosize[8] = {1, 2, 3, 4, 6, 8, 12, 16};

	if (req >= ARRAY_SIZE(codetosize))
		return 0;
	else
		return codetosize[req];
}

static u16 slim_slicesize(int code)
{
	static const u8 sizetocode[16] = {
		0, 1, 2, 3, 3, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7
	};

	clamp(code, 1, (int)ARRAY_SIZE(sizetocode));
	return sizetocode[code - 1];
}

int slim_xfer_msg(struct slim_controller *ctrl,
			struct slim_device *sbdev, struct slim_val_inf *msg,
			u8 mc)
{
	DEFINE_SLIM_LDEST_TXN(txn_stack, mc, 6, sbdev->laddr, msg);
	struct slim_msg_txn *txn = &txn_stack;
	int ret;
	u16 sl, cur;

	ret = slim_val_inf_sanity(ctrl, msg, mc);
	if (ret)
		return ret;

	sl = slim_slicesize(msg->num_bytes);

	dev_dbg(&ctrl->dev, "SB xfer msg:os:%x, len:%d, MC:%x, sl:%x\n",
		msg->start_offset, msg->num_bytes, mc, sl);

	cur = slim_slicecodefromsize(sl);
	txn->ec = ((sl | (1 << 3)) | ((msg->start_offset & 0xFFF) << 4));

	switch (mc) {
	case SLIM_MSG_MC_REQUEST_CHANGE_VALUE:
	case SLIM_MSG_MC_CHANGE_VALUE:
	case SLIM_MSG_MC_REQUEST_CLEAR_INFORMATION:
	case SLIM_MSG_MC_CLEAR_INFORMATION:
		txn->rl += msg->num_bytes;
	default:
		break;
	}

	if (slim_tid_txn(txn->mt, txn->mc))
		txn->rl++;

	return slim_processtxn(ctrl, txn);
}
EXPORT_SYMBOL_GPL(slim_xfer_msg);

/* Message APIs Unicast message APIs used by slimbus slave drivers */

/*
 * Message API access routines.
 * @sb: client handle requesting elemental message reads, writes.
 * @msg: Input structure for start-offset, number of bytes to read.
 * context: can sleep
 * Returns:
 * -EINVAL: Invalid parameters
 * -ETIMEDOUT: If transmission of this message timed out (e.g. due to bus lines
 *	not being clocked or driven by controller)
 * -ENOTCONN: If the transmitted message was not ACKed by destination device.
 */
int slim_request_val_element(struct slim_device *sb,
				struct slim_val_inf *msg)
{
	struct slim_controller *ctrl = sb->ctrl;

	if (!ctrl)
		return -EINVAL;

	return slim_xfer_msg(ctrl, sb, msg, SLIM_MSG_MC_REQUEST_VALUE);
}
EXPORT_SYMBOL_GPL(slim_request_val_element);

int slim_request_inf_element(struct slim_device *sb,
				struct slim_val_inf *msg)
{
	struct slim_controller *ctrl = sb->ctrl;

	if (!ctrl)
		return -EINVAL;

	return slim_xfer_msg(ctrl, sb, msg, SLIM_MSG_MC_REQUEST_INFORMATION);
}
EXPORT_SYMBOL_GPL(slim_request_inf_element);

int slim_change_val_element(struct slim_device *sb, struct slim_val_inf *msg)
{
	struct slim_controller *ctrl = sb->ctrl;

	if (!ctrl)
		return -EINVAL;

	return slim_xfer_msg(ctrl, sb, msg, SLIM_MSG_MC_CHANGE_VALUE);
}
EXPORT_SYMBOL_GPL(slim_change_val_element);

int slim_clear_inf_element(struct slim_device *sb, struct slim_val_inf *msg)
{
	struct slim_controller *ctrl = sb->ctrl;

	if (!ctrl)
		return -EINVAL;

	return slim_xfer_msg(ctrl, sb, msg, SLIM_MSG_MC_CLEAR_INFORMATION);
}
EXPORT_SYMBOL_GPL(slim_clear_inf_element);

int slim_request_change_val_element(struct slim_device *sb,
					struct slim_val_inf *msg)
{
	struct slim_controller *ctrl = sb->ctrl;

	if (!ctrl)
		return -EINVAL;

	return slim_xfer_msg(ctrl, sb, msg, SLIM_MSG_MC_REQUEST_CHANGE_VALUE);
}
EXPORT_SYMBOL_GPL(slim_request_change_val_element);

int slim_request_clear_inf_element(struct slim_device *sb,
					struct slim_val_inf *msg)
{
	struct slim_controller *ctrl = sb->ctrl;

	if (!ctrl)
		return -EINVAL;

	return slim_xfer_msg(ctrl, sb, msg,
					SLIM_MSG_MC_REQUEST_CLEAR_INFORMATION);
}
EXPORT_SYMBOL_GPL(slim_request_clear_inf_element);

/* Functions to get/return TX, RX buffers for messaging. */

void *slim_get_rx(struct slim_controller *ctrl)
{
	unsigned long flags;
	int idx;

	spin_lock_irqsave(&ctrl->rx.lock, flags);
	if ((ctrl->rx.tail + 1) % ctrl->rx.n == ctrl->rx.head) {
		spin_unlock_irqrestore(&ctrl->rx.lock, flags);
		dev_err(&ctrl->dev, "RX QUEUE full!");
		return NULL;
	}
	idx = ctrl->rx.tail;
	ctrl->rx.tail = (ctrl->rx.tail + 1) % ctrl->rx.n;
	spin_unlock_irqrestore(&ctrl->rx.lock, flags);

	return ctrl->rx.base + (idx * ctrl->rx.sl_sz);
}
EXPORT_SYMBOL_GPL(slim_get_rx);

int slim_return_rx(struct slim_controller *ctrl, void *buf)
{
	unsigned long flags;

	spin_lock_irqsave(&ctrl->rx.lock, flags);
	if (ctrl->rx.tail == ctrl->rx.head) {
		spin_unlock_irqrestore(&ctrl->rx.lock, flags);
		return -ENODATA;
	}
	memcpy(buf, ctrl->rx.base + (ctrl->rx.head * ctrl->rx.sl_sz),
				ctrl->rx.sl_sz);
	ctrl->rx.head = (ctrl->rx.head + 1) % ctrl->rx.n;
	spin_unlock_irqrestore(&ctrl->rx.lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(slim_return_rx);

void slim_return_tx(struct slim_controller *ctrl, int err)
{
	unsigned long flags;
	int idx;
	struct slim_pending cur;

	spin_lock_irqsave(&ctrl->tx.lock, flags);
	idx = ctrl->tx.head;
	ctrl->tx.head = (ctrl->tx.head + 1) % ctrl->tx.n;
	cur = ctrl->pending_wr[idx];
	spin_unlock_irqrestore(&ctrl->tx.lock, flags);

	if (!cur.cb)
		dev_err(&ctrl->dev, "NULL Transaction or completion");
	else
		cur.cb(cur.ctx, err);

	up(&ctrl->tx_sem);
	if (!cur.clk_pause && (!cur.need_tid || err)) {
		/**
		 * remove runtime-pm vote if this was TX only, or
		 * if there was error during this transaction
		 */
		pm_runtime_mark_last_busy(ctrl->dev.parent);
		pm_runtime_put_autosuspend(ctrl->dev.parent);
	}
}
EXPORT_SYMBOL_GPL(slim_return_tx);

void *slim_get_tx(struct slim_controller *ctrl, struct slim_msg_txn *txn,
		bool need_tid, bool clk_pause)
{
	unsigned long flags;
	int ret, idx;

	if (!clk_pause) {
		ret = pm_runtime_get_sync(ctrl->dev.parent);

		if (ctrl->sched.clk_state != SLIM_CLK_ACTIVE) {
			dev_err(&ctrl->dev, "ctrl wrong state:%d, ret:%d\n",
				ctrl->sched.clk_state, ret);
			goto slim_tx_err;
		}
	}

	ret = down_interruptible(&ctrl->tx_sem);
	if (ret < 0) {
		dev_err(&ctrl->dev, "TX semaphore down returned:%d", ret);
		goto slim_tx_err;
	}
	spin_lock_irqsave(&ctrl->tx.lock, flags);
	if (((ctrl->tx.head + 1) % ctrl->tx.n) == ctrl->tx.tail) {
		spin_unlock_irqrestore(&ctrl->tx.lock, flags);
		dev_err(&ctrl->dev, "controller TX buf unavailable");
		up(&ctrl->tx_sem);
		goto slim_tx_err;
	}
	idx = ctrl->tx.tail;
	ctrl->tx.tail = (ctrl->tx.tail + 1) % ctrl->tx.n;
	ctrl->pending_wr[idx].cb = txn->msg->comp_cb;
	ctrl->pending_wr[idx].ctx = txn->msg->ctx;
	ctrl->pending_wr[idx].need_tid = need_tid;
	ctrl->pending_wr[idx].clk_pause = clk_pause;
	spin_unlock_irqrestore(&ctrl->tx.lock, flags);

	return ctrl->tx.base + (idx * ctrl->tx.sl_sz);
slim_tx_err:
	pm_runtime_mark_last_busy(ctrl->dev.parent);
	pm_runtime_put_autosuspend(ctrl->dev.parent);
	return NULL;
}
EXPORT_SYMBOL_GPL(slim_get_tx);
