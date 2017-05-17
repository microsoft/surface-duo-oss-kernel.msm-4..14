/*
 * Copyright (c) 2017 Redpine Signals Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	1. Redistributions of source code must retain the above copyright
 * 	   notice, this list of conditions and the following disclaimer.
 *
 * 	2. Redistributions in binary form must reproduce the above copyright
 * 	   notice, this list of conditions and the following disclaimer in the
 * 	   documentation and/or other materials provided with the distribution.
 *
 * 	3. Neither the name of the copyright holder nor the names of its
 * 	   contributors may be used to endorse or promote products derived from
 * 	   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rsi_main.h"
#include "rsi_coex.h"
#include "rsi_hal.h"
#include "rsi_mgmt.h"

static u8 rsi_coex_determine_coex_q(struct rsi_coex_ctrl_block *coex_cb)
{
	u8 q_num = INVALID_QUEUE;

	if (skb_queue_len(&coex_cb->coex_tx_qs[VIP_Q]) > 0)
		q_num = VIP_Q;
	if (skb_queue_len(&coex_cb->coex_tx_qs[COEX_Q]) > 0)
		q_num = COEX_Q;
	if (skb_queue_len(&coex_cb->coex_tx_qs[BT_Q]) > 0)
		q_num = BT_Q;
	if (skb_queue_len(&coex_cb->coex_tx_qs[ZIGB_Q]) > 0)
		q_num = ZIGB_Q;
	if (skb_queue_len(&coex_cb->coex_tx_qs[WLAN_Q]) > 0)
		q_num = WLAN_Q;

	return q_num;
}

static void rsi_coex_sched_tx_pkts(struct rsi_coex_ctrl_block *coex_cb)
{
	u8 coex_q;
	struct sk_buff *skb;

	while (1) {
		coex_q = rsi_coex_determine_coex_q(coex_cb);
		ven_rsi_dbg(INFO_ZONE, "queue = %d\n", coex_q);

		if (coex_q == INVALID_QUEUE) {
			ven_rsi_dbg(DATA_TX_ZONE, "No more pkt\n");
			break;
		}

		down(&coex_cb->priv->tx_bus_lock);

		if (coex_q == BT_Q) {
			skb = skb_dequeue(&coex_cb->coex_tx_qs[BT_Q]);
			rsi_send_bt_pkt(coex_cb->priv, skb);
		}

		up(&coex_cb->priv->tx_bus_lock);
	}
}

/**
 * rsi_coex_scheduler_thread() - This function is a kernel thread to schedule
 *			       the coex packets to device
 * @common: Pointer to the driver private structure.
 *
 * Return: None.
 */
static void rsi_coex_scheduler_thread(struct rsi_common *common)
{
	struct rsi_coex_ctrl_block *coex_cb =
		(struct rsi_coex_ctrl_block *)common->coex_cb;

	u32 timeout = EVENT_WAIT_FOREVER;

	do {
		rsi_wait_event(&coex_cb->coex_tx_thread.event, timeout);
		rsi_reset_event(&coex_cb->coex_tx_thread.event);

		rsi_coex_sched_tx_pkts(coex_cb);
	} while (atomic_read(&coex_cb->coex_tx_thread.thread_done) == 0);

	complete_and_exit(&coex_cb->coex_tx_thread.completion, 0);
}

int rsi_coex_recv_pkt(struct rsi_common *common, u8 *msg)
{
	u16 msg_type = msg[2];

	if (msg_type == COMMON_CARD_READY_IND) {
		ven_rsi_dbg(INFO_ZONE, "COMMON CARD READY RECEIVED\n");
		rsi_handle_card_ready(common, msg);
	} else if (msg_type == SLEEP_NOTIFY_IND) {
		ven_rsi_dbg(INFO_ZONE, "\n\n sleep notify RECEIVED\n");
		rsi_mgmt_pkt_recv(common, msg);
	}

	return 0;
}

int rsi_coex_send_pkt(struct rsi_common *common,
		      struct sk_buff *skb,
		      u8 hal_queue)
{
	struct rsi_coex_ctrl_block *coex_cb =
		(struct rsi_coex_ctrl_block *)common->coex_cb;
	struct skb_info *tx_params = NULL;
	int status = 0;

	/* Add pkt to queue if not WLAN packet */
	if (hal_queue != RSI_WLAN_Q) {
		skb_queue_tail(&coex_cb->coex_tx_qs[hal_queue], skb);
		rsi_set_event(&coex_cb->coex_tx_thread.event);
		return status;
	}
	if (common->iface_down) {
		tx_params = (struct skb_info *)&IEEE80211_SKB_CB(skb)->driver_data;

		if (!(tx_params->flags & INTERNAL_MGMT_PKT)) {
			rsi_indicate_tx_status(common->priv, skb, -EINVAL);
			return 0;
		}
	}

	/* Send packet to hal */
	if (skb->priority == MGMT_SOFT_Q)
		status = rsi_send_mgmt_pkt(common, skb);
	else
		status = rsi_send_data_pkt(common, skb);

	return 0;
}

int rsi_coex_init(struct rsi_common *common)
{
	struct rsi_coex_ctrl_block *coex_cb = NULL;
	int cnt;

	coex_cb = kzalloc(sizeof(*coex_cb), GFP_KERNEL);
	if (!coex_cb)
		return -ENOMEM;

	common->coex_cb = (void *)coex_cb;
	coex_cb->priv = common;
	sema_init(&coex_cb->tx_bus_lock, 1);

	/* Initialize co-ex queues */
	for (cnt = 0; cnt < NUM_COEX_TX_QUEUES; cnt++)
		skb_queue_head_init(&coex_cb->coex_tx_qs[cnt]);
	rsi_init_event(&coex_cb->coex_tx_thread.event);

	/* Initialize co-ex thread */
	if (rsi_create_kthread(common,
			       &coex_cb->coex_tx_thread,
			       rsi_coex_scheduler_thread,
			       "Coex-Tx-Thread")) {
		ven_rsi_dbg(ERR_ZONE, "%s: Unable to init tx thrd\n", __func__);
		goto err;
	}
	return 0;

err:
	return -EINVAL;
}

void rsi_coex_deinit(struct rsi_common *common)
{
	int cnt;

	struct rsi_coex_ctrl_block *coex_cb =
		(struct rsi_coex_ctrl_block *)common->coex_cb;

	/* Stop the coex tx thread */
	rsi_kill_thread(&coex_cb->coex_tx_thread);

	/* Empty the coex queue */
	for (cnt = 0; cnt < NUM_COEX_TX_QUEUES; cnt++)
		skb_queue_purge(&coex_cb->coex_tx_qs[cnt]);

	/* Free the coex control block */
	kfree(coex_cb);
}
