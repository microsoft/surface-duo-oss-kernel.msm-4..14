/**
 * Copyright (c) 2014 Redpine Signals Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef __RSI_COEX_H__
#define __RSI_COEX_H__

#include "rsi_common.h"

#define RSI_COEX_TXQ_MAX_PKTS		64
#define RSI_COEX_TXQ_WATER_MARK		50
#define COMMON_CARD_READY_IND           0

#define COEX_Q				0
#define BT_Q				1
#define WLAN_Q				2
#define VIP_Q				3
#define ZIGB_Q				4
#define NUM_COEX_TX_QUEUES		5

#include "rsi_main.h"

enum rsi_proto {
	RSI_PROTO_WLAN = 0,
	RSI_PROTO_BT
};

struct rsi_coex_ctrl_block {
	struct rsi_common *priv;
	struct sk_buff_head coex_tx_qs[NUM_COEX_TX_QUEUES];
        struct semaphore tx_bus_lock;
	struct rsi_thread coex_tx_thread;
};

int rsi_coex_init(struct rsi_common *common);
int rsi_coex_send_pkt(struct rsi_common *common,
		      struct sk_buff *skb,
		      u8 proto_type);
int rsi_coex_recv_pkt(struct rsi_common *common, u8 *msg);
void rsi_coex_deinit(struct rsi_common *common);
#endif
