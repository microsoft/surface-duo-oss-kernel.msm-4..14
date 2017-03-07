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
