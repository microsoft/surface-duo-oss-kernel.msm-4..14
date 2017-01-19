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

#ifndef __RSI_HCI_H__
#define __RSI_HCI_H__

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <net/genetlink.h>
#include <linux/version.h>

#include "rsi_main.h"

#define BB_READ 			0x0
#define BB_WRITE 			0x1
#define RF_READ 			0x2
#define RF_WRITE 			0x3
#define BT_PER_TRANSMIT 	0x4
#define BT_RECEIVE 			0x5
#define BUFFER_READ 		0x6
#define BUFFER_WRITE 		0x7
#define BT_PER_STATS 		0x8
#define ANT_SEL 			0x9
#define BT_BER_PKT_CNT 		0xA
#define BT_BER_RECEIVE 		0xB
#define BT_BER_MODE 		0xC
#define BT_CW_MODE 			0xD 
#define TX_STATUS 			0xE
#define GET_DRV_COEX_MODE 	0xF

/* RX frame types */
#define RESULT_CONFIRM		0x80
#define BT_PER 				0x10
#define BT_BER 				0x11
#define BT_CW 				0x12

#define REQUIRED_HEADROOM_FOR_BT_HAL     16

#define GET_ADAPTER_FROM_GENLCB (gcb) \
	        (struct rsi_hci_adapter *)((gcb)->gc_drvpriv)

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 6, 11)
# define get_portid(_info) (_info)->snd_pid
#else
# define get_portid(_info) (_info)->snd_portid
#endif
    
enum {
	RSI_USER_A_UNSPEC,
	RSI_USER_A_MSG,
	__RSI_USER_A_MAX,
};

enum {
	RSI_USER_C_UNSPEC,
	RSI_USER_C_CMD,
	__RSI_USER_C_MAX,
};

struct genl_cb {
	unsigned char gc_cmd, *gc_name;
	int gc_seq, gc_pid;
	int gc_done;
	int gc_n_ops; 
	void  *gc_drvpriv;
	struct nla_policy *gc_policy;
	struct genl_family *gc_family;
	struct genl_ops *gc_ops; 
	struct genl_info *gc_info;
	struct sk_buff *gc_skb;
};

enum {
	BT_DEVICE_NOT_READY = 0,
	BT_DEVICE_READY
};

struct rsi_hci_adapter {
	struct rsi_common *priv;
	struct hci_dev *hdev;
	struct genl_cb *gcb;
	struct sk_buff_head hci_tx_queue;
};

int rsi_genl_recv (struct sk_buff *skb, struct genl_info *info);
int rsi_hci_attach (struct rsi_common *common);
void rsi_hci_detach(struct rsi_common *common);
int rsi_hci_recv_pkt(struct rsi_common *common, u8 *pkt);

#endif
