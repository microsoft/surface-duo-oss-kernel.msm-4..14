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

/* TX BT command packet types */
#define RSI_BT_PKT_TYPE_DEREGISTR		0x11
#define RSI_BT_PKT_TYPE_RFMODE			0x55

struct rsi_bt_cmd_frame {
#ifdef __LITTLE_ENDIAN
	u16 len:12;
	u16 q_no:4;
#else
	u16 reserved1:4;
	u16 q_no:12;
#endif
	__le16 reserved2[6];
	u8 pkt_type;
	u8 reserved3;
};

struct rsi_bt_rfmode_frame {
	struct rsi_bt_cmd_frame desc;
#ifdef __LITTLE_ENDIAN
	u8 bt_rf_tx_power_mode:4;
	u8 bt_rf_rx_power_mode:4;
#else
	u8 bt_rf_rx_power_mode:4;
	u8 bt_rf_tx_power_mode:4;
#endif
	u8 reserved;
};

int rsi_genl_recv (struct sk_buff *skb, struct genl_info *info);
int rsi_hci_attach (struct rsi_common *common);
void rsi_hci_detach(struct rsi_common *common);
int rsi_hci_recv_pkt(struct rsi_common *common, u8 *pkt);

#endif
