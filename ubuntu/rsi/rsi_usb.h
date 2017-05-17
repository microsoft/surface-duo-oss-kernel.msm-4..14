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

#ifndef __RSI_USB_INTF__
#define __RSI_USB_INTF__

#include <linux/usb.h>
#include "rsi_main.h"
#include "rsi_common.h"

#define FW_STATUS_REG                0x41050012

#define USB_VENDOR_REGISTER_READ     0x15
#define USB_VENDOR_REGISTER_WRITE    0x16
#define RSI_USB_TX_HEAD_ROOM         128

#define MAX_TX_URBS                  1
#if defined (CONFIG_VEN_RSI_BT_ALONE) || defined(CONFIG_VEN_RSI_COEX)
#define MAX_RX_URBS                  2
#else
#define MAX_RX_URBS                  1
#endif
#define MAX_BULK_EP                  8
#define MGMT_EP                      1
#define DATA_EP                      2

struct rx_usb_ctrl_block {
	u8 *data;
	struct urb *rx_urb;
	u8 *rx_buffer;
	u8 *orig_rx_buffer;
	u8 ep_num;
	u8 pend;
};

struct receive_info {
	bool buffer_full;
	bool semi_buffer_full;
	bool mgmt_buffer_full;
	u32 mgmt_buf_full_counter;
	u32 buf_semi_full_counter;
	u8 watch_bufferfull_count;
	u32 buf_full_counter;
	u32 buf_available_counter;
};

struct rsi_91x_usbdev {
	void *priv;
	struct receive_info rx_info;
	struct rsi_thread rx_thread;
	u8 endpoint;
	struct usb_device *usbdev;
	struct usb_interface *pfunction;
	struct rx_usb_ctrl_block rx_cb[MAX_RX_URBS];
	u8 *tx_buffer;
	u8 *saved_tx_buffer;
	__le16 bulkin_size[MAX_BULK_EP];
	u8 bulkin_endpoint_addr[MAX_BULK_EP];
	__le16 bulkout_size[MAX_BULK_EP];
	u8 bulkout_endpoint_addr[MAX_BULK_EP];
	u32 tx_blk_size;
	u8 write_fail;
};

static inline int rsi_usb_event_timeout(struct rsi_hw *adapter)
{
	return EVENT_WAIT_FOREVER;
}

int rsi_usb_device_init(struct rsi_common *common);
int rsi_usb_read_register_multiple(struct rsi_hw *adapter, u32 addr,
				   u8 *data, u16 count);
int rsi_usb_write_register_multiple(struct rsi_hw *adapter, u32 addr,
				    u8 *data, u16 count);
void rsi_usb_rx_thread(struct rsi_common *common);

int rsi_usb_host_intf_write_pkt(struct rsi_hw *adapter, u8 *pkt, u32 len);
int rsi_usb_master_reg_read(struct rsi_hw *adapter, u32 reg,
			    u32 *value, u16 len);
int rsi_usb_master_reg_write(struct rsi_hw *adapter, unsigned long reg,
			     unsigned long value, u16 len);
int rsi_usb_load_data_master_write(struct rsi_hw *adapter, u32 base_address,
				   u32 instructions_sz,
				   u16 block_size,
				   u8 *ta_firmware);
int rsi_usb_check_queue_status(struct rsi_hw *adapter, u8 q_num);
#endif
