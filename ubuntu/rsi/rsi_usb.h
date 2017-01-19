/**
 * @section LICENSE
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
#if defined (CONFIG_VEN_RSI_HCI) || defined(CONFIG_VEN_RSI_COEX)
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

struct rsi_91x_usbdev {
	void *priv;
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

static inline int rsi_usb_check_queue_status(struct rsi_hw *adapter, u8 q_num)
{
	/* In USB, there isn't any need to check the queue status */
	return QUEUE_NOT_FULL;
}

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
#endif
