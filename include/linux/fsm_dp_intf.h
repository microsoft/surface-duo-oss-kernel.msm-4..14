/* Copyright (c) 2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef _FSM_DP_INTF_H_
#define _FSM_DP_INTF_H_

#include <linux/skbuff.h>
#include <linux/fsm_dp_ioctl.h>

int fsm_dp_tx_skb(
	void *handle,
	struct sk_buff *skb
);

int fsm_dp_rel_rx_buf(
	void *handle,
	unsigned char *buf
);

void *fsm_dp_register_kernel_client(
	enum fsm_dp_msg_type msg_type,
	int (*tx_cmplt_cb)(struct sk_buff *skb),
	int (*rx_cb)(struct page *p, unsigned int page_offset, char *buf,
						unsigned int length)
);

void fsm_dp_deregister_kernel_client(
	void *handle,
	enum fsm_dp_msg_type msg_type
);

void fsm_dp_hex_dump(
	unsigned char *buf,
	unsigned int len
);

#endif /* _FSM_DP_INTF_H_ */
