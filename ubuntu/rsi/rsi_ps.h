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
#ifndef __RSI_PS_H__
#define __RSI_PS_H__

#define PS_CONFIRM_INDEX	12

enum ps_state {
	PS_NONE = 0,
	PS_ENABLE_REQ_SENT = 1,
	PS_DISABLE_REQ_SENT = 2,
	PS_ENABLED = 3
};

struct ps_sleep_params {
	u8 enable;
	u8 sleep_type; //LP or ULP type
	u8 connected_sleep;
	u8 reserved1;
	u16 num_bcns_per_lis_int;
	u16 wakeup_type;
	u32 sleep_duration;
} __packed;

struct rsi_ps_info {
	u8 enabled;
	u8 sleep_type;
	u8 tx_threshold;
	u8 rx_threshold;
	u8 tx_hysterisis;
	u8 rx_hysterisis;
	u16 monitor_interval;
	u32 listen_interval;
	u16 num_bcns_per_lis_int;
	u32 dtim_interval_duration;
	u16 num_dtims_per_sleep;
	u32 deep_sleep_wakeup_period;
} __packed;

char *str_psstate(enum ps_state state);
void rsi_enable_ps(struct rsi_hw *adapter);
void rsi_disable_ps(struct rsi_hw *adapter);
int rsi_handle_ps_confirm(struct rsi_hw *adapter, u8 *msg);
void rsi_default_ps_params(struct rsi_hw *hw);
int rsi_send_ps_request(struct rsi_hw *adapter, bool enable);
void rsi_conf_uapsd(struct rsi_hw *adapter);

#endif
