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
