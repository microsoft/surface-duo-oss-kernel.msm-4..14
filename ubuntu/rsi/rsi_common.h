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

#ifndef __RSI_COMMON_H__
#define __RSI_COMMON_H__

#include <linux/kthread.h>

#define EVENT_WAIT_FOREVER              0
#define QUEUE_NOT_FULL                  1
#define QUEUE_FULL                      0

static inline int rsi_init_event(struct rsi_event *pevent)
{
	atomic_set(&pevent->event_condition, 1);
	init_waitqueue_head(&pevent->event_queue);
	return 0;
}

static inline int rsi_wait_event(struct rsi_event *event, u32 timeout)
{
	int status = 0;

	if (!timeout)
		status = wait_event_interruptible(event->event_queue,
				(!atomic_read(&event->event_condition)));
	else
		status = wait_event_interruptible_timeout(event->event_queue,
				(!atomic_read(&event->event_condition)),
				timeout);
	return status;
}

static inline void rsi_set_event(struct rsi_event *event)
{
	atomic_set(&event->event_condition, 0);
	wake_up_interruptible(&event->event_queue);
}

static inline void rsi_reset_event(struct rsi_event *event)
{
	atomic_set(&event->event_condition, 1);
}

static inline int rsi_create_kthread(struct rsi_common *common,
				     struct rsi_thread *thread,
				     void *func_ptr,
				     u8 *name)
{
	init_completion(&thread->completion);
	atomic_set(&thread->thread_done, 0);
	thread->task = kthread_run(func_ptr, common, "%s", name);
	if (IS_ERR(thread->task))
		return (int)PTR_ERR(thread->task);

	return 0;
}

static inline int rsi_kill_thread(struct rsi_thread *handle)
{
	if (atomic_read(&handle->thread_done) > 0)
		return 0;
	atomic_inc(&handle->thread_done);
	rsi_set_event(&handle->event);

	wait_for_completion(&handle->completion);
	return kthread_stop(handle->task);
}

void ven_rsi_mac80211_detach(struct rsi_hw *hw);
u16 rsi_get_connected_channel(struct rsi_hw *adapter);
struct rsi_hw *ven_rsi_91x_init(void);
void ven_rsi_91x_deinit(struct rsi_hw *adapter);
int ven_rsi_read_pkt(struct rsi_common *common, u8 *rx_pkt, s32 rcv_pkt_len);
void rsi_indicate_bcnmiss(struct rsi_common *common);
void rsi_resume_conn_channel(struct rsi_hw *adapter, struct ieee80211_vif *vif);
void rsi_hci_detach(struct rsi_common *common);
char *dot11_pkt_type(__le16 frame_control);
struct rsi_sta *rsi_find_sta(struct rsi_common *common, u8 *mac_addr);
void rsi_init_bcn_timer(struct rsi_common *common);
void rsi_del_bcn_timer(struct rsi_common *common);
void rsi_bcn_scheduler_thread(struct rsi_common *common);
#ifdef CONFIG_SDIO_INTR_POLL
void init_sdio_intr_status_poll_thread(struct rsi_common *common);
#endif
void rsi_roc_timeout(unsigned long data);
struct ieee80211_vif *rsi_get_vif(struct rsi_hw *adapter, u8 *mac);
#ifdef CONFIG_VEN_RSI_WOW
int rsi_config_wowlan(struct rsi_hw *adapter, struct cfg80211_wowlan *wowlan);
#endif
void rsi_mac80211_hw_scan_cancel(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif);
#endif
