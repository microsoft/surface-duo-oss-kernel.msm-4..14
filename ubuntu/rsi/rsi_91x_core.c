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

#include "rsi_mgmt.h"
#include "rsi_common.h"
#include "rsi_hal.h"
#ifdef CONFIG_VEN_RSI_COEX
#include "rsi_coex.h"
#endif

/**
 * rsi_determine_min_weight_queue() - This function determines the queue with
 *				      the min weight.
 * @common: Pointer to the driver private structure.
 *
 * Return: q_num: Corresponding queue number.
 */
static u8 rsi_determine_min_weight_queue(struct rsi_common *common)
{
	struct wmm_qinfo *tx_qinfo = common->tx_qinfo;
	u32 q_len = 0;
	u8 ii = 0;

	for (ii = 0; ii < NUM_EDCA_QUEUES; ii++) {
		q_len = skb_queue_len(&common->tx_queue[ii]);
		if ((tx_qinfo[ii].pkt_contended) && q_len) {
			common->min_weight = tx_qinfo[ii].weight;
			break;
		}
	}
	return ii;
}

/**
 * rsi_recalculate_weights() - This function recalculates the weights
 *			       corresponding to each queue.
 * @common: Pointer to the driver private structure.
 *
 * Return: recontend_queue bool variable
 */
static bool rsi_recalculate_weights(struct rsi_common *common)
{
	struct wmm_qinfo *tx_qinfo = common->tx_qinfo;
	bool recontend_queue = false;
	u8 ii = 0;
	u32 q_len = 0;

	for (ii = 0; ii < NUM_EDCA_QUEUES; ii++) {
		q_len = skb_queue_len(&common->tx_queue[ii]);
		/* Check for the need of contention */
		if (q_len) {
			if (tx_qinfo[ii].pkt_contended) {
				tx_qinfo[ii].weight =
				((tx_qinfo[ii].weight > common->min_weight) ?
				 tx_qinfo[ii].weight - common->min_weight : 0);
			} else {
				tx_qinfo[ii].pkt_contended = 1;
				tx_qinfo[ii].weight = tx_qinfo[ii].wme_params;
				recontend_queue = true;
			}
		} else { /* No packets so no contention */
			tx_qinfo[ii].weight = 0;
			tx_qinfo[ii].pkt_contended = 0;
		}
	}

	return recontend_queue;
}

/**
 * rsi_get_num_pkts_dequeue() - This function determines the number of
 *		                packets to be dequeued based on the number
 *			        of bytes calculated using txop.
 *
 * @common: Pointer to the driver private structure.
 * @q_num: the queue from which pkts have to be dequeued
 *
 * Return: pkt_num: Number of pkts to be dequeued.
 */
static u32 rsi_get_num_pkts_dequeue(struct rsi_common *common, u8 q_num)
{
	struct rsi_hw *adapter = common->priv;
	struct sk_buff *skb;
	u32 pkt_cnt = 0;
	s16 txop = common->tx_qinfo[q_num].txop * 32;
	__le16 r_txop;
	struct ieee80211_rate rate;

	rate.bitrate = RSI_RATE_MCS0 * 5 * 10; /* Convert to Kbps */
	if (q_num == VI_Q)
		txop = ((txop << 5) / 80);

	if (skb_queue_len(&common->tx_queue[q_num]))
		skb = skb_peek(&common->tx_queue[q_num]);
	else
		return 0;

	do {
		r_txop = ieee80211_generic_frame_duration(adapter->hw,
							  adapter->vifs[0],
							  common->band,
							  skb->len, &rate);
		txop -= le16_to_cpu(r_txop);
		pkt_cnt += 1;
		/*checking if pkts are still there*/
		if (skb_queue_len(&common->tx_queue[q_num]) - pkt_cnt)
			skb = skb->next;
		else
			break;

	} while (txop > 0);

	return pkt_cnt;
}

/**
 * rsi_core_determine_hal_queue() - This function determines the queue from
 *				    which packet has to be dequeued.
 * @common: Pointer to the driver private structure.
 *
 * Return: q_num: Corresponding queue number on success.
 */
static u8 rsi_core_determine_hal_queue(struct rsi_common *common)
{
	bool recontend_queue = false;
	u32 q_len = 0;
	u8 q_num = INVALID_QUEUE;
	u8 ii;

	if (skb_queue_len(&common->tx_queue[MGMT_SOFT_Q])) {
		if (!common->mgmt_q_block)
			q_num = MGMT_SOFT_Q;
		return q_num;
	}

	if (common->hw_data_qs_blocked) {
		ven_rsi_dbg(INFO_ZONE, "%s: data queue blocked\n", __func__);
		return q_num;
	}

	if (common->pkt_cnt != 0) {
		--common->pkt_cnt;
		return common->selected_qnum;
	}

get_queue_num:
	recontend_queue = false;

	q_num = rsi_determine_min_weight_queue(common);

	ii = q_num;

	/* Selecting the queue with least back off */
	for (; ii < NUM_EDCA_QUEUES; ii++) {
		q_len = skb_queue_len(&common->tx_queue[ii]);
		if (((common->tx_qinfo[ii].pkt_contended) &&
		     (common->tx_qinfo[ii].weight < common->min_weight)) &&
		      q_len) {
			common->min_weight = common->tx_qinfo[ii].weight;
			q_num = ii;
		}
	}

	if (q_num < NUM_EDCA_QUEUES)
		common->tx_qinfo[q_num].pkt_contended = 0;

	/* Adjust the back off values for all queues again */
	recontend_queue = rsi_recalculate_weights(common);

	q_len = skb_queue_len(&common->tx_queue[q_num]);
	if (!q_len) {
		/* If any queues are freshly contended and the selected queue
		 * doesn't have any packets
		 * then get the queue number again with fresh values
		 */
		if (recontend_queue)
			goto get_queue_num;

		q_num = INVALID_QUEUE;
		return q_num;
	}

	common->selected_qnum = q_num;
	q_len = skb_queue_len(&common->tx_queue[q_num]);

	if (q_num == VO_Q || q_num == VI_Q) {
		common->pkt_cnt = rsi_get_num_pkts_dequeue(common, q_num);
		common->pkt_cnt -= 1;
	}

	return q_num;
}

/**
 * rsi_core_queue_pkt() - This functions enqueues the packet to the queue
 *			  specified by the queue number.
 * @common: Pointer to the driver private structure.
 * @skb: Pointer to the socket buffer structure.
 *
 * Return: None.
 */
static void rsi_core_queue_pkt(struct rsi_common *common,
			       struct sk_buff *skb)
{
	u8 q_num = skb->priority;

	if (q_num >= NUM_SOFT_QUEUES) {
		ven_rsi_dbg(ERR_ZONE, "%s: Invalid Queue Number: q_num = %d\n",
			__func__, q_num);
		dev_kfree_skb(skb);
		return;
	}

	skb_queue_tail(&common->tx_queue[q_num], skb);
}

/**
 * rsi_core_dequeue_pkt() - This functions dequeues the packet from the queue
 *			    specified by the queue number.
 * @common: Pointer to the driver private structure.
 * @q_num: Queue number.
 *
 * Return: Pointer to sk_buff structure.
 */
static struct sk_buff *rsi_core_dequeue_pkt(struct rsi_common *common,
					    u8 q_num)
{
	if (q_num >= NUM_SOFT_QUEUES) {
		ven_rsi_dbg(ERR_ZONE, "%s: Invalid Queue Number: q_num = %d\n",
			__func__, q_num);
		return NULL;
	}

	return skb_dequeue(&common->tx_queue[q_num]);
}

/**
 * rsi_core_qos_processor() - This function is used to determine the wmm queue
 *			      based on the backoff procedure. Data packets are
 *			      dequeued from the selected hal queue and sent to
 *			      the below layers.
 * @common: Pointer to the driver private structure.
 *
 * Return: None.
 */
void rsi_core_qos_processor(struct rsi_common *common)
{
	struct rsi_hw *adapter = common->priv;
	struct sk_buff *skb;
	unsigned long tstamp_1, tstamp_2;
	u8 q_num;
	int status;

	tstamp_1 = jiffies;
	while (1) {
		q_num = rsi_core_determine_hal_queue(common);
		ven_rsi_dbg(DATA_TX_ZONE,
			"%s: Queue number = %d\n", __func__, q_num);

		if (q_num == INVALID_QUEUE)
			break;

		mutex_lock(&common->tx_lock);

		status = adapter->check_hw_queue_status(adapter, q_num);
		if (status <= 0) {
			mutex_unlock(&common->tx_lock);
			break;
		}

		if ((q_num < MGMT_SOFT_Q) &&
		    ((skb_queue_len(&common->tx_queue[q_num])) <=
		      MIN_DATA_QUEUE_WATER_MARK)) {
			if (!adapter->hw)
				break;
			if (ieee80211_queue_stopped(adapter->hw, WME_AC(q_num)))
				ieee80211_wake_queue(adapter->hw,
						     WME_AC(q_num));
		}

		skb = rsi_core_dequeue_pkt(common, q_num);
		if (!skb) {
			ven_rsi_dbg(ERR_ZONE, "skb null\n");
			mutex_unlock(&common->tx_lock);
			break;
		}
#ifdef CONFIG_VEN_RSI_COEX
		status = rsi_coex_send_pkt(common, skb, RSI_WLAN_Q);
#else
		if (q_num == MGMT_SOFT_Q)
			status = rsi_send_mgmt_pkt(common, skb);
		else
			status = rsi_send_data_pkt(common, skb);
#endif

		if (status) {
			mutex_unlock(&common->tx_lock);
			break;
		}

		common->tx_stats.total_tx_pkt_send[q_num]++;

		tstamp_2 = jiffies;
		mutex_unlock(&common->tx_lock);

		if (tstamp_2 > tstamp_1 + (300 * HZ / 1000))
			schedule();
	}
}

inline char *dot11_pkt_type(__le16 frame_control)
{
	if (ieee80211_is_beacon(frame_control))
		return "BEACON";
	if (ieee80211_is_assoc_req(frame_control))
		return "ASSOC_REQ";
	if (ieee80211_is_assoc_resp(frame_control))
		return "ASSOC_RESP";
	if (ieee80211_is_reassoc_req(frame_control))
		return "REASSOC_REQ";
	if (ieee80211_is_reassoc_resp(frame_control))
		return "REASSOC_RESP";
	if (ieee80211_is_auth(frame_control))
		return "AUTH";
	if (ieee80211_is_probe_req(frame_control))
		return "PROBE_REQ";
	if (ieee80211_is_probe_resp(frame_control))
		return "PROBE_RESP";
	if (ieee80211_is_disassoc(frame_control))
		return "DISASSOC";
	if (ieee80211_is_deauth(frame_control))
		return "DEAUTH";
	if (ieee80211_is_action(frame_control))
		return "ACTION";
	if (ieee80211_is_data_qos(frame_control))
		return "QOS DATA";
	if (ieee80211_is_pspoll(frame_control))
		return "PS_POLL";
	if (ieee80211_is_nullfunc(frame_control))
		return "NULL_DATA";
	if (ieee80211_is_qos_nullfunc(frame_control))
		return "QOS_NULL_DATA";

	if (ieee80211_is_mgmt(frame_control))
		return "DOT11_MGMT";
	if (ieee80211_is_data(frame_control))
		return "DOT11_DATA";
	if (ieee80211_is_ctl(frame_control))
		return "DOT11_CTRL";

	return "UNKNOWN";
}

struct rsi_sta *rsi_find_sta(struct rsi_common *common, u8 *mac_addr)
{
	int i;

	for (i = 0; i < RSI_MAX_ASSOC_STAS; i++) {
		if (!common->stations[i].sta)
			continue;
		if (!(memcmp(common->stations[i].sta->addr,
			     mac_addr, ETH_ALEN)))
			return &common->stations[i];
	}
	return NULL;
}

/**
 * rsi_core_xmit() - This function transmits the packets received from mac80211
 * @common: Pointer to the driver private structure.
 * @skb: Pointer to the socket buffer structure.
 *
 * Return: None.
 */
void rsi_core_xmit(struct rsi_common *common, struct sk_buff *skb)
{
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_tx_info *info;
	struct skb_info *tx_params;
	struct ieee80211_hdr *wlh = NULL;
	struct ieee80211_vif *vif = adapter->vifs[0];
	u8 q_num, tid = 0;

	if ((!skb) || (!skb->len)) {
		ven_rsi_dbg(ERR_ZONE, "%s: Null skb/zero Length packet\n",
			__func__);
		goto xmit_fail;
	}
#ifdef CONFIG_RSI_WOW
	if(common->suspend_flag) {
		ven_rsi_dbg(ERR_ZONE, "%s: Blocking Tx_packets when WOWLAN is enabled\n", __func__);
		goto xmit_fail;
	}
#endif
	if (common->fsm_state != FSM_MAC_INIT_DONE) {
		ven_rsi_dbg(ERR_ZONE, "%s: FSM state not open\n", __func__);
		goto xmit_fail;
	}

	info = IEEE80211_SKB_CB(skb);
	tx_params = (struct skb_info *)info->driver_data;
	wlh = (struct ieee80211_hdr *)&skb->data[0];

	if ((ieee80211_is_mgmt(wlh->frame_control)) ||
	    (ieee80211_is_ctl(wlh->frame_control)) ||
	    (ieee80211_is_qos_nullfunc(wlh->frame_control))) {
		if ((ieee80211_is_assoc_req(wlh->frame_control)) ||
	 	    (ieee80211_is_reassoc_req(wlh->frame_control))) {
			struct ieee80211_bss_conf *bss = NULL;

			bss = &adapter->vifs[0]->bss_conf;
			rsi_send_sta_notify_frame(common, STA_OPMODE,
						  STA_CONNECTED,
						  bss->bssid, bss->qos,
						  bss->aid, 0);
		}
		q_num = MGMT_SOFT_Q;
		skb->priority = q_num;
#ifdef CONFIG_RSI_WOW          
		if ((ieee80211_is_deauth(wlh->frame_control)) && (common->suspend_flag)) {
			ven_rsi_dbg(ERR_ZONE, "%s: Discarding Deauth when WOWLAN is enabled\n", __func__);
			goto xmit_fail; 
		}
#endif
		ven_rsi_dbg(INFO_ZONE, "Core: TX Dot11 Mgmt Pkt Type: %s\n",
			dot11_pkt_type(wlh->frame_control));
		if (ieee80211_is_probe_req(wlh->frame_control)) {
			if ((is_broadcast_ether_addr(wlh->addr1)) &&
			    (skb->data[MIN_802_11_HDR_LEN + 1] == 0)) {
				memcpy(common->bgscan_probe_req,
				       skb->data, skb->len);
				common->bgscan_probe_req_len = skb->len;
			}
		}
		if (rsi_prepare_mgmt_desc(common, skb)) {
			ven_rsi_dbg(ERR_ZONE, "Failed to prepeare desc\n");
			goto xmit_fail;
		}
	} else {
		struct rsi_sta *sta = NULL;

		ven_rsi_dbg(INFO_ZONE, "Core: TX Data Packet\n");
		rsi_hex_dump(DATA_TX_ZONE, "TX Data Packet",
			     skb->data, skb->len);

		/* Drop the null packets if bgscan is enabled
 		 * as it is already handled in firmware */
		if ((vif->type == NL80211_IFTYPE_STATION) && (common->bgscan_en)) {
			if (ieee80211_is_qos_nullfunc(wlh->frame_control)) {
				++common->tx_stats.total_tx_pkt_freed[skb->priority];
				rsi_indicate_tx_status(adapter, skb, 0);
				return;
			}
		}

		if (ieee80211_is_data_qos(wlh->frame_control)) {
			u8 *qos = ieee80211_get_qos_ctl(wlh);
			
			tid = *qos & IEEE80211_QOS_CTL_TID_MASK;
			skb->priority = TID_TO_WME_AC(tid);

			if ((vif->type == NL80211_IFTYPE_AP) &&
			    (!is_broadcast_ether_addr(wlh->addr1)) &&
			    (!is_multicast_ether_addr(wlh->addr1))) {
				sta = rsi_find_sta(common, wlh->addr1);
				if (!sta)
					goto xmit_fail;
			}
		} else {
			tid = IEEE80211_NONQOS_TID;
			skb->priority = BE_Q;
		
			if ((!is_broadcast_ether_addr(wlh->addr1)) &&
			    (!is_multicast_ether_addr(wlh->addr1)) &&
			    (vif->type == NL80211_IFTYPE_AP)) {
				sta = rsi_find_sta(common, wlh->addr1);
				if (!sta)
					goto xmit_fail;
			}
		}
		q_num = skb->priority;
		tx_params->tid = tid;

		if (sta) {
			wlh->seq_ctrl =
				cpu_to_le16((sta->seq_no[skb->priority] << 4) &
					    IEEE80211_SCTL_SEQ);
			sta->seq_no[skb->priority] =
				(sta->seq_no[skb->priority] + 1) % IEEE80211_MAX_SN;
			tx_params->sta_id = sta->sta_id;
		} else {
			if (vif->type == NL80211_IFTYPE_AP) {
				wlh->seq_ctrl =
					cpu_to_le16((common->bc_mc_seqno << 4) &
						    IEEE80211_SCTL_SEQ);
				common->bc_mc_seqno =
					(common->bc_mc_seqno + 1) % IEEE80211_MAX_SN;
			}
			tx_params->sta_id = 0;
		}

#ifdef EAPOL_IN_MGMT_Q
		if (skb->protocol == cpu_to_le16(ETH_P_PAE)) {
			q_num = MGMT_SOFT_Q;
			skb->priority = q_num;
		}
#endif
		if (rsi_prepare_data_desc(common, skb)) {
			ven_rsi_dbg(ERR_ZONE, "Failed to prepare data desc\n");
			goto xmit_fail;
		}
	}

	if ((q_num != MGMT_SOFT_Q) &&
	    ((skb_queue_len(&common->tx_queue[q_num]) + 1) >=
	      DATA_QUEUE_WATER_MARK)) {
		ven_rsi_dbg(ERR_ZONE, "%s: sw queue full\n", __func__);
		if (!ieee80211_queue_stopped(adapter->hw, WME_AC(q_num)))
			ieee80211_stop_queue(adapter->hw, WME_AC(q_num));
		rsi_set_event(&common->tx_thread.event);
		goto xmit_fail;
	}

	rsi_core_queue_pkt(common, skb);
	ven_rsi_dbg(DATA_TX_ZONE, "%s: ===> Scheduling TX thead <===\n", __func__);
	rsi_set_event(&common->tx_thread.event);

	return;

xmit_fail:
	ven_rsi_dbg(ERR_ZONE, "%s: Failed to queue packet\n", __func__);
	/* Dropping pkt here */
	ieee80211_free_txskb(common->priv->hw, skb);
}
