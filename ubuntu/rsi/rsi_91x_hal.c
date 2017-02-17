/**
 * Copyright (c) 2014 Redpine Signals Inc.
 *
 * Developers
 *	Prameela Rani Garnepudi 2016 <prameela.garnepudi@redpinesignals.com>
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

#include <linux/firmware.h>
#include <linux/version.h>
#include <linux/jiffies.h>
#include <net/bluetooth/bluetooth.h>
#include "rsi_mgmt.h"
#include "rsi_hal.h"
#include "rsi_sdio.h"
#include "rsi_common.h"
#if defined(CONFIG_VEN_RSI_COEX) || defined(CONFIG_VEN_RSI_HCI)
#include "rsi_hci.h"
#endif
#ifdef CONFIG_VEN_RSI_COEX
#include "rsi_coex.h"
#endif

/* FLASH Firmware */
struct ta_metadata metadata_flash_content[] = {
	{"flash_content", 0x00010000},
	{"RS9113_WLAN_QSPI.rps", 0x00010000},
	{"RS9113_WLAN_BT_DUAL_MODE.rps", 0x00010000},
	{"RS9113_WLAN_ZIGBEE.rps", 0x00010000},
	{"RS9113_AP_BT_DUAL_MODE.rps", 0x00010000},
	{"RS9113_WLAN_QSPI.rps", 0x00010000}
};

/**
 * rsi_send_pkt() - This function sends the received packet from
 *			 driver to device.
 * @common: Pointer to the driver private structure.
 * @skb: Pointer to the socket buffer structure.
 *
 * Return: status: 0 on success, -1 on failure.
 */
int rsi_send_pkt(struct rsi_common *common, struct sk_buff *skb)
{
	struct rsi_hw *adapter = common->priv;
#ifdef CONFIG_VEN_RSI_COEX
	struct rsi_coex_ctrl_block *coex_cb =
		(struct rsi_coex_ctrl_block *)common->coex_cb;
#endif
	int status = -EINVAL;

#ifdef CONFIG_VEN_RSI_COEX
	down(&coex_cb->tx_bus_lock);
#endif
	status = adapter->host_intf_ops->write_pkt(common->priv,
						   skb->data, skb->len);
#ifdef CONFIG_VEN_RSI_COEX
	up(&coex_cb->tx_bus_lock);
#endif
	return status;
}

/**
 * rsi_prepare_data_desc() - This function prepares the device specific descriptor
 *			     for the given data packet
 *
 * @common: Pointer to the driver private structure.
 * @skb: Pointer to the socket buffer structure.
 *
 * Return: status: 0 on success, negative error code on failure.
 */
int rsi_prepare_data_desc(struct rsi_common *common, struct sk_buff *skb)
{
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_vif *vif = adapter->vifs[0];
	struct ieee80211_hdr *wh = NULL;
	struct ieee80211_tx_info *info;
	struct skb_info *tx_params;
	int status = -EINVAL;
	u8 ieee80211_hdr_size = MIN_802_11_HDR_LEN;
	u8 dword_align_bytes = 0;
	u8 header_size = 0;
	__le16 *frame_desc;
	struct xtended_desc *xtend_desc;
	u16 seq_num = 0;

	info = IEEE80211_SKB_CB(skb);
	tx_params = (struct skb_info *)info->driver_data;

	header_size = FRAME_DESC_SZ + sizeof(struct xtended_desc);
	if (header_size > skb_headroom(skb)) {
		ven_rsi_dbg(ERR_ZONE, "%s: Not enough headroom\n", __func__);
		status = -ENOSPC;
		goto err;
	}
	skb_push(skb, header_size);
	dword_align_bytes = ((unsigned long)skb->data & 0x3f);
	if (header_size > skb_headroom(skb)) {
		ven_rsi_dbg(ERR_ZONE, "%s: Not enough headroom\n", __func__);
		status = -ENOSPC;
		goto err;
	}
	skb_push(skb, dword_align_bytes);
	header_size += dword_align_bytes;

	tx_params->internal_hdr_size = header_size;
	frame_desc = (__le16 *)&skb->data[0];
	xtend_desc = (struct xtended_desc *)&skb->data[FRAME_DESC_SZ];
	memset((u8 *)frame_desc, 0, header_size);

	wh = (struct ieee80211_hdr *)&skb->data[header_size];
	seq_num = le16_to_cpu(IEEE80211_SEQ_TO_SN(wh->seq_ctrl));

	frame_desc[2] = cpu_to_le16(header_size - FRAME_DESC_SZ);
	if (ieee80211_is_data_qos(wh->frame_control)) {
		ieee80211_hdr_size += 2;
		frame_desc[6] |= cpu_to_le16(BIT(12));
	}

	if ((vif->type == NL80211_IFTYPE_STATION) &&
	    (adapter->ps_state == PS_ENABLED))
		wh->frame_control |= BIT(12);

	if ((!(info->flags & IEEE80211_TX_INTFL_DONT_ENCRYPT)) &&
	    (common->secinfo.security_enable)) {
		if (rsi_is_cipher_wep(common))
			ieee80211_hdr_size += 4;
		else
			ieee80211_hdr_size += 8;
		frame_desc[6] |= cpu_to_le16(BIT(15));
	}

	frame_desc[0] = cpu_to_le16((skb->len - FRAME_DESC_SZ) |
				    (RSI_WIFI_DATA_Q << 12));
	frame_desc[2] |= cpu_to_le16(ieee80211_hdr_size << 8);

	if (common->min_rate != 0xffff) {
		/* Send fixed rate */
		frame_desc[3] = cpu_to_le16(RATE_INFO_ENABLE);
		frame_desc[4] = cpu_to_le16(common->min_rate);

		if (conf_is_ht40(&common->priv->hw->conf))
			frame_desc[5] = cpu_to_le16(FULL40M_ENABLE);

		if ((common->vif_info[0].sgi) && (common->min_rate & 0x100)) {
			/* Only MCS rates */
			frame_desc[4] |= cpu_to_le16(ENABLE_SHORTGI_RATE);
		}
	}

	if (skb->protocol == cpu_to_be16(ETH_P_PAE)) {
		ven_rsi_dbg(INFO_ZONE, "*** Tx EAPOL ***\n");
		
		frame_desc[3] = cpu_to_le16(RATE_INFO_ENABLE);
		if (common->band == NL80211_BAND_5GHZ)
				frame_desc[4] = cpu_to_le16(RSI_RATE_6);
			else
				frame_desc[4] = cpu_to_le16(RSI_RATE_1);
		frame_desc[6] |= cpu_to_le16(BIT(13));
		frame_desc[1] |= cpu_to_le16(BIT(12));
		if (vif->type == NL80211_IFTYPE_STATION) {
			frame_desc[0] = cpu_to_le16((skb->len - FRAME_DESC_SZ) |
					(RSI_WIFI_MGMT_Q << 12));
			if ((skb->len - header_size) == 133) {
				ven_rsi_dbg(INFO_ZONE, "*** Tx EAPOL 4*****\n");
				frame_desc[1] |=
					cpu_to_le16(RSI_DESC_REQUIRE_CFM_TO_HOST);
				xtend_desc->confirm_frame_type = EAPOL4_CONFIRM;
			}
		}
#define EAPOL_RETRY_CNT 15
		xtend_desc->retry_cnt = EAPOL_RETRY_CNT;
#if 0 
		skb->priority = VO_Q;
#endif
	}

	frame_desc[6] |= cpu_to_le16(seq_num);
	frame_desc[7] = cpu_to_le16(((tx_params->tid & 0xf) << 4) |
				    (skb->priority & 0xf) |
				    (tx_params->sta_id << 8));

	if ((is_broadcast_ether_addr(wh->addr1)) ||
	    (is_multicast_ether_addr(wh->addr1))) {
		frame_desc[3] = cpu_to_le16(RATE_INFO_ENABLE);
		frame_desc[3] |= cpu_to_le16(RSI_BROADCAST_PKT);
		if (vif->type == NL80211_IFTYPE_AP) {
			if (common->band == NL80211_BAND_5GHZ)
				frame_desc[4] = cpu_to_le16(RSI_RATE_6);
			else
				frame_desc[4] = cpu_to_le16(RSI_RATE_1);
		}
	}

	if ((vif->type == NL80211_IFTYPE_AP) &&
	    (ieee80211_has_moredata(wh->frame_control)))
		frame_desc[3] |= cpu_to_le16(MORE_DATA_PRESENT);

	return 0;

err:
	++common->tx_stats.total_tx_pkt_freed[skb->priority];
	rsi_indicate_tx_status(adapter, skb, status);
	return status;
}

/**
 * rsi_prepare_mgmt_desc() - This functions prepares the descriptor for
 *			     the given management packet.
 *
 * @common: Pointer to the driver private structure.
 * @skb: Pointer to the socket buffer structure.
 *
 * Return: status: 0 on success, -1 on failure.
 */
int rsi_prepare_mgmt_desc(struct rsi_common *common,struct sk_buff *skb)
{
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_hdr *wh = NULL;
	struct ieee80211_tx_info *info;
	struct ieee80211_conf *conf = &adapter->hw->conf;
	struct ieee80211_vif *vif = adapter->vifs[0];
	struct skb_info *tx_params;
	int status = -EINVAL;
	__le16 *desc = NULL;
	struct xtended_desc *xtend_desc = NULL;
	u8 header_size = 0;
	u8 vap_id = 0;
	u32 dword_align_bytes = 0;

	info = IEEE80211_SKB_CB(skb);
	tx_params = (struct skb_info *)info->driver_data;

	/* Update header size */
	header_size = FRAME_DESC_SZ + sizeof(struct xtended_desc);
	if (header_size > skb_headroom(skb)) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Failed to add extended descriptor\n",
			__func__);
		status = -ENOSPC;
		goto err;
	}
	skb_push(skb, header_size);
	dword_align_bytes = ((unsigned long)skb->data & 0x3f);
	if (dword_align_bytes > skb_headroom(skb)) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Failed to add dword align\n", __func__);
		status = -ENOSPC;
		goto err;
	}
	skb_push(skb, dword_align_bytes);
	header_size += dword_align_bytes;

	tx_params->internal_hdr_size = header_size;
	memset(&skb->data[0], 0, header_size);

	wh = (struct ieee80211_hdr *)&skb->data[header_size];

	desc = (__le16 *)skb->data;
	xtend_desc = (struct xtended_desc *)&skb->data[FRAME_DESC_SZ];

	if (skb->len > MAX_MGMT_PKT_SIZE) {
		ven_rsi_dbg(INFO_ZONE, "%s: Dropping mgmt pkt > 512\n", __func__);
		goto err;
	}

	desc[0] = cpu_to_le16((skb->len - FRAME_DESC_SZ) |
			      (RSI_WIFI_MGMT_Q << 12));
	desc[1] = cpu_to_le16(TX_DOT11_MGMT);
	desc[2] = cpu_to_le16(MIN_802_11_HDR_LEN << 8);
	desc[2] |= cpu_to_le16(header_size - FRAME_DESC_SZ);
	desc[3] = cpu_to_le16(RATE_INFO_ENABLE);
	if (wh->addr1[0] & BIT(0))
		desc[3] |= cpu_to_le16(RSI_BROADCAST_PKT);
	desc[6] = cpu_to_le16(IEEE80211_SEQ_TO_SN(wh->seq_ctrl));

	if (common->band == NL80211_BAND_2GHZ)
		desc[4] = cpu_to_le16(RSI_11B_MODE);
	else
		desc[4] = cpu_to_le16((RSI_RATE_6 & 0x0f) | RSI_11G_MODE);

	if (conf_is_ht40(conf)) {
		desc[5] = cpu_to_le16(FULL40M_ENABLE);
	}

	if (ieee80211_is_probe_resp(wh->frame_control)) {
		desc[1] |= cpu_to_le16(ADD_DELTA_TSF_VAP_ID |
				       FETCH_RETRY_CNT_FRM_HST);
#define PROBE_RESP_RETRY_CNT	3
		xtend_desc->retry_cnt = PROBE_RESP_RETRY_CNT;
	}

	if ((vif->type == NL80211_IFTYPE_AP) &&
	    (ieee80211_is_action(wh->frame_control))) {
		struct rsi_sta *sta = rsi_find_sta(common, wh->addr1);
		if (sta)
			desc[7] |= cpu_to_le16(sta->sta_id << 8);
		else
			goto err;
	} else
		desc[7] |= cpu_to_le16(vap_id << 8); /* Station ID */
	desc[4] |= cpu_to_le16(vap_id << 14);

	return 0;

err:
	return status;
}

/**
 * rsi_send_data_pkt() - This function sends the received data packet from
 *			 driver to device.
 * @common: Pointer to the driver private structure.
 * @skb: Pointer to the socket buffer structure.
 *
 * Return: status: 0 on success, -1 on failure.
 */
int rsi_send_data_pkt(struct rsi_common *common, struct sk_buff *skb)
{
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_vif *vif = adapter->vifs[0];
	struct ieee80211_hdr *wh = NULL;
	struct ieee80211_tx_info *info;
	struct skb_info *tx_params;
	struct ieee80211_bss_conf *bss = NULL;
	int status = -EINVAL;
	u8 header_size = 0;

	info = IEEE80211_SKB_CB(skb);
	if (!info->control.vif)
		goto err;
	bss = &info->control.vif->bss_conf;
	tx_params = (struct skb_info *)info->driver_data;

	header_size = tx_params->internal_hdr_size;
	wh = (struct ieee80211_hdr *)&skb->data[header_size];

	if (vif->type == NL80211_IFTYPE_STATION) {
		if (!bss->assoc)
			goto err;
		if (!ether_addr_equal(wh->addr1, bss->bssid))
			goto err;
	}

	ven_rsi_dbg(INFO_ZONE, "hal: Sending data pkt");
	rsi_hex_dump(DATA_TX_ZONE, "TX data pkt", skb->data, skb->len);

	status = rsi_send_pkt(common, skb);
	if (status)
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to write data pkt\n", __func__);

err:
	++common->tx_stats.total_tx_pkt_freed[skb->priority];
	rsi_indicate_tx_status(common->priv, skb, status);
	return status;
}

/**
 * rsi_send_mgmt_pkt() - This function prepares sends the given mgmt packet
 *			 to device.
 *
 * @common: Pointer to the driver private structure.
 * @skb: Pointer to the socket buffer structure.
 *
 * Return: status: 0 on success, -1 on failure.
 */
int rsi_send_mgmt_pkt(struct rsi_common *common, struct sk_buff *skb)
{
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_hdr *wh = NULL;
	struct ieee80211_tx_info *info;
	struct skb_info *tx_params;
	u8 header_size = 0;
	int status = -EINVAL;
	struct ieee80211_bss_conf *bss = NULL;
	__le16 *desc = NULL;
	struct xtended_desc *xtend_desc = NULL;

	info = IEEE80211_SKB_CB(skb);
	tx_params = (struct skb_info *)info->driver_data;
	header_size = tx_params->internal_hdr_size;

	if (tx_params->flags & INTERNAL_MGMT_PKT) {
		skb->data[1] |= BIT(7); /* Immediate Wakeup bit*/
		rsi_hex_dump(MGMT_TX_ZONE,
			     "Tx Command Packet",
			     skb->data, skb->len);

		status = rsi_send_pkt(common, skb);
		if (status) {
			ven_rsi_dbg(ERR_ZONE,
				"%s: Failed to write the packet\n",
				__func__);
		}
		dev_kfree_skb(skb);
		return status;
	}

	if (!info->control.vif)
		goto out;
	bss = &info->control.vif->bss_conf;
	wh = (struct ieee80211_hdr *)&skb->data[header_size];

	desc = (__le16 *)skb->data;
	xtend_desc = (struct xtended_desc *)&skb->data[FRAME_DESC_SZ];

	/* Indicate to firmware to give cfm */
	if (ieee80211_is_probe_req(wh->frame_control)) {
		if (!bss->assoc) {
			ven_rsi_dbg(INFO_ZONE,
				"%s: blocking mgmt queue\n", __func__);
			desc[1] |= cpu_to_le16(RSI_DESC_REQUIRE_CFM_TO_HOST);
			xtend_desc->confirm_frame_type = PROBEREQ_CONFIRM;
			common->mgmt_q_block = true;
			ven_rsi_dbg(INFO_ZONE, "Mgmt queue blocked\n");
		} else if (common->bgscan_en) {
			if (common->mac80211_cur_channel !=
			    rsi_get_connected_channel(adapter)) { 
				/* Drop off channel probe request */
				status = 0;
				goto out;
			} else if (wh->addr1[0] == 0xff) {
				/* Drop broadcast probe in connected channel*/
				status = 0;
				goto out;
			}
		}
		ven_rsi_dbg(MGMT_TX_ZONE, "Sending PROBE REQUEST =====>\n");
	}

	ven_rsi_dbg(MGMT_TX_ZONE,
		"Sending Packet : %s =====>\n",
		dot11_pkt_type(wh->frame_control));

	rsi_hex_dump(MGMT_TX_ZONE, "Tx Mgmt Packet", skb->data, skb->len);
	status = rsi_send_pkt(common, skb);

	if (status) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Failed to write the packet\n",
			__func__);
	}

out:
	rsi_indicate_tx_status(common->priv, skb, status);
	return status;
}

int rsi_send_bt_pkt(struct rsi_common *common, struct sk_buff *skb)
{
	struct rsi_hw *adapter = common->priv;
	int status = -EINVAL;
	u8 header_size = 0;
	__le16 *frame_desc;

	header_size = FRAME_DESC_SZ;
	if (header_size > skb_headroom(skb)) {
		ven_rsi_dbg(ERR_ZONE, "%s: Not enough headroom\n", __func__);
		status = -ENOSPC;
		goto err;
	}
	skb_push(skb, header_size);
	frame_desc = (__le16 *)&skb->data[0];
	memset((u8 *)frame_desc, 0, header_size);

	frame_desc[0] = cpu_to_le16(skb->len - FRAME_DESC_SZ);
	frame_desc[0] |= (cpu_to_le16(RSI_BT_DATA_Q) & 0x7) << 12;

	frame_desc[7] = cpu_to_le16(bt_cb(skb)->pkt_type);

	rsi_hex_dump(DATA_TX_ZONE, "TX BT pkt", skb->data, skb->len);
	status = adapter->host_intf_ops->write_pkt(common->priv,
						   skb->data, skb->len);
	if (status)
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to write bt pkt\n", __func__);

err:
	dev_kfree_skb(skb);
	return status;
}

int rsi_send_beacon(struct rsi_common *common)
{
	struct rsi_mac_frame *bcn_frm = NULL;
	u16 bcn_len = common->beacon_frame_len;
	struct sk_buff *skb = NULL;
	struct ieee80211_hw *hw = common->priv->hw;
	struct ieee80211_conf *conf = &hw->conf;
	u8 vap_id = 0;
	u8 dword_align_bytes = 0;
	u8 header_size = 0;
	int status = 0;

	skb = dev_alloc_skb(MAX_MGMT_PKT_SIZE);
	if (!skb)
		return -ENOMEM;

	dword_align_bytes = ((unsigned long)skb->data & 0x3f);
	if (dword_align_bytes) {
		skb_pull(skb, (64 - dword_align_bytes));
	}
	header_size = FRAME_DESC_SZ;
	memset(skb->data, 0, MAX_MGMT_PKT_SIZE);

	common->beacon_cnt++;
	bcn_frm = (struct rsi_mac_frame *)skb->data;
	bcn_frm->desc_word[0] = cpu_to_le16(bcn_len | (RSI_WIFI_DATA_Q << 12));
	bcn_frm->desc_word[1] = 0; // FIXME: Fill type later
	bcn_frm->desc_word[2] = cpu_to_le16((MIN_802_11_HDR_LEN << 8) |
					    dword_align_bytes);
	bcn_frm->desc_word[3] = cpu_to_le16(MAC_BBP_INFO | NO_ACK_IND |
					    BEACON_FRAME | INSERT_TSF |
					    INSERT_SEQ_NO);
	bcn_frm->desc_word[3] |= cpu_to_le16(RATE_INFO_ENABLE);
	bcn_frm->desc_word[4] = cpu_to_le16(vap_id << 14);
	bcn_frm->desc_word[7] = cpu_to_le16(BEACON_HW_Q);
	
	if (conf_is_ht40_plus(conf)) {
		bcn_frm->desc_word[5] = cpu_to_le16(LOWER_20_ENABLE);
		bcn_frm->desc_word[5] |= cpu_to_le16(LOWER_20_ENABLE >> 12);
	} else if (conf_is_ht40_minus(conf)) {
		bcn_frm->desc_word[5] = cpu_to_le16(UPPER_20_ENABLE);
		bcn_frm->desc_word[5] |= cpu_to_le16(UPPER_20_ENABLE >> 12);
	}

	if (common->band == NL80211_BAND_2GHZ)
		bcn_frm->desc_word[4] |= cpu_to_le16(RSI_RATE_1);
	else
		bcn_frm->desc_word[4] |= cpu_to_le16(RSI_RATE_6);

	//if (!(common->beacon_cnt % common->dtim_cnt))
	if (1) //FIXME check this
		bcn_frm->desc_word[3] |= cpu_to_le16(DTIM_BEACON);

	//mutex_lock(&common->mutex);
	memcpy(&skb->data[header_size], common->beacon_frame, bcn_len);
	//mutex_unlock(&common->mutex);

	skb_put(skb, bcn_len + header_size);

	rsi_hex_dump(MGMT_TX_ZONE, "Beacon Frame", skb->data, skb->len);	

	mutex_lock(&common->tx_lock);
	if (rsi_send_pkt(common, skb)) {
		ven_rsi_dbg(ERR_ZONE, "Failed to send Beacon\n");
		status = -EINVAL;
	}
	mutex_unlock(&common->tx_lock);

	dev_kfree_skb(skb);
	return status;
}

/**
 * bl_cmd_timeout() - This function is called when BL command timed out
 * @priv: Pointer to the hardware structure.
 *
 * Return: NONE.
 */
static void bl_cmd_timeout(unsigned long priv)
{
	struct rsi_hw *adapter = (struct rsi_hw *)priv;

	adapter->blcmd_timer_expired = 1;
	del_timer(&adapter->bl_cmd_timer);
}

/**
 * bl_start_cmd_timer() - This function starts the BL command timer
 * @adapter: Pointer to the hardware structure.
 * @timeout: Timeout of the command in milliseconds
 *
 * Return: 0 on success.
 */
static int bl_start_cmd_timer(struct rsi_hw *adapter, u32 timeout)
{
	init_timer(&adapter->bl_cmd_timer);
	adapter->bl_cmd_timer.data = (unsigned long)adapter;
	adapter->bl_cmd_timer.function = (void *)&bl_cmd_timeout;
	adapter->bl_cmd_timer.expires = (msecs_to_jiffies(timeout) + jiffies);

	adapter->blcmd_timer_expired = 0;
	add_timer(&adapter->bl_cmd_timer);

	return 0;
}

/**
 * bl_stop_cmd_timer() - This function stops the BL command timer
 * @adapter: Pointer to the hardware structure.
 *
 * Return: 0 on success.
 */
static int bl_stop_cmd_timer(struct rsi_hw *adapter)
{
	adapter->blcmd_timer_expired = 0;
	if (timer_pending(&adapter->bl_cmd_timer))
		del_timer(&adapter->bl_cmd_timer);

	return 0;
}

/**
 * bl_write_cmd() - This function writes the BL command to device
 * @adapter: Pointer to the hardware structure.
 * @cmd: Command to write
 * @exp_resp: Expected Response
 * @cmd_resp: Received Response
 *
 * Return: 0 on success.
 */
int bl_write_cmd(struct rsi_hw *adapter, u8 cmd, u8 exp_resp, u16 *cmd_resp)
{
	struct rsi_host_intf_ops *hif_ops = adapter->host_intf_ops;
	u32 regin_val = 0, regout_val = 0;
	u8 output = 0;
	u32 regin_input = 0;

	regin_input = (REGIN_INPUT | adapter->priv->coex_mode);

	while (!adapter->blcmd_timer_expired) {
		regin_val = 0;
		if (hif_ops->master_reg_read(adapter,
					     SWBL_REGIN,
					     &regin_val,
					     2) < 0) {
			ven_rsi_dbg(ERR_ZONE,
				"%s: Command %0x REGIN reading failed..\n",
				__func__, cmd);
			goto fail;
		}
		mdelay(1);
		if ((regin_val >> 12) != REGIN_VALID)
			break;
	}
	if (adapter->blcmd_timer_expired) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Command %0x REGIN reading timed out..\n",
			__func__, cmd);
		goto fail;
	}

	ven_rsi_dbg(INFO_ZONE,
		"Issuing write to Regin regin_val:%0x sending cmd:%0x\n",
		regin_val, (cmd | regin_input << 8));
	if ((hif_ops->master_reg_write(adapter,
				       SWBL_REGIN,
				       (cmd | regin_input << 8),
				       2)) < 0) {
		goto fail;
	}
	mdelay(1);

	if (cmd == LOAD_HOSTED_FW || cmd == JUMP_TO_ZERO_PC) {
		/* JUMP_TO_ZERO_PC doesn't expect
		 * any response. So return from here
		 */
		return 0;
	}

	while (!adapter->blcmd_timer_expired) {
		regout_val = 0;
		if (hif_ops->master_reg_read(adapter,
					     SWBL_REGOUT,
					     &regout_val,
					     2) < 0) {
			ven_rsi_dbg(ERR_ZONE,
				"%s: Command %0x REGOUT reading failed..\n",
				__func__, cmd);
			goto fail;
		}
		mdelay(1);
		if ((regout_val >> 8) == REGOUT_VALID)
			break;
	}
	if (adapter->blcmd_timer_expired) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Command %0x REGOUT reading timed out..\n",
			__func__, cmd);
		goto fail;
	}

	*cmd_resp = ((u16 *)&regout_val)[0] & 0xffff;

	output = ((u8 *)&regout_val)[0] & 0xff;

	ven_rsi_dbg(INFO_ZONE, "Invalidating regout\n");
	if ((hif_ops->master_reg_write(adapter,
				       SWBL_REGOUT,
				       (cmd | REGOUT_INVALID << 8),
				       2)) < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Command %0x REGOUT writing failed..\n",
			__func__, cmd);
		goto fail;
	}
	mdelay(1);

	if (output == exp_resp) {
		ven_rsi_dbg(INFO_ZONE,
			"%s: Recvd Expected resp %x for cmd %0x\n",
			__func__, output, cmd);
	} else {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Recvd resp %x for cmd %0x\n",
			__func__, output, cmd);
		goto fail;
	}
	return 0;

fail:
	return -1;
}

/**
 * bl_cmd() - This function initiates the BL command
 * @adapter: Pointer to the hardware structure.
 * @cmd: Command to write
 * @exp_resp: Expected Response
 * @str: Command string
 *
 * Return: 0 on success, -1 on failure.
 */
int bl_cmd(struct rsi_hw *adapter, u8 cmd, u8 exp_resp, char *str)
{
	u16 regout_val = 0;
	u32 timeout = 0;

	ven_rsi_dbg(INFO_ZONE, "Issuing cmd: \"%s\"\n", str);

	if ((cmd == EOF_REACHED) || (cmd == PING_VALID) || (cmd == PONG_VALID))
		timeout = BL_BURN_TIMEOUT;
	else
		timeout = BL_CMD_TIMEOUT;

	bl_start_cmd_timer(adapter, timeout);
	if (bl_write_cmd(adapter, cmd, exp_resp, &regout_val) < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Command %s (%0x) writing failed..\n",
			__func__, str, cmd);
		goto fail;
	}
	bl_stop_cmd_timer(adapter);
	return 0;

fail:
	return -1;
}

/**
 * bl_write_header() - This function writes the BL header
 * @adapter: Pointer to the hardware structure.
 * @flash_content: Flash content
 * @content_size: Flash content size
 *
 * Return: 0 on success, -1 on failure.
 */
static int bl_write_header(struct rsi_hw *adapter,
			   u8 *flash_content, u32 content_size)
{
	struct rsi_host_intf_ops *hif_ops = adapter->host_intf_ops;
	struct bl_header bl_hdr;
	u32 write_addr, write_len;

#define CHECK_SUM_OFFSET 20
#define LEN_OFFSET 8
#define ADDR_OFFSET 16

	bl_hdr.flags = 0;
	bl_hdr.image_no = cpu_to_le32(adapter->priv->coex_mode);
	bl_hdr.check_sum = cpu_to_le32(
				*(u32 *)&flash_content[CHECK_SUM_OFFSET]);
	bl_hdr.flash_start_address = cpu_to_le32(
					*(u32 *)&flash_content[ADDR_OFFSET]);
	bl_hdr.flash_len = cpu_to_le32(*(u32 *)&flash_content[LEN_OFFSET]);
	write_len = sizeof(struct bl_header);

	if (adapter->rsi_host_intf == RSI_HOST_INTF_USB) {
		write_addr = PING_BUFFER_ADDRESS;
		if ((hif_ops->write_reg_multiple(adapter,
						 write_addr,
						 (u8 *)&bl_hdr,
						 write_len)) < 0) {
			ven_rsi_dbg(ERR_ZONE,
				"%s: Failed to load Version/CRC structure\n",
				__func__);
			goto fail;
		}
	} else {
		write_addr = PING_BUFFER_ADDRESS >> 16;
		if ((hif_ops->master_access_msword(adapter, write_addr)) < 0) {
			ven_rsi_dbg(ERR_ZONE,
				"%s: Unable to set ms word to common reg\n",
				__func__);
			goto fail;
		}
		write_addr = SD_REQUEST_MASTER |
			     (PING_BUFFER_ADDRESS & 0xFFFF);
		if ((hif_ops->write_reg_multiple(adapter,
						 write_addr,
						 (u8 *)&bl_hdr,
						 write_len)) < 0) {
			ven_rsi_dbg(ERR_ZONE,
				"%s: Failed to load Version/CRC structure\n",
				__func__);
			goto fail;
		}
	}
	return 0;

fail:
	return -1;
}

/**
 * read_flash_capacity() - This function reads the flash size from device
 * @adapter: Pointer to the hardware structure.
 *
 * Return: flash capacity on success, 0 on failure.
 */
static u32 read_flash_capacity(struct rsi_hw *adapter)
{
	u32 flash_sz = 0;

	if ((adapter->host_intf_ops->master_reg_read(adapter,
						     FLASH_SIZE_ADDR,
						     &flash_sz, 2)) < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Flash size reading failed..\n",
			__func__);
		return 0;
	}
	ven_rsi_dbg(INIT_ZONE, "Flash capacity: %d KiloBytes\n", flash_sz);

	return (flash_sz * 1024); /* Return size in kbytes */
}

/**
 * ping_pong_write() - This function writes the flash contents throgh ping
 *			pong buffers
 * @adapter: Pointer to the hardware structure.
 * @cmd: command ping/pong write
 * @addr: address to write
 * @size: size
 *
 * Return: 0 on success, -1 on failure.
 */
static int ping_pong_write(struct rsi_hw *adapter, u8 cmd, u8 *addr, u32 size)
{
	struct rsi_host_intf_ops *hif_ops = adapter->host_intf_ops;
	u32 block_size = 0;
	u32 cmd_addr;
	u16 cmd_resp = 0, cmd_req = 0;
	u8 *str;

	if (adapter->rsi_host_intf == RSI_HOST_INTF_SDIO)
		block_size = 256;
	else
		block_size = 252;

	if (cmd == PING_WRITE) {
		cmd_addr = PING_BUFFER_ADDRESS;
		cmd_resp = PONG_AVAIL;
		cmd_req = PING_VALID;
		str = "PING_VALID";
	} else {
		cmd_addr = PONG_BUFFER_ADDRESS;
		cmd_resp = PING_AVAIL;
		cmd_req = PONG_VALID;
		str = "PONG_VALID";
	}

	if (hif_ops->load_data_master_write(adapter,
					    cmd_addr,
					    size,
					    block_size,
					    addr)) {
		ven_rsi_dbg(ERR_ZONE, "%s: Unable to write blk at addr %0x\n",
			__func__, *addr);
		goto fail;
	}
	if (bl_cmd(adapter, cmd_req, cmd_resp, str) < 0) {
		bl_stop_cmd_timer(adapter);
		goto fail;
	}
	return 0;

fail:
	return -1;
}

/**
 * auto_fw_upgrade() - This function loads the firmware to device
 * @adapter: Pointer to the hardware structure.
 * @flash_content: Firmware to load
 * @content_size: Size of the firmware
 *
 * Return: 0 on success, -1 on failure.
 */
static int auto_fw_upgrade(struct rsi_hw *adapter,
			   u8 *flash_content,
			   u32 content_size)
{
	u8 cmd;
	u8 *temp_flash_content;
	u32 temp_content_size;
	u32 num_flash;
	u32 index;
	u32 flash_start_address;

	temp_flash_content = flash_content;

	if (content_size > MAX_FLASH_FILE_SIZE) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Flash Content size is more than 400K %u\n",
			__func__, MAX_FLASH_FILE_SIZE);
		goto fail;
	}

	flash_start_address = cpu_to_le32(
				*(u32 *)&flash_content[FLASHING_START_ADDRESS]);
	ven_rsi_dbg(INFO_ZONE, "flash start address: %08x\n", flash_start_address);

	if (flash_start_address < FW_IMAGE_MIN_ADDRESS) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Fw image Flash Start Address is less than 64K\n",
			__func__);
		goto fail;
	}

	if (flash_start_address % FLASH_SECTOR_SIZE) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Flash Start Address is not multiple of 4K\n",
			__func__);
		goto fail;
	}

	if ((flash_start_address + content_size) > adapter->flash_capacity) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Flash Content will cross max flash size\n",
			__func__);
		goto fail;
	}

	temp_content_size  = content_size;
	num_flash = content_size / FLASH_WRITE_CHUNK_SIZE;

	ven_rsi_dbg(INFO_ZONE, "content_size: %d\n", content_size);
	ven_rsi_dbg(INFO_ZONE, "num_flash: %d\n", num_flash);

	for (index = 0; index <= num_flash; index++) {
		ven_rsi_dbg(INFO_ZONE, "flash index: %d\n", index);
		if (index != num_flash) {
			content_size = FLASH_WRITE_CHUNK_SIZE;
			ven_rsi_dbg(INFO_ZONE,
				"QSPI content_size:%d\n",
				content_size);
		} else {
			content_size =
				temp_content_size % FLASH_WRITE_CHUNK_SIZE;
			ven_rsi_dbg(INFO_ZONE,
				"Writing last sector content_size:%d\n",
				content_size);
			if (!content_size) {
				ven_rsi_dbg(INFO_ZONE, "INSTRUCTION SIZE ZERO\n");
				break;
			}
		}

		if (index % 2)
			cmd = PING_WRITE;
		else
			cmd = PONG_WRITE;

		if (ping_pong_write(adapter,
				    cmd,
				    flash_content,
				    content_size)) {
			ven_rsi_dbg(ERR_ZONE,
				"%s: Unable to load %d block\n",
				__func__, index);
			goto fail;
		}

		ven_rsi_dbg(INFO_ZONE,
			"%s: Successfully loaded %d instructions\n",
			__func__, index);
		flash_content += content_size;
	}

	if (bl_cmd(adapter, EOF_REACHED, FW_LOADING_SUCCESSFUL,
		   "EOF_REACHED") < 0) {
		bl_stop_cmd_timer(adapter);
		goto fail;
	}
	ven_rsi_dbg(INFO_ZONE, "FW loading is done and FW is running..\n");
	return 0;

fail:
	return -1;
}

/**
 * read_flash_content() - This function reads the flash content
 *				from device
 * @common: Pointer to the driver private structure.
 *
 * Return: status: 0 on success, -1 on failure.
 */
static int read_flash_content(struct rsi_hw *adapter,
			      u8 *temp_buf,
			      u32 address,
			      u32 len)
{
	struct rsi_host_intf_ops *hif_ops = adapter->host_intf_ops;

	if (adapter->rsi_host_intf == RSI_HOST_INTF_SDIO) {
		if (hif_ops->master_access_msword(adapter,
						  address >> 16) < 0) {
			ven_rsi_dbg(ERR_ZONE,
				"%s: Unable to set ms word to common reg\n",
				__func__);
			return -1;
		}
		address &= 0xFFFF;
		return hif_ops->read_reg_multiple(adapter,
						  address | SD_REQUEST_MASTER,
						  temp_buf, len);
	} else {
		return hif_ops->read_reg_multiple(adapter, address,
						  temp_buf, len);
	}

	return 0;
}

/**
 * verify_flash_content() - This function verifies the loaded flash content
 *				from device
 * @common: Pointer to the driver private structure.
 *
 * Return: status: 0 on success, -1 on failure.
 */
int verify_flash_content(struct rsi_hw *adapter,
			 u8 *flash_content,
			 u32 instructions_sz,
			 u32 eeprom_offset,
			 u8  read_mode)
{
	int status = 0;
	u32 num_loops = 0, idx;
	u32 chunk_size = 0;
	u8 *dest_addr = NULL;
	u32 addr = 0;
	u32 flash_chunk_size;

	if (adapter->rsi_host_intf == RSI_HOST_INTF_USB)
		flash_chunk_size = USB_FLASH_READ_CHUNK_SIZE;
	else
		flash_chunk_size = SDIO_FLASH_READ_CHUNK_SIZE;

	num_loops = instructions_sz / flash_chunk_size;

	if (instructions_sz % flash_chunk_size)
		num_loops++;

	if (read_mode != EEPROM_READ_MODE) {
		dest_addr  = kzalloc(instructions_sz, GFP_KERNEL);
		if (!dest_addr) {
			ven_rsi_dbg(ERR_ZONE,
				"%s: Memory allocation for dest_addr failed\n",
				__func__);
			return -1;
		}
	}

	ven_rsi_dbg(INFO_ZONE, "Number of loops required: %d\n", num_loops);
	for (idx = 0; idx < num_loops; idx++) {
		if (instructions_sz < flash_chunk_size)
			chunk_size = instructions_sz;
		else
			chunk_size = flash_chunk_size;
		ven_rsi_dbg(INFO_ZONE, "idx is %d and chunk size is %d\n",
			idx, chunk_size);
		if (read_mode == EEPROM_READ_MODE) {
			adapter->eeprom.offset = eeprom_offset;
			ven_rsi_dbg(INFO_ZONE,
				"eeprom offset is %x\n", eeprom_offset);
			adapter->eeprom.length = chunk_size;
			status = rsi_flash_read(adapter);
			if (status == 0) {
				ven_rsi_dbg(INFO_ZONE,
					"%s: BLOCK/SECTOR READING SUCCESSFUL\n",
					__func__);
			} else {
				ven_rsi_dbg(ERR_ZONE,
					"%s: READING FROM FLASH FAILED\n",
					__func__);
				return -1;
			}
		} else {
			memset(dest_addr, 0, chunk_size);
			addr = SOC_FLASH_ADDR + eeprom_offset;
			ven_rsi_dbg(INFO_ZONE,
				"Reading flash addr 0x%0x\n", addr);
			if (read_flash_content(adapter, dest_addr, addr,
					       flash_chunk_size) < 0) {
				ven_rsi_dbg(ERR_ZONE,
					"%s:Failed to read calib data\n",
					__func__);
				status = -1;
				goto out;
			}
		}
		if (read_mode == EEPROM_READ_MODE) {
			/* Wait for receive packet */
			mdelay(10);
			dest_addr = adapter->priv->rx_data_pkt;
			if (!dest_addr) {
				ven_rsi_dbg(ERR_ZONE,
					"Failed reading flash content\n");
				status = -1;
				goto out;
			}
		}
		if (memcmp(&flash_content[idx * flash_chunk_size],
			   dest_addr,
			   chunk_size)) {
			ven_rsi_dbg(ERR_ZONE,
				"%s: VERIFICATION OF FLASH CHUNK FAILED\n",
				__func__);
			kfree(dest_addr);
			status = -1;
			goto out;
		}
		eeprom_offset += chunk_size;
		instructions_sz -= chunk_size;
	}

out:
	if (read_mode == MASTER_READ_MODE)
		kfree(dest_addr);
	return 0;
}

/**
 * rsi_load_9113_firmware () - This function loads the TA firmware for 9113
 *				device.
 * @adapter: Pointer to the rsi hw.
 *
 * Return: status: 0 on success, -1 on failure.
 */
int rsi_load_9113_firmware(struct rsi_hw *adapter)
{
	struct rsi_host_intf_ops *hif_ops = adapter->host_intf_ops;
	const struct firmware *fw_entry = NULL;
	u32 regout_val = 0;
	u16 tmp_regout_val = 0;
	u8 *flash_content = NULL;
	u32 content_size = 0;
	struct ta_metadata *metadata_p;

	bl_start_cmd_timer(adapter, BL_CMD_TIMEOUT);

	while (!adapter->blcmd_timer_expired) {
		if ((hif_ops->master_reg_read(adapter,
					      SWBL_REGOUT,
					      &regout_val,
					      2)) < 0) {
			ven_rsi_dbg(ERR_ZONE,
				"%s: REGOUT read failed\n", __func__);
			goto fail;
		}
		mdelay(1);
		if ((regout_val >> 8) == REGOUT_VALID)
			break;
	}
	if (adapter->blcmd_timer_expired) {
		ven_rsi_dbg(ERR_ZONE, "%s: REGOUT read timedout\n", __func__);
		ven_rsi_dbg(ERR_ZONE,
			"%s: Soft boot loader not present\n", __func__);
		goto fail;
	}
	bl_stop_cmd_timer(adapter);

	ven_rsi_dbg(INFO_ZONE, "Received Board Version Number: %x\n",
		(regout_val & 0xff));

	if ((hif_ops->master_reg_write(adapter,
				       SWBL_REGOUT,
				       (REGOUT_INVALID | REGOUT_INVALID << 8),
				       2)) < 0) {
		ven_rsi_dbg(ERR_ZONE, "%s: REGOUT writing failed..\n", __func__);
		goto fail;
	}
	mdelay(1);

	if ((bl_cmd(adapter, CONFIG_AUTO_READ_MODE, CMD_PASS,
		    "AUTO_READ_CMD")) < 0)
		goto fail;

	adapter->flash_capacity = read_flash_capacity(adapter);
	if (adapter->flash_capacity <= 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Unable to read flash size from EEPROM\n",
			__func__);
		goto fail;
	}

	metadata_p = &metadata_flash_content[adapter->priv->coex_mode];

	ven_rsi_dbg(INIT_ZONE, "%s: loading file %s\n", __func__, metadata_p->name);

	if ((request_firmware(&fw_entry, metadata_p->name,
			      adapter->device)) < 0) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to open file %s\n",
			__func__, metadata_p->name);
		goto fail;
	}
	flash_content = kmemdup(fw_entry->data, fw_entry->size, GFP_KERNEL);
	if (!flash_content) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to copy firmware\n", __func__);
		goto fail;
	}
	content_size = fw_entry->size;
	ven_rsi_dbg(INFO_ZONE, "FW Length = %d bytes\n", content_size);

	if (bl_write_header(adapter, flash_content, content_size)) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: RPS Image header loading failed\n",
			__func__);
		goto fail;
	}

	bl_start_cmd_timer(adapter, BL_CMD_TIMEOUT);
	if (bl_write_cmd(adapter, CHECK_CRC, CMD_PASS, &tmp_regout_val) < 0) {
		bl_stop_cmd_timer(adapter);
		ven_rsi_dbg(ERR_ZONE,
			"%s: CHECK_CRC Command writing failed..\n",
			__func__);
		if ((tmp_regout_val & 0xff) == CMD_FAIL) {
			ven_rsi_dbg(ERR_ZONE,
				"CRC Fail.. Proceeding to Upgrade mode\n");
			goto fw_upgrade;
		}
	}
	bl_stop_cmd_timer(adapter);

	if (bl_cmd(adapter, POLLING_MODE, CMD_PASS, "POLLING_MODE") < 0)
		goto fail;

load_image_cmd:
	if ((bl_cmd(adapter,
		    LOAD_HOSTED_FW,
		    LOADING_INITIATED,
		    "LOAD_HOSTED_FW")) < 0)
		goto fail;
	ven_rsi_dbg(INFO_ZONE, "Load Image command passed..\n");
	goto success;

fw_upgrade:
	/* After burning the RPS header, firmware has to be
	 * burned using the below steps
	 */
	if (bl_cmd(adapter, BURN_HOSTED_FW, SEND_RPS_FILE, "FW_UPGRADE") < 0)
		goto fail;

	ven_rsi_dbg(INFO_ZONE, "Burn Command Pass.. Upgrading the firmware\n");

	if (auto_fw_upgrade(adapter, flash_content, content_size) == 0) {
		ven_rsi_dbg(ERR_ZONE, "***** Auto firmware successful *****\n");
		goto load_image_cmd;
	}

	if (bl_cmd(adapter, CONFIG_AUTO_READ_MODE,
		   CMD_PASS, "AUTO_READ_MODE") < 0)
		goto fail;

	/* Not required for current flash mode */
#if 0
	ven_rsi_dbg(INFO_ZONE, "Starting Flash Verification Process\n");

	if ((verify_flash_content(adapter,
				  flash_content,
				  EEPROM_DATA_SIZE,
				  0,
				  EEPROM_READ_MODE)) < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: FLASHING SBL failed in Calib VERIFICATION phase\n",
			__func__);
		goto fail;
	}
	if ((verify_flash_content(adapter,
				  flash_content + BL_HEADER,
				  (content_size - BL_HEADER),
				  EEPROM_DATA_SIZE,
				  MASTER_READ_MODE)) < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s:FLASHING SBL failed in SBL VERIFICATION phase\n",
			__func__);
		goto fail;
	}
	ven_rsi_dbg(INFO_ZONE,
		"Flash Verification Process Completed Successfully\n");
#endif
	ven_rsi_dbg(INFO_ZONE, "SWBL FLASHING THROUGH SWBL PASSED...\n");

success:
	kfree(flash_content);
	release_firmware(fw_entry);
	return 0;

fail:
	kfree(flash_content);
	release_firmware(fw_entry);
	return -1;
}

/**
 * rsi_hal_device_init() - This function initializes the Device
 * @adapter: Pointer to the hardware structure
 *
 * Return: status: 0 on success, -1 on failure.
 */
int rsi_hal_device_init(struct rsi_hw *adapter)
{
#if defined(CONFIG_VEN_RSI_HCI)
	adapter->priv->coex_mode = 2;
#elif defined(CONFIG_VEN_RSI_COEX)
	adapter->priv->coex_mode = 2;
#else
	adapter->priv->coex_mode = 1;
#endif

#ifdef CONFIG_RSI_BT_LE
	adapter->priv->coex_mode = 2;
#endif
	adapter->device_model = RSI_DEV_9113;
	switch (adapter->device_model) {
	case RSI_DEV_9110:
		/* Add code for 9110 */
		break;
	case RSI_DEV_9113:
		if (rsi_load_9113_firmware(adapter)) {
			ven_rsi_dbg(ERR_ZONE,
				"%s: Failed to load TA instructions\n",
				__func__);
			return -1;
		}
		break;
	case RSI_DEV_9116:
		/* Add code for 9116 */
		break;
	default:
		return -1;
	}
	adapter->common_hal_fsm = COMMAN_HAL_WAIT_FOR_CARD_READY;

#if defined(CONFIG_VEN_RSI_HCI) || defined(CONFIG_VEN_RSI_COEX)
	adapter->priv->bt_fsm_state = BT_DEVICE_NOT_READY;
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(rsi_hal_device_init);

