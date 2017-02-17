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

#include <linux/etherdevice.h>
#include <linux/if.h>
#include <linux/version.h>
#include "rsi_debugfs.h"
#include "rsi_mgmt.h"
#include "rsi_common.h"
#include "rsi_ps.h"

extern int g_bgscan_enable;

static const struct ieee80211_channel rsi_2ghz_channels[] = {
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2412,
	  .hw_value = 1 }, /* Channel 1 */
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2417,
	  .hw_value = 2 }, /* Channel 2 */
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2422,
	  .hw_value = 3 }, /* Channel 3 */
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2427,
	  .hw_value = 4 }, /* Channel 4 */
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2432,
	  .hw_value = 5 }, /* Channel 5 */
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2437,
	  .hw_value = 6 }, /* Channel 6 */
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2442,
	  .hw_value = 7 }, /* Channel 7 */
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2447,
	  .hw_value = 8 }, /* Channel 8 */
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2452,
	  .hw_value = 9 }, /* Channel 9 */
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2457,
	  .hw_value = 10 }, /* Channel 10 */
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2462,
	  .hw_value = 11 }, /* Channel 11 */
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2467,
	  .hw_value = 12 }, /* Channel 12 */
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2472,
	  .hw_value = 13 }, /* Channel 13 */
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2484,
	  .hw_value = 14 }, /* Channel 14 */
};

static const struct ieee80211_channel rsi_5ghz_channels[] = {
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5180,
	  .hw_value = 36,  }, /* Channel 36 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5200,
	  .hw_value = 40, }, /* Channel 40 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5220,
	  .hw_value = 44, }, /* Channel 44 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5240,
	  .hw_value = 48, }, /* Channel 48 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5260,
	  .hw_value = 52, }, /* Channel 52 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5280,
	  .hw_value = 56, }, /* Channel 56 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5300,
	  .hw_value = 60, }, /* Channel 60 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5320,
	  .hw_value = 64, }, /* Channel 64 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5500,
	  .hw_value = 100, }, /* Channel 100 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5520,
	  .hw_value = 104, }, /* Channel 104 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5540,
	  .hw_value = 108, }, /* Channel 108 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5560,
	  .hw_value = 112, }, /* Channel 112 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5580,
	  .hw_value = 116, }, /* Channel 116 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5600,
	  .hw_value = 120, }, /* Channel 120 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5620,
	  .hw_value = 124, }, /* Channel 124 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5640,
	  .hw_value = 128, }, /* Channel 128 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5660,
	  .hw_value = 132, }, /* Channel 132 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5680,
	  .hw_value = 136, }, /* Channel 136 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5700,
	  .hw_value = 140, }, /* Channel 140 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5745,
	  .hw_value = 149, }, /* Channel 149 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5765,
	  .hw_value = 153, }, /* Channel 153 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5785,
	  .hw_value = 157, }, /* Channel 157 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5805,
	  .hw_value = 161, }, /* Channel 161 */
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5825,
	  .hw_value = 165, }, /* Channel 165 */
};

struct ieee80211_rate rsi_rates[12] = {
	{ .bitrate = STD_RATE_01  * 5, .hw_value = RSI_RATE_1 },
	{ .bitrate = STD_RATE_02  * 5, .hw_value = RSI_RATE_2 },
	{ .bitrate = STD_RATE_5_5 * 5, .hw_value = RSI_RATE_5_5 },
	{ .bitrate = STD_RATE_11  * 5, .hw_value = RSI_RATE_11 },
	{ .bitrate = STD_RATE_06  * 5, .hw_value = RSI_RATE_6 },
	{ .bitrate = STD_RATE_09  * 5, .hw_value = RSI_RATE_9 },
	{ .bitrate = STD_RATE_12  * 5, .hw_value = RSI_RATE_12 },
	{ .bitrate = STD_RATE_18  * 5, .hw_value = RSI_RATE_18 },
	{ .bitrate = STD_RATE_24  * 5, .hw_value = RSI_RATE_24 },
	{ .bitrate = STD_RATE_36  * 5, .hw_value = RSI_RATE_36 },
	{ .bitrate = STD_RATE_48  * 5, .hw_value = RSI_RATE_48 },
	{ .bitrate = STD_RATE_54  * 5, .hw_value = RSI_RATE_54 },
};

const u16 rsi_mcsrates[8] = {
	RSI_RATE_MCS0, RSI_RATE_MCS1, RSI_RATE_MCS2, RSI_RATE_MCS3,
	RSI_RATE_MCS4, RSI_RATE_MCS5, RSI_RATE_MCS6, RSI_RATE_MCS7
};

/**
 * rsi_is_cipher_wep() -  This function determines if the cipher is WEP or not.
 * @common: Pointer to the driver private structure.
 *
 * Return: If cipher type is WEP, a value of 1 is returned, else 0.
 */

bool rsi_is_cipher_wep(struct rsi_common *common)
{
	if (((common->secinfo.gtk_cipher == WLAN_CIPHER_SUITE_WEP104) ||
	     (common->secinfo.gtk_cipher == WLAN_CIPHER_SUITE_WEP40)) &&
	    (!common->secinfo.ptk_cipher))
		return true;
	else
		return false;
}

/**
 * rsi_register_rates_channels() - This function registers channels and rates.
 * @adapter: Pointer to the adapter structure.
 * @band: Operating band to be set.
 *
 * Return: None.
 */
static void rsi_register_rates_channels(struct rsi_hw *adapter, int band)
{
	struct ieee80211_supported_band *sbands = &adapter->sbands[band];
	void *channels = NULL;

	if (band == NL80211_BAND_2GHZ) {
		channels = kzalloc(sizeof(rsi_2ghz_channels), GFP_KERNEL);
		memcpy(channels,
		       rsi_2ghz_channels,
		       sizeof(rsi_2ghz_channels));
		sbands->band = NL80211_BAND_2GHZ;
		sbands->n_channels = ARRAY_SIZE(rsi_2ghz_channels);
		sbands->bitrates = rsi_rates;
		sbands->n_bitrates = ARRAY_SIZE(rsi_rates);
	} else {
		channels = kzalloc(sizeof(rsi_5ghz_channels), GFP_KERNEL);
		memcpy(channels,
		       rsi_5ghz_channels,
		       sizeof(rsi_5ghz_channels));
		sbands->band = NL80211_BAND_5GHZ;
		sbands->n_channels = ARRAY_SIZE(rsi_5ghz_channels);
		sbands->bitrates = &rsi_rates[4];
		sbands->n_bitrates = ARRAY_SIZE(rsi_rates) - 4;
	}

	sbands->channels = channels;

	memset(&sbands->ht_cap, 0, sizeof(struct ieee80211_sta_ht_cap));
	sbands->ht_cap.ht_supported = true;
	sbands->ht_cap.cap = (IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
			      IEEE80211_HT_CAP_SGI_20 |
			      IEEE80211_HT_CAP_SGI_40);
	sbands->ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_16K;
	sbands->ht_cap.ampdu_density = IEEE80211_HT_MPDU_DENSITY_NONE;
	sbands->ht_cap.mcs.rx_mask[0] = 0xff;
	sbands->ht_cap.mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;
	/* sbands->ht_cap.mcs.rx_highest = 0x82; */
}

static void rsi_set_min_rate(struct ieee80211_hw *hw,
			     struct ieee80211_sta *sta,
			     struct rsi_common *common)
{
	struct ieee80211_vif *vif = common->priv->vifs[0];
	u8 band = hw->conf.chandef.chan->band;
	u8 ii;
	u32 rate_bitmap;
	bool matched = false;

	if (vif->type == NL80211_IFTYPE_AP) {
		common->bitrate_mask[band] = common->fixedrate_mask[band];
		rate_bitmap = common->bitrate_mask[band];
	} else {
		common->bitrate_mask[band] = sta->supp_rates[band];
		rate_bitmap = (common->fixedrate_mask[band] &
			       sta->supp_rates[band]);
	}
	ven_rsi_dbg(INFO_ZONE, "bitrate_mask = %x\n", common->bitrate_mask[band]);
	ven_rsi_dbg(INFO_ZONE, "rate_bitmap = %x\n", rate_bitmap);

	if (rate_bitmap & 0xfff) {
		/* Find out the min rate */
		for (ii = 0; ii < ARRAY_SIZE(rsi_rates); ii++) {
			if (rate_bitmap & BIT(ii)) {
				common->min_rate = rsi_rates[ii].hw_value;
				matched = true;
				break;
			}
		}
	}

	if (vif->type == NL80211_IFTYPE_STATION)
		common->vif_info[0].is_ht = sta->ht_cap.ht_supported;

	if ((common->vif_info[0].is_ht) && (rate_bitmap >> 12)) {
		for (ii = 0; ii < ARRAY_SIZE(rsi_mcsrates); ii++) {
			if ((rate_bitmap >> 12) & BIT(ii)) {
				common->min_rate = rsi_mcsrates[ii];
				matched = true;
				break;
			}
		}
	}

	if (!matched)
		common->min_rate = 0xffff;

	ven_rsi_dbg(INFO_ZONE, "Min Rate = %d\n", common->min_rate);
}

/**
 * ven_rsi_mac80211_detach() - This function is used to de-initialize the
 *			   Mac80211 stack.
 * @adapter: Pointer to the adapter structure.
 *
 * Return: None.
 */
void ven_rsi_mac80211_detach(struct rsi_hw *adapter)
{
	struct ieee80211_hw *hw = adapter->hw;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
	enum nl80211_band band;
#else
	int band;
#endif

	ven_rsi_dbg(INFO_ZONE, "Detach mac80211...\n");

	if (hw) {
		ieee80211_stop_queues(hw);
		ieee80211_unregister_hw(hw);
		ieee80211_free_hw(hw);
		adapter->hw = NULL;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
	for (band = 0; band < NUM_NL80211_BANDS; band++) {
#else
	for (band = 0; band < IEEE80211_NUM_BANDS; band++) {
#endif
		struct ieee80211_supported_band *sband =
					&adapter->sbands[band];

		kfree(sband->channels);
	}

#ifdef CONFIG_VEN_RSI_DEBUGFS
	rsi_remove_dbgfs(adapter);
	kfree(adapter->dfsentry);
#endif
}
EXPORT_SYMBOL_GPL(ven_rsi_mac80211_detach);

/**
 * rsi_indicate_tx_status() - This function indicates the transmit status.
 * @adapter: Pointer to the adapter structure.
 * @skb: Pointer to the socket buffer structure.
 * @status: Status
 *
 * Return: None.
 */
void rsi_indicate_tx_status(struct rsi_hw *adapter,
			    struct sk_buff *skb,
			    int status)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);

	if (!adapter->hw) {
		ven_rsi_dbg(ERR_ZONE, "##### No Hardware #####\n");
		return;
	}

	memset(info->driver_data, 0, IEEE80211_TX_INFO_DRIVER_DATA_SIZE);

	if (!status)
		info->flags |= IEEE80211_TX_STAT_ACK;

	ieee80211_tx_status_irqsafe(adapter->hw, skb);
}

/**
 * rsi_mac80211_tx() - This is the handler that 802.11 module calls for each
 *		       transmitted frame.SKB contains the buffer starting
 *		       from the IEEE 802.11 header.
 * @hw: Pointer to the ieee80211_hw structure.
 * @control: Pointer to the ieee80211_tx_control structure
 * @skb: Pointer to the socket buffer structure.
 *
 * Return: None
 */
static void rsi_mac80211_tx(struct ieee80211_hw *hw,
			    struct ieee80211_tx_control *control,
			    struct sk_buff *skb)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	struct ieee80211_hdr *wlh = (struct ieee80211_hdr *)skb->data;

	if (ieee80211_is_beacon(wlh->frame_control)) {
		ieee80211_free_txskb(common->priv->hw, skb);
		return;
	}

#ifndef CONFIG_VEN_RSI_HCI
	rsi_core_xmit(common, skb);
#else
#ifndef CONFIG_VEN_RSI_COEX
        ieee80211_free_txskb(common->priv->hw, skb); 
#endif
#endif
}

/**
 * rsi_mac80211_start() - This is first handler that 802.11 module calls, since
 *			  the driver init is complete by then, just
 *			  returns success.
 * @hw: Pointer to the ieee80211_hw structure.
 *
 * Return: 0 as success.
 */
static int rsi_mac80211_start(struct ieee80211_hw *hw)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;

	ven_rsi_dbg(ERR_ZONE, "===> Interface UP <===\n");
	mutex_lock(&common->mutex);
	
	common->iface_down = false;
        wiphy_rfkill_start_polling(hw->wiphy);
	rsi_send_rx_filter_frame(common, 0);

	mutex_unlock(&common->mutex);

	return 0;
}

/**
 * rsi_mac80211_stop() - This is the last handler that 802.11 module calls.
 * @hw: Pointer to the ieee80211_hw structure.
 *
 * Return: None.
 */
static void rsi_mac80211_stop(struct ieee80211_hw *hw)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;

	ven_rsi_dbg(ERR_ZONE, "===> Interface DOWN <===\n");

	mutex_lock(&common->mutex);
	
	common->iface_down = true;
	wiphy_rfkill_stop_polling(hw->wiphy);

	/* Block all rx frames */
	rsi_send_rx_filter_frame(common, 0xffff);
	
	mutex_unlock(&common->mutex);
}

/**
 * rsi_mac80211_add_interface() - This function is called when a netdevice
 *				  attached to the hardware is enabled.
 * @hw: Pointer to the ieee80211_hw structure.
 * @vif: Pointer to the ieee80211_vif structure.
 *
 * Return: ret: 0 on success, negative error code on failure.
 */
static int rsi_mac80211_add_interface(struct ieee80211_hw *hw,
				      struct ieee80211_vif *vif)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	enum opmode intf_mode;
	int ret = 0;

	ven_rsi_dbg(INFO_ZONE, "Add Interface Called\n");

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
	vif->driver_flags |= IEEE80211_VIF_SUPPORTS_UAPSD;
#endif

	if ((vif->type != NL80211_IFTYPE_STATION) &&
	    (vif->type != NL80211_IFTYPE_AP))
		return -1;

	mutex_lock(&common->mutex);

	/* Not supporting concurrent mode now */	
	if (adapter->sc_nvifs > 0)
		return -1;
	
	adapter->vifs[adapter->sc_nvifs++] = vif;

	switch (vif->type) {
	case NL80211_IFTYPE_STATION:
		intf_mode = STA_OPMODE;
		break;
	case NL80211_IFTYPE_AP:
		intf_mode = AP_OPMODE;
		break;
	default:
		return -1;
	}
	ret = rsi_set_vap_capabilities(common, intf_mode, VAP_ADD);
	if (ret) {
		ven_rsi_dbg(ERR_ZONE, "Failed to send VAP capabilities\n");
		return ret;
	}

	if (vif->type == NL80211_IFTYPE_AP) {
		int i;

		common->bc_mc_seqno = 1;
		rsi_send_rx_filter_frame(common, DISALLOW_BEACONS);
		common->min_rate = 0xffff;
		//common->bitrate_mask[NL80211_BAND_2GHZ] = 0xfff;
		//common->bitrate_mask[NL80211_BAND_5GHZ] = 0xfff;
		for (i = 0; i < RSI_MAX_ASSOC_STAS; i++)
			common->stations[i].sta = NULL;
	}

	mutex_unlock(&common->mutex);

	return ret;
}

/**
 * rsi_mac80211_remove_interface() - This function notifies driver that an
 *				     interface is going down.
 * @hw: Pointer to the ieee80211_hw structure.
 * @vif: Pointer to the ieee80211_vif structure.
 *
 * Return: None.
 */
static void rsi_mac80211_remove_interface(struct ieee80211_hw *hw,
					  struct ieee80211_vif *vif)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;

	ven_rsi_dbg(INFO_ZONE, "Remove Interface Called\n");
	mutex_lock(&common->mutex);

	adapter->sc_nvifs--;
	if (vif->type == NL80211_IFTYPE_STATION)
		rsi_set_vap_capabilities(common, STA_OPMODE, VAP_DELETE);
	else if (vif->type == NL80211_IFTYPE_AP)
		rsi_set_vap_capabilities(common, AP_OPMODE, VAP_DELETE);

	if (!memcmp(adapter->vifs[0], vif, sizeof(struct ieee80211_vif)))
		adapter->vifs[0] = NULL;
	mutex_unlock(&common->mutex);
}

/**
 * rsi_channel_change() - This function is a performs the checks
 *			  required for changing a channel and sets
 *			  the channel accordingly.
 * @hw: Pointer to the ieee80211_hw structure.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int rsi_channel_change(struct ieee80211_hw *hw)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	int status = -EOPNOTSUPP;
	struct ieee80211_channel *curchan = hw->conf.chandef.chan;
	u16 channel = curchan->hw_value;
	struct ieee80211_bss_conf *bss = NULL;
	struct ieee80211_vif *vif = adapter->vifs[0];

	if (adapter->sc_nvifs <= 0) {
		ven_rsi_dbg(ERR_ZONE, "%s: No virtual interface found\n", __func__);
		return -EINVAL;
	}
	bss = &vif->bss_conf;

	ven_rsi_dbg(INFO_ZONE,
		"%s: Set channel: %d MHz type: %d channel_no %d\n",
		__func__, curchan->center_freq,
		curchan->flags, channel);

	if (vif->type == NL80211_IFTYPE_AP) {
		ven_rsi_dbg(INFO_ZONE, "Configure channel %d for AP\n", channel);
		if (rsi_band_check(common)) {
			ven_rsi_dbg(ERR_ZONE, "Failed to set band\n");
			return -EINVAL;
		}
		if (rsi_set_channel(common, curchan)) {
			ven_rsi_dbg(ERR_ZONE, "Failed to set the channel\n");
			return -EINVAL;
		}
		common->ap_channel = curchan;
		return 0;
	}
	common->mac80211_cur_channel = channel;
	if (bss->assoc) {
		ven_rsi_dbg(INFO_ZONE, "%s: connected\n", __func__);

		if (common->bgscan_en)
			return 0;

		if (!common->hw_data_qs_blocked &&
		    (rsi_get_connected_channel(adapter) != channel)) {
			ven_rsi_dbg(INFO_ZONE, "blk data q %d\n", channel);
			if (!rsi_send_block_unblock_frame(common, true))
				common->hw_data_qs_blocked = true;
		}
	} else {
		ven_rsi_dbg(INFO_ZONE, "assoc status:%d channel:%d\n",
			bss->assoc, channel);
	}

	status = rsi_band_check(common);
	if (!status)
		status = rsi_set_channel(adapter->priv, curchan);

	if (bss->assoc) {
		if (common->hw_data_qs_blocked &&
		    (rsi_get_connected_channel(adapter) == channel)) {
			ven_rsi_dbg(INFO_ZONE, "unblk data q %d\n", channel);
			if (!rsi_send_block_unblock_frame(common, false))
				common->hw_data_qs_blocked = false;
		}
	} else {
		if (common->hw_data_qs_blocked) {
			ven_rsi_dbg(INFO_ZONE, "unblk data q %d\n", channel);
			if (!rsi_send_block_unblock_frame(common, false))
				common->hw_data_qs_blocked = false;
		}
	}

	return status;
}

/**
 * rsi_config_power() - This function configures tx power in device
 * @hw: Pointer to the ieee80211_hw structure.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int rsi_config_power(struct ieee80211_hw *hw)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	struct ieee80211_conf *conf = &hw->conf;
	int status;

	if (adapter->sc_nvifs <= 0) {
		ven_rsi_dbg(ERR_ZONE, "%s: No virtual interface found\n", __func__);
		return -EINVAL;
	}

	ven_rsi_dbg(INFO_ZONE,
		"%s: Set tx power: %d dBM\n", __func__, conf->power_level);

	if (conf->power_level == common->tx_power)
		return 0;

	common->tx_power = conf->power_level;

	status = rsi_send_radio_params_update(common);

	return status;
}

/**
 * rsi_mac80211_config() - This function is a handler for configuration
 *			   requests. The stack calls this function to
 *			   change hardware configuration, e.g., channel.
 * @hw: Pointer to the ieee80211_hw structure.
 * @changed: Changed flags set.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int rsi_mac80211_config(struct ieee80211_hw *hw,
			       u32 changed)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	struct ieee80211_vif *vif = adapter->vifs[0];
	struct ieee80211_conf *conf = &hw->conf;
	int status = -EOPNOTSUPP;

	mutex_lock(&common->mutex);

	/* channel */
	if (changed & IEEE80211_CONF_CHANGE_CHANNEL)
		status = rsi_channel_change(hw);

	/* listen interval */
	if (changed & IEEE80211_CONF_CHANGE_LISTEN_INTERVAL) {
		ven_rsi_dbg(INFO_ZONE,
			"listen_int = %d\n", conf->listen_interval);
		adapter->ps_info.num_bcns_per_lis_int = conf->listen_interval;
	}

	/* tx power */
	if (changed & IEEE80211_CONF_CHANGE_POWER) {
		ven_rsi_dbg(INFO_ZONE, "%s: Configuring Power\n", __func__);
		status = rsi_config_power(hw);
	}

	/* retry limit */
	if (changed & IEEE80211_CONF_CHANGE_RETRY_LIMITS) {
		/* FIXME */
	}

	/* Power save parameters */
	if ((changed & IEEE80211_CONF_CHANGE_PS) &&
	    (vif->type == NL80211_IFTYPE_STATION)) {
		unsigned long flags;

		spin_lock_irqsave(&adapter->ps_lock, flags);
		if (conf->flags & IEEE80211_CONF_PS)
			rsi_enable_ps(adapter);
		else
			rsi_disable_ps(adapter);
		spin_unlock_irqrestore(&adapter->ps_lock, flags);
	}

	/* RTS threshold */
	if (changed & WIPHY_PARAM_RTS_THRESHOLD) {
		ven_rsi_dbg(INFO_ZONE,"RTS threshold\n");
		if ((common->rts_threshold) <= IEEE80211_MAX_RTS_THRESHOLD) {
			ven_rsi_dbg(INFO_ZONE,
				"%s: Sending vap updates....\n", __func__);
			status = rsi_send_vap_dynamic_update(common);
		}
	}

	mutex_unlock(&common->mutex);

	return status;
}

/**
 * rsi_get_connected_channel() - This function is used to get the current
 *				 connected channel number.
 * @adapter: Pointer to the adapter structure.
 *
 * Return: Current connected AP's channel number is returned.
 */
u16 rsi_get_connected_channel(struct rsi_hw *adapter)
{
	struct ieee80211_vif *vif = adapter->vifs[0];

	if (vif) {
		struct ieee80211_bss_conf *bss = &vif->bss_conf;
		struct ieee80211_channel *channel = bss->chandef.chan;

		return channel->hw_value;
	}

	return 0;
}

void rsi_resume_conn_channel(struct rsi_hw *adapter)
{
	struct rsi_common *common = adapter->priv;
	struct ieee80211_bss_conf *bss = &adapter->vifs[0]->bss_conf;

	mutex_lock(&common->mutex);
	if (bss->assoc) {
		struct ieee80211_channel *channel = bss->chandef.chan;
	 
#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 6, 0))
		if (channel->band == NL80211_BAND_5GHZ)
#else
		if (channel->band == IEEE80211_BAND_5GHZ)
#endif
		       	rsi_program_bb_rf(common);

		rsi_set_channel(common, channel);
	}
	mutex_unlock(&common->mutex);
}

static void rsi_update_beacon(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	struct ieee80211_mutable_offsets offs = {};
	struct sk_buff *bcn;

	bcn = ieee80211_beacon_get_template(hw, vif, &offs);
	if (!bcn) {
		ven_rsi_dbg(ERR_ZONE,
			"failed to get beacon template from mac80211\n");
		return;
	}
	memcpy(common->beacon_frame, bcn->data, bcn->len);
	common->beacon_frame_len = bcn->len;

	rsi_hex_dump(INFO_ZONE, "mac80211: Beacon",
		     common->beacon_frame, common->beacon_frame_len);
}

/**
 * rsi_mac80211_bss_info_changed() - This function is a handler for config
 *				     requests related to BSS parameters that
 *				     may vary during BSS's lifespan.
 * @hw: Pointer to the ieee80211_hw structure.
 * @vif: Pointer to the ieee80211_vif structure.
 * @bss_conf: Pointer to the ieee80211_bss_conf structure.
 * @changed: Changed flags set.
 *
 * Return: None.
 */
static void rsi_mac80211_bss_info_changed(struct ieee80211_hw *hw,
					  struct ieee80211_vif *vif,
					  struct ieee80211_bss_conf *bss_conf,
					  u32 changed)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	struct ieee80211_bss_conf *bss = &adapter->vifs[0]->bss_conf;
	u16 rx_filter_word = 0;

	ven_rsi_dbg(INFO_ZONE, "%s: BSS status changed\n", __func__);

	mutex_lock(&common->mutex);

	if ((changed & BSS_CHANGED_ASSOC) &&
	    (vif->type == NL80211_IFTYPE_STATION)) {
		ven_rsi_dbg(INFO_ZONE, "%s: Changed Association status: %d\n",
			__func__, bss_conf->assoc);
		bss->assoc = bss_conf->assoc;
		if (bss->assoc) {
			/* Send the RX filter frame */
			rx_filter_word = (ALLOW_DATA_ASSOC_PEER |
					  ALLOW_CTRL_ASSOC_PEER |
					  ALLOW_MGMT_ASSOC_PEER |
#ifdef RSI_HW_CONN_MONITOR
					  //DISALLOW_BEACONS |
#endif
					0);
			rsi_send_rx_filter_frame(common, rx_filter_word);
		}
		ven_rsi_dbg(INFO_ZONE,
			"assoc_status=%d, qos=%d, aid=%d\n",
			bss->assoc, bss->qos, bss->aid);
		ven_rsi_dbg(INFO_ZONE,
				"bssid=%02x:%02x:%02x:%02x:%02x:%02x",
				bss->bssid[0], bss->bssid[1], bss->bssid[2],
				bss->bssid[3], bss->bssid[4], bss->bssid[5]);

		/* Send peer notify to device */
		ven_rsi_dbg(INFO_ZONE, "Indicate bss status to device\n");
		rsi_inform_bss_status(common, STA_OPMODE, bss->assoc,
				      bss->bssid, bss->qos, bss->aid, NULL, 0);

		adapter->ps_info.listen_interval =
			bss->beacon_int * adapter->ps_info.num_bcns_per_lis_int;
		adapter->ps_info.deep_sleep_wakeup_period = bss->beacon_int;

		/* If UAPSD is updated send ps params */
		if (common->uapsd_bitmap) {
			ven_rsi_dbg(INFO_ZONE, "Configuring UAPSD\n");
			rsi_conf_uapsd(adapter);
		}
	}

	if ((vif->type == NL80211_IFTYPE_STATION) &&
	    changed & BSS_CHANGED_CQM) {
		ven_rsi_dbg(INFO_ZONE, "%s: Changed CQM\n", __func__);
		common->cqm_info.last_cqm_event_rssi = 0;
		common->cqm_info.rssi_thold = bss_conf->cqm_rssi_thold;
		common->cqm_info.rssi_hyst = bss_conf->cqm_rssi_hyst;
		ven_rsi_dbg(INFO_ZONE, "RSSI throld & hysteresis are: %d %d\n",
			common->cqm_info.rssi_thold,
			common->cqm_info.rssi_hyst);
	}

	if (changed & BSS_CHANGED_TXPOWER) {
		ven_rsi_dbg(INFO_ZONE, "%s: Changed TX power: %d\n",
			__func__, bss_conf->txpower);
	}

	if (changed & BSS_CHANGED_BEACON_INT) {
		ven_rsi_dbg(INFO_ZONE, "%s: Changed Beacon interval: %d\n",
			__func__, bss_conf->beacon_int);
		common->beacon_interval = bss->beacon_int; 
		adapter->ps_info.listen_interval =
			bss->beacon_int * adapter->ps_info.num_bcns_per_lis_int;
	}

	if ((changed & BSS_CHANGED_BEACON) &&
	    (vif->type == NL80211_IFTYPE_AP)) {
		ven_rsi_dbg(INFO_ZONE, "%s: Changed Beacon\n", __func__);
		rsi_update_beacon(hw, vif);
	}

	if ((changed & BSS_CHANGED_BEACON_ENABLED) &&
	    (vif->type == NL80211_IFTYPE_AP)) {
		if (bss->enable_beacon) {
			ven_rsi_dbg(INFO_ZONE, "===> BEACON ENABLED <===\n");
			common->beacon_enabled = 1;
#ifdef CONFIG_CARACALLA_BOARD
			rsi_init_bcn_timer(common);
#endif
		} else {
			ven_rsi_dbg(INFO_ZONE, "===> BEACON DISABLED <===\n");
			common->beacon_enabled = 0;
#ifdef CONFIG_CARACALLA_BOARD
			rsi_del_bcn_timer(common);
#endif
		}
	}

	mutex_unlock(&common->mutex);
}

/**
 * rsi_mac80211_conf_filter() - This function configure the device's RX filter.
 * @hw: Pointer to the ieee80211_hw structure.
 * @changed: Changed flags set.
 * @total_flags: Total initial flags set.
 * @multicast: Multicast.
 *
 * Return: None.
 */
static void rsi_mac80211_conf_filter(struct ieee80211_hw *hw,
				     u32 changed_flags,
				     u32 *total_flags,
				     u64 multicast)
{
#if 0
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	u16 rx_filter_word = 0;
#endif

	/* Not doing much here as of now */
	*total_flags &= RSI_SUPP_FILTERS;

//	rsi_send_rx_filter_frame(common, rx_filter_word);
}

/**
 * rsi_mac80211_conf_tx() - This function configures TX queue parameters
 *			    (EDCF (aifs, cw_min, cw_max), bursting)
 *			    for a hardware TX queue.
 * @hw: Pointer to the ieee80211_hw structure
 * @vif: Pointer to the ieee80211_vif structure.
 * @queue: Queue number.
 * @params: Pointer to ieee80211_tx_queue_params structure.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int rsi_mac80211_conf_tx(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif, u16 queue,
				const struct ieee80211_tx_queue_params *params)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	u8 idx = 0;

	if (queue >= IEEE80211_NUM_ACS)
		return 0;

	ven_rsi_dbg(INFO_ZONE,
		"[Conf] queue:%d, aifs:%d, cwmin:%d cwmax:%d, txop:%d uapsd:%d\n",
		queue, params->aifs, params->cw_min, params->cw_max,
		params->txop, params->uapsd);

	mutex_lock(&common->mutex);
	/* Map into the way the f/w expects */
	switch (queue) {
	case IEEE80211_AC_VO:
		idx = VO_Q;
		break;
	case IEEE80211_AC_VI:
		idx = VI_Q;
		break;
	case IEEE80211_AC_BE:
		idx = BE_Q;
		break;
	case IEEE80211_AC_BK:
		idx = BK_Q;
		break;
	default:
		idx = BE_Q;
		break;
	}

	memcpy(&common->edca_params[idx],
	       params,
	       sizeof(struct ieee80211_tx_queue_params));

	if (params->uapsd)
		common->uapsd_bitmap |= idx;

	mutex_unlock(&common->mutex);
	return 0;
}

/**
 * rsi_hal_key_config() - This function loads the keys into the firmware.
 * @hw: Pointer to the ieee80211_hw structure.
 * @vif: Pointer to the ieee80211_vif structure.
 * @key: Pointer to the ieee80211_key_conf structure.
 *
 * Return: status: 0 on success, -1 on failure.
 */
static int rsi_hal_key_config(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif,
			      struct ieee80211_key_conf *key,
			      struct ieee80211_sta *sta)
{
	struct rsi_hw *adapter = hw->priv;
	int status;
	u8 key_type;
	s16 sta_id;

	if (key->flags & IEEE80211_KEY_FLAG_PAIRWISE)
		key_type = RSI_PAIRWISE_KEY;
	else
		key_type = RSI_GROUP_KEY;

	ven_rsi_dbg(ERR_ZONE, "%s: Cipher 0x%x key_type: %d key_len: %d\n",
		__func__, key->cipher, key_type, key->keylen);
	ven_rsi_dbg(INFO_ZONE, "hw_key_idx %d\n", key->hw_key_idx);

	if (sta && vif->type == NL80211_IFTYPE_AP) {
		struct rsi_sta *rsta = rsi_find_sta(adapter->priv, sta->addr);

		if (rsta)
			sta_id = rsta->sta_id;
		else
			return -EINVAL;
	} else
		sta_id = 0;

	if ((key->cipher == WLAN_CIPHER_SUITE_WEP104) ||
	    (key->cipher == WLAN_CIPHER_SUITE_WEP40)) {
		status = rsi_load_key(adapter->priv,
				      key->key,
				      key->keylen,
				      RSI_PAIRWISE_KEY,
				      key->keyidx,
				      key->cipher,
				      sta_id);
		if (status)
			return status;
	}
	return rsi_load_key(adapter->priv,
				key->key,
				key->keylen,
				key_type,
				key->keyidx,
				key->cipher,
				sta_id);
}

/**
 * rsi_mac80211_set_key() - This function sets type of key to be loaded.
 * @hw: Pointer to the ieee80211_hw structure.
 * @cmd: enum set_key_cmd.
 * @vif: Pointer to the ieee80211_vif structure.
 * @sta: Pointer to the ieee80211_sta structure.
 * @key: Pointer to the ieee80211_key_conf structure.
 *
 * Return: status: 0 on success, negative error code on failure.
 */
static int rsi_mac80211_set_key(struct ieee80211_hw *hw,
				enum set_key_cmd cmd,
				struct ieee80211_vif *vif,
				struct ieee80211_sta *sta,
				struct ieee80211_key_conf *key)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	struct security_info *secinfo = &common->secinfo;
	int status;

	mutex_lock(&common->mutex);
	switch (cmd) {
	case SET_KEY:
		secinfo->security_enable = true;
		status = rsi_hal_key_config(hw, vif, key, sta);
		if (status) {
			mutex_unlock(&common->mutex);
			return status;
		}

		if (key->flags & IEEE80211_KEY_FLAG_PAIRWISE)
			secinfo->ptk_cipher = key->cipher;
		else
			secinfo->gtk_cipher = key->cipher;

		key->hw_key_idx = key->keyidx;
		key->flags |= IEEE80211_KEY_FLAG_GENERATE_IV;

		ven_rsi_dbg(ERR_ZONE, "%s: RSI set_key\n", __func__);
		break;

	case DISABLE_KEY:
		secinfo->security_enable = false;
		ven_rsi_dbg(ERR_ZONE, "%s: RSI del key\n", __func__);
		memset(key, 0, sizeof(struct ieee80211_key_conf));
		status = rsi_hal_key_config(hw, vif, key, sta);
		break;

	default:
		status = -EOPNOTSUPP;
		break;
	}

	mutex_unlock(&common->mutex);
	return status;
}

/**
 * rsi_mac80211_ampdu_action() - This function selects the AMPDU action for
 *				 the corresponding mlme_action flag and
 *				 informs the f/w regarding this.
 * @hw: Pointer to the ieee80211_hw structure.
 * @vif: Pointer to the ieee80211_vif structure.
 * @action: ieee80211_ampdu_mlme_action enum.
 * @sta: Pointer to the ieee80211_sta structure.
 * @tid: Traffic identifier.
 * @ssn: Pointer to ssn value.
 * @buf_size: Buffer size (for kernel version > 2.6.38).
 *
 * Return: status: 0 on success, negative error code on failure.
 */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
static int rsi_mac80211_ampdu_action(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif,
				     enum ieee80211_ampdu_mlme_action action,
				     struct ieee80211_sta *sta,
				     u16 tid,
				     u16 *ssn,
				     u8 buf_size)
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0))
static int rsi_mac80211_ampdu_action(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif,
				     enum ieee80211_ampdu_mlme_action action,
				     struct ieee80211_sta *sta,
				     u16 tid,
				     u16 *ssn,
				     u8 buf_size,
				     bool amsdu)
#else
static int rsi_mac80211_ampdu_action(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif,
				     struct ieee80211_ampdu_params *params)
#endif
{
	int status = 1;
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	u16 seq_no = 0;
	u8 ii = 0;
	u8 sta_id = 0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0))
	u16 tid = params->tid;
	u8 buf_size = params->buf_size;
	enum ieee80211_ampdu_mlme_action action = params->action;
	struct ieee80211_sta *sta = params->sta;
#endif

	for (ii = 0; ii < RSI_MAX_VIFS; ii++) {
		if (vif == adapter->vifs[ii])
			break;
	}

	mutex_lock(&common->mutex);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0))
	if (ssn != NULL)
		seq_no = *ssn;
#else
	seq_no = params->ssn;
#endif
	if (vif->type == NL80211_IFTYPE_AP) {
		struct rsi_sta *rsta = rsi_find_sta(common, sta->addr);

		if (!rsta) {
			ven_rsi_dbg(ERR_ZONE, "No station mapped\n");
			return 0;
		}
		sta_id = rsta->sta_id;
	}

	ven_rsi_dbg(INFO_ZONE,
		"%s: AMPDU action tid=%d ssn=0x%x, buf_size=%d\n",
		__func__, tid, seq_no, buf_size);

	switch (action) {
	case IEEE80211_AMPDU_RX_START:
		ven_rsi_dbg(INFO_ZONE, "AMPDU action RX_START (%d)\n", action);
		status = rsi_send_aggr_params_frame(common,
						    tid,
						    seq_no,
						    buf_size,
						    STA_RX_ADDBA_DONE,
						    sta_id);
		break;

	case IEEE80211_AMPDU_RX_STOP:
		ven_rsi_dbg(INFO_ZONE,
			"AMPDU action RX_STOP (%d) called\n", action);
		status = rsi_send_aggr_params_frame(common,
						    tid,
						    0,
						    buf_size,
						    STA_RX_DELBA,
						    sta_id);
		break;

	case IEEE80211_AMPDU_TX_START:
		ven_rsi_dbg(INFO_ZONE,
			"AMPDU action TX_START (%d) called\n", action);
		common->vif_info[ii].seq_start = seq_no;
		ieee80211_start_tx_ba_cb_irqsafe(vif, sta->addr, tid);
		status = 0;
		break;

	case IEEE80211_AMPDU_TX_STOP_CONT:
	case IEEE80211_AMPDU_TX_STOP_FLUSH:
	case IEEE80211_AMPDU_TX_STOP_FLUSH_CONT:
		ven_rsi_dbg(INFO_ZONE,
			"AMPDU action TX_STOP_CONT / TX_STOP_FLUSH /"
			" TX_STOP_FLUSH_CONT (%d) called\n", action);
		status = rsi_send_aggr_params_frame(common,
						    tid,
						    seq_no,
						    buf_size,
						    STA_TX_DELBA,
						    sta_id);
		if (!status)
			ieee80211_stop_tx_ba_cb_irqsafe(vif, sta->addr, tid);
		break;

	case IEEE80211_AMPDU_TX_OPERATIONAL:
		ven_rsi_dbg(INFO_ZONE,
			"AMPDU action TX_OPERATIONAL(%d) called\n",
			action);
		status = rsi_send_aggr_params_frame(common,
						tid,
						common->vif_info[ii].seq_start,
						buf_size,
						STA_TX_ADDBA_DONE,
						sta_id);
		break;

	default:
		ven_rsi_dbg(ERR_ZONE, "%s: Uknown AMPDU action\n", __func__);
		break;
	}

	mutex_unlock(&common->mutex);
	return status;
}

/**
 * rsi_mac80211_set_rts_threshold() - This function sets rts threshold value.
 * @hw: Pointer to the ieee80211_hw structure.
 * @value: Rts threshold value.
 *
 * Return: 0 on success.
 */
static int rsi_mac80211_set_rts_threshold(struct ieee80211_hw *hw,
					  u32 value)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;

	mutex_lock(&common->mutex);
	common->rts_threshold = value;
	mutex_unlock(&common->mutex);

	return 0;
}

/**
 * rsi_mac80211_set_rate_mask() - This function sets bitrate_mask to be used.
 * @hw: Pointer to the ieee80211_hw structure
 * @vif: Pointer to the ieee80211_vif structure.
 * @mask: Pointer to the cfg80211_bitrate_mask structure.
 *
 * Return: 0 on success.
 */
static int rsi_mac80211_set_rate_mask(struct ieee80211_hw *hw,
				      struct ieee80211_vif *vif,
				      const struct cfg80211_bitrate_mask *mask)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	enum nl80211_band band = hw->conf.chandef.chan->band;

	mutex_lock(&common->mutex);
	common->fixedrate_mask[band] = 0;

	if (mask->control[band].legacy == 0xfff) {
		common->fixedrate_mask[band] =
			(mask->control[band].ht_mcs[0] << 12);
	} else {
		common->fixedrate_mask[band] = mask->control[band].legacy;
	}
	mutex_unlock(&common->mutex);

	return 0;
}

/**
 * rsi_perform_cqm() - This function performs cqm.
 * @common: Pointer to the driver private structure.
 * @bssid: pointer to the bssid.
 * @rssi: RSSI value.
 */
static void rsi_perform_cqm(struct rsi_common *common,
			    u8 *bssid,
			    s8 rssi)
{
	struct rsi_hw *adapter = common->priv;
	s8 last_event = common->cqm_info.last_cqm_event_rssi;
	int thold = common->cqm_info.rssi_thold;
	u32 hyst = common->cqm_info.rssi_hyst;
	enum nl80211_cqm_rssi_threshold_event event;

	if (rssi < thold && (last_event == 0 || rssi < (last_event - hyst)))
		event = NL80211_CQM_RSSI_THRESHOLD_EVENT_LOW;
	else if (rssi > thold &&
		 (last_event == 0 || rssi > (last_event + hyst)))
		event = NL80211_CQM_RSSI_THRESHOLD_EVENT_HIGH;
	else
		return;

	common->cqm_info.last_cqm_event_rssi = rssi;
	ven_rsi_dbg(INFO_ZONE, "CQM: Notifying event: %d\n", event);
	ieee80211_cqm_rssi_notify(adapter->vifs[0], event, GFP_KERNEL);
}

void rsi_indicate_bcnmiss(struct rsi_common *common)
{
	struct rsi_hw *adapter = common->priv;

	ven_rsi_dbg(INFO_ZONE, "CQM: Notifying beacon miss\n" );
	ieee80211_beacon_loss(adapter->vifs[0]);
	return;
}

/**
 * rsi_fill_rx_status() - This function fills rx status in
 *			  ieee80211_rx_status structure.
 * @hw: Pointer to the ieee80211_hw structure.
 * @skb: Pointer to the socket buffer structure.
 * @common: Pointer to the driver private structure.
 * @rxs: Pointer to the ieee80211_rx_status structure.
 *
 * Return: None.
 */
static void rsi_fill_rx_status(struct ieee80211_hw *hw,
			       struct sk_buff *skb,
			       struct rsi_common *common,
			       struct ieee80211_rx_status *rxs)
{
	struct ieee80211_vif *vif = common->priv->vifs[0];
	struct ieee80211_bss_conf *bss = &vif->bss_conf;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct skb_info *rx_params = (struct skb_info *)info->driver_data;
	struct ieee80211_hdr *hdr;
	char rssi = rx_params->rssi;
	u8 hdrlen = 0;
	u8 channel = rx_params->channel;
	s32 freq;

	hdr = ((struct ieee80211_hdr *)(skb->data));
	hdrlen = ieee80211_hdrlen(hdr->frame_control);

	memset(info, 0, sizeof(struct ieee80211_tx_info));

	rxs->signal = -(rssi);

	rxs->band = common->band;

	freq = ieee80211_channel_to_frequency(channel, rxs->band);

	if (freq)
		rxs->freq = freq;

	if (ieee80211_has_protected(hdr->frame_control)) {
		if (rsi_is_cipher_wep(common)) {
			memmove(skb->data + 4, skb->data, hdrlen);
			skb_pull(skb, 4);
		} else {
			memmove(skb->data + 8, skb->data, hdrlen);
			skb_pull(skb, 8);
			rxs->flag |= RX_FLAG_MMIC_STRIPPED;
		}
		rxs->flag |= RX_FLAG_DECRYPTED;
		rxs->flag |= RX_FLAG_IV_STRIPPED;
	}

	/* CQM only for connected AP beacons, the RSSI is a weighted avg */
	if ((vif->type == NL80211_IFTYPE_STATION) && bss->assoc &&
	    ether_addr_equal(bss->bssid, hdr->addr2)) {
		if (ieee80211_is_beacon(hdr->frame_control))
			rsi_perform_cqm(common, hdr->addr2, rxs->signal);
	}
}

/**
 * rsi_indicate_pkt_to_os() - This function sends received packet to mac80211.
 * @common: Pointer to the driver private structure.
 * @skb: Pointer to the socket buffer structure.
 *
 * Return: None.
 */
void rsi_indicate_pkt_to_os(struct rsi_common *common,
			    struct sk_buff *skb)
{
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_hw *hw = adapter->hw;
	struct ieee80211_rx_status *rx_status = IEEE80211_SKB_RXCB(skb);

	if ((common->iface_down) || (!adapter->sc_nvifs)) {
		dev_kfree_skb(skb);
		return;
	}

	/* filling in the ieee80211_rx_status flags */
	rsi_fill_rx_status(hw, skb, common, rx_status);

	ven_rsi_dbg(INFO_ZONE, "RX Packet Type: %s\n",
		dot11_pkt_type(skb->data[0]));
	rsi_hex_dump(DATA_RX_ZONE, "802.11 RX packet", skb->data, skb->len);
	ieee80211_rx_irqsafe(hw, skb);
}

/**
 * rsi_mac80211_sta_add() - This function notifies driver about a peer getting
 *			    connected.
 * @hw: pointer to the ieee80211_hw structure.
 * @vif: Pointer to the ieee80211_vif structure.
 * @sta: Pointer to the ieee80211_sta structure.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_mac80211_sta_add(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif,
				struct ieee80211_sta *sta)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	bool sta_exist = 0;

	rsi_hex_dump(INFO_ZONE, "Station Add: ", sta->addr, ETH_ALEN);

	mutex_lock(&common->mutex);

	if (vif->type == NL80211_IFTYPE_AP) {
		u8 i, j;
		int free_index = -1;

		/* Check if max stations reached */
		if (common->num_stations >= RSI_MAX_ASSOC_STAS) {
			ven_rsi_dbg(ERR_ZONE, "Reject: Max Stations exists\n");
			return -EINVAL;
		}

		/* Send peer notify to device */
		ven_rsi_dbg(INFO_ZONE, "Indicate bss status to device\n");
		for (i = 0; i < RSI_MAX_ASSOC_STAS; i++) {
			if (!common->stations[i].sta) {
				if (free_index < 0)
					free_index = i;
				continue; 
			}
			if (!memcmp(common->stations[i].sta->addr,
				    sta->addr, ETH_ALEN)) {
				ven_rsi_dbg(INFO_ZONE, "Station exists\n");
				sta_exist = 1;
				break;
			}
		}
		if (!sta_exist) {
			ven_rsi_dbg(INFO_ZONE, "New Station\n");
			if (free_index >= 0)
				i = free_index;
			common->stations[i].sta = sta;
			common->stations[i].sta_id = i;
			rsi_inform_bss_status(common, AP_OPMODE, 1, sta->addr,
					      sta->wme, sta->aid, sta, i);
			for (j = 0; j < IEEE80211_NUM_ACS; j++)
				common->stations[i].seq_no[j] = 1;
			common->num_stations++;
		} else {
			common->stations[i].sta = sta;
			common->stations[i].sta_id = i;
			for (j = 0; j < IEEE80211_NUM_ACS; j++)
				common->stations[i].seq_no[j] = 1;
		}
	}

	if ((sta->ht_cap.cap & IEEE80211_HT_CAP_SGI_20) ||
	    (sta->ht_cap.cap & IEEE80211_HT_CAP_SGI_40)) {
		common->vif_info[0].sgi = true;
	}
	if (vif->type == NL80211_IFTYPE_STATION) {
		rsi_set_min_rate(hw, sta, common);
		if (g_bgscan_enable) {
			if (!rsi_send_bgscan_params(common, 1)) {
				if (!rsi_send_bgscan_probe_req(common)) {
					ven_rsi_dbg(INFO_ZONE,
						"Bgscan started ===>\n");
					common->bgscan_en = 1;
				}
			}
		}
	}

#if 0
	if ((vif->type == NL80211_IFTYPE_STATION) &&
	    sta->ht_cap.ht_supported)
#endif
	if (sta->ht_cap.ht_supported)
		ieee80211_start_tx_ba_session(sta, 0, 0);

	mutex_unlock(&common->mutex);

	return 0;
}

/**
 * rsi_mac80211_sta_remove() - This function notifies driver about a peer
 *			       getting disconnected.
 * @hw: Pointer to the ieee80211_hw structure.
 * @vif: Pointer to the ieee80211_vif structure.
 * @sta: Pointer to the ieee80211_sta structure.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_mac80211_sta_remove(struct ieee80211_hw *hw,
				   struct ieee80211_vif *vif,
				   struct ieee80211_sta *sta)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	struct ieee80211_bss_conf *bss = &adapter->vifs[0]->bss_conf;

	rsi_hex_dump(INFO_ZONE, "Station Removed: ", sta->addr, ETH_ALEN);

	mutex_lock(&common->mutex);
	if (vif->type == NL80211_IFTYPE_AP) {
		u8 i, j;

		/* Send peer notify to device */
		ven_rsi_dbg(INFO_ZONE, "Indicate bss status to device\n");
		for (i = 0; i < RSI_MAX_ASSOC_STAS; i++) {
			if (!common->stations[i].sta)
				continue;
			if (!memcmp(common->stations[i].sta->addr,
				    sta->addr, ETH_ALEN)) {
				rsi_inform_bss_status(common, AP_OPMODE, 0,
						      sta->addr, sta->wme,
						      sta->aid, sta, i);
				common->stations[i].sta = NULL;
				common->stations[i].sta_id = -1;
				for (j = 0; j < IEEE80211_NUM_ACS; j++)
					common->stations[i].seq_no[j] = 0;
				common->num_stations--;
				if (common->num_stations < 0)
					common->num_stations = 0;
				break;
			}
		}
		if (i >= RSI_MAX_ASSOC_STAS)
			ven_rsi_dbg(ERR_ZONE, "%s: No station found\n", __func__);
	}

	if (vif->type == NL80211_IFTYPE_STATION) {
		/* Resetting all the fields to default values */
		memcpy((u8 *)bss->bssid, (u8 *)sta->addr, ETH_ALEN);
		bss->qos = sta->wme;
		common->bitrate_mask[NL80211_BAND_2GHZ] = 0;
		common->bitrate_mask[NL80211_BAND_5GHZ] = 0;
		common->min_rate = 0xffff;
		common->vif_info[0].is_ht = false;
		common->vif_info[0].sgi = false;
		common->vif_info[0].seq_start = 0;
		common->secinfo.ptk_cipher = 0;
		common->secinfo.gtk_cipher = 0;
		if (common->bgscan_en)
			common->bgscan_en = 0;

		if (!common->iface_down)
			rsi_send_rx_filter_frame(common, 0);
	}
	mutex_unlock(&common->mutex);
	return 0;
}
#if 0
static void rsi_mac80211_sw_scan_start(struct ieee80211_hw *hw,
				       struct ieee80211_vif *vif,
				       const u8 *mac_addr)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	struct ieee80211_bss_conf *bss = &adapter->vifs[0]->bss_conf;
	u8 status = 0;
//	u16 rx_filter_word = 0;

	if (!bss->assoc)
		return;

	mutex_lock(&common->mutex);

	status = rsi_send_bgscan_params(common, 1);
	if (!status) {
		ven_rsi_dbg(INFO_ZONE, "Background scan commensing\n");
		if (!rsi_send_bgscan_probe_req(common))
			common->bgscan_en = 1;
	}
	
	mutex_unlock(&common->mutex);

	return ;
}

static void rsi_mac80211_sw_scan_stop(struct ieee80211_hw *hw,
				      struct ieee80211_vif *vif)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
//	struct ieee80211_bss_conf *bss = &adapter->vifs[0]->bss_conf;
//	u16 rx_filter_word = 0;

	mutex_lock(&common->mutex);
	
	if (common->bgscan_en) {
		if (!rsi_send_bgscan_params(common, 0))
			common->bgscan_en = 0;
	}
#if 0
	if (bss->assoc) {
		rx_filter_word = (ALLOW_DATA_ASSOC_PEER |
				ALLOW_CTRL_ASSOC_PEER |
				ALLOW_MGMT_ASSOC_PEER);
		rsi_send_rx_filter_frame(common, rx_filter_word);
	}

	if (bss->assoc) {
		if (!rsi_band_check(common)) {
			rsi_set_channel(adapter->priv,
					rsi_get_connected_channel(adapter));
		}
	}
#endif

	mutex_unlock(&common->mutex);

	return;
}
#endif

/**
 * rsi_mac80211_set_antenna() - This function is used to configure
 *				tx and rx antennas.
 * @hw: Pointer to the ieee80211_hw structure.
 * @tx_ant: Bitmap for tx antenna
 * @rx_ant: Bitmap for rx antenna
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_mac80211_set_antenna(struct ieee80211_hw *hw,
				    u32 tx_ant, u32 rx_ant)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	u32 antenna = 0;

	if (tx_ant > 1 || rx_ant > 1) {
		ven_rsi_dbg(ERR_ZONE,
			"Invalid antenna selection (tx: %d, rx:%d)\n",
			tx_ant, rx_ant);
		ven_rsi_dbg(ERR_ZONE,
			"Use 0 for int_ant, 1 for ext_ant\n");
		return -EINVAL; 
	}

	ven_rsi_dbg(INFO_ZONE, "%s: Antenna map Tx %x Rx %d\n",
			__func__, tx_ant, rx_ant);

	mutex_lock(&common->mutex);

	antenna = tx_ant ? ANTENNA_SEL_UFL : ANTENNA_SEL_INT;
	if (common->ant_in_use != antenna)
		if (rsi_set_antenna(common, antenna))
			goto fail_set_antenna;

	ven_rsi_dbg(INFO_ZONE, "(%s) Antenna path configured successfully\n",
		tx_ant ? "UFL" : "INT");

	common->ant_in_use = antenna;
	
	mutex_unlock(&common->mutex);
	
	return 0;

fail_set_antenna:
	ven_rsi_dbg(ERR_ZONE, "%s: Failed.\n", __func__);
	mutex_unlock(&common->mutex);
	return -EINVAL;
}

/**
 * rsi_mac80211_get_antenna() - This function is used to configure 
 * 				tx and rx antennas.
 *
 * @hw: Pointer to the ieee80211_hw structure.
 * @tx_ant: Bitmap for tx antenna
 * @rx_ant: Bitmap for rx antenna
 * 
 * Return: 0 on success, -1 on failure.
 */
static int rsi_mac80211_get_antenna(struct ieee80211_hw *hw,
				    u32 *tx_ant, u32 *rx_ant)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;

	mutex_lock(&common->mutex);

	*tx_ant = (common->ant_in_use == ANTENNA_SEL_UFL) ? 1 : 0;
	*rx_ant = 0;

	mutex_unlock(&common->mutex);
	
	return 0;	
}

static void rsi_reg_notify(struct wiphy *wiphy,
			  struct regulatory_request *request)
{
	struct ieee80211_supported_band *sband;
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct rsi_hw * adapter = hw->priv; 
	struct rsi_common *common = adapter->priv;
	struct ieee80211_channel *ch;
	int i;

	if (common->num_supp_bands == 1)
		return;
	mutex_lock(&common->mutex);

	sband = wiphy->bands[NL80211_BAND_5GHZ];
	
	for (i = 0; i < sband->n_channels; i++) {
		ch = &sband->channels[i];
		if (ch->flags & IEEE80211_CHAN_DISABLED)
			continue;

		if (ch->flags & IEEE80211_CHAN_RADAR)
			ch->flags |= IEEE80211_CHAN_NO_IR;
	}
	
	ven_rsi_dbg(INFO_ZONE,
		"country = %s dfs_region = %d\n",
		request->alpha2, request->dfs_region);

	/* If DFS region or country is changed configure back ground scan
	 * params to device again */
	if ((adapter->dfs_region != request->dfs_region) ||
	    (memcmp(adapter->country, request->alpha2, 2))) {
		if (common->bgscan_en) {
			rsi_send_bgscan_params(common, 0);
			common->bgscan_en = 0;
			mdelay(10);
			rsi_send_bgscan_params(common, 1);
			common->bgscan_en = 1;
		}
	}
	
	adapter->dfs_region = request->dfs_region;
	adapter->country[0] = request->alpha2[0];
	adapter->country[1] = request->alpha2[1];
	mutex_unlock(&common->mutex);
}

void rsi_mac80211_rfkill_poll(struct ieee80211_hw *hw)
{
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	
	mutex_lock(&common->mutex);

	if (common->fsm_state != FSM_MAC_INIT_DONE)
		wiphy_rfkill_set_hw_state(hw->wiphy, true);
	else
		wiphy_rfkill_set_hw_state(hw->wiphy, false);

	mutex_unlock(&common->mutex);
}

#ifdef CONFIG_RSI_WOW
static const struct wiphy_wowlan_support rsi_wowlan_support = {
	.flags = WIPHY_WOWLAN_ANY |
		 WIPHY_WOWLAN_MAGIC_PKT |
		 WIPHY_WOWLAN_DISCONNECT |
		 WIPHY_WOWLAN_GTK_REKEY_FAILURE  |
		 WIPHY_WOWLAN_SUPPORTS_GTK_REKEY |
		 WIPHY_WOWLAN_EAP_IDENTITY_REQ   |
		 WIPHY_WOWLAN_4WAY_HANDSHAKE,
	.n_patterns = 0,
	.pattern_min_len = 1,
	.pattern_max_len = 0,
};

static u16 rsi_wow_map_triggers(struct rsi_common *common,
				struct cfg80211_wowlan *wowlan)
{
	u16 wow_triggers = 0;

	ven_rsi_dbg(INFO_ZONE,"Mapping wowlan triggers\n");

	if (wowlan->any)
		wow_triggers |= RSI_WOW_ANY;
	if (wowlan->magic_pkt)
		wow_triggers |= RSI_WOW_MAGIC_PKT;
	if (wowlan->disconnect)
		wow_triggers |= RSI_WOW_DISCONNECT;
	if (wowlan->gtk_rekey_failure || wowlan->eap_identity_req ||
	    wowlan->four_way_handshake)
		wow_triggers |= RSI_WOW_SUPPORTS_GTK_REKEY;
	
	return wow_triggers;
}
#endif

#ifdef CONFIG_PM
int rsi_mac80211_suspend(struct ieee80211_hw *hw,
			 struct cfg80211_wowlan *wowlan)
{
#ifdef CONFIG_RSI_WOW
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	u16 triggers, rx_filter_word = 0;
#endif
	int ret = 0;

	ven_rsi_dbg(INFO_ZONE, "***** mac80211 suspend called ******\n");

#ifdef CONFIG_RSI_WOW
	if (WARN_ON(!wowlan)) {
		ven_rsi_dbg(ERR_ZONE,
			"##### WoW triggers not enabled #####\n");
		ret = -EINVAL;
		goto fail_wow;
	}

	triggers = rsi_wow_map_triggers(common, wowlan);
	if (!triggers) {
		ven_rsi_dbg(ERR_ZONE, "%s:No valid WoW triggers\n",__func__);
		ret = -EINVAL;
		goto fail_wow;
	}
	ven_rsi_dbg(INFO_ZONE, "TRIGGERS %x\n", triggers);

	rsi_send_wowlan_request(common, triggers, 1);

 	 rx_filter_word = (ALLOW_DATA_ASSOC_PEER |
			   ALLOW_CTRL_ASSOC_PEER |
			   ALLOW_MGMT_ASSOC_PEER |
			   DISALLOW_BEACONS |
			   0);
	rsi_send_rx_filter_frame(common, rx_filter_word);
        common->suspend_flag = 1;
fail_wow:
#endif
        return ret;
}

static int rsi_mac80211_resume(struct ieee80211_hw *hw)
{
#ifdef CONFIG_RSI_WOW
	struct rsi_hw *adapter = hw->priv;
	struct rsi_common *common = adapter->priv;
	u16 rx_filter_word = 0;
#endif
	
	ven_rsi_dbg(INFO_ZONE, "%s: mac80211 resume\n", __func__);

#ifdef CONFIG_RSI_WOW
	rsi_send_wowlan_request(common, 0, 0);
	//rx_filter_word = 0xE ;

	rx_filter_word = (ALLOW_DATA_ASSOC_PEER |
			  ALLOW_CTRL_ASSOC_PEER |
			  ALLOW_MGMT_ASSOC_PEER |
			  0);
	rsi_send_rx_filter_frame(common, rx_filter_word);
#endif
	return 0;
}
#endif

static struct ieee80211_ops mac80211_ops = {
	.tx = rsi_mac80211_tx,
	.start = rsi_mac80211_start,
	.stop = rsi_mac80211_stop,
	.add_interface = rsi_mac80211_add_interface,
	.remove_interface = rsi_mac80211_remove_interface,
	.config = rsi_mac80211_config,
	.bss_info_changed = rsi_mac80211_bss_info_changed,
	.conf_tx = rsi_mac80211_conf_tx,
	.configure_filter = rsi_mac80211_conf_filter,
	.set_key = rsi_mac80211_set_key,
	.set_rts_threshold = rsi_mac80211_set_rts_threshold,
	.set_bitrate_mask = rsi_mac80211_set_rate_mask,
	.ampdu_action = rsi_mac80211_ampdu_action,
	.sta_add = rsi_mac80211_sta_add,
	.sta_remove = rsi_mac80211_sta_remove,
	.set_antenna = rsi_mac80211_set_antenna,
	.get_antenna = rsi_mac80211_get_antenna,
        .rfkill_poll = rsi_mac80211_rfkill_poll,
#ifdef CONFIG_PM 
	.suspend = rsi_mac80211_suspend,
	.resume  = rsi_mac80211_resume,
#endif

};

/**
 * rsi_mac80211_attach() - This function is used to initialize Mac80211 stack.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, -1 on failure.
 */
int rsi_mac80211_attach(struct rsi_common *common)
{
	int status = 0;
	struct ieee80211_hw *hw = NULL;
	struct wiphy *wiphy = NULL;
	struct rsi_hw *adapter = common->priv;
	u8 addr_mask[ETH_ALEN] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x3};

	ven_rsi_dbg(INIT_ZONE, "%s: Performing mac80211 attach\n", __func__);

	hw = ieee80211_alloc_hw(sizeof(struct rsi_hw), &mac80211_ops);
	if (!hw) {
		ven_rsi_dbg(ERR_ZONE, "%s: ieee80211 hw alloc failed\n", __func__);
		return -ENOMEM;
	}

	wiphy = hw->wiphy;

	SET_IEEE80211_DEV(hw, adapter->device);

	hw->priv = adapter;
	adapter->hw = hw;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 2, 0))
	ieee80211_hw_set(hw, SIGNAL_DBM);
	ieee80211_hw_set(hw, HAS_RATE_CONTROL);
	ieee80211_hw_set(hw, AMPDU_AGGREGATION);
	ieee80211_hw_set(hw, SUPPORTS_PS);
	ieee80211_hw_set(hw, SUPPORTS_DYNAMIC_PS);
//	ieee80211_hw_set(hw, CONNECTION_MONITOR);
	ieee80211_hw_set(hw, SPECTRUM_MGMT);
	ieee80211_hw_set(hw, MFP_CAPABLE);
#else
	hw->flags = IEEE80211_HW_SIGNAL_DBM |
		    IEEE80211_HW_HAS_RATE_CONTROL |
		    IEEE80211_HW_AMPDU_AGGREGATION |
		    IEEE80211_HW_SUPPORTS_PS |
		    IEEE80211_HW_SUPPORTS_DYNAMIC_PS |
//		    IEEE80211_HW_CONNECTION_MONITOR |
		    IEEE80211_HW_MFP_CAPABLE |
		    0;
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0))
	wiphy_ext_feature_set(wiphy, NL80211_EXT_FEATURE_RRM);
#endif

	hw->queues = MAX_HW_QUEUES;
	hw->extra_tx_headroom = RSI_NEEDED_HEADROOM;

	hw->max_rates = 1;
	hw->max_rate_tries = MAX_RETRIES;
	hw->uapsd_queues = IEEE80211_MARKALL_UAPSD_QUEUES;
	hw->uapsd_max_sp_len = IEEE80211_STA_SP_ALL_PKTS;
//	hw->max_tx_aggregation_subframes = 6;
	hw->max_tx_aggregation_subframes = 4;

	rsi_register_rates_channels(adapter, NL80211_BAND_2GHZ);
	wiphy->bands[NL80211_BAND_2GHZ] =
		&adapter->sbands[NL80211_BAND_2GHZ];
	if (common->num_supp_bands == 2) {
		rsi_register_rates_channels(adapter, NL80211_BAND_5GHZ);
		wiphy->bands[NL80211_BAND_5GHZ] =
			&adapter->sbands[NL80211_BAND_5GHZ];
	}
	hw->rate_control_algorithm = "AARF";
	hw->sta_data_size = sizeof(struct rsi_sta);

	SET_IEEE80211_PERM_ADDR(hw, common->mac_addr);
	ether_addr_copy(hw->wiphy->addr_mask, addr_mask);

	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) |
				 BIT(NL80211_IFTYPE_AP);
	wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	wiphy->retry_short = RETRY_SHORT;
	wiphy->retry_long  = RETRY_LONG;
	wiphy->frag_threshold = IEEE80211_MAX_FRAG_THRESHOLD;
	wiphy->rts_threshold = IEEE80211_MAX_RTS_THRESHOLD;
	wiphy->available_antennas_tx = 1;
	wiphy->available_antennas_rx = 1;
	wiphy->max_ap_assoc_sta = RSI_MAX_ASSOC_STAS;

	wiphy->flags = WIPHY_FLAG_REPORTS_OBSS;

	wiphy->reg_notifier = rsi_reg_notify;

#ifdef CONFIG_RSI_WOW
	wiphy->wowlan = &rsi_wowlan_support;
#endif

	status = ieee80211_register_hw(hw);
	if (status)
		return status;

	return rsi_init_dbgfs(adapter);
}
