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
#include "rsi_mgmt.h"
#include "rsi_common.h"
#include "rsi_ps.h"
#include "rsi_hal.h"
#ifdef CONFIG_VEN_RSI_COEX
#include "rsi_coex.h"
#endif

struct rsi_config_vals dev_config_vals[] = {
	{
		.lp_ps_handshake = 0,
		.ulp_ps_handshake = 0,
		.sleep_config_params = 0,
		.ext_pa_or_bt_coex_en = 0,
	},
};

/* Bootup Parameters for 20MHz */
static struct bootup_params boot_params_20 = {
	.magic_number = cpu_to_le16(0x5aa5),
	.crystal_good_time = 0x0,
	.valid = cpu_to_le32(VALID_20),
	.reserved_for_valids = 0x0,
	.bootup_mode_info = 0x0,
	.digital_loop_back_params = 0x0,
	.rtls_timestamp_en = 0x0,
	.host_spi_intr_cfg = 0x0,
	.device_clk_info = {{
		/* WLAN params */
		.pll_config_g = {
			.tapll_info_g = {
				.pll_reg_1 = cpu_to_le16((TAPLL_N_VAL_20 << 8) |
							 (TAPLL_M_VAL_20)),
				.pll_reg_2 = cpu_to_le16(TAPLL_P_VAL_20),
			},
			.pll960_info_g = {
				.pll_reg_1 = cpu_to_le16((PLL960_P_VAL_20 << 8) |
							 (PLL960_N_VAL_20)),
				.pll_reg_2 = cpu_to_le16(PLL960_M_VAL_20),
				.pll_reg_3 = 0x0,
			},
			.afepll_info_g = {
				.pll_reg = cpu_to_le16(0x9f0),
			}
		},
		.switch_clk_g = {
			.switch_umac_clk = 0x1,
			.switch_qspi_clk = 0x1,
			.switch_slp_clk_2_32 = 0x0,
			.switch_bbp_lmac_clk_reg = 0x1,
			.switch_mem_ctrl_cfg = 0x0,
			.reserved = 0x0,
			.bbp_lmac_clk_reg_val = cpu_to_le16(0x111),
			.umac_clock_reg_config = cpu_to_le16(0x48),
			.qspi_uart_clock_reg_config = cpu_to_le16(0x1211)
		}
	},
	/* Bluetooth params */
	{
		.pll_config_g = {
			.tapll_info_g = {
				.pll_reg_1 = cpu_to_le16((TAPLL_N_VAL_20 << 8) |
							 (TAPLL_M_VAL_20)),
				.pll_reg_2 = cpu_to_le16(TAPLL_P_VAL_20),
			},
			.pll960_info_g = {
				.pll_reg_1 = cpu_to_le16((PLL960_P_VAL_20 << 8) |
							 (PLL960_N_VAL_20)),
				.pll_reg_2 = cpu_to_le16(PLL960_M_VAL_20),
				.pll_reg_3 = 0x0,
			},
			.afepll_info_g = {
				.pll_reg = cpu_to_le16(0x9f0),
			}
		},
		.switch_clk_g = {
			.switch_umac_clk = 0x0,
			.switch_qspi_clk = 0x0,
			.switch_slp_clk_2_32 = 0x0,
			.switch_bbp_lmac_clk_reg = 0x0,
			.switch_mem_ctrl_cfg = 0x0,
			.reserved = 0x0,
			.bbp_lmac_clk_reg_val = 0x0,
			.umac_clock_reg_config = 0x0,
			.qspi_uart_clock_reg_config = 0x0
		}
	},
	/* Zigbee params */
	{
		.pll_config_g = {
			.tapll_info_g = {
				.pll_reg_1 = cpu_to_le16((TAPLL_N_VAL_20 << 8) |
							 (TAPLL_M_VAL_20)),
				.pll_reg_2 = cpu_to_le16(TAPLL_P_VAL_20),
			},
			.pll960_info_g = {
				.pll_reg_1 = cpu_to_le16((PLL960_P_VAL_20 << 8) |
							 (PLL960_N_VAL_20)),
				.pll_reg_2 = cpu_to_le16(PLL960_M_VAL_20),
				.pll_reg_3 = 0x0,
			},
			.afepll_info_g = {
				.pll_reg = cpu_to_le16(0x9f0),
			}
		},
		.switch_clk_g = {
			.switch_umac_clk = 0x0,
			.switch_qspi_clk = 0x0,
			.switch_slp_clk_2_32 = 0x0,
			.switch_bbp_lmac_clk_reg = 0x0,
			.switch_mem_ctrl_cfg = 0x0,
			.reserved = 0x0,
			.bbp_lmac_clk_reg_val = 0x0,
			.umac_clock_reg_config = 0x0,
			.qspi_uart_clock_reg_config = 0x0
		}
	} },
	/* ULP Params */
	.buckboost_wakeup_cnt = 0x0,
	.pmu_wakeup_wait = 0x0,
	.shutdown_wait_time = 0x0,
	.pmu_slp_clkout_sel = 0x0,
	.wdt_prog_value = 0x0,
	.wdt_soc_rst_delay = 0x0,
	.dcdc_operation_mode = 0x0,
	.soc_reset_wait_cnt = 0x0,
	.waiting_time_at_fresh_sleep = 0x0,
	.max_threshold_to_avoid_sleep = 0x0,
	.beacon_resedue_alg_en = 0,
};

/* Bootup parameters for 40MHz */
static struct bootup_params boot_params_40 = {
	.magic_number = cpu_to_le16(0x5aa5),
	.crystal_good_time = 0x0,
	.valid = cpu_to_le32(VALID_40),
	.reserved_for_valids = 0x0,
	.bootup_mode_info = 0x0,
	.digital_loop_back_params = 0x0,
	.rtls_timestamp_en = 0x0,
	.host_spi_intr_cfg = 0x0,
	.device_clk_info = {{
		/* WLAN params */
		.pll_config_g = {
			.tapll_info_g = {
				.pll_reg_1 = cpu_to_le16((TAPLL_N_VAL_40 << 8) |
							 (TAPLL_M_VAL_40)),
				.pll_reg_2 = cpu_to_le16(TAPLL_P_VAL_40),
			},
			.pll960_info_g = {
				.pll_reg_1 = cpu_to_le16((PLL960_P_VAL_40 << 8) |
							 (PLL960_N_VAL_40)),
				.pll_reg_2 = cpu_to_le16(PLL960_M_VAL_40),
				.pll_reg_3 = 0x0,
			},
			.afepll_info_g = {
				.pll_reg = cpu_to_le16(0x9f0),
			}
		},
		.switch_clk_g = {
			.switch_umac_clk = 0x1,
			.switch_qspi_clk = 0x1,
			.switch_slp_clk_2_32 = 0x0,
			.switch_bbp_lmac_clk_reg = 0x1,
			.switch_mem_ctrl_cfg = 0x0,
			.reserved = 0x0,
			.bbp_lmac_clk_reg_val = cpu_to_le16(0x1121),
			.umac_clock_reg_config = cpu_to_le16(0x48),
			.qspi_uart_clock_reg_config = cpu_to_le16(0x1211)
		}
	},
	/* Bluetooth Params */
	{
		.pll_config_g = {
			.tapll_info_g = {
				.pll_reg_1 = cpu_to_le16((TAPLL_N_VAL_40 << 8) |
							 (TAPLL_M_VAL_40)),
				.pll_reg_2 = cpu_to_le16(TAPLL_P_VAL_40),
			},
			.pll960_info_g = {
				.pll_reg_1 = cpu_to_le16((PLL960_P_VAL_40 << 8) |
							 (PLL960_N_VAL_40)),
				.pll_reg_2 = cpu_to_le16(PLL960_M_VAL_40),
				.pll_reg_3 = 0x0,
			},
			.afepll_info_g = {
				.pll_reg = cpu_to_le16(0x9f0),
			}
		},
		.switch_clk_g = {
			.switch_umac_clk = 0x0,
			.switch_qspi_clk = 0x0,
			.switch_slp_clk_2_32 = 0x0,
			.switch_bbp_lmac_clk_reg = 0x0,
			.switch_mem_ctrl_cfg = 0x0,
			.reserved = 0x0,
			.bbp_lmac_clk_reg_val = 0x0,
			.umac_clock_reg_config = 0x0,
			.qspi_uart_clock_reg_config = 0x0
		}
	},
	/* Zigbee Params */
	{
		.pll_config_g = {
			.tapll_info_g = {
				.pll_reg_1 = cpu_to_le16((TAPLL_N_VAL_40 << 8) |
							 (TAPLL_M_VAL_40)),
				.pll_reg_2 = cpu_to_le16(TAPLL_P_VAL_40),
			},
			.pll960_info_g = {
				.pll_reg_1 = cpu_to_le16((PLL960_P_VAL_40 << 8) |
							 (PLL960_N_VAL_40)),
				.pll_reg_2 = cpu_to_le16(PLL960_M_VAL_40),
				.pll_reg_3 = 0x0,
			},
			.afepll_info_g = {
				.pll_reg = cpu_to_le16(0x9f0),
			}
		},
		.switch_clk_g = {
			.switch_umac_clk = 0x0,
			.switch_qspi_clk = 0x0,
			.switch_slp_clk_2_32 = 0x0,
			.switch_bbp_lmac_clk_reg = 0x0,
			.switch_mem_ctrl_cfg = 0x0,
			.reserved = 0x0,
			.bbp_lmac_clk_reg_val = 0x0,
			.umac_clock_reg_config = 0x0,
			.qspi_uart_clock_reg_config = 0x0
		}
	} },
	/* ULP Params */
	.buckboost_wakeup_cnt = 0x0,
	.pmu_wakeup_wait = 0x0,
	.shutdown_wait_time = 0x0,
	.pmu_slp_clkout_sel = 0x0,
	.wdt_prog_value = 0x0,
	.wdt_soc_rst_delay = 0x0,
	.dcdc_operation_mode = 0x0,
	.soc_reset_wait_cnt = 0x0,
	.waiting_time_at_fresh_sleep = 0x0,
	.max_threshold_to_avoid_sleep = 0x0,
	.beacon_resedue_alg_en = 0,
};

#define UNUSED_GPIO	1
#define USED_GPIO	0
struct rsi_ulp_gpio_vals unused_ulp_gpio_bitmap = {
	.motion_sensor_gpio_ulp_wakeup = UNUSED_GPIO,
	.sleep_ind_from_device = UNUSED_GPIO,
	.ulp_gpio_2 = UNUSED_GPIO,
	.push_button_ulp_wakeup = UNUSED_GPIO,
};

struct rsi_soc_gpio_vals unused_soc_gpio_bitmap = {
	.pspi_csn_0		= USED_GPIO,	//GPIO_0
	.pspi_csn_1		= USED_GPIO,	//GPIO_1
	.host_wakeup_intr	= UNUSED_GPIO,	//GPIO_2
	.pspi_data_0		= USED_GPIO,	//GPIO_3
	.pspi_data_1		= USED_GPIO,	//GPIO_4
	.pspi_data_2		= USED_GPIO,	//GPIO_5
	.pspi_data_3		= USED_GPIO,	//GPIO_6
	.i2c_scl		= USED_GPIO,	//GPIO_7
	.i2c_sda		= USED_GPIO,	//GPIO_8
	.uart1_rx		= UNUSED_GPIO,	//GPIO_9
	.uart1_tx		= UNUSED_GPIO,	//GPIO_10
	.uart1_rts_i2s_clk	= UNUSED_GPIO,	//GPIO_11
	.uart1_cts_i2s_ws	= UNUSED_GPIO,	//GPIO_12
	.dbg_uart_rx_i2s_din	= UNUSED_GPIO,	//GPIO_13
	.dbg_uart_tx_i2s_dout	= UNUSED_GPIO,	//GPIO_14
	.lp_wakeup_boot_bypass	= UNUSED_GPIO,	//GPIO_15
	.led_0			= USED_GPIO,	//GPIO_16
	.btcoex_wlan_active_ext_pa_ant_sel_A = UNUSED_GPIO, //GPIO_17
	.btcoex_bt_priority_ext_pa_ant_sel_B = UNUSED_GPIO, //GPIO_18
	.btcoex_bt_active_ext_pa_on_off = UNUSED_GPIO, //GPIO_19
	.rf_reset		= USED_GPIO, //GPIO_20
	.sleep_ind_from_device	= UNUSED_GPIO,
};

static u16 mcs[] = {13, 26, 39, 52, 78, 104, 117, 130};

/**
 * rsi_set_default_parameters() - This function sets default parameters.
 * @common: Pointer to the driver private structure.
 *
 * Return: none
 */
static void rsi_set_default_parameters(struct rsi_common *common)
{
	common->band = NL80211_BAND_2GHZ;
	common->channel_width = BW_20MHZ;
	common->rts_threshold = IEEE80211_MAX_RTS_THRESHOLD;
	common->channel = 1;
	common->min_rate = 0xffff;
	common->fsm_state = FSM_CARD_NOT_READY;
	common->iface_down = true;
	common->endpoint = EP_2GHZ_20MHZ;
	common->driver_mode = 1; /* End-to-End Mode */
#if defined(CONFIG_VEN_RSI_HCI)
	common->coex_mode = 2;
	common->oper_mode = 4;
#elif defined(CONFIG_VEN_RSI_COEX)
	common->coex_mode = 2; /*Default coex mode is WIFI alone */
	common->oper_mode = 5;
#else
	common->coex_mode = 1; /*Default coex mode is WIFI alone */
	common->oper_mode = 1;
#endif

#ifdef CONFIG_RSI_BT_LE
	common->coex_mode = 2;
	common->oper_mode = 8;
#endif
	common->ta_aggr = 0;
	common->skip_fw_load = 0; /* Default disable skipping fw loading */
	common->lp_ps_handshake_mode = 0; /* Default No HandShake mode*/
	common->ulp_ps_handshake_mode = 2; /* Default PKT HandShake mode*/
	common->rf_power_val = 0; /* Default 1.9V */
	common->device_gpio_type = TA_GPIO; /* Default TA GPIO */
	common->country_code = 840; /* Default US */
	common->wlan_rf_power_mode = 0;
	common->bt_rf_power_mode = 0;
	common->obm_ant_sel_val = 2;
	common->antenna_diversity = 0;
	common->tx_power = RSI_TXPOWER_MAX;
	common->dtim_cnt = 2;
	common->beacon_interval = 100;
}

void init_bgscan_params(struct rsi_common *common)
{
	common->bgscan_info.bgscan_threshold = 50;
	common->bgscan_info.roam_threshold = 10;
	common->bgscan_info.bgscan_periodicity = 15;
	common->bgscan_info.num_bg_channels = 11;
	common->bgscan_info.two_probe = 1;
	common->bgscan_info.active_scan_duration = 20;
	common->bgscan_info.passive_scan_duration = 70;
	common->bgscan_info.channels2scan[0] = 1;
	common->bgscan_info.channels2scan[1] = 2;
	common->bgscan_info.channels2scan[2] = 3;
	common->bgscan_info.channels2scan[3] = 4;
	common->bgscan_info.channels2scan[4] = 5;
	common->bgscan_info.channels2scan[5] = 6;
	common->bgscan_info.channels2scan[6] = 7;
	common->bgscan_info.channels2scan[7] = 8;
	common->bgscan_info.channels2scan[8] = 9;
	common->bgscan_info.channels2scan[9] = 10;
	common->bgscan_info.channels2scan[10] = 11;
//	common->bgscan_info.channels2scan[11] = 12;
//	common->bgscan_info.channels2scan[12] = 13;
//	common->bgscan_info.channels2scan[13] = 14;
#if 0
	common->bgscan_info.channels2scan[11] = 36;
	common->bgscan_info.channels2scan[12] = 40;
	common->bgscan_info.channels2scan[13] = 44;
	common->bgscan_info.channels2scan[14] = 48;
	common->bgscan_info.channels2scan[15] = 52;
	common->bgscan_info.channels2scan[16] = 56;
	common->bgscan_info.channels2scan[17] = 60;
	common->bgscan_info.channels2scan[18] = 64;
	common->bgscan_info.channels2scan[19] = 100;
	common->bgscan_info.channels2scan[20] = 104;
#endif
}

/**
 * rsi_set_contention_vals() - This function sets the contention values for the
 *			       backoff procedure.
 * @common: Pointer to the driver private structure.
 *
 * Return: None.
 */
static void rsi_set_contention_vals(struct rsi_common *common)
{
	u8 ii = 0;

	for (; ii < NUM_EDCA_QUEUES; ii++) {
		common->tx_qinfo[ii].wme_params =
			(((common->edca_params[ii].cw_min / 2) +
			  (common->edca_params[ii].aifs)) *
			  WMM_SHORT_SLOT_TIME + SIFS_DURATION);
		common->tx_qinfo[ii].weight = common->tx_qinfo[ii].wme_params;
		common->tx_qinfo[ii].pkt_contended = 0;
	}
}

/**
 * rsi_send_internal_mgmt_frame() - This function sends management frames to
 *				    firmware.Also schedules packet to queue
 *				    for transmission.
 * @common: Pointer to the driver private structure.
 * @skb: Pointer to the socket buffer structure.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_send_internal_mgmt_frame(struct rsi_common *common,
					struct sk_buff *skb)
{
	struct skb_info *tx_params;

	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: SKB is NULL\n", __func__);
		return -EINVAL;
	}
	skb->data[1] |= BIT(7);
	tx_params = (struct skb_info *)&IEEE80211_SKB_CB(skb)->driver_data;
	tx_params->flags |= INTERNAL_MGMT_PKT;
	skb->priority = MGMT_SOFT_Q;
	skb_queue_tail(&common->tx_queue[MGMT_SOFT_Q], skb);
	rsi_set_event(&common->tx_thread.event);
	return 0;
}

/**
 * rsi_load_radio_caps() - This function is used to send radio capabilities
 *			   values to firmware.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding negative error code on failure.
 */
static int rsi_load_radio_caps(struct rsi_common *common)
{
	struct rsi_radio_caps *radio_caps;
	struct rsi_hw *adapter = common->priv;
	u16 inx = 0;
	int ii;
	u8 radio_id = 0;
	u16 gc[20] = {0xf0, 0xf0, 0xf0, 0xf0,
		      0xf0, 0xf0, 0xf0, 0xf0,
		      0xf0, 0xf0, 0xf0, 0xf0,
		      0xf0, 0xf0, 0xf0, 0xf0,
		      0xf0, 0xf0, 0xf0, 0xf0};
	struct sk_buff *skb;

	ven_rsi_dbg(INFO_ZONE, "%s: Sending rate symbol req frame\n", __func__);

	skb = dev_alloc_skb(sizeof(struct rsi_radio_caps));
	if (!skb)
		return -ENOMEM;

	memset(skb->data, 0, sizeof(struct rsi_radio_caps));
	radio_caps = (struct rsi_radio_caps *)skb->data;

	radio_caps->desc_word[1] = cpu_to_le16(RADIO_CAPABILITIES);
	radio_caps->desc_word[4] = cpu_to_le16(common->channel);
	radio_caps->desc_word[4] |= cpu_to_le16(RSI_RF_TYPE << 8);

	radio_caps->desc_word[7] |= cpu_to_le16(RSI_LMAC_CLOCK_80MHZ);
	if (common->channel_width == BW_40MHZ) {
		radio_caps->desc_word[7] |= cpu_to_le16(RSI_ENABLE_40MHZ);

		if (common->fsm_state == FSM_MAC_INIT_DONE) {
			struct ieee80211_hw *hw = adapter->hw;
			struct ieee80211_conf *conf = &hw->conf;

			if (conf_is_ht40_plus(conf)) {
				radio_caps->desc_word[5] =
					cpu_to_le16(LOWER_20_ENABLE);
				radio_caps->desc_word[5] |=
					cpu_to_le16(LOWER_20_ENABLE >> 12);
			} else if (conf_is_ht40_minus(conf)) {
				radio_caps->desc_word[5] =
					cpu_to_le16(UPPER_20_ENABLE);
				radio_caps->desc_word[5] |=
					cpu_to_le16(UPPER_20_ENABLE >> 12);
			} else {
				radio_caps->desc_word[5] =
					cpu_to_le16(BW_40MHZ << 12);
				radio_caps->desc_word[5] |=
					cpu_to_le16(FULL40M_ENABLE);
			}
		}
	}

	radio_caps->sifs_tx_11n = cpu_to_le16(SIFS_TX_11N_VALUE);
	radio_caps->sifs_tx_11b = cpu_to_le16(SIFS_TX_11B_VALUE);
	radio_caps->slot_rx_11n = cpu_to_le16(SHORT_SLOT_VALUE);
	radio_caps->ofdm_ack_tout = cpu_to_le16(OFDM_ACK_TOUT_VALUE);
	radio_caps->cck_ack_tout = cpu_to_le16(CCK_ACK_TOUT_VALUE);
	radio_caps->preamble_type = cpu_to_le16(LONG_PREAMBLE);

	radio_caps->desc_word[7] |= cpu_to_le16(radio_id << 8);

	for (ii = 0; ii < MAX_HW_QUEUES; ii++) {
		radio_caps->qos_params[ii].cont_win_min_q = cpu_to_le16(3);
		radio_caps->qos_params[ii].cont_win_max_q = cpu_to_le16(0x3f);
		radio_caps->qos_params[ii].aifsn_val_q = cpu_to_le16(2);
		radio_caps->qos_params[ii].txop_q = 0;
	}

	for (ii = 0; ii < NUM_EDCA_QUEUES; ii++) {
		radio_caps->qos_params[ii].cont_win_min_q =
			cpu_to_le16(common->edca_params[ii].cw_min);
		radio_caps->qos_params[ii].cont_win_max_q =
			cpu_to_le16(common->edca_params[ii].cw_max);
		radio_caps->qos_params[ii].aifsn_val_q =
			cpu_to_le16((common->edca_params[ii].aifs) << 8);
		radio_caps->qos_params[ii].txop_q =
			cpu_to_le16(common->edca_params[ii].txop);
	}

	radio_caps->qos_params[BROADCAST_HW_Q].txop_q = 0xffff;
	radio_caps->qos_params[MGMT_HW_Q].txop_q = 0;
	radio_caps->qos_params[BEACON_HW_Q].txop_q = 0xffff;

	memcpy(&common->rate_pwr[0], &gc[0], 40);
	for (ii = 0; ii < 20; ii++)
		radio_caps->gcpd_per_rate[inx++] =
			cpu_to_le16(common->rate_pwr[ii]  & 0x00FF);

	radio_caps->desc_word[0] = cpu_to_le16((sizeof(struct rsi_radio_caps) -
						FRAME_DESC_SZ) |
						(RSI_WIFI_MGMT_Q << 12));

	skb_put(skb, (sizeof(struct rsi_radio_caps)));

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_mgmt_pkt_to_core() - This function is the entry point for Mgmt module.
 * @common: Pointer to the driver private structure.
 * @msg: Pointer to received packet.
 * @msg_len: Length of the received packet.
 * @type: Type of received packet.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_mgmt_pkt_to_core(struct rsi_common *common,
				u8 *msg,
				s32 msg_len)
{
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_tx_info *info;
	struct skb_info *rx_params;
	u8 pad_bytes = msg[4];
	u8 pkt_recv;
	struct sk_buff *skb;
	char *buffer;
	struct ieee80211_hdr *wlh;

	if (!adapter->sc_nvifs)
		return -ENOLINK;

	msg_len -= pad_bytes;
	if ((msg_len <= 0) || (!msg)) {
		ven_rsi_dbg(MGMT_RX_ZONE,
			"%s: Invalid rx msg of len = %d\n",
			__func__, msg_len);
		return -EINVAL;
	}

	skb = dev_alloc_skb(msg_len);
	if (!skb)
		return -ENOMEM;

	buffer = skb_put(skb, msg_len);

	memcpy(buffer,
		(u8 *)(msg +  FRAME_DESC_SZ + pad_bytes),
		msg_len);

	pkt_recv = buffer[0];

	info = IEEE80211_SKB_CB(skb);
	rx_params = (struct skb_info *)info->driver_data;
	rx_params->rssi = rsi_get_rssi(msg);
	rx_params->channel = rsi_get_channel(msg);
	ven_rsi_dbg(MGMT_RX_ZONE,
		"%s: rssi=%d channel=%d\n",
		__func__, rx_params->rssi, rx_params->channel);
	wlh = (struct ieee80211_hdr *)skb->data;
	ven_rsi_dbg(INFO_ZONE, "RX Dot11 Mgmt Pkt Type: %s\n",
		dot11_pkt_type(wlh->frame_control));
	rsi_indicate_pkt_to_os(common, skb);

	return 0;
}

/**
 * rsi_send_sta_notify_frame() - This function sends the station notify
 *				     frame to firmware.
 * @common: Pointer to the driver private structure.
 * @opmode: Operating mode of device.
 * @notify_event: Notification about station connection.
 * @bssid: bssid.
 * @qos_enable: Qos is enabled.
 * @aid: Aid (unique for all STA).
 *
 * Return: status: 0 on success, corresponding negative error code on failure.
 */
int rsi_send_sta_notify_frame(struct rsi_common *common,
			      enum opmode opmode,
			      u8 notify_event,
			      const unsigned char *bssid,
			      u8 qos_enable,
			      u16 aid,
			      u16 sta_id)
{
	struct ieee80211_vif *vif = common->priv->vifs[0];
	struct sk_buff *skb = NULL;
	struct rsi_peer_notify *peer_notify;
	int status;
	u16 vap_id = 0;
	int frame_len = sizeof(*peer_notify);

	ven_rsi_dbg(MGMT_TX_ZONE, "%s: Sending station notify frame\n", __func__);

	skb = dev_alloc_skb(frame_len);
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}
	memset(skb->data, 0, frame_len);

	peer_notify = (struct rsi_peer_notify *)skb->data;

	if (opmode == STA_OPMODE)
		peer_notify->command = cpu_to_le16(PEER_TYPE_AP << 1);
	else if (opmode == AP_OPMODE)
		peer_notify->command = cpu_to_le16(PEER_TYPE_STA << 1);

	switch (notify_event) {
	case STA_CONNECTED:
		peer_notify->command |= cpu_to_le16(RSI_ADD_PEER);
		break;
	case STA_DISCONNECTED:
		peer_notify->command |= cpu_to_le16(RSI_DELETE_PEER);
		break;
	default:
		break;
	}
	peer_notify->command |= cpu_to_le16((aid & 0xfff) << 4);
	ether_addr_copy(peer_notify->mac_addr, bssid);
	peer_notify->mpdu_density = cpu_to_le16(0x08); //FIXME check this
	peer_notify->sta_flags = cpu_to_le32((qos_enable) ? 1 : 0);
	peer_notify->desc_word[0] = cpu_to_le16((frame_len - FRAME_DESC_SZ) |
						(RSI_WIFI_MGMT_Q << 12));
	peer_notify->desc_word[1] = cpu_to_le16(PEER_NOTIFY);
	peer_notify->desc_word[7] |= cpu_to_le16(sta_id | vap_id << 8);

	skb_put(skb, frame_len);
	status = rsi_send_internal_mgmt_frame(common, skb);

	if ((vif->type == NL80211_IFTYPE_STATION) &&
	    (!status) && qos_enable) {
		rsi_set_contention_vals(common);
		mdelay(1);
		status = rsi_load_radio_caps(common);
	}

	return status;
}

/**
 * rsi_send_aggr_params_frame() - This function sends the ampdu
 *					 indication frame to firmware.
 * @common: Pointer to the driver private structure.
 * @tid: traffic identifier.
 * @ssn: ssn.
 * @buf_size: buffer size.
 * @event: notification about station connection.
 *
 * Return: 0 on success, corresponding negative error code on failure.
 */
int rsi_send_aggr_params_frame(struct rsi_common *common,
			       u16 tid,
			       u16 ssn,
			       u8 buf_size,
			       u8 event,
			       u8 sta_id)
{
	struct sk_buff *skb = NULL;
	struct rsi_mac_frame *mgmt_frame;
	u8 peer_id = 0;
	u8 window_size = 1;

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	ven_rsi_dbg(MGMT_TX_ZONE,
		"%s: Sending AMPDU indication frame\n",
		__func__);

	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = cpu_to_le16(AMPDU_IND);

	if (event == STA_TX_ADDBA_DONE) {
		mgmt_frame->desc_word[4] = cpu_to_le16(ssn);
//		mgmt_frame->desc_word[5] = cpu_to_le16(buf_size);
		mgmt_frame->desc_word[5] = cpu_to_le16(window_size);
		mgmt_frame->desc_word[7] =
			cpu_to_le16((tid |
				    (START_AMPDU_AGGR << 4) |
				    (peer_id << 8)));
	} else if (event == STA_RX_ADDBA_DONE) {
		mgmt_frame->desc_word[4] = cpu_to_le16(ssn);
		mgmt_frame->desc_word[7] = cpu_to_le16(tid |
						       (START_AMPDU_AGGR << 4) |
						       (RX_BA_INDICATION << 5) |
						       (peer_id << 8));
	} else if (event == STA_TX_DELBA) {
		mgmt_frame->desc_word[7] = cpu_to_le16(tid |
						       (STOP_AMPDU_AGGR << 4) |
						       (peer_id << 8));
	} else if (event == STA_RX_DELBA) {
		mgmt_frame->desc_word[7] = cpu_to_le16(tid |
						       (STOP_AMPDU_AGGR << 4) |
						       (RX_BA_INDICATION << 5) |
						       (peer_id << 8));
	}

	skb_put(skb, FRAME_DESC_SZ);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_program_bb_rf() - This function starts base band and RF programming.
 *			 This is called after initial configurations are done.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding negative error code on failure.
 */
int rsi_program_bb_rf(struct rsi_common *common)
{
	struct sk_buff *skb;
	struct rsi_mac_frame *mgmt_frame;

	ven_rsi_dbg(MGMT_TX_ZONE, "%s: Sending BB/RF program frame\n", __func__);

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = cpu_to_le16(BBP_PROG_IN_TA);
	mgmt_frame->desc_word[4] = cpu_to_le16(common->endpoint);
	mgmt_frame->desc_word[3] = cpu_to_le16(common->rf_pwr_mode);

	if (common->rf_reset) {
		mgmt_frame->desc_word[7] =  cpu_to_le16(RF_RESET_ENABLE);
		ven_rsi_dbg(MGMT_TX_ZONE, "%s: ===> RF RESET REQUEST SENT <===\n",
			__func__);
		common->rf_reset = 0;
	}
	common->bb_rf_prog_count = 1;
	mgmt_frame->desc_word[7] |= cpu_to_le16(PUT_BBP_RESET |
				     BBP_REG_WRITE | (RSI_RF_TYPE << 4));

	skb_put(skb, FRAME_DESC_SZ);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_set_vap_capabilities() - This function send vap capability to firmware.
 * @common: Pointer to the driver private structure.
 * @opmode: Operating mode of device.
 *
 * Return: 0 on success, corresponding negative error code on failure.
 */
int rsi_set_vap_capabilities(struct rsi_common *common,
			     enum opmode mode,
			     u8 vap_status)
{
	struct sk_buff *skb = NULL;
	struct rsi_vap_caps *vap_caps;
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_hw *hw = adapter->hw;
	struct ieee80211_conf *conf = &hw->conf;
	u16 vap_id = 0;

	ven_rsi_dbg(MGMT_TX_ZONE,
		"%s: Sending VAP capabilities frame\n", __func__);

	skb = dev_alloc_skb(sizeof(struct rsi_vap_caps));
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, sizeof(struct rsi_vap_caps));
	vap_caps = (struct rsi_vap_caps *)skb->data;

	vap_caps->desc_word[0] = cpu_to_le16((sizeof(struct rsi_vap_caps) -
					     FRAME_DESC_SZ) |
					     (RSI_WIFI_MGMT_Q << 12));
	vap_caps->desc_word[1] = cpu_to_le16(VAP_CAPABILITIES);
	vap_caps->desc_word[2] = cpu_to_le16(vap_status << 8);
	vap_caps->desc_word[4] = cpu_to_le16(mode |
					     (common->channel_width << 8));
	vap_caps->desc_word[7] = cpu_to_le16((vap_id << 8) |
					     (common->mac_id << 4) |
					     common->radio_id);

	memcpy(vap_caps->mac_addr, common->mac_addr, IEEE80211_ADDR_LEN);
	vap_caps->keep_alive_period = cpu_to_le16(90);
	vap_caps->frag_threshold = cpu_to_le16(IEEE80211_MAX_FRAG_THRESHOLD);

	vap_caps->rts_threshold = cpu_to_le16(common->rts_threshold);
	vap_caps->default_mgmt_rate = cpu_to_le32(RSI_RATE_6);

#if 0
	if (common->band == NL80211_BAND_5GHZ) {
		vap_caps->default_ctrl_rate = cpu_to_le32(RSI_RATE_6);
		if (conf_is_ht40(&common->priv->hw->conf)) {
			vap_caps->default_ctrl_rate |=
				cpu_to_le32(FULL40M_ENABLE << 16);
		}
	} else {
		vap_caps->default_ctrl_rate = cpu_to_le32(RSI_RATE_1);
		if (conf_is_ht40_minus(conf))
			vap_caps->default_ctrl_rate |=
				cpu_to_le32(UPPER_20_ENABLE << 16);
		else if (conf_is_ht40_plus(conf))
			vap_caps->default_ctrl_rate |=
				cpu_to_le32(LOWER_20_ENABLE << 16);
	}
#else
	if (common->band == NL80211_BAND_5GHZ)
		vap_caps->default_ctrl_rate = cpu_to_le32(RSI_RATE_6);
	else
		vap_caps->default_ctrl_rate = cpu_to_le32(RSI_RATE_1);

	if (conf_is_ht40(conf)) {
		if (conf_is_ht40_minus(conf))
			vap_caps->default_ctrl_rate |=
				cpu_to_le32(UPPER_20_ENABLE << 16);
		else if (conf_is_ht40_plus(conf))
			vap_caps->default_ctrl_rate |=
				cpu_to_le32(LOWER_20_ENABLE << 16);
		else
			vap_caps->default_ctrl_rate |=
				cpu_to_le32(FULL40M_ENABLE << 16);
	}
#endif

	vap_caps->default_data_rate = 0;
	vap_caps->beacon_interval = cpu_to_le16(common->beacon_interval);
	vap_caps->dtim_period = cpu_to_le16(common->dtim_cnt);
//	vap_caps->beacon_miss_threshold = cpu_to_le16(10);
	if (mode == AP_OPMODE)
		vap_caps->beacon_miss_threshold = cpu_to_le16(10);

	skb_put(skb, sizeof(*vap_caps));

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_load_key() - This function is used to load keys within the firmware.
 * @common: Pointer to the driver private structure.
 * @data: Pointer to the key data.
 * @key_len: Key length to be loaded.
 * @key_type: Type of key: GROUP/PAIRWISE.
 * @key_id: Key index.
 * @cipher: Type of cipher used.
 *
 * Return: 0 on success, -1 on failure.
 */
int rsi_load_key(struct rsi_common *common,
		 u8 *data,
		 u16 key_len,
		 u8 key_type,
		 u8 key_id,
		 u32 cipher,
		 s16 sta_id)
{
	struct ieee80211_vif *vif = common->priv->vifs[0];
	struct sk_buff *skb = NULL;
	struct rsi_set_key *set_key;
	u16 key_descriptor = 0;
	u8 key_t1 = 0;
	u8 vap_id = 0;

	ven_rsi_dbg(MGMT_TX_ZONE, "%s: Sending load key frame\n", __func__);

	skb = dev_alloc_skb(sizeof(struct rsi_set_key));
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, sizeof(struct rsi_set_key));
	set_key = (struct rsi_set_key *)skb->data;

	switch (key_type) {
	case RSI_GROUP_KEY:
		key_t1 = 1 << 1;
		if (vif->type == NL80211_IFTYPE_AP)
			key_descriptor = BIT(7);
		break;
	case RSI_PAIRWISE_KEY:
		if ((vif->type == NL80211_IFTYPE_AP) &&
		    (sta_id >= RSI_MAX_ASSOC_STAS)) {
			ven_rsi_dbg(INFO_ZONE, "Invalid Sta_id %d\n", sta_id);
			return -1;
		}
		key_t1 = 0 << 1;
		if ((cipher != WLAN_CIPHER_SUITE_WEP40) &&
		    (cipher != WLAN_CIPHER_SUITE_WEP104))
			key_id = 0;
		break;
	}
	if ((cipher == WLAN_CIPHER_SUITE_WEP40) ||
	    (cipher == WLAN_CIPHER_SUITE_WEP104)) {
		key_descriptor |= BIT(2);
		if (key_len >= 13) {
			key_descriptor |= BIT(3);
		}
	} else if (cipher != KEY_TYPE_CLEAR) {
		key_descriptor |= BIT(4);
		if (cipher == WLAN_CIPHER_SUITE_TKIP)
			key_descriptor |= BIT(5);
	}
	key_descriptor |= (key_t1 | BIT(13) | (key_id << 14));

#if 0
	if ((cipher == WLAN_CIPHER_SUITE_WEP40) ||
	    (cipher == WLAN_CIPHER_SUITE_WEP104)) {
		key_len += 1;
		key_descriptor |= BIT(2);
		if (key_len >= 13)
			key_descriptor |= BIT(3);
	} else if (cipher != KEY_TYPE_CLEAR) {
		key_descriptor |= BIT(4);
		if (key_type == RSI_PAIRWISE_KEY)
			key_id = 0;
		if (cipher == WLAN_CIPHER_SUITE_TKIP)
			key_descriptor |= BIT(5);
	}
	key_descriptor |= (key_type | BIT(13) | (key_id << 14));
#endif

	set_key->desc_word[0] = cpu_to_le16((sizeof(struct rsi_set_key) -
					    FRAME_DESC_SZ) |
					    (RSI_WIFI_MGMT_Q << 12));
	set_key->desc_word[1] = cpu_to_le16(SET_KEY_REQ);
	set_key->desc_word[4] = cpu_to_le16(key_descriptor);
	set_key->desc_word[7] = cpu_to_le16(sta_id | (vap_id << 8));

	if (data) {
		if ((cipher == WLAN_CIPHER_SUITE_WEP40) ||
		    (cipher == WLAN_CIPHER_SUITE_WEP104)) {
			memcpy(&set_key->key[key_id][1], data, key_len * 2);
		} else {
			memcpy(&set_key->key[0][0], data, key_len);
		}
		memcpy(set_key->tx_mic_key, &data[16], 8);
		memcpy(set_key->rx_mic_key, &data[24], 8);
	} else {
		memset(&set_key[FRAME_DESC_SZ], 0,
		       sizeof(struct rsi_set_key) - FRAME_DESC_SZ);				
	}

	skb_put(skb, sizeof(struct rsi_set_key));

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_send_common_dev_params() - This function send the common device
 *				configuration parameters to device.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, -1 on failure.
 */
int rsi_send_common_dev_params(struct rsi_common *common)
{
	struct sk_buff *skb = NULL;
	u32 *soc_gpio, len;
	u16 *frame, *ulp_gpio, *desc;

	ven_rsi_dbg(INFO_ZONE, "Sending common dev config params\n");

	len = 0x20;

	skb = dev_alloc_skb(len + FRAME_DESC_SZ);
	if (!skb)
		return -ENOMEM;
	memset(skb->data, 0, len + FRAME_DESC_SZ);

	desc = (u16 *)&skb->data[0];
	frame = (u16 *)&skb->data[FRAME_DESC_SZ];

	desc[0] = cpu_to_le16(len | (RSI_COEX_Q << 12));
	desc[1] = cpu_to_le16(COMMON_DEV_CONFIG);

	frame[0] = (u16)common->lp_ps_handshake_mode;
	frame[0] |= (u16)common->ulp_ps_handshake_mode << 8;

	ulp_gpio = (u16 *)&unused_ulp_gpio_bitmap;
	soc_gpio = (u32 *)&unused_soc_gpio_bitmap;

	frame[1] |= (*ulp_gpio) << 8;
	*(u32 *)&frame[2] = *soc_gpio;
	frame[4] |= cpu_to_le16((u16)common->oper_mode << 8);
	frame[5] |= cpu_to_le16((u16)common->wlan_rf_power_mode);
	frame[5] |= cpu_to_le16((u16)common->bt_rf_power_mode << 8);
	frame[6] |= cpu_to_le16((u16)common->driver_mode << 8);
	frame[7] = cpu_to_le16(3); //((u16 )d_assets->region_code);
	frame[7] |= cpu_to_le16((u16)common->obm_ant_sel_val << 8);

	skb_put(skb, len + FRAME_DESC_SZ);

	return rsi_send_internal_mgmt_frame(common, skb);
}

#if 0
int rsi_send_common_dev_params(struct rsi_common *common)
{
	struct sk_buff *skb = NULL;
	u32 *unused_soc_gpio;
	u32 frame_len = 0;
	struct rsi_config_vals *dev_cfgs = NULL;

	frame_len = sizeof(struct rsi_config_vals);

	ven_rsi_dbg(MGMT_TX_ZONE,
		"%s: Sending common device config params frame\n",
		__func__);
	skb = dev_alloc_skb(frame_len);
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Unable to allocate skb\n", __func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, frame_len);

	dev_cfgs = (struct rsi_config_vals *)&skb->data[0];
	memset(dev_cfgs, 0, (sizeof(struct rsi_config_vals)));

	dev_cfgs->desc_word[0] = cpu_to_le16((frame_len - FRAME_DESC_SZ) |
					     (RSI_COEX_Q << 12));
	dev_cfgs->desc_word[1] = cpu_to_le16(COMMON_DEV_CONFIG);

	dev_cfgs->lp_ps_handshake = common->lp_ps_handshake_mode;
	dev_cfgs->ulp_ps_handshake = common->ulp_ps_handshake_mode;

	if (common->host_wakeup_intr_enable) {
		dev_cfgs->sleep_config_params |=
			common->host_wakeup_intr_enable;
		dev_cfgs->sleep_config_params |= BIT(2);
		if (common->host_wakeup_intr_active_high)
			dev_cfgs->sleep_config_params |= BIT(3);
	}

	dev_config_vals[0].opermode = common->coex_mode;

	if (dev_config_vals[0].ext_pa_or_bt_coex_en)
		dev_cfgs->ext_pa_or_bt_coex_en =
			dev_config_vals[0].ext_pa_or_bt_coex_en;
	dev_cfgs->opermode = dev_config_vals[0].opermode;
	dev_cfgs->wlan_rf_pwr_mode = common->wlan_rf_power_mode;
	dev_cfgs->driver_mode = common->driver_mode;
	dev_cfgs->region_code = 0; /* Default US */
	dev_cfgs->antenna_sel_val = common->obm_ant_sel_val;

	unused_soc_gpio = (u32 *)&unused_soc_gpio_bitmap;
	dev_cfgs->unused_soc_gpio_bitmap = *unused_soc_gpio;

	skb_put(skb, frame_len);

	rsi_hex_dump(ERR_ZONE, "common dev config params ",
		     skb->data, skb->len);
	return rsi_send_internal_mgmt_frame(common, skb);
}
#endif

/*
 * rsi_load_bootup_params() - This function send bootup params to the firmware.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
static int rsi_load_bootup_params(struct rsi_common *common)
{
	struct sk_buff *skb;
	struct rsi_boot_params *boot_params;

	ven_rsi_dbg(MGMT_TX_ZONE, "%s: Sending boot params frame\n", __func__);
	skb = dev_alloc_skb(sizeof(struct rsi_boot_params));
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, sizeof(struct rsi_boot_params));
	boot_params = (struct rsi_boot_params *)skb->data;

	ven_rsi_dbg(MGMT_TX_ZONE, "%s:\n", __func__);

	if (common->channel_width == BW_40MHZ) {
		memcpy(&boot_params->bootup_params,
		       &boot_params_40,
		       sizeof(struct bootup_params));
		ven_rsi_dbg(MGMT_TX_ZONE,
			"%s: Packet 40MHZ <=== %d\n", __func__,
			UMAC_CLK_40BW);
		boot_params->desc_word[7] = cpu_to_le16(UMAC_CLK_40BW);
	} else {
		memcpy(&boot_params->bootup_params,
		       &boot_params_20,
		       sizeof(struct bootup_params));
		if (boot_params_20.valid != cpu_to_le32(VALID_20)) {
			boot_params->desc_word[7] = cpu_to_le16(UMAC_CLK_20BW);
			ven_rsi_dbg(MGMT_TX_ZONE,
				"%s: Packet 20MHZ <=== %d\n", __func__,
				UMAC_CLK_20BW);
		} else {
			boot_params->desc_word[7] = cpu_to_le16(UMAC_CLK_40MHZ);
			ven_rsi_dbg(MGMT_TX_ZONE,
				"%s: Packet 20MHZ <=== %d\n", __func__,
				UMAC_CLK_40MHZ);
		}
	}

	/**
	 * Bit{0:11} indicates length of the Packet
	 * Bit{12:15} indicates host queue number
	 */
	boot_params->desc_word[0] = cpu_to_le16(sizeof(struct bootup_params) |
				    (RSI_WIFI_MGMT_Q << 12));
	boot_params->desc_word[1] = cpu_to_le16(BOOTUP_PARAMS_REQUEST);

	skb_put(skb, sizeof(struct rsi_boot_params));

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_send_reset_mac() - This function prepares reset MAC request and sends an
 *			  internal management frame to indicate it to firmware.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
static int rsi_send_reset_mac(struct rsi_common *common)
{
	struct sk_buff *skb;
	struct rsi_mac_frame *mgmt_frame;

	ven_rsi_dbg(MGMT_TX_ZONE, "%s: Sending reset MAC frame\n", __func__);

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = cpu_to_le16(RESET_MAC_REQ);
#ifdef BYPASS_RX_DATA_PATH
	mgmt_frame->desc_word[4] = cpu_to_le16(0x0001);
#endif
	mgmt_frame->desc_word[4] |= cpu_to_le16(RETRY_COUNT << 8);

	/*TA level aggregation of pkts to host */
	mgmt_frame->desc_word[3] |=  common->ta_aggr << 8;

	if (common->antenna_diversity)
		mgmt_frame->desc_word[6] = common->antenna_diversity;

	skb_put(skb, FRAME_DESC_SZ);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_band_check() - This function programs the band
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
int rsi_band_check(struct rsi_common *common)
{
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_hw *hw = adapter->hw;
	u8 prev_bw = common->channel_width;
	u8 prev_ep = common->endpoint;
	struct ieee80211_channel *curchan = hw->conf.chandef.chan;
	int status = 0;

	if (common->band != curchan->band) {
		common->rf_reset = 1;
		common->band = curchan->band;
	}

	if ((hw->conf.chandef.width == NL80211_CHAN_WIDTH_20_NOHT) ||
	    (hw->conf.chandef.width == NL80211_CHAN_WIDTH_20))
		common->channel_width = BW_20MHZ;
	else
		common->channel_width = BW_40MHZ;

	if (common->band == NL80211_BAND_2GHZ) {
		if (common->channel_width)
			common->endpoint = EP_2GHZ_40MHZ;
		else
			common->endpoint = EP_2GHZ_20MHZ;
	} else {
		if (common->channel_width)
			common->endpoint = EP_5GHZ_40MHZ;
		else
			common->endpoint = EP_5GHZ_20MHZ;
	}

	if (common->endpoint != prev_ep) {
		status = rsi_program_bb_rf(common);
		if (status)
			return status;
	}

	if (common->channel_width != prev_bw) {
		status = rsi_load_bootup_params(common);
		if (status)
			return status;

		status = rsi_load_radio_caps(common);
		if (status)
			return status;
	}

	return status;
}

/**
 * rsi_set_channel() - This function programs the channel.
 * @common: Pointer to the driver private structure.
 * @channel: Channel value to be set.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
int rsi_set_channel(struct rsi_common *common,
		    struct ieee80211_channel *channel)
{
	struct sk_buff *skb = NULL;
	struct rsi_mac_frame *mgmt_frame;

	ven_rsi_dbg(MGMT_TX_ZONE,
		"%s: Sending scan req frame\n", __func__);

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	if (!channel) {
		dev_kfree_skb(skb);
		return 0;
	}
	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = cpu_to_le16(SCAN_REQUEST);
	mgmt_frame->desc_word[4] = cpu_to_le16(channel->hw_value);

	mgmt_frame->desc_word[4] |=
		cpu_to_le16(((char)(channel->max_antenna_gain)) << 8);
	mgmt_frame->desc_word[5] =
		cpu_to_le16((char)(channel->max_antenna_gain));

	mgmt_frame->desc_word[7] = cpu_to_le16(PUT_BBP_RESET |
					       BBP_REG_WRITE |
					       (RSI_RF_TYPE << 4));

	if ((channel->flags & IEEE80211_CHAN_NO_IR) ||
	    (channel->flags & IEEE80211_CHAN_RADAR)) {
		mgmt_frame->desc_word[4] |= BIT(15);
	} else {
		if (common->tx_power < channel->max_power)
			mgmt_frame->desc_word[6] =
				cpu_to_le16(common->tx_power);
		else
			mgmt_frame->desc_word[6] =
				cpu_to_le16(channel->max_power);
	}
	mgmt_frame->desc_word[7] = cpu_to_le16(common->priv->dfs_region);

	if (common->channel_width == BW_40MHZ)
		mgmt_frame->desc_word[5] |= cpu_to_le16(0x1 << 8);

	common->channel = channel->hw_value;

	skb_put(skb, FRAME_DESC_SZ);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_send_radio_params_update() - This function sends the radio
 *				parameters update to device
 * @common: Pointer to the driver private structure.
 * @channel: Channel value to be set.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
int rsi_send_radio_params_update(struct rsi_common *common)
{
	struct rsi_mac_frame *mgmt_frame;
	struct sk_buff *skb = NULL;

	ven_rsi_dbg(MGMT_TX_ZONE,
		"%s: Sending Radio Params update frame\n", __func__);

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = cpu_to_le16(RADIO_PARAMS_UPDATE);
	mgmt_frame->desc_word[3] = cpu_to_le16(BIT(0));

	mgmt_frame->desc_word[3] |= cpu_to_le16(common->tx_power << 8);

	skb_put(skb, FRAME_DESC_SZ);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_send_vap_dynamic_update() - This function programs the threshold.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
int rsi_send_vap_dynamic_update(struct rsi_common *common)
{
	struct sk_buff *skb = NULL;
	struct rsi_dynamic_s *dynamic_frame = NULL;

	ven_rsi_dbg(MGMT_TX_ZONE,
		"%s: Sending vap update indication frame\n", __func__);

	skb = dev_alloc_skb(sizeof(struct rsi_dynamic_s));
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, sizeof(struct rsi_dynamic_s));
	dynamic_frame = (struct rsi_dynamic_s *)skb->data;

	dynamic_frame->desc_word[0] = cpu_to_le16(
					(sizeof(dynamic_frame->frame_body)) |
					(RSI_WIFI_MGMT_Q << 12));
	dynamic_frame->desc_word[1] = cpu_to_le16(VAP_DYNAMIC_UPDATE);
	dynamic_frame->desc_word[4] = cpu_to_le16(common->rts_threshold);
#if 0
	dynamic_frame->desc_word[5] = cpu_to_le16(common->frag_threshold);
	dynamic_frame->desc_word[5] = cpu_to_le16(2352);
#endif

#ifdef CONFIG_RSI_WOW
	dynamic_frame->desc_word[6] = cpu_to_le16(24); /* bmiss_threshold */
	dynamic_frame->frame_body.keep_alive_period = cpu_to_le16(10);
#else
	dynamic_frame->frame_body.keep_alive_period = cpu_to_le16(90);
#endif

#if 0
	dynamic_frame->frame_body.mgmt_rate = cpu_to_le32(RSI_RATE_6);

	dynamic_frame->desc_word[2] |= cpu_to_le32(BIT(1));/* Self cts enable */

	dynamic_frame->desc_word[3] |= cpu_to_le16(BIT(0));/* fixed rate */
	dynamic_frame->frame_body.data_rate = cpu_to_le16(0);
#endif

	dynamic_frame->desc_word[7] |= cpu_to_le16((0 << 8)); /* vap id */

	skb_put(skb, sizeof(struct rsi_dynamic_s));

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_flash_read() - This function sends the frash read frame to device
 * @adapter: Pointer to the hardware structure.
 *
 * Return: status: 0 on success, -1 on failure.
 */
int rsi_flash_read(struct rsi_hw *adapter)
{
	struct rsi_common *common = adapter->priv;
	struct rsi_mac_frame *cmd_frame = NULL;
	struct sk_buff *skb;

	ven_rsi_dbg(MGMT_TX_ZONE, "%s: Sending flash read frame\n", __func__);

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb)
		return -ENOMEM;

	memset(skb->data, 0, FRAME_DESC_SZ);
	cmd_frame = (struct rsi_mac_frame *)skb->data;

	/* FrameType */
	cmd_frame->desc_word[1] = cpu_to_le16(EEPROM_READ);

	/* Format of length and offset differs for
	 * autoflashing and swbl flashing
	 */
	cmd_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);

	/* Number of bytes to read */
	ven_rsi_dbg(INFO_ZONE, " eeprom length  0x%x, %d\n",
		adapter->eeprom.length, adapter->eeprom.length);
	cmd_frame->desc_word[3] = cpu_to_le16(adapter->eeprom.length << 4);

	cmd_frame->desc_word[2] |= cpu_to_le16(3 << 8);
	if (adapter->eeprom_init) {
		ven_rsi_dbg(INFO_ZONE, "spi init sent");
		cmd_frame->desc_word[2] |= cpu_to_le16(BIT(13));
	}

	/* Address to read */
	cmd_frame->desc_word[4] = cpu_to_le16(adapter->eeprom.offset);
	cmd_frame->desc_word[5] = cpu_to_le16(adapter->eeprom.offset >> 16);
	cmd_frame->desc_word[6] = cpu_to_le16(0); //delay = 0

	skb_put(skb, FRAME_DESC_SZ);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_compare() - This function is used to compare two integers
 * @a: pointer to the first integer
 * @b: pointer to the second integer
 *
 * Return: 0 if both are equal, -1 if the first is smaller, else 1
 */
static int rsi_compare(const void *a, const void *b)
{
	u16 _a = *(const u16 *)(a);
	u16 _b = *(const u16 *)(b);

	if (_a > _b)
		return -1;

	if (_a < _b)
		return 1;

	return 0;
}

/**
 * rsi_map_rates() - This function is used to map selected rates to hw rates.
 * @rate: The standard rate to be mapped.
 * @offset: Offset that will be returned.
 *
 * Return: 0 if it is a mcs rate, else 1
 */
static bool rsi_map_rates(u16 rate, int *offset)
{
	int kk;

	for (kk = 0; kk < ARRAY_SIZE(rsi_mcsrates); kk++) {
		if (rate == mcs[kk]) {
			*offset = kk;
			return false;
		}
	}

	for (kk = 0; kk < ARRAY_SIZE(rsi_rates); kk++) {
		if (rate == rsi_rates[kk].bitrate / 5) {
			*offset = kk;
			break;
		}
	}
	return true;
}

/**
 * rsi_send_auto_rate_request() - This function is to set rates for connection
 *				  and send autorate request to firmware.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
static int rsi_send_auto_rate_request(struct rsi_common *common,
				      struct ieee80211_sta *sta,
				      u16 sta_id)
{
	struct ieee80211_vif *vif = common->priv->vifs[0];
	struct sk_buff *skb;
	struct rsi_auto_rate *auto_rate;
	int ii = 0, jj = 0, kk = 0;
	struct ieee80211_hw *hw = common->priv->hw;
	u8 band = hw->conf.chandef.chan->band;
	u8 num_supported_rates = 0;
	u8 rate_table_offset, rate_offset = 0;
	u32 rate_bitmap = 0;
	u16 *selected_rates, min_rate;
	bool is_ht = false, is_sgi = false;

	ven_rsi_dbg(MGMT_TX_ZONE,
		"%s: Sending auto rate request frame\n", __func__);

	skb = dev_alloc_skb(MAX_MGMT_PKT_SIZE);
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, MAX_MGMT_PKT_SIZE);

	selected_rates = kzalloc(2 * RSI_TBL_SZ, GFP_KERNEL);
	if (!selected_rates) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of mem\n",
			__func__);
		dev_kfree_skb(skb);
		return -ENOMEM;
	}
	memset(selected_rates, 0, 2 * RSI_TBL_SZ);

	auto_rate = (struct rsi_auto_rate *)skb->data;

	auto_rate->aarf_rssi = cpu_to_le16(((u16)3 << 6) | (u16)(18 & 0x3f));
	auto_rate->collision_tolerance = cpu_to_le16(3);
	auto_rate->failure_limit = cpu_to_le16(3);
	auto_rate->initial_boundary = cpu_to_le16(3);
	auto_rate->max_threshold_limt = cpu_to_le16(27);

	auto_rate->desc_word[1] = cpu_to_le16(AUTO_RATE_IND);

	if (common->channel_width == BW_40MHZ)
		auto_rate->desc_word[7] = cpu_to_le16(1);
	auto_rate->desc_word[7] |= cpu_to_le16(sta_id << 8);

	if (vif->type == NL80211_IFTYPE_STATION) {
		rate_bitmap = common->bitrate_mask[band];
		is_ht = common->vif_info[0].is_ht;
		is_sgi = common->vif_info[0].sgi;
	} else {
		rate_bitmap = sta->supp_rates[band];
		is_ht = sta->ht_cap.ht_supported;
		if ((sta->ht_cap.cap & IEEE80211_HT_CAP_SGI_20) ||
		    (sta->ht_cap.cap & IEEE80211_HT_CAP_SGI_40))
			is_sgi = true;
	}
	printk("rate_bitmap = %x\n", rate_bitmap);
	printk("is_ht = %d\n", is_ht);

	if (band == NL80211_BAND_2GHZ) {
		if ((rate_bitmap == 0) && (is_ht))
			min_rate = RSI_RATE_MCS0;
		else
			min_rate = RSI_RATE_1;
		rate_table_offset = 0;
	} else {
		if ((rate_bitmap == 0) && (is_ht))
			min_rate = RSI_RATE_MCS0;
		else
			min_rate = RSI_RATE_6;
		rate_table_offset = 4;
	}

	for (ii = 0, jj = 0;
	     ii < (ARRAY_SIZE(rsi_rates) - rate_table_offset); ii++) {
		if (rate_bitmap & BIT(ii)) {
			selected_rates[jj++] =
			(rsi_rates[ii + rate_table_offset].bitrate / 5);
			rate_offset++;
		}
	}
	num_supported_rates = jj;

	if (is_ht) {
		for (ii = 0; ii < ARRAY_SIZE(mcs); ii++)
			selected_rates[jj++] = mcs[ii];
		num_supported_rates += ARRAY_SIZE(mcs);
		rate_offset += ARRAY_SIZE(mcs);
	}

	sort(selected_rates, jj, sizeof(u16), &rsi_compare, NULL);

	/* mapping the rates to RSI rates */
	for (ii = 0; ii < jj; ii++) {
		if (rsi_map_rates(selected_rates[ii], &kk)) {
			auto_rate->supported_rates[ii] =
				cpu_to_le16(rsi_rates[kk].hw_value);
		} else {
			auto_rate->supported_rates[ii] =
				cpu_to_le16(rsi_mcsrates[kk]);
		}
	}

	/* loading HT rates in the bottom half of the auto rate table */
	if (is_ht) {
		for (ii = rate_offset, kk = ARRAY_SIZE(rsi_mcsrates) - 1;
		     ii < rate_offset + 2 * ARRAY_SIZE(rsi_mcsrates); ii++) {
			if (is_sgi || conf_is_ht40(&common->priv->hw->conf)) {
				auto_rate->supported_rates[ii++] =
					cpu_to_le16(rsi_mcsrates[kk] | BIT(9));
			} else {
				auto_rate->supported_rates[ii++] =
					cpu_to_le16(rsi_mcsrates[kk]);
			}
			auto_rate->supported_rates[ii] =
				cpu_to_le16(rsi_mcsrates[kk--]);
		}

		for (; ii < (RSI_TBL_SZ - 1); ii++) {
			auto_rate->supported_rates[ii] =
				cpu_to_le16(rsi_mcsrates[0]);
		}
	}

	for (; ii < RSI_TBL_SZ; ii++)
		auto_rate->supported_rates[ii] = cpu_to_le16(min_rate);

	auto_rate->num_supported_rates = cpu_to_le16(num_supported_rates * 2);
	auto_rate->moderate_rate_inx = cpu_to_le16(num_supported_rates / 2);
	num_supported_rates *= 2;

	auto_rate->desc_word[0] = cpu_to_le16((sizeof(*auto_rate) -
					      FRAME_DESC_SZ) |
					      (RSI_WIFI_MGMT_Q << 12));

	skb_put(skb, sizeof(struct rsi_auto_rate));
	kfree(selected_rates);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_validate_bgscan_channels() - This function is used to validate
 *				the user configured bgscan channels for
 *				current regulatory domain
 * @chn_num: It holds the user or default channel for validation.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
static void rsi_validate_bgscan_channels(struct rsi_hw *adapter,
					 struct bgscan_config_params *params)
{
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel *ch;
	struct wiphy *wiphy = adapter->hw->wiphy; 
	u16 bgscan_channels[MAX_BGSCAN_CHANNELS] = {1, 2, 3, 4, 5, 6, 7, 8, 9,
						    10, 11, 12, 13, 14, 36, 40,
						    44, 48, 52, 56, 60, 64, 100,
						    104, 108, 112, 116, 120, 124,
						    128, 132, 136, 140, 149, 153,
						    157, 161, 165};

	int ch_num, i;
	int num_valid_chs = 0, cnt;

	/* If user passes 0 for num of bgscan channels, take all channels */
	if (params->num_user_channels == 0) {
		params->num_user_channels = MAX_BGSCAN_CHANNELS;
		for (cnt = 0; cnt < MAX_BGSCAN_CHANNELS; cnt++)
			params->user_channels[cnt] = bgscan_channels[cnt];
	}

	ven_rsi_dbg(INFO_ZONE, "Final bgscan channels:\n");
	for (cnt = 0; cnt < params->num_user_channels; cnt++) {
		ch_num = params->user_channels[cnt];

		if ((ch_num < 1) ||
		    ((ch_num > 14) && (ch_num < 36)) ||
		    ((ch_num > 64) && (ch_num < 100)) ||
		    ((ch_num > 140) && (ch_num < 149)) ||
		    (ch_num > 165))
			continue;
		if ((ch_num >= 36) && (ch_num < 149) && (ch_num % 4))
			continue;

		if (ch_num > 14)
			sband = wiphy->bands[NL80211_BAND_5GHZ];
		else
			sband = wiphy->bands[NL80211_BAND_2GHZ];

		for (i = 0; i < sband->n_channels; i++) {
			ch = &sband->channels[i];

			if (ch->hw_value == ch_num)
				break;
		}
		if (i >= sband->n_channels)
			continue;

		/* Check channel availablity for the current reg domain */
		if (ch->flags & IEEE80211_CHAN_DISABLED)
			continue;

		params->channels2scan[num_valid_chs] = ch_num;
		printk("%d ", ch_num);
		if ((ch->flags & IEEE80211_CHAN_NO_IR) ||
		    (ch->flags & IEEE80211_CHAN_RADAR)) {
			printk("[DFS]");
			params->channels2scan[num_valid_chs] |=
				(cpu_to_le16(BIT(15))); /* DFS indication */
		}
		num_valid_chs++;
		printk(" ");
	}
	printk("\n");
	params->num_bg_channels = num_valid_chs;
}

/**
 * rsi_send_bgscan_params() - This function sends the background
 *			      scan parameters to firmware.
 * @common: Pointer to the driver private structure.
 * @enable: bgscan enable/disable
 *
 * Return: 0 on success, corresponding error code on failure.
 */
int rsi_send_bgscan_params(struct rsi_common *common, int enable)
{
	struct rsi_bgscan_params *bgscan;
	struct bgscan_config_params *info = &common->bgscan_info;
	struct sk_buff *skb;
	u16 frame_len = sizeof(*bgscan);

	ven_rsi_dbg(MGMT_TX_ZONE, "%s: Sending bgscan params frame\n", __func__);

	rsi_validate_bgscan_channels(common->priv, info);
	if (!info->num_bg_channels) {
		ven_rsi_dbg(ERR_ZONE, "##### No valid bgscan channels #####\n");
		return -1;
	}
	
	skb = dev_alloc_skb(frame_len);
	if (!skb)
		return -ENOMEM;
	memset(skb->data, 0, frame_len);

	bgscan = (struct rsi_bgscan_params *)skb->data;

	bgscan->desc_word[0] = cpu_to_le16((frame_len - FRAME_DESC_SZ) |
					   (RSI_WIFI_MGMT_Q << 12));
	bgscan->desc_word[1] = cpu_to_le16(BG_SCAN_PARAMS);

	bgscan->bgscan_threshold = cpu_to_le16(info->bgscan_threshold);
	bgscan->roam_threshold = cpu_to_le16(info->roam_threshold);
	if (enable)
		bgscan->bgscan_periodicity =
			cpu_to_le16(info->bgscan_periodicity);
	bgscan->active_scan_duration =
			cpu_to_le16(info->active_scan_duration);
	bgscan->passive_scan_duration =
			cpu_to_le16(info->passive_scan_duration);
	bgscan->two_probe = info->two_probe;

	memcpy(bgscan->channels2scan,
	       info->channels2scan,
	       info->num_bg_channels * 2);
	bgscan->num_bg_channels = info->num_bg_channels;

	skb_put(skb, frame_len);

	rsi_hex_dump(MGMT_TX_ZONE, "bgscan params req", skb->data, skb->len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_send_bgscan_probe_req() - This function sends the background
 *                               scan probe request to firmware.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, corresponding error code on failure.
 */
int rsi_send_bgscan_probe_req(struct rsi_common *common)
{
	struct rsi_bgscan_probe *bgscan;
	struct sk_buff *skb;
	u16 frame_len = sizeof(*bgscan);
	u16 len = 1500;
	u16 pbreq_len = 0;

	ven_rsi_dbg(MGMT_TX_ZONE,
		"%s: Sending bgscan probe req frame\n", __func__);

	skb = dev_alloc_skb(frame_len + len);
	if (!skb)
		return -ENOMEM;
	memset(skb->data, 0, frame_len + len);

	bgscan = (struct rsi_bgscan_probe *)skb->data;

	bgscan->desc_word[1] = cpu_to_le16(BG_SCAN_PROBE_REQ);

	if (common->band == NL80211_BAND_5GHZ) {
		bgscan->mgmt_rate = cpu_to_le16(RSI_RATE_6);
		bgscan->channel_num = cpu_to_le16(40);
	} else {
		bgscan->mgmt_rate = cpu_to_le16(RSI_RATE_1);
		bgscan->channel_num = cpu_to_le16(11);
	}

	bgscan->channel_scan_time = cpu_to_le16(20);
	if (common->bgscan_probe_req_len > 0) {
		pbreq_len = common->bgscan_probe_req_len;
		bgscan->probe_req_length = pbreq_len;
		memcpy(&skb->data[frame_len], common->bgscan_probe_req,
		       common->bgscan_probe_req_len);
	}

	bgscan->desc_word[0] = cpu_to_le16((frame_len - FRAME_DESC_SZ + pbreq_len) |
					   (RSI_WIFI_MGMT_Q << 12));

	skb_put(skb, frame_len + pbreq_len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_inform_bss_status() - This function informs about bss status with the
 *			     help of sta notify params by sending an internal
 *			     management frame to firmware.
 * @common: Pointer to the driver private structure.
 * @status: Bss status type.
 * @bssid: Bssid.
 * @qos_enable: Qos is enabled.
 * @aid: Aid (unique for all STAs).
 *
 * Return: None.
 */
void rsi_inform_bss_status(struct rsi_common *common,
			   enum opmode opmode,
			   u8 status,
			   u8 *bssid,
			   u8 qos_enable,
			   u16 aid,
			   struct ieee80211_sta *sta,
			   u16 sta_id)
{
	if (status) {
		if (opmode == STA_OPMODE)
			common->hw_data_qs_blocked = true;
		rsi_send_sta_notify_frame(common,
					  opmode,
					  STA_CONNECTED,
					  bssid,
					  qos_enable,
					  aid,
					  sta_id);
		if (common->min_rate == 0xffff) {
			ven_rsi_dbg(INFO_ZONE, "Send auto rate request\n");
			rsi_send_auto_rate_request(common, sta, sta_id);
		}
		if (opmode == STA_OPMODE) {
			if ((!common->secinfo.security_enable) ||
			    (rsi_is_cipher_wep(common))) {
				if (!rsi_send_block_unblock_frame(common, false))
					common->hw_data_qs_blocked = false;
			}
		}
	} else {
		if (opmode == STA_OPMODE)
			common->hw_data_qs_blocked = true;
#ifdef CONFIG_RSI_WOW
		if (!common->suspend_flag) {
#endif
		rsi_send_sta_notify_frame(common,
					  opmode,
					  STA_DISCONNECTED,
					  bssid,
					  qos_enable,
					  aid,
					  sta_id);
#ifdef CONFIG_RSI_WOW
		}
#endif
		if (opmode == STA_OPMODE)
			rsi_send_block_unblock_frame(common, true);
	}
}

/**
 * rsi_eeprom_read() - This function sends a frame to read the mac address
 *		       from the eeprom.
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_eeprom_read(struct rsi_common *common)
{
	struct rsi_mac_frame *mgmt_frame = NULL;
	struct rsi_hw *adapter = common->priv;
	struct sk_buff *skb;

	ven_rsi_dbg(MGMT_TX_ZONE,
		"%s: Sending EEPROM read req frame\n", __func__);

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	/* FrameType */
	mgmt_frame->desc_word[1] = cpu_to_le16(EEPROM_READ);
	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);

	/* Number of bytes to read */
	mgmt_frame->desc_word[3] = cpu_to_le16(adapter->eeprom.length << 4);
	mgmt_frame->desc_word[2] |= cpu_to_le16(3 << 8);

	/* Address to read*/
	mgmt_frame->desc_word[4] = cpu_to_le16(adapter->eeprom.offset);
	mgmt_frame->desc_word[5] = cpu_to_le16(adapter->eeprom.offset >> 16);
	mgmt_frame->desc_word[6] = cpu_to_le16(0); //delay = 0

	skb_put(skb, FRAME_DESC_SZ);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * This function sends a frame to block/unblock
 * data queues in the firmware
 *
 * @param common Pointer to the driver private structure.
 * @param block event - block if true, unblock if false
 * @return 0 on success, -1 on failure.
 */
int rsi_send_block_unblock_frame(struct rsi_common *common, bool block_event)
{
	struct rsi_mac_frame *mgmt_frame;
	struct sk_buff *skb;

	ven_rsi_dbg(MGMT_TX_ZONE, "%s: Sending block/unblock frame\n", __func__);

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = cpu_to_le16(BLOCK_HW_QUEUE);
	mgmt_frame->desc_word[3] = cpu_to_le16(0x1);

	if (block_event == true) {
		ven_rsi_dbg(INFO_ZONE, "blocking the data qs\n");
		mgmt_frame->desc_word[4] = cpu_to_le16(0xf);
		mgmt_frame->desc_word[4] |= cpu_to_le16(0xf << 4);
	} else {
		ven_rsi_dbg(INFO_ZONE, "unblocking the data qs\n");
		mgmt_frame->desc_word[5] = cpu_to_le16(0xf);
		mgmt_frame->desc_word[5] |= cpu_to_le16(0xf << 4);
	}

	skb_put(skb, FRAME_DESC_SZ);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * This function sends a frame to filter the RX packets
 *
 * @param common Pointer to the driver private structure.
 * @param rx_filter_word - Flags of filter packets
 * @return 0 on success, -1 on failure.
 */
int rsi_send_rx_filter_frame(struct rsi_common *common, u16 rx_filter_word)
{
	struct rsi_mac_frame *mgmt_frame;
	struct sk_buff *skb;

	ven_rsi_dbg(MGMT_TX_ZONE, "%s: Sending RX filter frame\n", __func__);

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
	mgmt_frame->desc_word[1] = cpu_to_le16(SET_RX_FILTER);
	mgmt_frame->desc_word[4] = cpu_to_le16(rx_filter_word);

	skb_put(skb, FRAME_DESC_SZ);

	return rsi_send_internal_mgmt_frame(common, skb);
}
EXPORT_SYMBOL_GPL(rsi_send_rx_filter_frame); 

/**
 * rsi_send_ps_request() - Sends power save request.
 *
 * @adapter: pointer to rsi_hw structure.
 * @enable: enable or disable power save.
 *
 * returns: 0 on success, negative error code on failure
 */
int rsi_send_ps_request(struct rsi_hw *adapter, bool enable)
{
	struct rsi_common *common = adapter->priv;
	struct ieee80211_bss_conf *bss = &adapter->vifs[0]->bss_conf;
	struct rsi_request_ps *ps = NULL;
	struct rsi_ps_info *ps_info = NULL;
	struct sk_buff *skb = NULL;
	int frame_len = sizeof(*ps);

	skb = dev_alloc_skb(frame_len);
	if (!skb)
		return -ENOMEM;
	memset(skb->data, 0, frame_len);

	ps = (struct rsi_request_ps *)&skb->data[0];
	ps_info = &adapter->ps_info;

	ps->desc_word[0] = cpu_to_le16((frame_len - FRAME_DESC_SZ) |
				       (RSI_WIFI_MGMT_Q << 12));
	ps->desc_word[1] = cpu_to_le16(WAKEUP_SLEEP_REQUEST);
	if (enable) {
		ps->ps_sleep.enable = 1;
		ps->desc_word[6] = SLEEP_REQUEST;
	} else {
		ps->ps_sleep.enable = 0;
		ps->desc_word[0] |= BIT(15);
		ps->desc_word[6] = WAKEUP_REQUEST;
	}

	if (common->uapsd_bitmap) {
//		ps->ps_mimic_support = 1;
		ps->ps_uapsd_acs = common->uapsd_bitmap;
	}

	ps->ps_sleep.sleep_type = ps_info->sleep_type;
	ps->ps_sleep.num_bcns_per_lis_int =
		cpu_to_le16(ps_info->num_bcns_per_lis_int);
	ps->ps_sleep.sleep_duration =
		cpu_to_le32(ps_info->deep_sleep_wakeup_period);

	if (bss->assoc)
		ps->ps_sleep.connected_sleep = CONNECTED_SLEEP;
	else
		ps->ps_sleep.connected_sleep = DEEP_SLEEP;

	ps->ps_listen_interval = cpu_to_le32(ps_info->listen_interval);
	ps->ps_dtim_interval_duration =
		cpu_to_le32(ps_info->dtim_interval_duration);
	ps->ps_num_dtim_intervals = cpu_to_le32(ps_info->num_dtims_per_sleep);

	skb_put(skb, frame_len);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_set_antenna() - This fuction handles antenna selection functionality.
 *
 * @common: Pointer to the driver private structure.
 * @antenna: bitmap for tx antenna selection
 *
 * Return: 0 on Success, < 0 on failure
 */
int rsi_set_antenna(struct rsi_common *common,
		    u8 antenna)
{
	struct rsi_mac_frame *mgmt_frame;
	struct sk_buff *skb;

	skb = dev_alloc_skb(FRAME_DESC_SZ);
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
			__func__);
		return -ENOMEM;
	}

	memset(skb->data, 0, FRAME_DESC_SZ);
	mgmt_frame = (struct rsi_mac_frame *)skb->data;

	mgmt_frame->desc_word[1] = cpu_to_le16(ANT_SEL_FRAME);
	mgmt_frame->desc_word[3] = cpu_to_le16(antenna & 0x00ff);
	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);

	skb_put(skb, FRAME_DESC_SZ);

	return rsi_send_internal_mgmt_frame(common, skb);
}

/**
 * rsi_handle_ta_confirm() - This function handles the confirm frames.
 * @common: Pointer to the driver private structure.
 * @msg: Pointer to received packet.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_handle_ta_confirm(struct rsi_common *common, u8 *msg)
{
	struct rsi_hw *adapter = common->priv;
	u8 sub_type = (msg[15] & 0xff);

	ven_rsi_dbg(MGMT_RX_ZONE, "%s: subtype=%d\n", __func__, sub_type);

	switch (sub_type) {
	case COMMON_DEV_CONFIG:
		ven_rsi_dbg(FSM_ZONE,
			"Common Dev Config params confirm received\n");
		if (common->fsm_state == FSM_COMMON_DEV_PARAMS_SENT) {
			if (rsi_load_bootup_params(common)) {
				common->fsm_state = FSM_CARD_NOT_READY;
				goto out;
			} else {
				common->fsm_state = FSM_BOOT_PARAMS_SENT;
			}
		} else {
			ven_rsi_dbg(INFO_ZONE,
				"%s: Received common dev config params cfm in %d state\n",
				 __func__, common->fsm_state);
			return 0;
		}
		break;

	case BOOTUP_PARAMS_REQUEST:
		ven_rsi_dbg(FSM_ZONE, "Bootup params confirmation.\n");
		if (common->fsm_state == FSM_BOOT_PARAMS_SENT) {
			adapter->eeprom.length = (IEEE80211_ADDR_LEN +
						  WLAN_MAC_MAGIC_WORD_LEN +
						  WLAN_HOST_MODE_LEN);
			adapter->eeprom.offset = WLAN_MAC_EEPROM_ADDR;
			if (rsi_eeprom_read(common)) {
				common->fsm_state = FSM_CARD_NOT_READY;
				goto out;
			} else
				common->fsm_state = FSM_EEPROM_READ_MAC_ADDR;
		} else {
			ven_rsi_dbg(INFO_ZONE,
				"%s: Received bootup params cfm in %d state\n",
				 __func__, common->fsm_state);
			return 0;
		}
		break;

	case EEPROM_READ:
		ven_rsi_dbg(FSM_ZONE, "EEPROM READ confirm received\n");
		if (common->fsm_state == FSM_EEPROM_READ_MAC_ADDR) {
			u32 msg_len = ((u16 *)msg)[0] & 0xfff;

			if (msg_len <= 0) {
				ven_rsi_dbg(FSM_ZONE,
					"%s: [EEPROM_READ] Invalid len %d\n",
					__func__, msg_len);
				goto out;
			}
			if (msg[16] == MAGIC_WORD) {
				u8 offset = (FRAME_DESC_SZ +
					     WLAN_HOST_MODE_LEN +
					     WLAN_MAC_MAGIC_WORD_LEN);

				memcpy(common->mac_addr,
				       &msg[offset],
				       IEEE80211_ADDR_LEN);
				rsi_hex_dump(INIT_ZONE,
					     "MAC Addr",
					     common->mac_addr, ETH_ALEN);
				adapter->eeprom.length =
					((WLAN_MAC_MAGIC_WORD_LEN + 3) & (~3));
				adapter->eeprom.offset =
					WLAN_EEPROM_RFTYPE_ADDR;
				if (rsi_eeprom_read(common)) {
					ven_rsi_dbg(ERR_ZONE,
						"%s: Failed reading RF band\n",
						__func__);
					common->fsm_state = FSM_CARD_NOT_READY;
				} else {
					common->fsm_state =
						FSM_EEPROM_READ_RF_TYPE;
				}
			} else {
				common->fsm_state = FSM_CARD_NOT_READY;
				break;
			}
		} else if (common->fsm_state == FSM_EEPROM_READ_RF_TYPE) {
			u32 msg_len = ((u16 *)msg)[0] & 0xfff;

			if (msg_len <= 0) {
				ven_rsi_dbg(FSM_ZONE,
					"%s:[EEPROM_READ_CFM] Invalid len %d\n",
					__func__, msg_len);
				goto out;
			}
			if (msg[16] == MAGIC_WORD) {
				if ((msg[17] & 0x3) == 0x3) {
					ven_rsi_dbg(INIT_ZONE,
						"Dual band supported\n");
					common->band = NL80211_BAND_5GHZ;
					common->num_supp_bands = 2;
				} else if ((msg[17] & 0x3) == 0x1) {
					ven_rsi_dbg(INIT_ZONE,
						"Only 2.4Ghz band supported\n");
					common->band = NL80211_BAND_2GHZ;
					common->num_supp_bands = 1;
				}
			} else {
				common->fsm_state = FSM_CARD_NOT_READY;
				break;
			}
			if (rsi_send_reset_mac(common))
				goto out;
			else
				common->fsm_state = FSM_RESET_MAC_SENT;
		} else {
			ven_rsi_dbg(ERR_ZONE,
				"%s: Received eeprom read in %d state\n",
				__func__, common->fsm_state);
			return 0;
		}
		break;

	case RESET_MAC_REQ:
		if (common->fsm_state == FSM_RESET_MAC_SENT) {
			ven_rsi_dbg(FSM_ZONE, "Reset MAC confirm\n");

			if (rsi_load_radio_caps(common))
				goto out;
			else
				common->fsm_state = FSM_RADIO_CAPS_SENT;
		} else {
			ven_rsi_dbg(ERR_ZONE,
				"%s: Received reset mac cfm in %d state\n",
				 __func__, common->fsm_state);
			return 0;
		}
		break;

	case RADIO_CAPABILITIES:
		if (common->fsm_state == FSM_RADIO_CAPS_SENT) {
			common->rf_reset = 1;
			if (rsi_program_bb_rf(common)) {
				goto out;
			} else {
				common->fsm_state = FSM_BB_RF_PROG_SENT;
				ven_rsi_dbg(FSM_ZONE, "Radio caps confirm\n");
			}
		} else {
			ven_rsi_dbg(INFO_ZONE,
				"%s: Received radio caps cfm in %d state\n",
				 __func__, common->fsm_state);
			return 0;
		}
		break;

	case BB_PROG_VALUES_REQUEST:
	case RF_PROG_VALUES_REQUEST:
	case BBP_PROG_IN_TA:
		ven_rsi_dbg(FSM_ZONE, "BB/RF confirmation.\n");
		if (common->fsm_state == FSM_BB_RF_PROG_SENT) {
			common->bb_rf_prog_count--;
			if (!common->bb_rf_prog_count) {
				common->fsm_state = FSM_MAC_INIT_DONE;
				return rsi_mac80211_attach(common);
			}
		} else {
			ven_rsi_dbg(INFO_ZONE,
				"%s: Received bb_rf cfm in %d state\n",
				 __func__, common->fsm_state);
			return 0;
		}
		break;

	case AMPDU_IND:
		ven_rsi_dbg(INFO_ZONE, "AMPDU indication.\n");
		break;

	case SCAN_REQUEST:
		ven_rsi_dbg(INFO_ZONE, "Scan confirm.\n");
		break;

	case SET_RX_FILTER:
		ven_rsi_dbg(INFO_ZONE, "RX Filter confirmation.\n");
		break;

	case WAKEUP_SLEEP_REQUEST:
		ven_rsi_dbg(INFO_ZONE, "Wakeup/Sleep confirmation.\n");
		return rsi_handle_ps_confirm(adapter, msg);

	case BG_SCAN_PROBE_REQ:
		ven_rsi_dbg(INFO_ZONE, "BG scan complete event\n");
	
		/* resume to connected channel if associated */
		rsi_resume_conn_channel(adapter);
		break;

	default:
		ven_rsi_dbg(INFO_ZONE,
			"%s: Invalid TA confirm type : %x\n",
			__func__, sub_type);
		break;
	}
	return 0;

out:
	ven_rsi_dbg(ERR_ZONE,
		"%s: Unable to send pkt/Invalid frame received\n",
		__func__);
	return -EINVAL;
}

/**
 *rsi_handle_card_ready() - This function handles the card ready
 *                       indication from firmware.
 *@common: Pointer to the driver private structure.
 *
 *Return: 0 on success, -1 on failure.
 */
int rsi_handle_card_ready(struct rsi_common *common)
{
	switch (common->fsm_state) {
	case FSM_CARD_NOT_READY:
		ven_rsi_dbg(INIT_ZONE, "Card ready indication from Common HAL\n");
		rsi_set_default_parameters(common);
		if (rsi_send_common_dev_params(common) < 0)
			return -EINVAL;
		common->fsm_state = FSM_COMMON_DEV_PARAMS_SENT;
		break;
	case FSM_COMMON_DEV_PARAMS_SENT:
		ven_rsi_dbg(INIT_ZONE, "Common dev config params confirm\n");
		if (rsi_load_bootup_params(common)) {
			common->fsm_state = FSM_CARD_NOT_READY;
			return -EINVAL;
		}
		common->fsm_state = FSM_BOOT_PARAMS_SENT;
		break;
	default:
		ven_rsi_dbg(ERR_ZONE,
			"%s: card ready indication in invalid state %d.\n",
			__func__, common->fsm_state);
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_RSI_WOW 
int rsi_send_wowlan_request(struct rsi_common *common, u16 flags,
		            u16 sleep_status)
{
        struct rsi_wowlan_req *cmd_frame;
        struct sk_buff *skb;
        u8 length;
        u8 sourceid[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        ven_rsi_dbg(ERR_ZONE, "%s: Sending wowlan request frame\n", __func__);

        skb = dev_alloc_skb(sizeof(*cmd_frame));
        if (!skb) {
                ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of skb\n",
                                __func__);
                return -ENOMEM;
        }
        memset(skb->data, 0, sizeof(*cmd_frame));
        cmd_frame = (struct rsi_wowlan_req *)skb->data;

        cmd_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);
        cmd_frame->desc_word[1] |= cpu_to_le16(WOWLAN_CONFIG_PARAMS);
 
        memcpy(cmd_frame->sourceid, &sourceid, IEEE80211_ADDR_LEN);
	
        cmd_frame->host_sleep_status = sleep_status;
	if (sleep_status)
		cmd_frame->wow_flags = flags; /* TODO: check for magic packet */
        ven_rsi_dbg(INFO_ZONE, "Host_Sleep_Status : %d Flags : %d\n",
		cmd_frame->host_sleep_status, cmd_frame->wow_flags );
	
        length = FRAME_DESC_SZ + IEEE80211_ADDR_LEN + 2 + 2;

        cmd_frame->desc_word[0] |= cpu_to_le16(length - FRAME_DESC_SZ);
        cmd_frame->desc_word[2] |= cpu_to_le16(0);
  
  	skb_put(skb, length);

        return rsi_send_internal_mgmt_frame(common, skb);
}
#endif

/**
 * rsi_mgmt_pkt_recv() - This function processes the management packets
 *			 received from the hardware.
 * @common: Pointer to the driver private structure.
 * @msg: Pointer to the received packet.
 *
 * Return: 0 on success, -1 on failure.
 */
int rsi_mgmt_pkt_recv(struct rsi_common *common, u8 *msg)
{
	s32 msg_len = (le16_to_cpu(*(__le16 *)&msg[0]) & 0x0fff);
	u16 msg_type = msg[2];
	struct ieee80211_vif *vif = common->priv->vifs[0];

	switch (msg_type) {
	case TA_CONFIRM_TYPE:
		return rsi_handle_ta_confirm(common, msg);

	case CARD_READY_IND:
		ven_rsi_dbg(INIT_ZONE, "CARD READY INDICATION FROM WLAN.\n");
		return rsi_handle_card_ready(common);

	case TX_STATUS_IND:
		if (msg[15] == PROBEREQ_CONFIRM) {
			common->mgmt_q_block = false;
			ven_rsi_dbg(INFO_ZONE, "Mgmt queue unblocked\n");
		}
		if ((msg[15] & 0xff) == EAPOL4_CONFIRM) {
			u8 status = msg[12];

			if (status) {	
				if(vif->type == NL80211_IFTYPE_STATION) {
					ven_rsi_dbg(ERR_ZONE, "EAPOL 4 confirm\n");
					common->eapol4_confirm = 1;
					if (!rsi_send_block_unblock_frame(common, false))
						common->hw_data_qs_blocked = false;
				}
			}
		}
		break;

	case PS_NOTIFY_IND:
		ven_rsi_dbg(FSM_ZONE, "Powersave notify indication.\n");
		break;

	case SLEEP_NOTIFY_IND:
		ven_rsi_dbg(FSM_ZONE, "Sleep notify indication.\n");
		break;

	case DECRYPT_ERROR_IND:
		ven_rsi_dbg(INFO_ZONE, "Error in decrypt.\n");
		break;

	case DEBUG_IND:
		ven_rsi_dbg(INFO_ZONE, "Debugging indication.\n");
		break;

	case RX_MISC_IND:
		ven_rsi_dbg(INFO_ZONE, "RX misc indication.\n");
		break;

	case HW_BMISS_EVENT:
		ven_rsi_dbg(INFO_ZONE, "Hardware beacon miss event\n");
		rsi_indicate_bcnmiss(common);
		rsi_resume_conn_channel(common->priv);
		break;

	case BEACON_EVENT_IND:
		ven_rsi_dbg(INFO_ZONE, "Beacon event\n");
#ifndef CONFIG_CARACALLA_BOARD
		if (!common->init_done)
			return -1;
		if (common->iface_down)
			return -1;
		if (!common->beacon_enabled)
			return -1;
		rsi_send_beacon(common);
#endif
		break;

	case RX_DOT11_MGMT:
		return rsi_mgmt_pkt_to_core(common, msg, msg_len);

	default:
		ven_rsi_dbg(INFO_ZONE, "Cmd Frame Type: %d\n", msg_type);
		break;
	}

	return 0;
}

