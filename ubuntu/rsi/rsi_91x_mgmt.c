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
	.host_wakeup_intr	= USED_GPIO,	//GPIO_2
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
	common->antenna_gain[0] = 0;
	common->antenna_gain[1] = 0;
}

void init_bgscan_params(struct rsi_common *common)
{
	common->bgscan_info.bgscan_threshold = 0;
	common->bgscan_info.roam_threshold = 10;
	common->bgscan_info.bgscan_periodicity = 30;
	common->bgscan_info.num_bg_channels = 0;
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
	if (skb->data[2] == PEER_NOTIFY)
		skb_queue_head(&common->tx_queue[MGMT_SOFT_Q], skb);
	else
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

	if (common->iface_down)
		return -ENODEV;

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
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_vif *vif = adapter->vifs[adapter->sc_nvifs - 1];
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
	adapter->peer_notify = true;
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
//	u8 window_size = 1;

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
		mgmt_frame->desc_word[5] = cpu_to_le16(buf_size);
		//mgmt_frame->desc_word[5] = cpu_to_le16(window_size);
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
			     u8 *mac_addr,
			     u8 vap_id,
			     u8 vap_status)
{
	struct sk_buff *skb = NULL;
	struct rsi_vap_caps *vap_caps;
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_hw *hw = adapter->hw;
	struct ieee80211_conf *conf = &hw->conf;

	ven_rsi_dbg(MGMT_TX_ZONE,
		"%s: Sending VAP capabilities frame\n", __func__);

	ven_rsi_dbg(INFO_ZONE, "Config VAP: id=%d mode=%d status=%d ",
		vap_id, mode, vap_status);
	rsi_hex_dump(INFO_ZONE, "mac", mac_addr, 6);
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

	memcpy(vap_caps->mac_addr, mac_addr, IEEE80211_ADDR_LEN);
	vap_caps->keep_alive_period = cpu_to_le16(90);
	vap_caps->frag_threshold = cpu_to_le16(IEEE80211_MAX_FRAG_THRESHOLD);

	vap_caps->rts_threshold = cpu_to_le16(common->rts_threshold);

	if (common->band == NL80211_BAND_5GHZ) {
		vap_caps->default_ctrl_rate = cpu_to_le32(RSI_RATE_6);
		vap_caps->default_mgmt_rate = cpu_to_le32(RSI_RATE_6);
	} else {
		if (common->p2p_enabled) {
			vap_caps->default_ctrl_rate = cpu_to_le32(RSI_RATE_6);
			vap_caps->default_mgmt_rate = cpu_to_le32(RSI_RATE_6);
		} else {
		vap_caps->default_ctrl_rate = cpu_to_le32(RSI_RATE_1);
		vap_caps->default_mgmt_rate = cpu_to_le32(RSI_RATE_1);
	}
	}

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

	vap_caps->default_data_rate = 0;
	vap_caps->beacon_interval = cpu_to_le16(common->beacon_interval);
	vap_caps->dtim_period = cpu_to_le16(common->dtim_cnt);
#ifdef RSI_HW_CONN_MONITOR
	vap_caps->beacon_miss_threshold = cpu_to_le16(10);
#else
	if (mode == AP_OPMODE)
		vap_caps->beacon_miss_threshold = cpu_to_le16(10);
#endif

	skb_put(skb, sizeof(*vap_caps));

	common->last_vap_type = mode;
	ether_addr_copy(common->last_vap_addr, mac_addr);
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
	struct ieee80211_vif *vif = common->priv->vifs[common->priv->sc_nvifs - 1];
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
		if ((vif->type == NL80211_IFTYPE_AP) ||
		    (vif->type == NL80211_IFTYPE_P2P_GO))
			key_descriptor = BIT(7);
		break;
	case RSI_PAIRWISE_KEY:
		if (((vif->type == NL80211_IFTYPE_AP) ||
		     (vif->type == NL80211_IFTYPE_P2P_GO)) &&
		    (sta_id >= common->max_stations)) {
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
	u32 frame_len = 0;
	struct rsi_config_vals *dev_cfgs = NULL;

	frame_len = sizeof(struct rsi_config_vals);

	ven_rsi_dbg(MGMT_TX_ZONE, "Sending common device config params\n");
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

	dev_cfgs->unused_ulp_gpio = *(u8 *)&unused_ulp_gpio_bitmap;
	dev_cfgs->unused_soc_gpio_bitmap =
		cpu_to_le32(*(u32 *)&unused_soc_gpio_bitmap);

	dev_cfgs->opermode = common->oper_mode;
	dev_cfgs->wlan_rf_pwr_mode = common->wlan_rf_power_mode;
	dev_cfgs->driver_mode = common->driver_mode;
	dev_cfgs->region_code = NL80211_DFS_FCC;
	dev_cfgs->antenna_sel_val = common->obm_ant_sel_val;

	skb_put(skb, frame_len);

	rsi_hex_dump(ERR_ZONE, "common dev config params ",
		     skb->data, skb->len);
	return rsi_send_internal_mgmt_frame(common, skb);
}

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
int rsi_band_check(struct rsi_common *common,
		   struct ieee80211_channel *curchan)
{
	struct rsi_hw *adapter = common->priv;
	struct ieee80211_hw *hw = adapter->hw;
	u8 prev_bw = common->channel_width;
	u8 prev_ep = common->endpoint;
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

#ifdef CONFIG_CARACALLA_BOARD
void rsi_apply_carcalla_power_values(struct rsi_hw *adapter,
				     struct ieee80211_vif *vif,
				     struct ieee80211_channel *channel)
{
	u16 max_power = 20;

	if (!conf_is_ht(&adapter->hw->conf)) {
		if (vif->bss_conf.basic_rates == 0xf) {
			if (channel->hw_value == 12)
				max_power = 15;
			else if (channel->hw_value == 13)
				max_power = 7; 
			else
				return;
		} else {
			if (channel->hw_value == 12)
				max_power = 8; 
			else if (channel->hw_value == 13)
				max_power = 7; 
			else
				return;
		}
	} else if (conf_is_ht20(&adapter->hw->conf)) {
		if (channel->hw_value == 12)
			max_power = 7; 
		else if (channel->hw_value == 13)
			max_power = 5; 
		else
			return;
	} else {
		if (channel->hw_value == 6)
			max_power = 9; 
		else if (channel->hw_value == 9)
			max_power = 5; 
		else if (channel->hw_value == 10)
			max_power = 4; 
		else
			return;
	}
	channel->max_power = max_power;
	channel->max_antenna_gain = 0;
}
#endif

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
	struct rsi_hw *adapter = common->priv;
#ifdef CONFIG_CARACALLA_BOARD
	struct ieee80211_vif *vif = adapter->vifs[adapter->sc_nvifs - 1];
#endif
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

#if 0
	channel->max_antenna_gain = common->antenna_gain;
	mgmt_frame->desc_word[4] |=
		cpu_to_le16(((char)(channel->max_antenna_gain)) << 8);
	mgmt_frame->desc_word[5] =
		cpu_to_le16((char)(channel->max_antenna_gain));
#endif
	mgmt_frame->desc_word[4] |=
		cpu_to_le16(common->antenna_gain[0] << 8);
	mgmt_frame->desc_word[5] =
		cpu_to_le16(common->antenna_gain[1]);

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
#ifdef CONFIG_CARACALLA_BOARD
	rsi_apply_carcalla_power_values(adapter, vif, channel);
	mgmt_frame->desc_word[6] = cpu_to_le16(channel->max_power);

	if ((channel->hw_value == 12) || (channel->hw_value == 13))
		mgmt_frame->desc_word[7] = cpu_to_le16(TARGET_BOARD_CARACALLA);
	if (conf_is_ht40(&adapter->hw->conf)) {
		if ((channel->hw_value == 6) ||
		    (channel->hw_value == 9) ||
		    (channel->hw_value == 10))
		mgmt_frame->desc_word[7] = cpu_to_le16(TARGET_BOARD_CARACALLA);
	}
#endif
	ven_rsi_dbg(INFO_ZONE, "reg_domain = %d, TX power = %d\n",
		adapter->dfs_region, mgmt_frame->desc_word[6]);
	mgmt_frame->desc_word[7] |= cpu_to_le16(adapter->dfs_region);

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

#ifdef CONFIG_VEN_RSI_WOW
//	if (common->suspend_flag) {
	if (1) {
		dynamic_frame->desc_word[6] =
			cpu_to_le16(24); /* bmiss_threshold */
		dynamic_frame->frame_body.keep_alive_period =
			cpu_to_le16(5);
	} else
		dynamic_frame->frame_body.keep_alive_period = cpu_to_le16(90);
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
	ven_rsi_dbg(INFO_ZONE, "rate_bitmap = %x\n", rate_bitmap);
	ven_rsi_dbg(INFO_ZONE, "is_ht = %d\n", is_ht);

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
					cpu_to_le16(rsi_mcsrates[kk]/* | BIT(9)*/);
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
void rsi_validate_bgscan_channels(struct rsi_hw *adapter,
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
		if (adapter->priv->num_supp_bands > 1)
			params->num_user_channels = MAX_BGSCAN_CHANNELS;
		else
			params->num_user_channels = 14;

		for (cnt = 0; cnt < params->num_user_channels; cnt++)
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
		ven_rsi_dbg(INFO_ZONE, "%d ", ch_num);
		if ((ch->flags & IEEE80211_CHAN_NO_IR) ||
		    (ch->flags & IEEE80211_CHAN_RADAR)) {
			if((ch->flags & IEEE80211_CHAN_RADAR))
				ven_rsi_dbg(INFO_ZONE, "[DFS]");
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
	if (enable) {
		bgscan->bgscan_periodicity =
			cpu_to_le16(info->bgscan_periodicity);
	}
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
	bgscan->flags = cpu_to_le16(HOST_BG_SCAN_TRIG);
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
			   const u8 *bssid,
			   u8 qos_enable,
			   u16 aid,
			   struct ieee80211_sta *sta,
			   u16 sta_id, u16 assoc_cap)
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
			if (!(assoc_cap & BIT(4))) {	
				ven_rsi_dbg(INFO_ZONE, "unblock data in Open case\n");
				if (!rsi_send_block_unblock_frame(common, false))
					common->hw_data_qs_blocked = false;
			}
		}
	} else {
		if (opmode == STA_OPMODE)
			common->hw_data_qs_blocked = true;
#ifdef CONFIG_VEN_RSI_WOW
		if (!(common->wow_flags & RSI_WOW_ENABLED)) {
#endif
		rsi_send_sta_notify_frame(common,
					  opmode,
					  STA_DISCONNECTED,
					  bssid,
					  qos_enable,
					  aid,
					  sta_id);
#ifdef CONFIG_VEN_RSI_WOW
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

	if (ps->ps_listen_interval > ps->ps_dtim_interval_duration)
		ps->ps_listen_interval = 0;

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
	mgmt_frame->desc_word[2] = cpu_to_le16(1 << 8); /* Antenna selection
							   type */
	mgmt_frame->desc_word[3] = cpu_to_le16(antenna & 0x00ff);
	mgmt_frame->desc_word[0] = cpu_to_le16(RSI_WIFI_MGMT_Q << 12);

	skb_put(skb, FRAME_DESC_SZ);

	return rsi_send_internal_mgmt_frame(common, skb);
}

#ifdef CONFIG_VEN_RSI_WOW
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
	if ((common->secinfo.security_enable) &&
	    (common->secinfo.gtk_cipher))
		flags |= RSI_WOW_GTK_REKEY;
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

int rsi_send_beacon(struct rsi_common *common)
{
	struct sk_buff *skb = NULL;
	u8 dword_align_bytes = 0;
	
	skb = dev_alloc_skb(MAX_MGMT_PKT_SIZE);
	if (!skb)
		return -ENOMEM;

	memset(skb->data, 0, MAX_MGMT_PKT_SIZE);
	
	dword_align_bytes = ((unsigned long)skb->data & 0x3f);
	if (dword_align_bytes) {
		skb_pull(skb, (64 - dword_align_bytes));
	}
	if (rsi_prepare_beacon(common, skb)) {
		ven_rsi_dbg(ERR_ZONE, "Failed to prepare beacon\n");
		return -EINVAL;
	}
	skb_queue_tail(&common->tx_queue[MGMT_BEACON_Q], skb);
	rsi_set_event(&common->tx_thread.event);
	ven_rsi_dbg(DATA_TX_ZONE,
		"%s: Added to beacon queue\n", __func__);

	return 0;
}

#ifdef CONFIG_HW_SCAN_OFFLOAD
static void channel_change_event(unsigned long priv)
{
	struct rsi_hw *adapter = (struct rsi_hw *)priv;
	struct rsi_common *common = adapter->priv;

	rsi_set_event(&common->chan_change_event);
	del_timer(&common->scan_timer);
}

static int init_channel_timer(struct rsi_hw *adapter, u32 timeout)
{
	struct rsi_common *common = adapter->priv;

	init_timer(&common->scan_timer);
	rsi_reset_event(&common->chan_change_event);
	common->scan_timer.data = (unsigned long)adapter;
	common->scan_timer.function = (void *)&channel_change_event;
	common->scan_timer.expires = msecs_to_jiffies(timeout) + jiffies;

	add_timer(&common->scan_timer);

	return 0;
}

/**
 * This function prepares Probe request frame and send it to LMAC.
 * @param  Pointer to Adapter structure.
 * @param  Type of scan(Active/Passive).
 * @param  Broadcast probe.
 * @param  Pointer to the destination address.
 * @param  Indicates LMAC/UMAC Q number.
 * @return 0 if success else -1.
 */
int rsi_send_probe_request(struct rsi_common *common,
			   struct cfg80211_scan_request *scan_req,
			   u8 n_ssid,
			   u8 channel,
			   u8 scan_type)
{
	struct cfg80211_ssid *ssid_info;
	struct ieee80211_tx_info *info;
	struct skb_info *tx_params;
	struct sk_buff *skb = NULL;
	struct ieee80211_hdr *hdr = NULL;
	u8 *pos;
	u32 len = 0;
	u8 ie_ssid_len;
	u8 q_num;

	if (common->priv->sc_nvifs <= 0)
		return 0;
  if (!scan_req)
    return 0;
	ssid_info = &scan_req->ssids[n_ssid];
	if (!ssid_info)
		return 0;
	ie_ssid_len = ssid_info->ssid_len + 2 ;
	len = (MIN_802_11_HDR_LEN + scan_req->ie_len + ie_ssid_len);

	if (scan_type == 0) {
		skb = dev_alloc_skb(len);
		if (!skb) {
			ven_rsi_dbg(ERR_ZONE, "Failed to alloc probe req\n");
			return -ENOMEM;
		}
		skb_put(skb, len);
		memset(skb->data,0,skb->len);
		pos = skb->data;
	
		/*
		 * probe req frame format
		 * ssid
		 * supported rates
		 * RSN (optional)
		 * extended supported rates
		 * WPA (optional)
		 * user-specified ie's
		 */
		hdr = (struct ieee80211_hdr *)skb->data;
	} else {
		pos = common->bgscan_probe_req;
		hdr = (struct ieee80211_hdr *)common->bgscan_probe_req; 
	}
	hdr->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT | 
					 IEEE80211_STYPE_PROBE_REQ);
	hdr->duration_id = 0x0;
	memset(hdr->addr1, 0xff, ETH_ALEN);
	memset(hdr->addr3, 0xFF, 6);
	memcpy(hdr->addr2, common->mac_addr, ETH_ALEN);
	hdr->seq_ctrl = 0x00;
	pos += MIN_802_11_HDR_LEN; 

	*pos++ = WLAN_EID_SSID;
        *pos++ = ssid_info->ssid_len;

	/* Check for hidden ssid case */
	if (ssid_info->ssid_len) {
                memcpy(pos, ssid_info->ssid, ssid_info->ssid_len);
	}
        pos += ssid_info->ssid_len;

        if (scan_req->ie_len) {
                memcpy(pos, scan_req->ie, scan_req->ie_len);
	}
       
	if (scan_type == 1) {
		common->bgscan_probe_req_len = len;	
		return 0;
	}

	if ((common->iface_down == true) || (!common->scan_in_prog))
		goto out;

	info = IEEE80211_SKB_CB(skb);
	tx_params = (struct skb_info *)info->driver_data;
	tx_params->internal_hdr_size = skb_headroom(skb);
	info->control.vif = common->priv->vifs[0];	
	q_num = MGMT_SOFT_Q;
	skb->priority = q_num;

	rsi_prepare_mgmt_desc(common,skb);
	skb_queue_tail(&common->tx_queue[MGMT_SOFT_Q], skb);
	rsi_set_event(&common->tx_thread.event);
	
	return 0;

out:
	dev_kfree_skb(skb);
	return 0;
}

void rsi_scan_start(struct work_struct *work)
{
	struct ieee80211_channel *cur_chan = NULL;
	struct cfg80211_scan_request *scan_req = NULL;
	struct rsi_common *common =
		container_of(work, struct rsi_common ,scan_work);
	u8 ii, jj;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0))
	struct cfg80211_scan_info info;
#endif
	
	scan_req = common->scan_request;
	if (!scan_req)
		return;

	common->scan_in_prog = true;
	
	for (ii =0; ii < scan_req->n_channels ; ii++) {
		if (common->iface_down)
			break;
	
		if (!common->scan_in_prog)
			break;
		
		ven_rsi_dbg(INFO_ZONE,
			"channle no: %d", scan_req->channels[ii]->hw_value);
		cur_chan = scan_req->channels[ii];

		rsi_band_check(common, cur_chan);
		if (cur_chan->flags & IEEE80211_CHAN_DISABLED)
			continue;

		/* maxdwell time macro */
		init_channel_timer(common->priv, 300);
				 
		if (rsi_set_channel(common, cur_chan)) {
			ven_rsi_dbg(ERR_ZONE, "Failed to set the channel\n");
			break;
		}
		rsi_reset_event(&common->chan_set_event);
		rsi_wait_event(&common->chan_set_event, msecs_to_jiffies(50));
		
		if (!common->scan_in_prog)
			break;
		rsi_reset_event(&common->chan_set_event);

		if ((cur_chan->flags & IEEE80211_CHAN_NO_IR) ||
		    (cur_chan->flags & IEEE80211_CHAN_RADAR)) {
			/* DFS Channel */
			/* Program passive scan duration */
		} else {
			/* Send probe request */
			for (jj = 0;jj < scan_req->n_ssids;jj++) {
				rsi_send_probe_request(common, 
						       scan_req,
						       jj,
						       cur_chan->hw_value,
						       0);
				if (common->iface_down == true) {
					common->scan_in_prog = false;
					return;
				}
				rsi_reset_event(&common->probe_cfm_event);
				rsi_wait_event(&common->probe_cfm_event,
					       msecs_to_jiffies(50));
				rsi_reset_event(&common->probe_cfm_event);
			}
		}
		if (!common->scan_in_prog)
			break;
		if (common->iface_down)
			break;
		rsi_wait_event(&common->chan_change_event, EVENT_WAIT_FOREVER);
		rsi_reset_event(&common->chan_change_event);
	}

	del_timer(&common->scan_timer);
	common->scan_in_prog = false;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0))
	info.aborted = false;
	ieee80211_scan_completed(common->priv->hw, &info);
#else	
	ieee80211_scan_completed(common->priv->hw, false);
#endif

	if (common->hw_scan_cancel) {	
		skb_queue_purge(&common->tx_queue[MGMT_SOFT_Q]);
		rsi_set_event(&common->cancel_hw_scan_event);
	}	
	return;
}
#endif

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
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0))
	struct cfg80211_scan_info info;
#endif

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
#ifdef CONFIG_HW_SCAN_OFFLOAD
		rsi_set_event(&common->chan_set_event);
#endif
		break;

	case SET_RX_FILTER:
		ven_rsi_dbg(INFO_ZONE, "RX Filter confirmation.\n");
		break;

	case WAKEUP_SLEEP_REQUEST:
		ven_rsi_dbg(INFO_ZONE, "Wakeup/Sleep confirmation.\n");
		return rsi_handle_ps_confirm(adapter, msg);

	case BG_SCAN_PROBE_REQ:
		ven_rsi_dbg(INFO_ZONE, "BG scan complete event\n");
		if (common->bgscan_en) {
			if (!rsi_send_bgscan_params(common, 0))
			common->bgscan_en = 0;
		}	
#ifdef CONFIG_HW_SCAN_OFFLOAD
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0))
		info.aborted = false;
		ieee80211_scan_completed(adapter->hw, &info);
#else	
		ieee80211_scan_completed(adapter->hw, false);
#endif
		if (common->hw_scan_cancel)
			rsi_set_event(&common->cancel_hw_scan_event);
#endif
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
int rsi_handle_card_ready(struct rsi_common *common, u8 *msg)
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
		ven_rsi_dbg(INIT_ZONE, "Card ready indication from WLAN HAL\n");

		/* Get usb buffer status register address */
		common->priv->usb_buffer_status_reg = *(u32 *)&msg[8];
		ven_rsi_dbg(INFO_ZONE, "USB buffer status register = %x\n",
			common->priv->usb_buffer_status_reg);

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
	struct rsi_hw *adapter = common->priv;
	s32 msg_len = (le16_to_cpu(*(__le16 *)&msg[0]) & 0x0fff);
	u16 msg_type = msg[2];
	struct ieee80211_vif *vif = adapter->vifs[adapter->sc_nvifs - 1];

	switch (msg_type) {
	case TA_CONFIRM_TYPE:
		return rsi_handle_ta_confirm(common, msg);

	case CARD_READY_IND:
		ven_rsi_dbg(INIT_ZONE, "CARD READY INDICATION FROM WLAN.\n");
		return rsi_handle_card_ready(common, msg);

	case TX_STATUS_IND:
		if (msg[15] == PROBEREQ_CONFIRM) {
			common->mgmt_q_block = false;
			ven_rsi_dbg(INFO_ZONE, "Mgmt queue unblocked\n");
#ifdef CONFIG_HW_SCAN_OFFLOAD
			rsi_set_event(&common->probe_cfm_event);
#endif
		}
		if ((msg[15] & 0xff) == EAPOL4_CONFIRM) {
			u8 status = msg[12];

			if (status) {	
				if(vif->type == NL80211_IFTYPE_STATION) {
					ven_rsi_dbg(ERR_ZONE, "EAPOL 4 confirm\n");
					common->start_bgscan = 1;
					common->eapol4_confirm = 1;
					if (!rsi_send_block_unblock_frame(common,
									  false))
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
		rsi_resume_conn_channel(common->priv,
					adapter->vifs[adapter->sc_nvifs - 1]);
		break;

	case BEACON_EVENT_IND:
		ven_rsi_dbg(INFO_ZONE, "Beacon event\n");
		if (!common->init_done)
			return -1;
		if (common->iface_down)
			return -1;
		if (!common->beacon_enabled)
			return -1;
		rsi_send_beacon(common);
		break;

	case WOWLAN_WAKEUP_REASON:
		rsi_hex_dump(INFO_ZONE, "WoWLAN Wakeup Trigger Pkt",
			     msg, msg_len);
		ven_rsi_dbg(ERR_ZONE, "\n\nWakeup Type: %x\n", msg[15]);
		switch(msg[15]) {
		case UNICAST_MAGIC_PKT:
			ven_rsi_dbg(ERR_ZONE,
				"*** Wakeup for Unicast magic packet ***\n");
			break;
		case BROADCAST_MAGICPKT:
			ven_rsi_dbg(ERR_ZONE,
				"*** Wakeup for Broadcast magic packet ***\n");
			break;
		case EAPOL_PKT:
			ven_rsi_dbg(ERR_ZONE,
				"*** Wakeup for GTK renewal ***\n");
			break;
		case DISCONNECT_PKT:
			ven_rsi_dbg(ERR_ZONE,
				"*** Wakeup for Disconnect ***\n");
			break;
		case HW_BMISS_PKT:
			ven_rsi_dbg(ERR_ZONE,
				"*** Wakeup for HW Beacon miss ***\n");
			break;
		default:
			ven_rsi_dbg(ERR_ZONE,
				"##### Un-intentional Wakeup #####\n");
			break;
		}

	case RX_DOT11_MGMT:
		return rsi_mgmt_pkt_to_core(common, msg, msg_len);

	default:
		ven_rsi_dbg(INFO_ZONE, "Cmd Frame Type: %d\n", msg_type);
		break;
	}

	return 0;
}


