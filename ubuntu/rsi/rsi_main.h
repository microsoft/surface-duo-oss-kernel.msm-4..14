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

#ifndef __RSI_MAIN_H__
#define __RSI_MAIN_H__

#include <linux/string.h>
#include <linux/skbuff.h>
#include <net/mac80211.h>
#include <linux/etherdevice.h>
#include <linux/version.h>

struct rsi_hw;

#include "rsi_ps.h"

#define ERR_ZONE                        BIT(0) /* Error Msgs		*/
#define INFO_ZONE                       BIT(1) /* Generic Debug Msgs	*/
#define INIT_ZONE                       BIT(2) /* Driver Init Msgs	*/
#define MGMT_TX_ZONE                    BIT(3) /* TX Mgmt Path Msgs	*/
#define MGMT_RX_ZONE                    BIT(4) /* RX Mgmt Path Msgs	*/
#define DATA_TX_ZONE                    BIT(5) /* TX Data Path Msgs	*/
#define DATA_RX_ZONE                    BIT(6) /* RX Data Path Msgs	*/
#define FSM_ZONE                        BIT(7) /* State Machine Msgs	*/
#define ISR_ZONE                        BIT(8) /* Interrupt Msgs	*/

#define FSM_CARD_NOT_READY              0
#define FSM_COMMON_DEV_PARAMS_SENT	1
#define FSM_BOOT_PARAMS_SENT            2
#define FSM_EEPROM_READ_MAC_ADDR        3
#define FSM_EEPROM_READ_RF_TYPE		4
#define FSM_RESET_MAC_SENT              5
#define FSM_RADIO_CAPS_SENT             6
#define FSM_BB_RF_PROG_SENT             7
#define FSM_MAC_INIT_DONE               8

extern u32 ven_rsi_zone_enabled;
extern __printf(2, 3) void ven_rsi_dbg(u32 zone, const char *fmt, ...);
void rsi_hex_dump(u32 zone, char *msg_str, const u8 *msg, u32 len);

#define RSI_MAX_VIFS                    1
#define NUM_EDCA_QUEUES                 4
#define IEEE80211_ADDR_LEN              6
#define FRAME_DESC_SZ                   16
#define MIN_802_11_HDR_LEN              24

#define DATA_QUEUE_WATER_MARK           400
#define MIN_DATA_QUEUE_WATER_MARK       300
#define MULTICAST_WATER_MARK            200
#define MAC_80211_HDR_FRAME_CONTROL     0
#define WME_NUM_AC                      4
#define NUM_SOFT_QUEUES                 5
#define MAX_HW_QUEUES                   12
#define INVALID_QUEUE                   0xff
#define MAX_CONTINUOUS_VO_PKTS          8
#define MAX_CONTINUOUS_VI_PKTS          4
#define MGMT_HW_Q			10 /* Queue No 10 is used for
					    * MGMT_QUEUE in Device FW,
					    *  Hence this is Reserved
					    */
#define BROADCAST_HW_Q			9
#define BEACON_HW_Q			11

/* Queue information */
#define RSI_COEX_Q			0x0
#define RSI_ZIGB_Q			0x1
#define RSI_BT_Q			0x2
#define RSI_WLAN_Q			0x3
#define RSI_WIFI_MGMT_Q                 0x4
#define RSI_WIFI_DATA_Q                 0x5
#define RSI_BT_MGMT_Q			0x6
#define RSI_BT_DATA_Q			0x7
#define IEEE80211_MGMT_FRAME            0x00
#define IEEE80211_CTL_FRAME             0x04

#define RSI_MAX_ASSOC_STAS		4
#define IEEE80211_QOS_TID               0x0f
#define IEEE80211_NONQOS_TID            16

#define MAX_DEBUGFS_ENTRIES             5
#define MAX_BGSCAN_CHANNELS		38


#define TID_TO_WME_AC(_tid) (      \
	((_tid) == 0 || (_tid) == 3) ? BE_Q : \
	((_tid) < 3) ? BK_Q : \
	((_tid) < 6) ? VI_Q : \
	VO_Q)

#define WME_AC(_q) (    \
	((_q) == BK_Q) ? IEEE80211_AC_BK : \
	((_q) == BE_Q) ? IEEE80211_AC_BE : \
	((_q) == VI_Q) ? IEEE80211_AC_VI : \
	IEEE80211_AC_VO)

struct version_info {
	u16 major;
	u16 minor;
	u16 release_num;
	u16 patch_num;
} __packed;

struct skb_info {
	s8 rssi;
	u32 flags;
	u16 channel;
	s8 tid;
	s8 sta_id;
	u8 internal_hdr_size;
	struct ieee80211_sta *sta;
};

enum edca_queue {
	BK_Q = 0,
	BE_Q,
	VI_Q,
	VO_Q,
	MGMT_SOFT_Q
};

struct security_info {
	bool security_enable;
	u32 ptk_cipher;
	u32 gtk_cipher;
};

struct wmm_qinfo {
	s32 weight;
	s32 wme_params;
	s32 pkt_contended;
	s32 txop;
};

struct transmit_q_stats {
	u32 total_tx_pkt_send[NUM_EDCA_QUEUES + 1];
	u32 total_tx_pkt_freed[NUM_EDCA_QUEUES + 1];
};

struct vif_priv {
	bool is_ht;
	bool sgi;
	u16 seq_start;
};

struct rsi_event {
	atomic_t event_condition;
	wait_queue_head_t event_queue;
};

struct rsi_thread {
	void (*thread_function)(void *);
	struct completion completion;
	struct task_struct *task;
	struct rsi_event event;
	atomic_t thread_done;
};

struct cqm_info {
	s8 last_cqm_event_rssi;
	int rssi_thold;
	u32 rssi_hyst;
};

struct bgscan_config_params {
	u16 bgscan_threshold;
	u16 roam_threshold;
	u16 bgscan_periodicity;
	u8 num_user_channels;
	u8 num_bg_channels;
	u8 two_probe;
	u16 active_scan_duration;
	u16 passive_scan_duration;
	u16 user_channels[MAX_BGSCAN_CHANNELS];
	u16 channels2scan[MAX_BGSCAN_CHANNELS];
};

struct xtended_desc {
	u8 confirm_frame_type;
	u8 retry_cnt;
	u16 reserved;
};

struct rsi_sta {
	struct ieee80211_sta *sta;
	s16 sta_id;
	u16 seq_no[IEEE80211_NUM_ACS];
};

struct rsi_hw;

struct rsi_common {
	struct rsi_hw *priv;
	struct vif_priv vif_info[RSI_MAX_VIFS];

	bool mgmt_q_block;
	struct version_info driver_ver;
	struct version_info fw_ver;

	struct rsi_thread tx_thread;
#ifdef CONFIG_SDIO_INTR_POLL
	struct rsi_thread sdio_intr_poll_thread;
#endif
	struct sk_buff_head tx_queue[NUM_EDCA_QUEUES + 1];
	/* Mutex declaration */
	struct mutex mutex;
	struct mutex pslock;
	/* Mutex used between tx/rx threads */
	struct mutex tx_lock;
	struct mutex rx_lock;
	u8 endpoint;

	/* Channel/band related */
	u8 band;
	u8 num_supp_bands;
	u8 channel_width;

	u16 rts_threshold;
	u16 bitrate_mask[2];
	u32 fixedrate_mask[2];

	u8 rf_reset;
	struct transmit_q_stats tx_stats;
	struct security_info secinfo;
	struct wmm_qinfo tx_qinfo[NUM_EDCA_QUEUES];
	struct ieee80211_tx_queue_params edca_params[NUM_EDCA_QUEUES];
	u8 mac_addr[IEEE80211_ADDR_LEN];

	/* state related */
	u32 fsm_state;
	u8 bt_fsm_state;
	bool init_done;
	u8 bb_rf_prog_count;
	bool iface_down;

	/* Generic */
	u8 channel;
	u8 *rx_data_pkt;
	u8 *saved_rx_data_pkt;
	u8 mac_id;
	u8 radio_id;
	u16 rate_pwr[20];
	u16 min_rate;

	/* WMM algo related */
	u8 selected_qnum;
	u32 pkt_cnt;
	u8 min_weight;

	/* bgscan related */
	struct cqm_info cqm_info;
	struct bgscan_config_params bgscan_info;
	int bgscan_en;
	u8 bgscan_probe_req[1500];
	int bgscan_probe_req_len;
	u16 bgscan_seq_ctrl;
	u8 mac80211_cur_channel;

	bool hw_data_qs_blocked;
	u8 driver_mode;
	u8 coex_mode;
	u8 oper_mode;
	u8 ta_aggr;
	u8 skip_fw_load;
	u8 lp_ps_handshake_mode;
	u8 ulp_ps_handshake_mode;
	u8 uapsd_bitmap;
	u8 rf_power_val;
	u8 device_gpio_type;
	u16 country_code;
	u8 wlan_rf_power_mode;
	u8 bt_rf_power_mode;
	u8 obm_ant_sel_val;
	u8 antenna_diversity;
	u16 rf_pwr_mode;
	char antenna_gain[2];
	u8 host_wakeup_intr_enable;
	u8 host_wakeup_intr_active_high;
	int tx_power;
	u8 ant_in_use;
#ifdef CONFIG_RSI_WOW
	u8 suspend_flag;
#endif

#if defined (CONFIG_VEN_RSI_HCI) || defined(CONFIG_VEN_RSI_COEX)
	void *hci_adapter;
#endif

#ifdef CONFIG_VEN_RSI_COEX
	void *coex_cb;
#endif

	/* AP mode related */
	u8 beacon_enabled;
	u16 beacon_interval;
	u8 *beacon_frame;
	u16 beacon_frame_len;
	u16 beacon_cnt;
	u8 dtim_cnt;
	u16 bc_mc_seqno;
	struct rsi_sta stations[RSI_MAX_ASSOC_STAS + 1];
	int num_stations;
	struct ieee80211_channel *ap_channel;
	struct rsi_thread bcn_thread;
	struct timer_list bcn_timer;
	struct ieee80211_key_conf *key;
	u8 eapol4_confirm;
};

enum host_intf {
	RSI_HOST_INTF_SDIO = 0,
	RSI_HOST_INTF_USB
};

enum rsi_dev_model {
	RSI_DEV_9110 = 0,
	RSI_DEV_9113,
	RSI_DEV_9116
};

struct eepromrw_info {
	u32 offset;
	u32 length;
	u8  write;
	u16 eeprom_erase;
	u8 data[480];
};

struct eeprom_read {
	u16 length;
	u16 off_set;
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0))
#define NUM_NL80211_BANDS	3
#endif

struct rsi_hw {
	struct rsi_common *priv;
	enum rsi_dev_model device_model;
	struct ieee80211_hw *hw;
	struct ieee80211_vif *vifs[RSI_MAX_VIFS];
	struct ieee80211_tx_queue_params edca_params[NUM_EDCA_QUEUES];

	struct ieee80211_supported_band sbands[NUM_NL80211_BANDS];

	struct device *device;
	u8 sc_nvifs;
	enum host_intf rsi_host_intf;
	enum ps_state ps_state;
	struct rsi_ps_info ps_info;
	spinlock_t ps_lock;
	u32 isr_pending;
#ifdef CONFIG_VEN_RSI_DEBUGFS
	struct rsi_debugfs *dfsentry;
	u8 num_debugfs_entries;
#endif

	struct timer_list bl_cmd_timer;
	u8 blcmd_timer_expired;
	u32 flash_capacity;
	u32 tx_blk_size;
	u32 common_hal_fsm;
	u8 eeprom_init;
	struct eepromrw_info eeprom;
	u32 interrupt_status;

	u8 dfs_region;
	char country[2];
	void *rsi_dev;

	struct rsi_host_intf_ops *host_intf_ops;
	int (*check_hw_queue_status)(struct rsi_hw *adapter, u8 q_num);
	int (*rx_urb_submit)(struct rsi_hw *adapter, u8 ep_num);
	int (*determine_event_timeout)(struct rsi_hw *adapter);
	void (*process_isr_hci)(struct rsi_hw *adapter);
	int  (*check_intr_status_reg)(struct rsi_hw *adapter);
};

struct rsi_host_intf_ops {
	int (*read_pkt)(struct rsi_hw *adapter, u8 *pkt, u32 len);
	int (*write_pkt)(struct rsi_hw *adapter, u8 *pkt, u32 len);
	int (*master_access_msword)(struct rsi_hw *adapter, u16 ms_word);
	int (*read_reg_multiple)(struct rsi_hw *adapter, u32 addr,
				 u8 *data, u16 count);
	int (*write_reg_multiple)(struct rsi_hw *adapter, u32 addr,
				  u8 *data, u16 count);
	int (*master_reg_read)(struct rsi_hw *adapter, u32 addr,
			       u32 *read_buf, u16 size);
	int (*master_reg_write)(struct rsi_hw *adapter,
				unsigned long addr, unsigned long data,
				u16 size);
	int (*load_data_master_write)(struct rsi_hw *adapter, u32 addr,
				      u32 instructions_size, u16 block_size,
				      u8 *fw);
};

#endif
