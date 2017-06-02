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

#ifndef __RSI_MAIN_H__
#define __RSI_MAIN_H__

#include <linux/string.h>
#include <linux/skbuff.h>
#include <net/mac80211.h>
#include <linux/etherdevice.h>
#include <linux/version.h>

struct rsi_hw;

#include "rsi_ps.h"

#define DRV_VER				"RS9113.NB0.NL.GNU.LNX.1.2.RC12"

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

extern u16 ven_rsi_zone_enabled;
extern __printf(2, 3) void ven_rsi_dbg(u32 zone, const char *fmt, ...);
void rsi_hex_dump(u32 zone, char *msg_str, const u8 *msg, u32 len);

#define RSI_MAX_VIFS                    3
#define NUM_EDCA_QUEUES                 4
#define IEEE80211_ADDR_LEN              6
#define FRAME_DESC_SZ                   16
#define MIN_802_11_HDR_LEN              24

#define DATA_QUEUE_WATER_MARK           400
#define MIN_DATA_QUEUE_WATER_MARK       300
#define BK_DATA_QUEUE_WATER_MARK	600
#define BE_DATA_QUEUE_WATER_MARK	3200
#define VI_DATA_QUEUE_WATER_MARK	3900
#define VO_DATA_QUEUE_WATER_MARK	4500
#define MULTICAST_WATER_MARK            200
#define MAC_80211_HDR_FRAME_CONTROL     0
#define WME_NUM_AC                      4
#define NUM_SOFT_QUEUES                 6
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

#define RSI_MAX_ASSOC_STAS		32
#define RSI_MAX_COEX_ASSOC_STAS		4
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

/* WoWLAN flags */
#define RSI_WOW_ENABLED			BIT(0)
#define RSI_WOW_NO_CONNECTION		BIT(1)

#define MAX_REG_COUNTRIES		30
#define NL80211_DFS_WORLD		4

struct version_info {
	u16 major;
	u16 minor;
	u8 release_num;
	u8 patch_num;
	union {
		struct {
			u8 fw_ver[8];
		} info;
	} ver;
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
	MGMT_SOFT_Q,
	MGMT_BEACON_Q
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
	u8 vap_id;
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
	u16 seq_start[IEEE80211_NUM_ACS];
	bool start_tx_aggr[IEEE80211_NUM_TIDS];
};

struct rsi_hw;

struct rsi_common {
	struct rsi_hw *priv;
	struct vif_priv vif_info[RSI_MAX_VIFS];

	bool mgmt_q_block;
	char driver_ver[32];
	struct version_info lmac_ver;

	struct rsi_thread tx_thread;
#ifdef CONFIG_SDIO_INTR_POLL
	struct rsi_thread sdio_intr_poll_thread;
#endif
	struct sk_buff_head tx_queue[NUM_EDCA_QUEUES + 2];

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
	u8  start_bgscan;
	u8 bgscan_probe_req[1500];
	int bgscan_probe_req_len;
	u16 bgscan_seq_ctrl;
	u8 mac80211_cur_channel;

	bool hw_data_qs_blocked;
	u8 driver_mode;
	u8 coex_mode;
	u16 oper_mode;
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
	bool suspend_in_prog;
	bool hibernate_resume;
#ifdef CONFIG_VEN_RSI_WOW
	u8 wow_flags;
#endif

#if defined (CONFIG_VEN_RSI_BT_ALONE) || defined(CONFIG_VEN_RSI_COEX)
	void *hci_adapter;
#endif

#ifdef CONFIG_VEN_RSI_COEX
	void *coex_cb;
#endif

	/* AP mode related */
	u8 beacon_enabled;
	u16 beacon_interval;
	u16 beacon_cnt;
	u8 dtim_cnt;
	u16 bc_mc_seqno;
	struct rsi_sta stations[RSI_MAX_ASSOC_STAS + 1];
	int num_stations;
	int max_stations;
	struct ieee80211_channel *ap_channel;
	struct ieee80211_key_conf *key;
	u8 eapol4_confirm;

	/* Wi-Fi direct mode related */
	bool p2p_enabled;
	struct timer_list roc_timer;
	struct ieee80211_vif *roc_vif;
	int last_vap_type;
	u8 last_vap_addr[6];
	u8 last_vap_id;

        struct semaphore tx_bus_lock;

#ifdef CONFIG_HW_SCAN_OFFLOAD
	struct cfg80211_scan_request *scan_request;
	struct ieee80211_vif *scan_vif;
	bool scan_in_prog;
	struct workqueue_struct *scan_workqueue;
	struct work_struct scan_work;
	struct rsi_event chan_set_event;
	struct rsi_event probe_cfm_event;
	struct rsi_event chan_change_event;
	struct rsi_event cancel_hw_scan_event;
	struct timer_list scan_timer;
	bool hw_scan_cancel;
#endif
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
	int sc_nvifs;
	enum host_intf rsi_host_intf;
	enum ps_state ps_state;
	struct rsi_ps_info ps_info;
	spinlock_t ps_lock;
	u32 isr_pending;
	u32 usb_buffer_status_reg;
#ifdef CONFIG_VEN_RSI_DEBUGFS
	struct rsi_debugfs *dfsentry;
	u8 num_debugfs_entries;
#endif

	char *fw_file_name;
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
	bool peer_notify;
	void *rsi_dev;

	struct rsi_host_intf_ops *host_intf_ops;
	int (*check_hw_queue_status)(struct rsi_hw *adapter, u8 q_num);
	int (*rx_urb_submit)(struct rsi_hw *adapter, u8 ep_num);
	int (*determine_event_timeout)(struct rsi_hw *adapter);
	void (*process_isr_hci)(struct rsi_hw *adapter);
	int  (*check_intr_status_reg)(struct rsi_hw *adapter);
};

void rsi_print_version(struct rsi_common *common);
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
	int (*rsi_check_bus_status)(struct rsi_hw *adapter);
	int (*check_hw_queue_status)(struct rsi_hw *adapter, u8 q_num);
};

#endif
