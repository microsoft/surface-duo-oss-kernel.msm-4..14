/* Copyright (c) 2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __FSM_ORU_FWD_IMP_H__
#define __FSM_ORU_FWD_IMP_H__

#include <linux/types.h>
#include <linux/if_ether.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <linux/workqueue.h>
#include <linux/netdevice.h>
#include <linux/spinlock_types.h>
#include <linux/hrtimer.h>
#include <linux/fsm_dp_ioctl.h>
#include <linux/fsm_oru_fwd.h>

#define FSM_ORU_FWD_NAME             "fsm-oru-forwarder"
#define ECPRI_ETHER_TYPE 0xAEFE /* eCPRI ether type */

#define ECPRI_UDP_SERVER_PORT 8080
#define DU_UDP_ECPRI_PORT 9090
#define DEFAULT_MY_IP_ADDR 0xc0a86401
#define FSM_IPV4_HEADER_SIZE sizeof(struct iphdr)
#define FSM_IPV4_HEADER_LEN (FSM_IPV4_HEADER_SIZE >> 2)
#define DEFAULT_UPLANE_DL_CONCAT_MIN_DELAY (6)
#define DEFAULT_UPLANE_DL_CONCAT_MAX_DELAY (10)
#define DEFAULT_CPLANE_DL_CONCAT_MIN_DELAY (10)
#define DEFAULT_CPLANE_DL_CONCAT_MAX_DELAY (30)
#define FSM_ORU_MAX_ECPRI_PDU_SIZE (FSM_DP_MAX_DL_MSG_LEN - \
					sizeof(struct fsm_dp_msghdr))




struct ofwd_netdev_priv {
	bool enabled;
	const char *interface_name;
	struct net_device *ndev;
	uint32_t mru;
};

struct ecpri_common_header {
	uint8_t rev_c;
	uint8_t msg_type;
	uint16_t payload_size;
} __attribute__((packed));

#define ECPRI_REV_MASK		0xf0
#define ECPRI_REV_SHIFT		4
#define ECRPI_C_MASK		1
#define ECRPI_REV_SUPPORTED	1  /* support version 1.0, 1.1, 1.2, 2.0 */

/* message type */
#define ECPRI_MSG_IQ_DATA	0
#define ECPRI_MSG_BIT_SEQ	1
#define ECPRI_MSG_RTIME_CNTRL	2
#define ECRRI_MSG_DATA_XFER	3
#define ECRRI_MSG_MEMORY_ACCESS	4
#define ECRRI_MSG_DELAY_MEASURE	5
#define ECRRI_MSG_REMOTE_RESET	6
#define ECRRI_MSG_EVENT_IND	7

#undef FSM_ORU_FWD_TEST

/* statistics */
struct fwd_statistics {
	uint64_t fwd_from_net_cnt;
	uint64_t fwd_tx_cnt;
	uint64_t fwd_tx_err;
	uint64_t fwd_rx_from_device_cnt;
	uint64_t fwd_drop;
	uint64_t fwd_to_net_cnt;
	uint64_t fwd_to_net_err;
	uint64_t fwd_free_ul_buf;
	uint64_t fwd_loopback_cnt;
	uint64_t fwd_netdev_other_cnt;
};

struct fwd_time_stamp {
	uint64_t arrival_cycle;
	uint64_t complete_cycle;
};

#define FWD_TRAFFIC_ARRAY_SIZE 256

#define FSM_QUEUE_MAX  FSM_DP_MAX_SG_IOV_SIZE

struct fsm_oru_fwdr {

	struct fsm_oru_fwd_cfg fwd_cfg;
	struct fsm_oru_fwd_dl_concat_cfg fwd_dl_concat_cfg;
	uint8_t  bogus_du_mac[ETH_ALEN];
	uint32_t bogus_du_ip;

	bool fwd_enable;
	char fwd_netdev_name[FSM_ORU_FWD_MAX_STR_LEN];

	void *fsm_dp_tx_handle;
	void *fsm_dp_rx_handle;
	bool fwd_has_target_eth;
	bool fwd_has_target_ip;
	unsigned char fwd_target_eth[ETH_ALEN];

	uint16_t fwd_vlan_id;
	uint8_t fwd_vlan_priority;
	bool fwd_vlan_enable;

	/* Use ether type or IP/UDP */
	bool fwd_use_ip;

	/* default DU UDP port */
	uint16_t fwd_du_udp_port;
	/* default My UDP port */
	uint16_t fwd_my_udp_port;

	/* default DU IP: in host order  */
	/* set to broadcast for learning */
	uint32_t fwd_du_ip_addr;

	/* default My IP: 192.168.100.01 of eth1 in host order */
	uint32_t fwd_my_ip_addr;
	uint16_t fwd_ip_hdr_id;

	uint32_t fwd_dl_cycles_pkt_cnt;
	uint32_t fwd_ul_cycles_pkt_cnt;
	uint64_t fwd_dl_cycles;
	uint64_t fwd_ul_cycles;

	uint32_t fwd_ul_traffic_index;
	bool fwd_ul_traffic_collect_done;
	bool fwd_ul_traffic_collect;
	struct fwd_time_stamp fwd_ul_traffic[FWD_TRAFFIC_ARRAY_SIZE];

	uint32_t fwd_dl_traffic_index;
	bool fwd_dl_traffic_collect_done;
	bool fwd_dl_traffic_collect;
	struct fwd_time_stamp fwd_dl_traffic[FWD_TRAFFIC_ARRAY_SIZE];

	bool fwd_dl_concat;
	struct workqueue_struct *fsm_oru_wq;

	spinlock_t fwd_uplane_lock;
	struct hrtimer fsm_oru_concat_uplane_hrtimer;
	struct work_struct fsm_oru_uplane_work;
	uint32_t fwd_queue_uplane_index;
	uint32_t fwd_queue_uplane_length;
	uint32_t fwd_queue_uplane_seg;
	uint64_t fwd_queue_uplane_1st_cycle;
	struct sk_buff *skb_uplane_queue[FSM_QUEUE_MAX];

	spinlock_t fwd_cplane_lock;
	struct hrtimer fsm_oru_concat_cplane_hrtimer;
	struct work_struct fsm_oru_cplane_work;
	uint32_t fwd_queue_cplane_index;
	uint32_t fwd_queue_cplane_length;
	uint32_t fwd_queue_cplane_seg;
	uint64_t fwd_queue_cplane_1st_cycle;
	struct sk_buff *skb_cplane_queue[FSM_QUEUE_MAX];

	uint32_t fwd_queue_max;
	uint32_t fwd_dl_max_pdu_size;

	uint32_t fwd_dl_concat_uplane_min_delay;
	uint32_t fwd_dl_concat_uplane_max_delay;
	uint32_t fwd_dl_concat_uplane_min_delay_cycle;
	uint32_t fwd_dl_concat_uplane_max_delay_cycle;

	uint32_t fwd_dl_concat_cplane_min_delay;
	uint32_t fwd_dl_concat_cplane_max_delay;
	uint32_t fwd_dl_concat_cplane_min_delay_cycle;
	uint32_t fwd_dl_concat_cplane_max_delay_cycle;

	struct fwd_statistics fwd_stats;
};

#endif /* __FSM_ORU_FWD_IMP_H__ */
