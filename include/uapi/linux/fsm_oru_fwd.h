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
#ifndef _FSM_ORU_FWD_H_
#define _FSM_ORU_FWD_H_

#include <linux/netlink.h>
#include <linux/if_ether.h>

#define FSM_ORU_MSG_TYPE_UPLANE	(0)
#define FSM_ORU_MSG_TYPE_CPLANE	(1)

/* Netlink API */
#define FSM_ORU_FWD_NETLINK_PROTO (MAX_LINKS - 1)
#define FSM_ORU_FWD_MAX_STR_LEN  16
#define FSM_ORU_FWD_NL_DATA_MAX_LEN 64
#define FSM_ORU_FWD_NETLINK_MSG_COMMAND    0
#define FSM_ORU_FWD_NETLINK_MSG_RETURNCODE 1
#define FSM_ORU_FWD_NETLINK_MSG_RETURNDATA 2

struct fsm_oru_fwd_cfg {
	uint8_t  dev[FSM_ORU_FWD_MAX_STR_LEN];
	uint16_t ether_type;
	uint8_t  du_mac[ETH_ALEN];
	bool     use_ip;
	uint32_t du_ip_addr;	/* DU IP address in host order */
	uint32_t my_ip_addr;	/* My IP address of dev interface */
	uint16_t du_udp_port;	/* DU UDP port */
	uint16_t my_udp_port;	/* My UDP port */
	uint8_t  vlan_enabled;
	uint8_t  vlan_id;
	uint8_t  vlan_priority;
};

struct fsm_oru_fwd_dl_concat_cfg {
	bool     dl_concat;
	uint32_t dl_concat_uplane_min_delay;
	uint32_t dl_concat_uplane_max_delay;
	uint32_t dl_concat_cplane_min_delay;
	uint32_t dl_concat_cplane_max_delay;
	uint32_t dl_max_pdu_size;
};

struct fwd_op_status {
	uint8_t  dev[FSM_ORU_FWD_MAX_STR_LEN];
	uint16_t ether_type;
	uint8_t  du_mac[ETH_ALEN];
	bool     use_ip;
	uint32_t du_ip_addr;
	uint32_t my_ip_addr;
	uint16_t du_udp_port;
	uint16_t my_udp_port;
	uint8_t  vlan_enabled;
	uint16_t vlan_id;
	uint8_t  vlan_priority;
	bool     dl_concat;
	uint32_t dl_concat_uplane_min_delay;
	uint32_t dl_concat_uplane_max_delay;
	uint32_t dl_concat_cplane_min_delay;
	uint32_t dl_concat_cplane_max_delay;
	uint32_t dl_max_pdu_size;
	uint8_t  on_off;
};

struct fwd_stats {
	uint64_t fwd_from_net_cnt;
	uint64_t fwd_tx_cnt;
	uint64_t fwd_tx_err;
	uint64_t fwd_rx_from_device_cnt;
	uint64_t fwd_drop;
	uint64_t fwd_to_net_cnt;
	uint64_t fwd_to_net_err;
	uint64_t fwd_free_ul_buf;
	uint64_t fwd_netdev_other_cnt;
};

struct fsm_oru_fwd_nl_msg_s {
	uint16_t reserved;
	uint16_t message_type;
	uint16_t reserved2:14;
	uint16_t crd:2;
	union {
		uint16_t arg_length;
		uint16_t return_code;
	};
	union {
		uint8_t data[FSM_ORU_FWD_NL_DATA_MAX_LEN];
		struct fsm_oru_fwd_cfg fwd_config;
		struct fwd_op_status fwd_op_status;
		struct fwd_stats fwd_stats;
		bool enable;
		struct fsm_oru_fwd_dl_concat_cfg fwd_dl_concat_config;
	};
};

enum fsm_oru_fwd_netlink_message_types_e {

	/*
	 * FSM_ORU_FWD_NETLINK_SET_CONFIG:
	 *	sets the configuration for the forwarder
	 *
	 * Args: char[] dev_name: Null terminated ASCII string,
	 *			max length: 15
c	 *       uint8_t[6] du: mac address. If 0xffffff,
	 *			then forwarder will do learning.
	 *			This is default.
	 *       uint16_t ether_type: Default 0xaefe for eCPRI.
	 *       bool vlan_enabled:  Default, not enabled.
	 *	 uint16_t vlan_id: vlan ID if vlan enabled.
	 *	 uint_8_t vlan_priority: vlan priority on 802.1Q header
	 * Returns: status code
	 */
	FSM_ORU_FWD_NETLINK_SET_CONFIG,

	/*
	 * FSM_ORU_FWD_NETLINK_GET_CONFIG:
	 *	Gets the configuration for the forwarder
	 * Args:
	 * Returns:
	 *       char[] dev_name: Null terminated ASCII string,
	 *				max length: 15
c	 *       unsigned char[6] du: mac address. If 0xffffff,
	 *				then unknown
	 *       uint16_t ether_type: Default 0xaefe for eCPRI.
	 *       bool vlan_enabled:
	 *	 uint16_t vlan_id: vlan ID if vlan enabled.
	 *	 uint_8_t vlan_priority: priority on 802.1Q header
	 */
	FSM_ORU_FWD_NETLINK_GET_CONFIG,

	/*
	 * FSM_ORU_FWD_NETLINK_CNTL: Control forwarder operation
	 *
	 * Args:
	 *       bool control_on_off :
i	 *		 true, turn on forwarder
	 *		false, turn off forwarderf
	 * Returns: status code
	 */
	FSM_ORU_FWD_NETLINK_CNTL,

	/*
	 * FSM_ORU_FWD_NETLINK_STATE: Get Control forwarder state.
	 * Args:
	 * Returns:
	 *       char[] dev_name: Null terminated ASCII string,
	 *					max length: 15
c	 *       unsigned char[6] du: mac address. If 0xffffff,
	 *					then unknown
	 *       uint16_t ether_type: Default 0xaefe for eCPRI
	 *       bool vlan_enabled:
	 *	 uint16_t vlan_id: if vlan enabled.
	 *	 uint_8_t vlan_priority: priority on 802.1Q header
	 *       uint8_t on_off: on, forwarder is on
	 *			off, forwarder is off. Default is on
	 */
	FSM_ORU_FWD_NETLINK_STATE,


	/*
	 * FSM_ORU_FWD_NETLINK_STATISTICS: get forwarder statistics.
	 * Args:
	 * Returns:
	 *	fsm_dp_fwd_nl_msg_s.fwd_stats
	 */
	FSM_ORU_FWD_NETLINK_STATISTICS,

	/*
	 * FSM_ORU_FWD_NETLINK_ECHO: forwarder echo netlink smg.
	 * Args:
	 *      data: msg to echo
	 * Returns:
	 *	data: echoed msg
	 */
	FSM_ORU_FWD_NETLINK_ECHO,


	/*
	 * FSM_ORU_FWD_NETLINK_SET_CONCAT_CONFIG:
	 *	Set the concatenation configuration for the forwarder
	 *
	 * Args:
	 *      struct fsm_oru_fwd_dl_concat_cfg
	 *
	 * Returns: status code
	 */
	FSM_ORU_FWD_NETLINK_SET_CONCAT_CONFIG,

	/*
	 * FSM_ORU_FWD_NETLINK_GET_CONCAT_CONFIG:
	 *	Get the concatenation configuration for the forwarder
	 * Args:
	 * Returns:
	 *      struct fsm_oru_fwd_dl_concat_cfg
	 */
	FSM_ORU_FWD_NETLINK_GET_CONCAT_CONFIG
};

enum  fsm_oru_fwd_netlink_config_return_e {
	/* OK */
	FSM_ORU_FWD_CONFIG_OK,
	/* Err */
	FSM_ORU_FWD_CONFIG_ERR
};

#endif /* _FSM_ORU_FWD_H_ */
