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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/rtnetlink.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/if_vlan.h>
#include <linux/timer.h>
#include <linux/if_arp.h>
#include <net/netlink.h>
#include <net/checksum.h>
#include <net/arp.h>
#include <net/ip.h>
#include <net/neighbour.h>
#include <linux/fsm_dp_intf.h>
#include <linux/fsm_oru_fwd.h>
#include <linux/debugfs.h>

#define FSM_ORU_FWD_NAME             "fsm-oru-forwarder"
#define ECPRI_ETHER_TYPE 0xAEFE /* eCPRI ether type */

#define ECPRI_UDP_SERVER_PORT 8080
#define DU_UDP_ECPRI_PORT 9090
#define DEFAULT_MY_IP_ADDR 0xc0a86401
#define FSM_IPV4_HEADER_SIZE sizeof(struct iphdr)
#define FSM_IPV4_HEADER_LEN (FSM_IPV4_HEADER_SIZE >> 2)

static unsigned int oru_fwd_etype = ECPRI_ETHER_TYPE;
module_param(oru_fwd_etype, int, 0660);

static struct net_device *fsm_oru_forwarder_netdev;
static struct packet_type fsm_oru_fwd_pt;
static void *fsm_dp_tx_handle;
static void *fsm_dp_rx_handle;

static unsigned char fsm_oru_fwd_target_eth[ETH_ALEN] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

static bool fsm_oru_fwd_has_target_eth;
static bool fsm_oru_fwd_has_target_ip;
static unsigned int _fwd_cycles_per_ms;

static bool fsm_oru_fwd_enable;
static char fsm_oru_fwd_netdev_name[FSM_ORU_FWD_MAX_STR_LEN] = "eth1";
static uint16_t fsm_oru_fwd_vlan_id = 100;
static uint8_t fsm_oru_fwd_vlan_priority;
static bool fsm_oru_fwd_vlan_enable;

/* Use ether type or IP/UDP */
static bool fsm_oru_use_ip;

/* default DU UDP port */
static uint16_t fsm_oru_du_udp_port = DU_UDP_ECPRI_PORT;
/* default My UDP port */
static uint16_t fsm_oru_my_udp_port = ECPRI_UDP_SERVER_PORT;

/* default DU IP: in host order  */
/* set to broadcast for learning */
static uint32_t fsm_oru_du_ip_addr = 0xffffffff;

/* default My IP: 192.168.100.01 of eth1 in host order */
static uint32_t fsm_oru_my_ip_addr = DEFAULT_MY_IP_ADDR;
static uint16_t fwd_oru_ip_hdr_id = 0x1234;

static struct fsm_oru_fwd_cfg fsm_oru_fwd_cfg = {
	"eth1",
	ECPRI_ETHER_TYPE,
	{0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
	false,
	0xffffffff, DEFAULT_MY_IP_ADDR,
	DU_UDP_ECPRI_PORT, ECPRI_UDP_SERVER_PORT,
	false, 0,
	0
};
static uint8_t  bogus_du_mac[ETH_ALEN] = {0x00, 0x10, 0xf3, 0x81, 0x92, 0x03};
static uint32_t bogus_du_ip = 0xc0a86402; /* host order */
static struct sock *nl_socket_handle;
static void fsm_oru_fwd_netlink_msg_handler(struct sk_buff *skb);
static struct netlink_kernel_cfg fsm_oru_fwd_netlink_cfg = {
	.input = fsm_oru_fwd_netlink_msg_handler
};
static struct net_device *_oru_netdev;


struct ofwd_netdev_priv {
	bool enabled;
	const char *interface_name;
	struct net_device *ndev;
	uint32_t mru;
};

struct ofwd_netdev_priv *ofwd_netdev_priv;

#undef FSM_ORU_FWD_TEST
#undef FSM_ORU_FWD_MEASURE_CYCLE

#define MEM_DUMP_COL_WIDTH 16
#define MAX_MEM_DUMP_SIZE 256

#define DEFINE_DEBUGFS_OPS(name, __read, __write)               \
static int name ##_open(struct inode *inode, struct file *file) \
{                                                               \
	return single_open(file, __read, inode->i_private);     \
}                                                               \
static const struct file_operations name ##_ops = {             \
	.open    = name ## _open,                               \
	.read = seq_read,                                       \
	.write = __write,                                       \
	.llseek = seq_lseek,                                    \
	.release = single_release,                              \
}

static struct dentry *__dent;

/* statistics */
static u64 _fwd_from_net_cnt;
static u64 _fwd_tx_cnt;
static u64 _fwd_tx_err;
static u64 _fwd_rx_from_device_cnt;
static u64 _fwd_drop;
static u64 _fwd_to_net_cnt;
static u64 _fwd_to_net_err;
static u64 _fwd_free_ul_buf;
static u64 _fwd_loopback_cnt;
static u64 _fwd_netdev_other_cnt;

#ifdef FSM_ORU_FWD_MEASURE_CYCLE
static u64 _fwd_ul_cycles;
static u64 _fwd_dl_cycles;
#endif

static bool _fwd_loop_back; /* recv and loop back to net control */

static int _fsm_oru_forwarder_rcv(
	struct sk_buff *skb,
	struct net_device *ifp,
	struct packet_type *pt,
	struct net_device *orig_dev
);
static int _fsm_oru_fwd_enable(void);
static int _fsm_oru_fwd_disable(void);

static int debugfs_fwd_status_show(struct seq_file *s, void *unused)
{
#ifdef FSM_ORU_FWD_MEASURE_CYCLE
	unsigned int avg_dl;
	unsigned int avg_ul;
#endif

	seq_printf(s, "FWD enabled:          %d\n", fsm_oru_fwd_enable);
	seq_printf(s, "FWD type              %s\n",
				(fsm_oru_use_ip) ? "UDP/IP" : "Ethertype");
	if (fsm_oru_use_ip)
		seq_printf(s, "FWD UDP port:         %d\n",
						fsm_oru_my_udp_port);
	seq_printf(s, "FWD loopback enabled: %d\n", _fwd_loop_back);
	seq_printf(s, "FWD_FROM_NET_CNT:     %llu\n", _fwd_from_net_cnt);
	seq_printf(s, "FWD_TO_DEVICE:        %llu\n", _fwd_tx_cnt);
	seq_printf(s, "FWD_TO_DEVICE_ERR:    %llu\n", _fwd_tx_err);
	seq_printf(s, "FWD_FROM_DEVICE:      %llu\n", _fwd_rx_from_device_cnt);
	seq_printf(s, "FWD_FROM_DEVICE_DROP: %llu\n", _fwd_drop);
	seq_printf(s, "FWD_TO_NET_ERR:       %llu\n", _fwd_to_net_err);
	seq_printf(s, "FWD_TO_NET_CNT:       %llu\n", _fwd_to_net_cnt);
	seq_printf(s, "FWD Free UL buffers:  %llu\n", _fwd_free_ul_buf);
	seq_printf(s, "FWD loopback CNT:     %llu\n", _fwd_loopback_cnt);
	seq_printf(s, "FWD netdev other CNT: %llu\n", _fwd_netdev_other_cnt);
#ifdef FSM_ORU_FWD_MEASURE_CYCLE
	seq_printf(s, "System Cycles per ms  %u\n", _fwd_cycles_per_ms);
	avg_ul = (_fwd_ul_cycles * 10000 / _fwd_rx_from_device_cnt) /
							_fwd_cycles_per_ms;
	avg_dl = (_fwd_dl_cycles * 10000 / _fwd_rx_from_device_cnt) /
							_fwd_cycles_per_ms;
	if (_fwd_from_net_cnt)
		seq_printf(s, "Avg UL Fwd us    %4u.%1u\n",
						avg_ul / 10, avg_ul % 10);
	if (_fwd_from_net_cnt)
		seq_printf(s, "Avg DL Fwd us    %4u.%1u\n",
						avg_dl / 10, avg_dl % 10);
#endif
	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_fwd_status, debugfs_fwd_status_show, NULL);

static ssize_t debugfs_fwd_control_set(
	struct file *s,
	const char __user *buf,
	size_t count,
	loff_t *ppos
)
{
	unsigned int enable = 0;

	if (kstrtouint_from_user(buf, count, 0, &enable))
		return -EFAULT;
	if (enable)
		_fsm_oru_fwd_enable();
	else
		_fsm_oru_fwd_disable();
	return count;
}
DEFINE_DEBUGFS_OPS(debugfs_fwd_control, NULL, debugfs_fwd_control_set);

static int debugfs_fwd_loopback_get(struct seq_file *s, void *unused)
{

	seq_printf(s, "FWD loopback:       %d\n",   _fwd_loop_back);
	return 0;
}

static ssize_t debugfs_fwd_loopback_set(
	struct file *s,
	const char __user *buf,
	size_t count,
	loff_t *ppos
)
{
	unsigned int enable = 0;

	if (kstrtouint_from_user(buf, count, 0, &enable))
		return -EFAULT;
	_fwd_loop_back = enable;
	return count;
}

DEFINE_DEBUGFS_OPS(
	debugfs_fwd_loopback,
	debugfs_fwd_loopback_get,
	debugfs_fwd_loopback_set
);

static int fsm_oru_fwd_debugfs_init(void)
{
	struct dentry *entry = NULL;

	if (unlikely(__dent))
		return -EBUSY;
	__dent = debugfs_create_dir(FSM_ORU_FWD_NAME, 0);
	if (IS_ERR(__dent))
		return -ENOMEM;
	entry = debugfs_create_file("status", 0444, __dent, NULL,
		&debugfs_fwd_status_ops);
	if (!entry)
		goto err;
	entry = debugfs_create_file("control", 0444, __dent, NULL,
		&debugfs_fwd_control_ops);
	if (!entry)
		goto err;
	entry = debugfs_create_file("loopback", 0444, __dent, NULL,
		&debugfs_fwd_loopback_ops);
	if (!entry)
		goto err;
	return 0;
err:
	debugfs_remove_recursive(__dent);
	__dent = NULL;
	return -ENOMEM;
}

static void fsm_oru_fwd_debugfs_cleanup(void)
{
	debugfs_remove_recursive(__dent);
	__dent = NULL;
}

static struct sock *_fsm_oru_fwd_start_netlink(void)
{

	return netlink_kernel_create(&init_net,
					FSM_ORU_FWD_NETLINK_PROTO,
					&fsm_oru_fwd_netlink_cfg);
}

static int _fsm_oru_fwd_enable(void)
{
	struct net_device *netdev;

	if (fsm_oru_fwd_enable)
		return 0;

	netdev = dev_get_by_name(&init_net, fsm_oru_fwd_netdev_name);
	if (!netdev) {
		pr_err("%s: can not get device %s\n", __func__,
					fsm_oru_fwd_netdev_name);
		return -ENODEV;
	}
	fsm_oru_forwarder_netdev = netdev;
	fsm_oru_fwd_pt.dev = netdev;
	fsm_oru_fwd_pt.type = htons(oru_fwd_etype);
	fsm_oru_fwd_pt.func = _fsm_oru_forwarder_rcv;
	dev_add_pack(&fsm_oru_fwd_pt);

	fsm_oru_fwd_has_target_eth = !(is_broadcast_ether_addr(
						fsm_oru_fwd_target_eth));
	fsm_oru_fwd_has_target_ip = !((fsm_oru_du_ip_addr == 0xffffffff) ||
					(fsm_oru_du_ip_addr == 0));
	if (!fsm_oru_fwd_has_target_ip && fsm_oru_use_ip)
		fsm_oru_fwd_has_target_eth = false;
	fsm_oru_fwd_enable = true;
	return 0;
}

static int _fsm_oru_fwd_disable(void)
{
	if (fsm_oru_forwarder_netdev)
		dev_remove_pack(&fsm_oru_fwd_pt);
	fsm_oru_forwarder_netdev = NULL;
	fsm_oru_fwd_has_target_eth = false;
	fsm_oru_fwd_has_target_ip = false;
	fsm_oru_fwd_enable = false;
	return 0;
}

static void fsm_oru_fwd_nl_get_cfg(
	struct fsm_oru_fwd_nl_msg_s *fwd_header,
	struct fsm_oru_fwd_nl_msg_s *resp_fwd
)
{
	unsigned char size =
		sizeof(((struct fsm_oru_fwd_nl_msg_s *)0)
				->fwd_config);
	resp_fwd->crd = FSM_ORU_FWD_NETLINK_MSG_RETURNDATA;
	resp_fwd->arg_length = size;
	resp_fwd->fwd_config = fsm_oru_fwd_cfg;
}

static void fsm_oru_fwd_nl_set_cfg(
	struct fsm_oru_fwd_nl_msg_s *fwd_header,
	struct fsm_oru_fwd_nl_msg_s *resp_fwd
)
{
	struct net_device *netdev;

	resp_fwd->crd = FSM_ORU_FWD_NETLINK_MSG_RETURNCODE;
	netdev = dev_get_by_name(&init_net, fwd_header->fwd_config.dev);
	if (!netdev) {
		pr_err("%s: can not get device %s\n", __func__,
					fwd_header->fwd_config.dev);
		resp_fwd->return_code = FSM_ORU_FWD_CONFIG_ERR;
		return;
	}
	fsm_oru_fwd_cfg = fwd_header->fwd_config;
	resp_fwd->return_code = FSM_ORU_FWD_CONFIG_OK;
}

static void fsm_oru_fwd_nl_control(
	struct fsm_oru_fwd_nl_msg_s *fwd_header,
	struct fsm_oru_fwd_nl_msg_s *resp_fwd
)
{
	resp_fwd->crd = FSM_ORU_FWD_NETLINK_MSG_RETURNCODE;
	if (fwd_header->enable) {

		memcpy(fsm_oru_fwd_netdev_name, fsm_oru_fwd_cfg.dev,
					FSM_ORU_FWD_MAX_STR_LEN);
		oru_fwd_etype = fsm_oru_fwd_cfg.ether_type;
		memcpy(fsm_oru_fwd_target_eth, fsm_oru_fwd_cfg.du_mac,
					ETH_ALEN);
		fsm_oru_fwd_vlan_enable = fsm_oru_fwd_cfg.vlan_enabled;
		fsm_oru_fwd_vlan_id = fsm_oru_fwd_cfg.vlan_id;
		fsm_oru_fwd_vlan_priority = fsm_oru_fwd_cfg.vlan_priority;

		fsm_oru_use_ip = fsm_oru_fwd_cfg.use_ip;
		fsm_oru_du_udp_port = fsm_oru_fwd_cfg.du_udp_port;
		fsm_oru_my_udp_port = fsm_oru_fwd_cfg.my_udp_port;
		fsm_oru_du_ip_addr = fsm_oru_fwd_cfg.du_ip_addr;
		fsm_oru_my_ip_addr = fsm_oru_fwd_cfg.my_ip_addr;

		resp_fwd->return_code = _fsm_oru_fwd_enable();
	} else
		resp_fwd->return_code = _fsm_oru_fwd_disable();
}

static void fsm_oru_fwd_nl_statitics(
	struct fsm_oru_fwd_nl_msg_s *fwd_header,
	struct fsm_oru_fwd_nl_msg_s *resp_fwd
)
{
	unsigned char size =
		sizeof(((struct fsm_oru_fwd_nl_msg_s *)0)
				->fwd_stats);
	resp_fwd->fwd_stats.fwd_from_net_cnt = _fwd_from_net_cnt;
	resp_fwd->fwd_stats.fwd_tx_cnt       = _fwd_tx_cnt;
	resp_fwd->fwd_stats.fwd_tx_err       = _fwd_tx_err;
	resp_fwd->fwd_stats.fwd_rx_from_device_cnt = _fwd_rx_from_device_cnt;
	resp_fwd->fwd_stats.fwd_drop         = _fwd_drop;
	resp_fwd->fwd_stats.fwd_to_net_cnt   = _fwd_to_net_cnt;
	resp_fwd->fwd_stats.fwd_to_net_err   = _fwd_to_net_err;
	resp_fwd->fwd_stats.fwd_free_ul_buf  = _fwd_free_ul_buf;
	resp_fwd->fwd_stats.fwd_netdev_other_cnt  = _fwd_netdev_other_cnt;
	resp_fwd->crd = FSM_ORU_FWD_NETLINK_MSG_RETURNDATA;
	resp_fwd->arg_length = size;
}

static void fsm_oru_fwd_nl_echo(
	struct fsm_oru_fwd_nl_msg_s *fwd_header,
	struct fsm_oru_fwd_nl_msg_s *resp_fwd
)
{
	memcpy(resp_fwd->data, fwd_header->data, FSM_ORU_FWD_NL_DATA_MAX_LEN);
	resp_fwd->arg_length = FSM_ORU_FWD_NL_DATA_MAX_LEN;
	resp_fwd->crd = FSM_ORU_FWD_NETLINK_MSG_RETURNDATA;
}

static void fsm_oru_fwd_nl_state(
	struct fsm_oru_fwd_nl_msg_s *fwd_header,
	struct fsm_oru_fwd_nl_msg_s *resp_fwd
)
{
	unsigned char size =
		sizeof(((struct fsm_oru_fwd_nl_msg_s *)0)
				->fwd_op_status);
	resp_fwd->arg_length = size;
	resp_fwd->crd = FSM_ORU_FWD_NETLINK_MSG_RETURNDATA;
	memcpy(resp_fwd->fwd_op_status.dev, fsm_oru_fwd_netdev_name,
					FSM_ORU_FWD_MAX_STR_LEN);
	resp_fwd->fwd_op_status.ether_type = oru_fwd_etype;
	memcpy(resp_fwd->fwd_op_status.du_mac, fsm_oru_fwd_target_eth,
					ETH_ALEN);
	resp_fwd->fwd_op_status.vlan_enabled = fsm_oru_fwd_vlan_enable;
	resp_fwd->fwd_op_status.vlan_id = fsm_oru_fwd_vlan_id;
	resp_fwd->fwd_op_status.vlan_priority = fsm_oru_fwd_vlan_priority;
	resp_fwd->fwd_op_status.on_off = fsm_oru_fwd_enable;
}

void fsm_oru_fwd_netlink_msg_handler(struct sk_buff *skb)
{
	struct nlmsghdr *nlmsg_header, *resp_nlmsg;
	struct fsm_oru_fwd_nl_msg_s *fwd_header, *resp_fwd;
	int return_pid, response_data_length;
	struct sk_buff *skb_response;

	response_data_length = 0;
	nlmsg_header = (struct nlmsghdr *)skb->data;
	fwd_header = (struct fsm_oru_fwd_nl_msg_s *)nlmsg_data(nlmsg_header);

	if (!nlmsg_header->nlmsg_pid ||
		(nlmsg_header->nlmsg_len < sizeof(struct nlmsghdr) +
			sizeof(struct fsm_oru_fwd_nl_msg_s))) {
		pr_warn("%s: ill-formed netlink msg\n", __func__);
		return;
	}
	return_pid = nlmsg_header->nlmsg_pid;
	skb_response = nlmsg_new(sizeof(struct nlmsghdr)
				+ sizeof(struct fsm_oru_fwd_nl_msg_s),
				GFP_KERNEL);

	if (!skb_response) {
		pr_err("%s: Failed to allocate response buffer\n", __func__);
		return;
	}

	resp_nlmsg = nlmsg_put(skb_response,
				0,
				nlmsg_header->nlmsg_seq,
				NLMSG_DONE,
				sizeof(struct fsm_oru_fwd_nl_msg_s),
				0);
	resp_fwd = nlmsg_data(resp_nlmsg);
	if (!resp_fwd)
		return;
	resp_fwd->message_type = fwd_header->message_type;
	rtnl_lock();
	switch (fwd_header->message_type) {

	case FSM_ORU_FWD_NETLINK_SET_CONFIG:
		fsm_oru_fwd_nl_set_cfg(fwd_header, resp_fwd);
		break;

	case FSM_ORU_FWD_NETLINK_GET_CONFIG:
		fsm_oru_fwd_nl_get_cfg(fwd_header, resp_fwd);
		break;

	case FSM_ORU_FWD_NETLINK_CNTL:
		fsm_oru_fwd_nl_control(fwd_header, resp_fwd);
		break;

	case FSM_ORU_FWD_NETLINK_STATE:
		fsm_oru_fwd_nl_state(fwd_header, resp_fwd);
		break;

	case FSM_ORU_FWD_NETLINK_STATISTICS:
		fsm_oru_fwd_nl_statitics(fwd_header, resp_fwd);
		break;

	case FSM_ORU_FWD_NETLINK_ECHO:
		fsm_oru_fwd_nl_echo(fwd_header, resp_fwd);
		break;

	default:
		break;
	}
	rtnl_unlock();
	nlmsg_unicast(nl_socket_handle, skb_response, return_pid);
}

static int _fwd_do_loopback(
	struct sk_buff *skb,
	struct net_device *ifp
)
{
	int rc = 0;
	struct ethhdr *hdr;
	struct udphdr *udphdr;
	struct iphdr *iph = (struct iphdr *) skb->data;
	uint16_t srcport;
	__be32 src_addr;
	__be32 dst_addr;
	__wsum wsum;
	uint16_t udplen;

	_fwd_loopback_cnt++;
	if (fsm_oru_fwd_vlan_enable) /* NO vlan support for loop back */
		goto drop;
	if (fsm_oru_use_ip)  {
		udphdr = (struct udphdr *) (iph + 1);
		srcport = udphdr->source;
		udphdr->source = udphdr->dest;
		udphdr->dest = srcport;
		udphdr->check = 0;
		udplen = ntohs(udphdr->len);

		dst_addr = iph->saddr;
		src_addr = htonl(fsm_oru_my_ip_addr);

		wsum = csum_partial(udphdr,  udplen, 0);
		udphdr->check = csum_tcpudp_magic(
			src_addr, dst_addr, udplen, IPPROTO_UDP, wsum);

		iph->saddr = src_addr;
		iph->daddr = dst_addr;
		iph->check = 0;
		iph->check = ip_fast_csum(iph, iph->ihl);

		skb_reset_transport_header(skb);
		skb_reset_network_header(skb);
		hdr = skb_push(skb, ETH_HLEN);
		hdr->h_proto = htons(ETH_P_IP);
		ether_addr_copy(hdr->h_source, skb->dev->dev_addr);
		ether_addr_copy(hdr->h_dest, fsm_oru_fwd_target_eth);

	} else {
		hdr = skb_push(skb, ETH_HLEN);
		hdr->h_proto = htons(oru_fwd_etype);
		ether_addr_copy(hdr->h_source, skb->dev->dev_addr);
		ether_addr_copy(hdr->h_dest, fsm_oru_fwd_target_eth);
	}
	skb->dev = ifp;
	rc = dev_queue_xmit(skb);
	if (rc) {
drop:
		_fwd_to_net_err++;
		return NET_RX_DROP;
	}
	_fwd_to_net_cnt++;
	return NET_RX_SUCCESS;
}

static int _fsm_oru_forwarder_rcv(
	struct sk_buff *skb,
	struct net_device *ifp,
	struct packet_type *pt,
	struct net_device *orig_dev
)
{
	struct ethhdr *hdr;
#ifdef FSM_ORU_FWD_MEASURE_CYCLE
	unsigned long cycle_start;
#endif

	if (!skb) {
		_fwd_drop++;
		return NET_RX_DROP;
	}
#ifdef FSM_ORU_FWD_MEASURE_CYCLE
	cycle_start = get_cycles();
#endif
	_fwd_from_net_cnt++;


	hdr = (struct ethhdr *)skb_mac_header(skb);

	if (!fsm_oru_fwd_has_target_eth) {
		fsm_oru_fwd_has_target_eth = true;
		memcpy(fsm_oru_fwd_target_eth, hdr->h_source, ETH_ALEN);
	}

	if (_fwd_loop_back)
		return _fwd_do_loopback(skb, ifp);

	if (!fsm_dp_tx_handle ||
			fsm_dp_tx_skb(fsm_dp_tx_handle, skb)) {
		kfree_skb(skb);
		_fwd_tx_err++;
		return NET_RX_DROP;
	}
	_fwd_tx_cnt++;
#ifdef FSM_ORU_FWD_MEASURE_CYCLE
	_fwd_dl_cycles += (get_cycles() - cycle_start);
#endif
	return  NET_RX_SUCCESS;
}

int fsm_dp_tx_cmplt_cb(struct sk_buff *skb)
{

	kfree_skb(skb);
	return 0;
}

void fsm_oru_forwarder_skb_free(struct sk_buff *skb)
{
	char *buf;

	/* buf is now pointing to the receive buffer start of user data area */
	buf = (char *) skb_uarg(skb);
	if (atomic_dec_and_test((atomic_t *)buf)) {
		fsm_dp_rel_rx_buf(fsm_dp_rx_handle, buf);
		_fwd_free_ul_buf++;
	}
}

static int fsm_oru_fwd_send2_net(
	struct page *page,
	unsigned int page_offset,
	char *orig_buf,
	unsigned int length
)
{
	int rc = 0;
	struct sk_buff *skb;
	struct ethhdr *hdr;
	struct vlan_ethhdr *veh;
	uint32_t hdr_len;
	struct udphdr *udphdr;
	struct iphdr *iph;
	__be32 src_addr;
	__be32 dst_addr;
	__wsum wsum;

	if (!fsm_oru_use_ip)
		hdr_len = 0;
	else
		hdr_len = sizeof(*iph) + sizeof(*udphdr);
	if (fsm_oru_fwd_vlan_enable)
		hdr_len += sizeof(*veh);
	else
		hdr_len += sizeof(*hdr);

	/* allocate skb */
	skb = __dev_alloc_skb(hdr_len, GFP_ATOMIC);
	if (!skb) {
		_fwd_to_net_err++;
		return -ENOMEM;
	}
	skb->dev = fsm_oru_forwarder_netdev;

	if (fsm_oru_use_ip) {
		udphdr = skb_push(skb, sizeof(*udphdr));
		iph = skb_push(skb, sizeof(*iph));
		src_addr = htonl(fsm_oru_my_ip_addr);
		if (fsm_oru_fwd_has_target_ip)
			dst_addr = htonl(fsm_oru_du_ip_addr);
		else
			dst_addr = htonl(bogus_du_ip);
		udphdr->source = htons(fsm_oru_my_udp_port);
		udphdr->dest =  htons(fsm_oru_du_udp_port);
		udphdr->len = htons(length + sizeof(*udphdr));
		udphdr->check = 0;
		wsum = csum_partial(udphdr, sizeof(*udphdr), 0);
		wsum = csum_partial(page_address(page) + page_offset,
					length, wsum);
		udphdr->check = csum_tcpudp_magic(src_addr, dst_addr,
				ntohs(udphdr->len), IPPROTO_UDP, wsum);

		iph->version = IPVERSION; /* IPV4 */
		iph->ihl = FSM_IPV4_HEADER_LEN;
		iph->protocol = IPPROTO_UDP;
		iph->tos = 0;
		iph->tot_len = htons(ntohs(udphdr->len) + FSM_IPV4_HEADER_SIZE);
		iph->id = fwd_oru_ip_hdr_id++;
		iph->frag_off = htons(IP_DF);  /* DF */
		iph->ttl = IPDEFTTL;
		iph->saddr = src_addr;
		iph->daddr = dst_addr;
		iph->check = 0;
		iph->check = ip_fast_csum(iph, iph->ihl);
	}
	/* fill ethernet header */
	if (fsm_oru_fwd_vlan_enable) {
		veh = skb_push(skb, sizeof(*veh));
		veh->h_vlan_proto = htons(ETH_P_8021Q);
		ether_addr_copy(veh->h_source, skb->dev->dev_addr);
		if (fsm_oru_fwd_has_target_eth)
			ether_addr_copy(veh->h_dest, fsm_oru_fwd_target_eth);
		else
			ether_addr_copy(veh->h_dest, bogus_du_mac);
		if (fsm_oru_use_ip)
			veh->h_vlan_encapsulated_proto = htons(ETH_P_IP);
		else
			veh->h_vlan_encapsulated_proto = htons(oru_fwd_etype);
		veh->h_vlan_TCI = htons((fsm_oru_fwd_vlan_id & VLAN_VID_MASK) |
			((fsm_oru_fwd_vlan_priority << VLAN_PRIO_SHIFT) &
							VLAN_PRIO_MASK));
	} else {
		hdr = (struct ethhdr *) skb_push(skb, ETH_HLEN);
		if (fsm_oru_use_ip)
			hdr->h_proto = htons(ETH_P_IP);
		else
			hdr->h_proto = htons(oru_fwd_etype);
		ether_addr_copy(hdr->h_source, skb->dev->dev_addr);
		if (fsm_oru_fwd_has_target_eth)
			ether_addr_copy(hdr->h_dest, fsm_oru_fwd_target_eth);
		else
			ether_addr_copy(hdr->h_dest, bogus_du_mac);
	}

	/* fill skb frag desc */
	skb_fill_page_desc(skb, 0, page, page_offset, length);
	skb->len += length;
	skb->data_len = length;
	skb_reset_network_header(skb);

	/* increase page reference count */
	page_ref_inc(page);

	/* set up skb desctrutor */
	skb->destructor = fsm_oru_forwarder_skb_free;
	skb_shinfo(skb)->destructor_arg = (struct ubuf_info *) orig_buf;

	/* tx to eth dev */
	rc = dev_queue_xmit(skb);
	if (rc)
		_fwd_to_net_err++;
	else
		_fwd_to_net_cnt++;
	return 0;
}

int fsm_oru_fwd_rx_ind_cb(
	struct page *page,
	unsigned int page_offset,
	char *buf,
	unsigned int length
)
{
	int rc = 0;
	struct fsm_dp_msghdr *pfsm_dp_msghdr;
	char *orig_buf = buf;
	unsigned int orig_length = length;
	atomic_t *ref;
	bool aggr;
	int i;
	int num_pkt = 0;
	int good_sent = 0;
	int poff;
	struct fsm_dp_aggrhdr *pa;
	struct fsm_dp_aggriob *piob;
	unsigned int plen;
#ifdef FSM_ORU_FWD_MEASURE_CYCLE
	unsigned long cycle_start;
#endif

	_fwd_rx_from_device_cnt++;
#ifdef FSM_ORU_FWD_MEASURE_CYCLE
	cycle_start = get_cycles();
#endif
	/* did we acquire target ethernet address ? */
	if (!fsm_oru_fwd_enable) {
		_fwd_drop++;
		goto err1_rel;
	}
	/*
	 * orig_buf is pointing to the receive buffer start of user data area,
	 * which has fsm_dp_msg_hdr.
	 */
	pfsm_dp_msghdr = (struct fsm_dp_msghdr *) orig_buf;
	if (length <= sizeof(*pfsm_dp_msghdr))
		goto ill_format;
	length -= sizeof(*pfsm_dp_msghdr);
	page_offset += sizeof(*pfsm_dp_msghdr);

	/*
	 * Here we are stealing first part of msg header for
	 * reference count. From now on, msg header
	 * should not be referenced.
	 */
	ref = (atomic_t *) (orig_buf);
	aggr = (pfsm_dp_msghdr->aggr != 0);
	atomic_set(ref, 0);
	if (!aggr) {
		num_pkt = 1;
		plen = length; /* packet length */
		poff = 0; /* packet offset after pass fsm_dp_msghdr */
		piob = NULL;
	} else {
		if (length <= sizeof(struct fsm_dp_aggrhdr *))
			goto ill_format;
		pa = (struct fsm_dp_aggrhdr *) (pfsm_dp_msghdr + 1);
		num_pkt = pa->n_iobs;
		piob = &pa->iob[0];
		if (length <= (sizeof(struct fsm_dp_aggrhdr *) + num_pkt *
					sizeof(struct fsm_dp_aggriob *)))
			goto ill_format;
		for (i = 0; i < num_pkt; i++)
			if ((piob[i].offset + piob[i].size) > length)
				goto ill_format;
		plen = piob->size, poff = piob->offset; /* first one */
	}
	for (i = 0; i < num_pkt; i++)
		atomic_inc(ref);

	for (i = 0; i < num_pkt; i++) {
		if (i != 0)
			piob++, plen = piob->size, poff = piob->offset;
		rc = fsm_oru_fwd_send2_net(
			page, page_offset + poff,
			orig_buf, plen);
		if (rc)
			goto err_rel;
		else
			good_sent++;
	}
#ifdef FSM_ORU_FWD_MEASURE_CYCLE
	_fwd_ul_cycles += (get_cycles() - cycle_start);
#endif
	return 0;
err_rel:
	for (i = 0; i < num_pkt - good_sent; i++)
		atomic_dec(ref);
	if (atomic_read(ref) == 0) {
		fsm_dp_rel_rx_buf(fsm_dp_rx_handle, orig_buf);
		_fwd_free_ul_buf++;
	}
	return rc;
ill_format:
	pr_err("%s: ill formatted fsm_dp msg length  %d\n",
						__func__, orig_length);
err1_rel:
	fsm_dp_rel_rx_buf(fsm_dp_rx_handle, orig_buf);
	_fwd_free_ul_buf++;
	return -EINVAL;
}

static void _fsm_oru_fwd_cleanup(void)
{
	fsm_oru_fwd_debugfs_cleanup();

	if (_oru_netdev) {
		unregister_netdev(_oru_netdev);
		free_netdev(_oru_netdev);
		_oru_netdev = NULL;
	}

#ifdef FSM_ORU_FWD_TEST
	if (fsm_dp_rx_handle)
		fsm_dp_deregister_kernel_client(fsm_dp_rx_handle,
					FSM_DP_MSG_TYPE_LPBK_RSP);
	if (fsm_dp_tx_handle)
		fsm_dp_deregister_kernel_client(fsm_dp_tx_handle,
					FSM_DP_MSG_TYPE_LPBK_REQ);
#else
	if (fsm_dp_tx_handle)
		fsm_dp_deregister_kernel_client(fsm_dp_tx_handle,
						FSM_DP_MSG_TYPE_ORU);
#endif
	fsm_dp_tx_handle = fsm_dp_rx_handle = NULL;

	_fsm_oru_fwd_disable();

	if (nl_socket_handle)
		netlink_kernel_release(nl_socket_handle);
}

static netdev_tx_t oru_netdev_start_xmit(
	struct sk_buff *skb, struct net_device *dev)
{

	struct udphdr *udphdr;
	struct iphdr *iph = (struct iphdr *) skb->data;
	uint16_t srcport;
	__be32 src_addr;
	struct neighbour *n;

#ifdef FSM_ORU_FWD_MEASURE_CYCLE
	cycle_start = get_cycles();
#endif
	/* check traffic for IP forwarding */
	if (!fsm_oru_use_ip)
		goto other_traffic;
	if (iph->version != IPVERSION || iph->ihl != FSM_IPV4_HEADER_LEN ||
					iph->protocol != IPPROTO_UDP)
		goto other_traffic;
	udphdr = (struct udphdr *) (iph + 1);

	if (skb->len <= (sizeof(struct iphdr) + sizeof(struct udphdr)))
		goto other_traffic;
	if (ntohs(udphdr->dest) != fsm_oru_my_udp_port)
		goto other_traffic;

	_fwd_from_net_cnt++;

	/* learning */
	srcport = udphdr->source;
	if (srcport != htons(fsm_oru_du_udp_port))
		fsm_oru_du_udp_port = ntohs(srcport);
	src_addr = iph->saddr;
	if (src_addr != htonl(fsm_oru_du_ip_addr)) {
		fsm_oru_du_ip_addr = ntohl(src_addr);
		fsm_oru_fwd_has_target_ip = false;
	}

	if (!fsm_oru_fwd_has_target_eth) {
		n = neigh_lookup(&arp_tbl, &src_addr, fsm_oru_forwarder_netdev);
		if (n) {
			ether_addr_copy(fsm_oru_fwd_target_eth, n->ha);
			fsm_oru_fwd_has_target_eth = true;
			neigh_release(n);
		} else
			fsm_oru_fwd_has_target_eth = false;
	}
	if (_fwd_loop_back)
		return _fwd_do_loopback(skb, fsm_oru_forwarder_netdev);
	skb_pull(skb, sizeof(*iph));
	skb_pull(skb, sizeof(*udphdr));
	if (!fsm_dp_tx_handle ||
		fsm_dp_tx_skb(fsm_dp_tx_handle, skb))
		goto tx_err;
	_fwd_tx_cnt++;
#ifdef FSM_ORU_FWD_MEASURE_CYCLE
	_fwd_dl_cycles += (get_cycles() - cycle_start);
#endif
	return NETDEV_TX_OK;
other_traffic:
	_fwd_netdev_other_cnt++;
	goto free_skb;
tx_err:
	_fwd_tx_err++;
free_skb:
	kfree_skb(skb);
	return NETDEV_TX_OK;
}


static int oru_netdev_change_mtu(struct net_device *dev, int new_mtu)
{

	if (new_mtu < 0 ||
			new_mtu > min(FSM_DP_MAX_DL_MSG_LEN,
					FSM_DP_MAX_UL_MSG_LEN))
		return -EINVAL;
	dev->mtu = new_mtu;
	return 0;
}

static int oru_netdev_ioctl(
		struct net_device *dev,
		struct ifreq *ifr,
		int cmd)
{
	int rc;

	/* to do */
	rc = EINVAL;
	return rc;
}


static const struct net_device_ops oru_netdev_ops_ip = {
	.ndo_init = 0,
	.ndo_start_xmit = oru_netdev_start_xmit,
	.ndo_do_ioctl = oru_netdev_ioctl,
	.ndo_change_mtu = oru_netdev_change_mtu,
	.ndo_set_mac_address = 0,
	.ndo_validate_addr = 0,
};


static void oru_netdev_setup(struct net_device *dev)
{
	dev->netdev_ops = &oru_netdev_ops_ip;
	ether_setup(dev);
	/* set this after calling ether_setup */
	dev->header_ops = 0;  /* No header */
	dev->type = ARPHRD_RAWIP;
	dev->mtu = min(FSM_DP_MAX_DL_MSG_LEN, FSM_DP_MAX_UL_MSG_LEN);
	dev->features = NETIF_F_HW_CSUM;
	ofwd_netdev_priv = netdev_priv(dev);
	ofwd_netdev_priv->ndev = dev;
	random_ether_addr(dev->dev_addr);
	/* Raw IP mode */
	dev->header_ops = 0;  /* No header */
	dev->flags &= ~(IFF_BROADCAST | IFF_MULTICAST);
}

static int fsm_oru_fwd_netdev_init(void)
{
	int ret;

	rtnl_lock();
	_oru_netdev = alloc_netdev(sizeof(*ofwd_netdev_priv),
		"ofwd", NET_NAME_PREDICTABLE,
		oru_netdev_setup);
	if (!_oru_netdev) {
		pr_err("%s: can not allocate oru netdev\n", __func__);
		rtnl_unlock();
		return -ENOMEM;
	}
	rtnl_unlock();
	ret = register_netdev(_oru_netdev);
	if (ret) {
		pr_err("%s: Network device registration failed\n",
							__func__);
		free_netdev(_oru_netdev);
		_oru_netdev = NULL;
		return -ENOMEM;
	}
	return 0;
}

static void _fsm_oru_fwd_exit(void)
{
	_fsm_oru_fwd_cleanup();
	pr_info("ORU Forwarder removed. oru_fwd_etype=%x\n",
							oru_fwd_etype);
}

static int _fsm_oru_fwd_init(void)
{
	int ret = 0;
	unsigned long cycle_start;

	pr_info("ORU Forwarder loaded. dev %s oru_fwd_etype=%x\n",
					fsm_oru_fwd_netdev_name, oru_fwd_etype);

	nl_socket_handle = _fsm_oru_fwd_start_netlink();
	if (!nl_socket_handle) {
		pr_err("%s: Failed to init netlink socket\n", __func__);
		return -ENOMEM;
	}

	ret =  _fsm_oru_fwd_enable();
	if (ret)
		goto out;

#ifdef FSM_ORU_FWD_TEST
	fsm_dp_tx_handle = fsm_dp_register_kernel_client(
			FSM_DP_MSG_TYPE_LPBK_REQ, fsm_dp_tx_cmplt_cb, NULL);
	if (!fsm_dp_tx_handle) {
		ret = -ENOMEM;
		goto out;
	}
	fsm_dp_rx_handle = fsm_dp_register_kernel_client(
		FSM_DP_MSG_TYPE_LPBK_RSP, NULL, fsm_oru_fwd_rx_ind_cb);
	if (!fsm_dp_rx_handle) {
		ret = -ENOMEM;
		goto out;
	}
#else
	fsm_dp_tx_handle = fsm_dp_register_kernel_client(
		FSM_DP_MSG_TYPE_ORU, fsm_dp_tx_cmplt_cb, fsm_oru_fwd_rx_ind_cb);
	if (!fsm_dp_tx_handle) {
		ret = -ENOMEM;
		goto out;
	}
	fsm_dp_rx_handle = fsm_dp_tx_handle;
#endif
	cycle_start = get_cycles();
	udelay(1000);
	_fwd_cycles_per_ms = get_cycles() - cycle_start;

	ret = fsm_oru_fwd_netdev_init();

	ret = fsm_oru_fwd_debugfs_init();
	if (!ret)
		return 0;
out:
	_fsm_oru_fwd_cleanup();
	return ret;
}


module_init(_fsm_oru_fwd_init);
module_exit(_fsm_oru_fwd_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("FSM ORU Forwarder");
