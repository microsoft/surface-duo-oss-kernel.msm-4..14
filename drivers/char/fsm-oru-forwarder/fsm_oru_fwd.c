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
#include <linux/workqueue.h>
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
#define DEFAULT_UPLANE_DL_CONCAT_MIN_DELAY (6)
#define DEFAULT_UPLANE_DL_CONCAT_MAX_DELAY (10)
#define DEFAULT_CPLANE_DL_CONCAT_MIN_DELAY (10)
#define DEFAULT_CPLANE_DL_CONCAT_MAX_DELAY (30)
#define FSM_ORU_MAX_ECPRI_PDU_SIZE (FSM_DP_MAX_DL_MSG_LEN - \
					sizeof(struct fsm_dp_msghdr))

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
static unsigned int _fwd_cycles_per_call;
static unsigned int _fwd_cycles_per_ktime_get;

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

static bool fsm_oru_dl_concat = false;

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

static struct fsm_oru_fwd_dl_concat_cfg fsm_oru_fwd_dl_concat_cfg = {
	false,
	DEFAULT_UPLANE_DL_CONCAT_MIN_DELAY, DEFAULT_UPLANE_DL_CONCAT_MAX_DELAY,
	DEFAULT_CPLANE_DL_CONCAT_MIN_DELAY, DEFAULT_CPLANE_DL_CONCAT_MAX_DELAY,
	FSM_ORU_MAX_ECPRI_PDU_SIZE
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

static inline uint8_t get_ecrpi_rev(struct ecpri_common_header *hdr)
{
	return ((hdr->rev_c & ECPRI_REV_MASK) >> ECPRI_REV_SHIFT);
}

static inline bool ecpri_cbit(struct ecpri_common_header *hdr)
{
	return (hdr->rev_c & ECRPI_C_MASK);
}

struct ofwd_netdev_priv *ofwd_netdev_priv;

#undef FSM_ORU_FWD_TEST

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

static unsigned int _fwd_dl_cycles_pkt_cnt;
static unsigned int _fwd_ul_cycles_pkt_cnt;
static u64 _fwd_dl_cycles;
static u64 _fwd_ul_cycles;

static int fwd_ul_traffic_index = -1;

static bool fwd_ul_traffic_collect_done;
static bool fwd_ul_traffic_collect;

static int fwd_dl_traffic_index = -1;

static bool fwd_dl_traffic_collect_done;
static bool fwd_dl_traffic_collect;
static bool _fwd_traffic_timestamp;

struct fwd_time_stamp {
	unsigned long arrival_cycle;
	unsigned long complete_cycle;
};

#define FWD_TRAFFIC_ARRAY_SIZE 256
static struct fwd_time_stamp fwd_ul_traffic[FWD_TRAFFIC_ARRAY_SIZE];
static struct fwd_time_stamp fwd_dl_traffic[FWD_TRAFFIC_ARRAY_SIZE];

static bool _fwd_loop_back; /* recv and loop back to net control */

static int _fsm_oru_forwarder_rcv(
	struct sk_buff *skb,
	struct net_device *ifp,
	struct packet_type *pt,
	struct net_device *orig_dev
);
static int _fsm_oru_fwd_enable(void);
static int _fsm_oru_fwd_disable(void);

static inline unsigned long fwd_get_cycles(void)
{
	if (_fwd_traffic_timestamp)
		return get_cycles();
	else
		return 0;
}

static spinlock_t fsm_oru_fwd_uplane_lock;
static spinlock_t fsm_oru_fwd_cplane_lock;

static struct hrtimer _fsm_oru_concat_uplane_hrtimer;
static struct hrtimer _fsm_oru_concat_cplane_hrtimer;

static struct workqueue_struct *_fsm_oru_wq;

static struct work_struct _fsm_oru_uplane_work;
static struct work_struct _fsm_oru_cplane_work;

#define FSM_QUEUE_MAX  FSM_DP_MAX_SG_IOV_SIZE

static uint32_t _fsm_queue_uplane_index;
static uint32_t _fsm_queue_uplane_length;
static int _fsm_oru_queue_uplane_seg;
static unsigned long _fsm_queue_uplane_1st_cycle;
static struct sk_buff *skb_uplane_queue[FSM_QUEUE_MAX];

static uint32_t _fsm_queue_cplane_index;
static uint32_t _fsm_queue_cplane_length;
static int _fsm_oru_queue_cplane_seg;
static unsigned long _fsm_queue_cplane_1st_cycle;
static struct sk_buff *skb_cplane_queue[FSM_QUEUE_MAX];

static int fsm_oru_queue_max = FSM_QUEUE_MAX;

static uint32_t fsm_oru_dl_concat_uplane_min_delay =
				DEFAULT_UPLANE_DL_CONCAT_MIN_DELAY;
static uint32_t fsm_oru_dl_concat_uplane_max_delay =
				DEFAULT_UPLANE_DL_CONCAT_MAX_DELAY;
static uint32_t fsm_oru_dl_concat_uplane_min_delay_cycle;
static uint32_t fsm_oru_dl_concat_uplane_max_delay_cycle;

static uint32_t fsm_oru_dl_concat_cplane_min_delay =
				DEFAULT_CPLANE_DL_CONCAT_MIN_DELAY;
static uint32_t fsm_oru_dl_concat_cplane_max_delay =
				DEFAULT_CPLANE_DL_CONCAT_MAX_DELAY;

static uint32_t fsm_oru_dl_max_pdu_size = FSM_ORU_MAX_ECPRI_PDU_SIZE;
static uint32_t fsm_oru_dl_concat_cplane_min_delay_cycle;
static uint32_t fsm_oru_dl_concat_cplane_max_delay_cycle;


static void fsm_queue_flush(unsigned long flags, int type);


static int debugfs_fwd_status_show(struct seq_file *s, void *unused)
{
	unsigned int avg_dl;
	unsigned int avg_ul;

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
	seq_printf(s, "System Cycles per ms  %u\n", _fwd_cycles_per_ms);
	if (_fwd_ul_cycles_pkt_cnt) {
		avg_ul = (_fwd_ul_cycles * 10000 /
				_fwd_ul_cycles_pkt_cnt) /
					_fwd_cycles_per_ms;
		seq_printf(s, "Avg UL Fwd us    %4u.%1u\n",
						avg_ul / 10, avg_ul % 10);
	}
	if (_fwd_dl_cycles_pkt_cnt) {
		avg_dl = (_fwd_dl_cycles * 10000 /
				_fwd_dl_cycles_pkt_cnt) /
					_fwd_cycles_per_ms;
		seq_printf(s, "Avg DL Fwd us    %4u.%1u\n",
						avg_dl / 10, avg_dl % 10);
	}
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

static ssize_t debugfs_fwd_traffic_ul_set(
	struct file *s,
	const char __user *buf,
	size_t count,
	loff_t *ppos
)
{
	unsigned int enable = 0;

	if (kstrtouint_from_user(buf, count, 0, &enable))
		return -EFAULT;
	if (enable && _fwd_traffic_timestamp) {
		fwd_ul_traffic_index = -1;
		fwd_ul_traffic_collect_done = false;
		fwd_ul_traffic_collect = true;
	} else {
		fwd_ul_traffic_collect = false;
	}
	return count;
}

static unsigned long fwd_gap_array[FWD_TRAFFIC_ARRAY_SIZE];
static int debugfs_fwd_traffic_ul_get(struct seq_file *s, void *unused)
{
	unsigned long max_gap = 0;
	unsigned long min_gap = 0xffffffff;
	unsigned long total_gap = 0;
	unsigned long gap;

	unsigned long max_service = 0;
	unsigned long min_service = 0xffffffff;
	unsigned long total_service = 0;
	unsigned long service;
	int i;
	unsigned int dist[5];

	seq_printf(s, "UL collect done:       %d\n\n",
					fwd_ul_traffic_collect_done);
	if (!fwd_ul_traffic_collect_done)
		return 0;

	for (i = 0; i < FWD_TRAFFIC_ARRAY_SIZE; i++) {
		service = fwd_ul_traffic[i].complete_cycle -
				fwd_ul_traffic[i].arrival_cycle;
		if (service >= max_service)
			max_service = service;
		if (service <= min_service)
			min_service = service;
		total_service += service;
	}
	seq_printf(s, "UL max serive time:  %ld us\n",
			(max_service * 1000) / _fwd_cycles_per_ms);
	seq_printf(s, "UL min service time: %ld us\n",
			(min_service * 1000) / _fwd_cycles_per_ms);
	seq_printf(s, "UL avg service time: %ld us\n",
			(total_service * 1000) / (_fwd_cycles_per_ms *
					FWD_TRAFFIC_ARRAY_SIZE));

	dist[0] = dist[1] = dist[2] = dist[3] = dist[4] = 0;
	for (i = 1; i < FWD_TRAFFIC_ARRAY_SIZE; i++) {
		gap = fwd_ul_traffic[i].arrival_cycle -
			fwd_ul_traffic[i - 1].arrival_cycle;
		fwd_gap_array[i - 1] = gap;
		if (gap >= max_gap)
			max_gap = gap;
		if (gap <= min_gap)
			min_gap = gap;
		total_gap += gap;
		if ((gap * 10) >= _fwd_cycles_per_ms)
			dist[4]++;
		else if ((gap * 200) < _fwd_cycles_per_ms)
			dist[0]++;
		else if ((gap * 100) < _fwd_cycles_per_ms)
			dist[1]++;
		else if ((gap * 50) < _fwd_cycles_per_ms)
			dist[2]++;
		else
			dist[3]++;
	}
	seq_printf(s, "UL max gap:       %ld us\n",
			(max_gap * 1000)/_fwd_cycles_per_ms);
	seq_printf(s, "UL min gap:       %ld us\n",
			(min_gap * 1000)/_fwd_cycles_per_ms);
	seq_printf(s, "UL avg gap:       %ld us\n",
			(total_gap * 1000) / (_fwd_cycles_per_ms *
					(FWD_TRAFFIC_ARRAY_SIZE - 1)));

	seq_printf(s, "UL dist: < 5 us %d , < 10 us %d, < 20 us %d, < 100 us %d, > 100 us %d\n",
			dist[0], dist[1], dist[2], dist[3], dist[4]);

	for (i = 0; i < (FWD_TRAFFIC_ARRAY_SIZE / 8) - 1; i++) {
		pr_info("gap %ld %ld %ld %ld %ld %ld %ld %ld\n",
			fwd_gap_array[i * 8], fwd_gap_array[i * 8 + 1],
			fwd_gap_array[i * 8 + 2], fwd_gap_array[i * 8 + 3],
			fwd_gap_array[i * 8 + 4], fwd_gap_array[i * 8 + 5],
			fwd_gap_array[i * 8 + 6], fwd_gap_array[i * 8 + 7]);
	}
	i = (FWD_TRAFFIC_ARRAY_SIZE / 8) - 1;
	pr_info("gap %ld %ld %ld %ld %ld %ld %ld\n",
			fwd_gap_array[i * 8], fwd_gap_array[i * 8 + 1],
			fwd_gap_array[i * 8 + 2], fwd_gap_array[i * 8 + 3],
			fwd_gap_array[i * 8 + 4], fwd_gap_array[i * 8 + 5],
			fwd_gap_array[i * 8 + 6]);
	return 0;
}
DEFINE_DEBUGFS_OPS(
	debugfs_fwd_traffic_ul,
	debugfs_fwd_traffic_ul_get,
	debugfs_fwd_traffic_ul_set);


static ssize_t debugfs_fwd_traffic_dl_set(
	struct file *s,
	const char __user *buf,
	size_t count,
	loff_t *ppos
)
{
	unsigned int enable = 0;

	if (kstrtouint_from_user(buf, count, 0, &enable))
		return -EFAULT;
	if (enable && _fwd_traffic_timestamp) {
		fwd_dl_traffic_index = -1;
		fwd_dl_traffic_collect_done = false;
		fwd_dl_traffic_collect = true;
	} else {
		fwd_dl_traffic_collect = false;
	}
	return count;
}

static int debugfs_fwd_traffic_dl_get(struct seq_file *s, void *unused)
{
	unsigned long max_gap = 0;
	unsigned long min_gap = 0xffffffff;
	unsigned long total_gap = 0;
	unsigned long gap;

	unsigned long max_service = 0;
	unsigned long min_service = 0xffffffff;
	unsigned long total_service = 0;
	unsigned long service;
	int i;
	unsigned int dist[5];

	seq_printf(s, "DL collect done:       %d\n\n",
					fwd_dl_traffic_collect_done);
	if (!fwd_dl_traffic_collect_done)
		return 0;

	for (i = 0; i < FWD_TRAFFIC_ARRAY_SIZE; i++) {
		service = fwd_dl_traffic[i].complete_cycle -
				fwd_dl_traffic[i].arrival_cycle;
		if (service >= max_service)
			max_service = service;
		if (service <= min_service)
			min_service = service;
		total_service += service;
	}
	seq_printf(s, "DL max serive time:  %ld us\n",
			(max_service * 1000) / _fwd_cycles_per_ms);
	seq_printf(s, "DL min service time: %ld us\n",
			(min_service * 1000) / _fwd_cycles_per_ms);
	seq_printf(s, "DL avg service time: %ld us\n",
			(total_service * 1000) / (_fwd_cycles_per_ms *
					FWD_TRAFFIC_ARRAY_SIZE));

	dist[0] = dist[1] = dist[2] = dist[3] = dist[4] = 0;
	for (i = 1; i < FWD_TRAFFIC_ARRAY_SIZE; i++) {
		gap = fwd_dl_traffic[i].arrival_cycle -
			fwd_dl_traffic[i - 1].arrival_cycle;
		fwd_gap_array[i - 1] = gap;
		if (gap >= max_gap)
			max_gap = gap;
		if (gap <= min_gap)
			min_gap = gap;
		total_gap += gap;
		if ((gap * 10) >= _fwd_cycles_per_ms)
			dist[4]++;
		else if ((gap * 200) < _fwd_cycles_per_ms)
			dist[0]++;
		else if ((gap * 100) < _fwd_cycles_per_ms)
			dist[1]++;
		else if ((gap * 50) < _fwd_cycles_per_ms)
			dist[2]++;
		else
			dist[3]++;
	}
	seq_printf(s, "DL max gap:       %ld us\n",
			(max_gap * 1000)/_fwd_cycles_per_ms);
	seq_printf(s, "DL min gap:       %ld us\n",
			(min_gap * 1000)/_fwd_cycles_per_ms);
	seq_printf(s, "DL avg gap:       %ld us\n",
			(total_gap * 1000) / (_fwd_cycles_per_ms *
					(FWD_TRAFFIC_ARRAY_SIZE - 1)));

	seq_printf(s, "DL dist: < 5 us %d , < 10 us %d, < 20 us %d, < 100 us %d, > 100 us %d\n",
			dist[0], dist[1], dist[2], dist[3], dist[4]);

	for (i = 0; i < (FWD_TRAFFIC_ARRAY_SIZE / 8) - 1; i++) {
		pr_info("DL gap %ld %ld %ld %ld %ld %ld %ld %ld\n",
			fwd_gap_array[i * 8], fwd_gap_array[i * 8 + 1],
			fwd_gap_array[i * 8 + 2], fwd_gap_array[i * 8 + 3],
			fwd_gap_array[i * 8 + 4], fwd_gap_array[i * 8 + 5],
			fwd_gap_array[i * 8 + 6], fwd_gap_array[i * 8 + 7]);
	}
	i = (FWD_TRAFFIC_ARRAY_SIZE / 8) - 1;
	pr_info("DL gap %ld %ld %ld %ld %ld %ld %ld\n",
			fwd_gap_array[i * 8], fwd_gap_array[i * 8 + 1],
			fwd_gap_array[i * 8 + 2], fwd_gap_array[i * 8 + 3],
			fwd_gap_array[i * 8 + 4], fwd_gap_array[i * 8 + 5],
			fwd_gap_array[i * 8 + 6]);
	return 0;
}
DEFINE_DEBUGFS_OPS(
	debugfs_fwd_traffic_dl,
	debugfs_fwd_traffic_dl_get,
	debugfs_fwd_traffic_dl_set);


static int debugfs_fwd_traffic_timestamp_get(struct seq_file *s, void *unused)
{

	seq_printf(s, "FWD traffic timtstamp:       %d\n",
					_fwd_traffic_timestamp);
	return 0;
}

static ssize_t debugfs_fwd_traffic_timestamp_set(
	struct file *s,
	const char __user *buf,
	size_t count,
	loff_t *ppos
)
{
	unsigned int enable = 0;

	if (kstrtouint_from_user(buf, count, 0, &enable))
		return -EFAULT;
	_fwd_traffic_timestamp = enable;
	if (enable) {
		_fwd_dl_cycles = 0;
		_fwd_ul_cycles = 0;
		_fwd_ul_cycles_pkt_cnt = 0;
		_fwd_dl_cycles_pkt_cnt = 0;
	}
	return count;
}
DEFINE_DEBUGFS_OPS(
	debugfs_fwd_traffic_timestamp,
	debugfs_fwd_traffic_timestamp_get,
	debugfs_fwd_traffic_timestamp_set);

static int debugfs_create_traffic_dir(struct dentry *parent)
{
	struct dentry *entry = NULL, *dentry = NULL;

	dentry = debugfs_create_dir("traffic", parent);
	if (IS_ERR(dentry))
		return -ENOMEM;
	entry = debugfs_create_file("fwd_ul", 0444, dentry, NULL,
		&debugfs_fwd_traffic_ul_ops);
	if (!entry)
		return -ENOMEM;
	entry = debugfs_create_file("fwd_dl", 0444, dentry, NULL,
		&debugfs_fwd_traffic_dl_ops);
	if (!entry)
		return -ENOMEM;
	entry = debugfs_create_file("time_stamp", 0444, dentry, NULL,
		&debugfs_fwd_traffic_timestamp_ops);
	if (!entry)
		return -ENOMEM;
	return 0;
}

static int debugfs_fwd_concat_get(struct seq_file *s, void *unused)
{
	seq_printf(s, "max number of concatenated messages: %d\n",
					fsm_oru_queue_max);
	return 0;
}

static ssize_t debugfs_fwd_concat_set(
	struct file *s,
	const char __user *buf,
	size_t count,
	loff_t *ppos
)
{
	unsigned int concat = 0;

	if (kstrtouint_from_user(buf, count, 0, &concat))
		return -EFAULT;
	if (concat > FSM_QUEUE_MAX)
		concat = FSM_QUEUE_MAX;
	fsm_oru_queue_max = concat;
	return count;
}
DEFINE_DEBUGFS_OPS(
	debugfs_fwd_concat,
	debugfs_fwd_concat_get,
	debugfs_fwd_concat_set
);

static int debugfs_fwd_dlmax_pdu_get(struct seq_file *s, void *unused)
{
	seq_printf(s, "max dl concatenated msg size:  %d\n",
					fsm_oru_dl_max_pdu_size);
	return 0;
}

static ssize_t debugfs_fwd_dlmax_pdu_set(
	struct file *s,
	const char __user *buf,
	size_t count,
	loff_t *ppos
)
{
	unsigned int concat = 0;

	if (kstrtouint_from_user(buf, count, 0, &concat))
		return -EFAULT;
	if (concat > FSM_ORU_MAX_ECPRI_PDU_SIZE)
		concat = FSM_ORU_MAX_ECPRI_PDU_SIZE;
	fsm_oru_dl_max_pdu_size = concat;
	return count;
}
DEFINE_DEBUGFS_OPS(
	debugfs_fwd_dlmax_pdu,
	debugfs_fwd_dlmax_pdu_get,
	debugfs_fwd_dlmax_pdu_set
);
static int debugfs_fwd_concat_max_get(struct seq_file *s, void *unused)
{

	seq_printf(s, "max concatenate timer:       %d us\n",
				fsm_oru_dl_concat_uplane_max_delay);
	seq_printf(s, "max concatenate cycle:       %d cycle\n",
				fsm_oru_dl_concat_uplane_max_delay_cycle);
	return 0;
}

static ssize_t debugfs_fwd_concat_max_set(
	struct file *s,
	const char __user *buf,
	size_t count,
	loff_t *ppos
)
{
	unsigned int concat = 0;

	if (kstrtouint_from_user(buf, count, 0, &concat))
		return -EFAULT;
	fsm_oru_dl_concat_uplane_max_delay = concat;
	fsm_oru_dl_concat_uplane_max_delay_cycle =
		fsm_oru_dl_concat_uplane_max_delay * _fwd_cycles_per_ms / 1000;
	return count;
}
DEFINE_DEBUGFS_OPS(
	debugfs_fwd_concat_max,
	debugfs_fwd_concat_max_get,
	debugfs_fwd_concat_max_set
);

static int debugfs_fwd_concat_min_get(struct seq_file *s, void *unused)
{

	seq_printf(s, "u plane min concatenate timer:    %d us\n",
				fsm_oru_dl_concat_uplane_min_delay);
	seq_printf(s, "u plane min concatenate cycle:    %d cycle\n",
				fsm_oru_dl_concat_uplane_min_delay_cycle);
	return 0;
}

static ssize_t debugfs_fwd_concat_min_set(
	struct file *s,
	const char __user *buf,
	size_t count,
	loff_t *ppos
)
{
	unsigned int concat = 0;

	if (kstrtouint_from_user(buf, count, 0, &concat))
		return -EFAULT;
	fsm_oru_dl_concat_uplane_min_delay = concat;
	fsm_oru_dl_concat_uplane_min_delay_cycle =
		fsm_oru_dl_concat_uplane_min_delay * _fwd_cycles_per_ms / 1000;
	return count;
}
DEFINE_DEBUGFS_OPS(
	debugfs_fwd_concat_min,
	debugfs_fwd_concat_min_get,
	debugfs_fwd_concat_min_set
);

static int debugfs_fwd_concat_cplane_max_get(struct seq_file *s, void *unused)
{

	seq_printf(s, "c plane max concatenate timer:       %d us\n",
				fsm_oru_dl_concat_cplane_max_delay);
	seq_printf(s, "c plane max concatenate cycle:       %d cycle\n",
				fsm_oru_dl_concat_cplane_max_delay_cycle);
	return 0;
}

static ssize_t debugfs_fwd_concat_cplane_max_set(
	struct file *s,
	const char __user *buf,
	size_t count,
	loff_t *ppos
)
{

	unsigned int concat = 0;

	if (kstrtouint_from_user(buf, count, 0, &concat))
		return -EFAULT;
	fsm_oru_dl_concat_cplane_max_delay = concat;
	fsm_oru_dl_concat_cplane_max_delay_cycle =
		fsm_oru_dl_concat_cplane_max_delay *
			_fwd_cycles_per_ms / 1000;
	return count;
}
DEFINE_DEBUGFS_OPS(
	debugfs_fwd_concat_cplane_max,
	debugfs_fwd_concat_cplane_max_get,
	debugfs_fwd_concat_cplane_max_set
);

static int debugfs_fwd_concat_cplane_min_get(struct seq_file *s, void *unused)
{
	seq_printf(s, "c plane min concatenate timer:    %d us\n",
				fsm_oru_dl_concat_cplane_min_delay);
	seq_printf(s, "c plane min concatenate cycle:    %d cycle\n",
				fsm_oru_dl_concat_cplane_min_delay_cycle);
	return 0;
}

static ssize_t debugfs_fwd_concat_cplane_min_set(
	struct file *s,
	const char __user *buf,
	size_t count,
	loff_t *ppos
)
{
	unsigned int concat = 0;

	if (kstrtouint_from_user(buf, count, 0, &concat))
		return -EFAULT;
	fsm_oru_dl_concat_cplane_min_delay = concat;
	fsm_oru_dl_concat_cplane_min_delay_cycle =
		fsm_oru_dl_concat_cplane_min_delay *
			_fwd_cycles_per_ms / 1000;
	return count;
}
DEFINE_DEBUGFS_OPS(
	debugfs_fwd_concat_cplane_min,
	debugfs_fwd_concat_cplane_min_get,
	debugfs_fwd_concat_cplane_min_set
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
	if (debugfs_create_traffic_dir(__dent))
		goto err;
	entry = debugfs_create_file("concat-max-msg", 0444, __dent, NULL,
		&debugfs_fwd_concat_ops);
	if (!entry)
		goto err;
	entry = debugfs_create_file("concat-dl-max-pdu-size", 0444,
		__dent, NULL, &debugfs_fwd_dlmax_pdu_ops);
	if (!entry)
		goto err;
	entry = debugfs_create_file("concat-uplane-min-delay", 0444,
		__dent, NULL, &debugfs_fwd_concat_min_ops);
	if (!entry)
		goto err;
	entry = debugfs_create_file("concat-uplane-max-delay", 0444,
		__dent, NULL, &debugfs_fwd_concat_max_ops);
	if (!entry)
		goto err;

	entry = debugfs_create_file("concat-cplane-min-delay", 0444,
		__dent, NULL, &debugfs_fwd_concat_cplane_min_ops);
	if (!entry)
		goto err;
	entry = debugfs_create_file("concat-cplane-max-delay", 0444,
		__dent, NULL, &debugfs_fwd_concat_cplane_max_ops);
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
		pr_warn("%s: can not get device %s\n", __func__,
					fsm_oru_fwd_netdev_name);
		return 0;
	}

	_fsm_queue_uplane_index = 0;
	_fsm_queue_uplane_length = 0;
	_fsm_queue_uplane_1st_cycle = 0;
	_fsm_oru_queue_uplane_seg = 0;
	memset(skb_uplane_queue, 0, sizeof(skb_uplane_queue));

	_fsm_queue_cplane_index = 0;
	_fsm_queue_cplane_length = 0;
	_fsm_queue_cplane_1st_cycle = 0;
	_fsm_oru_queue_cplane_seg = 0;
	memset(skb_cplane_queue, 0, sizeof(skb_cplane_queue));

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
	unsigned long flags;

	if (!fsm_oru_fwd_enable)
		return 0;

	spin_lock_irqsave(&fsm_oru_fwd_uplane_lock, flags);
	fsm_queue_flush(flags, FSM_ORU_MSG_TYPE_UPLANE);

	spin_lock_irqsave(&fsm_oru_fwd_cplane_lock, flags);
	fsm_queue_flush(flags, FSM_ORU_MSG_TYPE_CPLANE);

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

static void fsm_oru_fwd_nl_get_dl_concat_cfg(
	struct fsm_oru_fwd_nl_msg_s *fwd_header,
	struct fsm_oru_fwd_nl_msg_s *resp_fwd
)
{
	unsigned char size =
		sizeof(((struct fsm_oru_fwd_nl_msg_s *)0)->fwd_config);
	resp_fwd->crd = FSM_ORU_FWD_NETLINK_MSG_RETURNDATA;
	resp_fwd->arg_length = size;
	resp_fwd->fwd_dl_concat_config = fsm_oru_fwd_dl_concat_cfg;
}

static void fsm_oru_fwd_nl_set_dl_concat_cfg(
	struct fsm_oru_fwd_nl_msg_s *fwd_header,
	struct fsm_oru_fwd_nl_msg_s *resp_fwd
)
{

	resp_fwd->crd = FSM_ORU_FWD_NETLINK_MSG_RETURNCODE;
	fsm_oru_fwd_dl_concat_cfg = fwd_header->fwd_dl_concat_config;
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

		fsm_oru_dl_concat = fsm_oru_fwd_dl_concat_cfg.dl_concat;
		fsm_oru_dl_concat_uplane_min_delay =
				fsm_oru_fwd_dl_concat_cfg.dl_concat_uplane_min_delay;
		fsm_oru_dl_concat_uplane_max_delay =
				fsm_oru_fwd_dl_concat_cfg.dl_concat_uplane_max_delay;
		fsm_oru_dl_concat_uplane_min_delay_cycle = _fwd_cycles_per_ms *
				fsm_oru_dl_concat_uplane_min_delay / 1000;
		fsm_oru_dl_concat_uplane_max_delay_cycle =  _fwd_cycles_per_ms *
				fsm_oru_dl_concat_uplane_max_delay / 1000;

		fsm_oru_dl_concat_cplane_min_delay =
				fsm_oru_fwd_dl_concat_cfg.dl_concat_cplane_min_delay;
		fsm_oru_dl_concat_cplane_max_delay =
				fsm_oru_fwd_dl_concat_cfg.dl_concat_cplane_max_delay;
		fsm_oru_dl_concat_cplane_min_delay_cycle = _fwd_cycles_per_ms *
				fsm_oru_dl_concat_cplane_min_delay / 1000;
		fsm_oru_dl_concat_cplane_max_delay_cycle =  _fwd_cycles_per_ms *
				fsm_oru_dl_concat_cplane_max_delay / 1000;
		if (fsm_oru_fwd_dl_concat_cfg.dl_max_pdu_size > FSM_ORU_MAX_ECPRI_PDU_SIZE)
			fsm_oru_dl_max_pdu_size = FSM_ORU_MAX_ECPRI_PDU_SIZE;
		else
			fsm_oru_dl_max_pdu_size = fsm_oru_fwd_dl_concat_cfg.dl_max_pdu_size;

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

	resp_fwd->fwd_op_status.use_ip = fsm_oru_use_ip;
	resp_fwd->fwd_op_status.du_udp_port = fsm_oru_du_udp_port;
	resp_fwd->fwd_op_status.my_udp_port = fsm_oru_my_udp_port;
	resp_fwd->fwd_op_status.du_ip_addr = fsm_oru_du_ip_addr;
	resp_fwd->fwd_op_status.my_ip_addr = fsm_oru_my_ip_addr;
	resp_fwd->fwd_op_status.my_ip_addr = fsm_oru_my_ip_addr;
	resp_fwd->fwd_op_status.dl_concat_uplane_min_delay =
					fsm_oru_dl_concat_uplane_min_delay;
	resp_fwd->fwd_op_status.dl_concat_uplane_max_delay =
					fsm_oru_dl_concat_uplane_max_delay;
	resp_fwd->fwd_op_status.dl_concat_cplane_min_delay =
					fsm_oru_dl_concat_cplane_min_delay;
	resp_fwd->fwd_op_status.dl_concat_cplane_max_delay =
					fsm_oru_dl_concat_cplane_max_delay;
	resp_fwd->fwd_op_status.dl_max_pdu_size =
					fsm_oru_dl_max_pdu_size;
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

	case FSM_ORU_FWD_NETLINK_SET_CONCAT_CONFIG:
		fsm_oru_fwd_nl_set_dl_concat_cfg(fwd_header, resp_fwd);
		break;

	case FSM_ORU_FWD_NETLINK_GET_CONCAT_CONFIG:
		fsm_oru_fwd_nl_get_dl_concat_cfg(fwd_header, resp_fwd);
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


/*
 * fsm_queue_flush: flush queued skbs
 * Before the call, spinlock is locked.
 * The spinlock will be unlocked when this function is done
 * type: 0 Uplane, 1 Cplane.
 */
static void fsm_queue_flush(unsigned long flags, int type)
{
	struct sk_buff *skb_queue_sav[FSM_QUEUE_MAX];
	struct sk_buff *tskb, *pskb, *fskb;
	int rc;
	struct ecpri_common_header *ecpri_hdr;
	unsigned int i;
	unsigned int num_queue_entries;
	unsigned long cycle_start;
	struct sk_buff **pskbq;

	if (type == FSM_ORU_MSG_TYPE_UPLANE) {
		if (_fsm_queue_uplane_index == 0) {
			spin_unlock_irqrestore(&fsm_oru_fwd_uplane_lock,
						flags);
			return;
		}
		num_queue_entries = _fsm_queue_uplane_index;
		fskb = pskb = skb_uplane_queue[0];
	} else {
		if (_fsm_queue_cplane_index == 0) {
			spin_unlock_irqrestore(&fsm_oru_fwd_cplane_lock,
						flags);
			return;
		}
		num_queue_entries = _fsm_queue_cplane_index;
		fskb = pskb = skb_cplane_queue[0];
	}

	cycle_start = fwd_get_cycles();

	pskbq = (type == FSM_ORU_MSG_TYPE_CPLANE) ?
			&skb_cplane_queue[0] : &skb_uplane_queue[0];
	for (i = 0; i < num_queue_entries; i++) {

		tskb = *pskbq++;
		skb_queue_sav[i] = tskb;
		tskb->next = NULL;
		skb_shinfo(tskb)->frag_list = NULL;
		ecpri_hdr = (struct ecpri_common_header *) tskb->data;
		if (i != num_queue_entries - 1)
			ecpri_hdr->rev_c |= ECRPI_C_MASK;
		if (i != 0) {
			if (i == 1)
				skb_shinfo(fskb)->frag_list = tskb;
			else
				pskb->next = tskb;
		}
		pskb = tskb;
	}
	if (type == FSM_ORU_MSG_TYPE_UPLANE) {
		_fsm_queue_uplane_index = 0;
		_fsm_queue_uplane_length = 0;
		_fsm_oru_queue_uplane_seg = 0;
		hrtimer_cancel(&_fsm_oru_concat_uplane_hrtimer);
		spin_unlock_irqrestore(&fsm_oru_fwd_uplane_lock, flags);
	} else {
		_fsm_queue_cplane_index = 0;
		_fsm_queue_cplane_length = 0;
		_fsm_oru_queue_cplane_seg = 0;
		hrtimer_cancel(&_fsm_oru_concat_cplane_hrtimer);
		spin_unlock_irqrestore(&fsm_oru_fwd_cplane_lock, flags);
	}

	fskb->tstamp = cycle_start;
	rc = fsm_dp_tx_skb(fsm_dp_tx_handle, fskb);
	_fwd_tx_cnt++;
	_fwd_dl_cycles += (fwd_get_cycles() - cycle_start);
	_fwd_dl_cycles_pkt_cnt++;

	if (!rc)
		return;
	if (rc) {
		for (i = 0; i < num_queue_entries; i++)
			kfree_skb(skb_queue_sav[i]);
		_fwd_tx_err++;
	}
}

static void _fsm_oru_timeout_work(struct work_struct *work)
{
	unsigned long flags;
	int type;

	type = (work == &_fsm_oru_cplane_work) ?
		FSM_ORU_MSG_TYPE_CPLANE : FSM_ORU_MSG_TYPE_UPLANE;

	if (type == FSM_ORU_MSG_TYPE_CPLANE)
		spin_lock_irqsave(&fsm_oru_fwd_cplane_lock, flags);
	else
		spin_lock_irqsave(&fsm_oru_fwd_uplane_lock, flags);
	fsm_queue_flush(flags, type);
}

static int _fsm_queue_tx_skb(struct sk_buff *skb)
{
	struct ecpri_common_header *ecpri_hdr;
	unsigned long flags;
	unsigned long pkt_cycle;
	int type;
	unsigned int ecpri_plen;
	unsigned int ecpri_mlen;

	if (!fsm_oru_dl_concat) {
		_fwd_tx_cnt++;
		return fsm_dp_tx_skb(fsm_dp_tx_handle, skb);
	}
	ecpri_hdr = (struct ecpri_common_header *) skb->data;
	type = ((ecpri_hdr->msg_type != ECPRI_MSG_IQ_DATA)
			&& (ecpri_hdr->msg_type != ECPRI_MSG_BIT_SEQ));


	/*
	 * if ecpri msg already has been ccncatenated, flush the queue and
	 * msg.
	 */
	if (ecpri_cbit(ecpri_hdr)) {
		if (type == FSM_ORU_MSG_TYPE_CPLANE)
			spin_lock_irqsave(&fsm_oru_fwd_cplane_lock, flags);
		else
			spin_lock_irqsave(&fsm_oru_fwd_uplane_lock, flags);
		fsm_queue_flush(flags, type);
		_fwd_tx_cnt++;
		return fsm_dp_tx_skb(fsm_dp_tx_handle, skb);
	}

	ecpri_mlen = ntohs(ecpri_hdr->payload_size);
	ecpri_plen =  ecpri_mlen + sizeof(*ecpri_hdr);
	if (skb->len < ecpri_plen) {
		pr_err("%s: ill formatted ecpri msg length %d, packet length %d\n",
				__func__, ecpri_plen, skb->len);
		return -1;
	}

	/* remove ethernet padding */
	if (!skb_is_nonlinear(skb) && skb->len != ecpri_plen)
		skb_trim(skb, ecpri_plen);

	/* do padding if necessary */
	if (ecpri_mlen & 3) {
		unsigned int pad_len;

		pad_len = 4 - (ecpri_mlen & 3);
		if (skb_is_nonlinear(skb)) {
			struct skb_shared_info *sinfo;
			skb_frag_t *frag;

			sinfo = skb_shinfo(skb);
			frag = &sinfo->frags[sinfo->nr_frags - 1];
			skb_frag_size_add(frag, pad_len);
			skb->len += pad_len;
			skb->data_len += pad_len;
		} else {
			__skb_put(skb, pad_len);
		}
	}

	pkt_cycle = get_cycles();

	if (type == FSM_ORU_MSG_TYPE_CPLANE) {
		spin_lock_irqsave(&fsm_oru_fwd_cplane_lock, flags);
		if (((_fsm_oru_queue_cplane_seg +
			skb_shinfo(skb)->nr_frags + 1) >
				FSM_DP_MAX_SG_IOV_SIZE) ||
			((_fsm_queue_cplane_length + skb->len)
						> fsm_oru_dl_max_pdu_size)) {
			fsm_queue_flush(flags, type);
			spin_lock_irqsave(&fsm_oru_fwd_cplane_lock, flags);
		}
		skb_cplane_queue[_fsm_queue_cplane_index] = skb;
		_fsm_queue_cplane_index++;
		_fsm_queue_cplane_length += skb->len;
		_fsm_oru_queue_cplane_seg += skb_shinfo(skb)->nr_frags + 1;
		if (_fsm_queue_cplane_index == 1) {
			_fsm_queue_cplane_1st_cycle = pkt_cycle;
			hrtimer_start(&_fsm_oru_concat_cplane_hrtimer,
				ns_to_ktime(fsm_oru_dl_concat_cplane_max_delay *
									1000),
				HRTIMER_MODE_REL_PINNED);
		} else if (_fsm_queue_cplane_index >= fsm_oru_queue_max ||
			(pkt_cycle - _fsm_queue_cplane_1st_cycle) >=
				fsm_oru_dl_concat_cplane_min_delay_cycle) {
			fsm_queue_flush(flags, type);
			return 0;
		}
		spin_unlock_irqrestore(&fsm_oru_fwd_cplane_lock, flags);
	} else {
		spin_lock_irqsave(&fsm_oru_fwd_uplane_lock, flags);
		if (((_fsm_oru_queue_uplane_seg +
				skb_shinfo(skb)->nr_frags + 1) >
					FSM_DP_MAX_SG_IOV_SIZE) ||
			((_fsm_queue_uplane_length + skb->len)
						> fsm_oru_dl_max_pdu_size)) {
			fsm_queue_flush(flags, type);
			spin_lock_irqsave(&fsm_oru_fwd_uplane_lock, flags);
		}
		skb_uplane_queue[_fsm_queue_uplane_index] = skb;
		_fsm_queue_uplane_index++;
		_fsm_queue_uplane_length += skb->len;
		_fsm_oru_queue_uplane_seg += skb_shinfo(skb)->nr_frags + 1;
		if (_fsm_queue_uplane_index == 1) {
			_fsm_queue_uplane_1st_cycle = pkt_cycle;
			hrtimer_start(&_fsm_oru_concat_uplane_hrtimer,
				ns_to_ktime(fsm_oru_dl_concat_uplane_max_delay *
									1000),
				HRTIMER_MODE_REL_PINNED);
		} else if (_fsm_queue_uplane_index >= fsm_oru_queue_max ||
			(pkt_cycle - _fsm_queue_uplane_1st_cycle) >=
				fsm_oru_dl_concat_uplane_min_delay_cycle) {
			fsm_queue_flush(flags, type);
			return 0;
		}
		spin_unlock_irqrestore(&fsm_oru_fwd_uplane_lock, flags);
	}
	return 0;
}

static int _fsm_oru_forwarder_rcv(
	struct sk_buff *skb,
	struct net_device *ifp,
	struct packet_type *pt,
	struct net_device *orig_dev
)
{
	struct ethhdr *hdr;

	if (!skb) {
		_fwd_drop++;
		return NET_RX_DROP;
	}

	_fwd_from_net_cnt++;


	hdr = (struct ethhdr *)skb_mac_header(skb);

	if (!fsm_oru_fwd_has_target_eth) {
		fsm_oru_fwd_has_target_eth = true;
		memcpy(fsm_oru_fwd_target_eth, hdr->h_source, ETH_ALEN);
	}

	if (_fwd_loop_back)
		return _fwd_do_loopback(skb, ifp);

	if (!fsm_dp_tx_handle || _fsm_queue_tx_skb(skb)) {
		kfree_skb(skb);
		_fwd_tx_err++;
		return NET_RX_DROP;
	}
	return  NET_RX_SUCCESS;
}

static inline void _fwd_dl_traffic_collect(struct sk_buff *skb)
{
	struct fwd_time_stamp *pts;

	if (fwd_dl_traffic_collect) {
		if (fwd_dl_traffic_index >= 0) {
			pts = &fwd_dl_traffic[fwd_dl_traffic_index];
			pts->arrival_cycle = skb->tstamp;
			pts->complete_cycle = fwd_get_cycles();
		}
		if (++fwd_dl_traffic_index >= FWD_TRAFFIC_ARRAY_SIZE) {
			fwd_dl_traffic_collect = false;
			fwd_dl_traffic_collect_done = true;
		}
	}
}

static inline void _fwd_ul_traffic_collect(struct sk_buff *skb)
{
	struct fwd_time_stamp *pts;

	if (fwd_ul_traffic_collect) {
		if (fwd_ul_traffic_index >= 0) {
			pts = &fwd_ul_traffic[fwd_ul_traffic_index];
			pts->arrival_cycle = skb->tstamp;
			pts->complete_cycle = fwd_get_cycles();
		}
		if (++fwd_ul_traffic_index >= FWD_TRAFFIC_ARRAY_SIZE) {
			fwd_ul_traffic_collect = false;
			fwd_ul_traffic_collect_done = true;
		}
	}
}

static void _fsm_oru_forwarder_free_skb_list(struct sk_buff *skb)
{
	struct sk_buff *fskb;

	fskb = skb_shinfo(skb)->frag_list;
	if (fskb)
		kfree_skb_list(fskb);
	skb_shinfo(skb)->frag_list = NULL;
	kfree_skb(skb);
}

int fsm_dp_tx_cmplt_cb(struct sk_buff *skb)
{

	_fwd_dl_traffic_collect(skb);
	_fsm_oru_forwarder_free_skb_list(skb);
	return 0;
}

void fsm_oru_forwarder_skb_free(struct sk_buff *skb)
{
	char *buf;

	/* buf is now pointing to the receive buffer start of user data area */
	buf = (char *) skb_uarg(skb);
	_fwd_ul_traffic_collect(skb);
	if (atomic_dec_and_test((atomic_t *)buf)) {
		fsm_dp_rel_rx_buf(fsm_dp_rx_handle, buf);
		_fwd_free_ul_buf++;
	}
}

static int fsm_oru_fwd_send2_net(
	struct page *page,
	unsigned int page_offset,
	char *orig_buf,
	unsigned int length,
	unsigned long cycle_start
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
	skb->tstamp = cycle_start;

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
	unsigned long cycle_start;

	_fwd_rx_from_device_cnt++;
	cycle_start = fwd_get_cycles();
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
			orig_buf, plen, cycle_start);
		if (rc)
			goto err_rel;
		else
			good_sent++;
	}
	_fwd_ul_cycles += (fwd_get_cycles() - cycle_start);
	_fwd_ul_cycles_pkt_cnt++;
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
	hrtimer_cancel(&_fsm_oru_concat_uplane_hrtimer);
	hrtimer_cancel(&_fsm_oru_concat_cplane_hrtimer);

	if (_fsm_oru_wq)
		destroy_workqueue(_fsm_oru_wq);
	_fsm_oru_wq = NULL;
}

static netdev_tx_t oru_netdev_start_xmit(
	struct sk_buff *skb, struct net_device *dev)
{

	struct udphdr *udphdr;
	struct iphdr *iph = (struct iphdr *) skb->data;
	uint16_t srcport;
	__be32 src_addr;
	struct neighbour *n;
	unsigned long cycle_start;

	cycle_start = fwd_get_cycles();
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
	if (!fsm_dp_tx_handle || _fsm_queue_tx_skb(skb))
		goto tx_err;
	_fwd_tx_cnt++;
	_fwd_dl_cycles += (fwd_get_cycles() - cycle_start);
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

static void _fsm_oru_fwd_init_time_measurement(void)
{
	unsigned long cycle_start;
	ktime_t ktime_start;
	ktime_t ns_per_get_cycles;
	ktime_t ns_per_ktime;

	cycle_start = get_cycles();
	_fwd_cycles_per_call =  get_cycles() - cycle_start;

	cycle_start = get_cycles();
	ktime_get();
	_fwd_cycles_per_ktime_get =  get_cycles() - cycle_start;

	cycle_start = get_cycles();
	udelay(1000);
	_fwd_cycles_per_ms = get_cycles() - cycle_start;
	pr_info("cycle per ms %d, per get_cycles call %d, per ktime_get %d\n",
		_fwd_cycles_per_ms, _fwd_cycles_per_call,
		_fwd_cycles_per_ktime_get);

	ktime_start = ktime_get();
	get_cycles();
	ns_per_get_cycles = ktime_get() - ktime_start;

	ktime_start = ktime_get();
	ktime_get();
	ns_per_ktime = ktime_get() - ktime_start;

	pr_info("ns per ktime_get %lld, per get_cycles %lld\n",
		ns_per_ktime, ns_per_get_cycles);

	fsm_oru_dl_concat_uplane_min_delay_cycle = _fwd_cycles_per_ms *
				fsm_oru_dl_concat_uplane_min_delay / 1000;
	fsm_oru_dl_concat_uplane_max_delay_cycle =  _fwd_cycles_per_ms *
				fsm_oru_dl_concat_uplane_max_delay / 1000;
	fsm_oru_dl_concat_cplane_min_delay_cycle =
		fsm_oru_dl_concat_cplane_min_delay *
			_fwd_cycles_per_ms / 1000;
	fsm_oru_dl_concat_cplane_max_delay_cycle =  _fwd_cycles_per_ms *
				fsm_oru_dl_concat_cplane_max_delay / 1000;
}

static void _fsm_oru_fwd_exit(void)
{
	_fsm_oru_fwd_cleanup();
	pr_info("ORU Forwarder removed. oru_fwd_etype=%x\n",
							oru_fwd_etype);
}

static enum hrtimer_restart _fsm_oru_fwd_concat_timer_handler(
		struct hrtimer *me)
{

	queue_work(_fsm_oru_wq, &_fsm_oru_uplane_work);
	return HRTIMER_NORESTART;
}

static int _fsm_oru_fwd_init(void)
{
	int ret = 0;
	struct workqueue_struct *wq;

	pr_info("ORU Forwarder loaded. dev %s oru_fwd_etype=%x\n",
					fsm_oru_fwd_netdev_name, oru_fwd_etype);


	nl_socket_handle = _fsm_oru_fwd_start_netlink();
	if (!nl_socket_handle) {
		pr_err("%s: Failed to init netlink socket\n", __func__);
		return -ENOMEM;
	}

	spin_lock_init(&fsm_oru_fwd_uplane_lock);
	hrtimer_init(&_fsm_oru_concat_uplane_hrtimer, CLOCK_MONOTONIC,
			HRTIMER_MODE_REL);
	hrtimer_init(&_fsm_oru_concat_cplane_hrtimer, CLOCK_MONOTONIC,
			HRTIMER_MODE_REL);
	_fsm_oru_concat_uplane_hrtimer.function =
				_fsm_oru_fwd_concat_timer_handler;
	_fsm_oru_concat_cplane_hrtimer.function =
				_fsm_oru_fwd_concat_timer_handler;
	wq = alloc_workqueue("fsm_oru_concat", WQ_MEM_RECLAIM, 0);
	if (!wq) {
		ret = -ENOMEM;
		goto out;
	}
	_fsm_oru_wq = wq;
	INIT_WORK(&_fsm_oru_uplane_work, _fsm_oru_timeout_work);
	INIT_WORK(&_fsm_oru_cplane_work, _fsm_oru_timeout_work);

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

	_fsm_oru_fwd_init_time_measurement();

	ret = fsm_oru_fwd_netdev_init();
	if (ret)
		goto out;

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
