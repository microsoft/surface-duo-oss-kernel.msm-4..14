/* Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
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
#ifndef __FSM_DP__
#define __FSM_DP__

#ifndef __KERNEL__
#define __KERNEL__
#endif

#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/netdevice.h>
#include <linux/atomic.h>
#include <linux/workqueue.h>

#include <linux/fsm_dp_ioctl.h>

#include "fsm_dp_mhi.h"
#include "fsm_dp_mem.h"

#define FSM_DP_MODULE_NAME		"fsm-dp"
#define FSM_DP_DEV_CLASS_NAME	FSM_DP_MODULE_NAME
#define FSM_DP_CDEV_NAME	FSM_DP_MODULE_NAME

#define FSM_DP_DEBUG	pr_debug
#define FSM_DP_INFO	pr_info
#define FSM_DP_ERROR	pr_err
#define FSM_DP_WARN	pr_warn

struct vm_area_struct;

#define MMAP_MEM_TYPE_SHIFT	24
#define MMAP_MEM_TYPE_MASK	0xFF
#define MMAP_TYPE_SHIFT		16
#define MMAP_TYPE_MASK		0xFF

#define MMAP_COOKIE(type, target) \
	((((type) & MMAP_MEM_TYPE_MASK) <<  MMAP_MEM_TYPE_SHIFT) | \
	(((target) & MMAP_TYPE_MASK) << MMAP_TYPE_SHIFT))

#define MMAP_COOKIE_TO_MEM_TYPE(cookie) \
	(((cookie) >> MMAP_MEM_TYPE_SHIFT) & MMAP_MEM_TYPE_MASK)

#define MMAP_COOKIE_TO_TYPE(cookie) \
	(((cookie) >> MMAP_TYPE_SHIFT) & MMAP_TYPE_MASK)

#define MMAP_RX_COOKIE(type) \
	MMAP_COOKIE((type)+FSM_DP_MEM_TYPE_LAST, FSM_DP_MMAP_TYPE_RING)

#define MMAP_RX_COOKIE_TO_TYPE(cookie) \
	(MMAP_COOKIE_TO_MEM_TYPE(cookie) - FSM_DP_MEM_TYPE_LAST)

#define TEST_RING_MMAP_COOKIE	0x80000000

#define TX_MODE_LOOPBACK 1

#define DEFAULT_FSM_MEM_BUF_SIZE	2048
#define DEFAULT_FSM_MEM_UL_BUF_CNT	4096
#define DEFAULT_RX_QUEUE_SIZE		1024

#define FSM_DP_TX_FLAG_SG	0x01
#define FSM_DP_TX_FLAG_LOOPBACK	0x02

#define FSM_DP_ASSERT(cond, msg) do { \
	if (cond) \
		panic(msg); \
} while (0)

/*
 * vma mapping for mempool which includes
 * - buffer memory region
 * - ring buffer shared between kernel and user space
 *   for buffer management
 */
struct fsm_dp_mempool_vma {
	struct fsm_dp_mempool **pp_mempool;
	struct vm_area_struct *vma[FSM_DP_MMAP_TYPE_LAST];	/* mmap vma */
	atomic_t refcnt[FSM_DP_MMAP_TYPE_LAST];
	bool usr_alloc;	/* allocated by user using ioctl */
};

/* vma mapping for receive queue */
struct fsm_dp_rxqueue_vma {
	enum fsm_dp_rx_type type;
	struct vm_area_struct *vma;
	atomic_t refcnt;
};

/* RX queue using ring buffer */
struct fsm_dp_rxqueue {
	enum fsm_dp_rx_type type;
	struct fsm_dp_ring ring;
	wait_queue_head_t wq;
	atomic_t refcnt;
	bool inited;
};

/* fsm_dp_cdev tx_mode field bitmap */
#define TX_MODE_LOOPBACK 1

/* Per-process character device structure */
struct fsm_dp_cdev {
	struct list_head list;
	struct fsm_dp_drv *pdrv;
	pid_t pid;

	/* vma mapping for memory pool */
	struct fsm_dp_mempool_vma mempool_vma[FSM_DP_MEM_TYPE_LAST];

	/* vma mapping for receiving queue */
	struct fsm_dp_rxqueue_vma rxqueue_vma[FSM_DP_RX_TYPE_LAST];
	unsigned int tx_mode;
};

struct fsm_dp_loopback_stats {
	unsigned long tx_cnt;
	unsigned long tx_enque;
	unsigned long tx_drop;
	unsigned long tx_err;
	unsigned long rx_cnt;
	unsigned long rx_enque;
	unsigned long rx_drop;
	unsigned long rx_err;
	unsigned long run;
	unsigned long sched;
};

struct fsm_dp_loopback_job {
	struct list_head list;
	void *data;
	unsigned int length;
	unsigned int dest;
	bool rx_loopback;
};

struct fsm_dp_loopback_task {
	struct list_head free_q;
	struct list_head job_q;
	spinlock_t lock;
	struct work_struct work;
	struct workqueue_struct *workq;
	void *alloc_ptr;
	bool inited;
	struct fsm_dp_loopback_stats stats;
};

struct fsm_dp_test_ring {
	struct fsm_dp_ring ring;
	bool enable;
};

struct fsm_dp_core_stats {
	unsigned long tx_cnt;
	unsigned long tx_err;

	unsigned long rx_cnt;
	unsigned long rx_badmsg;
	unsigned long rx_drop;
	unsigned long rx_int;
	unsigned long rx_budget_overflow;
};

struct fsm_dp_drv {
	struct device *dev;
	struct class *dev_class;
	struct fsm_dp_mhi mhi;
	struct cdev cdev;
	struct net_device dummy_dev;
	struct napi_struct napi;
	struct mutex cdev_lock;
	struct list_head cdev_head;
	struct mutex mempool_lock;
	atomic_t tx_seqnum;
	struct fsm_dp_mempool *mempool[FSM_DP_MEM_TYPE_LAST];
	struct fsm_dp_rxqueue rxq[FSM_DP_RX_TYPE_LAST];
	struct fsm_dp_loopback_task loopback;
	struct fsm_dp_core_stats stats;
	struct work_struct alloc_work;

#ifdef CONFIG_FSM_DP_TEST
	struct fsm_dp_test_ring test_ring;
#endif
};

struct fsm_dp_kernel_register_db_entry {
	struct fsm_dp_drv *pdrv;
	enum fsm_dp_msg_type msg_type;
	int (*tx_cmplt_cb)(struct sk_buff *skb);
	int (*rx_cb)(struct page *p, unsigned int page_offset, char *buf,
			unsigned int length);
};

int fsm_dp_cdev_init(struct fsm_dp_drv *pdrv);
void fsm_dp_cdev_cleanup(struct fsm_dp_drv *pdrv);

int fsm_dp_debugfs_init(struct fsm_dp_drv *pdrv);
void fsm_dp_debugfs_cleanup(struct fsm_dp_drv *pdrv);


int fsm_dp_tx(
	struct fsm_dp_drv *pdrv,
	struct iovec *iov,
	unsigned int iov_nr,
	unsigned int flag,
	dma_addr_t dma_addr[]);

void fsm_dp_rx(struct fsm_dp_drv *pdrv, void *data, unsigned int length);

void fsm_dp_hex_dump(unsigned char *buf, unsigned int len);

static inline struct fsm_dp_kernel_register_db_entry *
fsm_dp_find_reg_db_type(enum fsm_dp_msg_type msg_type)
{
	extern struct fsm_dp_kernel_register_db_entry fsm_dp_reg_db[];

	switch (msg_type) {
	case FSM_DP_MSG_TYPE_L1:
	case FSM_DP_MSG_TYPE_RF:
	case FSM_DP_MSG_TYPE_TA:
	case FSM_DP_MSG_TYPE_ORU:
		return &fsm_dp_reg_db[msg_type];
	case FSM_DP_MSG_TYPE_LPBK_REQ:
		return &fsm_dp_reg_db[FSM_DP_MSG_TYPE_ORU] + 1;
	case FSM_DP_MSG_TYPE_LPBK_RSP:
		return &fsm_dp_reg_db[FSM_DP_MSG_TYPE_ORU] + 2;
	default:
		break;
	}
	return NULL;
};

void fsm_dp_mempool_dev_destroy(struct fsm_dp_drv *pdrv);

#endif /* __FSM_DP__ */
