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
#ifndef __FSM_DP_IOCTL_H__
#define __FSM_DP_IOCTL_H__

#include <linux/types.h>
#ifdef __KERNEL__
#include <linux/uio.h>
#else
#include <sys/uio.h>
#endif

#define FSM_DP_MAX_IOV_SIZE	64
#define FSM_DP_MAX_SG_IOV_SIZE	8



#define FSM_DP_IOCTL_BASE			'f'

#define FSM_DP_IOCTL_MEMPOOL_ALLOC	\
		_IOWR(FSM_DP_IOCTL_BASE, 1, struct fsm_dp_ioctl_mempool_alloc)

#define FSM_DP_IOCTL_MEMPOOL_GET_CONFIG	\
		_IOWR(FSM_DP_IOCTL_BASE, 2, struct fsm_dp_ioctl_getcfg)

#define FSM_DP_IOCTL_RX_GET_CONFIG	\
		_IOWR(FSM_DP_IOCTL_BASE, 3, struct fsm_dp_ioctl_getcfg)

#define FSM_DP_IOCTL_TX			\
		_IOWR(FSM_DP_IOCTL_BASE, 4, struct iovec)

#define FSM_DP_IOCTL_SG_TX		\
		_IOWR(FSM_DP_IOCTL_BASE, 5, struct iovec)

#define FSM_DP_IOCTL_TX_MODE_CONFIG	\
		_IOWR(FSM_DP_IOCTL_BASE, 6, unsigned int)

/* ioctl command for testing */
#define FSM_DP_IOCTL_TEST_RING_WRITE	_IO(FSM_DP_IOCTL_BASE, 0x11)
#define FSM_DP_IOCTL_TEST_RING_GET_CONFIG	\
		_IOWR(FSM_DP_IOCTL_BASE, 0x12, struct fsm_dp_ioctl_getcfg)

/* special value to write for testing */
#define TEST_RING_WRITE_MAGIC_VALUE	0xFFFFFFFE

/* message header version */
#define FSM_DP_MSG_HDR_VERSION		0x1

enum fsm_dp_mem_type {
	FSM_DP_MEM_TYPE_DL_L1_DATA,
	FSM_DP_MEM_TYPE_DL_L1_CTL,
	FSM_DP_MEM_TYPE_DL_RF,
	FSM_DP_MEM_TYPE_UL,
	FSM_DP_MEM_TYPE_LAST,
};

enum fsm_dp_mmap_type {
	FSM_DP_MMAP_TYPE_MEM,
	FSM_DP_MMAP_TYPE_RING,
	FSM_DP_MMAP_TYPE_LAST,
};

enum fsm_dp_rx_type {
	FSM_DP_RX_TYPE_L1,
	FSM_DP_RX_TYPE_RF,
	FSM_DP_RX_TYPE_TA,
	FSM_DP_RX_TYPE_LPBK,
	FSM_DP_RX_TYPE_ORU,
	FSM_DP_RX_TYPE_LAST,
};

enum fsm_dp_msg_type {
	FSM_DP_MSG_TYPE_L1		= 0,
	FSM_DP_MSG_TYPE_RF		= 1,
	FSM_DP_MSG_TYPE_TA		= 2,
	FSM_DP_MSG_TYPE_ORU		= 3,
	FSM_DP_MSG_TYPE_LPBK_REQ	= 0xFE,
	FSM_DP_MSG_TYPE_LPBK_RSP	= 0xFF,
};

/* Note! when we add a new message type, change this macro */
#define FSM_DP_NUM_MSG_TYPE (FSM_DP_MSG_TYPE_ORU + 3)


struct fsm_dp_msghdr {
	uint32_t version : 8;
	uint32_t type : 8;
	uint32_t reserved : 15;
	uint32_t aggr: 1 ; /* if set, fsm_dp_aggrhdr follows */
	uint32_t length : 16;
	uint32_t sequence : 16;
} __attribute__((packed));


#define FSM_DP_BUFFER_FENCE_SIG 0xDEADFACE
#define FSM_DP_BUFFER_SIG       0xDAC0FFEE
#define FSM_DP_BUFFER_FENCING   1

/*
 * The corresponding aggr msg is starting at offset from aggr header of
 * size bytes.
 */
struct fsm_dp_aggriob {
	uint16_t offset;
	uint16_t size;
};

/* fsm_dp_aggrhdr */
struct fsm_dp_aggrhdr {
	uint32_t reserved : 24;
	uint32_t n_iobs: 8; /* number of fsm_dp_aggriov */
	struct fsm_dp_aggriob iob[0]; /* nIovs fsm_dp_aggriov follows */
} __attribute__((packed));

/*
 * A buffer control is an area with size of
 * L1_CACHE_BYTES (64 bytes for arm64).
 * It is placed at the beginging
 * of a buffer.
 * fsm_dp_buf_cntrl is placed at the
 * control area. The last
 * 4 bytes of the area is a fence defined as
 * FSM_DP_BUFFER_FENCE_SIG
 * The size of fsm_dp_buf_cntrl
 * should be less  L1_CACHE_BYTES.
 * User data is placed after the
 * control area of L1_CACHE_BYTES size.
 * User data starts with fsm_dp_msghdr
 */
#define FSM_DP_L1_CACHE_BYTES 64  /*
				   * FSM_DP_L1_CACHE_BYTES is the same as
				   * L1_CACHE_BYTES.
				   * fsm_dp_ioctl.h is included in
				   * the applications,
				   * The symbol L1_CACHE_BYTES is defined in the
				   * kernel, not be used here. Therefore,
				   * it is redefined.
				   */
/*
 * xmit_status definition
 * If xmit errors, defined as -(error code)
 */
#define FSM_DP_XMIT_IN_PROGRESS (1)
#define FSM_DP_XMIT_OK		0

/*
 * maximum mtu size for FSM DP application, including fsm_dp header
 * Note, need to make sure both sides in sync between NPU, and Q6
 */
#define FSM_DP_MAX_DL_MSG_LEN   ((16 * 1024) - FSM_DP_L1_CACHE_BYTES)
#define FSM_DP_MAX_UL_MSG_LEN   ((16 * 1024) - FSM_DP_L1_CACHE_BYTES)

struct fsm_dp_buf_cntrl {
	uint32_t signature;
	uint32_t state;
	struct timespec ts;
	int32_t xmit_status;
	uint32_t buf_index;
	unsigned char spare[FSM_DP_L1_CACHE_BYTES
		- sizeof(uint32_t) /* signature */
		- sizeof(uint32_t) /* state */
		- sizeof(struct timespec) /* ts */
		- sizeof(int32_t) /* xmit_status */
		- sizeof(uint32_t) /* buf_index */
		- sizeof(uint32_t)];/* fence */
	uint32_t fence;
} __attribute__((packed));

enum fsm_dp_buf_state {
	FSM_DP_BUF_STATE_KERNEL_FREE,
	FSM_DP_BUF_STATE_KERNEL_ALLOC_RECV_DMA,
	FSM_DP_BUF_STATE_KERNEL_RECVCMP_MSGQ_TO_APP,
	FSM_DP_BUF_STATE_KERNEL_XMIT_DMA,
	FSM_DP_BUF_STATE_KERNEL_XMIT_DMA_COMP,
	FSM_DP_BUF_STATE_USER_FREE,
	FSM_DP_BUF_STATE_USER_ALLOC,
	FSM_DP_BUF_STATE_USER_RECV,
	FSM_DP_BUF_STATE_LAST,
};

typedef unsigned long fsm_dp_ring_element_data_t;
typedef unsigned int fsm_dp_ring_index_t;

struct fsm_dp_ring_element {
	uint64_t element_ctrl;	/* 1 entry not valid, 0 valid */
				/* Other bits for control flags: tbd */

	fsm_dp_ring_element_data_t element_data;
				/*
				 * If the ring is used for
				 * fsm dp buffer management,
				 * ring data is pointing to
				 * a buffer fsm_dp_msghdr area
				 */
};

typedef struct fsm_dp_ring_element fsm_dp_ring_element_t;

struct fsm_dp_mmap_cfg {
	__u32 length;	/* length parameter for mmap */
	__u32 cookie;	/* last parameter for mmap */
};

struct fsm_dp_ring_cfg {
	struct fsm_dp_mmap_cfg mmap;	/* mmap parameters */
	__u32 size;			/* ring size */
	__u32 prod_head_off;		/* page offset of prod_head */
	__u32 prod_tail_off;		/* page offset of prod_tail */
	__u32 cons_head_off;		/* page offset of cons_head */
	__u32 cons_tail_off;		/* page offset of cons_tail */
	__u32 ringbuf_off;		/* page offset of ring buffer */
};

struct fsm_dp_mem_cfg {
	struct fsm_dp_mmap_cfg mmap;	/* mmap parameters */
	__u32 buf_sz;			/* size of buffer for user data */
	__u32 buf_cnt;			/* number of buffer */
	__u32 buf_overhead_sz;		/*
					 * size of buffer overhead,
					 * on top of buf_sz.
					 */
	__u32 cluster_size;		/* cluster size in bytes.
					 * number of buffers in a cluster:
					 *  cluster_size /(buf_overhead_sz +
					 *                 buf_sz)
					 * A buffer starts at beginning of
					 * a cluster. Spared space with
					 * size less than (buf_overhead_sz
					 *          +  buf_sz) at end of
					 * a cluster is not used.
					 */
	__u32 num_cluster;		/* number of cluster */
	__u32 buf_per_cluster;		/* number of buffers per cluster  */
};

struct fsm_dp_mempool_cfg {
	enum fsm_dp_mem_type type;
	struct fsm_dp_mem_cfg mem;
	struct fsm_dp_ring_cfg ring;
};

struct fsm_dp_ioctl_mempool_alloc {
	__u32 type;		/* type defined in enum fsm_dp_mem_type */
	__u32 buf_sz;		/* size of buffer */
	__u32 buf_num;		/* number of buffer */
	struct fsm_dp_mempool_cfg *cfg;	/* for kernel to return config info */
};

struct fsm_dp_ioctl_getcfg {
	__u32 type;
	void *cfg;
};

static inline int fsm_dp_mem_type_is_valid(enum fsm_dp_mem_type type)
{
	return (type >= 0 && type < FSM_DP_MEM_TYPE_LAST);
}

static inline const char *fsm_dp_mem_type_to_str(enum fsm_dp_mem_type type)
{
	switch (type) {
	case FSM_DP_MEM_TYPE_DL_L1_DATA: return "DL_L1_DATA";
	case FSM_DP_MEM_TYPE_DL_L1_CTL: return "DL_L1_CTRL";
	case FSM_DP_MEM_TYPE_DL_RF: return "DL_RF";
	case FSM_DP_MEM_TYPE_UL: return "UL";
	default: return "unknown";
	}
}

static inline int fsm_dp_mmap_type_is_valid(enum fsm_dp_mmap_type type)
{
	return (type >= 0 && type < FSM_DP_MMAP_TYPE_LAST);
}

static inline const char *fsm_dp_mmap_type_to_str(enum fsm_dp_mmap_type type)
{
	switch (type) {
	case FSM_DP_MMAP_TYPE_MEM: return "Memory";
	case FSM_DP_MMAP_TYPE_RING: return "Ring";
	default: return "unknown";
	}
}

static inline int fsm_dp_rx_type_is_valid(enum fsm_dp_rx_type type)
{
	return (type >= 0 && type < FSM_DP_RX_TYPE_LAST);
}

static inline const char *fsm_dp_rx_type_to_str(enum fsm_dp_rx_type type)
{
	switch (type) {
	case FSM_DP_RX_TYPE_L1: return "L1";
	case FSM_DP_RX_TYPE_RF: return "RF";
	case FSM_DP_RX_TYPE_TA: return "TA";
	case FSM_DP_RX_TYPE_ORU: return "ORU";
	case FSM_DP_RX_TYPE_LPBK: return "LOOPBACK";
	default: return "unknown";
	}
}


static inline const char *fsm_dp_buf_state_to_str(enum fsm_dp_buf_state state)
{
	switch (state) {
	case FSM_DP_BUF_STATE_KERNEL_FREE:
		return "KERNEL FREE";
	case FSM_DP_BUF_STATE_KERNEL_ALLOC_RECV_DMA:
		return "KERNEL ALLOC RECV DMA";
	case FSM_DP_BUF_STATE_KERNEL_RECVCMP_MSGQ_TO_APP:
		return "KERNEL RECV CMP MSGQ TO APP";
	case FSM_DP_BUF_STATE_KERNEL_XMIT_DMA:
		return "KERNEL XMIT DMA";
	case FSM_DP_BUF_STATE_KERNEL_XMIT_DMA_COMP:
		return "KERNEL XMIT DMA COMP";
	case FSM_DP_BUF_STATE_USER_FREE:
		return "USER FREE";
	case FSM_DP_BUF_STATE_USER_ALLOC:
		return "USER ALLOC";
	case FSM_DP_BUF_STATE_USER_RECV:
		return "USER RECV";
	case FSM_DP_BUF_STATE_LAST:
	default:
		return "unknown";
	};
}

#endif /* __FSM_DP_IOCTL_H__ */
