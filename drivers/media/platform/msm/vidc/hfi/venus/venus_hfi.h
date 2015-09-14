/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __H_VENUS_HFI_H__
#define __H_VENUS_HFI_H__

#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include "msm_vidc_resources.h"

#include "hfi/vidc_hfi_api.h"
#include "hfi/vidc_hfi_helper.h"
#include "hfi/hfi_packetization.h"

#define HFI_MASK_QHDR_TX_TYPE			0xFF000000
#define HFI_MASK_QHDR_RX_TYPE			0x00FF0000
#define HFI_MASK_QHDR_PRI_TYPE			0x0000FF00
#define HFI_MASK_QHDR_Q_ID_TYPE			0x000000FF

#define HFI_Q_ID_HOST_TO_CTRL_CMD_Q		0x00
#define HFI_Q_ID_CTRL_TO_HOST_MSG_Q		0x01
#define HFI_Q_ID_CTRL_TO_HOST_DEBUG_Q		0x02
#define HFI_MASK_QHDR_STATUS			0x000000FF

#define VIDC_MAX_UNCOMPRESSED_FMT_PLANES	3

#define IFACEQ_NUM				3
#define IFACEQ_CMD_IDX				0
#define IFACEQ_MSG_IDX				1
#define IFACEQ_DBG_IDX				2
#define IFACEQ_MAX_BUF_COUNT			50
#define IFACEQ_MAX_PARALLEL_CLNTS		16
#define IFACEQ_DFLT_QHDR			0x01010000

struct hfi_queue_table_header {
	u32 version;
	u32 size;
	u32 qhdr0_offset;
	u32 qhdr_size;
	u32 num_q;
	u32 num_active_q;
};

struct hfi_queue_header {
	u32 status;
	u32 start_addr;
	u32 type;
	u32 q_size;
	u32 pkt_size;
	u32 pkt_drop_cnt;
	u32 rx_wm;
	u32 tx_wm;
	u32 rx_req;
	u32 tx_req;
	u32 rx_irq_status;
	u32 tx_irq_status;
	u32 read_idx;
	u32 write_idx;
};

#define IFACEQ_TABLE_SIZE	\
	(sizeof(struct hfi_queue_table_header) +	\
	 sizeof(struct hfi_queue_header) * IFACEQ_NUM)

#define IFACEQ_QUEUE_SIZE	(VIDC_IFACEQ_MAX_PKT_SIZE *	\
	IFACEQ_MAX_BUF_COUNT * IFACEQ_MAX_PARALLEL_CLNTS)

#define IFACEQ_GET_QHDR_START_ADDR(ptr, i)	\
	(void *)((ptr + sizeof(struct hfi_queue_table_header)) +	\
		(i * sizeof(struct hfi_queue_header)))

#define QDSS_SIZE		4096
#define SFR_SIZE		4096
#define QUEUE_SIZE		(IFACEQ_TABLE_SIZE + \
				(IFACEQ_QUEUE_SIZE * IFACEQ_NUM))

#define ALIGNED_QDSS_SIZE	ALIGN(QDSS_SIZE, SZ_4K)
#define ALIGNED_SFR_SIZE	ALIGN(SFR_SIZE, SZ_4K)
#define ALIGNED_QUEUE_SIZE	ALIGN(QUEUE_SIZE, SZ_4K)
#define SHARED_QSIZE		ALIGN(ALIGNED_SFR_SIZE + ALIGNED_QUEUE_SIZE + \
				      ALIGNED_QDSS_SIZE, SZ_1M)

struct mem_desc {
	u32 da;		/* device address */
	void *kva;	/* kernel virtual address */
	u32 size;
	struct smem *smem;
};

struct iface_queue {
	void *qhdr;
	struct mem_desc qarray;
};

/* Internal data used in vidc_hal not exposed to msm_vidc*/
struct hal_data {
	u32 irq;
	void __iomem *base;
};

enum venus_state {
	VENUS_STATE_DEINIT = 1,
	VENUS_STATE_INIT,
};

struct venus_hfi_device {
	struct list_head list;
	struct list_head sessions;
	u32 intr_status;
	u32 device_id;
	u32 clk_load;
	u32 codecs_enabled;
	u32 last_packet_type;
	bool power_enabled;
	struct mutex lock;
	struct mutex session_lock;
	struct mem_desc ifaceq_table;
	struct mem_desc sfr;
	struct iface_queue queues[IFACEQ_NUM];
	struct smem_client *mem_client;
	struct hal_data *hal_data;
	struct workqueue_struct *venus_pm_workq;
	int spur_count;
	int reg_count;
	struct vidc_resources *res;
	enum venus_state state;
	const struct hfi_packetization_ops *pkt_ops;
	enum hfi_packetization_type packetization_type;
	u8 pkt_buf[VIDC_IFACEQ_VAR_HUGE_PKT_SIZE];
};

void venus_hfi_deinitialize(struct hfi_device *device);

struct hfi_device *
venus_hfi_initialize(u32 device_id, struct vidc_resources *res);

#endif
