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

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/qcom_scm.h>

#include <linux/soc/qcom/smem.h>

#include "hfi/hfi_packetization.h"
#include "msm_vidc_debug.h"
#include "venus_hfi.h"
#include "vidc_hfi_io.h"
#include "msm_smem.h"

static struct hal_device_data hal_ctxt;

/* Poll interval in uS */
#define POLL_INTERVAL_US	50

enum tzbsp_video_state {
	TZBSP_VIDEO_STATE_SUSPEND = 0,
	TZBSP_VIDEO_STATE_RESUME
};

static int venus_hfi_session_clean(void *session);

static void venus_pm_handler(struct work_struct *work);
static DECLARE_DELAYED_WORK(venus_hfi_pm_work, venus_pm_handler);
static int venus_hfi_resume(void *dev);
static inline int venus_power_on(struct venus_hfi_device *device);
static void venus_flush_debug_queue(struct venus_hfi_device *device,
				    void *packet);
static int venus_hfi_initialize_packetization(struct venus_hfi_device *device);

static DECLARE_COMPLETION(release_resources_done);
static DECLARE_COMPLETION(pc_prep_done);

static inline void venus_set_state(struct venus_hfi_device *hdev,
				   enum venus_state state)
{
	mutex_lock(&hdev->lock);
	hdev->state = state;
	mutex_unlock(&hdev->lock);
}

static inline bool venus_is_valid_state(struct venus_hfi_device *hdev)
{
	return hdev->state != VENUS_STATE_DEINIT;
}

static void venus_dump_packet(u8 *packet)
{
	u32 c = 0, packet_size = *(u32 *)packet;
	const int row_size = 32;
	/*
	 * row must contain enough for 0xdeadbaad * 8 to be converted into
	 * "de ad ba ab " * 8 + '\0'
	 */
	char row[3 * row_size];

	for (c = 0; c * row_size < packet_size; ++c) {
		int bytes_to_read = ((c + 1) * row_size > packet_size) ?
			packet_size % row_size : row_size;
		hex_dump_to_buffer(packet + c * row_size, bytes_to_read,
				   row_size, 4, row, sizeof(row), false);
		dprintk(VIDC_PKT, "%s\n", row);
	}
}

static int venus_write_queue(struct iface_queue *queue, u8 *packet,
			     u32 *rx_req_is_set)
{
	struct hfi_queue_header *qhdr;
	u32 packet_size_in_words, new_write_idx;
	u32 empty_space, read_idx, write_idx;
	u32 *write_ptr;

	if (!queue->qarray.kva)
		return -EINVAL;

	qhdr = queue->qhdr;
	if (!qhdr)
		return -ENOMEM;

	if (vidc_debug & VIDC_PKT)
		venus_dump_packet(packet);

	packet_size_in_words = (*(u32 *)packet) >> 2;
	if (!packet_size_in_words)
		return -EINVAL;

	read_idx = qhdr->read_idx;
	write_idx = qhdr->write_idx;

	if (write_idx >= read_idx)
		empty_space = qhdr->q_size - (write_idx - read_idx);
	else
		empty_space = read_idx - write_idx;

	if (empty_space <= packet_size_in_words) {
		qhdr->tx_req = 1;
		return -ENOSPC;
	}

	qhdr->tx_req = 0;

	new_write_idx = write_idx + packet_size_in_words;
	write_ptr = (u32 *)(queue->qarray.kva + (write_idx << 2));
	if (new_write_idx < qhdr->q_size) {
		memcpy(write_ptr, packet, packet_size_in_words << 2);
	} else {
		new_write_idx -= qhdr->q_size;
		memcpy(write_ptr, packet,
		       (packet_size_in_words - new_write_idx) << 2);
		memcpy((void *)queue->qarray.kva,
			packet + ((packet_size_in_words - new_write_idx) << 2),
			new_write_idx << 2);
	}

	/* Memory barrier to make sure packet is written before updating the
	 * write index
	 */
	mb();

	qhdr->write_idx = new_write_idx;
	*rx_req_is_set = (1 == qhdr->rx_req) ? 1 : 0;

	/* Memory barrier to make sure write index is updated before an
	 * interupt is raised on venus.
	 */
	mb();

	return 0;
}

static int venus_read_queue(struct iface_queue *queue, u8 *packet,
			    u32 *pb_tx_req_is_set)
{
	struct hfi_queue_header *qhdr;
	u32 packet_size_in_words, new_read_idx;
	u32 *read_ptr;
	u32 receive_request = 0;
	int ret = 0;

	if (!queue->qarray.kva)
		return -EINVAL;

	/* Memory barrier to make sure data is valid before reading it */
	mb();

	qhdr = queue->qhdr;
	if (!qhdr)
		return -ENOMEM;

	/*
	 * Do not set receive request for debug queue, if set,
	 * Venus generates interrupt for debug messages even
	 * when there is no response message available.
	 * In general debug queue will not become full as it
	 * is being emptied out for every interrupt from Venus.
	 * Venus will anyway generates interrupt if it is full.
	 */
	if (qhdr->type & HFI_Q_ID_CTRL_TO_HOST_MSG_Q)
		receive_request = 1;

	if (qhdr->read_idx == qhdr->write_idx) {
		qhdr->rx_req = receive_request;
		*pb_tx_req_is_set = 0;
		return -ENODATA;
	}

	read_ptr = (u32 *)(queue->qarray.kva + (qhdr->read_idx << 2));
	packet_size_in_words = (*read_ptr) >> 2;
	if (!packet_size_in_words)
		return -EINVAL;

	new_read_idx = qhdr->read_idx + packet_size_in_words;
	if (((packet_size_in_words << 2) <= VIDC_IFACEQ_VAR_HUGE_PKT_SIZE) &&
	    qhdr->read_idx <= qhdr->q_size) {
		if (new_read_idx < qhdr->q_size) {
			memcpy(packet, read_ptr, packet_size_in_words << 2);
		} else {
			new_read_idx -= qhdr->q_size;
			memcpy(packet, read_ptr,
			       (packet_size_in_words - new_read_idx) << 2);
			memcpy(packet +
			       ((packet_size_in_words - new_read_idx) << 2),
			       (u8 *)queue->qarray.kva,
			       new_read_idx << 2);
		}
	} else {
		/* bad packet received, dropping */
		new_read_idx = qhdr->write_idx;
		ret = -EBADMSG;
	}

	qhdr->read_idx = new_read_idx;

	if (qhdr->read_idx != qhdr->write_idx)
		qhdr->rx_req = 0;
	else
		qhdr->rx_req = receive_request;

	*pb_tx_req_is_set = (1 == qhdr->tx_req) ? 1 : 0;

	if (vidc_debug & VIDC_PKT)
		venus_dump_packet(packet);

	return ret;
}

static int venus_alloc(struct venus_hfi_device *hdev, struct mem_desc *vmem,
		       u32 size, u32 align, u32 flags, u32 usage)
{
	struct smem *mem;
	int ret;

	venus_hfi_resume(hdev);

	mem = smem_alloc(hdev->mem_client, size, align, flags, usage, 1);
	if (IS_ERR(mem))
		return PTR_ERR(mem);

	ret = smem_cache_operations(hdev->mem_client, mem, SMEM_CACHE_CLEAN);
	if (ret) {
		dprintk(VIDC_WARN, "Failed to clean cache\n");
		dprintk(VIDC_WARN, "This may result in undefined behavior\n");
	}

	vmem->size = mem->size;
	vmem->smem = mem;
	vmem->kva = mem->kvaddr;
	vmem->da = mem->da;

	return 0;
}

static void venus_free(struct venus_hfi_device *hdev, struct smem *mem)
{
	smem_free(hdev->mem_client, mem);
}

static void venus_writel(struct venus_hfi_device *hdev, u32 reg, u32 value)
{
	writel(value, hdev->hal_data->base + reg);
}

static u32 venus_readl(struct venus_hfi_device *hdev, u32 reg)
{
	return readl(hdev->hal_data->base + reg);
}

static void venus_set_registers(struct venus_hfi_device *hdev)
{
	struct reg_set *regs = &hdev->res->reg_set;
	const struct reg_value_pair *tbl = regs->reg_tbl;
	int i;

	for (i = 0; i < regs->count; i++)
		venus_writel(hdev, tbl[i].reg, tbl[i].value);
}

static int venus_iface_cmdq_write_nolock(struct venus_hfi_device *hdev,
					 void *pkt)
{
	u32 rx_req_is_set = 0;
	struct iface_queue *queue;
	struct vidc_hal_cmd_pkt_hdr *cmd_packet;
	int ret;

	WARN(!mutex_is_locked(&hdev->lock),
	     "Cmd queue write lock must be acquired");

	if (!venus_is_valid_state(hdev)) {
		dprintk(VIDC_DBG, "%s: fw not in init state\n", __func__);
		return -EINVAL;
	}

	cmd_packet = (struct vidc_hal_cmd_pkt_hdr *)pkt;
	hdev->last_packet_type = cmd_packet->packet_type;

	queue = &hdev->queues[IFACEQ_CMD_IDX];

	ret = venus_write_queue(queue, pkt, &rx_req_is_set);
	if (ret) {
		dprintk(VIDC_ERR, "iface cmdq queue is full\n");
		return ret;
	}

	ret = venus_power_on(hdev);
	if (ret)
		return ret;

	if (rx_req_is_set)
		venus_writel(hdev, VIDC_CPU_IC_SOFTINT,
			     1 << VIDC_CPU_IC_SOFTINT_H2A_SHFT);

	if (hdev->res->sw_power_collapsible) {
		dprintk(VIDC_DBG, "cancel and queue delayed work again\n");

		cancel_delayed_work(&venus_hfi_pm_work);

		if (!queue_delayed_work(hdev->venus_pm_workq,
					&venus_hfi_pm_work,
					msecs_to_jiffies(
					vidc_pwr_collapse_delay))) {
			dprintk(VIDC_DBG,
				"PM work already scheduled\n");
		}
	}

	return 0;
}

static int venus_iface_cmdq_write(struct venus_hfi_device *hdev, void *pkt)
{
	int ret;

	mutex_lock(&hdev->lock);
	ret = venus_iface_cmdq_write_nolock(hdev, pkt);
	mutex_unlock(&hdev->lock);

	return ret;
}

static int
venus_hfi_core_set_resource(void *device, struct vidc_resource_hdr *hdr,
			    void *resource_value, bool locked)
{
	struct venus_hfi_device *hdev = device;
	struct hfi_sys_set_resource_pkt *pkt;
	u8 packet[VIDC_IFACEQ_VAR_SMALL_PKT_SIZE];
	int ret;

	pkt = (struct hfi_sys_set_resource_pkt *) packet;

	ret = call_hfi_pkt_op(hdev, sys_set_resource, pkt, hdr, resource_value);
	if (ret)
		return ret;

	ret = locked ? venus_iface_cmdq_write(hdev, pkt) :
		       venus_iface_cmdq_write_nolock(hdev, pkt);
	if (ret)
		return ret;

	return 0;
}

static int
venus_hfi_core_release_resource(void *device, struct vidc_resource_hdr *hdr)
{
	struct hfi_sys_release_resource_pkt pkt;
	struct venus_hfi_device *hdev = device;
	int ret;

	ret = call_hfi_pkt_op(hdev, sys_release_resource, &pkt, hdr);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, &pkt);
	if (ret)
		return ret;

	return 0;
}

static int venus_tzbsp_set_video_state(enum tzbsp_video_state state)
{
	return qcom_scm_set_video_state(state, 0);
}

static int venus_reset_core(struct venus_hfi_device *hdev)
{
	u32 ctrl_status = 0, count = 0;
	int max_tries = 100, ret = 0;

	venus_writel(hdev, VIDC_CTRL_INIT, 0x1);

	venus_writel(hdev, VIDC_WRAPPER_INTR_MASK,
		     VIDC_WRAPPER_INTR_MASK_A2HVCODEC_BMSK);

	while (!ctrl_status && count < max_tries) {
		ctrl_status = venus_readl(hdev, VIDC_CPU_CS_SCIACMDARG0);
		if ((ctrl_status & 0xfe) == 0x4) {
			dprintk(VIDC_ERR, "invalid setting for UC_REGION\n");
			ret = -EINVAL;
			break;
		}

		usleep_range(500, 1000);
		count++;
	}

	if (count >= max_tries)
		ret = -ETIMEDOUT;

	return ret;
}

static int venus_run(struct venus_hfi_device *hdev)
{
	int ret;

	/*
	 * Re-program all of the registers that get reset as a result of
	 * regulator_disable() and _enable()
	 */
	venus_set_registers(hdev);

	venus_writel(hdev, VIDC_UC_REGION_ADDR, hdev->ifaceq_table.da);
	venus_writel(hdev, VIDC_UC_REGION_SIZE, SHARED_QSIZE);
	venus_writel(hdev, VIDC_CPU_CS_SCIACMDARG2, hdev->ifaceq_table.da);
	venus_writel(hdev, VIDC_CPU_CS_SCIACMDARG1, 0x01);
	if (hdev->sfr.da)
		venus_writel(hdev, VIDC_SFR_ADDR, hdev->sfr.da);

	venus_writel(hdev, VIDC_WRAPPER_CLOCK_CONFIG, 0);
	venus_writel(hdev, VIDC_WRAPPER_CPU_CLOCK_CONFIG, 0);

	ret = venus_reset_core(hdev);
	if (ret) {
		dprintk(VIDC_ERR, "Failed to reset venus core\n");
		return ret;
	}

	return 0;
}

static int venus_halt_axi(struct venus_hfi_device *hdev)
{
	void __iomem *base = hdev->hal_data->base;
	u32 val;
	int ret;

	/* Halt AXI and AXI IMEM VBIF Access */
	val = venus_readl(hdev, VENUS_VBIF_AXI_HALT_CTRL0);
	val |= VENUS_VBIF_AXI_HALT_CTRL0_HALT_REQ;
	venus_writel(hdev, VENUS_VBIF_AXI_HALT_CTRL0, val);

	/* Request for AXI bus port halt */
	ret = readl_poll_timeout(base + VENUS_VBIF_AXI_HALT_CTRL1, val,
				 val & VENUS_VBIF_AXI_HALT_CTRL1_HALT_ACK,
				 POLL_INTERVAL_US,
				 VENUS_VBIF_AXI_HALT_ACK_TIMEOUT_US);
	if (ret) {
		dprintk(VIDC_WARN, "AXI bus port halt timeout\n");
		return ret;
	}

	return 0;
}

static int venus_power_off(struct venus_hfi_device *hdev)
{
	int ret;

	if (!hdev->power_enabled)
		return 0;

	ret = venus_halt_axi(hdev);
	if (ret) {
		dprintk(VIDC_WARN, "Failed to halt AXI\n");
		return ret;
	}

	dprintk(VIDC_DBG, "Entering power collapse\n");

	ret = venus_tzbsp_set_video_state(TZBSP_VIDEO_STATE_SUSPEND);
	if (ret) {
		dprintk(VIDC_WARN, "Failed to suspend video core %d\n", ret);
		return ret;
	}

	/*
	 * For some regulators, driver might have transfered the control to HW.
	 * So before touching any clocks, driver should get the regulator
	 * control back. Acquire regulators also makes sure that the regulators
	 * are turned ON. So driver can touch the clocks safely.
	 */

	hdev->power_enabled = false;

	dprintk(VIDC_INFO, "Venus power collapsed\n");

	return 0;
}

static int venus_power_on(struct venus_hfi_device *hdev)
{
	int ret;

	if (hdev->power_enabled)
		return 0;

	dprintk(VIDC_DBG, "Resuming from power collapse\n");

	/* Reboot the firmware */
	ret = venus_tzbsp_set_video_state(TZBSP_VIDEO_STATE_RESUME);
	if (ret)
		goto err_set_video_state;

	ret = venus_run(hdev);
	if (ret) {
		dprintk(VIDC_ERR, "Failed to run venus core\n");
		goto err_run;
	}

	/*
	 * Set the flag here to skip venus_power_on() which is
	 * being called again via *_alloc_set_imem() if imem is enabled
	 */
	hdev->power_enabled = true;

	dprintk(VIDC_INFO, "Resumed from power collapse\n");

	return 0;

err_run:
	venus_tzbsp_set_video_state(TZBSP_VIDEO_STATE_SUSPEND);
err_set_video_state:
	hdev->power_enabled = false;
	dprintk(VIDC_ERR, "Failed to resume from power collapse\n");
	return ret;
}

static int venus_iface_msgq_read_nolock(struct venus_hfi_device *hdev,
					void *pkt)
{
	struct iface_queue *queue;
	u32 tx_req_is_set = 0;
	int ret;

	if (!venus_is_valid_state(hdev)) {
		dprintk(VIDC_DBG, "%s: fw not in init state\n", __func__);
		return -EINVAL;
	}

	queue = &hdev->queues[IFACEQ_MSG_IDX];

	ret = venus_read_queue(queue, (u8 *)pkt, &tx_req_is_set);
	if (ret)
		return ret;

	if (tx_req_is_set)
		venus_writel(hdev, VIDC_CPU_IC_SOFTINT,
			     1 << VIDC_CPU_IC_SOFTINT_H2A_SHFT);

	return 0;
}

static int venus_iface_msgq_read(struct venus_hfi_device *hdev, void *pkt)
{
	int ret;

	mutex_lock(&hdev->lock);
	ret = venus_iface_msgq_read_nolock(hdev, pkt);
	mutex_unlock(&hdev->lock);

	return ret;
}

static int venus_iface_dbgq_read_nolock(struct venus_hfi_device *hdev,
					void *pkt)
{
	struct iface_queue *queue;
	u32 tx_req_is_set = 0, value;
	int ret;

	ret = venus_is_valid_state(hdev);
	if (!ret) {
		dprintk(VIDC_DBG, "%s: fw not in init state\n", __func__);
		return -EINVAL;
	}

	queue = &hdev->queues[IFACEQ_DBG_IDX];

	ret = venus_read_queue(queue, pkt, &tx_req_is_set);
	if (ret)
		return ret;

	if (tx_req_is_set) {
		value = 1 << VIDC_CPU_IC_SOFTINT_H2A_SHFT;
		venus_writel(hdev, VIDC_CPU_IC_SOFTINT, value);
	}

	return 0;
}

static int venus_iface_dbgq_read(struct venus_hfi_device *hdev, void *pkt)
{
	int ret;

	if (!pkt)
		return -EINVAL;

	mutex_lock(&hdev->lock);
	ret = venus_iface_dbgq_read_nolock(hdev, pkt);
	mutex_unlock(&hdev->lock);

	return ret;
}

static void venus_set_qhdr_defaults(struct hfi_queue_header *qhdr)
{
	qhdr->status = 1;
	qhdr->type = IFACEQ_DFLT_QHDR;
	qhdr->q_size = IFACEQ_QUEUE_SIZE / 4;
	qhdr->pkt_size = 0;
	qhdr->rx_wm = 1;
	qhdr->tx_wm = 1;
	qhdr->rx_req = 1;
	qhdr->tx_req = 0;
	qhdr->rx_irq_status = 0;
	qhdr->tx_irq_status = 0;
	qhdr->read_idx = 0;
	qhdr->write_idx = 0;
}

static void venus_interface_queues_release(struct venus_hfi_device *hdev)
{
	mutex_lock(&hdev->lock);

	venus_free(hdev, hdev->ifaceq_table.smem);
	venus_free(hdev, hdev->sfr.smem);

	memset(hdev->queues, 0, sizeof(hdev->queues));
	memset(&hdev->ifaceq_table, 0, sizeof(hdev->ifaceq_table));
	memset(&hdev->sfr, 0, sizeof(hdev->sfr));

	smem_delete_client(hdev->mem_client);

	hdev->mem_client = NULL;

	mutex_unlock(&hdev->lock);
}

static int venus_interface_queues_init(struct venus_hfi_device *hdev)
{
	struct hfi_queue_table_header *tbl_hdr;
	struct hfi_queue_header *hdr;
	struct iface_queue *queue;
	struct hfi_sfr *sfr;
	struct mem_desc mem;
	int offset = 0, ret;
	u32 i, size;

	size = SHARED_QSIZE - ALIGNED_SFR_SIZE - ALIGNED_QDSS_SIZE;

	ret = venus_alloc(hdev, &mem, size, 1, 0,
			  HAL_BUFFER_INTERNAL_CMD_QUEUE);
	if (ret)
		return ret;

	hdev->ifaceq_table.kva = mem.kva;
	hdev->ifaceq_table.da = mem.da;
	hdev->ifaceq_table.size = IFACEQ_TABLE_SIZE;
	hdev->ifaceq_table.smem = mem.smem;
	offset += hdev->ifaceq_table.size;

	for (i = 0; i < IFACEQ_NUM; i++) {
		queue = &hdev->queues[i];
		queue->qarray.da = mem.da + offset;
		queue->qarray.kva = mem.kva + offset;
		queue->qarray.size = IFACEQ_QUEUE_SIZE;
		queue->qarray.smem = NULL;
		offset += queue->qarray.size;
		queue->qhdr =
			IFACEQ_GET_QHDR_START_ADDR(hdev->ifaceq_table.kva, i);
		venus_set_qhdr_defaults(queue->qhdr);
	}

	ret = venus_alloc(hdev, &mem, ALIGNED_SFR_SIZE, 1, 0,
			  HAL_BUFFER_INTERNAL_CMD_QUEUE);
	if (ret) {
		hdev->sfr.da = 0;
	} else {
		hdev->sfr.da = mem.da;
		hdev->sfr.kva = mem.kva;
		hdev->sfr.size = ALIGNED_SFR_SIZE;
		hdev->sfr.smem = mem.smem;
	}

	tbl_hdr = hdev->ifaceq_table.kva;
	tbl_hdr->version = 0;
	tbl_hdr->size = IFACEQ_TABLE_SIZE;
	tbl_hdr->qhdr0_offset = sizeof(struct hfi_queue_table_header);
	tbl_hdr->qhdr_size = sizeof(struct hfi_queue_header);
	tbl_hdr->num_q = IFACEQ_NUM;
	tbl_hdr->num_active_q = IFACEQ_NUM;

	queue = &hdev->queues[IFACEQ_CMD_IDX];
	hdr = queue->qhdr;
	hdr->start_addr = queue->qarray.da;
	hdr->type |= HFI_Q_ID_HOST_TO_CTRL_CMD_Q;

	queue = &hdev->queues[IFACEQ_MSG_IDX];
	hdr = queue->qhdr;
	hdr->start_addr = queue->qarray.da;
	hdr->type |= HFI_Q_ID_CTRL_TO_HOST_MSG_Q;

	queue = &hdev->queues[IFACEQ_DBG_IDX];
	hdr = queue->qhdr;
	hdr->start_addr = queue->qarray.da;
	hdr->type |= HFI_Q_ID_CTRL_TO_HOST_DEBUG_Q;

	/*
	 * Set receive request to zero on debug queue as there is no
	 * need of interrupt from video hardware for debug messages
	 */
	hdr->rx_req = 0;

	sfr = hdev->sfr.kva;
	sfr->buf_size = ALIGNED_SFR_SIZE;

	return 0;
}

static int venus_sys_set_debug(struct venus_hfi_device *hdev, u32 debug)
{
	struct hfi_sys_set_property_pkt *pkt;
	u8 packet[VIDC_IFACEQ_VAR_SMALL_PKT_SIZE];
	int ret;

	pkt = (struct hfi_sys_set_property_pkt *) &packet;

	ret = call_hfi_pkt_op(hdev, sys_debug_config, pkt, debug);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, pkt);
	if (ret)
		return ret;

	return 0;
}

static int venus_sys_set_coverage(struct venus_hfi_device *hdev, u32 mode)
{
	struct hfi_sys_set_property_pkt *pkt;
	u8 packet[VIDC_IFACEQ_VAR_SMALL_PKT_SIZE];
	int ret;

	pkt = (struct hfi_sys_set_property_pkt *) packet;

	ret = call_hfi_pkt_op(hdev, sys_coverage_config, pkt, mode);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, pkt);
	if (ret)
		return ret;

	return 0;
}

static int venus_sys_set_idle_message(struct venus_hfi_device *hdev,
				      bool enable)
{
	struct hfi_sys_set_property_pkt *pkt;
	u8 packet[VIDC_IFACEQ_VAR_SMALL_PKT_SIZE];
	int ret;

	if (!enable)
		return 0;

	pkt = (struct hfi_sys_set_property_pkt *) packet;

	ret = call_hfi_pkt_op(hdev, sys_idle_indicator, pkt, enable);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, pkt);
	if (ret)
		return ret;

	return 0;
}

static int venus_sys_set_power_control(struct venus_hfi_device *hdev,
				       bool enable)
{
	struct hfi_sys_set_property_pkt *pkt;
	u8 packet[VIDC_IFACEQ_VAR_SMALL_PKT_SIZE];
	bool supported = false;
	int ret;

	if (!supported)
		return 0;

	pkt = (struct hfi_sys_set_property_pkt *) packet;

	ret = call_hfi_pkt_op(hdev, sys_power_control, pkt, enable);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, pkt);
	if (ret)
		return ret;

	return 0;
}

static int venus_get_queue_size(struct venus_hfi_device *hdev,
				unsigned int index)
{
	struct hfi_queue_header *qhdr;

	if (index >= IFACEQ_NUM) {
		dprintk(VIDC_ERR, "Invalid index: %d\n", index);
		return -EINVAL;
	}

	qhdr = hdev->queues[index].qhdr;
	if (!qhdr) {
		dprintk(VIDC_ERR, "queue not present\n");
		return -EINVAL;
	}

	return qhdr->read_idx - qhdr->write_idx;
}

static void venus_set_default_sys_properties(struct venus_hfi_device *hdev)
{
	if (venus_sys_set_debug(hdev, vidc_fw_debug))
		dprintk(VIDC_WARN, "Setting fw_debug msg ON failed\n");

	if (venus_sys_set_idle_message(hdev,
		hdev->res->sys_idle_indicator || vidc_sys_idle_indicator))
		dprintk(VIDC_WARN, "Setting idle response ON failed\n");

	if (venus_sys_set_power_control(hdev, vidc_fw_low_power_mode))
		dprintk(VIDC_WARN, "Setting h/w power collapse ON failed\n");
}

static int venus_send_session_cmd(void *session, int pkt_type)
{
	struct vidc_hal_session_cmd_pkt pkt;
	struct hal_session *sess = session;
	struct venus_hfi_device *hdev = sess->device;
	int ret;

	ret = call_hfi_pkt_op(hdev, session_cmd, &pkt, pkt_type, session);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, &pkt);
	if (ret)
		return ret;

	return 0;
}

static int venus_prepare_power_collapse(struct venus_hfi_device *hdev)
{
	unsigned long timeout = msecs_to_jiffies(vidc_hw_rsp_timeout);
	struct hfi_sys_pc_prep_pkt pkt;
	int ret;

	init_completion(&pc_prep_done);

	ret = call_hfi_pkt_op(hdev, sys_pc_prep, &pkt);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, &pkt);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&pc_prep_done, timeout);
	if (!ret) {
		venus_flush_debug_queue(hdev, NULL);
		return -ETIMEDOUT;
	}

	return 0;
}

static void venus_pm_handler(struct work_struct *work)
{
	struct venus_hfi_device *hdev;
	u32 ctrl_status = 0;
	int ret;

	hdev = list_first_entry(&hal_ctxt.dev_head, struct venus_hfi_device,
				list);
	if (!hdev) {
		dprintk(VIDC_ERR, "%s: NULL device\n", __func__);
		return;
	}

	if (!hdev->power_enabled) {
		dprintk(VIDC_DBG, "%s: Power already disabled\n", __func__);
		return;
	}

	mutex_lock(&hdev->lock);
	ret = venus_is_valid_state(hdev);
	mutex_unlock(&hdev->lock);

	if (!ret) {
		dprintk(VIDC_WARN,
			"Core is in bad state, Skipping power collapse\n");
		return;
	}

	dprintk(VIDC_DBG, "Prepare for power collapse\n");

	ret = venus_prepare_power_collapse(hdev);
	if (ret) {
		dprintk(VIDC_ERR, "Failed to prepare for PC %d\n", ret);
		goto err_prepare_pc;
	}

	mutex_lock(&hdev->lock);

	if (hdev->last_packet_type != HFI_CMD_SYS_PC_PREP) {
		dprintk(VIDC_DBG,
			"Last command (%#x) is not PC_PREP cmd\n",
			hdev->last_packet_type);
		goto skip_power_off;
	}

	if (venus_get_queue_size(hdev, IFACEQ_MSG_IDX) ||
	    venus_get_queue_size(hdev, IFACEQ_CMD_IDX)) {
		dprintk(VIDC_DBG, "Cmd/msg queues are not empty\n");
		goto skip_power_off;
	}

	ctrl_status = venus_readl(hdev, VIDC_CPU_CS_SCIACMDARG0);
	if (!(ctrl_status & VIDC_CPU_CS_SCIACMDARG0_HFI_CTRL_PC_READY)) {
		dprintk(VIDC_DBG,
			"Venus is not ready for power collapse (%#x)\n",
			ctrl_status);
		goto skip_power_off;
	}

	ret = venus_power_off(hdev);
	if (ret) {
		dprintk(VIDC_ERR, "Failed venus power off\n");
		goto err_power_off;
	}

	/* Cancel pending delayed works if any */
	cancel_delayed_work(&venus_hfi_pm_work);

	mutex_unlock(&hdev->lock);

	return;

err_power_off:
skip_power_off:

	/*
	* When power collapse is escaped, driver no need to inform Venus.
	* Venus is self-sufficient to come out of the power collapse at
	* any stage. Driver can skip power collapse and continue with
	* normal execution.
	*/

	/* Cancel pending delayed works if any */
	cancel_delayed_work(&venus_hfi_pm_work);
	dprintk(VIDC_WARN, "Power off skipped (last pkt %#x, status: %#x)\n",
		hdev->last_packet_type, ctrl_status);

	mutex_unlock(&hdev->lock);
err_prepare_pc:
	return;
}

static void venus_sfr_print(struct venus_hfi_device *hdev)
{
	struct hfi_sfr *sfr = hdev->sfr.kva;
	void *p;

	if (!sfr)
		return;

	p = memchr(sfr->rg_data, '\0', sfr->buf_size);
	/*
	 * SFR isn't guaranteed to be NULL terminated since SYS_ERROR indicates
	 * that Venus is in the process of crashing.
	 */
	if (p == NULL)
		sfr->rg_data[sfr->buf_size - 1] = '\0';

	dprintk(VIDC_ERR, "SFR Message from FW: %s\n", sfr->rg_data);
}

static void venus_process_msg_sys_error(struct venus_hfi_device *hdev,
					void *packet)
{
	struct hfi_msg_event_notify_pkt *event_pkt = packet;

	if (event_pkt->event_id != HFI_EVENT_SYS_ERROR)
		return;

	venus_set_state(hdev, VENUS_STATE_DEINIT);

	/*
	 * Once SYS_ERROR received from HW, it is safe to halt the AXI.
	 * With SYS_ERROR, Venus FW may have crashed and HW might be
	 * active and causing unnecessary transactions. Hence it is
	 * safe to stop all AXI transactions from venus subsystem.
	 */
	venus_halt_axi(hdev);
	venus_sfr_print(hdev);
}

static void venus_flush_debug_queue(struct venus_hfi_device *hdev, void *packet)
{
	bool local_packet = false;

	if (!hdev)
		return;

	if (!packet) {
		packet = kzalloc(VIDC_IFACEQ_VAR_HUGE_PKT_SIZE, GFP_TEMPORARY);
		if (!packet) {
			dprintk(VIDC_ERR, "In %s() Fail to allocate mem\n",
				__func__);
			return;
		}
		local_packet = true;
	}

	while (!venus_iface_dbgq_read(hdev, packet)) {
		struct hfi_msg_sys_coverage_pkt *pkt = packet;

		if (pkt->packet_type == HFI_MSG_SYS_COV) {
#ifdef CONFIG_MSM_VIDC_COV
			int stm_size = 0;
			dprintk(VIDC_DBG, "DbgQ pkt size: %d\n", pkt->msg_size);
			stm_size = stm_log_inv_ts(0, 0, pkt->rg_msg_data,
						  pkt->msg_size);
			if (stm_size == 0)
				dprintk(VIDC_ERR,
					"In %s, stm_log returned size of 0\n",
					__func__);
#endif
		} else {
			struct hfi_msg_sys_debug_pkt *pkt = packet;
			dprintk(VIDC_FW, "%s", pkt->rg_msg_data);
		}
	}

	if (local_packet)
		kfree(packet);
}

static void venus_response_handler(struct venus_hfi_device *hdev)
{
	void *pkt;
	u32 ret;

	if (!hdev) {
		dprintk(VIDC_ERR, "SPURIOUS_INTERRUPT\n");
		return;
	}

	pkt = hdev->pkt_buf;

	if (hdev->intr_status & VIDC_WRAPPER_INTR_CLEAR_A2HWD_BMSK) {
		dprintk(VIDC_ERR, "received: watchdog timeout\n");
		venus_sfr_print(hdev);
		disable_irq_nosync(hdev->hal_data->irq);
		hfi_process_watchdog_timeout(hdev->device_id);
	}

	while (!venus_iface_msgq_read(hdev, pkt)) {
		ret = hfi_process_msg_packet(hdev->device_id, pkt,
					     &hdev->sessions,
					     &hdev->session_lock);
		switch (ret) {
		case HFI_MSG_EVENT_NOTIFY:
			venus_process_msg_sys_error(hdev, pkt);
			break;
		case HFI_MSG_SYS_RELEASE_RESOURCE:
			dprintk(VIDC_DBG, "received: HFI_MSG_SYS_RELEASE_RESOURCE\n");
			complete(&release_resources_done);
			break;
		case HFI_MSG_SYS_PC_PREP_DONE:
			dprintk(VIDC_DBG, "received: HFI_MSG_SYS_PC_PREP_DONE\n");
			complete(&pc_prep_done);
			break;
		default:
			break;
		}
	}

	venus_flush_debug_queue(hdev, pkt);
}

static irqreturn_t venus_isr_thread(int irq, void *dev)
{
	struct venus_hfi_device *hdev = dev;
	int ret;

	ret = venus_hfi_resume(hdev);
	if (ret) {
		dprintk(VIDC_ERR, "%s: Power enable failed\n", __func__);
		return IRQ_NONE;
	}

	if (hdev->res->sw_power_collapsible) {
		dprintk(VIDC_DBG, "Cancel and queue delayed work again.\n");
		cancel_delayed_work(&venus_hfi_pm_work);
		if (!queue_delayed_work(hdev->venus_pm_workq,
					&venus_hfi_pm_work,
			msecs_to_jiffies(vidc_pwr_collapse_delay))) {
			dprintk(VIDC_DBG, "PM work already scheduled\n");
		}
	}

	venus_response_handler(hdev);

	return IRQ_HANDLED;
}

static irqreturn_t venus_isr(int irq, void *dev)
{
	struct venus_hfi_device *hdev = dev;
	u32 status;

	status = venus_readl(hdev, VIDC_WRAPPER_INTR_STATUS);

	if (status & VIDC_WRAPPER_INTR_STATUS_A2H_BMSK ||
	    status & VIDC_WRAPPER_INTR_STATUS_A2HWD_BMSK ||
	    status & VIDC_CPU_CS_SCIACMDARG0_HFI_CTRL_INIT_IDLE_MSG_BMSK) {
		hdev->intr_status |= status;
		hdev->reg_count++;
	} else {
		hdev->spur_count++;
	}

	venus_writel(hdev, VIDC_CPU_CS_A2HSOFTINTCLR, 1);
	venus_writel(hdev, VIDC_WRAPPER_INTR_CLEAR, status);

	return IRQ_WAKE_THREAD;
}

static int venus_check_core_registered(struct hal_device_data *core,
				       void __iomem *reg_base, int irq)
{
	struct venus_hfi_device *hdev;
	struct list_head *curr, *next;

	if (!core->dev_count)
		return -EINVAL;

	list_for_each_safe(curr, next, &core->dev_head) {
		hdev = list_entry(curr, struct venus_hfi_device, list);
		if (hdev && hdev->hal_data->irq == irq &&
		    hdev->hal_data->base == reg_base)
			return 0;
	}

	dprintk(VIDC_INFO, "Device not registered\n");

	return -ENODEV;
}

static int venus_request_interrupt(struct venus_hfi_device *hdev,
				   struct vidc_resources *res)
{
	struct hal_data *hal;
	int ret;

	if (res->irq < 0 || !res->base)
		return -EINVAL;

	ret = venus_check_core_registered(&hal_ctxt, res->base, res->irq);
	if (!ret) {
		dprintk(VIDC_ERR, "Core present/Already added\n");
		return -EEXIST;
	}

	hal = kzalloc(sizeof(struct hal_data), GFP_KERNEL);
	if (!hal) {
		dprintk(VIDC_ERR, "Failed to alloc\n");
		return -ENOMEM;
	}

	hdev->hal_data = hal;
	hal->irq = res->irq;
	hal->base = res->base;

	ret = request_threaded_irq(res->irq, venus_isr,
				   venus_isr_thread,
				   IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				   "vidc", hdev);
	if (ret) {
		dprintk(VIDC_ERR, "request_irq failed\n");
		goto err;
	}

	return 0;

err:
	kfree(hal);
	return ret;

}

static int venus_protect_cp_mem(struct venus_hfi_device *hdev)
{
	u32 cp_nonpixel_start = 0, cp_nonpixel_size = 0;
	u32 cp_start = 0, cp_size = 0;
	struct context_bank_info *cb;
	int ret;

	if (!hdev)
		return -EINVAL;

	list_for_each_entry(cb, &hdev->res->context_banks, list) {
		if (!strcmp(cb->name, "venus_ns")) {
			cp_size = cb->addr_range.start;
			dprintk(VIDC_DBG, "%s memprot.cp_size: %#x\n",
				__func__, cp_size);
		}

		if (!strcmp(cb->name, "venus_sec_non_pixel")) {
			cp_nonpixel_start = cb->addr_range.start;
			cp_nonpixel_size = cb->addr_range.size;
			dprintk(VIDC_DBG,
				"%s memprot.cp_start: %#x size: %#x\n",
				__func__, cp_nonpixel_start,
				cp_nonpixel_size);
		}
	}

	ret = qcom_scm_mem_protect_video_var(cp_start, cp_size,
					     cp_nonpixel_start,
					     cp_nonpixel_size);
	if (ret) {
		dprintk(VIDC_ERR, "Failed to protect memory (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int venus_hfi_core_init(void *device)
{
	struct venus_hfi_device *hdev = device;
	struct hfi_sys_get_property_pkt version_pkt;
	struct hfi_sys_init_pkt pkt;
	struct list_head *ptr, *next;
	struct hal_session *session = NULL;
	int ret;

	hdev->intr_status = 0;
	hdev->power_enabled = true;

	mutex_lock(&hdev->session_lock);
	list_for_each_safe(ptr, next, &hdev->sessions) {
		/* This means that session list is not empty. Kick stale
		 * sessions out of our valid instance list, but keep the
		 * list_head inited so that list_del (in the future, called
		 * by session_clean()) will be valid. When client doesn't close
		 * them, then it is a genuine leak which driver can't fix. */
		session = list_entry(ptr, struct hal_session, list);
		list_del_init(&session->list);
	}
	INIT_LIST_HEAD(&hdev->sessions);
	mutex_unlock(&hdev->session_lock);

	if (hdev->mem_client) {
		dprintk(VIDC_ERR, "memory client exists\n");
		return -EINVAL;
	}

	hdev->mem_client = smem_new_client(hdev->res);
	if (IS_ERR(hdev->mem_client))
		return PTR_ERR(hdev->mem_client);

	ret = venus_interface_queues_init(hdev);
	if (ret) {
		dprintk(VIDC_ERR, "failed to init queues\n");
		return ret;
	}

	ret = venus_run(hdev);
	if (ret) {
		dprintk(VIDC_ERR, "run venus core failed (%d)\n", ret);
		return ret;
	}

	ret = call_hfi_pkt_op(hdev, sys_init, &pkt, HFI_VIDEO_ARCH_OX);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, &pkt);
	if (ret)
		return ret;

	ret = call_hfi_pkt_op(hdev, sys_image_version, &version_pkt);
	if (ret || venus_iface_cmdq_write(hdev, &version_pkt))
		dprintk(VIDC_WARN, "Failed to send image version pkt to fw\n");

	ret = venus_protect_cp_mem(hdev);
	if (ret)
		return ret;

	venus_set_state(hdev, VENUS_STATE_INIT);

	return 0;
}

static int venus_hfi_core_release(void *device)
{
	struct venus_hfi_device *hdev = device;
	int ret;

	if (hdev->mem_client) {
		ret = venus_hfi_resume(device);
		if (ret)
			return ret;

		hdev->intr_status = 0;
	}

	venus_set_state(hdev, VENUS_STATE_DEINIT);

	cancel_delayed_work(&venus_hfi_pm_work);
	flush_workqueue(hdev->venus_pm_workq);

	venus_interface_queues_release(hdev);

	/*
	 * Halt the AXI to make sure there are no pending transactions.
	 * Clocks should be unprepared after making sure axi is halted.
	 */
	venus_halt_axi(hdev);

	hdev->power_enabled = false;

	return 0;
}

static int venus_hfi_core_ping(void *device)
{
	struct venus_hfi_device *hdev = device;
	struct hfi_cmd_sys_ping_packet pkt;
	int ret;

	ret = call_hfi_pkt_op(hdev, sys_ping, &pkt);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, &pkt);
	if (ret)
		return ret;

	return 0;
}

static int venus_hfi_core_trigger_ssr(void *device,
				      enum hal_ssr_trigger_type type)
{
	struct hfi_sys_test_ssr_pkt pkt;
	struct venus_hfi_device *hdev = device;
	int ret;

	ret = call_hfi_pkt_op(hdev, ssr_cmd, type, &pkt);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, &pkt);
	if (ret)
		return ret;

	return 0;
}

static void *venus_hfi_session_init(void *device, void *session_id,
				    enum hal_domain session_type,
				    enum hal_video_codec codec_type)
{
	struct venus_hfi_device *hdev = device;
	struct hfi_session_init_pkt pkt;
	struct hal_session *new_session;
	int ret;

	new_session = kzalloc(sizeof(*new_session), GFP_KERNEL);
	if (!new_session)
		return ERR_PTR(-ENOMEM);

	new_session->session_id = session_id;
	new_session->is_decoder = session_type == HAL_VIDEO_DOMAIN_DECODER;
	new_session->device = hdev;

	mutex_lock(&hdev->session_lock);
	list_add_tail(&new_session->list, &hdev->sessions);
	mutex_unlock(&hdev->session_lock);

	venus_set_default_sys_properties(device);

	ret = call_hfi_pkt_op(hdev, session_init, &pkt, new_session,
			      session_type, codec_type);
	if (ret)
		goto err_session_init_fail;

	ret = venus_iface_cmdq_write(hdev, &pkt);
	if (ret)
		goto err_session_init_fail;

	return new_session;

err_session_init_fail:
	venus_hfi_session_clean(new_session);
	return ERR_PTR(ret);
}

static int venus_hfi_session_end(void *session)
{
	struct hal_session *sess = session;
	struct venus_hfi_device *hdev = sess->device;

	if (vidc_fw_coverage) {
		if (venus_sys_set_coverage(hdev, vidc_fw_coverage))
			dprintk(VIDC_WARN, "Fw_coverage msg ON failed\n");
	}

	return venus_send_session_cmd(session, HFI_CMD_SYS_SESSION_END);
}

static int venus_hfi_session_abort(void *session)
{
	struct hal_session *sess = session;
	struct venus_hfi_device *hdev = sess->device;

	venus_flush_debug_queue(hdev, NULL);

	return venus_send_session_cmd(session, HFI_CMD_SYS_SESSION_ABORT);
}

static int venus_hfi_session_clean(void *session)
{
	struct hal_session *sess = session;
	struct venus_hfi_device *hdev = sess->device;

	venus_flush_debug_queue(hdev, NULL);

	mutex_lock(&hdev->session_lock);
	list_del(&sess->list);
	kfree(sess);
	mutex_unlock(&hdev->session_lock);

	return 0;
}

static int venus_hfi_session_flush(void *sess, enum hal_flush flush_mode)
{
	struct hal_session *session = sess;
	struct venus_hfi_device *hdev = session->device;
	struct hfi_cmd_session_flush_packet pkt;
	int ret;

	ret = call_hfi_pkt_op(hdev, session_flush, &pkt, session, flush_mode);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, &pkt);
	if (ret)
		return ret;

	return 0;
}

static int venus_hfi_session_start(void *sess)
{
	return venus_send_session_cmd(sess, HFI_CMD_SESSION_START);
}

static int venus_hfi_session_stop(void *sess)
{
	return venus_send_session_cmd(sess, HFI_CMD_SESSION_STOP);
}

static int venus_hfi_session_etb(void *sess, struct vidc_frame_data *in_frame)
{
	struct hal_session *session = sess;
	struct venus_hfi_device *hdev = session->device;
	int ret;

	if (session->is_decoder) {
		struct hfi_cmd_session_empty_buffer_compressed_packet pkt;

		ret = call_hfi_pkt_op(hdev, session_etb_decoder,
				      &pkt, session, in_frame);
		if (ret)
			return ret;

		ret = venus_iface_cmdq_write(hdev, &pkt);
	} else {
		struct hfi_cmd_session_empty_buffer_uncompressed_plane0_packet
			pkt;

		ret = call_hfi_pkt_op(hdev, session_etb_encoder,
				      &pkt, session, in_frame);
		if (ret)
			return ret;

		ret = venus_iface_cmdq_write(hdev, &pkt);
	}

	if (ret)
		return ret;

	return 0;
}

static int venus_hfi_session_ftb(void *sess, struct vidc_frame_data *out_frame)
{
	struct hal_session *session = sess;
	struct venus_hfi_device *hdev = session->device;
	struct hfi_cmd_session_fill_buffer_packet pkt;
	int ret;

	ret = call_hfi_pkt_op(hdev, session_ftb, &pkt, session, out_frame);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, &pkt);
	if (ret)
		return ret;

	return 0;
}

static int
venus_hfi_session_set_buffers(void *sess, struct vidc_buffer_addr_info *bai)
{
	struct hal_session *session = sess;
	struct venus_hfi_device *hdev = session->device;
	struct hfi_session_set_buffers_pkt *pkt;
	u8 packet[VIDC_IFACEQ_VAR_LARGE_PKT_SIZE];
	int ret;

	if (bai->buffer_type == HAL_BUFFER_INPUT)
		return 0;

	pkt = (struct hfi_session_set_buffers_pkt *)packet;

	ret = call_hfi_pkt_op(hdev, session_set_buffers, pkt, session, bai);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, pkt);
	if (ret)
		return ret;

	return 0;
}

static int venus_hfi_session_release_buffers(void *sess,
					     struct vidc_buffer_addr_info *bai)
{
	struct hal_session *session = sess;
	struct venus_hfi_device *hdev = session->device;
	struct hfi_cmd_session_release_buffer_packet *pkt;
	u8 packet[VIDC_IFACEQ_VAR_LARGE_PKT_SIZE];
	int ret;

	if (bai->buffer_type == HAL_BUFFER_INPUT)
		return 0;

	pkt = (struct hfi_cmd_session_release_buffer_packet *) packet;

	ret = call_hfi_pkt_op(hdev, session_release_buffers, pkt, session, bai);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, pkt);
	if (ret)
		return ret;

	return 0;
}

static int venus_hfi_session_load_res(void *sess)
{
	return venus_send_session_cmd(sess, HFI_CMD_SESSION_LOAD_RESOURCES);
}

static int venus_hfi_session_release_res(void *sess)
{
	return venus_send_session_cmd(sess, HFI_CMD_SESSION_RELEASE_RESOURCES);
}

static int venus_hfi_session_parse_seq_hdr(void *sess,
					   struct vidc_seq_hdr *seq_hdr)
{
	struct hal_session *session = sess;
	struct venus_hfi_device *hdev = session->device;
	struct hfi_cmd_session_parse_sequence_header_packet *pkt;
	u8 packet[VIDC_IFACEQ_VAR_SMALL_PKT_SIZE];
	int ret;

	pkt = (struct hfi_cmd_session_parse_sequence_header_packet *) packet;

	ret = call_hfi_pkt_op(hdev, session_parse_seq_header,
			      pkt, session, seq_hdr);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, pkt);
	if (ret)
		return ret;

	return 0;
}

static int
venus_hfi_session_get_seq_hdr(void *sess, struct vidc_seq_hdr *seq_hdr)
{
	struct hal_session *session = sess;
	struct venus_hfi_device *hdev = session->device;
	struct hfi_session_get_sequence_header_pkt *pkt;
	u8 packet[VIDC_IFACEQ_VAR_SMALL_PKT_SIZE];
	int ret;

	pkt = (struct hfi_session_get_sequence_header_pkt *) packet;

	ret = call_hfi_pkt_op(hdev, session_get_seq_hdr, pkt, session, seq_hdr);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, pkt);
	if (ret)
		return ret;;

	return 0;
}

static int venus_hfi_session_set_property(void *sess, enum hal_property ptype,
					  void *pdata)
{
	struct hal_session *session = sess;
	struct venus_hfi_device *hdev = session->device;
	struct hfi_session_set_property_pkt *pkt;
	u8 packet[VIDC_IFACEQ_VAR_LARGE_PKT_SIZE];
	int ret;

	pkt = (struct hfi_session_set_property_pkt *) packet;

	ret = call_hfi_pkt_op(hdev, session_set_property, pkt, session, ptype,
			      pdata);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, pkt);
	if (ret)
		return ret;

	return 0;
}

static int venus_hfi_session_get_property(void *sess, enum hal_property ptype)
{
	struct hal_session *session = sess;
	struct venus_hfi_device *hdev = session->device;
	struct hfi_cmd_session_get_property_packet pkt = {0};
	int ret;

	ret = call_hfi_pkt_op(hdev, session_get_property, &pkt, session, ptype);
	if (ret)
		return ret;

	ret = venus_iface_cmdq_write(hdev, &pkt);
	if (ret)
		return ret;

	return 0;
}

static int venus_hfi_get_stride_scanline(int color_fmt, int width, int height,
					 int *stride, int *scanlines)
{
	if (stride)
		*stride = VENUS_Y_STRIDE(color_fmt, width);
	if (scanlines)
		*scanlines = VENUS_Y_SCANLINES(color_fmt, height);

	return 0;
}

static int venus_hfi_get_core_capabilities(void)
{
	return HAL_VIDEO_ENCODER_ROTATION_CAPABILITY |
	       HAL_VIDEO_ENCODER_SCALING_CAPABILITY |
	       HAL_VIDEO_ENCODER_DEINTERLACE_CAPABILITY |
	       HAL_VIDEO_DECODER_MULTI_STREAM_CAPABILITY;
}

static int venus_hfi_resume(void *dev)
{
	struct venus_hfi_device *hdev = dev;
	int ret;

	mutex_lock(&hdev->lock);
	ret = venus_power_on(hdev);
	mutex_unlock(&hdev->lock);

	return ret;
}

static int venus_hfi_suspend(void *dev)
{
	struct venus_hfi_device *hdev = dev;
	int ret;

	if (hdev->power_enabled) {
		ret = flush_delayed_work(&venus_hfi_pm_work);
		dprintk(VIDC_INFO, "%s flush delayed work %d\n", __func__, ret);
	}

	return 0;
}

static enum hal_default_properties venus_hfi_get_default_properties(void *dev)
{
	enum hal_default_properties prop = 0;
	struct venus_hfi_device *hdev = dev;

	if (hdev->packetization_type == HFI_PACKETIZATION_3XX)
		prop = HAL_VIDEO_DYNAMIC_BUF_MODE;

	return prop;
}

static const struct hfi_ops venus_hfi_ops = {
	.core_init			= venus_hfi_core_init,
	.core_release			= venus_hfi_core_release,
	.core_ping			= venus_hfi_core_ping,
	.core_trigger_ssr		= venus_hfi_core_trigger_ssr,

	.session_init			= venus_hfi_session_init,
	.session_end			= venus_hfi_session_end,
	.session_abort			= venus_hfi_session_abort,
	.session_clean			= venus_hfi_session_clean,
	.session_flush			= venus_hfi_session_flush,
	.session_start			= venus_hfi_session_start,
	.session_stop			= venus_hfi_session_stop,
	.session_etb			= venus_hfi_session_etb,
	.session_ftb			= venus_hfi_session_ftb,
	.session_set_buffers		= venus_hfi_session_set_buffers,
	.session_release_buffers	= venus_hfi_session_release_buffers,
	.session_load_res		= venus_hfi_session_load_res,
	.session_release_res		= venus_hfi_session_release_res,
	.session_parse_seq_hdr		= venus_hfi_session_parse_seq_hdr,
	.session_get_seq_hdr		= venus_hfi_session_get_seq_hdr,
	.session_set_property		= venus_hfi_session_set_property,
	.session_get_property		= venus_hfi_session_get_property,

	.get_stride_scanline		= venus_hfi_get_stride_scanline,
	.get_core_capabilities		= venus_hfi_get_core_capabilities,
	.resume				= venus_hfi_resume,
	.suspend			= venus_hfi_suspend,
	.get_default_properties		= venus_hfi_get_default_properties,
};

void venus_hfi_deinitialize(struct hfi_device *hfidev)
{
	struct venus_hfi_device *close, *tmp, *dev;

	dev = hfidev->hfi_device_data;

	list_for_each_entry_safe(close, tmp, &hal_ctxt.dev_head, list) {
		if (close->hal_data->irq != dev->hal_data->irq)
			continue;

		hal_ctxt.dev_count--;
		list_del(&close->list);
		destroy_workqueue(close->venus_pm_workq);
		free_irq(dev->hal_data->irq, close);
		kfree(close->hal_data);
		kfree(close);
		break;
	}

	kfree(hfidev);
}

static int venus_hfi_initialize_packetization(struct venus_hfi_device *hdev)
{
	const char *hfi_version;

	hfi_version = hdev->res->hfi_version;

	if (!hfi_version) {
		hdev->packetization_type = HFI_PACKETIZATION_LEGACY;
	} else if (!strcmp(hfi_version, "3xx")) {
		hdev->packetization_type = HFI_PACKETIZATION_3XX;
	} else {
		dprintk(VIDC_ERR, "Unsupported hfi version\n");
		return -EINVAL;
	}

	hdev->pkt_ops = hfi_get_pkt_ops(hdev->packetization_type);
	if (!hdev->pkt_ops) {
		dprintk(VIDC_ERR, "Failed to get pkt_ops handle\n");
		return -EINVAL;
	}

	return 0;
}

static void *venus_hfi_add_device(u32 device_id, struct vidc_resources *res)
{
	struct venus_hfi_device *hdev;
	int ret;

	hdev = kzalloc(sizeof(*hdev), GFP_KERNEL);
	if (!hdev)
		return ERR_PTR(-ENOMEM);

	ret = venus_request_interrupt(hdev, res);
	if (ret)
		goto err_kfree;

	hdev->res = res;
	hdev->device_id = device_id;

	ret = venus_hfi_initialize_packetization(hdev);
	if (ret) {
		dprintk(VIDC_ERR, "Failed to initialize packetization\n");
		goto err_kfree;
	}

	hdev->venus_pm_workq =
		create_singlethread_workqueue("pm_workerq_venus");
	if (!hdev->venus_pm_workq) {
		dprintk(VIDC_ERR, ": create pm workq failed\n");
		ret = -ENOMEM;
		goto err_kfree;
	}

	mutex_init(&hdev->lock);
	mutex_init(&hdev->session_lock);

	if (!hal_ctxt.dev_count)
		INIT_LIST_HEAD(&hal_ctxt.dev_head);

	INIT_LIST_HEAD(&hdev->list);
	INIT_LIST_HEAD(&hdev->sessions);
	list_add_tail(&hdev->list, &hal_ctxt.dev_head);
	hal_ctxt.dev_count++;

	return hdev;

err_kfree:
	kfree(hdev);
	return ERR_PTR(ret);
}

struct hfi_device *
venus_hfi_initialize(u32 device_id, struct vidc_resources *res)
{
	struct hfi_device *hfidev;

	if (!res)
		return ERR_PTR(-EINVAL);

	hfidev = kzalloc(sizeof(*hfidev), GFP_KERNEL);
	if (!hfidev)
		return ERR_PTR(-ENOMEM);

	hfidev->hfi_device_data = venus_hfi_add_device(device_id, res);
	if (IS_ERR(hfidev->hfi_device_data))
		return ERR_CAST(hfidev->hfi_device_data);

	hfidev->ops = &venus_hfi_ops;

	return hfidev;
}
