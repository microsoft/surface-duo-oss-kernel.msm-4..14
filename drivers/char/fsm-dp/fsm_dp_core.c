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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/skbuff.h>
#include "fsm_dp.h"

#define DEFAULT_LOOPBACK_JOB_NUM 8192
#define FSM_DP_NAPI_WEIGHT 64
static struct fsm_dp_drv *fsm_dp_pdrv;
struct fsm_dp_kernel_register_db_entry fsm_dp_reg_db[FSM_DP_NUM_MSG_TYPE];

#ifdef CONFIG_FSM_DP_TEST

#define DEFAULT_TEST_RING_SIZE 2048
#define TEST_RING_MMAP_COOKIE	0x80000000

static int fsm_dp_test_init(struct fsm_dp_drv *pdrv)
{
	int ret;

	ret = fsm_dp_ring_init(&pdrv->test_ring.ring,
			       DEFAULT_TEST_RING_SIZE,
			       TEST_RING_MMAP_COOKIE);
	return ret;
}

static void fsm_dp_test_cleanup(struct fsm_dp_drv *pdrv)
{
	fsm_dp_ring_cleanup(&pdrv->test_ring.ring);
}
#else
static int fsm_dp_test_init(struct fsm_dp_drv *pdrv)
{
	return 0;
}

static void fsm_dp_test_cleanup(struct fsm_dp_drv *pdrv)
{
}
#endif

static void handle_rx_loopback(
	struct fsm_dp_drv *drv,
	struct iovec *iov,
	unsigned int num)
{
	struct fsm_dp_msghdr *msghdr;
	int ret;
	int i;
	struct fsm_dp_mempool *mempool = drv->mempool[FSM_DP_MEM_TYPE_UL];
	dma_addr_t dma_addr_array[FSM_DP_MAX_IOV_SIZE];

	for (i = 0; i < num; i++) {
		msghdr = (struct fsm_dp_msghdr *) iov[i].iov_base;
		msghdr->type = FSM_DP_MSG_TYPE_LPBK_RSP;
		fsm_dp_set_buf_state(msghdr, FSM_DP_BUF_STATE_KERNEL_XMIT_DMA);
		atomic_inc(&mempool->out_xmit);
		dma_addr_array[i] = 0;
	}
	ret = fsm_dp_tx(drv, iov, num, 0, dma_addr_array);
	if (ret) {
		FSM_DP_DEBUG("%s: failed to send response\n", __func__);
		drv->loopback.stats.rx_err++; /* update error stats */
		fsm_dp_set_buf_state(msghdr,
			FSM_DP_BUF_STATE_KERNEL_XMIT_DMA_COMP);
		atomic_dec(&mempool->out_xmit);
		goto free_rxbuf;
	}

	drv->loopback.stats.rx_cnt++;
	return;

free_rxbuf:
	for (i = 0; i < num; i++)
		fsm_dp_mempool_put_buf(mempool, iov[i].iov_base);
}

static void handle_tx_loopback(
	struct fsm_dp_drv *drv,
	struct fsm_dp_loopback_job *job)
{
	struct fsm_dp_mempool *tx_mempool;
	struct fsm_dp_mempool *mempool;
	struct fsm_dp_rxqueue *rxq;
	unsigned int offset;
	void *dst;
	unsigned int cluster, tx_cl;
	unsigned int c_offset, tx_off;
	struct fsm_dp_buf_cntrl *p;
	int err = FSM_DP_XMIT_OK;

	tx_mempool = fsm_dp_find_mempool(drv, job->data, true, &tx_cl);
	if (tx_mempool == NULL) {
		drv->loopback.stats.tx_drop++;
		FSM_DP_ERROR("%s: cannot to find source memory pool\n",
			  __func__);
		return;
	}
	tx_off = vaddr_offset(job->data,
			tx_mempool->mem.loc.cluster_kernel_addr[tx_cl]);
	tx_off = tx_off % fsm_dp_buf_true_size(&tx_mempool->mem);
	p = (struct fsm_dp_buf_cntrl *) (job->data - tx_off);
	p->state = FSM_DP_BUF_STATE_KERNEL_XMIT_DMA_COMP;

	if (!fsm_dp_rx_type_is_valid(job->dest)) {
		drv->loopback.stats.tx_err++;
		FSM_DP_ERROR("%s: invalid dest %u\n",
			  __func__, job->dest);
		err = -EINVAL;
		goto done_txbuf;
	}

	rxq = &drv->rxq[job->dest];
	if (!atomic_read(&rxq->refcnt)) {
		drv->loopback.stats.tx_drop++;
		FSM_DP_DEBUG("%s: drop packet\n", __func__);
		err = -EIO;
		goto done_txbuf;
	}

	mempool = drv->mempool[FSM_DP_MEM_TYPE_UL];
	if (mempool == NULL) {
		drv->loopback.stats.tx_err++;
		FSM_DP_ERROR("%s: UL memory is not created\n", __func__);
		err = -EIO;
		goto done_txbuf;
	}

	dst = fsm_dp_mempool_get_buf(mempool, &cluster, &c_offset);
	if (dst == NULL) {
		drv->loopback.stats.tx_err++;
		FSM_DP_ERROR("%s: failed to get buffer\n", __func__);
		err = -ENOMEM;
		goto done_txbuf;
	}
	memcpy(dst, job->data, job->length);

	offset = (cluster << FSM_DP_MEMPOOL_CLUSTER_SHIFT) + c_offset;

#ifdef FSM_DP_BUFFER_FENCING
	fsm_dp_set_buf_state(dst, FSM_DP_BUF_STATE_KERNEL_RECVCMP_MSGQ_TO_APP);
#endif
	if (fsm_dp_ring_write(&rxq->ring, offset, 0)) {
		drv->loopback.stats.tx_err++;
		FSM_DP_ERROR("%s: rx enqueue failed!\n", __func__);
		fsm_dp_mempool_put_buf(mempool, dst);
		err = -EIO;
		goto done_txbuf;
	}

	drv->loopback.stats.tx_cnt++;
	wake_up(&rxq->wq);

done_txbuf:
	atomic_dec(&tx_mempool->out_xmit);
	p->state = FSM_DP_BUF_STATE_KERNEL_XMIT_DMA_COMP;
	p->xmit_status = err;
}

static void loopback_cb(struct work_struct *work)
{
	struct fsm_dp_loopback_task *task = container_of(work,
				struct fsm_dp_loopback_task, work);
	struct fsm_dp_drv *drv = container_of(task,
				struct fsm_dp_drv, loopback);
	struct list_head q;
	struct fsm_dp_loopback_job *job;
	unsigned long flags;
	struct iovec iov[FSM_DP_MAX_IOV_SIZE];
	int num = 0;

	INIT_LIST_HEAD(&q);

	task->stats.run++;

	while (1) {
		spin_lock_irqsave(&task->lock, flags);
		list_splice_tail_init(&q, &task->free_q);
		list_splice_tail_init(&task->job_q, &q);
		spin_unlock_irqrestore(&task->lock, flags);

		if (list_empty(&q))
			break;

		list_for_each_entry(job, &q, list) {
			if (job->rx_loopback) {
				iov[num].iov_base = job->data;
				iov[num].iov_len = job->length;
				num++;
				if (num >= FSM_DP_MAX_IOV_SIZE) {
					handle_rx_loopback(drv, iov,
						num);
					num = 0;
				}
			} else
				handle_tx_loopback(drv, job);
		}
	}
	if (num)
		handle_rx_loopback(drv, iov, num);
}

static int tx_loopback(
	struct fsm_dp_drv *pdrv,
	void *data,
	unsigned int length)
{
	struct fsm_dp_loopback_task *task;
	struct fsm_dp_mempool *mempool;
	struct fsm_dp_loopback_job *job;
	struct fsm_dp_msghdr *msghdr = data;
	unsigned int dest = FSM_DP_RX_TYPE_LPBK;
	unsigned long flags;
	unsigned int cl;

	mempool = fsm_dp_find_mempool(pdrv, data, true, &cl);
	if (mempool == NULL) {
		FSM_DP_ERROR("%s: failed find memory pool\n", __func__);
		return -EINVAL;
	}

	switch (msghdr->type) {
	case FSM_DP_MSG_TYPE_L1:
		dest = FSM_DP_RX_TYPE_L1;
		break;
	case FSM_DP_MSG_TYPE_RF:
		dest = FSM_DP_RX_TYPE_RF;
		break;
	case FSM_DP_MSG_TYPE_TA:
		dest = FSM_DP_RX_TYPE_TA;
		break;
	case FSM_DP_MSG_TYPE_ORU:
		dest = FSM_DP_RX_TYPE_ORU;
		break;
	default:
		break;
	}

	task = &pdrv->loopback;
	spin_lock_irqsave(&task->lock, flags);
	job = list_first_entry_or_null(&task->free_q,
				       struct fsm_dp_loopback_job,
				       list);
	if (job) {
		list_del(&job->list);
		job->data = data;
		job->length = length;
		job->dest = dest;
		job->rx_loopback = false;
		list_add_tail(&job->list, &task->job_q);
		task->stats.tx_enque++;
	}
	spin_unlock_irqrestore(&task->lock, flags);

	if (job == NULL) {
		FSM_DP_DEBUG("%s: job queue is full!\n", __func__);
		task->stats.tx_drop++;
		return -EAGAIN;
	}
	if (queue_work_on(0, task->workq, &task->work))
		task->stats.sched++;
	return 0;
}

static int rx_loopback(
	struct fsm_dp_drv *pdrv,
	void *data,
	unsigned int length)
{
	struct fsm_dp_loopback_task *task;
	struct fsm_dp_loopback_job *job;
	unsigned long flags;

	task = &pdrv->loopback;
	spin_lock_irqsave(&task->lock, flags);
	job = list_first_entry_or_null(&task->free_q,
				       struct fsm_dp_loopback_job,
				       list);
	if (job) {
		list_del(&job->list);
		job->data = data;
		job->length = length;
		job->rx_loopback = true;
		list_add_tail(&job->list, &task->job_q);
		task->stats.rx_enque++;
	}
	spin_unlock_irqrestore(&task->lock, flags);

	if (job == NULL) {
		FSM_DP_DEBUG("%s: job queue is full!\n", __func__);
		task->stats.rx_drop++;
		return -EAGAIN;
	}
	if (queue_work_on(0, task->workq, &task->work))
		task->stats.sched++;
	return 0;
}

static int fsm_dp_loopback_init(struct fsm_dp_loopback_task *task)
{
	struct fsm_dp_loopback_job *job;
	struct workqueue_struct *wq;
	unsigned int i, allocsz;

	INIT_LIST_HEAD(&task->free_q);
	INIT_LIST_HEAD(&task->job_q);

	wq = alloc_workqueue("fsm_loopback", WQ_MEM_RECLAIM, 0);
	if (wq == NULL) {
		FSM_DP_ERROR("%s: failed to allocate workqueue\n", __func__);
		return -ENOMEM;
	}

	allocsz = DEFAULT_LOOPBACK_JOB_NUM * sizeof(struct fsm_dp_loopback_job);
	job = kzalloc(allocsz, GFP_KERNEL);
	if (IS_ERR(job)) {
		FSM_DP_ERROR("%s: failed to allocate memory\n", __func__);
		destroy_workqueue(wq);
		return -ENOMEM;
	}

	for (i = 0; i < DEFAULT_LOOPBACK_JOB_NUM; i++)
		list_add_tail(&job[i].list, &task->free_q);

	spin_lock_init(&task->lock);
	INIT_WORK(&task->work, loopback_cb);
	task->alloc_ptr = job;
	task->workq = wq;
	task->inited = true;
	return 0;
}

static void fsm_dp_loopback_cleanup(struct fsm_dp_loopback_task *task)
{
	unsigned long flags;

	if (task->inited) {
		cancel_work_sync(&task->work);
		destroy_workqueue(task->workq);
		spin_lock_irqsave(&task->lock, flags);
		INIT_LIST_HEAD(&task->free_q);
		INIT_LIST_HEAD(&task->job_q);
		spin_unlock_irqrestore(&task->lock, flags);
		kfree(task->alloc_ptr);
		task->alloc_ptr = NULL;
		task->inited = false;
	}
}

static int fsm_dp_rxqueue_init(
	struct fsm_dp_rxqueue *rxq,
	enum fsm_dp_rx_type rx_type,
	unsigned int size)
{
	unsigned int ring_size;
	int ret;

	if (!fsm_dp_rx_type_is_valid(rx_type))
		return -EINVAL;

	if (rxq->inited) {
		FSM_DP_ERROR("%s: rx queue already initialized!\n", __func__);
		return -EINVAL;
	}

	ring_size = calc_ring_size(size);
	if (!ring_size)
		return -EINVAL;

	ret = fsm_dp_ring_init(&rxq->ring, ring_size, MMAP_RX_COOKIE(rx_type));
	if (ret) {
		FSM_DP_DEBUG("%s: failed to initialize rx ring!\n", __func__);
		return ret;
	}

	init_waitqueue_head(&rxq->wq);
	rxq->type = rx_type,
	rxq->inited = true;
	atomic_set(&rxq->refcnt, 0);

	return 0;
}

static void fsm_dp_rxqueue_cleanup(struct fsm_dp_rxqueue *rxq)
{
	if (rxq->inited) {
		wake_up(&rxq->wq);
		fsm_dp_ring_cleanup(&rxq->ring);
		rxq->inited = false;
	}
}

void fsm_dp_rx(struct fsm_dp_drv *pdrv, void *addr, unsigned int length)
{
	struct fsm_dp_mempool *mempool;
	struct fsm_dp_rxqueue *rxq;
	struct fsm_dp_msghdr *msghdr;
	unsigned int offset;
	unsigned int cl;
	struct fsm_dp_kernel_register_db_entry *preg;
	ktime_t start;
	struct fsm_dp_buf_cntrl *pf;

	if (unlikely(pdrv == NULL || addr == NULL || !length)) {
		FSM_DP_ERROR("%s: invalid argument\n", __func__);
		return;
	}

	pf = addr - FSM_DP_L1_CACHE_BYTES;
	start = fsm_dp_traffic_ts_begin();

	mempool = fsm_dp_find_mempool(pdrv, addr, false, &cl);
	if (mempool == NULL) {
		FSM_DP_ERROR("%s: not UL address, addr=%p\n",
			  __func__, addr);
		return;
	}

	msghdr = (struct fsm_dp_msghdr *)addr;
	if (msghdr->length != length - sizeof(*msghdr)) {
		FSM_DP_ERROR("%s: length mismatch, payload=%u total=%u\n",
			     __func__, msghdr->length, length);
		pdrv->stats.rx_badmsg++;
		goto free_rxbuf;
	}

	preg = fsm_dp_find_reg_db_type(msghdr->type);
	if (preg && preg->pdrv && preg->rx_cb) {
		preg->rx_cb(
			mempool->mem.loc.page[cl],
			(char *) addr -
				mempool->mem.loc.cluster_kernel_addr[cl],
			(char *) addr,
			length);
		goto done;
	}
	switch (msghdr->type) {
	case FSM_DP_MSG_TYPE_LPBK_REQ:
		if (rx_loopback(pdrv, addr, length))
			goto free_rxbuf;
		goto done;
	case FSM_DP_MSG_TYPE_LPBK_RSP:
		rxq = &pdrv->rxq[FSM_DP_RX_TYPE_LPBK];
		break;
	case FSM_DP_MSG_TYPE_L1:
		rxq = &pdrv->rxq[FSM_DP_RX_TYPE_L1];
		break;
	case FSM_DP_MSG_TYPE_RF:
		rxq = &pdrv->rxq[FSM_DP_RX_TYPE_RF];
		break;
	case FSM_DP_MSG_TYPE_TA:
		rxq = &pdrv->rxq[FSM_DP_RX_TYPE_TA];
		break;
	case FSM_DP_MSG_TYPE_ORU:
		rxq = &pdrv->rxq[FSM_DP_RX_TYPE_ORU];
		break;
	default:
		FSM_DP_DEBUG("%s: unsupport msg type(%u)\n",
			     __func__, msghdr->type);
		goto free_rxbuf;
	}

	if (!atomic_read(&rxq->refcnt)) {
		FSM_DP_DEBUG("%s: rxq not active, drop message\n", __func__);
		goto free_rxbuf;
	}

#ifdef FSM_DP_BUFFER_FENCING
	fsm_dp_set_buf_state(msghdr,
			FSM_DP_BUF_STATE_KERNEL_RECVCMP_MSGQ_TO_APP);
#endif
	offset = fsm_dp_get_mem_offset(addr, &mempool->mem.loc, cl);

	fsm_dp_traffic_ts_ul_end_and_collect(&pdrv->traffic, pf, start);

	if (fsm_dp_ring_write(&rxq->ring, offset, 0)) {
		FSM_DP_ERROR("%s: failed to enqueue rx packet\n", __func__);
		goto free_rxbuf;
	}
	wake_up(&rxq->wq);
done:
	pdrv->stats.rx_cnt++;
	return;
free_rxbuf:
	pdrv->stats.rx_drop++;
	fsm_dp_mempool_put_buf(mempool, addr);
}

int fsm_dp_rx_init(struct fsm_dp_drv *pdrv)
{
	struct device_node *of_node = pdrv->dev->of_node;
	const __be32 *of_prop = NULL;
	const void *prop = NULL;
	unsigned int len = 0, type;
	int ret;
	unsigned int fsm_dp_ul_buf_size = DEFAULT_FSM_MEM_BUF_SIZE;
	unsigned int fsm_dp_ul_buf_cnt = DEFAULT_FSM_MEM_UL_BUF_CNT;

	prop = of_get_property(of_node, "qcom,ul-bufs", &len);
	if (prop && len == (sizeof(unsigned int) * 2)) {
		of_prop = prop;
		fsm_dp_ul_buf_size = be32_to_cpu(of_prop[0]);
		fsm_dp_ul_buf_cnt = be32_to_cpu(of_prop[1]);
		if (fsm_dp_ul_buf_size > FSM_DP_MAX_UL_MSG_LEN) {
			FSM_DP_ERROR("%s: UL buffer size %d defined in dts exceeds limit %d\n",
				__func__,
				fsm_dp_ul_buf_size,
				FSM_DP_MAX_UL_MSG_LEN);
			return -ENOMEM;
		}
	}

	pdrv->mempool[FSM_DP_MEM_TYPE_UL] = fsm_dp_mempool_alloc(
		pdrv,
		FSM_DP_MEM_TYPE_UL,
		fsm_dp_ul_buf_size,
		fsm_dp_ul_buf_cnt,
		false); /* no dma map yet since io dev is not ready */
	if (pdrv->mempool[FSM_DP_MEM_TYPE_UL] == NULL) {
		FSM_DP_ERROR("%s: failed to allocate UL memory pool!\n",
				  __func__);
		return -ENOMEM;
	}

	of_prop = NULL;
	prop = of_get_property(of_node, "qcom,rx-queue-size", &len);
	if (prop && len == (sizeof(unsigned int) * FSM_DP_RX_TYPE_LAST))
		of_prop = prop;
	for (type = 0; type < FSM_DP_RX_TYPE_LAST; type++) {
		ret = fsm_dp_rxqueue_init(&pdrv->rxq[type], type,
			(of_prop) ?
			be32_to_cpu(of_prop[type]) :
			DEFAULT_RX_QUEUE_SIZE);
		if (ret) {
			FSM_DP_ERROR("%s: failed to init rxqueue!\n", __func__);
			return ret;
		}
	}

	return 0;
}

static void fsm_dp_rx_cleanup(struct fsm_dp_drv *pdrv)
{
	unsigned int type;

	if (pdrv->mempool[FSM_DP_MEM_TYPE_UL])
		fsm_dp_mempool_free(pdrv->mempool[FSM_DP_MEM_TYPE_UL]);

	for (type = 0; type < FSM_DP_RX_TYPE_LAST; type++)
		fsm_dp_rxqueue_cleanup(&pdrv->rxq[type]);
}

int fsm_dp_rel_rx_buf(
	void *handle,
	unsigned char *buf
)
{
	struct fsm_dp_kernel_register_db_entry *preg =
			(struct fsm_dp_kernel_register_db_entry *) handle;
	struct fsm_dp_mempool *mempool;

	if (!preg || !preg->pdrv)
		return -EINVAL;
	mempool = preg->pdrv->mempool[FSM_DP_MEM_TYPE_UL];
	if (!mempool)
		return -EINVAL;
	fsm_dp_mempool_put_buf(mempool, buf);
	return 0;
}
EXPORT_SYMBOL(fsm_dp_rel_rx_buf);

void fsm_dp_deregister_kernel_client(
	void *handle,
	enum fsm_dp_msg_type msg_type
)
{
	struct fsm_dp_kernel_register_db_entry *preg =
			(struct fsm_dp_kernel_register_db_entry *) handle;

	if (!handle)
		return;
	if (preg->msg_type != msg_type)
		return;
	preg->pdrv = NULL;
};
EXPORT_SYMBOL(fsm_dp_deregister_kernel_client);

void *fsm_dp_register_kernel_client(
	enum fsm_dp_msg_type msg_type,
	int (*tx_cmplt_cb)(struct sk_buff *skb),
	int (*rx_cb)(struct page *p, unsigned int page_offset, char *buf,
				unsigned int length)
)
{
	struct fsm_dp_kernel_register_db_entry *preg;

	if (!fsm_dp_pdrv)
		return NULL;
	preg = fsm_dp_find_reg_db_type(msg_type);
	if (!preg)
		return NULL;
	if (preg->pdrv) /* already register ? */
		return NULL;
	preg->msg_type = msg_type;
	preg->pdrv = fsm_dp_pdrv;
	preg->tx_cmplt_cb = tx_cmplt_cb;
	preg->rx_cb = rx_cb;
	return preg;
}
EXPORT_SYMBOL(fsm_dp_register_kernel_client);

/*
 * fsm_dp_tx_skb
 *     Tx skb to device. skb its data is pointing to fsm dp packet payload.
 *
 *     skb can have frag list. And each skb can be non-linear.
 *     The leading skb should have enough head space to accommodate
 *     fsm_dp_msghdr.
 */
int fsm_dp_tx_skb(
	void *handle,
	struct sk_buff *skb
)
{
	struct fsm_dp_kernel_register_db_entry *preg =
			(struct fsm_dp_kernel_register_db_entry *) handle;
	struct fsm_dp_msghdr *msghdr;
	unsigned int plen;
	int ret = 0;
	struct sk_buff *iter;

	if (!preg || !preg->pdrv || !skb)
		return -EINVAL;
	if (skb_headroom(skb) < sizeof(*msghdr))
		return -ENOMEM;
	if (!fsm_dp_mhi_is_ready(&preg->pdrv->mhi))
		return -EIO;
	plen = skb->len;
	if (skb_has_frag_list(skb))
		skb_walk_frags(skb, iter)
			plen += iter->len;
	skb_push(skb, sizeof(*msghdr));
	msghdr = (struct fsm_dp_msghdr *)skb->data;
	msghdr->type = preg->msg_type;
	msghdr->reserved = 0;
	msghdr->aggr = 0;
	msghdr->version = FSM_DP_MSG_HDR_VERSION;
	msghdr->sequence = atomic_inc_return(&preg->pdrv->tx_seqnum);
	msghdr->length = plen;

	spin_lock_bh(&preg->pdrv->mhi.tx_lock);
	ret = fsm_dp_mhi_skb_ul_xfer(&preg->pdrv->mhi, skb);
	if (ret)
		preg->pdrv->stats.tx_err++;
	else
		preg->pdrv->stats.tx_cnt++;
	spin_unlock_bh(&preg->pdrv->mhi.tx_lock);
	return ret;
}
EXPORT_SYMBOL(fsm_dp_tx_skb);

int fsm_dp_tx(
	struct fsm_dp_drv *pdrv,
	struct iovec *iov,
	unsigned int iov_nr,
	unsigned int flag,
	dma_addr_t dma_addr_array[])
{
	int ret, n;
	unsigned int num, to_send;
	int j;

	if (unlikely(!pdrv || !iov || !iov_nr))
		return -EINVAL;

	ret = 0;
	if (unlikely(flag & FSM_DP_TX_FLAG_LOOPBACK)) {
		for (n = 0; n < iov_nr; n++) {
			ret = tx_loopback(pdrv,
					  iov[n].iov_base,
					  iov[n].iov_len);
			if (ret) {
				pdrv->stats.tx_err++;
				return ret;
			}
			pdrv->stats.tx_cnt++;
		}
		return 0;
	}

	if (!fsm_dp_mhi_is_ready(&pdrv->mhi)) {
		FSM_DP_ERROR("%s: mhi is not ready!\n", __func__);
		pdrv->stats.tx_err++;
		return -EIO;
	}

	if (flag & FSM_DP_TX_FLAG_SG) {
		if (iov_nr > FSM_DP_MAX_SG_IOV_SIZE) {
			FSM_DP_ERROR("%s: sg iov size too big!\n", __func__);
			return -EINVAL;
		}
	}
	spin_lock_bh(&pdrv->mhi.tx_lock);
	to_send = 0;
	for (n = 0, to_send = iov_nr; to_send > 0; ) {
		if (to_send > FSM_DP_MAX_IOV_SIZE)
			num = FSM_DP_MAX_IOV_SIZE;
		else
			num = to_send;
		for (j = 0; j < num; j++) {
			if ((flag & FSM_DP_TX_FLAG_SG) && n != (iov_nr - 1))
				pdrv->mhi.dl_flag_array[j] = MHI_CHAIN;
			else
				pdrv->mhi.dl_flag_array[j] =  MHI_EOT;
			pdrv->mhi.dl_size_array[j] = iov[n].iov_len;
			pdrv->mhi.dl_buf_array[j] = iov[n].iov_base;
			if (dma_addr_array[n]) {
				pdrv->mhi.dl_flag_array[j] |=
					MHI_FLAGS_DMA_ADDR;
				pdrv->mhi.dl_dma_addr_array[j] =
					dma_addr_array[n];
			}
			n++;
		}
		ret = fsm_dp_mhi_n_tx(&pdrv->mhi,
				    num);
		if (ret) {
			pdrv->stats.tx_err++;
			break;
		}
		to_send -= num;
	}

	if (!(flag & FSM_DP_TX_FLAG_SG))
		pdrv->stats.tx_cnt += (iov_nr - to_send);
	else if (!to_send)
		pdrv->stats.tx_cnt++;
	spin_unlock_bh(&pdrv->mhi.tx_lock);
	return ret;
}

static int fsm_dp_core_init(struct fsm_dp_drv *pdrv)
{
	struct device *dev = pdrv->dev;
	int ret;

	mutex_init(&pdrv->mempool_lock);

	of_dma_configure(dev, dev->of_node);

	ret = fsm_dp_rx_init(pdrv);
	if (ret)
		goto exit;

	ret = fsm_dp_loopback_init(&pdrv->loopback);
	if (ret)
		goto exit;

	ret = fsm_dp_test_init(pdrv);

exit:
	of_node_put(dev->of_node);
	return ret;
}

static void fsm_dp_core_cleanup(struct fsm_dp_drv *pdrv)
{
	fsm_dp_rx_cleanup(pdrv);
	fsm_dp_loopback_cleanup(&pdrv->loopback);
	fsm_dp_test_cleanup(pdrv);
	kfree(pdrv);
}

static int fsm_dp_poll(struct napi_struct *napi, int budget)
{
	int rx_work = 0;
	struct fsm_dp_drv *pdrv;
	int ret;

	pdrv = container_of(napi, struct fsm_dp_drv, napi);
	rx_work = mhi_poll(pdrv->mhi.mhi_dev, budget);
	if (rx_work < 0) {
		rx_work = 0;
		pr_err("Error polling ret:%d\n", rx_work);
		napi_complete(napi);
		goto exit_poll;
	}

	ret = fsm_dp_mhi_rx_replenish(pdrv);
	if (ret == -ENOMEM)
		schedule_work(&pdrv->alloc_work);  /* later */
	if (rx_work < budget)
		napi_complete(napi);
	else
		pdrv->stats.rx_budget_overflow++;
exit_poll:
	return rx_work;
}

static void fsm_dp_alloc_work(struct work_struct *work)
{
	struct fsm_dp_drv *pdrv;
	const int sleep_ms =  1000;
	int retry = 60;
	int ret;

	pdrv = container_of(work, struct fsm_dp_drv, alloc_work);

	do {
		ret = fsm_dp_mhi_rx_replenish(pdrv);
		/* sleep and try again */
		if (ret == -ENOMEM) {
			msleep(sleep_ms);
			retry--;
		}
	} while (ret == -ENOMEM && retry);
}

static int fsm_dp_probe(struct platform_device *pdev)
{
	struct fsm_dp_drv *pdrv;
	int ret;

	pr_info("FSM-DP: probing FSM\n");

	pdrv = kzalloc(sizeof(*pdrv), GFP_KERNEL);
	if (IS_ERR(pdrv))
		return -ENOMEM;

	pdrv->dev = &pdev->dev;
	fsm_dp_pdrv = pdrv;

	ret = fsm_dp_core_init(pdrv);
	if (ret)
		goto cleanup;

	ret = fsm_dp_mhi_init(pdrv);
	if (ret)
		goto cleanup;

	ret = fsm_dp_cdev_init(pdrv);
	if (ret)
		goto cleanup_mhi;

	ret = fsm_dp_debugfs_init(pdrv);
	if (ret)
		goto cleanup_cdev;

	platform_set_drvdata(pdev, pdrv);

	init_dummy_netdev(&pdrv->dummy_dev);
	netif_napi_add(&pdrv->dummy_dev, &pdrv->napi, fsm_dp_poll,
						FSM_DP_NAPI_WEIGHT);
	napi_enable(&pdrv->napi);
	INIT_WORK(&pdrv->alloc_work, fsm_dp_alloc_work);

	pr_info("FSM-DP: module initialized now\n");
	return 0;

cleanup_cdev:
	fsm_dp_cdev_cleanup(pdrv);
cleanup_mhi:
	fsm_dp_mhi_cleanup(pdrv);
cleanup:
	fsm_dp_core_cleanup(pdrv);
	fsm_dp_pdrv = NULL;
	pr_err("FSM-DP: module init failed!\n");
	return ret;
}

static int fsm_dp_remove(struct platform_device *pdev)
{
	struct fsm_dp_drv *pdrv = platform_get_drvdata(pdev);

	if (pdrv) {
		flush_work(&pdrv->alloc_work);
		napi_disable(&pdrv->napi);
		netif_napi_del(&pdrv->napi);
		fsm_dp_cdev_cleanup(pdrv);
		fsm_dp_mhi_cleanup(pdrv);
		fsm_dp_debugfs_cleanup(pdrv);
		fsm_dp_core_cleanup(pdrv);
	}
	fsm_dp_pdrv = NULL;

	return 0;
}

static const struct of_device_id fsm_dp_of_table[] = {
	{ .compatible = "qcom,fsm-dp" },
	{ },
};
MODULE_DEVICE_TABLE(of, fsm_dp_of_table);

static struct platform_driver __fsm_dp_platform_drv = {
	.probe	= fsm_dp_probe,
	.remove	= fsm_dp_remove,
	.driver	= {
		.name		= KBUILD_MODNAME,
		.of_match_table	= fsm_dp_of_table,
	},
};

module_platform_driver(__fsm_dp_platform_drv);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("FSM DP driver");
