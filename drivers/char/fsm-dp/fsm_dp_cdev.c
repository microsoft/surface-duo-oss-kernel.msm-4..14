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
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/dma-mapping.h>

#include "fsm_dp.h"

static inline bool is_rxqueue_mmap_cookie(unsigned int cookie)
{
	unsigned int type, mmap_type;

	mmap_type = MMAP_COOKIE_TO_TYPE(cookie);
	type = MMAP_COOKIE_TO_MEM_TYPE(cookie);
	if (mmap_type == FSM_DP_MMAP_TYPE_RING && type >= FSM_DP_MEM_TYPE_LAST)
		return true;
	return false;
}

static inline void *usr_to_kern_vaddr(
	struct fsm_dp_mempool_vma *mempool_vma,
	void __user *addr,
	unsigned int *cluster,
	unsigned int *c_offset)
{
	struct fsm_dp_mempool *mempool = *mempool_vma->pp_mempool;
	unsigned long offset = (unsigned long)addr -
		mempool_vma->vma[FSM_DP_MMAP_TYPE_MEM]->vm_start;

	*cluster = offset >> FSM_DP_MEMPOOL_CLUSTER_SHIFT;
	*c_offset = offset & FSM_DP_MEMPOOL_CLUSTER_MASK;

	return ((char *)mempool->mem.loc.cluster_kernel_addr[*cluster] +
								*c_offset);
}

static inline struct fsm_dp_rxqueue *rxqueue_vma_to_rxqueue(
	struct fsm_dp_rxqueue_vma *rxq_vma)
{
	struct fsm_dp_cdev *cdev = container_of(rxq_vma,
						struct fsm_dp_cdev,
						rxqueue_vma[rxq_vma->type]);
	return &cdev->pdrv->rxq[rxq_vma->type];
}

static void __cdev_init_mempool_vma(struct fsm_dp_cdev *cdev)
{
	struct fsm_dp_drv *drv = cdev->pdrv;
	int type;

	memset(cdev->mempool_vma, 0, sizeof(cdev->mempool_vma));

	for (type = 0; type < FSM_DP_MEM_TYPE_LAST; type++)
		cdev->mempool_vma[type].pp_mempool = &drv->mempool[type];
}

static struct fsm_dp_mempool_vma *find_mempool_vma(
	struct fsm_dp_cdev *cdev,
	void __user *addr,
	unsigned int len)
{
	struct fsm_dp_mempool_vma *mempool_vma = NULL;
	struct vm_area_struct *vma;
	unsigned int mem_type;

	for (mem_type = 0; mem_type < FSM_DP_MEM_TYPE_LAST; mem_type++) {
		mempool_vma = &cdev->mempool_vma[mem_type];
		vma = mempool_vma->vma[FSM_DP_MMAP_TYPE_MEM];
		if (vma && vaddr_in_vma_range(addr, len, vma))
			return mempool_vma;
	}
	return NULL;
}

static int __cdev_tx(
	struct fsm_dp_cdev *cdev,
	struct iovec __user *uiov,
	unsigned int iov_nr,
	bool sg)
{
	struct fsm_dp_drv *pdrv = cdev->pdrv;
	struct fsm_dp_mempool_vma *mempool_vma;
	struct iovec iov[FSM_DP_MAX_IOV_SIZE];
	dma_addr_t dma_addr[FSM_DP_MAX_IOV_SIZE];
	unsigned int n;
	int ret;
	unsigned int flag = 0;
	struct fsm_dp_mempool *mempool;
#ifdef FSM_DP_BUFFER_FENCING
	uint32_t iov_off_array[FSM_DP_MAX_IOV_SIZE];
#endif
	unsigned int c_offset;
	unsigned int cluster;

	FSM_DP_DEBUG("%s: iov_nr=%u\n", __func__, iov_nr);
	if (iov_nr > FSM_DP_MAX_IOV_SIZE)
		return  -E2BIG;

	if (copy_from_user(iov, (void __user *)uiov,
				   sizeof(struct iovec) * iov_nr))
		return -EFAULT;

	for (n = 0; n < iov_nr; n++) {
		mempool_vma = find_mempool_vma(cdev,
					       iov[n].iov_base,
					       iov[n].iov_len);
		if (mempool_vma == NULL) {
			FSM_DP_DEBUG(
				"%s: cannot find mempool addr=%p, len=%lu\n",
				__func__, iov[n].iov_base, iov[n].iov_len);
			return -EINVAL;
		}
		mempool = *mempool_vma->pp_mempool;

		/* User passes in the pointer to message payload */
		iov[n].iov_base = usr_to_kern_vaddr(
					mempool_vma,
					iov[n].iov_base,
					&cluster,
					&c_offset);
		if (!sg || !n) {
			iov[n].iov_base = (char *)iov[n].iov_base -
				sizeof(struct fsm_dp_msghdr);
			iov[n].iov_len += sizeof(struct fsm_dp_msghdr);
			((struct fsm_dp_msghdr *)iov[n].iov_base)->sequence =
				atomic_inc_return(&pdrv->tx_seqnum);
			c_offset -= sizeof(struct fsm_dp_msghdr);
		}
#ifdef FSM_DP_BUFFER_FENCING
		{
			unsigned long b_backtrack;
			struct fsm_dp_buf_cntrl *p;
			b_backtrack = c_offset %
				fsm_dp_buf_true_size(&mempool->mem);
			iov_off_array[n] = b_backtrack;
			p = (struct fsm_dp_buf_cntrl *)
				(iov[n].iov_base - b_backtrack);
			if (p->signature != FSM_DP_BUFFER_SIG) {
				FSM_DP_ERROR("%s: mempool type %d buffer at "
					"kernel addr %p corrupted, %x, exp %x\n",
					__func__,
					(*mempool_vma->pp_mempool)->type,
					iov[n].iov_base, p->signature,
						FSM_DP_BUFFER_SIG);
				return -EINVAL;
			}
			if (p->fence != FSM_DP_BUFFER_FENCE_SIG) {
				FSM_DP_ERROR("%s: mempool type %d buffer at "
					"kernel addr %p corrupted, fence %x, "
					"exp %x\n", __func__,
					(*mempool_vma->pp_mempool)->type,
					iov[n].iov_base, p->fence,
					FSM_DP_BUFFER_FENCE_SIG);
				return -EINVAL;
			}
			p->state = FSM_DP_BUF_STATE_KERNEL_XMIT_DMA;
			p->xmit_status = FSM_DP_XMIT_IN_PROGRESS;
		}
#endif
		atomic_inc(&mempool->out_xmit);
		if (mempool->mem.loc.dma_mapped &&
				cdev->tx_mode != TX_MODE_LOOPBACK) {
			/*
			 * set to indicate iov_base is
			 * dma handle instead of
			 * kernal virtual addr
			 */
			dma_addr[n] =
				mempool->mem.loc.cluster_dma_addr[cluster] +
								c_offset;
		} else
			dma_addr[n] = 0;

		FSM_DP_DEBUG("%s: start tx, kaddr=%p len=%lu\n",
			  __func__, iov[n].iov_base, iov[n].iov_len);
	}

	if (sg)
		flag |= FSM_DP_TX_FLAG_SG;
	if (cdev->tx_mode == TX_MODE_LOOPBACK)
		flag |= FSM_DP_TX_FLAG_LOOPBACK;

	ret = fsm_dp_tx(pdrv, iov, iov_nr, flag, dma_addr);

	if (ret) {
#ifdef FSM_DP_BUFFER_FENCING
		struct fsm_dp_buf_cntrl *p;

		for (n = 0; n < iov_nr; n++) {
			p = (struct fsm_dp_buf_cntrl *)
				(iov[n].iov_base - iov_off_array[n]);
			p->state = FSM_DP_BUF_STATE_KERNEL_XMIT_DMA_COMP;
			atomic_dec(&mempool->out_xmit);
			p->xmit_status = ret;
		}
#endif
	} else
		ret = iov_nr;
	wmb(); /* make other CPU see */
	return ret;
}

/* Character device interfaces */
static int __cdev_ioctl_mempool_alloc(
	struct fsm_dp_cdev *cdev,
	unsigned long ioarg)
{
	struct fsm_dp_drv *pdrv = cdev->pdrv;
	struct fsm_dp_ioctl_mempool_alloc req;
	struct fsm_dp_mempool *mempool;

	if (copy_from_user(&req, (void __user *)ioarg, sizeof(req)))
		return -EFAULT;

	mempool = fsm_dp_mempool_alloc(pdrv, req.type, req.buf_sz, req.buf_num,
				true); /* may do dma_map */
	if (mempool == NULL) {
		FSM_DP_ERROR("%s: fsm_mem_alloc failed!\n", __func__);
		return -ENOMEM;
	}

	cdev->mempool_vma[req.type].usr_alloc = true;

	if (req.cfg) {
		struct fsm_dp_mempool_cfg cfg;

		fsm_dp_mempool_get_cfg(mempool, &cfg);
		if (copy_to_user((void __user *)req.cfg, &cfg, sizeof(cfg))) {
			FSM_DP_ERROR("%s: copy_to_user failed\n", __func__);
			return -EFAULT;
		}
	}
	return 0;
}

static int __cdev_ioctl_mempool_getcfg(
	struct fsm_dp_cdev *cdev,
	unsigned long ioarg)
{
	struct fsm_dp_drv *pdrv = cdev->pdrv;
	struct fsm_dp_ioctl_getcfg req;
	struct fsm_dp_mempool_cfg cfg;

	if (copy_from_user(&req, (void __user *)ioarg, sizeof(req)))
		return -EFAULT;

	if (!fsm_dp_mem_type_is_valid(req.type))
		return -EINVAL;

	if (fsm_dp_mempool_get_cfg(pdrv->mempool[req.type], &cfg))
		return -EAGAIN;

	if (copy_to_user((void __user *)req.cfg, &cfg, sizeof(cfg))) {
		FSM_DP_ERROR("%s: copy_to_user failed\n", __func__);
		return -EFAULT;
	}
	return 0;
}

static int __cdev_ioctl_tx(struct fsm_dp_cdev *cdev, unsigned long ioarg)
{
	struct iovec iov;
	int ret;

	if (copy_from_user(&iov, (void __user *)ioarg, sizeof(iov)))
		return -EFAULT;

	if (!iov.iov_len || iov.iov_len > FSM_DP_MAX_IOV_SIZE)
		return -EINVAL;

	ret = __cdev_tx(cdev, iov.iov_base, iov.iov_len, false);
	return ret;
}

static int __cdev_ioctl_sg_tx(struct fsm_dp_cdev *cdev, unsigned long ioarg)
{
	struct iovec iov;
	int ret;

	if (copy_from_user(&iov, (void __user *)ioarg, sizeof(iov)))
		return -EFAULT;

	if (!iov.iov_len || iov.iov_len > FSM_DP_MAX_IOV_SIZE)
		return -EINVAL;

	ret = __cdev_tx(cdev, iov.iov_base, iov.iov_len, true);
	return ret;
}

static int __cdev_ioctl_rx_getcfg(struct fsm_dp_cdev *cdev, unsigned long ioarg)
{
	struct fsm_dp_drv *pdrv = cdev->pdrv;
	struct fsm_dp_ioctl_getcfg req;
	struct fsm_dp_ring_cfg cfg;

	if (copy_from_user(&req, (void __user *)ioarg, sizeof(req)))
		return -EFAULT;

	if (!fsm_dp_rx_type_is_valid(req.type) || req.cfg == NULL)
		return -EINVAL;

	fsm_dp_ring_get_cfg(&pdrv->rxq[req.type].ring, &cfg);
	if (copy_to_user((void __user *)req.cfg, &cfg, sizeof(cfg))) {
		FSM_DP_ERROR("%s: copy_to_user failed\n", __func__);
		return -EFAULT;
	}
	return 0;
}

static int __cdev_ioctl_txmode_cfg(
	struct fsm_dp_cdev *cdev,
	unsigned long ioarg)
{
	int ret;
	unsigned int mode;

	ret = get_user(mode, (unsigned int __user *)ioarg);
	if (!ret)
		cdev->tx_mode = mode;
	return ret;
}

#ifdef CONFIG_FSM_DP_TEST
static int __cdev_ioctl_testring_write(
	struct fsm_dp_cdev *cdev,
	unsigned long ioarg)
{
	struct fsm_dp_drv *drv = cdev->pdrv;
	struct fsm_dp_test_ring *test_ring = &drv->test_ring;
	int ret = -EIO;

	if (drv->test_ring.enable)
		ret = fsm_dp_ring_write(&test_ring->ring,
				     TEST_RING_WRITE_MAGIC_VALUE, 0);
	return ret;
}

static int __cdev_ioctl_testring_getcfg(
	struct fsm_dp_cdev *cdev,
	unsigned long ioarg)
{
	struct fsm_dp_drv *drv = cdev->pdrv;
	struct fsm_dp_test_ring *test_ring = &drv->test_ring;
	struct fsm_dp_ring_cfg cfg;
	int ret;

	fsm_dp_ring_get_cfg(&test_ring->ring, &cfg);
	ret = copy_to_user((void __user *)ioarg, &cfg, sizeof(cfg));
	return ret;
}
#endif

static unsigned int fsm_dp_cdev_poll(struct file *file, poll_table *wait)
{
	struct fsm_dp_cdev *cdev = (struct fsm_dp_cdev *)file->private_data;
	struct fsm_dp_drv *drv = cdev->pdrv;
	struct fsm_dp_rxqueue *rxq;
	unsigned int mask = 0;
	int type, n;

	FSM_DP_DEBUG("%s: poll_table %p\n", __func__, wait);

	for (type = 0, n = 0; type < FSM_DP_RX_TYPE_LAST; type++) {
		if (cdev->rxqueue_vma[type].vma) {
			rxq = &drv->rxq[type];
			poll_wait(file, &rxq->wq, wait);
			n++;
		}
	}
	if (unlikely(!n)) {
		FSM_DP_DEBUG("%s: rxqueue not mapped!\n", __func__);
		return POLLERR;
	}

	for (type = 0; type < FSM_DP_RX_TYPE_LAST; type++) {
		if (cdev->rxqueue_vma[type].vma) {
			rxq = &drv->rxq[type];
			if (!fsm_dp_ring_is_empty(&rxq->ring)) {
				mask |= POLLIN | POLLRDNORM;
				break;
			}
		}
	}
	return mask;
}

static long fsm_dp_cdev_ioctl(
	struct file *file,
	unsigned int iocmd,
	unsigned long ioarg)
{
	struct fsm_dp_cdev *cdev = (struct fsm_dp_cdev *)file->private_data;
	int ret = -EINVAL;

	FSM_DP_DEBUG("%s: ioctl_cmd=%08x pid=%u cdev=%p\n",
		  __func__, iocmd, cdev->pid, cdev);

	switch (iocmd) {
	case FSM_DP_IOCTL_MEMPOOL_ALLOC:
		ret = __cdev_ioctl_mempool_alloc(cdev, ioarg);
		break;
	case FSM_DP_IOCTL_MEMPOOL_GET_CONFIG:
		ret = __cdev_ioctl_mempool_getcfg(cdev, ioarg);
		break;
	case FSM_DP_IOCTL_RX_GET_CONFIG:
		ret = __cdev_ioctl_rx_getcfg(cdev, ioarg);
		break;
	case FSM_DP_IOCTL_TX:
		ret = __cdev_ioctl_tx(cdev, ioarg);
		break;
	case FSM_DP_IOCTL_SG_TX:
		ret = __cdev_ioctl_sg_tx(cdev, ioarg);
		break;
	case FSM_DP_IOCTL_TX_MODE_CONFIG:
		ret = __cdev_ioctl_txmode_cfg(cdev, ioarg);
		break;
#ifdef CONFIG_FSM_DP_TEST
	case FSM_DP_IOCTL_TEST_RING_WRITE:
		ret = __cdev_ioctl_testring_write(cdev, ioarg);
		break;
	case FSM_DP_IOCTL_TEST_RING_GET_CONFIG:
		ret = __cdev_ioctl_testring_getcfg(cdev, ioarg);
		break;
#endif
	default:
		break;
	}
	return ret;
}

static void __mempool_mem_vma_open(struct vm_area_struct *vma)
{
	struct fsm_dp_mempool_vma *mempool_vma = vma->vm_private_data;
	atomic_t *refcnt = &mempool_vma->refcnt[FSM_DP_MMAP_TYPE_MEM];

	FSM_DP_DEBUG("%s: vma %p\n", __func__, vma);

	if (atomic_add_return(1, refcnt) == 1) {
		struct fsm_dp_mempool *mempool = *mempool_vma->pp_mempool;

		mempool_vma->vma[FSM_DP_MMAP_TYPE_MEM] = vma;
		if (!fsm_dp_mempool_hold(mempool))
			atomic_dec(refcnt);
	}
}

static void __mempool_mem_vma_close(struct vm_area_struct *vma)
{
	struct fsm_dp_mempool_vma *mempool_vma = vma->vm_private_data;
	atomic_t *refcnt = &mempool_vma->refcnt[FSM_DP_MMAP_TYPE_MEM];

	FSM_DP_DEBUG("%s: vma %p\n", __func__, vma);

	if (atomic_dec_and_test(refcnt)) {
		struct fsm_dp_mempool *mempool = *mempool_vma->pp_mempool;

		mempool_vma->vma[FSM_DP_MMAP_TYPE_MEM] = NULL;
		fsm_dp_mempool_put(mempool);
	}
}

static const struct vm_operations_struct __mempool_mem_vma_ops = {
	.open   = __mempool_mem_vma_open,
	.close  = __mempool_mem_vma_close,
};

static int __mempool_mem_mmap(
	struct fsm_dp_mempool_vma *mempool_vma,
	struct vm_area_struct *vma)
{
	struct fsm_dp_mempool *mempool = *mempool_vma->pp_mempool;
	struct fsm_dp_mem *mem;
	unsigned long size;
	int ret;
	unsigned long addr = vma->vm_start;
	int i;
	unsigned long remainder;

	if (mempool_vma->vma[FSM_DP_MMAP_TYPE_MEM]) {
		FSM_DP_ERROR("%s: memory already mapped\n", __func__);
		return -EBUSY;
	}
	if (!fsm_dp_mempool_hold(mempool)) {
		FSM_DP_ERROR("%s: mempool does not exist, mempool %p\n", __func__, mempool);
		return -EAGAIN;
	}

	mem = &mempool->mem;
	size = vma->vm_end - vma->vm_start;
	remainder = mem->loc.size;
	if (size < remainder) {
		ret = -EINVAL;
		FSM_DP_ERROR(
			"%s: size(0x%lx) too small, expect at least 0x%lx\n",
			__func__, size, remainder);
		goto out;
	}

	/* Reset pgoff */
	vma->vm_pgoff = 0;

	for (i = 0; i < mem->loc.num_cluster; i++) {
		unsigned long len;

		if (i ==  mem->loc.num_cluster - 1)
			len = remainder;
		else
			len = FSM_DP_MEMPOOL_CLUSTER_SIZE;

		ret = remap_pfn_range(vma,
				addr,
				page_to_pfn(mem->loc.page[i]),
				len,
				vma->vm_page_prot);
		if (ret) {
			FSM_DP_ERROR("%s: dma mmap failed\n", __func__);
			goto out;
		}
		addr += len;
		remainder -= len;
	}

	vma->vm_private_data = mempool_vma;
	vma->vm_ops = &__mempool_mem_vma_ops;
	__mempool_mem_vma_open(vma);

out:
	fsm_dp_mempool_put(mempool);
	return ret;
}

static void __mempool_ring_vma_open(struct vm_area_struct *vma)
{
	struct fsm_dp_mempool_vma *mempool_vma = vma->vm_private_data;
	atomic_t *refcnt = &mempool_vma->refcnt[FSM_DP_MMAP_TYPE_RING];

	FSM_DP_DEBUG("%s: vma %p\n", __func__, vma);

	if (atomic_add_return(1, refcnt) == 1) {
		struct fsm_dp_mempool *mempool = *mempool_vma->pp_mempool;

		mempool_vma->vma[FSM_DP_MMAP_TYPE_RING] = vma;
		__fsm_dp_mempool_hold(mempool);
	}
}

static void __mempool_ring_vma_close(struct vm_area_struct *vma)
{
	struct fsm_dp_mempool_vma *mempool_vma = vma->vm_private_data;
	atomic_t *refcnt = &mempool_vma->refcnt[FSM_DP_MMAP_TYPE_RING];

	FSM_DP_DEBUG("%s: vma %p\n", __func__, vma);

	if (atomic_dec_and_test(refcnt)) {
		struct fsm_dp_mempool *mempool = *mempool_vma->pp_mempool;

		mempool_vma->vma[FSM_DP_MMAP_TYPE_RING] = NULL;
		fsm_dp_mempool_put(mempool);
	}
}

static const struct vm_operations_struct __mempool_ring_vma_ops = {
	.open   = __mempool_ring_vma_open,
	.close  = __mempool_ring_vma_close,
};

static int __mempool_ring_mmap(
	struct fsm_dp_mempool_vma *mempool_vma,
	struct vm_area_struct *vma)
{
	struct fsm_dp_mempool *mempool = *mempool_vma->pp_mempool;
	struct fsm_dp_ring *ring;
	unsigned long size;
	int ret;

	if (mempool_vma->vma[FSM_DP_MMAP_TYPE_RING]) {
		FSM_DP_ERROR("%s: ring already mapped, mem_type=%u\n",
			  __func__, mempool->type);
		ret = -EBUSY;
	}

	if (!fsm_dp_mempool_hold(mempool)) {
		FSM_DP_ERROR("%s: mempool not exist\n", __func__);
		return -EAGAIN;
	}

	ring = &mempool->ring;
	size = vma->vm_end - vma->vm_start;
	if (size < fsm_dp_mem_loc_mmap_size(&ring->loc)) {
		FSM_DP_ERROR(
			"%s: size(0x%lx) too small, expect at least 0x%lx\n",
			__func__, size, fsm_dp_mem_loc_mmap_size(&ring->loc));
		ret = -EINVAL;
		goto out;
	}

	ret = remap_pfn_range(vma,
			      vma->vm_start,
			      page_to_pfn(ring->loc.page[0]),
			      ring->loc.size,
			      vma->vm_page_prot);
	if (ret) {
		FSM_DP_ERROR("%s: remap_pfn_range failed\n", __func__);
		goto out;
	}

	/* Reset pgoff */
	vma->vm_pgoff = 0;
	vma->vm_private_data = mempool_vma;
	vma->vm_ops = &__mempool_ring_vma_ops;
	__mempool_ring_vma_open(vma);

out:
	fsm_dp_mempool_put(mempool);
	return ret;
}

/* mmap mempool into user space */
static int __cdev_mempool_mmap(
	struct fsm_dp_cdev *cdev,
	struct vm_area_struct *vma)
{
	struct fsm_dp_mempool_vma *mempool_vma;
	unsigned int mem_type, type, cookie;
	int ret = 0;

	/* use vm_pgoff to distinguish different area to map */
	cookie = vma->vm_pgoff << PAGE_SHIFT;
	type = MMAP_COOKIE_TO_TYPE(cookie);
	mem_type = MMAP_COOKIE_TO_MEM_TYPE(cookie);

	if (!fsm_dp_mem_type_is_valid(mem_type) ||
	    !fsm_dp_mmap_type_is_valid(type)) {
		FSM_DP_ERROR("%s: invalid cookie(0x%x)\n", __func__, cookie);
		return -EINVAL;
	}

	mempool_vma = &cdev->mempool_vma[mem_type];
	switch (type) {
	case FSM_DP_MMAP_TYPE_RING:
		/* map ring for buffer manangement */
		ret = __mempool_ring_mmap(mempool_vma, vma);
		break;
	case FSM_DP_MMAP_TYPE_MEM:
		/* map buffer memory */
		ret = __mempool_mem_mmap(mempool_vma, vma);
		break;
	}

	return ret;
}

static void __rxqueue_vma_open(struct vm_area_struct *vma)
{
	struct fsm_dp_rxqueue_vma *rxq_vma = vma->vm_private_data;

	FSM_DP_DEBUG("%s: vma %p\n", __func__, vma);

	if (atomic_add_return(1, &rxq_vma->refcnt) == 1) {
		struct fsm_dp_rxqueue *rxq;

		rxq_vma->vma = vma;
		rxq_vma->type = MMAP_RX_COOKIE_TO_TYPE(
			vma->vm_pgoff << PAGE_SHIFT);

		rxq = rxqueue_vma_to_rxqueue(rxq_vma);
		atomic_inc(&rxq->refcnt);
	}
}

static void __rxqueue_vma_close(struct vm_area_struct *vma)
{
	struct fsm_dp_rxqueue_vma *rxq_vma = vma->vm_private_data;
	struct fsm_dp_rxqueue *rxq = rxqueue_vma_to_rxqueue(rxq_vma);

	FSM_DP_DEBUG("%s: vma %p\n", __func__, vma);

	if (!atomic_dec_and_test(&rxq_vma->refcnt))
		return;
	rxq_vma->vma = NULL;
	atomic_dec(&rxq->refcnt);
}

static const struct vm_operations_struct __rxqueue_vma_ops = {
	.open   = __rxqueue_vma_open,
	.close  = __rxqueue_vma_close,
};

/* mmap RXQ into user space */
static int __cdev_rxqueue_mmap(
	struct fsm_dp_cdev *cdev,
	struct vm_area_struct *vma)
{
	struct fsm_dp_drv *drv = cdev->pdrv;
	struct fsm_dp_rxqueue_vma *rxq_vma = cdev->rxqueue_vma;
	struct fsm_dp_ring *ring;
	unsigned int type, cookie;
	unsigned long size;
	int ret = 0;

	cookie = vma->vm_pgoff << PAGE_SHIFT;

	type = MMAP_RX_COOKIE_TO_TYPE(cookie);
	if (!fsm_dp_rx_type_is_valid(type)) {
		FSM_DP_ERROR(
			"%s: invalid rx queue type, cookie=0x%x, type=%u\n",
			__func__, cookie, type);
		return -EINVAL;
	}

	if (rxq_vma[type].vma) {
		FSM_DP_ERROR("%s: rxqueue already mapped\n", __func__);
		return -EBUSY;
	}

	ring = &drv->rxq[type].ring;
	size = vma->vm_end - vma->vm_start;
	if (size < fsm_dp_mem_loc_mmap_size(&ring->loc)) {
		FSM_DP_ERROR(
			"%s: size(0x%lx) too small, expect at least 0x%lx\n",
			__func__, size, fsm_dp_mem_loc_mmap_size(&ring->loc));
		return -EINVAL;
	}
	ret = remap_pfn_range(vma,
			      vma->vm_start,
			      page_to_pfn(ring->loc.page[0]),
			      ring->loc.size,
			      vma->vm_page_prot);
	if (ret) {
		FSM_DP_ERROR("%s: rxqueue mmap failed, error=%d\n",
			     __func__, ret);
		return ret;
	}

	vma->vm_private_data = &rxq_vma[type];
	vma->vm_ops = &__rxqueue_vma_ops;
	__rxqueue_vma_open(vma);

	return 0;
}

#ifdef CONFIG_FSM_DP_TEST
static int __fsm_dp_cdev_testring_mmap(
	struct fsm_dp_cdev *cdev,
	struct vm_area_struct *vma)
{
	struct fsm_dp_drv *drv = cdev->pdrv;
	struct fsm_dp_ring *ring;
	int ret = 0;

	ring = &drv->test_ring.ring;

	ret = remap_pfn_range(vma,
			      vma->vm_start,
			      page_to_pfn(ring->loc.page[0])
			      (ring->loc.size),
			      vma->vm_page_prot);
	if (ret) {
		FSM_DP_DEBUG("%s: mmap failed\n", __func__);
		return ret;
	}
	return 0;
}
#endif

static int fsm_dp_cdev_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct fsm_dp_cdev *cdev = (struct fsm_dp_cdev *)file->private_data;
	unsigned int cookie;
	int ret = 0;

	FSM_DP_DEBUG("%s: start=%lx end=%lx off=%lx proto=%lx flag=%lx",
		  __func__, vma->vm_start, vma->vm_end, vma->vm_pgoff,
		  (unsigned long)vma->vm_page_prot.pgprot, vma->vm_flags);

	cookie = vma->vm_pgoff << PAGE_SHIFT;

#ifdef CONFIG_FSM_DP_TEST
	if (cookie == TEST_RING_MMAP_COOKIE)
		return __fsm_dp_cdev_testring_mmap(cdev, vma);
#endif

	if (is_rxqueue_mmap_cookie(cookie))
		ret = __cdev_rxqueue_mmap(cdev, vma);
	else
		ret = __cdev_mempool_mmap(cdev, vma);

	return ret;
}

static int fsm_dp_cdev_open(struct inode *inode, struct file *file)
{
	struct fsm_dp_drv *pdrv = container_of(inode->i_cdev,
					    struct fsm_dp_drv, cdev);
	struct fsm_dp_cdev *cdev;

	cdev = kzalloc(sizeof(*cdev), GFP_KERNEL);
	if (IS_ERR(cdev)) {
		FSM_DP_ERROR("%s: failed to alloc memory\n!", __func__);
		return -ENOMEM;
	}

	cdev->pdrv = pdrv;
	cdev->pid = current->tgid;

	__cdev_init_mempool_vma(cdev);

	mutex_lock(&pdrv->cdev_lock);
	list_add_tail(&cdev->list, &pdrv->cdev_head);
	mutex_unlock(&pdrv->cdev_lock);

	FSM_DP_DEBUG("%s: cdev=%p pid=%u\n", __func__, cdev, cdev->pid);

	file->private_data = cdev;
	return 0;
}

static int fsm_dp_cdev_close(struct inode *inode, struct file *file)
{
	struct fsm_dp_cdev *cdev = (struct fsm_dp_cdev *)file->private_data;
	struct fsm_dp_mempool_vma *mempool_vma = cdev->mempool_vma;
	struct fsm_dp_drv *pdrv = cdev->pdrv;
	int type;

	FSM_DP_DEBUG("%s: device close, pid=%u, cdev=%p\n",
		  __func__, cdev->pid, cdev);

	for (type = 0; type < FSM_DP_MEM_TYPE_LAST; type++, mempool_vma++) {
		if (mempool_vma->usr_alloc)
			fsm_dp_mempool_put(*mempool_vma->pp_mempool);
	}
	mutex_lock(&pdrv->cdev_lock);
	list_del(&cdev->list);
	mutex_unlock(&pdrv->cdev_lock);

	kfree(cdev);
	return 0;
}

static const struct file_operations fsm_dp_cdev_fops = {
	.owner = THIS_MODULE,
	.poll = fsm_dp_cdev_poll,
	.unlocked_ioctl = fsm_dp_cdev_ioctl,
	.mmap = fsm_dp_cdev_mmap,
	.open = fsm_dp_cdev_open,
	.release = fsm_dp_cdev_close
};

int fsm_dp_cdev_init(struct fsm_dp_drv *pdrv)
{
	struct device *dev;
	dev_t devno;
	int ret;

	pdrv->dev_class = class_create(THIS_MODULE, FSM_DP_DEV_CLASS_NAME);
	if (IS_ERR(pdrv->dev_class)) {
		FSM_DP_ERROR("%s: class_create failed\n", __func__);
		return -ENOMEM;
	}

	ret = alloc_chrdev_region(&devno, 0, 1, FSM_DP_CDEV_NAME);
	if (ret) {
		FSM_DP_ERROR("%s: alloc_chrdev_region failed\n", __func__);
		goto cleanup_class;
	}

	cdev_init(&pdrv->cdev, &fsm_dp_cdev_fops);
	ret = cdev_add(&pdrv->cdev, devno, 1);
	if (ret) {
		FSM_DP_ERROR("%s: cdev_add failed!\n", __func__);
		goto unregister_cdev;
	}

	dev = device_create(pdrv->dev_class, pdrv->dev, devno,
			    pdrv, FSM_DP_CDEV_NAME);
	if (IS_ERR(dev)) {
		FSM_DP_ERROR("%s: device_create failed\n", __func__);
		ret = PTR_ERR(dev);
		goto del_cdev;
	}

	mutex_init(&pdrv->cdev_lock);
	INIT_LIST_HEAD(&pdrv->cdev_head);

	pr_info("FSM-DP: cdev initialized. __cdev_tx at 0x%p\n", __cdev_tx);
	return 0;

del_cdev:
	cdev_del(&pdrv->cdev);
unregister_cdev:
	unregister_chrdev_region(pdrv->cdev.dev, 1);
cleanup_class:
	class_destroy(pdrv->dev_class);
	pdrv->dev_class = NULL;
	pr_err("FSM-DP: failed to initialize cdev\n");
	return ret;
}

void fsm_dp_cdev_cleanup(struct fsm_dp_drv *pdrv)
{
	if (pdrv->dev_class) {
		device_destroy(pdrv->dev_class, pdrv->cdev.dev);
		cdev_del(&pdrv->cdev);
		unregister_chrdev_region(pdrv->cdev.dev, 1);
		class_destroy(pdrv->dev_class);
		mutex_destroy(&pdrv->cdev_lock);
		pdrv->dev_class = NULL;
	}
}
