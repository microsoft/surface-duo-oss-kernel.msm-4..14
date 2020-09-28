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
#include <linux/list.h>

#include "fsm_dp.h"
#include "fsm_dp_mem.h"

#define FSM_DP_MEMPOOL_RELEASE_DELAY	(HZ * 2)

static inline struct fsm_dp_mempool *fsm_dp_mem_to_mempool(
	struct fsm_dp_mem *mem)
{
	struct fsm_dp_mempool *mempool = container_of(mem,
						   struct fsm_dp_mempool, mem);
	return mempool;
}

static inline void fsm_dp_mem_loc_set(
	struct fsm_dp_mem_loc *loc,
	size_t size,
	unsigned int mmap_cookie)
{
	loc->base = loc->cluster_kernel_addr[0];
	loc->size = size;
	loc->cookie = mmap_cookie;
	loc->dma_mapped = false;
}

static inline int __alloc_ring(
	size_t size,
	unsigned int mmap_cookie,
	struct fsm_dp_mem_loc *loc)
{
	unsigned int order;
	struct page *page;

	order = get_order(size);
	if (order > get_order(FSM_DP_MEMPOOL_CLUSTER_SIZE)) {
		FSM_DP_ERROR("%s: failed to allocate memory. Too Big %ld\n",
				__func__, size);
		return -ENOMEM;
	}
	page = alloc_pages(GFP_KERNEL, order);
	loc->last_cl_order = order;
	loc->num_cluster = 1;
	if (page) {
		loc->page[0] = page;
		loc->cluster_kernel_addr[0] = page_address(page);
		fsm_dp_mem_loc_set(loc, size, mmap_cookie);
		return 0;
	}
	return -ENOMEM;
}

static inline void __free_ring(struct fsm_dp_mem_loc *loc)
{
	if (loc && loc->page[0]) {
		__free_pages(loc->page[0], loc->last_cl_order);
		memset(loc, 0, sizeof(*loc));
	}
}

static inline int __buf_mem_alloc(size_t size,
			      unsigned int mmap_cookie,
			      struct fsm_dp_mem_loc *loc)
{
	unsigned int order;
	struct page *page;
	int i;
	unsigned long rem = size;
	unsigned long len;

	for (i = 0; i < loc->num_cluster; i++) {
		if (i == loc->num_cluster - 1)
			len = rem;
		else
			len = FSM_DP_MEMPOOL_CLUSTER_SIZE;
		order = get_order(len);
		if (i == loc->num_cluster - 1)
			loc->last_cl_order = order;
		page = alloc_pages(GFP_KERNEL, order);
		if (!page)
			goto error;
		loc->page[i] = page;
		loc->cluster_kernel_addr[i] = page_address(page);
		rem -= len;
	}
	fsm_dp_mem_loc_set(loc, size, mmap_cookie);
	return 0;
error:
	for (i = 0; i < loc->num_cluster; i++) {
		if (loc->page[i]) {
			if (i == loc->num_cluster - 1)
				order = loc->last_cl_order;
			else
				order = get_order(FSM_DP_MEMPOOL_CLUSTER_SIZE);
			__free_pages(loc->page[i], order);
			loc->page[i] = NULL;
		}
	}
	loc->num_cluster = 0;
	return -ENOMEM;
}

static inline void __buf_mem_free(struct fsm_dp_mem_loc *loc)
{
	int i;
	unsigned int order = get_order(FSM_DP_MEMPOOL_CLUSTER_SIZE);

	if (loc) {
		for (i = 0; i < loc->num_cluster; i++) {
			if (loc->page[i]) {
				if (i == loc->num_cluster - 1)
					order = loc->last_cl_order;
				__free_pages(loc->page[i], order);
			}
		}
		memset(loc, 0, sizeof(*loc));
	}
}

int fsm_dp_ring_init(
	struct fsm_dp_ring *ring,
	unsigned int ringsz,
	unsigned int mmap_cookie)
{
	unsigned int allocsz = ringsz * sizeof(*ring->element);
	char *aligned_ptr;
	fsm_dp_ring_element_t *elem_p;
	int i;

	/* cons and prod index space, aligned to cache line */
	allocsz += 4 * cache_line_size();
	allocsz = ALIGN(allocsz, cache_line_size());

	if (__alloc_ring(allocsz, mmap_cookie, &ring->loc)) {
		FSM_DP_ERROR("%s: failed to allocate ring memory\n", __func__);
		return -ENOMEM;
	}

	aligned_ptr = (char *)ALIGN((unsigned long)ring->loc.base,
				    cache_line_size());
	ring->prod_head = (fsm_dp_ring_index_t *)aligned_ptr;
	aligned_ptr += cache_line_size();
	ring->prod_tail = (fsm_dp_ring_index_t *)aligned_ptr;
	aligned_ptr += cache_line_size();
	ring->cons_head = (fsm_dp_ring_index_t *)aligned_ptr;
	aligned_ptr += cache_line_size();
	ring->cons_tail = (fsm_dp_ring_index_t *)aligned_ptr;
	aligned_ptr += cache_line_size();
	ring->element = elem_p = (fsm_dp_ring_element_t *)aligned_ptr;
	for (i = 0; i < ringsz; i++, elem_p++)
		elem_p->element_ctrl = 1; /* not valid */
	ring->size = ringsz;
	*ring->prod_head = *ring->prod_tail = *ring->cons_head =
						*ring->cons_tail = 0;
	return 0;
}

void fsm_dp_ring_cleanup(struct fsm_dp_ring *ring)
{
	if (ring) {
		__free_ring(&ring->loc);
		memset(ring, 0, sizeof(*ring));
	}
}

int fsm_dp_ring_get_cfg(struct fsm_dp_ring *ring, struct fsm_dp_ring_cfg *cfg)
{
	if (unlikely(ring == NULL || cfg == NULL))
		return -EINVAL;
	cfg->mmap.length = ring->loc.size;
	cfg->mmap.cookie = ring->loc.cookie;

	cfg->size = ring->size;
	cfg->prod_head_off = vaddr_offset((void *)ring->prod_head,
							ring->loc.base);
	cfg->prod_tail_off = vaddr_offset((void *)ring->prod_tail,
							ring->loc.base);
	cfg->cons_head_off = vaddr_offset((void *)ring->cons_head,
							ring->loc.base);
	cfg->cons_tail_off = vaddr_offset((void *)ring->cons_tail,
							ring->loc.base);
	cfg->ringbuf_off = vaddr_offset((void *)ring->element,
							ring->loc.base);
	return 0;
}

/* Read from ring */
int fsm_dp_ring_read(
	struct fsm_dp_ring *ring,
	fsm_dp_ring_element_data_t *element_ptr, unsigned int *flag)
{
	register fsm_dp_ring_index_t cons_head, cons_next, cons_tail;
	register fsm_dp_ring_index_t prod_tail, mask;
	fsm_dp_ring_element_data_t data;

	if (unlikely(ring == NULL))
		return -EINVAL;

	mask = ring->size - 1;

again:
	/* test to see if the ring is empty.
	 * If not, advance cons_head and read the data
	 */
	cons_head = *ring->cons_head;
	prod_tail = *ring->prod_tail;
	rmb();	/* Get current cons_head and prod_tail */
	if ((cons_head & mask) == (prod_tail & mask)) {
		ring->opstats.read_empty++;
		return -EAGAIN;
	}
	cons_next = cons_head + 1;
	if (atomic_cmpxchg((atomic_t *)ring->cons_head,
			   cons_head,
			   cons_next) != cons_head) {
		ring->opstats.cons_head_updt_retry++;
		goto again;
	}

	/* Read the ring */
	data = ring->element[(cons_head & mask)].element_data;
	if (flag)
		*flag = ring->element[(cons_head & mask)].element_ctrl >> 1;
	rmb();	/* Get current element */

	/* After read, write to ring with bit0 on */

	ring->element[(cons_head & mask)].element_ctrl = 1;
	wmb();	/* Ensure element is written */

	if (element_ptr)
		*element_ptr = data;

	/* Move the tail */
	cons_tail = *ring->cons_tail;
	rmb();	/* Get current cons_tail */

	/* If tail is behind, let other producer to update it */
	if (cons_head != cons_tail) {
		ring->opstats.cons_tail_no_updt++;
		return 0;
	}

repeat:
	/* Potential two consumer is updating */
	if (atomic_cmpxchg((atomic_t *)ring->cons_tail,
			   cons_tail,
			   cons_next) != cons_tail) {
		/* the other producer wins */
		ring->opstats.cons_tail_updt_backoff++;
		return 0;
	}

	ring->opstats.cons_tail_updt++;
	cons_tail = cons_next;
	cons_next++;

	/* This consumer win, read the cons_head */
	cons_head = *ring->cons_head;
	rmb();	/* Get current cons_head */

	if (cons_tail == cons_head)
		return 0;

	/* The reader has not cleared the bit0 */
	if (!(ring->element[(cons_tail & mask)].element_ctrl & 1)) {
		ring->opstats.cons_tail_updt_stop++;
		return 0;
	}

	goto repeat;
}

/* Write to ring */
int fsm_dp_ring_write(struct fsm_dp_ring *ring, fsm_dp_ring_element_data_t data,
		unsigned int flag)
{
	register fsm_dp_ring_index_t prod_head, prod_next, prod_tail;
	register fsm_dp_ring_index_t cons_tail, mask;

	if (unlikely(ring == NULL))
		return -EINVAL;

	mask = ring->size - 1;

again:
	/* test to see if the ring is full.
	 * If not, advance prod_head and write the data
	 */
	prod_head = *ring->prod_head;
	cons_tail = *ring->cons_tail;
	rmb();	/* Get current prod_head and cons_tail */
	prod_next = prod_head + 1;
	if ((prod_next & mask) == (cons_tail & mask)) {
		ring->opstats.write_full++;
		return -EAGAIN;
	}
	if (atomic_cmpxchg((atomic_t *)ring->prod_head,
			   prod_head,
			   prod_next) != prod_head) {
		ring->opstats.prod_head_updt_retry++;
		goto again;
	}

#ifdef CONFIG_FSM_DP_TEST
	if (data == TEST_RING_WRITE_MAGIC_VALUE)
		data = prod_head << 1;
#endif
	/* Write to ring buffer with bit0 off */
	ring->element[(prod_head & mask)].element_data = data;
	ring->element[(prod_head & mask)].element_ctrl = flag << 1;
	wmb();	/* Ensure element is written */

	ring->opstats.write_ok++;
	/* Move the tail */
	prod_tail = *ring->prod_tail;
	rmb();	/* Get current prod_tail */

	/* If tail is behind, let other producer to update it */
	if (prod_head != prod_tail) {
		ring->opstats.prod_tail_no_updt++;
		return 0;
	}

repeat:
	/* Potential two producer is updating */
	if (atomic_cmpxchg((atomic_t *)ring->prod_tail,
			   prod_tail,
			   prod_next) != prod_tail) {
		/* the other producer wins */
		ring->opstats.prod_tail_updt_backoff++;
		return 0;
	}

	ring->opstats.prod_tail_updt++;
	prod_tail = prod_next;
	prod_next++;

	/* This producer win, read the prod_head */
	prod_head = *ring->prod_head;
	rmb();	/* Get current prod_head */

	if (prod_tail == prod_head)
		return 0;

	/* The writer has not written the data yet */
	if (ring->element[(prod_tail & mask)].element_ctrl & 1) {
		ring->opstats.prod_tail_updt_stop++;
		return 0;
	}

	goto repeat;
}

bool fsm_dp_ring_is_empty(struct fsm_dp_ring *ring)
{
	fsm_dp_ring_index_t prod_tail, cons_tail;

	prod_tail = *ring->prod_tail;
	cons_tail = *ring->cons_tail;
	if (prod_tail == cons_tail)
		return true;
	return false;
}

static int fsm_dp_mem_init(
	struct fsm_dp_mem *mem,
	unsigned int bufcnt,
	unsigned int bufsz,
	unsigned int cookie)
{
	unsigned int num_buf_cl; /* number of buffers per cluster */
	unsigned int num_cl; /* number of clusters */
	unsigned long size;
	unsigned long rem_size = 0;
	unsigned int rem_buf;

	mem->buf_cnt = bufcnt;
	mem->buf_sz = ALIGN(bufsz, cache_line_size());
	mem->buf_overhead_sz = FSM_DP_L1_CACHE_BYTES;

	num_buf_cl = FSM_DP_MEMPOOL_CLUSTER_SIZE / fsm_dp_buf_true_size(mem);
	num_cl = bufcnt / num_buf_cl;
	size = num_cl * FSM_DP_MEMPOOL_CLUSTER_SIZE;
	rem_buf = bufcnt % num_buf_cl;
	if (rem_buf) {
		num_cl++;
		rem_size = fsm_dp_buf_true_size(mem) * rem_buf;
	}
	if (num_cl > MAX_FSM_DP_MEMPOOL_CLUSTER) {
		FSM_DP_ERROR("%s: failed to allocate DMA memory. Too Big %d\n",
				__func__, num_cl);
		return -ENOMEM;
	}
	mem->loc.num_cluster = num_cl;
	mem->loc.buf_per_cluster = num_buf_cl;
	size += rem_size;
	if (__buf_mem_alloc(size, cookie, &mem->loc)) {
		FSM_DP_ERROR("%s: failed to allocate DMA memory\n", __func__);
		return -ENOMEM;
	}
	return 0;
}

static void fsm_dp_mem_cleanup(struct fsm_dp_mem *mem)
{
	struct fsm_dp_mempool *mempool = fsm_dp_mem_to_mempool(mem);
	struct fsm_dp_drv *pdrv = mempool->drv;
	int i;
	unsigned int size;

	spin_lock(&mempool->lock);
	if (mem->loc.dma_mapped) {
		size = mem->loc.size;
		for (i = 0; i < mem->loc.num_cluster; i++) {
			if (i ==  mem->loc.num_cluster - 1) {
				dma_unmap_single(
					pdrv->mhi.mhi_dev->mhi_cntrl->dev,
					mem->loc.cluster_dma_addr[i],
					size,
					mem->loc.direction);
			} else {
				dma_unmap_single(
					pdrv->mhi.mhi_dev->mhi_cntrl->dev,
					mem->loc.cluster_dma_addr[i],
					FSM_DP_MEMPOOL_CLUSTER_SIZE,
					mem->loc.direction);
				size -= FSM_DP_MEMPOOL_CLUSTER_SIZE;
			}
		}
	}
	mem->loc.dma_mapped = false;
	spin_unlock(&mempool->lock);

	__buf_mem_free(&mem->loc);
	memset(mem, 0, sizeof(*mem));
}

static int fsm_dp_mem_get_cfg(
	struct fsm_dp_mem *mem,
	struct fsm_dp_mem_cfg *cfg)
{
	cfg->mmap.length = mem->loc.size;
	cfg->mmap.cookie = mem->loc.cookie;

	cfg->buf_sz = mem->buf_sz;
	cfg->buf_cnt = mem->buf_cnt;
	cfg->buf_overhead_sz = FSM_DP_L1_CACHE_BYTES;
	cfg->cluster_size = FSM_DP_MEMPOOL_CLUSTER_SIZE;
	cfg->num_cluster = mem->loc.num_cluster;
	cfg->buf_per_cluster = mem->loc.buf_per_cluster;
	return 0;
}

static void fsm_dp_mempool_init(struct fsm_dp_mempool *mempool)
{
	struct fsm_dp_mem *mem = &mempool->mem;
	struct fsm_dp_ring *ring = &mempool->ring;
	fsm_dp_ring_element_data_t element_data;
	int i, j;
	struct fsm_dp_buf_cntrl *p;
	unsigned int cl_buf_cnt;
	unsigned int buf_index = 0;
	char *cl_start;

	switch (mempool->type) {
	case FSM_DP_MEM_TYPE_DL_L1_DATA:
	case FSM_DP_MEM_TYPE_DL_L1_CTL:
	case FSM_DP_MEM_TYPE_DL_RF:
	case FSM_DP_MEM_TYPE_UL:
		for (j = 0; j < mem->loc.num_cluster; j++) {
			element_data = j * FSM_DP_MEMPOOL_CLUSTER_SIZE;
			cl_start = mem->loc.cluster_kernel_addr[j];
			if (j == mem->loc.num_cluster - 1)
				cl_buf_cnt = mem->buf_cnt -
					(mem->loc.buf_per_cluster * j);
			else
				cl_buf_cnt = mem->loc.buf_per_cluster;
			for (i = 0; i < cl_buf_cnt; i++) {
				p = (struct fsm_dp_buf_cntrl *) (cl_start +
					(i * fsm_dp_buf_true_size(mem)));
				p->signature = FSM_DP_BUFFER_SIG;
				p->fence = FSM_DP_BUFFER_FENCE_SIG;
				p->state = FSM_DP_BUF_STATE_KERNEL_FREE;
				p->buf_index = buf_index;
				if (mempool->type != FSM_DP_MEM_TYPE_UL)
					p->xmit_status = FSM_DP_XMIT_OK;
				/* pointing to start of user data */
				ring->element[buf_index].element_data =
					element_data + mem->buf_overhead_sz;
				/* entry valid */
				ring->element[buf_index].element_ctrl = 0;
				element_data += fsm_dp_buf_true_size(mem);
				buf_index++;
			}
		}
		*ring->cons_head = *ring->cons_tail = 0;
		*ring->prod_head = *ring->prod_tail = mem->buf_cnt - 1;
		wmb();	/* Ensure all the data are written */
		break;
	default:
		break;
	}
}

static struct fsm_dp_mempool *__fsm_dp_mempool_alloc(
	struct fsm_dp_drv *pdrv,
	enum fsm_dp_mem_type type,
	unsigned int buf_sz,
	unsigned int buf_cnt,
	unsigned int ring_sz,
	bool may_map)
{
	struct fsm_dp_mempool *mempool;
	unsigned int cookie;

	mempool = kzalloc(sizeof(*mempool), GFP_KERNEL);
	if (IS_ERR(mempool)) {
		FSM_DP_ERROR("%s: failed to allocate mempool\n", __func__);
		return NULL;
	}

	mempool->drv = pdrv;
	mempool->type = type;
	mempool->signature = FSM_DP_MEMPOOL_SIG;

	/*
	 * allocate dummy buffer for out of buffer condition
	 * if FSM_DP_MEM_TYPE_UL pool
	 */
	if (type == FSM_DP_MEM_TYPE_UL) {
		mempool->dummy_buf = kzalloc(buf_sz, GFP_KERNEL);
		if (IS_ERR(mempool->dummy_buf)) {
			mempool->dummy_buf = NULL;
			goto cleanup;
		}
	}

	cookie = MMAP_COOKIE(type, FSM_DP_MMAP_TYPE_MEM);
	if (fsm_dp_mem_init(&mempool->mem, buf_cnt, buf_sz, cookie)) {
		FSM_DP_ERROR("%s: failed to initialize memory\n", __func__);
		goto cleanup;
	}

	if (fsm_dp_mhi_is_ready(&pdrv->mhi) && may_map &&
			fsm_dp_mempool_dma_map(pdrv, mempool, type))
		goto cleanup_mem;
	cookie = MMAP_COOKIE(type, FSM_DP_MMAP_TYPE_RING);
	if (fsm_dp_ring_init(&mempool->ring, ring_sz, cookie)) {
		FSM_DP_ERROR("%s: failed to initialize ring\n", __func__);
		goto cleanup_mem;
	}

	fsm_dp_mempool_init(mempool);

	FSM_DP_DEBUG("%s: mempool is created, type=%u bufsz=%u bufcnt=%u\n",
		  __func__, type, buf_sz, buf_cnt);

	return mempool;

cleanup_mem:
	fsm_dp_mem_cleanup(&mempool->mem);
cleanup:
	kfree(mempool->dummy_buf);
	kfree(mempool);
	return NULL;
}

static void fsm_dp_mempool_release(struct fsm_dp_mempool *mempool)
{
	if (mempool) {
		enum fsm_dp_mem_type type = mempool->type;

		mempool->signature = FSM_DP_MEMPOOL_SIG_BAD;
		wmb();
		fsm_dp_mem_cleanup(&mempool->mem);
		fsm_dp_ring_cleanup(&mempool->ring);
		kfree(mempool->dummy_buf);
		kfree(mempool);
		FSM_DP_DEBUG("%s: mempool is freed, type=%u\n", __func__, type);
	}
}

#define FSM_DP_MEMPOOL_RELEASE_SLEEP 60 /* 60 ms */
void fsm_dp_mempool_release_no_delay(struct fsm_dp_mempool *mempool)
{
	unsigned int out_xmit, out_xmit1;

	if (!mempool)
		return;
	/* wait for all tx buffers done */

	out_xmit1 = atomic_read(&mempool->out_xmit);
	if (out_xmit1) {
		msleep(FSM_DP_MEMPOOL_RELEASE_SLEEP);
		out_xmit = atomic_read(&mempool->out_xmit);
		if (out_xmit)
			FSM_DP_ERROR(
				"mempool %p out_xmit changed from %d to %d after %d ms\n",
				mempool, out_xmit1, out_xmit, FSM_DP_MEMPOOL_RELEASE_SLEEP);
	}

	fsm_dp_mempool_release(mempool);
}

int fsm_dp_mempool_dma_map(
	struct fsm_dp_drv *pdrv,
	struct fsm_dp_mempool *mpool,
	enum fsm_dp_mem_type type)
{
	enum dma_data_direction direction;
	struct device *dev;	/* device for iommu ops */
	int i, k;
	unsigned int size;
	struct fsm_dp_mem_loc *loc;

	loc = &mpool->mem.loc;
	if (loc->dma_mapped)
		return 0;
	dev = pdrv->mhi.mhi_dev->mhi_cntrl->dev;
	if (type == FSM_DP_MEM_TYPE_UL)
		direction = DMA_BIDIRECTIONAL; /* rx, tx for rx loopback */
	else
		direction = DMA_TO_DEVICE;
	size = loc->size;
	for (i = 0; i < loc->num_cluster; i++) {
		if (i == loc->num_cluster - 1)
			loc->cluster_dma_addr[i] =
				dma_map_single(dev,
					loc->cluster_kernel_addr[i],
					size,
					direction);
		else {
			loc->cluster_dma_addr[i] =
				dma_map_single(dev,
					loc->cluster_kernel_addr[i],
					FSM_DP_MEMPOOL_CLUSTER_SIZE,
					direction);
			size -= FSM_DP_MEMPOOL_CLUSTER_SIZE;
		}
		if (dma_mapping_error(dev, loc->cluster_dma_addr[i]))
			goto error;
	}
	mpool->mem.loc.dma_mapped = true;
	mpool->mem.loc.direction = direction;
	return 0;
error:
	for (k = 0; k < i - 1; k++) {
		dma_unmap_single(dev, loc->cluster_dma_addr[k],
			FSM_DP_MEMPOOL_CLUSTER_SIZE,
			loc->direction);
	}
	return -ENOMEM;
}

struct fsm_dp_mempool *fsm_dp_mempool_alloc(
	struct fsm_dp_drv *pdrv,
	enum fsm_dp_mem_type type,
	unsigned int buf_sz,
	unsigned int buf_cnt,
	bool may_dma_map)
{
	struct fsm_dp_mempool *mempool;
	unsigned int ring_sz;

	if (unlikely(!buf_sz || !buf_cnt || !fsm_dp_mem_type_is_valid(type)))
		return NULL;
	if (unlikely(((ULONG_MAX) / (buf_sz + FSM_DP_L1_CACHE_BYTES) < buf_cnt)))
		return NULL;
	if (buf_sz > FSM_DP_MAX_DL_MSG_LEN) {
		FSM_DP_ERROR("%s: mempool alloc buffer size %d exceeds limit %d\n",
			__func__, buf_sz, FSM_DP_MAX_DL_MSG_LEN);
		return NULL;
	}
	if (pdrv->mhi.mhi_destroyed) {
		FSM_DP_WARN("%s: mhi device destroyed\n", __func__);
		return NULL;
	}

	ring_sz = calc_ring_size(buf_cnt);
	if (unlikely(!ring_sz))
		return NULL;

	mutex_lock(&pdrv->mempool_lock);
	mempool = pdrv->mempool[type];
	if (mempool) {
		if (type !=  FSM_DP_MEM_TYPE_UL &&
			(buf_sz > mempool->mem.buf_sz ||
				buf_cnt > mempool->mem.buf_cnt)) {
			FSM_DP_ERROR(
				"%s: can't use existing mempool, type=%u\n",
				__func__, type);
			mempool = NULL;
			goto done;
		}
		goto mempool_hold;
	}

	mempool = __fsm_dp_mempool_alloc(pdrv, type, buf_sz,
					buf_cnt, ring_sz, may_dma_map);
	if (mempool == NULL)
		goto done;
	atomic_set(&mempool->ref, 1);
	atomic_set(&mempool->out_xmit, 0);
	spin_lock_init(&mempool->lock);
	pdrv->mempool[type] = mempool;
	goto done;
mempool_hold:
	if (!fsm_dp_mempool_hold(mempool))
		mempool = NULL;
done:
	mutex_unlock(&pdrv->mempool_lock);
	return mempool;
}

void fsm_dp_mempool_free(struct fsm_dp_mempool *mempool)
{
	struct fsm_dp_drv *pdrv = NULL;
	enum fsm_dp_mem_type mempool_type;

	if (!mempool)
		return;

	pdrv = mempool->drv;
	mempool_type = mempool->type;
	fsm_dp_mempool_release_no_delay(mempool);
	pdrv->mempool[mempool_type] = NULL;
	wmb();
	return;
}

void fsm_dp_mempool_dev_destroy(struct fsm_dp_drv *pdrv)
{
	struct fsm_dp_mempool *mempool;
	int i;
	int j;
	unsigned int size;
	struct fsm_dp_mem *mem;


	for (j = 0; j < FSM_DP_MEM_TYPE_LAST; j++) {
		mempool = pdrv->mempool[j];
		if (!mempool)
			continue;
		if (!spin_trylock(&mempool->lock))
			continue;
		mem = &mempool->mem;
		if (mem->loc.dma_mapped) {
			size = mem->loc.size;
			for (i = 0; i < mem->loc.num_cluster; i++) {
				if (i ==  mem->loc.num_cluster - 1) {
					dma_unmap_single(
						pdrv->mhi.mhi_dev->mhi_cntrl->dev,
						mem->loc.cluster_dma_addr[i],
						size,
						mem->loc.direction);
				} else {
					dma_unmap_single(
						pdrv->mhi.mhi_dev->mhi_cntrl->dev,
						mem->loc.cluster_dma_addr[i],
						FSM_DP_MEMPOOL_CLUSTER_SIZE,
						mem->loc.direction);
					size -= FSM_DP_MEMPOOL_CLUSTER_SIZE;
				}
			}
			mem->loc.dma_mapped = false;
		}
		spin_unlock(&mempool->lock);
	}
}

int fsm_dp_mempool_get_cfg(
	struct fsm_dp_mempool *mempool,
	struct fsm_dp_mempool_cfg *cfg)
{
	if (unlikely(mempool == NULL || cfg == NULL))
		return -EINVAL;

	cfg->type = mempool->type;
	fsm_dp_mem_get_cfg(&mempool->mem, &cfg->mem);
	fsm_dp_ring_get_cfg(&mempool->ring, &cfg->ring);
	return 0;
}

/* For FSM_DP_MEM_TYPE_UL pool only */
int fsm_dp_mempool_put_buf(struct fsm_dp_mempool *mempool, void *vaddr)
{
	struct fsm_dp_mem *mem;
	unsigned long offset;
	int ret;
	struct fsm_dp_buf_cntrl *p;
	unsigned int buf_index;
	unsigned int cluster;

	if (unlikely(mempool == NULL || vaddr == NULL))
		return -EINVAL;

	mem = &mempool->mem;
	p = (vaddr - mem->buf_overhead_sz);
	buf_index = p->buf_index;
	if (buf_index >= mem->buf_cnt) {
		FSM_DP_ERROR("%s: buf_index %d exceed %d\n", __func__,
				buf_index, mem->buf_cnt);
		return -EINVAL;
	}
	cluster = buf_index / mem->loc.buf_per_cluster;
	offset = (cluster * FSM_DP_MEMPOOL_CLUSTER_SIZE) +
			(buf_index % mem->loc.buf_per_cluster) *
					fsm_dp_buf_true_size(mem);
#ifdef FSM_DP_BUFFER_FENCING
	if (p->signature != FSM_DP_BUFFER_SIG) {
		mempool->stats.invalid_buf_put++;
		FSM_DP_ERROR("%s: mempool %llx type %d buffer at "
			"offset %ld corrupted, sig %x, exp %x\n",
			__func__, (u64) mempool, mempool->type,
			offset, p->signature, FSM_DP_BUFFER_SIG);
		return -EINVAL;
	}
	if (p->fence != FSM_DP_BUFFER_FENCE_SIG) {
		mempool->stats.invalid_buf_put++;
		FSM_DP_ERROR("%s: mempool %llx type %d buffer at "
			"offset %ld corrupted, fence %x, exp %x\n",
			__func__, (u64) mempool, mempool->type,
			offset, p->fence, FSM_DP_BUFFER_FENCE_SIG);
		FSM_DP_ERROR("%s: vaddr %llx  p %llx\n",
			__func__, (u64) vaddr, (u64) p);
		return -EINVAL;
	}
	p->state = FSM_DP_BUF_STATE_KERNEL_FREE;
#endif
	offset += sizeof(struct fsm_dp_buf_cntrl);

	ret = fsm_dp_ring_write(&mempool->ring, (fsm_dp_ring_element_data_t)offset, 0);
	if (ret)
		mempool->stats.buf_put_err++;
	else
		mempool->stats.buf_put++;

	return ret;
}

/* For FSM_DP_MEM_TYPE_UL pool only */
void *fsm_dp_mempool_get_buf(struct fsm_dp_mempool *mempool,
				unsigned int *cluster,  unsigned int *c_offset)
{
	struct fsm_dp_mem *mem;
	fsm_dp_ring_element_data_t val;
	unsigned int flag;
	void *ptr;
#ifdef FSM_DP_BUFFER_FENCING
	struct fsm_dp_buf_cntrl *p;
#endif

	if (unlikely(mempool == NULL))
		return NULL;

	if (fsm_dp_ring_read(&mempool->ring, &val, &flag)) {
		mempool->stats.buf_get_err++;
		return NULL;
	}

	mem = &mempool->mem;
	*cluster = val >> FSM_DP_MEMPOOL_CLUSTER_SHIFT;
	*c_offset = val & FSM_DP_MEMPOOL_CLUSTER_MASK;
	ptr = (char *)mempool->mem.loc.cluster_kernel_addr[*cluster] +
								*c_offset;
	if ((*c_offset - mem->buf_overhead_sz) % fsm_dp_buf_true_size(mem)) {
		mempool->stats.invalid_buf_get++;
		FSM_DP_ERROR("%s: get unaligned buffer from ring, buf true size %d offset %d\n",
			__func__, fsm_dp_buf_true_size(mem), *c_offset);
		return NULL;
	}
#ifdef FSM_DP_BUFFER_FENCING
	p = ptr - mem->buf_overhead_sz;
	if (p->signature !=  FSM_DP_BUFFER_SIG) {
		mempool->stats.invalid_buf_get++;
		FSM_DP_ERROR("%s: mempool type %d buffer "
			"at %d corrupted, %x, exp %x\n",
			__func__, *c_offset, mempool->type,
			p->signature, FSM_DP_BUFFER_SIG);
		return NULL;
	}
	if (p->fence !=  FSM_DP_BUFFER_FENCE_SIG) {
		mempool->stats.invalid_buf_get++;
		FSM_DP_ERROR("%s: mempool type %d "
			"buffer at %d corrupted, fence %x, exp %x\n",
			__func__, *c_offset, mempool->type,
			p->fence, FSM_DP_BUFFER_FENCE_SIG);
		return NULL;
	}
#endif
	mempool->stats.buf_get++;
	return ptr;
}

struct fsm_dp_mempool *fsm_dp_find_mempool(
	struct fsm_dp_drv *pdrv,
	void *addr,
	bool tx,
	unsigned int *cluster)
{
	struct fsm_dp_mempool *mempool = NULL;
	struct fsm_dp_mem *mem;
	unsigned int mem_type, mem_type_last;

	if (unlikely(pdrv == NULL || addr == NULL))
		return NULL;

	if (tx) {
		mem_type = 0;
		mem_type_last = FSM_DP_MEM_TYPE_UL;
	} else {
		mem_type = FSM_DP_MEM_TYPE_UL;
		mem_type_last = FSM_DP_MEM_TYPE_LAST;
	}

	for (; mem_type < mem_type_last; mem_type++) {
		unsigned int clust;
		unsigned int remainder;
		unsigned int len;

		mempool = pdrv->mempool[mem_type];
		if (mempool) {
			if (mempool->signature != FSM_DP_MEMPOOL_SIG) {
				FSM_DP_ERROR(
					"%s: mempool %p signature 0x%x error, expect 0x%x\n",
					__func__,
					mempool,
					mempool->signature,
					FSM_DP_MEMPOOL_SIG);
				continue;
			}
			mem = &mempool->mem;
			remainder = mem->loc.size;
			for (clust = 0; clust < mem->loc.num_cluster;
							clust++) {
				if (clust == mem->loc.num_cluster - 1)
					len = remainder;
				else
					len = FSM_DP_MEMPOOL_CLUSTER_SIZE;
				if (vaddr_in_range(
					addr,
					mem->loc.cluster_kernel_addr[clust],
					len)) {
					*cluster = clust;
					return mempool;
				}

				remainder -= len;
			}
		}
	}
	return NULL;
}

