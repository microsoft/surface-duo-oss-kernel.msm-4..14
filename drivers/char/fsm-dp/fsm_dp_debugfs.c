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
#include "fsm_dp.h"
#ifdef CONFIG_DEBUG_FS

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/atomic.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#define MEM_DUMP_COL_WIDTH 16
#define MAX_MEM_DUMP_SIZE 256

#define DEFINE_DEBUGFS_OPS(name, __read, __write)		\
static int name ##_open(struct inode *inode, struct file *file)	\
{								\
	return single_open(file, __read, inode->i_private);	\
}								\
static const struct file_operations name ##_ops = {		\
	.open	 = name ## _open,				\
	.read = seq_read,					\
	.write = __write,					\
	.llseek = seq_lseek,					\
	.release = single_release,				\
}

#ifdef CONFIG_FSM_DP_TEST
static int debugfs_create_testring_dir(struct dentry *, struct fsm_dp_drv *);
#endif

static struct dentry *__dent;

static int __fsm_dp_rxqueue_vma_dump(
	struct seq_file *s,
	struct fsm_dp_rxqueue_vma *rxq_vma)
{
	if (rxq_vma->vma) {
		struct vm_area_struct *vma = rxq_vma->vma;

		seq_printf(s, "    Type:               %s\n",
			   fsm_dp_rx_type_to_str(rxq_vma->type));
		seq_printf(s, "    RefCnt:             %d\n",
			   atomic_read(&rxq_vma->refcnt));
		seq_printf(s,
			   "        vm_start:       %lx\n"
			   "        vm_end:         %lx\n"
			   "        vm_pgoff:       %lx\n"
			   "        vm_flags:       %lx\n",
			   vma->vm_start,
			   vma->vm_end,
			   vma->vm_pgoff,
			   vma->vm_flags);
	}
	return 0;
}

static int __fsm_dp_mempool_vma_dump(
	struct seq_file *s,
	struct fsm_dp_mempool_vma *mempool_vma)
{
	struct fsm_dp_mempool *mempool = *mempool_vma->pp_mempool;
	struct vm_area_struct *vma;
	int i;

	if (mempool)
		seq_printf(s, "    Type:               %s\n",
			   fsm_dp_mem_type_to_str(mempool->type));

	for (i = 0; i < FSM_DP_MMAP_TYPE_LAST; i++) {
		if (mempool_vma->vma[i]) {
			vma = mempool_vma->vma[i];
			seq_printf(s, "    VMA[%d]:             %s\n",
				   i, fsm_dp_mmap_type_to_str(i));
			seq_printf(s,
				   "        vm_start:       %lx\n"
				   "        vm_end:         %lx\n"
				   "        vm_pgoff:       %lx\n"
				   "        vm_flags:       %lx\n",
				   vma->vm_start,
				   vma->vm_end,
				   vma->vm_pgoff,
				   vma->vm_flags);
			seq_printf(s, "        refcnt:         %d\n",
				   atomic_read(&mempool_vma->refcnt[i]));
		}
	}
	return 0;
}

static int __fsm_dp_ring_opstats_dump(
	struct seq_file *s,
	struct fsm_dp_ring_opstats *stats)
{
	seq_puts(s, "Read:\n");
	seq_printf(s, "    Ok:                %lu\n", stats->read_ok);
	seq_printf(s, "    Empty:             %lu\n", stats->read_empty);
	seq_printf(s, "    TailUpdt:          %lu\n", stats->cons_tail_updt);
	seq_printf(s, "    TailNoUpdt:        %lu\n", stats->cons_tail_no_updt);
	seq_printf(s, "    TailUpdtBackOff:   %lu\n",
		   stats->cons_tail_updt_backoff);
	seq_printf(s, "    TailUpdtStop:      %lu\n",
		   stats->cons_tail_updt_stop);
	seq_printf(s, "    HdrUpdtRetry:      %lu\n",
		   stats->cons_head_updt_retry);
	seq_puts(s, "Write:\n");
	seq_printf(s, "    Ok:                %lu\n", stats->write_ok);
	seq_printf(s, "    Full:              %lu\n", stats->write_full);
	seq_printf(s, "    TailUpdt:          %lu\n", stats->prod_tail_updt);
	seq_printf(s, "    TailNoUpdt:        %lu\n", stats->prod_tail_no_updt);
	seq_printf(s, "    TailUpdtBackOff:   %lu\n",
		   stats->prod_tail_updt_backoff);
	seq_printf(s, "    TailUpdtStop:      %lu\n",
		   stats->prod_tail_updt_stop);
	seq_printf(s, "    HdrUpdtRetry:      %lu\n",
		   stats->prod_head_updt_retry);
	return 0;
}

static int __fsm_dp_ring_runtime_dump(
	struct seq_file *s,
	struct fsm_dp_ring *ring)
{
	seq_printf(s, "ProdHdr:                %u\n", *ring->prod_head);
	seq_printf(s, "ProdTail:               %u\n", *ring->prod_tail);
	seq_printf(s, "ConsHdr:                %u\n", *ring->cons_head);
	seq_printf(s, "ConsTail:               %u\n", *ring->cons_tail);
	seq_printf(s, "NumOfElementAvail:      %u\n",
		   (*ring->prod_head - *ring->cons_tail) & (ring->size - 1));
	return 0;
}

static int __fsm_dp_ring_config_dump(
	struct seq_file *s,
	struct fsm_dp_ring *ring)
{
	seq_printf(s, "Ring %llx MemoryAlloc:\n", (u64) ring);
	seq_printf(s, "         AllocAddr:     %llx\n", (u64) ring->loc.base);
	seq_printf(s, "         AllocSize:     0x%08lx\n", ring->loc.size);
	seq_printf(s, "         MmapCookie:    0x%08x\n", ring->loc.cookie);
	seq_printf(s, "Size:                   0x%x\n", ring->size);
	seq_printf(s, "ProdHdr:                %llx\n", (u64) ring->prod_head);
	seq_printf(s, "ProdTail:               %llx\n", (u64) ring->prod_tail);
	seq_printf(s, "ConsHdr:                %llx\n", (u64) ring->cons_head);
	seq_printf(s, "ConsTail:               %llx\n", (u64) ring->cons_tail);
	seq_printf(s, "RingBuf:                %llx\n", (u64) ring->element);
	return 0;
}

static int debugfs_loopback_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_loopback_task *task =
		(struct fsm_dp_loopback_task *)s->private;

	seq_puts(s, "TX Loopback\n");
	seq_printf(s, "    Count:              %lu\n", task->stats.tx_cnt);
	seq_printf(s, "    Enqueue:            %lu\n", task->stats.tx_enque);
	seq_printf(s, "    Error:              %lu\n", task->stats.tx_err);
	seq_printf(s, "    Drop:               %lu\n", task->stats.tx_drop);
	seq_puts(s, "RX Loopback\n");
	seq_printf(s, "    Count:              %lu\n", task->stats.rx_cnt);
	seq_printf(s, "    Enqueue:            %lu\n", task->stats.rx_enque);
	seq_printf(s, "    Error:              %lu\n", task->stats.rx_err);
	seq_printf(s, "    Drop:               %lu\n", task->stats.rx_drop);
	seq_printf(s, "Run:                    %lu\n", task->stats.run);
	seq_printf(s, "Schedule:               %lu\n", task->stats.sched);

	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_loopback, debugfs_loopback_read, NULL);

static int debugfs_rxq_refcnt_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_rxqueue *rxq = (struct fsm_dp_rxqueue *)s->private;

	if (rxq->inited)
		seq_printf(s, "%d\n", atomic_read(&rxq->refcnt));

	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_rxq_refcnt, debugfs_rxq_refcnt_read, NULL);

static int debugfs_rxq_opstats_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_rxqueue *rxq = (struct fsm_dp_rxqueue *)s->private;

	if (rxq->inited)
		__fsm_dp_ring_opstats_dump(s, &rxq->ring.opstats);

	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_rxq_opstats, debugfs_rxq_opstats_read, NULL);

static int debugfs_rxq_config_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_rxqueue *rxq = (struct fsm_dp_rxqueue *)s->private;

	if (rxq->inited) {
		seq_printf(s, "Type:                   %s\n",
			   fsm_dp_rx_type_to_str(rxq->type));
		__fsm_dp_ring_config_dump(s, &rxq->ring);
	}

	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_rxq_config, debugfs_rxq_config_read, NULL);

static int debugfs_rxq_runtime_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_rxqueue *rxq = (struct fsm_dp_rxqueue *)s->private;

	if (rxq->inited)
		__fsm_dp_ring_runtime_dump(s, &rxq->ring);

	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_rxq_runtime, debugfs_rxq_runtime_read, NULL);

static unsigned int __mem_dump_size[FSM_DP_MEM_TYPE_LAST];
static unsigned int __mem_offset[FSM_DP_MEM_TYPE_LAST];

static int debugfs_mem_data_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_mempool *mempool =
		*((struct fsm_dp_mempool **)s->private);

	if (mempool) {
		struct fsm_dp_mem *mem = &mempool->mem;
		unsigned int n = __mem_dump_size[mempool->type];
		unsigned int offset = __mem_offset[mempool->type];
		unsigned int i, j;
		unsigned int cluster, c_offset;
		unsigned char *data = (unsigned char *)mem->loc.base + offset;

		data = fsm_dp_mem_offset_addr(mem, offset, &cluster, &c_offset);
		if (data == NULL)
			return 0;
		if (n > (mem->loc.size - offset))
			n = mem->loc.size - offset;

		for (i = 0; i < offset % MEM_DUMP_COL_WIDTH; i++)
			seq_puts(s, "   ");

		for (j = 0; j < n; j++, i++) {
			if (i && !(i % MEM_DUMP_COL_WIDTH))
				seq_puts(s, "\n");
			seq_printf(s, "%02x ", *data);
			data++;
			c_offset++;
			if (c_offset >= FSM_DP_MEMPOOL_CLUSTER_SIZE) {
				c_offset = 0;
				cluster++;
				data = mem->loc.cluster_kernel_addr[cluster];
			}
		}
		seq_puts(s, "\n");
	}
	return 0;
}

static ssize_t debugfs_mem_data_write(
	struct file *fp,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	struct fsm_dp_mempool *mempool = *((struct fsm_dp_mempool **)
			(((struct seq_file *)fp->private_data)->private));

	if (mempool) {
		struct fsm_dp_mem *mem = &mempool->mem;
		unsigned int value = 0;
		unsigned int *data;
		unsigned int offset = __mem_offset[mempool->type];
		unsigned int cluster, c_offset;

		if (kstrtouint_from_user(buf, count, 0, &value))
			return -EFAULT;
		data = (unsigned int *)fsm_dp_mem_offset_addr(
				mem, offset, &cluster, &c_offset);
		if (data == NULL)
			return count;
		*data = value;
	}
	return count;
}
DEFINE_DEBUGFS_OPS(debugfs_mem_data, debugfs_mem_data_read,
		   debugfs_mem_data_write);

static int debugfs_mem_dump_size_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_mempool *mempool =
		*((struct fsm_dp_mempool **)s->private);

	if (mempool)
		seq_printf(s, "%u\n", __mem_dump_size[mempool->type]);
	return 0;
}

static ssize_t debugfs_mem_dump_size_write(
	struct file *fp,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	struct fsm_dp_mempool *mempool = *((struct fsm_dp_mempool **)
			(((struct seq_file *)fp->private_data)->private));
	unsigned int value = 0;

	if (!mempool)
		goto done;

	if (kstrtouint_from_user(buf, count, 0, &value))
		return -EFAULT;

	if (value > MAX_MEM_DUMP_SIZE)
		return -EINVAL;

	__mem_dump_size[mempool->type] = value;
done:
	return count;
}
DEFINE_DEBUGFS_OPS(debugfs_mem_dump_size, debugfs_mem_dump_size_read,
		   debugfs_mem_dump_size_write);

static int debugfs_mem_offset_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_mempool *mempool =
		*((struct fsm_dp_mempool **)s->private);

	if (mempool)
		seq_printf(s, "0x%08x\n", __mem_offset[mempool->type]);
	return 0;
}

static ssize_t debugfs_mem_offset_write(
	struct file *fp,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	struct fsm_dp_mempool *mempool = *((struct fsm_dp_mempool **)
			(((struct seq_file *)fp->private_data)->private));

	if (mempool) {
		struct fsm_dp_mem *mem = &mempool->mem;
		unsigned int value = 0;

		if (kstrtouint_from_user(buf, count, 0, &value))
			return -EFAULT;

		if (value >= mem->loc.size)
			return -EINVAL;
		if (value & 3)
			return -EINVAL;

		__mem_offset[mempool->type] = value;
	}
	return count;
}
DEFINE_DEBUGFS_OPS(debugfs_mem_offset, debugfs_mem_offset_read,
		   debugfs_mem_offset_write);

static int debugfs_mem_config_show(struct seq_file *s, void *unused)
{
	struct fsm_dp_mempool *mempool =
		*((struct fsm_dp_mempool **)s->private);
	int i;

	if (mempool) {
		struct fsm_dp_mem *mem = &mempool->mem;

		seq_puts(s, "MemoryAlloc:\n");
		seq_printf(s, "    AllocSize:     0x%08lx\n",
			   mem->loc.size);
		seq_printf(s, "    Total Cluster:  %d\n",
			   mem->loc.num_cluster);
		seq_printf(s, "    Cluster Size:  0x%x\n",
			   FSM_DP_MEMPOOL_CLUSTER_SIZE);
		for (i = 0; i < mem->loc.num_cluster; i++)
			seq_printf(s, "    Cluster %d Addr: %llx\n", i,
					(u64) mem->loc.cluster_kernel_addr[i]);
		seq_printf(s, "    Buffer Per Cluster:  %d\n",
			   mem->loc.buf_per_cluster);
		seq_printf(s, "    Last Cluster Order:  %d\n",
			   mem->loc.last_cl_order);
		seq_printf(s, "    MmapCookie:    %08x\n",
			   mem->loc.cookie);
		seq_printf(s, "BufSize:                0x%x\n", mem->buf_sz);
		seq_printf(s, "BufCount:               0x%x\n", mem->buf_cnt);
		seq_printf(s, "BufTrueSize:            0x%x\n",
			   fsm_dp_buf_true_size(mem));

	}

	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_mem_config, debugfs_mem_config_show, NULL);

static int debugfs_ring_config_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_mempool *mempool =
		*((struct fsm_dp_mempool **)s->private);

	if (mempool)
		__fsm_dp_ring_config_dump(s, &mempool->ring);
	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_ring_config, debugfs_ring_config_read, NULL);

static int debugfs_ring_runtime_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_mempool *mempool =
		*((struct fsm_dp_mempool **)s->private);

	if (mempool)
		__fsm_dp_ring_runtime_dump(s, &mempool->ring);
	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_ring_runtime, debugfs_ring_runtime_read, NULL);

static int debugfs_ring_opstats_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_mempool *mempool =
		*((struct fsm_dp_mempool **)s->private);

	if (mempool)
		__fsm_dp_ring_opstats_dump(s, &mempool->ring.opstats);
	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_ring_opstats, debugfs_ring_opstats_read, NULL);

unsigned long __ring_index[FSM_DP_MEM_TYPE_LAST];

static int debugfs_ring_index_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_mempool *mempool =
		*((struct fsm_dp_mempool **)s->private);

	if (mempool)
		seq_printf(s, "%lu\n", __ring_index[mempool->type]);
	return 0;
}

static ssize_t debugfs_ring_index_write(
	struct file *fp,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	struct fsm_dp_mempool *mempool = *((struct fsm_dp_mempool **)
			(((struct seq_file *)fp->private_data)->private));
	unsigned int value = 0;

	if (!mempool)
		goto done;

	if (kstrtouint_from_user(buf, count, 0, &value))
		return -EFAULT;

	if (value >= mempool->ring.size)
		return -EINVAL;

	__ring_index[mempool->type] = value;
done:
	return count;
}
DEFINE_DEBUGFS_OPS(debugfs_ring_index, debugfs_ring_index_read,
		   debugfs_ring_index_write);

static int debugfs_ring_data_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_mempool *mempool =
		*((struct fsm_dp_mempool **)s->private);

	if (mempool) {
		fsm_dp_ring_element_t *elem_p;

		elem_p = (mempool->ring.element + __ring_index[mempool->type]);

		seq_printf(s, "0x%lx\n", elem_p->element_data);
	}
	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_ring_data, debugfs_ring_data_read, NULL);

static int debugfs_mempool_status_show(struct seq_file *s, void *unused)
{
	struct fsm_dp_mempool *mempool =
		*((struct fsm_dp_mempool **)s->private);

	if (mempool) {
		seq_printf(s, "BufPut:                 %lu\n",
			   mempool->stats.buf_put);
		seq_printf(s, "InvalidBufPut:          %lu\n",
			   mempool->stats.invalid_buf_put);
		seq_printf(s, "ErrBufPut:              %lu\n",
			   mempool->stats.buf_put_err);
		seq_printf(s, "BufGet:                 %lu\n",
			   mempool->stats.buf_get);
		seq_printf(s, "InvalidBufGet:          %lu\n",
			   mempool->stats.invalid_buf_get);
		seq_printf(s, "ErrBufGet:              %lu\n",
			   mempool->stats.buf_get_err);
	}
	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_mempool_status, debugfs_mempool_status_show, NULL);

static int debugfs_mempool_state_show(struct seq_file *s, void *unused)
{
	struct fsm_dp_mempool *mempool =
		*((struct fsm_dp_mempool **)s->private);
	unsigned long state_cnt[FSM_DP_BUF_STATE_LAST];
	unsigned long buf_bad = 0;
	unsigned long unknown_state = 0;
	int i;

	memset(state_cnt, 0, sizeof(state_cnt));
	if (mempool) {
		struct fsm_dp_mem *mem = &mempool->mem;
		struct fsm_dp_buf_cntrl *p;

		for (i = 0; i < mem->buf_cnt; i++) {
			p = (struct fsm_dp_buf_cntrl *)
				fsm_dp_mem_rec_addr(mem, i);
			if (p == NULL)
				return 0;
#ifdef FSM_DP_BUFFER_FENCING
			if (p->signature != FSM_DP_BUFFER_SIG ||
					p->fence != FSM_DP_BUFFER_FENCE_SIG ||
					p->buf_index != i)
				buf_bad++;
			else if (p->state >= FSM_DP_BUF_STATE_LAST)
#else
			if (p->state >= FSM_DP_BUF_STATE_LAST)
#endif
				unknown_state++;
			else
				state_cnt[p->state]++;

		}

		seq_printf(s, "Total Buf:                  %u\n",
			   mem->buf_cnt);
		seq_printf(s, "Buf Real Size:              %u\n",
			   mem->buf_sz + mem->buf_overhead_sz);
		seq_printf(s, "Buf Corrupted:              %lu\n",
			   buf_bad);
		seq_printf(s, "Buf Unknown State:          %lu\n",
			   unknown_state);

		for (i = 0; i < FSM_DP_BUF_STATE_LAST; i++) {
			if (state_cnt[i]) {
				seq_printf(s, "Buf State %s:        ",
						fsm_dp_buf_state_to_str(i));
				seq_printf(s, "                    %lu\n",
						state_cnt[i]);
			}
		}
	}
	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_mempool_state, debugfs_mempool_state_show, NULL);

static int debugfs_mempool_active_show(struct seq_file *s, void *unused)
{
	struct fsm_dp_drv *drv = (struct fsm_dp_drv *)s->private;
	unsigned int type;

	for (type = 0; type < FSM_DP_MEM_TYPE_LAST; type++) {
		if (drv->mempool[type])
			seq_printf(s, "%s ", fsm_dp_mem_type_to_str(type));
	}
	seq_puts(s, "\n");
	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_mempool_active, debugfs_mempool_active_show, NULL);

static int debugfs_mempool_info_show(struct seq_file *s, void *unused)
{
	struct fsm_dp_mempool *mempool =
		*((struct fsm_dp_mempool **)s->private);

	if (mempool) {
		seq_printf(s, "Driver:                 %llx\n",
							(u64) mempool->drv);
		seq_printf(s, "MemPool:                %llx\n",
							(u64) mempool);
		seq_printf(s, "Type:                   %s\n",
			   fsm_dp_mem_type_to_str(mempool->type));
		seq_printf(s, "Ref:                    %d\n",
			   atomic_read(&mempool->ref));
	}
	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_mempool_info, debugfs_mempool_info_show, NULL);

static int debugfs_mhi_show(struct seq_file *s, void *unused)
{
	struct fsm_dp_drv *drv = (struct fsm_dp_drv *)s->private;
	struct fsm_dp_mhi *mhi = &drv->mhi;

	seq_printf(s, "MHIDevice:              %llx\n", (u64) mhi->mhi_dev);
	seq_puts(s, "Stats:\n");
	seq_printf(s, "    TX:                 %lu\n", mhi->stats.tx_cnt);
	seq_printf(s, "    TX_ACKED:           %lu\n", mhi->stats.tx_acked);
	seq_printf(s, "    TX_ERR:             %lu\n", mhi->stats.tx_err);
	seq_printf(s, "    RX:                 %lu\n", mhi->stats.rx_cnt);
	seq_printf(s, "    RX_ERR:             %lu\n", mhi->stats.rx_err);
	seq_printf(s, "    RX_OUT_OF_BUF:      %lu\n",
		   mhi->stats.rx_out_of_buf);
	seq_printf(s, "    RX_REPLENISH:       %lu\n",
		   mhi->stats.rx_replenish);
	seq_printf(s, "    RX_REPLENISH_ERR:   %lu\n",
		   mhi->stats.rx_replenish_err);
	seq_printf(s, "    RX_OUTOFBUF_DROP:   %lu\n",
		   mhi->stats.rx_outofbuf_drop);
	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_mhi, debugfs_mhi_show, NULL);

static int debugfs_cdev_show(struct seq_file *s, void *unused)
{
	struct fsm_dp_drv *drv = (struct fsm_dp_drv *)s->private;
	struct fsm_dp_cdev *cdev;
	int n = 0;
	int i;

	mutex_lock(&drv->cdev_lock);
	list_for_each_entry(cdev, &drv->cdev_head, list) {
		seq_printf(s, "CDEV(%d)\n", n++);
		seq_printf(s, "Driver:                 %llx\n",
							(u64) cdev->pdrv);
		seq_printf(s, "Cdev:                   %llx\n",
							(u64) cdev);
		seq_printf(s, "PID:                    %d\n",
							cdev->pid);
		seq_printf(s, "TX_Mode:                %d\n",
							cdev->tx_mode);

		for (i = 0; i < FSM_DP_MEM_TYPE_LAST; i++) {
			seq_printf(s, "MemPoolVMA[%d]\n", i);
			__fsm_dp_mempool_vma_dump(s, &cdev->mempool_vma[i]);
		}
		seq_puts(s, "RxQueue\n");
		for (i = 0; i < FSM_DP_RX_TYPE_LAST; i++)
			__fsm_dp_rxqueue_vma_dump(s, &cdev->rxqueue_vma[i]);
	}
	mutex_unlock(&drv->cdev_lock);

	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_cdev, debugfs_cdev_show, NULL);

static int debugfs_drv_status_show(struct seq_file *s, void *unused)
{
	struct fsm_dp_drv *drv = (struct fsm_dp_drv *)s->private;
	struct fsm_dp_core_stats *stats = &drv->stats;

	seq_printf(s, "TX:             %lu\n", stats->tx_cnt);
	seq_printf(s, "TX_ERR:         %lu\n", stats->tx_err);
	seq_printf(s, "RX:             %lu\n", stats->rx_cnt);
	seq_printf(s, "RX_BADMSG:      %lu\n", stats->rx_badmsg);
	seq_printf(s, "RX_DROP:        %lu\n", stats->rx_drop);
	seq_printf(s, "RX_INT:         %lu\n", stats->rx_int);
	seq_printf(s, "RX_BUDGET_OVF:  %lu\n", stats->rx_budget_overflow);
	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_drv_status, debugfs_drv_status_show, NULL);

static int debugfs_drv_show(struct seq_file *s, void *unused)
{
	struct fsm_dp_drv *drv = (struct fsm_dp_drv *)s->private;
	struct platform_device *pdev = to_platform_device(drv->dev);

	seq_printf(s, "Driver:         %llx\n", (u64) drv);
	seq_printf(s, "Name:           %s\n", pdev->name);
	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_drv, debugfs_drv_show, NULL);


static int debugfs_traffic_status_show(struct seq_file *s, void *unused)
{
	struct fsm_dp_traffic *traffic = (struct fsm_dp_traffic *)s->private;

	if (!traffic->traffic_timestamp)
		return 0;
	if (traffic->ul_cnt)
		seq_printf(s, "Avg FSM-DP UL ns    %6llu\n",
			traffic->ul_ktime / traffic->ul_cnt);
	if (traffic->dl_cnt)
		seq_printf(s, "Avg FSM-DP DL ns    %6llu\n",
			traffic->dl_ktime / traffic->dl_cnt);
	return 0;
}

DEFINE_DEBUGFS_OPS(debugfs_traffic_status, debugfs_traffic_status_show, NULL);

static int debugfs_traffic_timestamp_get(struct seq_file *s, void *unused)
{
	struct fsm_dp_traffic *traffic = (struct fsm_dp_traffic *)s->private;

	seq_printf(s, "timestamp    %d\n", traffic->traffic_timestamp);
	return 0;
}

static ssize_t debugfs_traffic_timestamp_set(
	struct file *fp,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	struct fsm_dp_traffic *traffic = (struct fsm_dp_traffic *)
			(((struct seq_file *)fp->private_data)->private);
	unsigned int enable = 0;

	if (kstrtouint_from_user(buf, count, 0, &enable))
		return -EFAULT;
	traffic->traffic_timestamp = enable;
	if (enable) {
		traffic->dl_ktime = 0;
		traffic->ul_ktime = 0;
		traffic->dl_cnt = 0;
		traffic->ul_cnt = 0;
	}
	return count;
}

DEFINE_DEBUGFS_OPS(
	debugfs_traffic_timestamp,
	debugfs_traffic_timestamp_get,
	debugfs_traffic_timestamp_set);

static unsigned long fsm_dp_gap_array[FSM_DP_TRAFFIC_ARRAY_SIZE];
static int debugfs_traffic_ul_get(struct seq_file *s, void *unused)
{
	struct fsm_dp_traffic *traffic = (struct fsm_dp_traffic *)s->private;
	unsigned long max_gap = 0;
	unsigned long min_gap = 0xffffffff;
	unsigned long total_gap = 0;
	unsigned long gap;

	ktime_t max_service = 0;
	ktime_t min_service = 0xffffffff;
	ktime_t total_service = 0;
	ktime_t service;
	int i;
	unsigned int dist[5];

	seq_printf(s, "UL collect done:       %d\n\n",
					traffic->ul_traffic_collect_done);
	if (!traffic->ul_traffic_collect_done)
		return 0;

	for (i = 0; i < FSM_DP_TRAFFIC_ARRAY_SIZE; i++) {

		service = traffic->ul_traffic[i].complete_ktime -
				traffic->ul_traffic[i].arrival_ktime;


		if (service >= max_service)
			max_service = service;
		if (service <= min_service)
			min_service = service;
		total_service += service;
	}
	seq_printf(s, "UL max sericve time:  %lld ns\n", max_service);
	seq_printf(s, "UL min service time: %lld ns\n", min_service);
	seq_printf(s, "UL avg service time: %lld ns\n",
				total_service / FSM_DP_TRAFFIC_ARRAY_SIZE);

	dist[0] = dist[1] = dist[2] = dist[3] = dist[4] = 0;
	for (i = 1; i < FSM_DP_TRAFFIC_ARRAY_SIZE; i++) {
		gap = traffic->ul_traffic[i].arrival_ktime -
			traffic->ul_traffic[i - 1].arrival_ktime;
		fsm_dp_gap_array[i - 1] = gap / 1000;
		if (gap >= max_gap)
			max_gap = gap;
		if (gap <= min_gap)
			min_gap = gap;
		total_gap += gap;
		if (gap  >= 1000000)
			dist[4]++;
		else if (gap < 100000)
			dist[0]++;
		else if (gap < 200000)
			dist[1]++;
		else if (gap  < 500000)
			dist[2]++;
		else
			dist[3]++;
	}
	seq_printf(s, "UL max gap:       %ld us\n",
						max_gap / 1000);
	seq_printf(s, "UL min gap:       %ld us\n",
						min_gap / 1000);
	seq_printf(s, "UL avg gap:       %ld us\n",
				total_gap / (1000 *
					(FSM_DP_TRAFFIC_ARRAY_SIZE - 1)));

	seq_printf(s, "UL dist: < 100 us %d , < 200 us %d, < 500 us %d, < 1000 us %d, > 1000 us %d\n",
			dist[0], dist[1], dist[2], dist[3], dist[4]);

	for (i = 0; i < (FSM_DP_TRAFFIC_ARRAY_SIZE / 8) - 1; i++) {
		pr_info("UL gap in us: %ld %ld %ld %ld %ld %ld %ld %ld\n",
			fsm_dp_gap_array[i * 8], fsm_dp_gap_array[i * 8 + 1],
			fsm_dp_gap_array[i * 8 + 2], fsm_dp_gap_array[i * 8 + 3],
			fsm_dp_gap_array[i * 8 + 4], fsm_dp_gap_array[i * 8 + 5],
			fsm_dp_gap_array[i * 8 + 6], fsm_dp_gap_array[i * 8 + 7]);
	}
	i = (FSM_DP_TRAFFIC_ARRAY_SIZE / 8) - 1;
	pr_info("UL gap in us: %ld %ld %ld %ld %ld %ld %ld\n",
			fsm_dp_gap_array[i * 8], fsm_dp_gap_array[i * 8 + 1],
			fsm_dp_gap_array[i * 8 + 2], fsm_dp_gap_array[i * 8 + 3],
			fsm_dp_gap_array[i * 8 + 4], fsm_dp_gap_array[i * 8 + 5],
			fsm_dp_gap_array[i * 8 + 6]);
	return 0;
}

static ssize_t debugfs_traffic_ul_set(
	struct file *fp,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	struct fsm_dp_traffic *traffic = (struct fsm_dp_traffic *)
			(((struct seq_file *)fp->private_data)->private);
	unsigned int enable = 0;

	if (kstrtouint_from_user(buf, count, 0, &enable))
		return -EFAULT;
	if (enable && traffic->traffic_timestamp) {
		traffic->ul_traffic_index = -1;
		traffic->ul_traffic_collect_done = false;
		traffic->ul_traffic_collect = true;
	} else {
		traffic->ul_traffic_collect = false;
	}
	return count;
}

DEFINE_DEBUGFS_OPS(
	debugfs_traffic_ul,
	debugfs_traffic_ul_get,
	debugfs_traffic_ul_set);

static int debugfs_traffic_dl_get(struct seq_file *s, void *unused)
{
	struct fsm_dp_traffic *traffic = (struct fsm_dp_traffic *)s->private;
	unsigned long max_gap = 0;
	unsigned long min_gap = 0xffffffff;
	unsigned long total_gap = 0;
	unsigned long gap;

	ktime_t max_service = 0;
	ktime_t min_service = 0xffffffff;
	ktime_t total_service = 0;
	ktime_t service;
	int i;
	unsigned int dist[5];

	seq_printf(s, "DL collect done:       %d\n\n",
					traffic->dl_traffic_collect_done);
	if (!traffic->dl_traffic_collect_done)
		return 0;

	for (i = 0; i < FSM_DP_TRAFFIC_ARRAY_SIZE; i++) {
		service = traffic->dl_traffic[i].complete_ktime -
				traffic->dl_traffic[i].arrival_ktime;

		if (service >= max_service)
			max_service = service;
		if (service <= min_service)
			min_service = service;
		total_service += service;
	}
	seq_printf(s, "DL max sericve time:  %lld us\n",
					max_service  / 1000);
	seq_printf(s, "DL min service time: %lld us\n",
					min_service  / 1000);
	seq_printf(s, "DL avg service time: %lld us\n",
				total_service / (1000 *
					FSM_DP_TRAFFIC_ARRAY_SIZE));

	dist[0] = dist[1] = dist[2] = dist[3] = dist[4] = 0;
	for (i = 1; i < FSM_DP_TRAFFIC_ARRAY_SIZE; i++) {
		gap = traffic->dl_traffic[i].arrival_ktime -
			traffic->dl_traffic[i - 1].arrival_ktime;
		fsm_dp_gap_array[i - 1] = gap / 1000;
		if (gap >= max_gap)
			max_gap = gap;
		if (gap <= min_gap)
			min_gap = gap;
		total_gap += gap;
		if (gap  >= 1000000)
			dist[4]++;
		else if (gap < 100000)
			dist[0]++;
		else if (gap < 200000)
			dist[1]++;
		else if (gap  < 500000)
			dist[2]++;
		else
			dist[3]++;
	}
	seq_printf(s, "DL max gap:       %ld us\n",
						max_gap / 1000);
	seq_printf(s, "DL min gap:       %ld us\n",
						min_gap / 1000);
	seq_printf(s, "DL avg gap:       %ld us\n",
				total_gap / (1000 *
					(FSM_DP_TRAFFIC_ARRAY_SIZE - 1)));

	seq_printf(s, "UL dist: < 100 us %d , < 200 us %d, < 500 us %d, < 1000 us %d, > 1000 us %d\n",
			dist[0], dist[1], dist[2], dist[3], dist[4]);

	for (i = 0; i < (FSM_DP_TRAFFIC_ARRAY_SIZE / 8) - 1; i++) {
		pr_info("DL gap in us: %ld %ld %ld %ld %ld %ld %ld %ld\n",
			fsm_dp_gap_array[i * 8], fsm_dp_gap_array[i * 8 + 1],
			fsm_dp_gap_array[i * 8 + 2], fsm_dp_gap_array[i * 8 + 3],
			fsm_dp_gap_array[i * 8 + 4], fsm_dp_gap_array[i * 8 + 5],
			fsm_dp_gap_array[i * 8 + 6], fsm_dp_gap_array[i * 8 + 7]);
	}
	i = (FSM_DP_TRAFFIC_ARRAY_SIZE / 8) - 1;
	pr_info("DL gap in us: %ld %ld %ld %ld %ld %ld %ld\n",
			fsm_dp_gap_array[i * 8], fsm_dp_gap_array[i * 8 + 1],
			fsm_dp_gap_array[i * 8 + 2], fsm_dp_gap_array[i * 8 + 3],
			fsm_dp_gap_array[i * 8 + 4], fsm_dp_gap_array[i * 8 + 5],
			fsm_dp_gap_array[i * 8 + 6]);
	return 0;
}

static ssize_t debugfs_traffic_dl_set(
	struct file *fp,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	struct fsm_dp_traffic *traffic = (struct fsm_dp_traffic *)
			(((struct seq_file *)fp->private_data)->private);
	unsigned int enable = 0;

	if (kstrtouint_from_user(buf, count, 0, &enable))
		return -EFAULT;
	if (enable && traffic->traffic_timestamp) {
		traffic->dl_traffic_index = -1;
		traffic->dl_traffic_collect_done = false;
		traffic->dl_traffic_collect = true;
	} else {
		traffic->dl_traffic_collect = false;
	}
	return count;
}

DEFINE_DEBUGFS_OPS(
	debugfs_traffic_dl,
	debugfs_traffic_dl_get,
	debugfs_traffic_dl_set);

static int debugfs_create_loopback_dir(struct dentry *parent,
				       struct fsm_dp_drv *drv)
{
	struct dentry *entry = NULL, *dentry = NULL;

	dentry = debugfs_create_dir("loopback", parent);
	if (IS_ERR(dentry))
		return -ENOMEM;

	entry = debugfs_create_file("status", 0444, dentry,
				    &drv->loopback,
				    &debugfs_loopback_ops);
	if (!entry)
		return -ENOMEM;
	return 0;
}

static int debugfs_create_traffic_dir(struct dentry *parent,
				       struct fsm_dp_drv *drv)
{
	struct dentry *entry = NULL, *dentry = NULL;

	dentry = debugfs_create_dir("traffic", parent);
	if (IS_ERR(dentry))
		return -ENOMEM;

	entry = debugfs_create_file("time_stamp", 0444, dentry,
				    &drv->traffic,
				    &debugfs_traffic_timestamp_ops);
	if (!entry)
		return -ENOMEM;

	entry = debugfs_create_file("dl", 0444, dentry,
				    &drv->traffic,
				    &debugfs_traffic_dl_ops);
	if (!entry)
		return -ENOMEM;

	entry = debugfs_create_file("ul", 0444, dentry,
				    &drv->traffic,
				    &debugfs_traffic_ul_ops);
	if (!entry)
		return -ENOMEM;

	entry = debugfs_create_file("status", 0444, dentry,
				    &drv->traffic,
				    &debugfs_traffic_status_ops);
	if (!entry)
		return -ENOMEM;
	return 0;
}

static int debugfs_create_rxq_dir(struct dentry *parent, struct fsm_dp_drv *drv)
{
	struct dentry *entry = NULL, *dentry = NULL, *root = NULL;
	unsigned int type;

	root = debugfs_create_dir("rxque", parent);
	if (IS_ERR(root))
		return -ENOMEM;

	for (type = 0; type < FSM_DP_RX_TYPE_LAST; type++) {
		dentry = debugfs_create_dir(fsm_dp_rx_type_to_str(type),
					    root);
		if (IS_ERR(dentry))
			return -ENOMEM;

		entry = debugfs_create_file("config", 0444, dentry,
					    &drv->rxq[type],
					    &debugfs_rxq_config_ops);
		if (!entry)
			return -ENOMEM;

		entry = debugfs_create_file("runtime", 0444, dentry,
					    &drv->rxq[type],
					    &debugfs_rxq_runtime_ops);
		if (!entry)
			return -ENOMEM;

		entry = debugfs_create_file("opstats", 0444, dentry,
					    &drv->rxq[type],
					    &debugfs_rxq_opstats_ops);
		if (!entry)
			return -ENOMEM;

		entry = debugfs_create_file("refcnt", 0444, dentry,
					    &drv->rxq[type],
					    &debugfs_rxq_refcnt_ops);
		if (!entry)
			return -ENOMEM;
	}
	return 0;
}

static int debugfs_create_ring_dir(
	struct dentry *parent,
	struct fsm_dp_mempool **mempool)
{
	struct dentry *entry = NULL, *dentry = NULL;

	dentry = debugfs_create_dir("ring", parent);
	if (IS_ERR(dentry))
		return -ENOMEM;

	entry = debugfs_create_file("config", 0444, dentry,
				    mempool,
				    &debugfs_ring_config_ops);
	if (!entry)
		return -ENOMEM;

	entry = debugfs_create_file("runtime", 0444, dentry,
				    mempool,
				    &debugfs_ring_runtime_ops);
	if (!entry)
		return -ENOMEM;

	entry = debugfs_create_file("index", 0644, dentry,
				    mempool,
				    &debugfs_ring_index_ops);
	if (!entry)
		return -ENOMEM;

	entry = debugfs_create_file("data", 0644, dentry,
				    mempool,
				    &debugfs_ring_data_ops);
	if (!entry)
		return -ENOMEM;

	entry = debugfs_create_file("opstats", 0444, dentry,
				    mempool,
				    &debugfs_ring_opstats_ops);
	if (!entry)
		return -ENOMEM;

	return 0;
}

static int debugfs_create_mem_dir(
	struct dentry *parent,
	struct fsm_dp_mempool **mempool)
{
	struct dentry *entry = NULL, *dentry = NULL;

	dentry = debugfs_create_dir("mem", parent);
	if (IS_ERR(dentry))
		return -ENOMEM;

	entry = debugfs_create_file("config", 0444, dentry,
				    mempool,
				    &debugfs_mem_config_ops);
	if (!entry)
		return -ENOMEM;

	entry = debugfs_create_file("offset", 0444, dentry,
				    mempool,
				    &debugfs_mem_offset_ops);
	if (!entry)
		return -ENOMEM;

	entry = debugfs_create_file("dump_size", 0644, dentry,
				    mempool,
				    &debugfs_mem_dump_size_ops);
	if (!entry)
		return -ENOMEM;

	entry = debugfs_create_file("data", 0644, dentry,
				    mempool,
				    &debugfs_mem_data_ops);
	if (!entry)
		return -ENOMEM;

	return 0;
}

static int debugfs_create_mempool_dir(
	struct dentry *parent,
	struct fsm_dp_drv *drv)
{
	struct dentry *entry = NULL, *dentry = NULL, *root = NULL;
	int ret;
	unsigned int type;

	root = debugfs_create_dir("mempool", parent);
	if (IS_ERR(root))
		return -ENOMEM;

	entry = debugfs_create_file("active", 0444, root, drv,
				    &debugfs_mempool_active_ops);
	if (!entry)
		return -ENOMEM;

	for (type = 0; type < FSM_DP_MEM_TYPE_LAST; type++) {
		dentry = debugfs_create_dir(fsm_dp_mem_type_to_str(type), root);
		if (IS_ERR(dentry))
			return -ENOMEM;

		ret = debugfs_create_ring_dir(dentry, &drv->mempool[type]);
		if (ret)
			return ret;

		ret = debugfs_create_mem_dir(dentry, &drv->mempool[type]);
		if (ret)
			return ret;

		entry = debugfs_create_file("info", 0444, dentry,
					    &drv->mempool[type],
					    &debugfs_mempool_info_ops);
		if (!entry)
			return -ENOMEM;

		entry = debugfs_create_file("status", 0444, dentry,
					    &drv->mempool[type],
					    &debugfs_mempool_status_ops);
		if (!entry)
			return -ENOMEM;

		entry = debugfs_create_file("state", 0444, dentry,
					    &drv->mempool[type],
					    &debugfs_mempool_state_ops);
		if (!entry)
			return -ENOMEM;
	}
	return 0;
}

int fsm_dp_debugfs_init(struct fsm_dp_drv *drv)
{
	struct dentry *entry = NULL;

	if (unlikely(drv == NULL))
		return -EINVAL;

	if (unlikely(__dent))
		return -EBUSY;

	__dent = debugfs_create_dir(FSM_DP_MODULE_NAME, 0);
	if (IS_ERR(__dent))
		return -ENOMEM;

	entry = debugfs_create_file("driver", 0444, __dent, drv,
				    &debugfs_drv_ops);
	if (!entry)
		goto err;

	entry = debugfs_create_file("cdev", 0444, __dent, drv,
				    &debugfs_cdev_ops);
	if (!entry)
		goto err;

	entry = debugfs_create_file("mhi", 0444, __dent, drv,
				    &debugfs_mhi_ops);
	if (!entry)
		goto err;

	entry = debugfs_create_file("status", 0444, __dent, drv,
				    &debugfs_drv_status_ops);
	if (!entry)
		goto err;

	if (debugfs_create_mempool_dir(__dent, drv))
		goto err;

	if (debugfs_create_rxq_dir(__dent, drv))
		goto err;

	if (debugfs_create_loopback_dir(__dent, drv))
		goto err;

	if (debugfs_create_traffic_dir(__dent, drv))
		goto err;

#ifdef CONFIG_FSM_DP_TEST
	if (debugfs_create_testring_dir(__dent, drv))
		goto err;
#endif

	return 0;
err:
	debugfs_remove_recursive(__dent);
	__dent = NULL;
	return -ENOMEM;
}

void fsm_dp_debugfs_cleanup(struct fsm_dp_drv *drv)
{
	debugfs_remove_recursive(__dent);
	__dent = NULL;
}

#ifdef CONFIG_FSM_DP_TEST
static int debugfs_testring_enable_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_test_ring *testrng =
		(struct fsm_dp_test_ring *)s->private;

	seq_printf(s, "%s\n", (testrng->enable) ? "enabled" : "disabled");

	return 0;
}

static ssize_t debugfs_testring_enable_write(
	struct file *fp,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	struct fsm_dp_test_ring *testrng = (struct fsm_dp_test_ring *)
			(((struct seq_file *)fp->private_data)->private);
	unsigned int value = 0;

	if (kstrtouint_from_user(buf, count, 0, &value))
		return -EFAULT;

	testrng->enable = (value) ? true : false;
	return count;
}
DEFINE_DEBUGFS_OPS(debugfs_testring_enable, debugfs_testring_enable_read,
		   debugfs_testring_enable_write);

static int debugfs_testring_opstats_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_test_ring *testrng =
		(struct fsm_dp_test_ring *)s->private;

	__fsm_dp_ring_opstats_dump(s, &testrng->ring.opstats);

	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_testring_opstats,
		   debugfs_testring_opstats_read, NULL);

static int debugfs_testring_config_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_test_ring *testrng =
		(struct fsm_dp_test_ring *)s->private;

	__fsm_dp_ring_config_dump(s, &testrng->ring);

	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_testring_config,
		   debugfs_testring_config_read, NULL);

static int debugfs_testring_runtime_read(struct seq_file *s, void *unused)
{
	struct fsm_dp_test_ring *testrng =
		(struct fsm_dp_test_ring *)s->private;

	__fsm_dp_ring_runtime_dump(s, &testrng->ring);

	return 0;
}
DEFINE_DEBUGFS_OPS(debugfs_testring_runtime,
		   debugfs_testring_runtime_read, NULL);

static int debugfs_create_testring_dir(
	struct dentry *parent,
	struct fsm_dp_drv *drv)
{
	struct dentry *entry = NULL, *dentry = NULL;

	dentry = debugfs_create_dir("test-ring", parent);
	if (IS_ERR(dentry))
		return -ENOMEM;

	entry = debugfs_create_file("config", 0444, dentry,
				    &drv->test_ring,
				    &debugfs_testring_config_ops);
	if (!entry)
		return -ENOMEM;

	entry = debugfs_create_file("runtime", 0444, dentry,
				    &drv->test_ring,
				    &debugfs_testring_runtime_ops);
	if (!entry)
		return -ENOMEM;

	entry = debugfs_create_file("opstats", 0444, dentry,
				    &drv->test_ring,
				    &debugfs_testring_opstats_ops);
	if (!entry)
		return -ENOMEM;

	entry = debugfs_create_file("enable", 0644, dentry,
				    &drv->test_ring,
				    &debugfs_testring_enable_ops);
	if (!entry)
		return -ENOMEM;
	return 0;
}

#endif /* CONFIG_FSM_DP_TEST */

#else

int fsm_dp_debugfs_init(struct fsm_dp_drv *drv)
{
	return 0;
}

void fsm_dp_debugfs_cleanup(struct fsm_dp_drv *drv)
{
}
#endif
