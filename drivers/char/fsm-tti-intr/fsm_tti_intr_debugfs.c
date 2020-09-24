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

#include <linux/err.h>
#include <linux/debugfs.h>

#include "fsm_tti_intr.h"

static struct dentry *__dent;

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

static int debugfs_tti_status_show(struct seq_file *s, void *unused)
{
	struct fsm_tti_intr_drv *tti_drv_cntx =
		(struct fsm_tti_intr_drv *)s->private;

	if (IS_ERR(tti_drv_cntx)) {
		FSM_TTI_ERROR("FSM-TTI: %s: driver context not allocated\n",
			__func__);
		return -ENOMEM;
	}

	seq_puts(s, "  TTI Interrupt Driver\n");
	seq_printf(s, "  Initial SF Number:                     %d\n",
		tti_drv_cntx->debugfs_stats.initial_sfn_slot.sfn);
	seq_printf(s, "  Initial Slot Number:                   %d\n",
		tti_drv_cntx->debugfs_stats.initial_sfn_slot.slot);
	seq_printf(s, "  First Interrupt Receive Time (nsec):   %lld\n",
		tti_drv_cntx->debugfs_stats.first_tti_recv_time);
	seq_printf(s, "  Current Interrupt Receive Time (nsec): %lld\n",
		tti_drv_cntx->debugfs_stats.current_tti_recv_time);
	seq_printf(s, "  Total Interrupt Receive Count:         %lld\n",
		tti_drv_cntx->debugfs_stats.current_tti_count);
	seq_printf(s, "  Sfn/Slot seeding time (nsec):          %lld\n",
		tti_drv_cntx->debugfs_stats.sfn_slot_seeding_time);
	seq_printf(s, "  Seeding to first TTI delay(nsec):      %lld\n",
		tti_drv_cntx->debugfs_stats.first_tti_recv_time -
		tti_drv_cntx->debugfs_stats.sfn_slot_seeding_time);

	return 0;
}

DEFINE_DEBUGFS_OPS(debugfs_tti_status, debugfs_tti_status_show, NULL);


int fsm_tti_debugfs_init(struct fsm_tti_intr_drv *tti_intr_drv)
{
	struct dentry *entry = NULL;

	if (IS_ERR(tti_intr_drv)) {
		FSM_TTI_ERROR("FSM-TTI: %s: driver context not allocated\n",
			__func__);
		return -ENOMEM;
	}

	__dent = debugfs_create_dir(FSM_TTI_MODULE_NAME, 0);
	if (IS_ERR(__dent))
		return -ENOMEM;

	entry = debugfs_create_file("stat", 0444, __dent,
			tti_intr_drv, &debugfs_tti_status_ops);
	if (!entry)
		goto error;

	/* initialize the debugfs stat structure*/
	memset(&tti_intr_drv->debugfs_stats, 0,
		sizeof(struct fsm_tti_internal_stats));

	FSM_TTI_INFO("FSM-TTI: debugfs initialized\n");
	return 0;

error:
	debugfs_remove_recursive(__dent);
	__dent = NULL;
	return -ENOMEM;
}

void fsm_tti_debugfs_cleanup(struct fsm_tti_intr_drv *tti_intr_drv)
{
	debugfs_remove_recursive(__dent);
	__dent = NULL;
}

