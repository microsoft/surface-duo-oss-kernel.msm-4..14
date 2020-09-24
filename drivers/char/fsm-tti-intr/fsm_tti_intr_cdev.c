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
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <linux/poll.h>
#include <linux/cdev.h>

#include "fsm_tti_intr.h"

static int fsm_tti_intr_cdev_close(
	struct inode *inode,
	struct file *file)
{
	struct fsm_tti_intr_drv *tti_drv_cntx =
		(struct fsm_tti_intr_drv *)file->private_data;

	if (IS_ERR(tti_drv_cntx)) {
		FSM_TTI_ERROR("FSM-TTI: %s: memory not allocated\n",
			__func__);
		return -ENOMEM;
	}

	atomic_set(&tti_drv_cntx->tti_updated, 0);
	tti_drv_cntx->is_poll_enabled = false;
	return 0;
}

static int fsm_tti_intr_cdev_open(
	struct inode *inode,
	struct file *file)
{
	struct fsm_tti_intr_drv *tti_drv_cntx = container_of(inode->i_cdev,
		struct fsm_tti_intr_drv, cdev);

	if (IS_ERR(tti_drv_cntx)) {
		FSM_TTI_ERROR("FSM-TTI: %s: memory not allocated\n",
			__func__);
		return -ENOMEM;
	}

	FSM_TTI_INFO("FSM-TTI: %s: shared ptr:%p allocated\n", __func__,
		tti_drv_cntx->shared_data);

	file->private_data = tti_drv_cntx;
	return 0;
}

static long fsm_tti_intr_cdev_ioctl(
	struct file *file,
	unsigned int iocmd,
	unsigned long ioarg)
{
	struct fsm_tti_intr_drv *tti_drv_cntx =
		(struct fsm_tti_intr_drv *)file->private_data;
	if (IS_ERR(tti_drv_cntx)) {
		FSM_TTI_ERROR("FSM-TTI: %s: memory not allocated\n",
			__func__);
		return -ENOMEM;
	}

	switch (iocmd) {
	case FSM_TTI_IOCTL_INITIAL_SFN_SLOT_INFO:
		if (copy_from_user(tti_drv_cntx->shared_data,
			(void __user *)ioarg,
			sizeof(struct fsm_tti_mmap_info))) {
			FSM_TTI_ERROR("FSM-TTI: %s: copy_from_user failed\n",
				__func__);
			return -EFAULT;
		}

		/* update seeding complete flag */
		tti_drv_cntx->is_seeding_done = true;

		/* Update into debugfs stats */
		tti_drv_cntx->debugfs_stats.initial_sfn_slot.sfn_slot =
			tti_drv_cntx->shared_data->sfn_slot_info.sfn_slot;
		tti_drv_cntx->debugfs_stats.sfn_slot_seeding_time = ktime_get();

		FSM_TTI_INFO(
			"FSM-TTI: %s: initial sfn: %u, slot: %u, max_slot: %u, time:%lld\n",
			__func__,
			tti_drv_cntx->shared_data->sfn_slot_info.sfn,
			tti_drv_cntx->shared_data->sfn_slot_info.slot,
			tti_drv_cntx->shared_data->max_slot,
			tti_drv_cntx->debugfs_stats.sfn_slot_seeding_time);
		break;
	default:
		tti_drv_cntx->shared_data->sfn_slot_info.sfn_slot = 0;
	}

	return 0;
}

static unsigned int fsm_tti_intr_cdev_poll(
	struct file *file,
	poll_table *wait)
{
	unsigned int mask = 0;
	struct fsm_tti_intr_drv *tti_drv_cntx =
		(struct fsm_tti_intr_drv *)file->private_data;

	if (!tti_drv_cntx->is_poll_enabled)
		tti_drv_cntx->is_poll_enabled = true;

	poll_wait(file, &tti_drv_cntx->tti_poll_waitqueue, wait);

	/* check wait is interrupted by interrupt handler only */
	if (atomic_sub_and_test(1, &tti_drv_cntx->tti_updated ))
		mask = POLLIN;

	return mask;
}


static int fsm_tti_intr_cdev_mmap(
	struct file *file,
	struct vm_area_struct *vma)
{
	int ret = 0;
	struct fsm_tti_intr_drv *tti_drv_cntx =
		(struct fsm_tti_intr_drv *)file->private_data;

	if (IS_ERR(tti_drv_cntx)) {
		FSM_TTI_ERROR("FSM-TTI: %s: memory not allocated\n",
			__func__);
		return -ENOMEM;
	}

	if (IS_ERR(tti_drv_cntx->shared_data)) {
		FSM_TTI_ERROR("FSM-TTI: %s: shared memory not allocated\n",
			__func__);
		return -ENOMEM;
	}

	FSM_TTI_INFO(
		"FSM-TTI: %s: start=%lx end=%lx off=%lx proto=%lx flag=%lx",
		__func__, vma->vm_start, vma->vm_end, vma->vm_pgoff,
		(unsigned long)vma->vm_page_prot.pgprot, vma->vm_flags);

	/* only support shared mapping */
	if ((vma->vm_flags & VM_WRITE) && !(vma->vm_flags & VM_SHARED)) {
		FSM_TTI_ERROR("FSM-TTI: %s: mappings must be shared\n",
			__func__);
		return -EINVAL;
	}

	ret = remap_pfn_range(vma,
			vma->vm_start,
			page_to_pfn(tti_drv_cntx->page),
			(vma->vm_end - vma->vm_start),
			vma->vm_page_prot);
	if (ret) {
		FSM_TTI_ERROR("FSM-TTI: %s: remap_pfn_range failed\n",
			__func__);
		return ret;
	}
	return 0;
}

static const struct file_operations fsm_tti_intr_cdev_fops = {
	.owner = THIS_MODULE,
	.poll = fsm_tti_intr_cdev_poll,
	.unlocked_ioctl = fsm_tti_intr_cdev_ioctl,
	.mmap = fsm_tti_intr_cdev_mmap,
	.open = fsm_tti_intr_cdev_open,
	.release = fsm_tti_intr_cdev_close,
};

int fsm_tti_cdev_init(struct fsm_tti_intr_drv *tti_intr_drv)
{
	int ret = 0;
	dev_t devno;
	struct device *dev;

	if (IS_ERR(tti_intr_drv)) {
		FSM_TTI_ERROR("FSM-TTI: %s: driver context not allocated\n",
			__func__);
		return -ENOMEM;
	}

	tti_intr_drv->dev_class = class_create(THIS_MODULE,
		FSM_TTI_DEV_CLASS_NAME);
	if (IS_ERR(tti_intr_drv->dev_class)) {
		FSM_TTI_ERROR("FSM-TTI: %s: class_create failed\n",
			__func__);
		return -ENOMEM;
	}

	ret = alloc_chrdev_region(&devno, 0, 1, FSM_TTI_CDEV_NAME);
	if (ret) {
		FSM_TTI_ERROR("FSM-TTI: %s: alloc_chrdev_region failed\n",
			__func__);
		goto cleanup_class;
	}

	cdev_init(&tti_intr_drv->cdev, &fsm_tti_intr_cdev_fops);
	ret = cdev_add(&tti_intr_drv->cdev, devno, 1);
	if (ret) {
		FSM_TTI_ERROR("FSM-TTI: %s: cdev_add failed!\n", __func__);
		goto unregister_cdev;
	}

	dev = device_create(tti_intr_drv->dev_class, tti_intr_drv->dev, devno,
			    tti_intr_drv, FSM_TTI_CDEV_NAME);
	if (IS_ERR(dev)) {
		FSM_TTI_ERROR("FSM-TTI: %s: device_create failed\n", __func__);
		ret = PTR_ERR(dev);
		goto del_cdev;
	}

	FSM_TTI_INFO("FSM-TTI: cdev initialized\n");
	return 0;

del_cdev:
	cdev_del(&tti_intr_drv->cdev);
unregister_cdev:
	unregister_chrdev_region(tti_intr_drv->cdev.dev, 1);
cleanup_class:
	class_destroy(tti_intr_drv->dev_class);
	tti_intr_drv->dev_class = NULL;
	FSM_TTI_ERROR("FSM-TTI: cdev initialization failed\n");
	return ret;
}

void fsm_tti_cdev_cleanup(struct fsm_tti_intr_drv *tti_intr_drv)
{
	if (tti_intr_drv->dev_class) {
		device_destroy(tti_intr_drv->dev_class, tti_intr_drv->cdev.dev);
		cdev_del(&tti_intr_drv->cdev);
		unregister_chrdev_region(tti_intr_drv->cdev.dev, 1);
		class_destroy(tti_intr_drv->dev_class);
		tti_intr_drv->dev_class = NULL;
	}
	FSM_TTI_INFO("FSM-TTI: cdev cleaned-up\n");
}


