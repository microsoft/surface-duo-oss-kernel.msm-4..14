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

#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/remoteproc.h>

#include "msm_vidc_common.h"
#include "msm_smem.h"
#include "msm_vdec.h"
#include "msm_venc.h"
#include "msm_vidc_debug.h"
#include "msm_vidc_internal.h"
#include "msm_vidc_resources.h"
#include "msm_hfi_interface.h"

/* Offset base for buffers on the destination queue - used to distinguish
 * between source and destination buffers when mmapping - they receive the same
 * offsets but for different queues */
#define DST_QUEUE_OFF_BASE	(1 << 30)

struct vidc_drv *vidc_driver;
unsigned int vidc_pwr_collapse_delay = 2000;

static inline struct vidc_inst *vidc_to_inst(struct file *file)
{
	return container_of(file->private_data, struct vidc_inst, fh);
}

static void vidc_add_inst(struct vidc_core *core, struct vidc_inst *inst)
{
	mutex_lock(&core->lock);
	list_add_tail(&inst->list, &core->instances);
	mutex_unlock(&core->lock);
}

static void vidc_del_inst(struct vidc_core *core, struct vidc_inst *inst)
{
	struct vidc_inst *pos, *n;

	mutex_lock(&core->lock);
	list_for_each_entry_safe(pos, n, &core->instances, list) {
		if (pos == inst)
			list_del(&inst->list);
	}
	mutex_unlock(&core->lock);
}

static int vidc_firmware_load(struct vidc_core *core)
{
	int ret;

	if (core->rproc_booted)
		return 0;

	ret = rproc_boot(core->rproc);
	if (ret)
		return ret;

	core->rproc_booted = true;

	return 0;
}

static void vidc_firmware_unload(struct vidc_core *core)
{
	if (!core->rproc_booted)
		return;

	rproc_shutdown(core->rproc);
	core->rproc_booted = false;
}

struct vidc_sys_error {
	struct vidc_core *core;
	struct delayed_work work;
};

static void vidc_sys_error_handler(struct work_struct *work)
{
	struct vidc_sys_error *handler;
	struct vidc_core *core;
	struct hfi_device *hdev;
	int ret;

	handler = container_of(work, struct vidc_sys_error, work.work);
	core = handler->core;
	hdev = core->hfidev;

	mutex_lock(&core->lock);
	if (core->state != CORE_INVALID) {
		mutex_unlock(&core->lock);
		goto exit;
	}

	ret = call_hfi_op(hdev, core_release, hdev->hfi_device_data);
	if (ret) {
		dprintk(VIDC_ERR, "%s: core_release failed: %d\n",
			__func__, ret);
	}

	rproc_report_crash(core->rproc, RPROC_FATAL_ERROR);

	vidc_firmware_unload(core);

	ret = vidc_firmware_load(core);
	if (ret)
		goto exit;

	core->state = CORE_INIT;

	mutex_unlock(&core->lock);

exit:
	kfree(handler);
}

static int vidc_event_notify(struct vidc_core *core, u32 device_id, u32 event)
{
	struct vidc_sys_error *handler;
	struct vidc_inst *inst = NULL;

	switch (event) {
	case SYS_WATCHDOG_TIMEOUT:
	case SYS_ERROR:
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&core->lock);

	core->state = CORE_INVALID;

	list_for_each_entry(inst, &core->instances, list)
		vidc_inst_set_state(inst, INST_INVALID);

	mutex_unlock(&core->lock);

	handler = kzalloc(sizeof(*handler), GFP_KERNEL);
	if (!handler)
		return -ENOMEM;

	handler->core = core;
	INIT_DELAYED_WORK(&handler->work, vidc_sys_error_handler);

	/*
	 * Sleep for 5 sec to ensure venus has completed any
	 * pending cache operations. Without this sleep, we see
	 * device reset when firmware is unloaded after a sys
	 * error.
	 */
	schedule_delayed_work(&handler->work, msecs_to_jiffies(5000));

	return 0;
}

static int vidc_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct vidc_core *core = video_drvdata(file);
	struct device *dev = &core->res.pdev->dev;
	struct vidc_inst *inst;
	int ret = 0;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	mutex_init(&inst->sync_lock);
	mutex_init(&inst->lock);

	INIT_VIDC_LIST(&inst->scratchbufs);
	INIT_VIDC_LIST(&inst->persistbufs);
	INIT_VIDC_LIST(&inst->pending_getpropq);
	INIT_VIDC_LIST(&inst->registeredbufs);

	INIT_LIST_HEAD(&inst->bufqueue);
	mutex_init(&inst->bufqueue_lock);

	if (vdev == &core->vdev_dec)
		inst->session_type = VIDC_DECODER;
	else
		inst->session_type = VIDC_ENCODER;

	inst->state = INST_UNINIT;
	inst->core = core;

	inst->mem_client = smem_new_client(&core->res);
	if (IS_ERR(inst->mem_client)) {
		ret = PTR_ERR(inst->mem_client);
		goto err_free_inst;
	}

	ret = enable_clocks(&core->res);
	if (ret) {
		dev_err(dev, "enable clocks\n");
		goto err_del_mem_clnt;
	}

	ret = vidc_firmware_load(core);
	if (ret)
		goto err_dis_clks;

	ret = hfi_core_init(core);
	if (ret)
		goto err_fw_unload;

	inst->debugfs_root = vidc_debugfs_init_inst(inst, core->debugfs_root);

	if (inst->session_type == VIDC_DECODER)
		ret = vdec_open(inst);
	else
		ret = venc_open(inst);

	if (ret)
		goto err_core_deinit;

	if (inst->session_type == VIDC_DECODER)
		v4l2_fh_init(&inst->fh, &core->vdev_dec);
	else
		v4l2_fh_init(&inst->fh, &core->vdev_enc);

	inst->fh.ctrl_handler = &inst->ctrl_handler;

	v4l2_fh_add(&inst->fh);

	file->private_data = &inst->fh;

	vidc_add_inst(core, inst);

	return 0;

err_core_deinit:
	hfi_core_deinit(core);
err_fw_unload:
	vidc_firmware_unload(core);
err_dis_clks:
	disable_clocks(&core->res);
err_del_mem_clnt:
	smem_delete_client(inst->mem_client);
err_free_inst:
	kfree(inst);
	return ret;
}

static int vidc_close(struct file *file)
{
	struct vidc_inst *inst = vidc_to_inst(file);
	struct vidc_core *core = inst->core;
	struct device *dev = &core->res.pdev->dev;
	int ret;

	if (inst->session_type == VIDC_DECODER)
		vdec_close(inst);
	else
		venc_close(inst);

	vidc_del_inst(core, inst);

	debugfs_remove_recursive(inst->debugfs_root);

	mutex_lock(&inst->pending_getpropq.lock);
	WARN_ON(!list_empty(&inst->pending_getpropq.list));
	mutex_unlock(&inst->pending_getpropq.lock);

	hfi_session_clean(inst);

	ret = hfi_core_deinit(core);
	if (ret)
		dev_err(dev, "core: deinit failed (%d)\n", ret);

	disable_clocks(&core->res);

	smem_delete_client(inst->mem_client);

	mutex_destroy(&inst->bufqueue_lock);
	mutex_destroy(&inst->scratchbufs.lock);
	mutex_destroy(&inst->persistbufs.lock);
	mutex_destroy(&inst->pending_getpropq.lock);
	mutex_destroy(&inst->registeredbufs.lock);

	v4l2_fh_del(&inst->fh);
	v4l2_fh_exit(&inst->fh);

	kfree(inst);

	return 0;
}

static int vidc_get_poll_flags(struct vidc_inst *inst)
{
	struct vb2_queue *outq = &inst->bufq_out;
	struct vb2_queue *capq = &inst->bufq_cap;
	struct vb2_buffer *out_vb = NULL;
	struct vb2_buffer *cap_vb = NULL;
	unsigned long flags;
	int ret = 0;

	if (v4l2_event_pending(&inst->fh))
		ret |= POLLPRI;

	spin_lock_irqsave(&capq->done_lock, flags);
	if (!list_empty(&capq->done_list))
		cap_vb = list_first_entry(&capq->done_list, struct vb2_buffer,
					  done_entry);
	if (cap_vb && (cap_vb->state == VB2_BUF_STATE_DONE ||
		       cap_vb->state == VB2_BUF_STATE_ERROR))
		ret |= POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&capq->done_lock, flags);

	spin_lock_irqsave(&outq->done_lock, flags);
	if (!list_empty(&outq->done_list))
		out_vb = list_first_entry(&outq->done_list, struct vb2_buffer,
					  done_entry);
	if (out_vb && (out_vb->state == VB2_BUF_STATE_DONE ||
		       out_vb->state == VB2_BUF_STATE_ERROR))
		ret |= POLLOUT | POLLWRNORM;
	spin_unlock_irqrestore(&outq->done_lock, flags);

	return ret;
}

static unsigned int vidc_poll(struct file *file, struct poll_table_struct *pt)
{
	struct vidc_inst *inst = vidc_to_inst(file);
	struct vb2_queue *outq = &inst->bufq_out;
	struct vb2_queue *capq = &inst->bufq_cap;

	poll_wait(file, &inst->fh.wait, pt);
	poll_wait(file, &capq->done_wq, pt);
	poll_wait(file, &outq->done_wq, pt);

	return vidc_get_poll_flags(inst);
}

static int vidc_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct vidc_inst *inst = vidc_to_inst(file);
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	int ret;

	if (offset < DST_QUEUE_OFF_BASE) {
		ret = vb2_mmap(&inst->bufq_out, vma);
	} else {
		vma->vm_pgoff -= DST_QUEUE_OFF_BASE >> PAGE_SHIFT;
		ret = vb2_mmap(&inst->bufq_cap, vma);
	}

	return ret;
}

const struct v4l2_file_operations vidc_fops = {
	.owner = THIS_MODULE,
	.open = vidc_open,
	.release = vidc_close,
	.unlocked_ioctl = video_ioctl2,
	.poll = vidc_poll,
	.mmap = vidc_mmap,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = v4l2_compat_ioctl32,
#endif
};

static const struct of_device_id vidc_dt_match[] = {
	{ .compatible = "qcom,msm-vidc" },
	{ }
};

MODULE_DEVICE_TABLE(of, vidc_dt_match);

static int vidc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vidc_core *core;
	struct vidc_resources *res;
	struct device_node *rproc;
	struct resource *r;
	int ret;

	if (!vidc_driver) {
		vidc_driver = kzalloc(sizeof(*vidc_driver), GFP_KERNEL);
		if (!vidc_driver)
			return -ENOMEM;

		INIT_LIST_HEAD(&vidc_driver->cores);
		mutex_init(&vidc_driver->lock);

		vidc_driver->debugfs_root = vidc_debugfs_init_drv();
		if (!vidc_driver->debugfs_root)
			dev_err(dev, "create debugfs for vidc\n");
	}

	mutex_lock(&vidc_driver->lock);
	if (vidc_driver->num_cores + 1 > VIDC_CORES_MAX) {
		mutex_unlock(&vidc_driver->lock);
		dev_warn(dev, "maximum cores reached\n");
		return -EBUSY;
	}
	vidc_driver->num_cores++;
	mutex_unlock(&vidc_driver->lock);

	core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->res.pdev = pdev;
	platform_set_drvdata(pdev, core);

	rproc = of_parse_phandle(dev->of_node, "rproc", 0);
	if (IS_ERR(rproc)) {
		dev_err(dev, "cannot parse phandle rproc\n");
		return PTR_ERR(rproc);
	}

	core->rproc = rproc_get_by_phandle(rproc->phandle);
	if (IS_ERR(core->rproc))
		return PTR_ERR(core->rproc);
	else if (!core->rproc)
		return -EPROBE_DEFER;

	res = &core->res;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res->base = devm_ioremap_resource(dev, r);
	if (IS_ERR(res->base))
		return PTR_ERR(res->base);

	res->irq = platform_get_irq(pdev, 0);
	if (IS_ERR_VALUE(res->irq))
		return res->irq;

	ret = get_platform_resources(core);
	if (ret)
		return ret;

	INIT_LIST_HEAD(&core->instances);
	mutex_init(&core->lock);
	core->state = CORE_UNINIT;

	if (core->hfi_type == VIDC_HFI_Q6)
		core->id = VIDC_CORE_Q6;
	else
		core->id = VIDC_CORE_VENUS;

	ret = v4l2_device_register(dev, &core->v4l2_dev);
	if (ret)
		return ret;

	ret = vdec_init(core, &core->vdev_dec);
	if (ret)
		goto err_dev_unregister;

	ret = venc_init(core, &core->vdev_enc);
	if (ret)
		goto err_vdec_deinit;

	core->hfidev = vidc_hfi_init(core->hfi_type, core->id, &core->res);
	if (IS_ERR(core->hfidev)) {
		mutex_lock(&vidc_driver->lock);
		vidc_driver->num_cores--;
		mutex_unlock(&vidc_driver->lock);

		ret = PTR_ERR(core->hfidev);
		goto err_venc_deinit;
	}

	core->event_notify = vidc_event_notify;

	mutex_lock(&vidc_driver->lock);
	list_add_tail(&core->list, &vidc_driver->cores);
	mutex_unlock(&vidc_driver->lock);

	core->debugfs_root =
		vidc_debugfs_init_core(core, vidc_driver->debugfs_root);

	return 0;

err_venc_deinit:
	venc_deinit(core, &core->vdev_enc);
err_vdec_deinit:
	vdec_deinit(core, &core->vdev_dec);
err_dev_unregister:
	v4l2_device_unregister(&core->v4l2_dev);
	return ret;
}

static int vidc_remove(struct platform_device *pdev)
{
	struct vidc_core *core;
	int empty;

	core = platform_get_drvdata(pdev);
	if (!core)
		return -EINVAL;

	vidc_hfi_deinit(core->hfi_type, core->hfidev);
	vdec_deinit(core, &core->vdev_dec);
	venc_deinit(core, &core->vdev_enc);
	v4l2_device_unregister(&core->v4l2_dev);

	put_platform_resources(core);

	mutex_lock(&vidc_driver->lock);
	list_del(&core->list);
	empty = list_empty(&vidc_driver->cores);
	mutex_unlock(&vidc_driver->lock);

	vidc_firmware_unload(core);

	if (!empty)
		return 0;

	debugfs_remove_recursive(vidc_driver->debugfs_root);
	kfree(vidc_driver);
	vidc_driver = NULL;

	return 0;
}

static int vidc_pm_suspend(struct device *dev)
{
	struct vidc_core *core = dev_get_drvdata(dev);

	if (!core)
		return -EINVAL;

	return hfi_core_suspend(core);
}

static int vidc_pm_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops vidc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(vidc_pm_suspend, vidc_pm_resume)
};

static struct platform_driver qcom_vidc_driver = {
	.probe = vidc_probe,
	.remove = vidc_remove,
	.driver = {
		.name = "qcom-vidc",
		.of_match_table = vidc_dt_match,
		.pm = &vidc_pm_ops,
	},
};

module_platform_driver(qcom_vidc_driver);

MODULE_ALIAS("platform:qcom-vidc");
MODULE_DESCRIPTION("Qualcomm video decoder driver");
MODULE_LICENSE("GPL v2");
