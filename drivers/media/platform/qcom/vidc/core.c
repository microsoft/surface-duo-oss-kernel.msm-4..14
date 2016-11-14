/*
 * Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2016 Linaro Ltd.
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
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/remoteproc.h>
#include <linux/pm_runtime.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-ioctl.h>

#include "core.h"
#include "vdec.h"
#include "venc.h"

struct vidc_sys_error {
	struct vidc_core *core;
	struct delayed_work work;
};

static void vidc_sys_error_handler(struct work_struct *work)
{
	struct vidc_sys_error *handler =
		container_of(work, struct vidc_sys_error, work.work);
	struct vidc_core *core = handler->core;
	struct device *dev = core->dev;
	int ret;

	mutex_lock(&core->lock);
	if (core->state != CORE_INVALID)
		goto exit;

	mutex_unlock(&core->lock);

	disable_irq(core->irq);

	ret = hfi_core_deinit(core);
	if (ret)
		dev_err(dev, "core: deinit failed (%d)\n", ret);

	mutex_lock(&core->lock);

	pm_runtime_get_sync(core->dev);

	rproc_report_crash(core->rproc, RPROC_FATAL_ERROR);

	rproc_shutdown(core->rproc);

	ret = rproc_boot(core->rproc);
	if (ret)
		goto exit;

	hfi_core_init(core);

//	core->state = CORE_INIT;

exit:
	mutex_unlock(&core->lock);
	kfree(handler);
	pm_runtime_put_sync(core->dev);
}

static int vidc_event_notify(struct vidc_core *core, u32 event)
{
	struct vidc_sys_error *handler;
	struct vidc_inst *inst;

	switch (event) {
	case EVT_SYS_WATCHDOG_TIMEOUT:
	case EVT_SYS_ERROR:
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&core->lock);

	core->state = CORE_INVALID;

	list_for_each_entry(inst, &core->instances, list) {
		mutex_lock(&inst->lock);
		inst->state = INST_INVALID;
		mutex_unlock(&inst->lock);
	}

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

static const struct hfi_core_ops vidc_core_ops = {
	.event_notify = vidc_event_notify,
};

static int vidc_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct vidc_core *core = video_drvdata(file);
	struct vidc_inst *inst;
	int ret;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	INIT_LIST_HEAD(&inst->registeredbufs);
	mutex_init(&inst->registeredbufs_lock);

	INIT_LIST_HEAD(&inst->internalbufs);
	mutex_init(&inst->internalbufs_lock);

	INIT_LIST_HEAD(&inst->bufqueue);
	mutex_init(&inst->bufqueue_lock);

	INIT_LIST_HEAD(&inst->list);
	mutex_init(&inst->lock);

	inst->core = core;

	if (vdev == core->vdev_dec) {
		inst->session_type = VIDC_SESSION_TYPE_DEC;
		ret = vdec_open(inst);
		if (ret)
			goto err_free_inst;
		v4l2_fh_init(&inst->fh, core->vdev_dec);
	} else {
		inst->session_type = VIDC_SESSION_TYPE_ENC;
		ret = venc_open(inst);
		if (ret)
			goto err_free_inst;
		v4l2_fh_init(&inst->fh, core->vdev_enc);
	}

	inst->fh.ctrl_handler = &inst->ctrl_handler;
	v4l2_fh_add(&inst->fh);
	file->private_data = &inst->fh;

	return 0;

err_free_inst:
	kfree(inst);
	return ret;
}

static int vidc_close(struct file *file)
{
	struct vidc_inst *inst = to_inst(file);

	if (inst->session_type == VIDC_SESSION_TYPE_DEC)
		vdec_close(inst);
	else
		venc_close(inst);

	mutex_destroy(&inst->bufqueue_lock);
	mutex_destroy(&inst->registeredbufs_lock);
	mutex_destroy(&inst->internalbufs_lock);
	mutex_destroy(&inst->lock);

	v4l2_fh_del(&inst->fh);
	v4l2_fh_exit(&inst->fh);

	kfree(inst);
	return 0;
}

static unsigned int vidc_poll(struct file *file, struct poll_table_struct *pt)
{
	struct vidc_inst *inst = to_inst(file);
	struct vb2_queue *outq = &inst->bufq_out;
	struct vb2_queue *capq = &inst->bufq_cap;
	unsigned int ret;

	ret = vb2_poll(outq, file, pt);
	ret |= vb2_poll(capq, file, pt);

	return ret;
}

static int vidc_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct vidc_inst *inst = to_inst(file);
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

static irqreturn_t vidc_isr_thread(int irq, void *dev_id)
{
	struct vidc_core *core = dev_id;

	return hfi_isr_thread(core);
}

static irqreturn_t vidc_isr(int irq, void *dev)
{
	struct vidc_core *core = dev;

	return hfi_isr(core);
}

static int vidc_clks_get(struct vidc_core *core)
{
	const struct vidc_resources *res = core->res;
	struct device *dev = core->dev;
	unsigned int i;

	for (i = 0; i < res->clks_num; i++) {
		core->clks[i] = devm_clk_get(dev, res->clks[i]);
		if (IS_ERR(core->clks[i]))
			return PTR_ERR(core->clks[i]);
	}

	return 0;
}

static int vidc_clks_enable(struct vidc_core *core)
{
	const struct vidc_resources *res = core->res;
	unsigned int i;
	int ret;

	for (i = 0; i < res->clks_num; i++) {
		ret = clk_prepare_enable(core->clks[i]);
		if (ret)
			goto err;
	}

	return 0;
err:
	while (--i)
		clk_disable_unprepare(core->clks[i]);

	return ret;
}

static void vidc_clks_disable(struct vidc_core *core)
{
	const struct vidc_resources *res = core->res;
	unsigned int i;

	for (i = 0; i < res->clks_num; i++)
		clk_disable_unprepare(core->clks[i]);
}

static int vidc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vidc_core *core;
	struct device_node *rproc;
	struct resource *r;
	int ret, irq;

	core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->dev = dev;
	platform_set_drvdata(pdev, core);

	rproc = of_parse_phandle(dev->of_node, "rproc", 0);
	if (!rproc)
		return -ENODEV;

	core->rproc = rproc_get_by_phandle(rproc->phandle);
	if (IS_ERR(core->rproc))
		return PTR_ERR(core->rproc);
//	else if (!core->rproc)
//		return -EPROBE_DEFER;

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "venus");
	core->base = devm_ioremap_resource(dev, r);
	if (IS_ERR(core->base))
		return PTR_ERR(core->base);

	irq = platform_get_irq_byname(pdev, "venus");
	if (irq < 0)
		return irq;

	core->irq = irq;

	core->res = of_device_get_match_data(dev);
	if (!core->res)
		return -ENODEV;

	ret = vidc_clks_get(core);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(dev, core->res->dma_mask);
	if (ret)
		return ret;

	INIT_LIST_HEAD(&core->instances);
	mutex_init(&core->lock);

	ret = devm_request_threaded_irq(dev, irq, vidc_isr, vidc_isr_thread,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					"vidc", core);
	if (ret)
		return ret;

	core->core_ops = &vidc_core_ops;

	ret = hfi_create(core);
	if (ret)
		return ret;

	ret = vidc_clks_enable(core);
	if (ret)
		goto err_hfi_destroy;

	ret = rproc_boot(core->rproc);
	if (ret) {
		vidc_clks_disable(core);
		goto err_hfi_destroy;
	}

	vidc_clks_disable(core);

	pm_runtime_enable(dev);

	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		goto err_runtime_disable;

	ret = hfi_core_init(core);
	if (ret)
		goto err_rproc_shutdown;

	ret = pm_runtime_put_sync(dev);
	if (ret)
		goto err_core_deinit;

	ret = v4l2_device_register(dev, &core->v4l2_dev);
	if (ret)
		goto err_core_deinit;

	core->vdev_dec = video_device_alloc();
	if (!core->vdev_dec) {
		ret = -ENOMEM;
		goto err_dev_unregister;
	}

	core->vdev_enc = video_device_alloc();
	if (!core->vdev_enc) {
		ret = -ENOMEM;
		goto err_dec_release;
	}

	ret = vdec_init(core, core->vdev_dec, &vidc_fops);
	if (ret)
		goto err_enc_release;

	ret = venc_init(core, core->vdev_enc, &vidc_fops);
	if (ret)
		goto err_vdec_deinit;

	return 0;

err_vdec_deinit:
	vdec_deinit(core, core->vdev_dec);
err_enc_release:
	video_device_release(core->vdev_enc);
err_dec_release:
	video_device_release(core->vdev_dec);
err_dev_unregister:
	v4l2_device_unregister(&core->v4l2_dev);
err_core_deinit:
	hfi_core_deinit(core);
err_rproc_shutdown:
	rproc_shutdown(core->rproc);
err_runtime_disable:
	pm_runtime_set_suspended(dev);
	pm_runtime_disable(dev);
err_hfi_destroy:
	hfi_destroy(core);
	return ret;
}

static int vidc_remove(struct platform_device *pdev)
{
	struct vidc_core *core = platform_get_drvdata(pdev);
	int ret;

	pm_runtime_get_sync(&pdev->dev);

	ret = hfi_core_deinit(core);

	rproc_shutdown(core->rproc);

	pm_runtime_put_sync(&pdev->dev);

	hfi_destroy(core);
	vdec_deinit(core, core->vdev_dec);
	venc_deinit(core, core->vdev_enc);
	v4l2_device_unregister(&core->v4l2_dev);

	pm_runtime_disable(core->dev);

	return ret;
}

#ifdef CONFIG_PM
static int vidc_runtime_suspend(struct device *dev)
{
	struct vidc_core *core = dev_get_drvdata(dev);
	int ret;

	ret = hfi_core_suspend(core);

	vidc_clks_disable(core);

	return ret;
}

static int vidc_runtime_resume(struct device *dev)
{
	struct vidc_core *core = dev_get_drvdata(dev);
	int ret;

	ret = vidc_clks_enable(core);
	if (ret)
		return ret;

	return hfi_core_resume(core);
}
#endif

static const struct dev_pm_ops vidc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(vidc_runtime_suspend, vidc_runtime_resume, NULL)
};

static const struct freq_tbl msm8916_freq_table[] = {
	{ 352800, 228570000 },	/* 1920x1088 @ 30 + 1280x720 @ 30 */
	{ 244800, 160000000 },	/* 1920x1088 @ 30 */
	{ 108000, 100000000 },	/* 1280x720 @ 30 */
};

static const struct reg_val msm8916_reg_preset[] = {
	{ 0xe0020, 0x05555556 },
	{ 0xe0024, 0x05555556 },
	{ 0x80124, 0x00000003 },
};

static const struct vidc_resources msm8916_res = {
	.freq_tbl = msm8916_freq_table,
	.freq_tbl_size = ARRAY_SIZE(msm8916_freq_table),
	.reg_tbl = msm8916_reg_preset,
	.reg_tbl_size = ARRAY_SIZE(msm8916_reg_preset),
	.clks = { "core", "iface", "bus", },
	.clks_num = 3,
	.max_load = 352800, /* 720p@30 + 1080p@30 */
	.hfi_version = HFI_VERSION_LEGACY,
	.vmem_id = VIDC_RESOURCE_NONE,
	.vmem_size = 0,
	.vmem_addr = 0,
	.dma_mask = 0xddc00000 - 1,
};

static const struct freq_tbl msm8996_freq_table[] = {
	{ 1944000, 490000000 },	/* 4k UHD @ 60 */
	{  972000, 320000000 },	/* 4k UHD @ 30 */
	{  489600, 150000000 },	/* 1080p @ 60 */
	{  244800,  75000000 },	/* 1080p @ 30 */
};

static const struct reg_val msm8996_reg_preset[] = {
	{ 0x80010, 0xffffffff },
	{ 0x80018, 0x00001556 },
	{ 0x8001C, 0x00001556 },
};

static const struct vidc_resources msm8996_res = {
	.freq_tbl = msm8996_freq_table,
	.freq_tbl_size = ARRAY_SIZE(msm8996_freq_table),
	.reg_tbl = msm8996_reg_preset,
	.reg_tbl_size = ARRAY_SIZE(msm8996_reg_preset),
	.clks = { "core", "core0", "core1", "iface", "bus", "rpm_mmaxi",
		  "mmagic_ahb", "mmagic_maxi", "mmagic_video_axi", "maxi_clk" },
	.clks_num = 10,
	.max_load = 2563200,
	.hfi_version = HFI_VERSION_3XX,
	.vmem_id = VIDC_RESOURCE_NONE,
	.vmem_size = 0,
	.vmem_addr = 0,
	.dma_mask = 0xddc00000 - 1,
};

static const struct of_device_id vidc_dt_match[] = {
	{ .compatible = "qcom,vidc-msm8916", .data = &msm8916_res, },
	{ .compatible = "qcom,vidc-msm8996", .data = &msm8996_res, },
	{ }
};

MODULE_DEVICE_TABLE(of, vidc_dt_match);

static struct platform_driver qcom_vidc_driver = {
	.probe = vidc_probe,
	.remove = vidc_remove,
	.driver = {
		.name = "qcom-venus",
		.of_match_table = vidc_dt_match,
		.pm = &vidc_pm_ops,
	},
};

module_platform_driver(qcom_vidc_driver);

MODULE_ALIAS("platform:qcom-vidc");
MODULE_DESCRIPTION("Qualcomm video encoder and decoder driver");
MODULE_LICENSE("GPL v2");
