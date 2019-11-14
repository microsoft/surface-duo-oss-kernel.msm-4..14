// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 *
 */

#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/list.h>
#include <linux/memblock.h>
#include <linux/mhi.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "mhi_qcom.h"

struct firmware_info {
	u32 dev_id;
	const char *fw_image;
	const char *edl_image;
};

static const struct firmware_info firmware_table[] = {
	{.dev_id = 0x305, .fw_image = "sdx50m/sbl1.mbn"},
	{.dev_id = 0x304, .fw_image = "sbl.mbn", .edl_image = "edl.mbn"},
	/* default, set to debug.mbn */
	{.fw_image = "debug.mbn"},
};

static struct mhi_channel_config qcom_mhi_channels[] = {
	{
		.num = 0,
		.name = "LOOPBACK",
		.num_elements = 32,
		.event_ring = 0,
		.dir = DMA_TO_DEVICE,
		.ee = MHI_EE_AMSS,
		.pollcfg = 0,
		.data_type = MHI_BUF_RAW,
		.doorbell = MHI_DB_BRST_DISABLE,
		.lpm_notify = false,
		.offload_channel = false,
		.doorbell_mode_switch = false,
		.auto_queue = false,
		.auto_start = false,
	},
	{
		.num = 1,
		.name = "LOOPBACK",
		.num_elements = 32,
		.event_ring = 0,
		.dir = DMA_FROM_DEVICE,
		.ee = MHI_EE_AMSS,
		.pollcfg = 0,
		.data_type = MHI_BUF_RAW,
		.doorbell = MHI_DB_BRST_DISABLE,
		.lpm_notify = false,
		.offload_channel = false,
		.doorbell_mode_switch = false,
		.auto_queue = false,
		.auto_start = false,
	},
};

static struct mhi_event_config qcom_mhi_events[] = {
	{
		.num_elements = 32,
		.irq_moderation_ms = 0,
		.irq = 0,
		.channel = 0,
		.mode = MHI_DB_BRST_DISABLE,
		.data_type = MHI_ER_CTRL,
		.hardware_event = false,
		.client_managed = false,
		.offload_channel = false,
	},
};

static struct mhi_controller_config mhi_qcom_config = {
	.max_channels = 128,
	.timeout_ms = 1000,
	.use_bounce_buf = false,
	.buf_len = 0,
	.num_channels = ARRAY_SIZE(qcom_mhi_channels),
	.ch_cfg = qcom_mhi_channels,
	.num_events = ARRAY_SIZE(qcom_mhi_events),
	.event_cfg = qcom_mhi_events,
};

static void qcom_mhi_deinit_pci_dev(struct mhi_controller *mhi_cntrl,
				    struct mhi_qcom_dev *qdev)
{
	struct pci_dev *pci_dev = qdev->pci_dev;

	pci_free_irq_vectors(pci_dev);
	kfree(mhi_cntrl->irq);
	mhi_cntrl->irq = NULL;
	iounmap(mhi_cntrl->regs);
	mhi_cntrl->regs = NULL;
	pci_clear_master(pci_dev);
	pci_release_region(pci_dev, qdev->bars);
	pci_disable_device(pci_dev);
}

static int qcom_mhi_init_pci_dev(struct mhi_controller *mhi_cntrl,
				 struct mhi_qcom_dev *qdev)
{
	struct pci_dev *pci_dev = qdev->pci_dev;
	int ret;
	resource_size_t start, len;
	int i;

	qdev->bars = QCOM_PCI_BAR_NUM;
	ret = pci_assign_resource(pci_dev, qdev->bars);
	if (ret)
		return ret;

	ret = pci_enable_device(pci_dev);
	if (ret)
		return ret;

	ret = pci_request_region(pci_dev, qdev->bars, "mhi");
	if (ret)
		goto error_request_region;

	pci_set_master(pci_dev);

	ret = pci_set_dma_mask(pci_dev, DMA_BIT_MASK(64));
	if (ret)
		goto error_dma_mask;

	ret = pci_set_consistent_dma_mask(pci_dev, DMA_BIT_MASK(64));
	if (ret)
		goto error_dma_mask;

	start = pci_resource_start(pci_dev, qdev->bars);
	len = pci_resource_len(pci_dev, qdev->bars);
	mhi_cntrl->regs = ioremap_nocache(start, len);
	if (!mhi_cntrl->regs)
		goto error_ioremap;

	ret = pci_alloc_irq_vectors(pci_dev, mhi_cntrl->nr_irqs_req,
				    mhi_cntrl->nr_irqs_req, PCI_IRQ_MSI);
	if (IS_ERR_VALUE((ulong)ret) || ret < mhi_cntrl->nr_irqs_req)
		goto error_req_irq;

	mhi_cntrl->nr_irqs = ret;
	mhi_cntrl->irq = kmalloc_array(mhi_cntrl->nr_irqs,
				       sizeof(*mhi_cntrl->irq), GFP_KERNEL);
	if (!mhi_cntrl->irq) {
		ret = -ENOMEM;
		goto error_alloc_irq_vec;
	}

	for (i = 0; i < mhi_cntrl->nr_irqs; i++) {
		mhi_cntrl->irq[i] = pci_irq_vector(pci_dev, i);
		if (mhi_cntrl->irq[i] < 0) {
			ret = mhi_cntrl->irq[i];
			goto error_get_irq_vec;
		}
	}

	/* Configure runtime PM */
	pm_runtime_set_autosuspend_delay(&pci_dev->dev, MHI_QCOM_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(&pci_dev->dev);
	pm_suspend_ignore_children(&pci_dev->dev, true);

	return 0;

error_get_irq_vec:
	kfree(mhi_cntrl->irq);
	mhi_cntrl->irq = NULL;

error_alloc_irq_vec:
	pci_free_irq_vectors(pci_dev);

error_req_irq:
	iounmap(mhi_cntrl->regs);

error_ioremap:
error_dma_mask:
	pci_clear_master(pci_dev);
	pci_release_region(pci_dev, qdev->bars);

error_request_region:
	pci_disable_device(pci_dev);

	return ret;
}

static int qcom_mhi_runtime_suspend(struct device *dev)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct mhi_qcom_dev *qdev = pci_get_drvdata(pci_dev);
	struct mhi_controller *mhi_cntrl = qdev->mhi_cntrl;
	int ret = 0;

	dev_info(mhi_cntrl->dev, "Enter\n");

	mutex_lock(&mhi_cntrl->pm_mutex);

	ret = mhi_pm_suspend(mhi_cntrl);

	mutex_unlock(&mhi_cntrl->pm_mutex);

	return ret;
}

static int qcom_mhi_runtime_idle(struct device *dev)
{
	/*
	 * In MHI power management, the device will go to runtime suspend only
	 * after entering MHI State M2, which is handled by the MHI core. So
	 * return -EBUSY here to indicate the PM framework that the device is
	 * not ready to be suspended even if the usage count is 0.
	 */
	return -EBUSY;
}

static int qcom_mhi_runtime_resume(struct device *dev)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct mhi_qcom_dev *qdev = pci_get_drvdata(pci_dev);
	struct mhi_controller *mhi_cntrl = qdev->mhi_cntrl;
	int ret = 0;

	dev_info(mhi_cntrl->dev, "Enter\n");

	mutex_lock(&mhi_cntrl->pm_mutex);

	if (!qdev->powered_on) {
		mutex_unlock(&mhi_cntrl->pm_mutex);
		return 0;
	}

	/* enter M0 state */
	ret = mhi_pm_resume(mhi_cntrl);

	mutex_unlock(&mhi_cntrl->pm_mutex);

	return ret;
}

static int qcom_mhi_system_resume(struct device *dev)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct mhi_qcom_dev *qdev = pci_get_drvdata(pci_dev);
	struct mhi_controller *mhi_cntrl = qdev->mhi_cntrl;
	int ret = 0;

	ret = qcom_mhi_runtime_resume(dev);
	if (ret) {
		dev_err(mhi_cntrl->dev, "Failed to resume link\n");
	} else {
		pm_runtime_set_active(dev);
		pm_runtime_enable(dev);
	}

	return ret;
}

static int qcom_mhi_system_suspend(struct device *dev)
{
	/* If rpm status is still active then force suspend */
	if (!pm_runtime_status_suspended(dev))
		return qcom_mhi_runtime_suspend(dev);

	pm_runtime_set_suspended(dev);
	pm_runtime_disable(dev);

	return 0;
}

/* Checks if link is down */
static int qcom_mhi_link_status(struct mhi_controller *mhi_cntrl, void *priv)
{
	struct mhi_qcom_dev *qdev = priv;
	u16 dev_id;
	int ret;

	/* Try reading device id, if dev id don't match then link is down */
	ret = pci_read_config_word(qdev->pci_dev, PCI_DEVICE_ID, &dev_id);

	return (ret || dev_id != mhi_cntrl->dev_id) ? -EIO : 0;
}

static int qcom_mhi_runtime_get(struct mhi_controller *mhi_cntrl, void *priv)
{
	struct mhi_qcom_dev *qdev = priv;
	struct device *dev = &qdev->pci_dev->dev;

	return pm_runtime_get(dev);
}

static void qcom_mhi_runtime_put(struct mhi_controller *mhi_cntrl, void *priv)
{
	struct mhi_qcom_dev *qdev = priv;
	struct device *dev = &qdev->pci_dev->dev;

	pm_runtime_put_noidle(dev);
}

static void qcom_mhi_status_cb(struct mhi_controller *mhi_cntrl,
			       void *priv, enum mhi_callback reason)
{
	struct mhi_qcom_dev *qdev = priv;
	struct device *dev = &qdev->pci_dev->dev;

	if (reason == MHI_CB_IDLE) {
		pm_runtime_mark_last_busy(dev);
		pm_request_autosuspend(dev);
	}
}

static struct mhi_controller *qcom_mhi_register_controller(struct pci_dev *pci_dev)
{
	struct mhi_controller *mhi_cntrl;
	const struct firmware_info *firmware_info;
	int ret, i;

	mhi_cntrl = mhi_alloc_controller();
	if (!mhi_cntrl)
		return ERR_PTR(-ENOMEM);

	mhi_cntrl->dev = &pci_dev->dev;
	mhi_cntrl->dev_id = pci_dev->device;
	mhi_cntrl->bus_id = pci_dev->bus->number;

	/*
	 * Covers the entire possible physical ram region. Remote side is
	 * going to calculate a size of this range, so subtract 1 to prevent
	 * rollover.
	 */
	mhi_cntrl->iova_start = 0;
	mhi_cntrl->iova_stop = U64_MAX - 1;

	/* Setup MHI power management callbacks */
	mhi_cntrl->status_cb = qcom_mhi_status_cb;
	mhi_cntrl->runtime_get = qcom_mhi_runtime_get;
	mhi_cntrl->runtime_put = qcom_mhi_runtime_put;
	mhi_cntrl->link_status = qcom_mhi_link_status;

	mhi_cntrl->name = pci_name(pci_dev);

	ret = mhi_register_controller(mhi_cntrl, &mhi_qcom_config);
	if (ret)
		goto error_register;

	for (i = 0; i < ARRAY_SIZE(firmware_table); i++) {
		firmware_info = firmware_table + i;
		if (mhi_cntrl->dev_id == firmware_info->dev_id)
			break;
	}

	mhi_cntrl->fw_image = firmware_info->fw_image;
	mhi_cntrl->edl_image = firmware_info->edl_image;

	return mhi_cntrl;

error_register:
	mhi_free_controller(mhi_cntrl);

	return ERR_PTR(-EINVAL);
}

int mhi_qcom_pci_probe(struct pci_dev *pci_dev,
		  const struct pci_device_id *device_id)
{
	struct mhi_controller *mhi_cntrl;
	struct mhi_qcom_dev *qdev;
	int ret;

	qdev = kzalloc(sizeof(*qdev), GFP_KERNEL);
	if (!qdev)
		return -ENOMEM;

	qdev->pci_dev = pci_dev;
	pci_set_drvdata(pci_dev, qdev);

	mhi_cntrl = qcom_mhi_register_controller(pci_dev);
	if (IS_ERR(mhi_cntrl)) {
		mhi_free_controller(mhi_cntrl);
		pci_set_drvdata(pci_dev, NULL);
		return PTR_ERR(mhi_cntrl);
	}

	ret = qcom_mhi_init_pci_dev(mhi_cntrl, qdev);
	if (ret)
		goto error_init_pci;

	/* Start power up sequence */
	ret = mhi_async_power_up(mhi_cntrl);
	if (ret)
		goto error_power_up;

	mhi_controller_set_devdata(mhi_cntrl, qdev);
	qdev->powered_on = true;
	qdev->mhi_cntrl = mhi_cntrl;

	/* Decrement usage count as recommended by PCI framework */
	pm_runtime_put_noidle(&pci_dev->dev);

	/* Mask last_busy so that runtime PM won't issue suspend immediately */
	pm_runtime_mark_last_busy(&pci_dev->dev);

	/* Allow runtime PM */
	pm_runtime_allow(&pci_dev->dev);

	return 0;

error_power_up:
	qcom_mhi_deinit_pci_dev(mhi_cntrl, qdev);

error_init_pci:
	mhi_unregister_controller(mhi_cntrl);
	mhi_free_controller(mhi_cntrl);
	pci_set_drvdata(pci_dev, NULL);

	return ret;
}

static void mhi_qcom_pci_remove(struct pci_dev *pci_dev)
{
	struct mhi_qcom_dev *qdev = pci_get_drvdata(pci_dev);
	struct mhi_controller *mhi_cntrl;

	if (!qdev)
		return;

	mhi_cntrl = qdev->mhi_cntrl;
	mhi_power_down(mhi_cntrl, true);
	mhi_unregister_controller(mhi_cntrl);
	mhi_free_controller(mhi_cntrl);
	qcom_mhi_deinit_pci_dev(mhi_cntrl, qdev);
}

static const struct dev_pm_ops pm_ops = {
	SET_RUNTIME_PM_OPS(qcom_mhi_runtime_suspend,
			   qcom_mhi_runtime_resume,
			   qcom_mhi_runtime_idle)
	SET_SYSTEM_SLEEP_PM_OPS(qcom_mhi_system_suspend, qcom_mhi_system_resume)
};

static struct pci_device_id mhi_qcom_pci_device_id[] = {
	{ PCI_DEVICE(QCOM_PCI_VENDOR_ID, 0x0300) },
	{ PCI_DEVICE(QCOM_PCI_VENDOR_ID, 0x0301) },
	{ PCI_DEVICE(QCOM_PCI_VENDOR_ID, 0x0302) },
	{ PCI_DEVICE(QCOM_PCI_VENDOR_ID, 0x0303) },
	{ PCI_DEVICE(QCOM_PCI_VENDOR_ID, 0x0304) },
	{ PCI_DEVICE(QCOM_PCI_VENDOR_ID, 0x0305) },
	{ PCI_DEVICE(QCOM_PCI_VENDOR_ID, 0x1101) },
	{ /* sentinel */ },
};

static struct pci_driver mhi_qcom_pci_driver = {
	.name = "mhi_qcom",
	.id_table = mhi_qcom_pci_device_id,
	.probe = mhi_qcom_pci_probe,
	.remove = mhi_qcom_pci_remove,
	.driver = {
		.pm = &pm_ops
	}
};

module_pci_driver(mhi_qcom_pci_driver);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("MHI_CORE");
MODULE_DESCRIPTION("MHI Qcom Controller Driver");
