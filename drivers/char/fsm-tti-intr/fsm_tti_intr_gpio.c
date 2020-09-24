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
#include <linux/of.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>

#include "fsm_tti_intr.h"

static irqreturn_t fsm_tti_gpio_irq_handler(int irq, void *irq_data)
{
	struct fsm_tti_mmap_info *sdata;
	struct fsm_tti_intr_drv *tti_intr_drv =
		(struct fsm_tti_intr_drv *)irq_data;

	/* update the SFN and slot number */
	sdata = tti_intr_drv->shared_data;
	if (sdata && (tti_intr_drv->is_seeding_done)) {
		/* Update stats */
		sdata->abs_recv_time = ktime_get();
		tti_intr_drv->debugfs_stats.current_tti_recv_time =
			sdata->abs_recv_time;

		/* Keep track of the time when first tti intr is received */
		if (tti_intr_drv->is_first_tti_intr == false) {
			tti_intr_drv->is_first_tti_intr = true;
			tti_intr_drv->debugfs_stats.first_tti_recv_time =
				sdata->abs_recv_time;
		} else {

			sdata->sfn_slot_info.slot =
				(sdata->sfn_slot_info.slot + 1) %
				sdata->max_slot;

			if (sdata->sfn_slot_info.slot == 0)
				sdata->sfn_slot_info.sfn =
				(sdata->sfn_slot_info.sfn + 1) &
				FSM_TTI_MAX_SFN_MOD_FACTOR;

			/* Make sure sfn/slot is updated before moving ahead */
			tti_smp_mb();
		}
		sdata->intr_recv_count = sdata->intr_recv_count + 1;
		tti_intr_drv->debugfs_stats.current_tti_count =
			sdata->intr_recv_count;
		/* Make sure timestamps are updated before sfn/slot */
		tti_smp_mb();

		/* wake up the poll ops */
		if (tti_intr_drv->is_poll_enabled) {
			atomic_set(&tti_intr_drv->tti_updated, 1);
			wake_up(&tti_intr_drv->tti_poll_waitqueue);
		}
	}
	return IRQ_HANDLED;
}

int fsm_tti_intr_probe(struct platform_device *pdev)
{
	int ret;
	struct page *page;
	unsigned long flags;
	const char *gpio_label;
	struct device_node *np;
	struct fsm_tti_intr_drv *tti_intr_drv;
	struct fsm_tti_gpio_platform_data *platform_data =
		pdev->dev.platform_data;

	FSM_TTI_INFO("FSM-TTI: probing device\n");

	tti_intr_drv = kzalloc(sizeof(*tti_intr_drv), GFP_KERNEL);
	if (IS_ERR(tti_intr_drv))
		return -ENOMEM;

	tti_intr_drv->dev = &pdev->dev;

	np = pdev->dev.of_node;
	if (!np) {
		kfree(tti_intr_drv);
		return -ENOENT;
	}

	/* allocate space for device info */
	tti_intr_drv->device_data = devm_kzalloc(&pdev->dev,
				sizeof(struct fsm_tti_gpio_device_data),
				GFP_KERNEL);
	if (!tti_intr_drv->device_data) {
		kfree(tti_intr_drv);
		return -ENOMEM;
	}

	if (platform_data) {
		/* update the device info to the driver context */
		tti_intr_drv->device_data->gpio_pin =
			platform_data->gpio_pin;
		tti_intr_drv->device_data->assert_falling_edge =
			platform_data->assert_falling_edge;
		tti_intr_drv->device_data->capture_clear =
			platform_data->capture_clear;
		gpio_label = platform_data->gpio_label;
	} else {
		/* read device tree information */
		ret = of_get_gpio(np, 0);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"failed to get GPIO from device tree\n");
			goto cleanup;

		}
		tti_intr_drv->device_data->gpio_pin = ret;
		gpio_label = FSM_TTI_GPIO_NAME;

		if (of_get_property(np, "assert-falling-edge", NULL))
			tti_intr_drv->device_data->assert_falling_edge = true;
	}

	/* GPIO setup */
	ret = devm_gpio_request(&pdev->dev,
				tti_intr_drv->device_data->gpio_pin,
				gpio_label);
	if (ret) {
		dev_err(&pdev->dev, "failed to request GPIO %u\n",
			tti_intr_drv->device_data->gpio_pin);
		goto cleanup;
	}

	ret = gpio_direction_input(tti_intr_drv->device_data->gpio_pin);
	if (ret) {
		dev_err(&pdev->dev, "failed to set pin direction\n");
		goto cleanup;
	}

	/* IRQ setup */
	ret = gpio_to_irq(tti_intr_drv->device_data->gpio_pin);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to map GPIO to IRQ: %d\n", ret);
		goto cleanup;
	}
	tti_intr_drv->device_data->irq = ret;

	/* interrupt handler flow type */
	flags = tti_intr_drv->device_data->assert_falling_edge ?
		IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
	if (tti_intr_drv->device_data->capture_clear) {
		flags |= ((flags & IRQF_TRIGGER_RISING) ?
				IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
	}

	/* prepare a device name */
	snprintf(tti_intr_drv->device_data->name,
		FSM_TTI_MAX_NAME_LEN - 1,
		"%s.%d",
		pdev->name, pdev->id);

	/* register IRQ interrupt handler */
	ret = devm_request_irq(&pdev->dev,
				tti_intr_drv->device_data->irq,
				fsm_tti_gpio_irq_handler,
				flags,
				tti_intr_drv->device_data->name,
				tti_intr_drv);
	if (ret) {
		dev_err(&pdev->dev, "failed to acquire IRQ %d\n",
			tti_intr_drv->device_data->irq);
		goto cleanup;
	}

	/* keep a driver reference to the device structure */
	platform_set_drvdata(pdev, tti_intr_drv);

	/* allocate shared data */
	page = alloc_page(GFP_KERNEL);
	if (page) {
		tti_intr_drv->page = page;
		tti_intr_drv->shared_data =
			(struct fsm_tti_mmap_info *)page_address(page);
	}
	if (IS_ERR(tti_intr_drv->shared_data)) {
		FSM_TTI_ERROR("FSM-TTI: %s: failed to alloc shared memory\n",
			__func__);
		ret = -ENOMEM;
		goto cleanup;
	}

	/* initialize char interface to userspace */
	ret = fsm_tti_cdev_init(tti_intr_drv);
	if (ret)
		goto cleanup_shared_data;

	ret = fsm_tti_debugfs_init(tti_intr_drv);
	if (ret)
		goto cleanup_cdev;

	/* initialize wait queue */
	init_waitqueue_head(&tti_intr_drv->tti_poll_waitqueue);
	/* initialize the flags */
	atomic_set(&tti_intr_drv->tti_updated, 0);
	tti_intr_drv->is_seeding_done = false;
	tti_intr_drv->is_poll_enabled = false;
	tti_intr_drv->is_first_tti_intr = false;

	FSM_TTI_INFO("FSM-TTI: module initialized\n");
	return 0;

cleanup_cdev:
	fsm_tti_cdev_cleanup(tti_intr_drv);
cleanup_shared_data:
	__free_page(tti_intr_drv->page);
	tti_intr_drv->shared_data = NULL;
cleanup:
	kfree(tti_intr_drv);
	FSM_TTI_ERROR("FSM-TTI: module init failed!\n");
	return ret;
}

int fsm_tti_intr_remove(struct platform_device *pdev)
{
	struct fsm_tti_intr_drv *tti_intr_drv = platform_get_drvdata(pdev);

	if (tti_intr_drv) {
		fsm_tti_debugfs_cleanup(tti_intr_drv);
		fsm_tti_cdev_cleanup(tti_intr_drv);
		__free_page(tti_intr_drv->page);
		tti_intr_drv->shared_data = NULL;
		kfree(tti_intr_drv);
	}
	FSM_TTI_INFO("FSM-TTI: module removed\n");
	return 0;
}

static const struct of_device_id fsm_tti_intr_of_table[] = {
	{ .compatible = "tti-gpio" },
	{ },
};
MODULE_DEVICE_TABLE(of, fsm_tti_intr_of_table);

static struct platform_driver __fsm_tti_intr_platform_drv = {
	.probe  = fsm_tti_intr_probe,
	.remove = fsm_tti_intr_remove,
	.driver = {
		.name           = KBUILD_MODNAME,
		.of_match_table = fsm_tti_intr_of_table,
		.owner          = THIS_MODULE,
	},
};

module_platform_driver(__fsm_tti_intr_platform_drv);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("FSM TTI interrupt driver");

