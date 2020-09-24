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
#ifndef __FSM_TTI_INTR__
#define __FSM_TTI_INTR__

#ifndef __KERNEL__
#define __KERNEL__
#endif

#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/platform_device.h>

#include <linux/fsm_tti_intr_if.h>

#define FSM_TTI_MODULE_NAME	"fsm-tti"
#define FSM_TTI_DEV_CLASS_NAME	FSM_TTI_MODULE_NAME
#define FSM_TTI_CDEV_NAME	FSM_TTI_MODULE_NAME

#define FSM_TTI_DEBUG	pr_debug
#define FSM_TTI_INFO	pr_info
#define FSM_TTI_ERROR	pr_err
#define FSM_TTI_WARN	pr_warn

#define FSM_TTI_GPIO_NAME	"tti-gpio"
#define FSM_TTI_PAGE_SIZE	PAGE_SIZE
#define FSM_TTI_MAX_NAME_LEN	32

#define dmb(opt) asm volatile("dmb " #opt : : : "memory")
#define tti_smp_mb() dmb(ish)

/* Info for tti interrupt gpio platform */
struct fsm_tti_gpio_platform_data {
	bool assert_falling_edge;
	bool capture_clear;
	unsigned int gpio_pin;
	const char *gpio_label;
};

/* Info for each registered platform device */
struct fsm_tti_gpio_device_data {
	int irq;			/* IRQ used as TTI source */
	bool assert_falling_edge;
	bool capture_clear;
	unsigned int gpio_pin;
	char name[FSM_TTI_MAX_NAME_LEN];	/* symbolic name */
};

struct fsm_tti_intr_drv {
	struct device *dev;
	struct class *dev_class;
	struct cdev cdev;
	struct page *page;
	bool is_poll_enabled;
	bool is_seeding_done;
	atomic_t tti_updated;
	bool is_first_tti_intr;
	wait_queue_head_t tti_poll_waitqueue;
	struct fsm_tti_gpio_device_data *device_data;
	struct fsm_tti_mmap_info *shared_data;
	struct fsm_tti_internal_stats debugfs_stats;
};

int fsm_tti_intr_probe(struct platform_device *pdev);
int fsm_tti_intr_remove(struct platform_device *pdev);

int fsm_tti_cdev_init(struct fsm_tti_intr_drv *tti_intr_drv);
void fsm_tti_cdev_cleanup(struct fsm_tti_intr_drv *tti_intr_drv);
int fsm_tti_debugfs_init(struct fsm_tti_intr_drv *tti_intr_drv);
void fsm_tti_debugfs_cleanup(struct fsm_tti_intr_drv *tti_intr_drv);

#endif /* __FSM_TTI_INTR__ */

