/*
 * Copyright (c) 2014, Linaro Limited.
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

#include <linux/cpuidle.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <soc/qcom/pm.h>
#include "dt_idle_states.h"

static struct qcom_cpu_pm_ops *lpm_ops;

static int qcom_cpu_stby(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int index)
{
	lpm_ops->standby(NULL);

	return index;
}

static int qcom_cpu_spc(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int index)
{
	lpm_ops->spc(NULL);

	return index;
}

static struct cpuidle_driver qcom_cpuidle_driver = {
	.name	= "qcom_cpuidle",
};

static const struct of_device_id qcom_idle_state_match[] = {
	{ .compatible = "qcom,idle-state-stby", .data = qcom_cpu_stby},
	{ .compatible = "qcom,idle-state-spc", .data = qcom_cpu_spc },
	{ },
};

static int qcom_cpuidle_probe(struct platform_device *pdev)
{
	struct cpuidle_driver *drv = &qcom_cpuidle_driver;
	int ret;

	lpm_ops = pdev->dev.platform_data;

	/* Probe for other states, including standby */
	ret = dt_init_idle_driver(drv, qcom_idle_state_match, 0);
	if (ret < 0)
		return ret;

	return cpuidle_register(drv, NULL);
}

static struct platform_driver qcom_cpuidle_plat_driver = {
	.probe	= qcom_cpuidle_probe,
	.driver = {
		.name = "qcom_cpuidle",
	},
};

module_platform_driver(qcom_cpuidle_plat_driver);
