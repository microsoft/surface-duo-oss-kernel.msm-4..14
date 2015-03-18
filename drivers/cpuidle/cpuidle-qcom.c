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
	int ret;

	ret = lpm_ops->standby(NULL);
	if (ret)
		return ret;

	return index;
}

static int qcom_cpu_spc(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int index)
{
	int ret;

	ret = lpm_ops->spc(NULL);
	if (ret)
		return ret;

	return index;
}

static struct cpuidle_driver qcom_cpuidle_driver = {
	.name = "qcom_cpuidle",
};

static const struct of_device_id qcom_idle_state_match[] = {
	{ .compatible = "qcom,idle-state-stby", .data = qcom_cpu_stby },
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

	/*
	 * We will not register for cpu's cpuidle device here,
	 * they will be registered as and when their power controllers
	 * are ready.
	 */
	return cpuidle_register_driver(drv);
}

static struct platform_driver qcom_cpuidle = {
	.probe	= qcom_cpuidle_probe,
	.driver = {
		.name = "qcom_cpuidle",
	},
};

/*
 * Register the driver early so the we have a successul registration
 * when the device shows up.
 * This way the cpuidle driver could be registered before the cpuidle
 * devices are registered.
 */
static int __init qcom_cpuidle_driver_init(void)
{
	return platform_driver_register(&qcom_cpuidle);
}
core_initcall(qcom_cpuidle_driver_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CPUIDLE driver for QCOM SoC");
MODULE_ALIAS("platform:qcom-cpuidle");
