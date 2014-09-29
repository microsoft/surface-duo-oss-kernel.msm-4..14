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

#include <linux/cpu_pm.h>
#include <linux/cpuidle.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include <soc/qcom/pm.h>
#include "dt_idle_states.h"

static void (*qcom_idle_enter)(enum msm_pm_sleep_mode);

static int qcom_lpm_enter_wfi(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int index)
{
	qcom_idle_enter(MSM_PM_SLEEP_MODE_WFI);

	return index;
}

static int qcom_lpm_enter_spc(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int index)
{
	cpu_pm_enter();
	qcom_idle_enter(MSM_PM_SLEEP_MODE_SPC);
	cpu_pm_exit();

	return index;
}

static struct cpuidle_driver qcom_cpuidle_driver = {
	.name	= "qcom_cpuidle",
	.owner	= THIS_MODULE,
};

static const struct of_device_id qcom_idle_state_match[] __initconst = {
	{ .compatible = "qcom,idle-state-wfi", .data = qcom_lpm_enter_wfi },
	{ .compatible = "qcom,idle-state-spc", .data = qcom_lpm_enter_spc },
	{ },
};

static int qcom_cpuidle_probe(struct platform_device *pdev)
{
	struct cpuidle_driver *drv = &qcom_cpuidle_driver;
	int ret;

	qcom_idle_enter = (void *)(pdev->dev.platform_data);

	 /* Probe for other states including platform WFI */
	ret = dt_init_idle_driver(drv, qcom_idle_state_match, 0);
	if (ret <= 0) {
		pr_err("%s: No cpuidle state found.\n", __func__);
		return ret;
	}

	ret = cpuidle_register(drv, NULL);
	if (ret) {
		pr_err("%s: failed to register cpuidle driver\n", __func__);
		return ret;
	}

	return 0;
}

static struct platform_driver qcom_cpuidle_plat_driver = {
	.probe	= qcom_cpuidle_probe,
	.driver = {
		.name = "qcom_cpuidle",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(qcom_cpuidle_plat_driver);
