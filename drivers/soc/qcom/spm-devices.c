/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include <soc/qcom/spm.h>

#include "spm-drv.h"

/**
 * All related information for an SPM device
 * Helps manage the collective.
 */
struct msm_spm_device {
	bool initialized;
	struct msm_spm_driver_data drv;
};

static DEFINE_PER_CPU_SHARED_ALIGNED(struct msm_spm_device, msm_cpu_spm_device);

/**
 * msm_spm_set_low_power_mode() - Configure SPM start address for low power mode
 * @mode: SPM LPM mode to enter
 */
int msm_spm_set_low_power_mode(u32 mode)
{
	struct msm_spm_device *dev = &__get_cpu_var(msm_cpu_spm_device);
	int ret = -EINVAL;

	if (!dev->initialized)
		return -ENXIO;

	if (mode == MSM_SPM_MODE_DISABLED)
		ret = msm_spm_drv_set_spm_enable(&dev->drv, false);
	else if (!msm_spm_drv_set_spm_enable(&dev->drv, true))
		ret = msm_spm_drv_set_low_power_mode(&dev->drv, mode);

	return ret;
}
EXPORT_SYMBOL(msm_spm_set_low_power_mode);

static int get_cpu_id(struct device_node *node)
{
	struct device_node *cpu_node;
	u32 cpu;
	int ret = -EINVAL;
	char *key = "qcom,cpu";

	cpu_node = of_parse_phandle(node, key, 0);
	if (cpu_node) {
		for_each_possible_cpu(cpu) {
			if (of_get_cpu_node(cpu, NULL) == cpu_node)
				return cpu;
		}
	}
	return ret;
}

static struct msm_spm_device *msm_spm_get_device(struct platform_device *pdev)
{
	struct msm_spm_device *dev = NULL;
	int cpu = get_cpu_id(pdev->dev.of_node);

	if ((cpu >= 0) && cpu < num_possible_cpus())
		dev = &per_cpu(msm_cpu_spm_device, cpu);

	return dev;
}

static int msm_spm_dev_probe(struct platform_device *pdev)
{
	int ret;
	int i;
	struct device_node *node = pdev->dev.of_node;
	char *key;
	u32 val;
	struct msm_spm_mode modes[MSM_SPM_MODE_NR];
	struct msm_spm_device *spm_dev;
	struct resource *res;
	u32 mode_count = 0;

	struct spm_of {
		char *key;
		u32 id;
	};

	/* SPM Configuration registers */
	struct spm_of spm_of_data[] = {
		{"qcom,saw2-clk-div", MSM_SPM_REG_SAW2_CFG},
		{"qcom,saw2-enable", MSM_SPM_REG_SAW2_SPM_CTL},
		{"qcom,saw2-delays", MSM_SPM_REG_SAW2_SPM_DLY},
	};

	/* SPM sleep sequences */
	struct spm_of mode_of_data[] = {
		{"qcom,saw2-spm-cmd-wfi", MSM_SPM_MODE_CLOCK_GATING},
		{"qcom,saw2-spm-cmd-spc", MSM_SPM_MODE_POWER_COLLAPSE},
		{"qcom,saw2-spm-cmd-ret", MSM_SPM_MODE_RETENTION},
	};

	 /* Get the right SPM device */
	spm_dev = msm_spm_get_device(pdev);
	if (IS_ERR_OR_NULL(spm_dev))
		return -EINVAL;

	/* Get the SAW start address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -EINVAL;
		goto fail;
	}
	spm_dev->drv.reg_base_addr = devm_ioremap(&pdev->dev, res->start,
					resource_size(res));
	if (!spm_dev->drv.reg_base_addr) {
		ret = -ENOMEM;
		goto fail;
	}

	/* Read the SPM configuration register values */
	for (i = 0; i < ARRAY_SIZE(spm_of_data); i++) {
		ret = of_property_read_u32(node, spm_of_data[i].key, &val);
		if (ret)
			continue;
		spm_dev->drv.reg_shadow[spm_of_data[i].id] = val;
	}

	/* Read the byte arrays for the SPM sleep sequences */
	for (i = 0; i < ARRAY_SIZE(mode_of_data); i++) {
		modes[mode_count].start_addr = 0;
		key = mode_of_data[i].key;
		modes[mode_count].cmd =
			(u8 *)of_get_property(node, key, &val);
		if (!modes[mode_count].cmd)
			continue;
		modes[mode_count].mode = mode_of_data[i].id;
		mode_count++;
	}

	spm_dev->drv.modes = devm_kcalloc(&pdev->dev, mode_count,
						sizeof(modes[0]), GFP_KERNEL);
	if (!spm_dev->drv.modes)
		return -ENOMEM;
	spm_dev->drv.num_modes = mode_count;
	memcpy(spm_dev->drv.modes, &modes[0], sizeof(modes[0]) * mode_count);

	/* Initialize the hardware */
	ret = msm_spm_drv_init(&spm_dev->drv);
	if (ret) {
		kfree(spm_dev->drv.modes);
		return ret;
	}

	spm_dev->initialized = true;
	return ret;

fail:
	dev_err(&pdev->dev, "SPM device probe failed: %d\n", ret);
	return ret;
}

static struct of_device_id msm_spm_match_table[] = {
	{.compatible = "qcom,spm-v2.1"},
	{},
};

static struct platform_driver msm_spm_device_driver = {
	.probe = msm_spm_dev_probe,
	.driver = {
		.name = "spm-v2",
		.owner = THIS_MODULE,
		.of_match_table = msm_spm_match_table,
	},
};

static int __init msm_spm_device_init(void)
{
	return platform_driver_register(&msm_spm_device_driver);
}
device_initcall(msm_spm_device_init);
