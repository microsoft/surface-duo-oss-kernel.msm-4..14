/*
 * Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>

#include <asm/proc-fns.h>
#include <asm/suspend.h>

#include <soc/qcom/pm.h>
#include <soc/qcom/pm.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/scm-boot.h>


#define SCM_CMD_TERMINATE_PC	0x2
#define SCM_FLUSH_FLAG_MASK	0x3
#define SCM_L2_ON		0x0
#define SCM_L2_OFF		0x1
#define MAX_PMIC_DATA		2
#define MAX_SEQ_DATA		64
#define SPM_CTL_INDEX		0x7f
#define SPM_CTL_INDEX_SHIFT	4
#define SPM_CTL_EN		BIT(0)

enum spm_reg {
	SPM_REG_CFG,
	SPM_REG_SPM_CTL,
	SPM_REG_DLY,
	SPM_REG_PMIC_DLY,
	SPM_REG_PMIC_DATA_0,
	SPM_REG_PMIC_DATA_1,
	SPM_REG_VCTL,
	SPM_REG_SEQ_ENTRY,
	SPM_REG_SPM_STS,
	SPM_REG_PMIC_STS,
	SPM_REG_NR,
};

struct spm_reg_data {
	const u8 *reg_offset;
	u32 spm_cfg;
	u32 spm_dly;
	u32 pmic_dly;
	u32 pmic_data[MAX_PMIC_DATA];
	u8 seq[MAX_SEQ_DATA];
	u8 start_index[PM_SLEEP_MODE_NR];
};

struct spm_driver_data {
	void __iomem *reg_base;
	const struct spm_reg_data *reg_data;
};

static const u8 spm_reg_offset_v2_1[SPM_REG_NR] = {
	[SPM_REG_CFG]		= 0x08,
	[SPM_REG_SPM_CTL]	= 0x30,
	[SPM_REG_DLY]		= 0x34,
	[SPM_REG_SEQ_ENTRY]	= 0x80,
};

/* SPM register data for 8974, 8084 */
static const struct spm_reg_data spm_reg_8974_8084_cpu  = {
	.reg_offset = spm_reg_offset_v2_1,
	.spm_cfg = 0x1,
	.spm_dly = 0x3C102800,
	.seq = { 0x03, 0x0B, 0x0F, 0x00, 0x20, 0x80, 0x10, 0xE8, 0x5B, 0x03,
		0x3B, 0xE8, 0x5B, 0x82, 0x10, 0x0B, 0x30, 0x06, 0x26, 0x30,
		0x0F },
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 3,
};

static const u8 spm_reg_offset_v1_1[SPM_REG_NR] = {
	[SPM_REG_CFG]		= 0x08,
	[SPM_REG_SPM_CTL]	= 0x20,
	[SPM_REG_PMIC_DLY]	= 0x24,
	[SPM_REG_PMIC_DATA_0]	= 0x28,
	[SPM_REG_PMIC_DATA_1]	= 0x2C,
	[SPM_REG_SEQ_ENTRY]	= 0x80,
};

/* SPM register data for 8064 */
static const struct spm_reg_data spm_reg_8064_cpu = {
	.reg_offset = spm_reg_offset_v1_1,
	.spm_cfg = 0x1F,
	.pmic_dly = 0x02020004,
	.pmic_data[0] = 0x0084009C,
	.pmic_data[1] = 0x00A4001C,
	.seq = { 0x03, 0x0F, 0x00, 0x24, 0x54, 0x10, 0x09, 0x03, 0x01,
		0x10, 0x54, 0x30, 0x0C, 0x24, 0x30, 0x0F },
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 2,
};

static DEFINE_PER_CPU_SHARED_ALIGNED(struct spm_driver_data *, cpu_spm_drv);

static inline void spm_register_write(struct spm_driver_data *drv,
					enum spm_reg reg, u32 val)
{
	if (drv->reg_data->reg_offset[reg])
		writel_relaxed(val, drv->reg_base +
				drv->reg_data->reg_offset[reg]);
}

/* Ensure a guaranteed write, before return */
static inline void spm_register_write_sync(struct spm_driver_data *drv,
					enum spm_reg reg, u32 val)
{
	u32 ret;

	if (!drv->reg_data->reg_offset[reg])
		return;

	do {
		writel_relaxed(val, drv->reg_base +
				drv->reg_data->reg_offset[reg]);
		ret = readl_relaxed(drv->reg_base +
				drv->reg_data->reg_offset[reg]);
		if (ret == val)
			break;
		cpu_relax();
	} while (1);
}

static inline u32 spm_register_read(struct spm_driver_data *drv,
					enum spm_reg reg)
{
	return readl_relaxed(drv->reg_base + drv->reg_data->reg_offset[reg]);
}

static void spm_set_low_power_mode(enum pm_sleep_mode mode)
{
	struct spm_driver_data *drv = per_cpu(cpu_spm_drv,
						smp_processor_id());
	u32 start_index;
	u32 ctl_val;

	start_index = drv->reg_data->start_index[mode];

	ctl_val = spm_register_read(drv, SPM_REG_SPM_CTL);
	ctl_val &= ~(SPM_CTL_INDEX << SPM_CTL_INDEX_SHIFT);
	ctl_val |= start_index << SPM_CTL_INDEX_SHIFT;
	ctl_val |= SPM_CTL_EN;
	spm_register_write_sync(drv, SPM_REG_SPM_CTL, ctl_val);
}

static int qcom_pm_collapse(unsigned long int unused)
{
	int ret;
	u32 flag;
	int cpu = smp_processor_id();

	ret = scm_set_warm_boot_addr(cpu_resume, cpu);
	if (ret)
		return ret;

	flag = SCM_L2_ON & SCM_FLUSH_FLAG_MASK;
	ret = scm_call_atomic1(SCM_SVC_BOOT, SCM_CMD_TERMINATE_PC, flag);

	/*
	 * Returns here only if there was a pending interrupt and we did not
	 * power down as a result.
	 */
	/* Hack:: Ignore scm call return values in power down path */
	return 0;
}

static int qcom_cpu_standby(void *unused)
{
	spm_set_low_power_mode(PM_SLEEP_MODE_STBY);
	cpu_do_idle();

	return 0;
}

static int qcom_cpu_spc(void *unused)
{
	int ret;

	spm_set_low_power_mode(PM_SLEEP_MODE_STBY);
	cpu_pm_enter();
	ret = cpu_suspend(0, qcom_pm_collapse);
	cpu_pm_exit();

	return ret;
}

static struct spm_driver_data *spm_get_drv(struct platform_device *pdev,
		int *spm_cpu)
{
	struct spm_driver_data *drv = NULL;
	struct device_node *cpu_node, *saw_node;
	int cpu;
	bool found;

	for_each_possible_cpu(cpu) {
		cpu_node = of_cpu_device_node_get(cpu);
		if (!cpu_node)
			continue;
		saw_node = of_parse_phandle(cpu_node, "qcom,saw", 0);
		found = (saw_node == pdev->dev.of_node);
		of_node_put(saw_node);
		of_node_put(cpu_node);
		if (found)
			break;
	}

	if (found) {
		drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_KERNEL);
		if (drv)
			*spm_cpu = cpu;
	}

	return drv;
}

static const struct of_device_id spm_match_table[] = {
	{ .compatible = "qcom,msm8974-saw2-v2.1-cpu",
	  .data = &spm_reg_8974_8084_cpu },
	{ .compatible = "qcom,apq8084-saw2-v2.1-cpu",
	  .data = &spm_reg_8974_8084_cpu },
	{ .compatible = "qcom,apq8064-saw2-v1.1-cpu",
	  .data = &spm_reg_8064_cpu },
	{ },
};

static int spm_dev_probe(struct platform_device *pdev)
{
	struct spm_driver_data *drv;
	struct resource *res;
	const struct of_device_id *match_id;
	void __iomem *addr;
	int cpu;
	struct cpuidle_device *dev;

	drv = spm_get_drv(pdev, &cpu);
	if (!drv)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drv->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(drv->reg_base))
		return PTR_ERR(drv->reg_base);

	match_id = of_match_node(spm_match_table, pdev->dev.of_node);
	if (!match_id)
		return -ENODEV;

	drv->reg_data = match_id->data;

	/* Write the SPM sequences first.. */
	addr = drv->reg_base + drv->reg_data->reg_offset[SPM_REG_SEQ_ENTRY];
	__iowrite32_copy(addr, drv->reg_data->seq,
			ARRAY_SIZE(drv->reg_data->seq) / 4);

	/*
	 * ..and then the control registers.
	 * On some SoC if the control registers are written first and if the
	 * CPU was held in reset, the reset signal could trigger the SPM state
	 * machine, before the sequences are completely written.
	 */
	spm_register_write(drv, SPM_REG_CFG, drv->reg_data->spm_cfg);
	spm_register_write(drv, SPM_REG_DLY, drv->reg_data->spm_dly);
	spm_register_write(drv, SPM_REG_PMIC_DLY, drv->reg_data->pmic_dly);
	spm_register_write(drv, SPM_REG_PMIC_DATA_0,
				drv->reg_data->pmic_data[0]);
	spm_register_write(drv, SPM_REG_PMIC_DATA_1,
				drv->reg_data->pmic_data[1]);

	per_cpu(cpu_spm_drv, cpu) = drv;

	/* Register the cpuidle device for the cpu, we are ready for cpuidle */
	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->cpu = cpu;
	return cpuidle_register_device(dev);
}

static struct qcom_cpu_pm_ops lpm_ops = {
	.standby = qcom_cpu_standby,
	.spc = qcom_cpu_spc,
};

static struct platform_device qcom_cpuidle_drv = {
	.name	= "qcom_cpuidle",
	.id	= -1,
	.dev.platform_data = &lpm_ops,
};

static struct platform_driver spm_driver = {
	.probe = spm_dev_probe,
	.driver = {
		.name = "saw",
		.of_match_table = spm_match_table,
	},
};

static int __init qcom_spm_init(void)
{
	int ret;

	/*
	 * cpuidle driver need to registered before the cpuidle device
	 * for any cpu. Register the device for the the cpuidle driver.
	 */
	ret = platform_device_register(&qcom_cpuidle_drv);
	if (ret)
		return ret;

	return platform_driver_register(&spm_driver);
}
module_init(qcom_spm_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SAW power controller driver");
MODULE_ALIAS("platform:saw");
