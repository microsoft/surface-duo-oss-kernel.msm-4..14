/* Copyright (c) 2010-2014, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/proc-fns.h>
#include <asm/suspend.h>

#include <soc/qcom/pm.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/scm-boot.h>
#include <soc/qcom/spm.h>

#define SCM_CMD_TERMINATE_PC	(0x2)
#define SCM_FLUSH_FLAG_MASK	(0x3)

static int set_up_boot_address(void *entry, int cpu)
{
	static int flags[NR_CPUS] = {
		SCM_FLAG_WARMBOOT_CPU0,
		SCM_FLAG_WARMBOOT_CPU1,
		SCM_FLAG_WARMBOOT_CPU2,
		SCM_FLAG_WARMBOOT_CPU3,
	};
	static DEFINE_PER_CPU(void *, last_known_entry);

	if (entry == per_cpu(last_known_entry, cpu))
		return 0;

	per_cpu(last_known_entry, cpu) = entry;
	return scm_set_boot_addr(virt_to_phys(entry), flags[cpu]);
}

static int msm_pm_collapse(unsigned long int unused)
{
	int ret;
	enum msm_pm_l2_scm_flag flag;

	ret = set_up_boot_address(cpu_resume, raw_smp_processor_id());
	if (ret) {
		pr_err("Failed to set warm boot address for cpu %d\n",
				raw_smp_processor_id());
		return ret;
	}

	flag = MSM_SCM_L2_ON & SCM_FLUSH_FLAG_MASK;
	scm_call_atomic1(SCM_SVC_BOOT, SCM_CMD_TERMINATE_PC, flag);

	return 0;
}

/**
 * msm_cpu_pm_enter_sleep(): Enter a low power mode on current cpu
 *
 * @mode - sleep mode to enter
 *
 * The code should be called with interrupts disabled and on the core on
 * which the low power mode is to be executed.
 *
 */
static int msm_cpu_pm_enter_sleep(enum msm_pm_sleep_mode mode)
{
	int ret;

	switch (mode) {
	case MSM_PM_SLEEP_MODE_SPC:
		msm_spm_set_low_power_mode(MSM_SPM_MODE_POWER_COLLAPSE);
		ret = cpu_suspend(0, msm_pm_collapse);
		break;
	default:
	case MSM_PM_SLEEP_MODE_WFI:
		msm_spm_set_low_power_mode(MSM_SPM_MODE_CLOCK_GATING);
		ret = cpu_do_idle();
		break;
	}

	local_irq_enable();

	return ret;
}

static struct platform_device qcom_cpuidle_device = {
	.name              = "qcom_cpuidle",
	.id                = -1,
	.dev.platform_data = msm_cpu_pm_enter_sleep,
};

static int __init msm_pm_device_init(void)
{
	platform_device_register(&qcom_cpuidle_device);

	return 0;
}
device_initcall(msm_pm_device_init);
