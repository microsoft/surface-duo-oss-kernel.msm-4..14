/* linux/arch/arm/mach-exynos4/cpuidle.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/err.h>
#include <asm/proc-fns.h>
#include <asm/smp_scu.h>
#include <asm/suspend.h>
#include <asm/unified.h>
#include <mach/regs-pmu.h>
#include <mach/pmu.h>

#include <plat/exynos4.h>
#include <plat/cpu.h>

#define REG_DIRECTGO_ADDR	(exynos4_subrev() == 0 ?\
				(S5P_VA_SYSRAM + 0x24) : S5P_INFORM7)
#define REG_DIRECTGO_FLAG	(exynos4_subrev() == 0 ?\
				(S5P_VA_SYSRAM + 0x20) : S5P_INFORM6)

static int exynos4_enter_idle(struct cpuidle_device *dev,
			      struct cpuidle_state *state);

static int exynos4_enter_lowpower(struct cpuidle_device *dev,
				  struct cpuidle_state *state);

static struct cpuidle_state exynos4_cpuidle_set[] = {
	[0] = {
		.enter			= exynos4_enter_idle,
		.exit_latency		= 1,
		.target_residency	= 100000,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "C0",
		.desc			= "ARM clock gating(WFI)",
	},
	[1] = {
		.enter			= exynos4_enter_lowpower,
		.exit_latency		= 300,
		.target_residency	= 100000,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "C1",
		.desc			= "ARM power down",
	},
};

static DEFINE_PER_CPU(struct cpuidle_device, exynos4_cpuidle_device);

static struct cpuidle_driver exynos4_idle_driver = {
	.name		= "exynos4_idle",
	.owner		= THIS_MODULE,
};

/* Ext-GIC nIRQ/nFIQ is the only wakeup source in AFTR */
static void exynos4_set_wakeupmask(void)
{
	__raw_writel(0x0000ff3e, S5P_WAKEUP_MASK);
}

static unsigned int g_pwr_ctrl, g_diag_reg;

static void save_cpu_arch_register(void)
{
	/*read power control register*/
	asm("mrc p15, 0, %0, c15, c0, 0" : "=r"(g_pwr_ctrl) : : "cc");
	/*read diagnostic register*/
	asm("mrc p15, 0, %0, c15, c0, 1" : "=r"(g_diag_reg) : : "cc");
	return;
}

static void restore_cpu_arch_register(void)
{
	/*write power control register*/
	asm("mcr p15, 0, %0, c15, c0, 0" : : "r"(g_pwr_ctrl) : "cc");
	/*write diagnostic register*/
	asm("mcr p15, 0, %0, c15, c0, 1" : : "r"(g_diag_reg) : "cc");
	return;
}

static int idle_finisher(unsigned long flags)
{
	cpu_do_idle();
	return 1;
}

static int exynos4_enter_core0_aftr(struct cpuidle_device *dev,
				    struct cpuidle_state *state)
{
	struct timeval before, after;
	int idle_time;
	unsigned long tmp;

	local_irq_disable();
	do_gettimeofday(&before);

	exynos4_set_wakeupmask();

	/* Set value of power down register for aftr mode */
	exynos4_sys_powerdown_conf(SYS_AFTR);

	save_cpu_arch_register();

	/* Setting Central Sequence Register for power down mode */
	tmp = __raw_readl(S5P_CENTRAL_SEQ_CONFIGURATION);
	tmp &= ~S5P_CENTRAL_LOWPWR_CFG;
	__raw_writel(tmp, S5P_CENTRAL_SEQ_CONFIGURATION);

	cpu_pm_enter();
	cpu_cluster_pm_enter();

	cpu_suspend(0, idle_finisher);

	scu_enable(S5P_VA_SCU);

	cpu_cluster_pm_exit();
	cpu_pm_exit();

	restore_cpu_arch_register();

	/*
	 * If PMU failed while entering sleep mode, WFI will be
	 * ignored by PMU and then exiting cpu_do_idle().
	 * S5P_CENTRAL_LOWPWR_CFG bit will not be set automatically
	 * in this situation.
	 */
	tmp = __raw_readl(S5P_CENTRAL_SEQ_CONFIGURATION);
	if (!(tmp & S5P_CENTRAL_LOWPWR_CFG)) {
		tmp |= S5P_CENTRAL_LOWPWR_CFG;
		__raw_writel(tmp, S5P_CENTRAL_SEQ_CONFIGURATION);
	}

	/* Clear wakeup state register */
	__raw_writel(0x0, S5P_WAKEUP_STAT);

	do_gettimeofday(&after);

	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
		    (after.tv_usec - before.tv_usec);

	return idle_time;
}

static int exynos4_enter_idle(struct cpuidle_device *dev,
			      struct cpuidle_state *state)
{
	struct timeval before, after;
	int idle_time;

	local_irq_disable();
	do_gettimeofday(&before);

	cpu_do_idle();

	do_gettimeofday(&after);
	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
		    (after.tv_usec - before.tv_usec);

	return idle_time;
}

static int exynos4_enter_lowpower(struct cpuidle_device *dev,
				  struct cpuidle_state *state)
{
	struct cpuidle_state *new_state = state;

	/* This mode only can be entered when Core1 is offline */
	if (num_online_cpus() > 1) {
		BUG_ON(!dev->safe_state);
		new_state = dev->safe_state;
	}
	dev->last_state = new_state;

	if (new_state == &dev->states[0])
		return exynos4_enter_idle(dev, new_state);
	else
		return exynos4_enter_core0_aftr(dev, new_state);

	return exynos4_enter_idle(dev, new_state);
}

static int __init exynos4_init_cpuidle(void)
{
	int i, max_cpuidle_state, cpu_id;
	struct cpuidle_device *device;

	cpuidle_register_driver(&exynos4_idle_driver);

	for_each_cpu(cpu_id, cpu_online_mask) {
		device = &per_cpu(exynos4_cpuidle_device, cpu_id);
		device->cpu = cpu_id;

		if (cpu_id == 0)
			device->state_count = (sizeof(exynos4_cpuidle_set) /
					       sizeof(struct cpuidle_state));
		else
			device->state_count = 1;	/* Support IDLE only */

		max_cpuidle_state = device->state_count;

		for (i = 0; i < max_cpuidle_state; i++) {
			memcpy(&device->states[i], &exynos4_cpuidle_set[i],
					sizeof(struct cpuidle_state));
		}

		device->safe_state = &device->states[0];

		if (cpuidle_register_device(device)) {
			printk(KERN_ERR "CPUidle register device failed\n,");
			return -EIO;
		}
	}

	__raw_writel(BSYM(virt_to_phys(s3c_cpu_resume)),
						 REG_DIRECTGO_ADDR);
	__raw_writel(0xfcba0d10, REG_DIRECTGO_FLAG);

	return 0;
}
device_initcall(exynos4_init_cpuidle);
