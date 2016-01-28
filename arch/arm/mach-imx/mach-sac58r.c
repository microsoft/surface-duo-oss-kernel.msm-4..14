/*
 * Copyright 2013-2014 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/of_platform.h>
#include <linux/clocksource.h>
#include <linux/irqchip.h>
#include <linux/clk-provider.h>
#include <asm/mach/arch.h>

#include "common.h"
#include "hardware.h"

#include <asm/siginfo.h>
#include <asm/signal.h>

static bool first_fault = true;

/* This abort handler is here because kernel catches an imprecise data abort
	right when we get in userspace. We could not find the exact reason/cause
	why this is happening. Using this workaround helps, but we should definitely
	understand where this comes from
	*/
static int sac58r_abort_handler(unsigned long addr, unsigned int fsr,
				 struct pt_regs *regs)
{
	if (fsr == 0x1c06 && first_fault) {
		first_fault = false;

		/*
		 * Still could not catch the reason why this is happening
		 * Using this abort handler allows working around the issue.
		 */
		pr_warn("External imprecise Data abort (0x%03x) at 0x%08lx ignored.\n",
		fsr, addr);

		/* Returning non-zero causes fault display and panic */
		return 0;
	}

	/* Others should cause a fault */
	return 1;
}

static void __init sac58r_init_early(void)
{
	/* Install our hook */
	hook_fault_code(16 + 6, sac58r_abort_handler, SIGBUS, BUS_OBJERR,
			"imprecise external abort");
}

static void __init sac58r_init_irq(void)
{
	imx_src_init();
	irqchip_init();
}

static void __init sac58r_init_machine(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	mxc_set_cpu_type(MXC_CPU_SAC58R);
}

static void __init sac58r_init_time(void)
{
	of_clk_init(NULL);
	clocksource_of_init();
}

static const char *sac58r_dt_compat[] __initdata = {
	"fsl,sac58r",
	NULL,
};

DT_MACHINE_START(rayleigh, "Freescale Radio sac58r")
	.smp		= smp_ops(sac58r_smp_ops),
	.init_early = sac58r_init_early,
	.init_irq	= sac58r_init_irq,
	.init_time	= sac58r_init_time,
	.init_machine   = sac58r_init_machine,
	.dt_compat	= sac58r_dt_compat,
	.restart	= mxc_restart,
MACHINE_END
