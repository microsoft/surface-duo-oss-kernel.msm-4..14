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

static void __init sac58r_init_machine(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
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
	.init_time	= sac58r_init_time,
	.init_machine   = sac58r_init_machine,
	.dt_compat	= sac58r_dt_compat,
MACHINE_END
