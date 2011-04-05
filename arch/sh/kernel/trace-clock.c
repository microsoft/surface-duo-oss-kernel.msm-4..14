/*
 * arch/sh/kernel/trace-clock.c
 *
 * Trace clock for SuperH.
 *
 * Copyright (C) 2010  STMicroelectronics Ltd
 *
 * Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
 *
 * Note: currently only tested and supported on SH4 CPU
 * (TODO: tests on other SuperH architectures).
 */

#include <linux/module.h>
#include <linux/clocksource.h>
#include <asm/clock.h>

static struct clocksource *clksrc;

/* In case of the TMU, for SH4 architectures, it returns
 * the value of timer counter register (TCNT). */
u32 sh_read_timer_count(void)
{
	u32 value = 0;

	if (likely(clksrc))
		value = (u32) clksrc->read(clksrc);

	return value;
}

/* Get the clock rate for the timer (e.g. TMU for SH4) */
u64 sh_get_clock_frequency(void)
{
	u64 rate = 0;
	struct clk *clk;

	clk = clk_get(NULL, "module_clk");
	if (likely(clk))
		rate = clk_get_rate(clk) / 4;

	return rate;
}

/* Get the clock source needed to read the timer counter.
 * For example a TMU channel for SH4 architectures. */
static __init int init_sh_clocksource(void)
{
	clksrc = clocksource_get_next();
	if (unlikely(!clksrc))
		pr_err("%s: no clocksource found\n", __func__);

	return 0;
}
early_initcall(init_sh_clocksource);
