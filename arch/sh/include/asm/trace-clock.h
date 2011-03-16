/*
 * Copyright (C) 2007,2008 Giuseppe Cavallaro <peppe.cavallaro@st.com>
 *                         Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 *
 * Trace clock definitions for SuperH.
 */

#ifndef _ASM_SH_TRACE_CLOCK_H
#define _ASM_SH_TRACE_CLOCK_H

#include <linux/timer.h>
#include <asm/clock.h>

extern u64 trace_clock_read_synthetic_tsc(void);

static inline u32 trace_clock_get_read32(void)
{
	return get_cycles();
}

static inline u64 trace_clock_get_read64(void)
{
	return trace_clock_read_synthetic_tsc();
}

static inline u64 trace_clock_frequency(void)
{
	u64 rate;
	struct clk *tmu1_clk;

	tmu1_clk = clk_get(NULL, "tmu1_clk");
	rate = clk_get_rate(tmu1_clk);

	return rate;
}

static inline u32 trace_clock_freq_scale(void)
{
	return 1;
}

extern void get_synthetic_tsc(void);
extern void put_synthetic_tsc(void);

static inline void get_trace_clock(void)
{
	get_synthetic_tsc();
}

static inline void put_trace_clock(void)
{
	put_synthetic_tsc();
}

static inline void set_trace_clock_is_sync(int state)
{
}
#endif /* _ASM_SH_TRACE_CLOCK_H */
