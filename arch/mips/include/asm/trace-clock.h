/*
 * Copyright (C) 2005,2008 Mathieu Desnoyers
 *
 * Trace clock MIPS definitions.
 */

#ifndef _ASM_MIPS_TRACE_CLOCK_H
#define _ASM_MIPS_TRACE_CLOCK_H

#include <linux/timex.h>
#include <asm/processor.h>

#define TRACE_CLOCK_MIN_PROBE_DURATION 200

extern u64 trace_clock_read_synthetic_tsc(void);

/*
 * MIPS get_cycles only returns a 32 bits TSC (see timex.h). The assumption
 * there is that the reschedule is done every 8 seconds or so. Given that
 * tracing needs to detect delays longer than 8 seconds, we need a full 64-bits
 * TSC, whic is provided by trace-clock-32-to-64.
*/
extern u64 trace_clock_async_tsc_read(void);

static inline u32 trace_clock_read32(void)
{
	u32 cycles;

	if (likely(tsc_is_sync()))
		cycles = (u32)get_cycles(); /* only need the 32 LSB */
	else
		cycles = (u32)trace_clock_async_tsc_read();
	return cycles;
}

static inline u64 trace_clock_read64(void)
{
	u64 cycles;

	if (likely(tsc_is_sync()))
		cycles = trace_clock_read_synthetic_tsc();
	else
		cycles = trace_clock_async_tsc_read();
	return cycles;
}

static inline u64 trace_clock_frequency(void)
{
	return get_cycles_rate();
}

static inline u32 trace_clock_freq_scale(void)
{
	return 1;
}

extern void get_trace_clock(void);
extern void put_trace_clock(void);
extern void get_synthetic_tsc(void);
extern void put_synthetic_tsc(void);

static inline void set_trace_clock_is_sync(int state)
{
}
#endif /* _ASM_MIPS_TRACE_CLOCK_H */
