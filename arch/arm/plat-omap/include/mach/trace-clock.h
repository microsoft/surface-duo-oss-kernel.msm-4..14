/*
 * Copyright (C) 2009 Mathieu Desnoyers
 *
 * Trace clock ARM OMAP3 definitions.
 */

#ifndef _ASM_ARM_TRACE_CLOCK_OMAP3_H
#define _ASM_ARM_TRACE_CLOCK_OMAP3_H

#include <linux/clk.h>
#include <asm/system.h>
#include <mach/dmtimer.h>

/*
 * Number of hardware clock bits. The higher order bits are expected to be 0.
 * If the hardware clock source has more than 32 bits, the bits higher than the
 * 32nd will be truncated by a cast to a 32 bits unsigned. Range : 1 - 32.
 * (too few bits would be unrealistic though, since we depend on the timer to
 * detect the overflows).
 * OMAP3-specific : we clear bit 31 periodically so it never overflows. There is
 * a hardware bug with CP14 and CP15 being executed at the same time a ccnt overflow
 * occurs.
 *
 * Siarhei Siamashka <siarhei.siamashka@nokia.com> :
 * Performance monitoring unit breaks if somebody is accessing CP14/CP15
 * coprocessor register exactly at the same time as CCNT overflows (regardless
 * of the fact if generation of interrupts is enabled or not). A workaround
 * suggested by ARM was to never allow it to overflow and reset it
 * periodically.
 */
#define TC_HW_BITS			31

/* Expected maximum interrupt latency in ms : 15ms, *2 for security */
#define TC_EXPECTED_INTERRUPT_LATENCY	30

extern u64 trace_clock_read_synthetic_tsc(void);
extern void _trace_clock_write_synthetic_tsc(u64 value);
extern struct omap_dm_timer *trace_clock_timer;
extern unsigned long long cpu_hz;

/*
 * ARM OMAP3 timers only return 32-bits values. We ened to extend it to a
 * 64-bit value, which is provided by trace-clock-32-to-64.
 */
extern u64 trace_clock_async_tsc_read(void);
/*
 * Update done by the architecture upon wakeup.
 */
extern void _trace_clock_write_synthetic_tsc(u64 value);

static inline u32 read_ccnt(void)
{
	u32 val;
        __asm__ __volatile__ ("mrc p15, 0, %0, c9, c13, 0" : "=r" (val));
	return val & ~(1 << TC_HW_BITS);
}

static inline u32 trace_clock_read32(void)
{
	u32 val;

	isb();
	val = read_ccnt();
	isb();
	return val;
}

static inline u64 trace_clock_read64(void)
{
	return trace_clock_read_synthetic_tsc();
}

static inline u64 trace_clock_frequency(void)
{
	return cpu_hz;
}

static inline u32 trace_clock_freq_scale(void)
{
	return 1;
}

extern void get_trace_clock(void);
extern void put_trace_clock(void);
extern void get_synthetic_tsc(void);
extern void put_synthetic_tsc(void);

/* Used by the architecture upon wakeup from PM idle */
extern void start_trace_clock(void);
/* Used by the architecture when going to PM idle */
extern void stop_trace_clock(void);

static inline void set_trace_clock_is_sync(int state)
{
}
#endif /* _ASM_MIPS_TRACE_CLOCK_OMAP3_H */
