/*
 * Copyright (C) 2005,2008 Mathieu Desnoyers
 *
 * Trace clock MIPS Octeon definitions.
 */

#ifndef _ASM_MIPS_OCTEON_TRACE_CLOCK_H
#define _ASM_MIPS_OCTEON_TRACE_CLOCK_H

#include <asm/octeon/octeon.h>

#define TC_HW_BITS			64

static inline u32 trace_clock_read32(void)
{
	return (u32)read_c0_cvmcount(); /* only need the 32 LSB */
}

static inline u64 trace_clock_read64(void)
{
	return read_c0_cvmcount();
}

static inline u64 trace_clock_frequency(void)
{
	return octeon_get_clock_rate();
}

static inline u32 trace_clock_freq_scale(void)
{
	return 1;
}

static inline void get_trace_clock(void)
{
	return;
}

static inline void put_trace_clock(void)
{
	return;
}
#endif /* _ASM_MIPS_OCTEON_TRACE_CLOCK_H */
