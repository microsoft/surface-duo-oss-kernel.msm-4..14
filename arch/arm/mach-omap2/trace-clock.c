/*
 * arch/arm/mach-omap2/trace-clock.c
 *
 * Trace clock for ARM OMAP3
 * Currently uniprocessor-only.
 *
 * Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>, February 2009
 */

#include <linux/module.h>
#include <linux/clocksource.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <plat/clock.h>
#include <plat/trace-clock.h>

/* depends on CONFIG_OMAP_32K_TIMER */
/* Need direct access to the clock from arch/arm/mach-omap2/timer-gp.c */
static struct clocksource *clock;

/* 32KHz counter count save upon PM sleep */
static u32 saved_32k_count;
static u64 saved_trace_clock;

static void clear_ccnt_ms(unsigned long data);

static DEFINE_TIMER(clear_ccnt_ms_timer, clear_ccnt_ms, 0, 0);

/* According to timer32k.c, this is a 32768Hz clock, not a 32000Hz clock. */
#define TIMER_32K_FREQ	32768
#define TIMER_32K_SHIFT	15

/*
 * Clear ccnt twice per 31-bit overflow, or 4 times per 32-bits period.
 */
static u32 clear_ccnt_interval;

static DEFINE_SPINLOCK(trace_clock_lock);
static int trace_clock_refcount;

static int print_info_done;

/*
 * Cycle counter management.
 */

static inline void write_pmnc(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c9, c12, 0" : : "r" (val));
}

static inline u32 read_pmnc(void)
{
	u32 val;
	__asm__ __volatile__ ("mrc p15, 0, %0, c9, c12, 0" : "=r" (val));
        return val;
}

static inline void write_ctens(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c9, c12, 1" : : "r" (val));
}

static inline u32 read_ctens(void)
{
	u32 val;
	__asm__ __volatile__ ("mrc p15, 0, %0, c9, c12, 1" : "=r" (val));
	return val;
}

static inline void write_intenc(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c9, c14, 2" : : "r" (val));
}

static inline u32 read_intenc(void)
{
	u32 val;
        __asm__ __volatile__ ("mrc p15, 0, %0, c9, c14, 2" : "=r" (val));
	return val;
}

static inline void write_useren(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c9, c14, 0" : : "r" (val));
}

static inline u32 read_useren(void)
{
	u32 val;
        __asm__ __volatile__ ("mrc p15, 0, %0, c9, c14, 0" : "=r" (val));
	return val;
}

/*
 * Must disable counter before writing to it.
 */
static inline void write_ccnt(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c9, c13, 0" : : "r" (val));
}

/*
 * Periodical timer handler, clears ccnt most significant bit each half-period
 * of 31-bit overflow. Makes sure the ccnt never overflows.
 */
static void clear_ccnt_ms(unsigned long data)
{
	unsigned int cycles;
	unsigned long flags;

	local_irq_save(flags);
	isb();	/* clear the pipeline so we can execute ASAP */
	write_ctens(read_ctens() & ~(1 << 31));	/* disable counter */
	cycles = read_ccnt();
	write_ccnt(cycles & ~(1 << 31));
	isb();
	write_ctens(read_ctens() |  (1 << 31));	/* enable counter */
	isb();
	local_irq_restore(flags);

	mod_timer(&clear_ccnt_ms_timer, jiffies + clear_ccnt_interval);
}

void _start_trace_clock(void)
{
	unsigned long flags;
	unsigned int count_32k, count_trace_clock;
	u32 regval;
	u64 ref_time, prev_time;

	/* Let userspace access performance counter registers */
	regval = read_useren();
	regval |=  (1 << 0);	/* User mode enable */
	write_useren(regval);

	regval = read_intenc();
	regval |=  (1 << 31);	/* CCNT overflow interrupt disable */
	write_intenc(regval);

	regval = read_pmnc();
	regval |=  (1 << 0);	/* Enable all counters */
	regval &= ~(1 << 3);	/* count every cycles */
	regval &= ~(1 << 5);	/* Enable even in non-invasive debug prohib. */
	write_pmnc(regval);

	/*
	 * Set the timer's value MSBs to the same as current 32K timer.
	 */
	ref_time = saved_trace_clock;
	local_irq_save(flags);
	count_32k = clock->read(clock);
	prev_time = trace_clock_read64();
	/*
	 * Delta done on 32-bits, then casted to u64. Must guarantee
	 * that we are called often enough so the difference does not
	 * overflow 32 bits anyway.
	 */
	ref_time += (u64)(count_32k - saved_32k_count)
			* (cpu_hz >> TIMER_32K_SHIFT);
	/* Make sure we never _ever_ decrement the clock value */
	if (ref_time < prev_time)
		ref_time = prev_time;
	write_ctens(read_ctens() & ~(1 << 31));	/* disable counter */
	write_ccnt((u32)ref_time & ~(1 << 31));
	write_ctens(read_ctens() |  (1 << 31));	/* enable counter */
	count_trace_clock = trace_clock_read32();
	_trace_clock_write_synthetic_tsc(ref_time);
	local_irq_restore(flags);

	get_synthetic_tsc();

	/* mod_timer generates a trace event. Must run after time-base update */
	mod_timer(&clear_ccnt_ms_timer, jiffies + clear_ccnt_interval);

	if (unlikely(!print_info_done || saved_trace_clock > ref_time)) {
		printk(KERN_INFO "Trace clock using cycle counter at %llu HZ\n"
			"32k clk value 0x%08X, cycle counter value 0x%08X\n"
			"saved 32k clk value 0x%08X, "
			"saved cycle counter value 0x%016llX\n"
			"synthetic value (write, read) 0x%016llX, 0x%016llX\n",
			cpu_hz,
			count_32k, count_trace_clock,
			saved_32k_count, saved_trace_clock,
			ref_time, trace_clock_read64());
		printk(KERN_INFO "Reference clock used : %s\n", clock->name);
		print_info_done = 1;
	}
}

void _stop_trace_clock(void)
{
	saved_32k_count = clock->read(clock);
	saved_trace_clock = trace_clock_read64();
	del_timer_sync(&clear_ccnt_ms_timer);
	put_synthetic_tsc();
}

void start_trace_clock(void)
{
	spin_lock(&trace_clock_lock);
	if (!trace_clock_refcount)
		goto end;
	_start_trace_clock();
end:
	spin_unlock(&trace_clock_lock);
}

void stop_trace_clock(void)
{
	spin_lock(&trace_clock_lock);
	if (!trace_clock_refcount)
		goto end;
	_stop_trace_clock();
end:
	spin_unlock(&trace_clock_lock);
}

void get_trace_clock(void)
{
	spin_lock(&trace_clock_lock);
	if (trace_clock_refcount++)
		goto end;
	_start_trace_clock();
end:
	spin_unlock(&trace_clock_lock);
}
EXPORT_SYMBOL_GPL(get_trace_clock);

void put_trace_clock(void)
{
	spin_lock(&trace_clock_lock);
	WARN_ON(trace_clock_refcount <= 0);
	if (trace_clock_refcount != 1)
		goto end;
	_stop_trace_clock();
end:
	trace_clock_refcount--;
	spin_unlock(&trace_clock_lock);
}
EXPORT_SYMBOL_GPL(put_trace_clock);

static __init int init_trace_clock(void)
{
	u64 rem;

	clock = get_clocksource_32k();
	clear_ccnt_interval = __iter_div_u64_rem(HZ * (1ULL << 30),
				cpu_hz, &rem);
	printk(KERN_INFO "LTTng will clear ccnt top bit every %u jiffies.\n",
		clear_ccnt_interval);
	return 0;
}
__initcall(init_trace_clock);
