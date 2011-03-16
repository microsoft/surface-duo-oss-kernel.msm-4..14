/*
 * arch/mips/kernel/trace-clock.c
 *
 * Trace clock for mips.
 *
 * Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>, October 2008
 */

#include <linux/module.h>
#include <linux/trace-clock.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/spinlock.h>

static u64 trace_clock_last_tsc;
static DEFINE_PER_CPU(struct timer_list, update_timer);
static DEFINE_SPINLOCK(async_tsc_lock);
static int async_tsc_refcount;	/* Number of readers */
static int async_tsc_enabled;	/* Async TSC enabled on all online CPUs */

#if (BITS_PER_LONG == 64)
static inline u64 trace_clock_cmpxchg64(u64 *ptr, u64 old, u64 new)
{
	return cmpxchg64(ptr, old, new);
}
#else
/*
 * Emulate an atomic 64-bits update with a spinlock.
 * Note : preempt_disable or irq save must be explicit with raw_spinlock_t.
 * Given we use a spinlock for this time base, we should never be called from
 * NMI context.
 */
static raw_spinlock_t trace_clock_lock =
	(raw_spinlock_t)__RAW_SPIN_LOCK_UNLOCKED;

/*
 * Must be called under irqoff+spinlock on MIPS32.
 */
static inline u64 trace_clock_cmpxchg64(u64 *ptr, u64 old, u64 new)
{
	u64 val;

	val = *ptr;
	if (likely(val == old))
		*ptr = new;
	return val;
}
#endif

/*
 * Must be called under irqoff+spinlock on MIPS32.
 */
static cycles_t read_last_tsc(void)
{
	return trace_clock_last_tsc;
}

/*
 * Support for architectures with non-sync TSCs.
 * When the local TSC is discovered to lag behind the highest TSC counter, we
 * increment the TSC count of an amount that should be, ideally, lower than the
 * execution time of this routine, in cycles : this is the granularity we look
 * for : we must be able to order the events.
 *
 * MIPS32 does not have atomic 64-bit updates. Emulate it with irqoff+spinlock.
 */

notrace u64 trace_clock_async_tsc_read(void)
{
	u64 new_tsc, last_tsc;
#if (BITS_PER_LONG == 32)
	unsigned long flags;

	local_irq_save(flags);
	__raw_spin_lock(&trace_clock_lock);
#endif

	WARN_ON(!async_tsc_refcount || !async_tsc_enabled);
	new_tsc = trace_clock_read_synthetic_tsc();
	barrier();
	last_tsc = read_last_tsc();
	do {
		if (new_tsc < last_tsc)
			new_tsc = last_tsc + TRACE_CLOCK_MIN_PROBE_DURATION;
		/*
		 * If cmpxchg fails with a value higher than the new_tsc, don't
		 * retry : the value has been incremented and the events
		 * happened almost at the same time.
		 * We must retry if cmpxchg fails with a lower value :
		 * it means that we are the CPU with highest frequency and
		 * therefore MUST update the value.
		 */
		last_tsc = trace_clock_cmpxchg64(&trace_clock_last_tsc,
						 last_tsc, new_tsc);
	} while (unlikely(last_tsc < new_tsc));
#if (BITS_PER_LONG == 32)
	__raw_spin_unlock(&trace_clock_lock);
	local_irq_restore(flags);
#endif
	return new_tsc;
}
EXPORT_SYMBOL_GPL(trace_clock_async_tsc_read);

static void update_timer_ipi(void *info)
{
	(void)trace_clock_async_tsc_read();
}

/*
 * update_timer_fct : - Timer function to resync the clocks
 * @data: unused
 *
 * Fires every jiffy.
 */
static void update_timer_fct(unsigned long data)
{
	(void)trace_clock_async_tsc_read();

	per_cpu(update_timer, smp_processor_id()).expires = jiffies + 1;
	add_timer_on(&per_cpu(update_timer, smp_processor_id()),
		     smp_processor_id());
}

static void enable_trace_clock(int cpu)
{
	init_timer(&per_cpu(update_timer, cpu));
	per_cpu(update_timer, cpu).function = update_timer_fct;
	per_cpu(update_timer, cpu).expires = jiffies + 1;
	smp_call_function_single(cpu, update_timer_ipi, NULL, 1);
	add_timer_on(&per_cpu(update_timer, cpu), cpu);
}

static void disable_trace_clock(int cpu)
{
	del_timer_sync(&per_cpu(update_timer, cpu));
}

void get_trace_clock(void)
{
	int cpu;

	spin_lock(&async_tsc_lock);
	if (async_tsc_refcount++ || tsc_is_sync()) {
		get_synthetic_tsc();
	} else {
		async_tsc_enabled = 1;
		get_synthetic_tsc();
		for_each_online_cpu(cpu)
			enable_trace_clock(cpu);
	}
	spin_unlock(&async_tsc_lock);
}
EXPORT_SYMBOL_GPL(get_trace_clock);

void put_trace_clock(void)
{
	int cpu;

	spin_lock(&async_tsc_lock);
	WARN_ON(async_tsc_refcount <= 0);
	if (async_tsc_refcount != 1 || !async_tsc_enabled) {
		put_synthetic_tsc();
	} else {
		for_each_online_cpu(cpu)
			disable_trace_clock(cpu);
		async_tsc_enabled = 0;
	}
	async_tsc_refcount--;
	spin_unlock(&async_tsc_lock);
}
EXPORT_SYMBOL_GPL(put_trace_clock);
