/*
 *  arch/arm/include/asm/okl4-microvisor/spinlock.h
 *
 * Copyright (c) 2012-2014 General Dynamics
 * Copyright (c) 2014 Open Kernel Labs, Inc.
 * Copyright (c) 2016-2017 Cog Systems Pty Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_OKL4_SPINLOCK_H
#define __ASM_OKL4_SPINLOCK_H

#include <linux/smp.h>
#include <linux/cpumask.h>
#include <microvisor/microvisor.h>

#define __NUM_SPIN_TRIES 100

/*
 * From Kernel v3.19 onwards ACCESS_ONCE works only on scalar types. Two new
 * macros (READ_ONCE and WRITE_ONCE) were added for non-scalar types.
 */
#if !defined(READ_ONCE)
#define READ_ONCE ACCESS_ONCE
#endif

static inline void dsb_sev(int sev)
{
	if (sev)
		_okl4_sys_vcpu_sync_sev();
}

/*
 * ARMv6 ticket-based spin-locking.
 *
 * A memory barrier is required after we get a lock, and before we
 * release it, because V6 CPUs are assumed to have weakly ordered
 * memory.
 */

#define arch_spin_unlock_wait(lock) \
	do { while (arch_spin_is_locked(lock)) cpu_relax(); } while (0)

#define arch_spin_lock_flags(lock, flags) arch_spin_lock(lock)

static inline void arch_spin_lock(arch_spinlock_t *lock)
{
	unsigned long counter;
	u32 newval, tmp;
	arch_spinlock_t lockval;

	__asm__ __volatile__(
"	prfm	pstl1strm, [%3]\n"
"1:	ldaxr	%w0, [%3]\n"
"	add	%w1, %w0, %w4\n"
"	stxr	%w2, %w1, [%3]\n"
"	cbnz	%w2, 1b\n"
	: "=&r" (lockval), "=&r" (newval), "=&r" (tmp)
	: "r" (&lock->slock), "I" (1 << TICKET_SHIFT)
	: "cc");

	counter = __NUM_SPIN_TRIES;
	while (lockval.tickets.next != lockval.tickets.owner) {
		if (--counter == 0) {
			_okl4_sys_vcpu_sync_wfe(OKL4_KCAP_INVALID);
			counter = __NUM_SPIN_TRIES;
		}
		lockval.tickets.owner = ACCESS_ONCE(lock->tickets.owner);
	}

	smp_mb();
}

static inline int arch_spin_trylock(arch_spinlock_t *lock)
{
	u32 tmp;
	u32 slock;

	__asm__ __volatile__(
"	prfm	pstl1strm, [%2]\n"
"	ldaxr	%w0, [%2]\n"
"	eor	%w1, %w0, %w0, ror #16\n"
"	cbnz	%w1, 1f\n"
"	add	%w0, %w0, %3\n"
"	stxr	%w1, %w0, [%2]\n"
"1:\n"
	: "=&r" (slock), "=&r" (tmp)
	: "r" (&lock->slock), "I" (1 << TICKET_SHIFT)
	: "cc");

	if (tmp == 0) {
		smp_mb();
		return 1;
	} else {
		return 0;
	}
}

static inline void arch_spin_unlock(arch_spinlock_t *lock)
{
	struct __raw_tickets tickets;

	smp_mb();
	lock->tickets.owner++;
	tickets = READ_ONCE(lock->tickets);
	dsb_sev(tickets.next != tickets.owner);
}

static inline int arch_spin_is_locked(arch_spinlock_t *lock)
{
	struct __raw_tickets tickets = READ_ONCE(lock->tickets);
	return tickets.owner != tickets.next;
}

static inline int arch_spin_value_unlocked(arch_spinlock_t lock)
{
	return !arch_spin_is_locked(&lock);
}

static inline int arch_spin_is_contended(arch_spinlock_t *lock)
{
	struct __raw_tickets tickets = READ_ONCE(lock->tickets);
	return (tickets.next - tickets.owner) > 1;
}
#define arch_spin_is_contended	arch_spin_is_contended


/*
 * RWLOCKS
 *
 * Write locks are simpler - we set bit 15 when locked.  When unlocking, we can
 * write zero since the lock is exclusively held.
 */

extern u32 vcpu_caps[NR_CPUS];

static inline void queue_waiters(arch_rwlock_t *rw)
{
	unsigned long tmp1, tmp2;

	__asm__ __volatile__(
"1:	ldaxrh	%w0, [%3]\n"
"	add	%w0, %w0, #1\n"
"	stxrh	%w1, %w0, [%3]\n"
"	cbnz	%w1, 1b\n"
	: "=&r" (tmp1), "=&r" (tmp2), "+Qo" (rw->lock.waiters)
	: "r" (&rw->lock.waiters)
	: "cc");
}

static inline void dequeue_waiters(arch_rwlock_t *rw)
{
	unsigned long tmp1, tmp2;

	__asm__ __volatile__(
"1:	ldaxrh	%w0, [%3]\n"
"	sub	%w0, %w0, #1\n"
"	stxrh	%w1, %w0, [%3]\n"
"	cbnz	%w1, 1b\n"
	: "=&r" (tmp1), "=&r" (tmp2), "+Qo" (rw->lock.waiters)
	: "r" (&rw->lock.waiters)
	: "cc");
}

static inline void
_arch_lock_wait(unsigned int holding_cpu)
{
	okl4_kcap_t vcpu_cap = holding_cpu;
	if (holding_cpu < nr_cpu_ids) {
		vcpu_cap = vcpu_caps[holding_cpu];
	}
	_okl4_sys_vcpu_sync_wfe(vcpu_cap);
}

/* Return zero if lock acquired */
static inline unsigned long _arch_write_lock_loop(arch_rwlock_t *rw)
{
	u32 lock, tmp;
	unsigned long counter = __NUM_SPIN_TRIES;
	unsigned long current_cpu = raw_smp_processor_id();

	__asm__ __volatile__(
"1:	ldaxrh	%w0, [%4]\n"
"	mov	%w1, %w0\n"
"	cbnz	%w0, 2f\n"
"	stxrh	%w1, %w5, [%4]\n"
"	cbz	%w1, 3f\n"
"2:	sub	%2, %2, #1\n"
"	cbnz	%2, 1b\n"
"3:\n"
	: "=&r" (lock), "=&r" (tmp), "+&r"(counter), "+Qo" (rw->rwlock)
	: "r" (&rw->rwlock), "r" (current_cpu | W_LOCKED)
	: "cc");

	return lock || (counter == 0);
}

static inline void arch_write_lock(arch_rwlock_t *rw)
{
	while (1) {
		unsigned long holding_cpu = _arch_write_lock_loop(rw);

		if (likely(holding_cpu == 0)) {
			smp_mb();
			break;
		} else {
			unsigned int holding_cpu;

			u16 holder = READ_ONCE(rw->lock.val);
			/* Very likely it's still locked */
			if (holder != 0) {
				queue_waiters(rw);
				if (holder & W_LOCKED) {
					/* donate time to holder */
					holding_cpu = holder & (W_LOCKED - 1);
				} else {
					/* read-locked, just wait */
					holding_cpu = OKL4_KCAP_INVALID;
				}
				_arch_lock_wait(holding_cpu);
				dequeue_waiters(rw);
			}
		}
	}
}


static inline int arch_write_trylock(arch_rwlock_t *rw)
{
	unsigned long current_cpu = raw_smp_processor_id();
	u32 tmp;

	__asm__ __volatile__(
"	ldaxrh	%w0, [%2]\n"
"	cbnz	%w0, 1f\n"
"	stxrh	%w0, %w3, [%2]\n"
"1:\n"
	: "=&r" (tmp), "+Qo" (rw->rwlock)
	: "r" (&rw->rwlock), "r" (current_cpu | W_LOCKED)
	: "cc");

	if (tmp == 0) {
		smp_mb();
		return 1;
	} else {
		return 0;
	}
}

static inline void arch_write_unlock(arch_rwlock_t *rw)
{
	u32 tmp;
	arch_rwlock_t l;
	smp_mb();

	__asm__ __volatile__(
"	strh	%w3, [%2]\n"
"	ldr	%w0, [%2]\n"
	: "=&r" (tmp), "+Qo" (rw->rwlock)
	: "r" (&rw->rwlock), "r" (0)
	: "cc");

	l.rwlock = tmp;

	dsb_sev(l.lock.waiters);
}

/* write_can_lock - would write_trylock() succeed? */
#define arch_write_can_lock(x)		(ACCESS_ONCE((x)->lock.val) == 0)

/*
 * Read locks are a bit more hairy:
 *  - Exclusively load the lock value.
 *  - Increment it.
 *  - Store new lock value if positive, and we still own this location.
 *    If the value is negative, we've already failed.
 *  - If we failed to store the value, we want a negative result.
 *  - If we failed, try again.
 * Unlocking is similarly hairy.  We may have multiple read locks
 * currently active.  However, we know we won't have any write
 * locks.
 */

/* Return positive if it acquired the lock */
static inline s32 _arch_read_lock_loop(arch_rwlock_t *rw)
{
	s32 tmp, tmp2;
	long counter = -__NUM_SPIN_TRIES;

	__asm__ __volatile__(
"1:	ldaxrh	%w0, [%4]\n"
"	sxth	%w0, %w0\n"
"	add	%w0, %w0, #1\n"
"	tbnz	%w0, #31, 2f\n"
"	stxrh	%w1, %w0, [%4]\n"
"	cbz	%w1, 3f\n"
"	neg	%w0, %w1\n"
"2:	sub	%2, %2, #1\n"
"	cbnz	%2, 1b\n"
"3:\n"
	: "=&r" (tmp), "=&r"(tmp2),  "+&r"(counter), "+Qo" (rw->rwlock)
	: "r" (&rw->rwlock)
	: "cc");

	return tmp;
}

static inline void arch_read_lock(arch_rwlock_t *rw)
{
	while (1) {
		s32 val = _arch_read_lock_loop(rw);
		if (likely(val >= 0)) {
			smp_mb();
			break;
		} else {
			unsigned int holding_cpu;

			u16 holder = READ_ONCE(rw->lock.val);
			/* Very likely it's still locked */
			if (holder & W_LOCKED) {
				queue_waiters(rw);
				holding_cpu = holder & (W_LOCKED - 1);
				_arch_lock_wait(holding_cpu);
				dequeue_waiters(rw);
			}
		}
	}
}

static inline void arch_read_unlock(arch_rwlock_t *rw)
{
	u32 tmp, tmp2;

	smp_mb();

	__asm__ __volatile__(
"1:	ldxr	%w0, [%3]\n"
"	sub	%w0, %w0, #1\n"
"	stlxr	%w1, %w0, [%3]\n"
"	cbnz	%w1, 1b\n"
	: "=&r" (tmp), "=&r" (tmp2), "+Qo" (rw->rwlock)
	: "r" (&rw->rwlock)
	: "cc");

	/* If the lock was released, wakeup the waiters */
	if ((tmp & 0xffff) == 0) {
		smp_mb();
		dsb_sev(tmp >> 16);
	}
}

static inline int arch_read_trylock(arch_rwlock_t *rw)
{
	unsigned long tmp, tmp2 = 1;

	__asm__ __volatile__(
"	ldaxrh	%w0, [%3]\n"
"	sxth	%w0, %w0\n"
"	add	%w0, %w0, #1\n"
"	tbnz	%w0, #31, 1f\n"
"	stxrh	%w1, %w0, [%3]\n"
"1:\n"
	: "=&r" (tmp), "+&r" (tmp2), "+Qo" (rw->rwlock)
	: "r" (&rw->rwlock)
	: "cc");

	smp_mb();
	return tmp2 == 0;
}

/* read_can_lock - would read_trylock() succeed? */
#define arch_read_can_lock(x)		(ACCESS_ONCE((x)->lock.val) >= 0)

#define arch_read_lock_flags(lock, flags) arch_read_lock(lock)
#define arch_write_lock_flags(lock, flags) arch_write_lock(lock)

#define arch_spin_relax(lock)	cpu_relax()
#define arch_read_relax(lock)	cpu_relax()
#define arch_write_relax(lock)	cpu_relax()

#endif /* __ASM_OKL4_SPINLOCK_H */
