/*
 *  arch/arm/include/asm/okl4-microvisor/spinlock_types.h
 *
 * Copyright (c) 2012-2014 General Dynamics
 * Copyright (c) 2014 Open Kernel Labs, Inc.
 * Copyright (c) 2016-2017 Cog Systems Pty Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_OKL4_SPINLOCK_TYPES_H
#define __ASM_OKL4_SPINLOCK_TYPES_H

#define TICKET_SHIFT	16

typedef struct {
	union {
		u32 slock;
		struct __raw_tickets {
#ifdef __AARCH64EB__
			u16 next;
			u16 owner;
#else
			u16 owner;
			u16 next;
#endif
		} tickets;
	};
} __aligned(4) arch_spinlock_t;

#define __ARCH_SPIN_LOCK_UNLOCKED	{ { 0 } }

#define W_LOCKED 0x8000

typedef struct {
	/* val: 0 = free, +ve = R locked, 0x800N = W locked by CPU 'N' */
	/* Waiters is a count of waiting CPUs */
	union {
		u32 rwlock;
		struct __raw_rw_lock {
#ifdef __AARCH64EB__
			u16 waiters;
			s16 val;
#else
			s16 val;
			u16 waiters;
#endif
		} lock;
	};
} __aligned(4) arch_rwlock_t;

#define __ARCH_RW_LOCK_UNLOCKED		{ { 0 } }

#endif /* __ASM_OKL4_SPINLOCK_TYPES_H */
