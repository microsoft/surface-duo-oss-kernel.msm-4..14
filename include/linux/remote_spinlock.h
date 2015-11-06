/* Copyright (c) 2008-2009, 2011, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __LINUX_REMOTE_SPINLOCK_H
#define __LINUX_REMOTE_SPINLOCK_H

#include <linux/io.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>

//#include <asm/remote_spinlock.h>


/* Remote spinlock definitions. */

struct dek_spinlock {
	volatile uint8_t self_lock;
	volatile uint8_t other_lock;
	volatile uint8_t next_yield;
	uint8_t pad;
};

typedef union {
	volatile uint32_t lock;
	struct dek_spinlock dek;
} raw_remote_spinlock_t;

typedef raw_remote_spinlock_t *_remote_spinlock_t;

#define remote_spinlock_id_t const char *
#define SMEM_SPINLOCK_PID_APPS 1

#define DEK_LOCK_REQUEST		1
#define DEK_LOCK_YIELD			(!DEK_LOCK_REQUEST)
#define DEK_YIELD_TURN_SELF		0
static inline void __raw_remote_dek_spin_lock(raw_remote_spinlock_t *lock)
{
	lock->dek.self_lock = DEK_LOCK_REQUEST;

	while (lock->dek.other_lock) {

		if (lock->dek.next_yield == DEK_YIELD_TURN_SELF)
			lock->dek.self_lock = DEK_LOCK_YIELD;

		while (lock->dek.other_lock)
			;

		lock->dek.self_lock = DEK_LOCK_REQUEST;
	}
	lock->dek.next_yield = DEK_YIELD_TURN_SELF;

	smp_mb();
}

static inline int __raw_remote_dek_spin_trylock(raw_remote_spinlock_t *lock)
{
	lock->dek.self_lock = DEK_LOCK_REQUEST;

	if (lock->dek.other_lock) {
		lock->dek.self_lock = DEK_LOCK_YIELD;
		return 0;
	}

	lock->dek.next_yield = DEK_YIELD_TURN_SELF;

	smp_mb();
	return 1;
}

static inline void __raw_remote_dek_spin_unlock(raw_remote_spinlock_t *lock)
{
	smp_mb();

	lock->dek.self_lock = DEK_LOCK_YIELD;
}

static inline int __raw_remote_dek_spin_release(raw_remote_spinlock_t *lock,
		uint32_t pid)
{
	return -EINVAL;
}

static inline void __raw_remote_sfpb_spin_lock(raw_remote_spinlock_t *lock)
{
	do {
		writel_relaxed(SMEM_SPINLOCK_PID_APPS, lock);
		smp_mb();
	} while (readl_relaxed(lock) != SMEM_SPINLOCK_PID_APPS);
}

static inline int __raw_remote_sfpb_spin_trylock(raw_remote_spinlock_t *lock)
{
	return 1;
}

static inline void __raw_remote_sfpb_spin_unlock(raw_remote_spinlock_t *lock)
{
	writel_relaxed(0, lock);
	smp_mb();
}

/**
 * Release spinlock if it is owned by @pid.
 *
 * This is only to be used for situations where the processor owning
 * the spinlock has crashed and the spinlock must be released.
 *
 * @lock - lock structure
 * @pid - processor ID of processor to release
 */
static inline int __raw_remote_gen_spin_release(raw_remote_spinlock_t *lock,
		uint32_t pid)
{
	int ret = 1;

	if (readl_relaxed(&lock->lock) == pid) {
		writel_relaxed(0, &lock->lock);
		wmb();
		ret = 0;
	}
	return ret;
}
#define CONFIG_MSM_REMOTE_SPINLOCK_SFPB
#if defined(CONFIG_QCOM_SMD) || defined(CONFIG_MSM_REMOTE_SPINLOCK_SFPB)
int _remote_spin_lock_init(remote_spinlock_id_t, _remote_spinlock_t *lock);
void _remote_spin_release_all(uint32_t pid);
#else
static inline
int _remote_spin_lock_init(remote_spinlock_id_t id, _remote_spinlock_t *lock)
{
	return -EINVAL;
}
static inline void _remote_spin_release_all(uint32_t pid) {}
#endif

/* Use SFPB Hardware Mutex Registers */
#define _remote_spin_lock(lock)		__raw_remote_sfpb_spin_lock(*lock)
#define _remote_spin_unlock(lock)	__raw_remote_sfpb_spin_unlock(*lock)
#define _remote_spin_trylock(lock)	__raw_remote_sfpb_spin_trylock(*lock)
#define _remote_spin_release(lock, pid)	__raw_remote_gen_spin_release(*lock,\
		pid)

/* Remote mutex definitions. */

typedef struct {
	_remote_spinlock_t	r_spinlock;
	uint32_t		delay_us;
} _remote_mutex_t;

struct remote_mutex_id {
	remote_spinlock_id_t	r_spinlock_id;
	uint32_t		delay_us;
};

#ifdef CONFIG_QCOM_SMD
int _remote_mutex_init(struct remote_mutex_id *id, _remote_mutex_t *lock);
void _remote_mutex_lock(_remote_mutex_t *lock);
void _remote_mutex_unlock(_remote_mutex_t *lock);
int _remote_mutex_trylock(_remote_mutex_t *lock);
#else
static inline
int _remote_mutex_init(struct remote_mutex_id *id, _remote_mutex_t *lock)
{
	return -EINVAL;
}
static inline void _remote_mutex_lock(_remote_mutex_t *lock) {}
static inline void _remote_mutex_unlock(_remote_mutex_t *lock) {}
static inline int _remote_mutex_trylock(_remote_mutex_t *lock)
{
	return 0;
}
#endif





/* Grabbing a local spin lock before going for a remote lock has several
 * advantages:
 * 1. Get calls to preempt enable/disable and IRQ save/restore for free.
 * 2. For UP kernel, there is no overhead.
 * 3. Reduces the possibility of executing the remote spin lock code. This is
 *    especially useful when the remote CPUs' mutual exclusion instructions
 *    don't work with the local CPUs' instructions. In such cases, one has to
 *    use software based mutex algorithms (e.g. Lamport's bakery algorithm)
 *    which could get expensive when the no. of contending CPUs is high.
 * 4. In the case of software based mutex algorithm the exection time will be
 *    smaller since the no. of contending CPUs is reduced by having just one
 *    contender for all the local CPUs.
 * 5. Get most of the spin lock debug features for free.
 * 6. The code will continue to work "gracefully" even when the remote spin
 *    lock code is stubbed out for debug purposes or when there is no remote
 *    CPU in some board/machine types.
 */
typedef struct {
	spinlock_t local;
	_remote_spinlock_t remote;
} remote_spinlock_t;

#define remote_spin_lock_init(lock, id) \
	({ \
		spin_lock_init(&((lock)->local)); \
		_remote_spin_lock_init(id, &((lock)->remote)); \
	})
#define remote_spin_lock(lock) \
	do { \
		spin_lock(&((lock)->local)); \
		_remote_spin_lock(&((lock)->remote)); \
	} while (0)
#define remote_spin_unlock(lock) \
	do { \
		_remote_spin_unlock(&((lock)->remote)); \
		spin_unlock(&((lock)->local)); \
	} while (0)
#define remote_spin_lock_irqsave(lock, flags) \
	do { \
		spin_lock_irqsave(&((lock)->local), flags); \
		_remote_spin_lock(&((lock)->remote)); \
	} while (0)
#define remote_spin_unlock_irqrestore(lock, flags) \
	do { \
		_remote_spin_unlock(&((lock)->remote)); \
		spin_unlock_irqrestore(&((lock)->local), flags); \
	} while (0)
#define remote_spin_trylock(lock) \
	({ \
		spin_trylock(&((lock)->local)) \
		? _remote_spin_trylock(&((lock)->remote)) \
			? 1 \
			: ({ spin_unlock(&((lock)->local)); 0; }) \
		: 0; \
	})
#define remote_spin_trylock_irqsave(lock, flags) \
	({ \
		spin_trylock_irqsave(&((lock)->local), flags) \
		? _remote_spin_trylock(&((lock)->remote)) \
			? 1 \
			: ({ spin_unlock_irqrestore(&((lock)->local), flags); \
				0; }) \
		: 0; \
	})

#define remote_spin_release(lock, pid) \
	_remote_spin_release(&((lock)->remote), pid)

#define remote_spin_release_all(pid) \
	_remote_spin_release_all(pid)

typedef struct {
	struct mutex local;
	_remote_mutex_t remote;
} remote_mutex_t;

#define remote_mutex_init(lock, id) \
	({ \
		mutex_init(&((lock)->local)); \
		_remote_mutex_init(id, &((lock)->remote)); \
	})
#define remote_mutex_lock(lock) \
	do { \
		mutex_lock(&((lock)->local)); \
		_remote_mutex_lock(&((lock)->remote)); \
	} while (0)
#define remote_mutex_trylock(lock) \
	({ \
		mutex_trylock(&((lock)->local)) \
		? _remote_mutex_trylock(&((lock)->remote)) \
			? 1 \
			: ({mutex_unlock(&((lock)->local)); 0; }) \
		: 0; \
	})
#define remote_mutex_unlock(lock) \
	do { \
		_remote_mutex_unlock(&((lock)->remote)); \
		mutex_unlock(&((lock)->local)); \
	} while (0)

#endif
