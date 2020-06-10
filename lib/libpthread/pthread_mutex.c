/*	$NetBSD: pthread_mutex.c,v 1.79 2020/06/03 22:10:24 ad Exp $	*/

/*-
 * Copyright (c) 2001, 2003, 2006, 2007, 2008, 2020 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Nathan J. Williams, by Jason R. Thorpe, and by Andrew Doran.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * To track threads waiting for mutexes to be released, we use lockless
 * lists built on atomic operations and memory barriers.
 *
 * A simple spinlock would be faster and make the code easier to
 * follow, but spinlocks are problematic in userspace.  If a thread is
 * preempted by the kernel while holding a spinlock, any other thread
 * attempting to acquire that spinlock will needlessly busy wait.
 *
 * There is no good way to know that the holding thread is no longer
 * running, nor to request a wake-up once it has begun running again. 
 * Of more concern, threads in the SCHED_FIFO class do not have a
 * limited time quantum and so could spin forever, preventing the
 * thread holding the spinlock from getting CPU time: it would never
 * be released.
 */

#include <sys/cdefs.h>
__RCSID("$NetBSD: pthread_mutex.c,v 1.79 2020/06/03 22:10:24 ad Exp $");

#include <sys/types.h>
#include <sys/lwpctl.h>
#include <sys/sched.h>
#include <sys/lock.h>

#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <stdio.h>

#include "pthread.h"
#include "pthread_int.h"
#include "reentrant.h"

#define	MUTEX_WAITERS_BIT		((uintptr_t)0x01)
#define	MUTEX_RECURSIVE_BIT		((uintptr_t)0x02)
#define	MUTEX_PROTECT_BIT		((uintptr_t)0x08)
#define	MUTEX_THREAD			((uintptr_t)~0x0f)

#define	MUTEX_HAS_WAITERS(x)		((uintptr_t)(x) & MUTEX_WAITERS_BIT)
#define	MUTEX_RECURSIVE(x)		((uintptr_t)(x) & MUTEX_RECURSIVE_BIT)
#define	MUTEX_PROTECT(x)		((uintptr_t)(x) & MUTEX_PROTECT_BIT)
#define	MUTEX_OWNER(x)			((uintptr_t)(x) & MUTEX_THREAD)

#define	MUTEX_GET_TYPE(x)		\
    ((int)(((uintptr_t)(x) & 0x000000ff) >> 0))
#define	MUTEX_SET_TYPE(x, t) 		\
    (x) = (void *)(((uintptr_t)(x) & ~0x000000ff) | ((t) << 0))
#define	MUTEX_GET_PROTOCOL(x)		\
    ((int)(((uintptr_t)(x) & 0x0000ff00) >> 8))
#define	MUTEX_SET_PROTOCOL(x, p)	\
    (x) = (void *)(((uintptr_t)(x) & ~0x0000ff00) | ((p) << 8))
#define	MUTEX_GET_CEILING(x)		\
    ((int)(((uintptr_t)(x) & 0x00ff0000) >> 16))
#define	MUTEX_SET_CEILING(x, c)	\
    (x) = (void *)(((uintptr_t)(x) & ~0x00ff0000) | ((c) << 16))

#if __GNUC_PREREQ__(3, 0)
#define	NOINLINE		__attribute ((noinline))
#else
#define	NOINLINE		/* nothing */
#endif

static void	pthread__mutex_wakeup(pthread_t, pthread_mutex_t *);
static int	pthread__mutex_lock_slow(pthread_mutex_t *,
    const struct timespec *);
static void	pthread__mutex_pause(void);

int		_pthread_mutex_held_np(pthread_mutex_t *);
pthread_t	_pthread_mutex_owner_np(pthread_mutex_t *);

__weak_alias(pthread_mutex_held_np,_pthread_mutex_held_np)
__weak_alias(pthread_mutex_owner_np,_pthread_mutex_owner_np)

__strong_alias(__libc_mutex_init,pthread_mutex_init)
__strong_alias(__libc_mutex_lock,pthread_mutex_lock)
__strong_alias(__libc_mutex_trylock,pthread_mutex_trylock)
__strong_alias(__libc_mutex_unlock,pthread_mutex_unlock)
__strong_alias(__libc_mutex_destroy,pthread_mutex_destroy)

__strong_alias(__libc_mutexattr_init,pthread_mutexattr_init)
__strong_alias(__libc_mutexattr_destroy,pthread_mutexattr_destroy)
__strong_alias(__libc_mutexattr_settype,pthread_mutexattr_settype)

int
pthread_mutex_init(pthread_mutex_t *ptm, const pthread_mutexattr_t *attr)
{
	uintptr_t type, proto, val, ceil;

#if 0
	/*
	 * Always initialize the mutex structure, maybe be used later
	 * and the cost should be minimal.
	 */
	if (__predict_false(__uselibcstub))
		return __libc_mutex_init_stub(ptm, attr);
#endif

	pthread__error(EINVAL, "Invalid mutes attribute",
	    attr == NULL || attr->ptma_magic == _PT_MUTEXATTR_MAGIC);

	if (attr == NULL) {
		type = PTHREAD_MUTEX_NORMAL;
		proto = PTHREAD_PRIO_NONE;
		ceil = 0;
	} else {
		val = (uintptr_t)attr->ptma_private;

		type = MUTEX_GET_TYPE(val);
		proto = MUTEX_GET_PROTOCOL(val);
		ceil = MUTEX_GET_CEILING(val);
	}
	switch (type) {
	case PTHREAD_MUTEX_ERRORCHECK:
		__cpu_simple_lock_set(&ptm->ptm_errorcheck);
		ptm->ptm_owner = NULL;
		break;
	case PTHREAD_MUTEX_RECURSIVE:
		__cpu_simple_lock_clear(&ptm->ptm_errorcheck);
		ptm->ptm_owner = (void *)MUTEX_RECURSIVE_BIT;
		break;
	default:
		__cpu_simple_lock_clear(&ptm->ptm_errorcheck);
		ptm->ptm_owner = NULL;
		break;
	}
	switch (proto) {
	case PTHREAD_PRIO_PROTECT:
		val = (uintptr_t)ptm->ptm_owner;
		val |= MUTEX_PROTECT_BIT;
		ptm->ptm_owner = (void *)val;
		break;

	}
	ptm->ptm_magic = _PT_MUTEX_MAGIC;
	ptm->ptm_waiters = NULL;
	ptm->ptm_recursed = 0;
	ptm->ptm_ceiling = (unsigned char)ceil;

	return 0;
}

int
pthread_mutex_destroy(pthread_mutex_t *ptm)
{

	if (__predict_false(__uselibcstub))
		return __libc_mutex_destroy_stub(ptm);

	pthread__error(EINVAL, "Invalid mutex",
	    ptm->ptm_magic == _PT_MUTEX_MAGIC);
	pthread__error(EBUSY, "Destroying locked mutex",
	    MUTEX_OWNER(ptm->ptm_owner) == 0);

	ptm->ptm_magic = _PT_MUTEX_DEAD;
	return 0;
}

int
pthread_mutex_lock(pthread_mutex_t *ptm)
{
	pthread_t self;
	void *val;

	if (__predict_false(__uselibcstub))
		return __libc_mutex_lock_stub(ptm);

	pthread__error(EINVAL, "Invalid mutex",
	    ptm->ptm_magic == _PT_MUTEX_MAGIC);

	self = pthread__self();
	val = atomic_cas_ptr(&ptm->ptm_owner, NULL, self);
	if (__predict_true(val == NULL)) {
#ifndef PTHREAD__ATOMIC_IS_MEMBAR
		membar_enter();
#endif
		return 0;
	}
	return pthread__mutex_lock_slow(ptm, NULL);
}

int
pthread_mutex_timedlock(pthread_mutex_t* ptm, const struct timespec *ts)
{
	pthread_t self;
	void *val;

	pthread__error(EINVAL, "Invalid mutex",
	    ptm->ptm_magic == _PT_MUTEX_MAGIC);

	self = pthread__self();
	val = atomic_cas_ptr(&ptm->ptm_owner, NULL, self);
	if (__predict_true(val == NULL)) {
#ifndef PTHREAD__ATOMIC_IS_MEMBAR
		membar_enter();
#endif
		return 0;
	}
	return pthread__mutex_lock_slow(ptm, ts);
}

/* We want function call overhead. */
NOINLINE static void
pthread__mutex_pause(void)
{

	pthread__smt_pause();
}

/*
 * Spin while the holder is running.  'lwpctl' gives us the true
 * status of the thread.
 */
NOINLINE static void *
pthread__mutex_spin(pthread_mutex_t *ptm, pthread_t owner)
{
	pthread_t thread;
	unsigned int count, i;

	for (count = 2;; owner = ptm->ptm_owner) {
		thread = (pthread_t)MUTEX_OWNER(owner);
		if (thread == NULL)
			break;
		if (thread->pt_lwpctl->lc_curcpu == LWPCTL_CPU_NONE)
			break;
		if (count < 128) 
			count += count;
		for (i = count; i != 0; i--)
			pthread__mutex_pause();
	}

	return owner;
}

NOINLINE static int
pthread__mutex_lock_slow(pthread_mutex_t *ptm, const struct timespec *ts)
{
	void *waiters, *newval, *owner, *next;
	pthread_t self;
	int serrno;
	int error;

	owner = ptm->ptm_owner;
	self = pthread__self();
	serrno = errno;

	pthread__assert(!self->pt_willpark);
	pthread__assert(!self->pt_mutexwait);

	/* Recursive or errorcheck? */
	if (MUTEX_OWNER(owner) == (uintptr_t)self) {
		if (MUTEX_RECURSIVE(owner)) {
			if (ptm->ptm_recursed == INT_MAX)
				return EAGAIN;
			ptm->ptm_recursed++;
			return 0;
		}
		if (__SIMPLELOCK_LOCKED_P(&ptm->ptm_errorcheck))
			return EDEADLK;
	}

	/* priority protect */
	if (MUTEX_PROTECT(owner) && _sched_protect(ptm->ptm_ceiling) == -1) {
		error = errno;
		errno = serrno;
		return error;
	}

	for (;;) {
		/* If it has become free, try to acquire it again. */
		if (MUTEX_OWNER(owner) == 0) {
			newval = (void *)((uintptr_t)self | (uintptr_t)owner);
			next = atomic_cas_ptr(&ptm->ptm_owner, owner, newval);
			if (__predict_false(next != owner)) {
				owner = next;
				continue;
			}
			errno = serrno;
#ifndef PTHREAD__ATOMIC_IS_MEMBAR
			membar_enter();
#endif
			return 0;
		} else if (MUTEX_OWNER(owner) != (uintptr_t)self) {
			/* Spin while the owner is running. */
			owner = pthread__mutex_spin(ptm, owner);
			if (MUTEX_OWNER(owner) == 0) {
				continue;
			}
		}

		/*
		 * Nope, still held.  Add thread to the list of waiters.
		 * Issue a memory barrier to ensure mutexwait/mutexnext
		 * are visible before we enter the waiters list.
		 */
		self->pt_mutexwait = 1;
		for (waiters = ptm->ptm_waiters;; waiters = next) {
			self->pt_mutexnext = waiters;
#ifndef PTHREAD__ATOMIC_IS_MEMBAR
			membar_producer();
#endif
			next = atomic_cas_ptr(&ptm->ptm_waiters, waiters, self);
			if (next == waiters)
			    	break;
		}
		
		/*
		 * Try to set the waiters bit.  If the mutex has become free
		 * since entering self onto the waiters list, need to wake
		 * everybody up (including self) and retry.  It's possible
		 * to race with the unlocking thread, so self may have
		 * already been awoken.
		 */
#ifndef PTHREAD__ATOMIC_IS_MEMBAR
		membar_sync();
#endif
		next = atomic_cas_ptr(&ptm->ptm_owner, owner,
		    (void *)((uintptr_t)owner | MUTEX_WAITERS_BIT));
		if (next != owner) {
			pthread__mutex_wakeup(self, ptm);
		}

		/*
		 * We must not proceed until told that we are no longer
		 * waiting (via pt_mutexwait being set to zero).  Otherwise
		 * it is unsafe to re-enter the thread onto the waiters
		 * list.
		 */
		do {
			pthread__assert(self->pt_nwaiters <= 1);
			pthread__assert(self->pt_nwaiters != 0 ||
			    self->pt_waiters[0] == 0);
			error = _lwp_park(CLOCK_REALTIME, TIMER_ABSTIME,
			    __UNCONST(ts), self->pt_waiters[0], NULL, NULL);
			self->pt_waiters[0] = 0;
			self->pt_nwaiters = 0;
			if (error < 0 && errno == ETIMEDOUT) {
				/* Remove self from waiters list */
				pthread__mutex_wakeup(self, ptm);

				/*
				 * Might have raced with another thread to
				 * do the wakeup.  In any case there will be
				 * a wakeup for sure.  Eat it and wait for
				 * pt_mutexwait to clear.
				 */
				do {
					(void)_lwp_park(CLOCK_REALTIME,
					   TIMER_ABSTIME, NULL, 0, NULL, NULL);
				} while (self->pt_mutexwait);

				/* Priority protect */
				if (MUTEX_PROTECT(owner))
					(void)_sched_protect(-1);
				errno = serrno;
				return ETIMEDOUT;
			}
		} while (self->pt_mutexwait);
		owner = ptm->ptm_owner;
	}
}

int
pthread_mutex_trylock(pthread_mutex_t *ptm)
{
	pthread_t self;
	void *val, *new, *next;

	if (__predict_false(__uselibcstub))
		return __libc_mutex_trylock_stub(ptm);

	pthread__error(EINVAL, "Invalid mutex",
	    ptm->ptm_magic == _PT_MUTEX_MAGIC);

	self = pthread__self();
	val = atomic_cas_ptr(&ptm->ptm_owner, NULL, self);
	if (__predict_true(val == NULL)) {
#ifndef PTHREAD__ATOMIC_IS_MEMBAR
		membar_enter();
#endif
		return 0;
	}

	if (MUTEX_RECURSIVE(val)) {
		if (MUTEX_OWNER(val) == 0) {
			new = (void *)((uintptr_t)self | (uintptr_t)val);
			next = atomic_cas_ptr(&ptm->ptm_owner, val, new);
			if (__predict_true(next == val)) {
#ifndef PTHREAD__ATOMIC_IS_MEMBAR
				membar_enter();
#endif
				return 0;
			}
		}
		if (MUTEX_OWNER(val) == (uintptr_t)self) {
			if (ptm->ptm_recursed == INT_MAX)
				return EAGAIN;
			ptm->ptm_recursed++;
			return 0;
		}
	}

	return EBUSY;
}

int
pthread_mutex_unlock(pthread_mutex_t *ptm)
{
	pthread_t self;
	void *val;
	int error;

	if (__predict_false(__uselibcstub))
		return __libc_mutex_unlock_stub(ptm);

	pthread__error(EINVAL, "Invalid mutex",
	    ptm->ptm_magic == _PT_MUTEX_MAGIC);

#ifndef PTHREAD__ATOMIC_IS_MEMBAR
	membar_exit();
#endif
	error = 0;
	self = pthread__self();

	val = atomic_cas_ptr(&ptm->ptm_owner, self, NULL);
	if (__predict_false(val != self)) {
		bool weown = (MUTEX_OWNER(val) == (uintptr_t)self);
		void *newval = val;
		if (__SIMPLELOCK_LOCKED_P(&ptm->ptm_errorcheck)) {
			if (!weown) {
				error = EPERM;
				newval = val;
			} else {
				newval = NULL;
			}
		} else if (MUTEX_RECURSIVE(val)) {
			if (!weown) {
				error = EPERM;
				newval = val;
			} else if (ptm->ptm_recursed) {
				ptm->ptm_recursed--;
				newval = val;
			} else {
				newval = (pthread_t)MUTEX_RECURSIVE_BIT;
			}
		} else {
			pthread__error(EPERM,
			    "Unlocking unlocked mutex", (val != NULL));
			pthread__error(EPERM,
			    "Unlocking mutex owned by another thread", weown);
			newval = NULL;
		}

		/*
		 * Release the mutex.  If there appear to be waiters, then
		 * wake them up.
		 */
		if (newval != val) {
			val = atomic_swap_ptr(&ptm->ptm_owner, newval);
			if (__predict_false(MUTEX_PROTECT(val))) {
				/* restore elevated priority */
				(void)_sched_protect(-1);
			}
		}
	}

	/*
	 * Finally, wake any waiters and return.
	 */
#ifndef PTHREAD__ATOMIC_IS_MEMBAR
	membar_enter();
#endif
	if (MUTEX_HAS_WAITERS(val)) {
		pthread__mutex_wakeup(self, ptm);
	} else if (self->pt_nwaiters > 0) {
		pthread__clear_waiters(self);
	}
	return error;
}

/*
 * pthread__mutex_wakeup: unpark threads waiting for us
 *
 * unpark threads on the ptm->ptm_waiters list and self->pt_waiters.
 */

static void
pthread__mutex_wakeup(pthread_t self, pthread_mutex_t *ptm)
{
	pthread_t thread, next;

	/* Take ownership of the current set of waiters. */
	thread = atomic_swap_ptr(&ptm->ptm_waiters, NULL);
	membar_datadep_consumer(); /* for alpha */

	/*
	 * Pull waiters from the queue and add to our list.  Use a memory
	 * barrier to ensure that we safely read the value of pt_mutexnext
	 * before 'thread' sees pt_mutexwait being cleared.
	 */
	while (thread != NULL) {
		if (self->pt_nwaiters >= pthread__unpark_max) {
			pthread__clear_waiters(self);
		}
		next = thread->pt_mutexnext;
		self->pt_waiters[self->pt_nwaiters++] = thread->pt_lid;
		membar_sync();
		thread->pt_mutexwait = 0;
		/* No longer safe to touch 'thread' */
		thread = next;
	}
	pthread__clear_waiters(self);
}

int
pthread_mutexattr_init(pthread_mutexattr_t *attr)
{
#if 0
	if (__predict_false(__uselibcstub))
		return __libc_mutexattr_init_stub(attr);
#endif

	attr->ptma_magic = _PT_MUTEXATTR_MAGIC;
	attr->ptma_private = (void *)PTHREAD_MUTEX_DEFAULT;
	return 0;
}

int
pthread_mutexattr_destroy(pthread_mutexattr_t *attr)
{
	if (__predict_false(__uselibcstub))
		return __libc_mutexattr_destroy_stub(attr);

	pthread__error(EINVAL, "Invalid mutex attribute",
	    attr->ptma_magic == _PT_MUTEXATTR_MAGIC);

	attr->ptma_magic = _PT_MUTEXATTR_DEAD;

	return 0;
}

int
pthread_mutexattr_gettype(const pthread_mutexattr_t *attr, int *typep)
{

	pthread__error(EINVAL, "Invalid mutex attribute",
	    attr->ptma_magic == _PT_MUTEXATTR_MAGIC);

	*typep = MUTEX_GET_TYPE(attr->ptma_private);
	return 0;
}

int
pthread_mutexattr_settype(pthread_mutexattr_t *attr, int type)
{

	if (__predict_false(__uselibcstub))
		return __libc_mutexattr_settype_stub(attr, type);

	pthread__error(EINVAL, "Invalid mutex attribute",
	    attr->ptma_magic == _PT_MUTEXATTR_MAGIC);

	switch (type) {
	case PTHREAD_MUTEX_NORMAL:
	case PTHREAD_MUTEX_ERRORCHECK:
	case PTHREAD_MUTEX_RECURSIVE:
		MUTEX_SET_TYPE(attr->ptma_private, type);
		return 0;
	default:
		return EINVAL;
	}
}

int
pthread_mutexattr_getprotocol(const pthread_mutexattr_t *attr, int*proto)
{
	
	pthread__error(EINVAL, "Invalid mutex attribute",
	    attr->ptma_magic == _PT_MUTEXATTR_MAGIC);

	*proto = MUTEX_GET_PROTOCOL(attr->ptma_private);
	return 0;
}

int 
pthread_mutexattr_setprotocol(pthread_mutexattr_t* attr, int proto)
{

	pthread__error(EINVAL, "Invalid mutex attribute",
	    attr->ptma_magic == _PT_MUTEXATTR_MAGIC);

	switch (proto) {
	case PTHREAD_PRIO_NONE:
	case PTHREAD_PRIO_PROTECT:
		MUTEX_SET_PROTOCOL(attr->ptma_private, proto);
		return 0;
	case PTHREAD_PRIO_INHERIT:
		return ENOTSUP;
	default:
		return EINVAL;
	}
}

int 
pthread_mutexattr_getprioceiling(const pthread_mutexattr_t *attr, int *ceil)
{
	
	pthread__error(EINVAL, "Invalid mutex attribute",
		attr->ptma_magic == _PT_MUTEXATTR_MAGIC);

	*ceil = MUTEX_GET_CEILING(attr->ptma_private);
	return 0;
}

int 
pthread_mutexattr_setprioceiling(pthread_mutexattr_t *attr, int ceil) 
{

	pthread__error(EINVAL, "Invalid mutex attribute",
		attr->ptma_magic == _PT_MUTEXATTR_MAGIC);

	if (ceil & ~0xff)
		return EINVAL;

	MUTEX_SET_CEILING(attr->ptma_private, ceil);
	return 0;
}

#ifdef _PTHREAD_PSHARED
int
pthread_mutexattr_getpshared(const pthread_mutexattr_t * __restrict attr,
    int * __restrict pshared)
{

	pthread__error(EINVAL, "Invalid mutex attribute",
		attr->ptma_magic == _PT_MUTEXATTR_MAGIC);

	*pshared = PTHREAD_PROCESS_PRIVATE;
	return 0;
}

int
pthread_mutexattr_setpshared(pthread_mutexattr_t *attr, int pshared)
{

	pthread__error(EINVAL, "Invalid mutex attribute",
		attr->ptma_magic == _PT_MUTEXATTR_MAGIC);

	switch(pshared) {
	case PTHREAD_PROCESS_PRIVATE:
		return 0;
	case PTHREAD_PROCESS_SHARED:
		return ENOSYS;
	}
	return EINVAL;
}
#endif

/*
 * pthread__mutex_deferwake: try to defer unparking threads in self->pt_waiters
 *
 * In order to avoid unnecessary contention on interlocking mutexes, we try
 * to defer waking up threads until we unlock the mutex.  The threads will
 * be woken up when the calling thread (self) releases a mutex.
 */
void
pthread__mutex_deferwake(pthread_t self, pthread_mutex_t *ptm)
{

	if (__predict_false(ptm == NULL ||
	    MUTEX_OWNER(ptm->ptm_owner) != (uintptr_t)self)) {
		pthread__clear_waiters(self);
	}
}

int
pthread_mutex_getprioceiling(const pthread_mutex_t *ptm, int *ceil) 
{

	pthread__error(EINVAL, "Invalid mutex",
	    ptm->ptm_magic == _PT_MUTEX_MAGIC);

	*ceil = ptm->ptm_ceiling;
	return 0;
}

int
pthread_mutex_setprioceiling(pthread_mutex_t *ptm, int ceil, int *old_ceil) 
{
	int error;

	pthread__error(EINVAL, "Invalid mutex",
	    ptm->ptm_magic == _PT_MUTEX_MAGIC);

	error = pthread_mutex_lock(ptm);
	if (error == 0) {
		*old_ceil = ptm->ptm_ceiling;
		/*check range*/
		ptm->ptm_ceiling = ceil;
		pthread_mutex_unlock(ptm);
	}
	return error;
}

int
_pthread_mutex_held_np(pthread_mutex_t *ptm)
{

	return MUTEX_OWNER(ptm->ptm_owner) == (uintptr_t)pthread__self();
}

pthread_t
_pthread_mutex_owner_np(pthread_mutex_t *ptm)
{

	return (pthread_t)MUTEX_OWNER(ptm->ptm_owner);
}
