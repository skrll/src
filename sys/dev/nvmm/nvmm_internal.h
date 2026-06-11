/*	$NetBSD: nvmm_internal.h,v 1.22 2026/02/08 10:28:04 nia Exp $	*/

/*
 * Copyright (c) 2018-2020 Maxime Villard, m00nbsd.net
 * All rights reserved.
 *
 * This code is part of the NVMM hypervisor.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _NVMM_INTERNAL_H_
#define _NVMM_INTERNAL_H_

#if defined(_KERNEL_OPT)
#include "opt_nvmm.h"
#endif

#include <sys/types.h>

#include <sys/lwp.h>
#include <sys/mutex.h>
#include <sys/rwlock.h>
#include <sys/sched.h>

#include <dev/nvmm/nvmm.h>

/*
 * Make NVMMHIST_PRINT force on KERNHIST_PRINT for at least NVMMHIST_* usage.
 */
#if defined(NVMMHIST_PRINT) && !defined(KERNHIST_PRINT)
#define KERNHIST_PRINT 1
#endif

#include <sys/kernhist.h>

#if defined(NVMM_DEBUG)

extern int nvmmdebug;

#define NVMMHIST_DECL(NAME)		KERNHIST_DECL(NAME)
#define NVMMHIST_DEFINE(NAME)		KERNHIST_DEFINE(NAME)
#define NVMMHIST_INIT(NAME,N)		KERNHIST_INIT(NAME,N)
#define NVMMHIST_LINK_STATIC(NAME)	KERNHIST_LINK_STATIC(NAME)
#define NVMMHIST_LOGN(NAME,N,FMT,A,B,C,D)	do {		\
	if ((NAME) >= (N)) {					\
		KERNHIST_LOG(nvmmhist,FMT,A,B,C,D);		\
	}							\
} while (0)
#define NVMMHIST_LOGM(NAME,N,FMT,A,B,C,D)	do {		\
	if ((NAME) & (N)) {					\
		KERNHIST_LOG(nvmmhist,FMT,A,B,C,D);		\
	}							\
} while (0)
#define NVMMHIST_LOG(NAME,FMT,A,B,C,D)	NVMMHIST_LOGN(NAME,1,FMT,A,B,C,D)
#define NVMMHIST_CALLED(NAME)			do {		\
	if ((NAME) != 0) {					\
		KERNHIST_CALLED(nvmmhist);			\
	}							\
} while (0)
#define NVMMHIST_CALLARGS(NAME,FMT,A,B,C,D) do {			\
	if ((NAME) != 0) {					\
		KERNHIST_CALLARGS(nvmmhist,FMT,A,B,C,D);		\
	}							\
} while (0)
#define NVMMHIST_CALLARGSN(NAME,N,FMT,A,B,C,D) do {		\
	if ((NAME) >= (N)) {					\
		KERNHIST_CALLARGS(nvmmhist,FMT,A,B,C,D);		\
	}							\
} while (0)
#define NVMMHIST_FUNC()			KERNHIST_FUNC(__func__)

NVMMHIST_DECL(nvmmhist);

#else

#define NVMMHIST_DECL(NAME)
#define NVMMHIST_DEFINE(NAME)
#define NVMMHIST_INIT(NAME,N)
#define NVMMHIST_LINK_STATIC(NAME)
#define NVMMHIST_LOGN(N,NAME,FMT,A,B,C,D)	do { } while(0)
#define NVMMHIST_LOGM(N,NAME,FMT,A,B,C,D)	do { } while(0)
#define NVMMHIST_LOG(NAME,FMT,A,B,C,D)		do { } while(0)
#define NVMMHIST_CALLARGS(NAME,FMT,A,B,C,D)
#define NVMMHIST_CALLARGSN(NAME,N,FMT,A,B,C,D)
#define NVMMHIST_CALLED(NAME)
#define NVMMHIST_FUNC()

#endif


struct uvm_object;
struct vmspace;

#define NVMM_MAX_MACHINES	128
#define NVMM_MAX_VCPUS		256
#define NVMM_MAX_HMAPPINGS	32
#define NVMM_MAX_RAM		(128ULL * (1 << 30))

struct nvmm_owner {
	pid_t pid;
};

struct nvmm_cpu {
	/* Shared. */
	bool present;
	nvmm_cpuid_t cpuid;
	kmutex_t lock;

	/* Comm page. */
	struct nvmm_comm_page *comm;

	/* Last host CPU on which the VCPU ran. */
	int hcpu_last;

	/* Implementation-specific. */
	void *cpudata;
};

struct nvmm_hmapping {
	bool present;
	uintptr_t hva;
	size_t size;
	struct uvm_object *uobj;
};

struct nvmm_machine {
	bool present;
	nvmm_machid_t machid;
	time_t time;
	struct nvmm_owner *owner;
	krwlock_t lock;

	/* Comm */
	struct uvm_object *commuobj;

	/* Kernel */
	struct vmspace *vm;
	gpaddr_t gpa_begin;
	gpaddr_t gpa_end;

	/* Host Mappings */
	struct nvmm_hmapping hmap[NVMM_MAX_HMAPPINGS];

	/* CPU */
	volatile unsigned int ncpus;
	struct nvmm_cpu cpus[NVMM_MAX_VCPUS];

	/* Implementation-specific */
	void *machdata;
};

struct nvmm_impl {
	const char *name;
	bool (*ident)(void);
	void (*init)(void);
	void (*fini)(void);
	void (*capability)(struct nvmm_capability *);
	void (*suspend_interrupt)(void);
	void (*suspend)(void);
	void (*resume)(void);

	size_t mach_conf_max;
	const size_t *mach_conf_sizes;

	size_t vcpu_conf_max;
	const size_t *vcpu_conf_sizes;

	size_t state_size;

	void (*machine_create)(struct nvmm_machine *);
	void (*machine_destroy)(struct nvmm_machine *);
	int (*machine_configure)(struct nvmm_machine *, uint64_t, void *);
	void (*machine_suspend)(struct nvmm_machine *);
	void (*machine_resume)(struct nvmm_machine *);

	int (*vcpu_create)(struct nvmm_machine *, struct nvmm_cpu *);
	void (*vcpu_destroy)(struct nvmm_machine *, struct nvmm_cpu *);
	int (*vcpu_configure)(struct nvmm_cpu *, uint64_t, void *);
	void (*vcpu_setstate)(struct nvmm_cpu *);
	void (*vcpu_getstate)(struct nvmm_cpu *);
	int (*vcpu_inject)(struct nvmm_cpu *);
	int (*vcpu_run)(struct nvmm_machine *, struct nvmm_cpu *,
	    struct nvmm_vcpu_exit *);
	void (*vcpu_suspend)(struct nvmm_machine *, struct nvmm_cpu *);
	void (*vcpu_resume)(struct nvmm_machine *, struct nvmm_cpu *);
};

#if defined(__aarch64__)
extern const struct nvmm_impl nvmm_aarch64;
#elif defined(__x86_64__)
extern const struct nvmm_impl nvmm_x86_svm;
extern const struct nvmm_impl nvmm_x86_vmx;
#endif

extern volatile bool nvmm_suspending;

static inline bool
nvmm_return_needed(struct nvmm_cpu *vcpu, struct nvmm_vcpu_exit *exit)
{

	if (__predict_false(preempt_needed())) {
		exit->reason = NVMM_VCPU_EXIT_NONE;
		return true;
	}
	if (__predict_false(curlwp->l_flag & LW_USERRET)) {
		exit->reason = NVMM_VCPU_EXIT_NONE;
		return true;
	}
	if (__predict_false(vcpu->comm->stop)) {
		exit->reason = NVMM_VCPU_EXIT_STOPPED;
		return true;
	}
	if (__predict_false(atomic_load_relaxed(&nvmm_suspending))) {
		exit->reason = NVMM_VCPU_EXIT_NONE;
		return true;
	}

	return false;
}

#endif /* _NVMM_INTERNAL_H_ */
