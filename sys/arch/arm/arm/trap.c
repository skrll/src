/*	$NetBSD: trap.c,v 1.41 2020/12/11 18:03:33 skrll Exp $	*/

/*-
 * Copyright (c) 2020 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Nick Hudson.
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

#include <sys/cdefs.h>
__KERNEL_RCSID(1, "$NetBSD$");

#include <sys/param.h>
#include <sys/types.h>

#include <arm/frame.h>


/* ------------------------------------------------------------------ */
// Only because machdep.h is a mess

#include <uvm/uvm_extern.h>

#include <arm/arm32/pmap.h>


/* ------------------------------------------------------------------ */




#include <arm/arm32/machdep.h>

#if 0
#include "opt_arm_intr_impl.h"
#include "opt_compat_netbsd32.h"
#include "opt_dtrace.h"

#include <sys/param.h>
#include <sys/kauth.h>
#include <sys/types.h>
#include <sys/atomic.h>
#include <sys/cpu.h>
#include <sys/evcnt.h>
#ifdef KDB
#include <sys/kdb.h>
#endif
#include <sys/proc.h>
#include <sys/systm.h>
#include <sys/signal.h>
#include <sys/signalvar.h>
#include <sys/siginfo.h>
#include <sys/xcall.h>

#endif

#if 0
#include <arm/cpufunc.h>

#include <aarch64/userret.h>
#include <aarch64/frame.h>
#include <aarch64/machdep.h>
#include <aarch64/armreg.h>
#include <aarch64/locore.h>

#include <arm/cpufunc.h>

#ifdef KDB
#include <machine/db_machdep.h>
#endif
#ifdef DDB
#include <ddb/db_output.h>
#include <machine/db_machdep.h>
#endif
#ifdef KDTRACE_HOOKS
#include <sys/dtrace_bsd.h>
#endif

#ifdef DDB
int sigill_debug = 0;
#endif

#endif

void
cpu_jump_onfault(struct trapframe *tf, const struct faultbuf *fb, int val)
{
	tf->tf_r4 = fb->fb_reg[FB_R4];
	tf->tf_r5 = fb->fb_reg[FB_R5];
	tf->tf_r6 = fb->fb_reg[FB_R6];
	tf->tf_r7 = fb->fb_reg[FB_R7];
	tf->tf_r8 = fb->fb_reg[FB_R8];
	tf->tf_r9 = fb->fb_reg[FB_R9];
	tf->tf_r10 = fb->fb_reg[FB_R10];
	tf->tf_r11 = fb->fb_reg[FB_R11];
	tf->tf_r12 = fb->fb_reg[FB_R12];
//	tf->tf_r13 = fb->fb_reg[FB_R13];
//	tf->tf_r29 = fb->fb_reg[FB_R14];
	tf->tf_svc_sp = fb->fb_reg[FB_R13];
	tf->tf_pc = fb->fb_reg[FB_R14];
	tf->tf_r0 = val;
}

