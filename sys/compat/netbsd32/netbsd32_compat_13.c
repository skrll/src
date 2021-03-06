/*	$NetBSD: netbsd32_compat_13.c,v 1.28 2021/01/19 03:20:13 simonb Exp $	*/

/*
 * Copyright (c) 1998, 2001 Matthew R. Green
 * All rights reserved.
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

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: netbsd32_compat_13.c,v 1.28 2021/01/19 03:20:13 simonb Exp $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/module.h>
#include <sys/mount.h>
#include <sys/proc.h>
#include <sys/signal.h>
#include <sys/signalvar.h>
#include <sys/syscallargs.h>
#include <sys/syscallvar.h>

#include <compat/netbsd32/netbsd32.h>
#include <compat/netbsd32/netbsd32_syscall.h>
#include <compat/netbsd32/netbsd32_syscallargs.h>

#include <compat/sys/stat.h>
#include <compat/sys/signal.h>
#include <compat/sys/signalvar.h>

#include <compat/common/compat_sigaltstack.h>

int
compat_13_netbsd32_sigaltstack13(struct lwp *l, const struct compat_13_netbsd32_sigaltstack13_args *uap, register_t *retval)
{
	compat_sigaltstack(uap, netbsd32_sigaltstack13, SS_ONSTACK, SS_DISABLE);
}


int
compat_13_netbsd32_sigprocmask(struct lwp *l, const struct compat_13_netbsd32_sigprocmask_args *uap, register_t *retval)
{
	/* {
		syscallarg(int) how;
		syscallarg(int) mask;
	} */
	sigset13_t ness, oess;
	sigset_t nbss, obss;
	struct proc *p = l->l_proc;
	int error;

	ness = SCARG(uap, mask);
	native_sigset13_to_sigset(&ness, &nbss);
	mutex_enter(p->p_lock);
	error = sigprocmask1(l, SCARG(uap, how), &nbss, &obss);
	mutex_exit(p->p_lock);
	if (error)
		return error;
	native_sigset_to_sigset13(&obss, &oess);
	*retval = oess;
	return 0;
}

int
compat_13_netbsd32_sigsuspend(struct lwp *l, const struct compat_13_netbsd32_sigsuspend_args *uap, register_t *retval)
{
	/* {
		syscallarg(sigset13_t) mask;
	} */
	sigset13_t ess;
	sigset_t bss;

	ess = SCARG(uap, mask);
	native_sigset13_to_sigset(&ess, &bss);
	return sigsuspend1(l, &bss);
}

static struct syscall_package compat_netbsd32_13_syscalls[] = {
	{ NETBSD32_SYS_compat_13_netbsd32_sigaltstack13, 0,
	    (sy_call_t *)compat_13_netbsd32_sigaltstack13 },
	{ NETBSD32_SYS_compat_13_sigprocmask13, 0,
	    (sy_call_t *)compat_13_netbsd32_sigprocmask },
	{ NETBSD32_SYS_compat_13_sigsuspend13, 0,
	    (sy_call_t *)compat_13_netbsd32_sigsuspend },
	{ 0, 0, NULL }
}; 

MODULE(MODULE_CLASS_EXEC, compat_netbsd32_13, "compat_13,compat_netbsd32_16");

static int
compat_netbsd32_13_modcmd(modcmd_t cmd, void *arg)
{

	switch (cmd) {
	case MODULE_CMD_INIT:
		netbsd32_machdep_md_13_init();
		return syscall_establish(&emul_netbsd32,
		    compat_netbsd32_13_syscalls);

	case MODULE_CMD_FINI:
		netbsd32_machdep_md_13_fini();
		return syscall_disestablish(&emul_netbsd32,
		    compat_netbsd32_13_syscalls);

	default:
		return ENOTTY;
	}
}
