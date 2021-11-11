/*	$NetBSD$	*/

/*
 * Copyright (c) 2021 Nick Hudson
 * All rights reserved.
 *
 * This code is part of the KCSAN subsystem of the NetBSD kernel.
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

#include <sys/ksyms.h>
#include <uvm/uvm.h>

#include <arm/armreg.h>
#include <arm/locore.h>

static inline bool
kcsan_md_unsupported(vaddr_t addr)
{

	/*
         * When/If recusrive page table access is defined then exclude the
         * range here.
	 */
	return true;
}

static inline bool
kcsan_md_is_avail(void)
{
	return true;
}

static inline void
kcsan_md_disable_intrs(register_t *psw)
{
	*psw = daif_disable(DAIF_I|DAIF_F);
}

static inline void
kcsan_md_enable_intrs(register_t *psw)
{
	daif_enable(*psw);
}

static inline void
kcsan_md_delay(uint64_t us)
{
	DELAY(us);
}


static inline bool
__md_unwind_end(const char *name)
{
	if (!strncmp(name, "el0_trap", 8) ||
	    !strncmp(name, "el1_trap", 8)) {
		return true;
	}

	return false;
}

static void
kcsan_md_unwind(void)
{
	uint64_t lr, *fp;
	const char *mod;
	const char *sym;
	size_t nsym;
	int error;

	fp = (uint64_t *)__builtin_frame_address(0);
	nsym = 0;

	while (1) {
		/*
		 * normal stack frame
		 *  fp[0]  saved fp(x29) value
		 *  fp[1]  saved lr(x30) value
		 */
		lr = fp[1];

		if (lr < VM_MIN_KERNEL_ADDRESS) {
			break;
		}
		error = ksyms_getname(&mod, &sym, (vaddr_t)lr, KSYMS_PROC);
		if (error) {
			break;
		}
		printf("#%zu %p in %s <%s>\n", nsym, (void *)lr, sym, mod);
		if (__md_unwind_end(sym)) {
			break;
		}

		fp = (uint64_t *)fp[0];
		if (fp == NULL) {
			break;
		}
		nsym++;

		if (nsym >= 15) {
			break;
		}
	}
}
