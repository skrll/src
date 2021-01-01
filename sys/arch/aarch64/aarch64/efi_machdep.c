/* $NetBSD: efi_machdep.c,v 1.8 2020/10/22 07:31:15 skrll Exp $ */

/*-
 * Copyright (c) 2018 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Jared McNeill <jmcneill@invisible.ca>.
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
__KERNEL_RCSID(0, "$NetBSD: efi_machdep.c,v 1.8 2020/10/22 07:31:15 skrll Exp $");

#include <sys/param.h>
#include <uvm/uvm_extern.h>

#include <arm/cpufunc.h>

#include <arm/arm/efi_runtime.h>

#include <aarch64/machdep.h>

static struct {
	struct faultbuf	faultbuf;
	bool		fpu_used;
} arm_efirt_state;

void
arm_efirt_md_map_range(vaddr_t va, paddr_t pa, size_t sz,
    enum arm_efirt_mem_type type)
{
	int flags = 0;
	int prot = 0;

	switch (type) {
	case ARM_EFIRT_MEM_CODE:
		/* need write permission because fw devs */
		prot = VM_PROT_READ | VM_PROT_WRITE | VM_PROT_EXECUTE;
		break;
	case ARM_EFIRT_MEM_DATA:
		prot = VM_PROT_READ | VM_PROT_WRITE;
		break;
	case ARM_EFIRT_MEM_MMIO:
		prot = VM_PROT_READ | VM_PROT_WRITE;
		flags = PMAP_DEV;
		break;
	default:
		panic("%s: unsupported type %d", __func__, type);
	}

	while (sz != 0) {
		pmap_kenter_pa(va, pa, prot, flags);
		va += PAGE_SIZE;
		pa += PAGE_SIZE;
		sz -= PAGE_SIZE;
	}
	pmap_update(pmap_kernel());
}

int
arm_efirt_md_enter(void)
{
	struct lwp *l = curlwp;

	/* Save FPU state */
	arm_efirt_state.fpu_used = fpu_used_p(l) != 0;
	if (arm_efirt_state.fpu_used)
		fpu_save(l);

	/* Enable FP access (AArch64 UEFI calling convention) */
	reg_cpacr_el1_write(CPACR_FPEN_ALL);
	isb();

	/*
	 * Install custom fault handler. EFI lock is held across calls so
	 * shared faultbuf is safe here.
	 */
	return cpu_set_onfault(&arm_efirt_state.faultbuf);
}

void
arm_efirt_md_exit(void)
{
	struct lwp *l = curlwp;

	/* Disable FP access */
	reg_cpacr_el1_write(CPACR_FPEN_NONE);
	isb();

	/* Restore FPU state */
	if (arm_efirt_state.fpu_used)
		fpu_load(l);

	/* Remove custom fault handler */
	cpu_unset_onfault();
}
