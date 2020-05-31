/*	$NetBSD$	*/

/*
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

#include <sys/atomic.h>
#include <sys/ksyms.h>

#include <arm/vmparam.h>
#include <arm/arm32/machdep.h>
#include <arm/arm32/pmap.h>
#if 0
#include <aarch64/pmap.h>
#include <aarch64/cpufunc.h>
#include <aarch64/armreg.h>
#include <aarch64/machdep.h>
#endif

// XXXNH Wrong place
#define KERNEL_IO_VBASE         0xf0000000
#define VM_KERNEL_IO_ADDRESS	KERNEL_IO_VBASE


#define __MD_VIRTUAL_SHIFT	29
#define __MD_CANONICAL_BASE	0x80000000

#define __MD_SHADOW_SIZE	(1U << (__MD_VIRTUAL_SHIFT - KASAN_SHADOW_SCALE_SHIFT))
#define KASAN_MD_SHADOW_START	(0xc0000000)
#define KASAN_MD_SHADOW_END	(KASAN_MD_SHADOW_START + __MD_SHADOW_SIZE)

static inline int8_t *
kasan_md_addr_to_shad(const void *addr)
{
	vaddr_t va = (vaddr_t)addr;
	return (int8_t *)(KASAN_MD_SHADOW_START +
	    ((va - __MD_CANONICAL_BASE) >> KASAN_SHADOW_SCALE_SHIFT));
}

static inline bool
kasan_md_unsupported(vaddr_t addr)
{
	return (addr < VM_MIN_KERNEL_ADDRESS) ||
	    (addr >= VM_KERNEL_IO_ADDRESS);
}

/* -------------------------------------------------------------------------- */

/*
 * Early mapping, used to map just the stack at boot time. We rely on the fact
 * that VA = PA + KERNEL_BASE.
 */

#define KASAN_NEARLYPAGES	1

static bool __md_early __read_mostly = true;
static uint8_t __md_earlypages[KASAN_NEARLYPAGES * L1_S_SIZE] __aligned(L1_S_SIZE);
static size_t __md_earlytaken = 0;

static paddr_t
__md_early_palloc(void)
{
	vaddr_t va;

	KASSERT(__md_earlytaken < KASAN_NEARLYPAGES);

	va = (vaddr_t)(&__md_earlypages[0] + __md_earlytaken * PAGE_SIZE);
	__md_earlytaken++;

	return KERN_VTOPHYS(va);
}

static void
__md_early_shadow_map_page(vaddr_t va)
{
	/* L1_TABLE_SIZE (16Kb) aligned */
	const uint32_t mask = L1_TABLE_SIZE - 1;
	const paddr_t ttb = (paddr_t)(armreg_ttbr_read() & ~mask);
	pd_entry_t * const pdep = (pd_entry_t *)KERN_PHYSTOV(ttb);
	const size_t l1slot = l1pte_index(va);

	if (!l1pte_valid_p(pdep[l1slot])) {
		const paddr_t pa = __md_early_palloc();
		const int prot = VM_PROT_READ | VM_PROT_WRITE;
		const pd_entry_t npde = L1_S_PROTO
		    | pa
		    | pte_l1_s_cache_mode
		    | L1_S_PROT(PTE_KERNEL, prot)
		    | L1_S_DOM(PMAP_DOMAIN_KERNEL);

		l1pte_set(&pdep[l1slot], npde);
		PDE_SYNC(&pdep[l1slot]);
	}
}

#if 0
static paddr_t
__md_palloc(void)
{
	/* The page is zeroed. */
	return pmap_get_physpage();
}
#endif

static inline paddr_t
__md_palloc_large(void)
{
	struct pglist pglist;
	int ret;

	if (!uvm.page_init_done)
		return 0;

	ret = uvm_pglistalloc(L1_S_SIZE, 0, ~0UL, L1_S_SIZE, 0,
	    &pglist, 1, 0);
	if (ret != 0)
		return 0;

	/* The page may not be zeroed. */
	return VM_PAGE_TO_PHYS(TAILQ_FIRST(&pglist));
}


static void
kasan_md_shadow_map_page(vaddr_t va)
{

	if (__predict_false(__md_early)) {
		__md_early_shadow_map_page(va);
		return;
	}

	/* 16Kb aligned */
	const uint32_t mask16k = (16 * 1024) - 1;
	const paddr_t ttb = (paddr_t)(armreg_ttbr_read() & ~mask16k);
	pd_entry_t * const pdep = (pd_entry_t *)KERN_PHYSTOV(ttb);
	const size_t l1slot = l1pte_index(va);

	if (!l1pte_valid_p(pdep[l1slot])) {
		const paddr_t pa = __md_palloc_large();
		const int prot = VM_PROT_READ | VM_PROT_WRITE;
		const pd_entry_t npde = L1_S_PROTO
		    | pa
		    | pte_l1_s_cache_mode
		    | L1_S_PROT(PTE_KERNEL, prot)
		    | L1_S_DOM(PMAP_DOMAIN_KERNEL);

		l1pte_set(&pdep[l1slot], npde);
		PDE_SYNC(&pdep[l1slot]);
	}
}

/*
 * Map only the current stack. We will map the rest in kasan_init.
 */
#define INIT_ARM_STACK_SIZE	2048

static void
kasan_md_early_init(void *stack)
{
	kasan_shadow_map(stack, INIT_ARM_STACK_SIZE);
	__md_early = false;
}


static void
kasan_md_init(void)
{
	vaddr_t eva, dummy;

//	CTASSERT((__MD_SHADOW_SIZE / L0_SIZE) == 64);

	/* The VAs we've created until now. */
	pmap_virtual_space(&eva, &dummy);
	kasan_shadow_map((void *)VM_MIN_KERNEL_ADDRESS,
	    eva - VM_MIN_KERNEL_ADDRESS);
}


static inline bool
__md_unwind_end(const char *name)
{
#if 0
	if (!strncmp(name, "el0_trap", 8) ||
	    !strncmp(name, "el1_trap", 8)) {
		return true;
	}
#endif
	return false;
}

static void
kasan_md_unwind(void)
{
#if 0
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
#endif
}
