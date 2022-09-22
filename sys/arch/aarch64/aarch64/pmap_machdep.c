/*	$NetBSD: pmap_machdep.c,v 1.18 2026/06/13 12:12:47 skrll Exp $	*/

/*-
 * Copyright (c) 2022 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Nick Hudson
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

#include "opt_arm_debug.h"
#include "opt_efi.h"
#include "opt_multiprocessor.h"
#include "opt_uvmhist.h"

#define __PMAP_PRIVATE

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: pmap_machdep.c,v 1.18 2026/06/13 12:12:47 skrll Exp $");

#include <sys/param.h>
#include <sys/types.h>

#include <sys/buf.h>
#include <sys/cpu.h>
#include <sys/kernel.h>

#include <uvm/uvm.h>
#include <uvm/uvm_page.h>
#include <uvm/pmap/pmap_pvt.h>

#include <aarch64/cpufunc.h>

#include <arm/locore.h>

#ifdef VERBOSE_INIT_ARM
#define VPRINTF(...)	printf(__VA_ARGS__)
#else
#define VPRINTF(...)	__nothing
#endif

/* Set to LX_BLKPAG_GP if supported. */
uint64_t pmap_attr_gp = 0;

/*
 * Misc variables
 */
vaddr_t virtual_avail;
vaddr_t virtual_end;

PMAP_COUNTER(fixup_referenced, "page reference emulations");
PMAP_COUNTER(fixup_modified, "page modification emulations");

paddr_t
vtophys(vaddr_t va)
{
	paddr_t pa;

	if (pmap_extract(pmap_kernel(), va, &pa) == false)
		return 0;
	return pa;
}

bool
pmap_extract_coherency(pmap_t pm, vaddr_t va, paddr_t *pap, bool *coherentp)
{
	paddr_t pa;
	bool coherency = false;

	if (pm == pmap_kernel()) {
		if (pmap_md_direct_mapped_vaddr_p(va)) {
			pa = pmap_md_direct_mapped_vaddr_to_paddr(va);
			goto done;
		}
		if (pmap_md_io_vaddr_p(va))
			panic("pmap_extract: io address %#"PRIxVADDR"", va);

		if (va >= pmap_limits.virtual_end)
			panic("%s: illegal kernel mapped address %#"PRIxVADDR,
			    __func__, va);
	}

	kpreempt_disable();
	const pt_entry_t * const ptep = pmap_pte_lookup(pm, va);
	pt_entry_t pte;

	if (ptep == NULL || !pte_valid_p(pte = *ptep)) {
		kpreempt_enable();
		return false;
	}
	kpreempt_enable();

	pa = pte_to_paddr(pte) | (va & PGOFSET);

	switch (pte & LX_BLKPAG_ATTR_MASK) {
	case LX_BLKPAG_ATTR_NORMAL_NC:
	case LX_BLKPAG_ATTR_DEVICE_MEM:
	case LX_BLKPAG_ATTR_DEVICE_MEM_NP:
		coherency = true;
		break;
	}

 done:
	if (pap != NULL) {
		*pap = pa;
	}
	if (coherentp != NULL) {
		*coherentp = coherency;
	}
	return true;
}

bool
pmap_fault_fixup(pmap_t pm, vaddr_t va, vm_prot_t ftype, bool user)
{
	UVMHIST_FUNC(__func__); UVMHIST_CALLED(pmaphist);

	KASSERT(!user || (pm != pmap_kernel()));

	kpreempt_disable();

//	struct cpu_info * const ci = curcpu();
	struct pmap_asid_info * const pai = PMAP_PAI(pm, cpu_tlb_info(ci));

	UVMHIST_LOG(pmaphist, " pm=%#jx, va=%#jx, ftype=%#jx, user=%jd",
	    (uintptr_t)pm, va, ftype, user);
	UVMHIST_LOG(pmaphist, " ti=%#jx pai=%#jx asid=%#jx",
	    (uintptr_t)cpu_tlb_info(curcpu()),
	    (uintptr_t)pai, (uintptr_t)pai->pai_asid, 0);

	bool fixed = false;
	pt_entry_t * const ptep = pmap_pte_lookup(pm, va);
	if (ptep == NULL) {
		UVMHIST_LOG(pmaphist, "... no ptep", 0, 0, 0, 0);
		goto done;
	}

	const pt_entry_t opte = atomic_load_relaxed(ptep);
	if (!l3pte_valid(opte)) {
		UVMHIST_LOG(pmaphist, "invalid pte: %016llx: va=%016lx",
		    opte, va, 0, 0);
		goto done;
	}

	const paddr_t pa = l3pte_pa(opte);
	struct vm_page * const pg = PHYS_TO_VM_PAGE(pa);
	if (pg == NULL) {
		UVMHIST_LOG(pmaphist, "pg not found: va=%016lx", va, 0, 0, 0);
		goto done;
	}

	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);
	UVMHIST_LOG(pmaphist, " pg=%#jx, opte=%#jx, ptep=%#jx", (uintptr_t)pg,
	    opte, (uintptr_t)ptep, 0);

	if ((ftype & VM_PROT_WRITE) && (opte & LX_BLKPAG_OS_MODEMUL) != 0) {
		/*
		 * This is "page modified" emulation...
		 */
		pmap_page_set_attributes(mdpg, VM_PAGEMD_MODIFIED | VM_PAGEMD_REFERENCED);

                /*
		 * Enable write permissions for the page by setting the Access
		 * Flag, marking the page as writeable. The PTE being writeable
		 * and OS_MODEMUL indicates it's been modified.
		 */
		const pt_entry_t npte =
		    (opte & ~LX_BLKPAG_AP) |
		    LX_BLKPAG_AF |
		    LX_BLKPAG_AP_RW;
		atomic_swap_64(ptep, npte);
		// tlb_invalidate_addr does dsb(ishst)
		tlb_invalidate_addr(va, pai->pai_asid);
		fixed = true;

		PMAP_COUNT(fixup_modified);

		UVMHIST_LOG(pmaphist, " <-- done (mod emul: changed pte "
		    "from %#jx to %#jx)", opte, npte, 0, 0);
	} else if ((ftype & VM_PROT_READ) && (opte & LX_BLKPAG_AP) == LX_BLKPAG_AP_RO) {
		/*
		 * This looks like a good candidate for "page referenced"
		 * emulation.
		 */

		KASSERT((opte & LX_BLKPAG_AF) == 0);

		pmap_page_set_attributes(mdpg, VM_PAGEMD_REFERENCED);

		/*
		 * Enable write permissions for the page by setting the Access Flag.
		 */
		const pt_entry_t npte = opte | LX_BLKPAG_AF;
		atomic_swap_64(ptep, npte);
		// tlb_invalidate_addr does dsb(ishst)
		tlb_invalidate_addr(va, pai->pai_asid);
		fixed = true;

		PMAP_COUNT(fixup_referenced);

		UVMHIST_LOG(pmaphist, " <-- done (ref emul: changed pte "
		    "from %#jx to %#jx)", opte, npte, 0, 0);
	}

done:
	kpreempt_enable();

	return fixed;
}


void
pmap_icache_sync_range(pmap_t pm, vaddr_t sva, vaddr_t eva)
{
	UVMHIST_FUNC(__func__);
	UVMHIST_CALLARGS(pmaphist, "pm %#jx sva %#jx eva %#jx",
	   (uintptr_t)pm, sva, eva, 0);

	KASSERT((sva & PAGE_MASK) == 0);
	KASSERT((eva & PAGE_MASK) == 0);

	pmap_lock(pm);

	for (vaddr_t va = sva; va < eva; va += PAGE_SIZE) {
		pt_entry_t * const ptep = pmap_pte_lookup(pm, va);
		if (ptep == NULL)
			continue;

		pt_entry_t opte = *ptep;
		if (!l3pte_valid(opte)) {
			UVMHIST_LOG(pmaphist, "invalid pte: %016llx: va=%016lx",
			    opte, va, 0, 0);
			goto done;
		}

		if (l3pte_readable(opte)) {
			cpu_icache_sync_range(va, PAGE_SIZE);
		} else {
			/*
			 * change to accessible temporarily
			 * to do cpu_icache_sync_range()
			 */
			struct pmap_asid_info * const pai = PMAP_PAI(pm,
			    cpu_tlb_info(ci));

			atomic_swap_64(ptep, opte | LX_BLKPAG_AF);
			// tlb_invalidate_addr does the dsb(ishst);
			tlb_invalidate_addr(va, pai->pai_asid);
			cpu_icache_sync_range(va, PAGE_SIZE);
			atomic_swap_64(ptep, opte);
			tlb_invalidate_addr(va, pai->pai_asid);
		}
	}
done:
	pmap_unlock(pm);
}


struct vm_page *
pmap_md_alloc_poolpage(int flags)
{

	/*
	 * Any managed page works for us.
	 */
	return uvm_pagealloc(NULL, 0, NULL, flags);
}


vaddr_t
pmap_md_map_poolpage(paddr_t pa, size_t len)
{
	struct vm_page * const pg = PHYS_TO_VM_PAGE(pa);
	const vaddr_t va = pmap_md_direct_map_paddr(pa);
	KASSERT(cold || pg != NULL);

	if (pg != NULL) {
		struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);
		const pv_entry_t pv = &mdpg->mdpg_first;
		const vaddr_t last_va = trunc_page(pv->pv_va);

		KASSERT(len == PAGE_SIZE || last_va == pa);
		KASSERT(pv->pv_pmap == NULL);
		KASSERT(pv->pv_next == NULL);
		KASSERT(!VM_PAGEMD_EXECPAGE_P(mdpg));

#ifdef PMAP_VIRTUAL_CACHE_ALIASES
		/*
		 * If this page was last mapped with an address that
		 * might cause aliases, flush the page from the cache.
		 */
		if (AARCH64_CACHE_VIRTUAL_ALIAS
		    && aarch64_cache_badalias(last_va, va)) {
			pmap_md_vca_page_wbinv(mdpg, false);
		}
#endif

		pv->pv_va = va;
	}

	return va;
}


paddr_t
pmap_md_unmap_poolpage(vaddr_t va, size_t len)
{
	KASSERT(len == PAGE_SIZE);
	KASSERT(pmap_md_direct_mapped_vaddr_p(va));

	const paddr_t pa = pmap_md_direct_mapped_vaddr_to_paddr(va);
	struct vm_page * const pg = PHYS_TO_VM_PAGE(pa);

	KASSERT(pg);
	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);

#ifdef PMAP_VIRTUAL_CACHE_ALIASES
	KASSERT(VM_PAGEMD_CACHED_P(mdpg));
#endif
	KASSERT(!VM_PAGEMD_EXECPAGE_P(mdpg));

	const pv_entry_t pv = &mdpg->mdpg_first;

	/* Note last mapped address for future color check */
	pv->pv_va = va;

	KASSERT(pv->pv_pmap == NULL);
	KASSERT(pv->pv_next == NULL);

	return pa;
}


bool
pmap_md_kernel_vaddr_p(vaddr_t va)
{
	extern char __kernel_text[];
	extern char _end[];
	extern long kernend_extra;

	const vaddr_t kernstart = trunc_page((vaddr_t)__kernel_text);
	const vaddr_t kernend = round_page((vaddr_t)_end);

	const vaddr_t fva = L2_TRUNC_BLOCK(kernstart);
	const vaddr_t lva = L2_ROUND_BLOCK(kernend + kernend_extra);

	if (va >= fva && va < lva) {
		return true;
	}

	return false;
}

paddr_t
pmap_md_kernel_vaddr_to_paddr(vaddr_t va)
{

	if (pmap_md_kernel_vaddr_p(va)) {
		return KERN_VTOPHYS(va);
	}
	panic("%s: va %#" PRIxVADDR " not direct mapped!", __func__, va);
}


bool
pmap_md_direct_mapped_vaddr_p(vaddr_t va)
{
	if (!AARCH64_KVA_P(va))
		return false;

	paddr_t pa = AARCH64_KVA_TO_PA(va);
	if (physical_start <= pa && pa < physical_end)
		return true;

	return false;
}


paddr_t
pmap_md_direct_mapped_vaddr_to_paddr(vaddr_t va)
{

	return AARCH64_KVA_TO_PA(va);
}


vaddr_t
pmap_md_direct_map_paddr(paddr_t pa)
{

	return AARCH64_PA_TO_KVA(pa);
}


bool
pmap_md_io_vaddr_p(vaddr_t va)
{

	if (pmap_devmap_find_va(va, PAGE_SIZE)) {
		return true;
	}
	return false;
}


static void
pmap_md_grow(pmap_pdetab_t *ptb, vaddr_t va, vsize_t vshift,
    vsize_t *remaining)
{
	KASSERT((va & (NBSEG - 1)) == 0);
	const vaddr_t pdetab_mask = PMAP_PDETABSIZE - 1;
	const vsize_t vinc = 1UL << vshift;

	for (size_t i = (va >> vshift) & pdetab_mask;
	    i < PMAP_PDETABSIZE; i++, va += vinc) {
		pd_entry_t * const pde_p =
		    &ptb->pde_pde[(va >> vshift) & pdetab_mask];

		vaddr_t pdeva;
		if (pte_pde_valid_p(*pde_p)) {
			const paddr_t pa = pte_pde_to_paddr(*pde_p);
			pdeva = pmap_md_direct_map_paddr(pa);
		} else {
			/*
			 * uvm_pageboot_alloc() returns a direct mapped address
			 */
			pdeva = uvm_pageboot_alloc(Ln_TABLE_SIZE);
			paddr_t pdepa = AARCH64_KVA_TO_PA(pdeva);
			*pde_p = pte_pde_pdetab(pdepa, true);
			memset((void *)pdeva, 0, PAGE_SIZE);
		}

		if (vshift > SEGSHIFT) {
			pmap_md_grow((pmap_pdetab_t *)pdeva, va,
			    vshift - SEGLENGTH, remaining);
		} else {
			if (*remaining > vinc)
				*remaining -= vinc;
			else
				*remaining = 0;
		}
		if (*remaining == 0)
			return;
	}
}


void
pmap_bootstrap(vaddr_t vstart, vaddr_t vend)
{
	pmap_t pm = pmap_kernel();

	/*
	 * Initialise the kernel pmap object
	 */
#if 0
	/* uvmexp.ncolors = icachesize / icacheways / PAGE_SIZE; */
	uvmexp.ncolors = aarch64_cache_vindexsize / PAGE_SIZE;

	/* devmap already uses last of va? */
	if ((virtual_devmap_addr != 0) && (virtual_devmap_addr < vend))
		vend = virtual_devmap_addr;

#endif

	virtual_avail = vstart;
	virtual_end = vend;

	aarch64_tlbi_all();

	pm->pm_l0_pa = __SHIFTOUT(reg_ttbr1_el1_read(), TTBR_BADDR);
	pm->pm_pdetab = (pmap_pdetab_t *)AARCH64_PA_TO_KVA(pm->pm_l0_pa);

	VPRINTF("common ");
	pmap_bootstrap_common();

	VPRINTF("tlb0 ");
	pmap_tlb_info_init(&pmap_tlb0_info);

#ifdef MULTIPROCESSOR
	VPRINTF("kcpusets ");

	kcpuset_create(&pm->pm_onproc, true);
	kcpuset_create(&pm->pm_active, true);
	KASSERT(pm->pm_onproc != NULL);
	KASSERT(pm->pm_active != NULL);
	kcpuset_set(pm->pm_onproc, cpu_number());
	kcpuset_set(pm->pm_active, cpu_number());
#endif

	VPRINTF("specials(none) ");

	/*
	 * does VIPT exist for aarch64?
	 */
	//nptes = 1

#if 0
	pmap_alloc_specials(&virtual_avail, nptes, &csrcp, &csrc_pte);
	pmap_set_pt_cache_mode(l1pt, (vaddr_t)csrc_pte, nptes);
	pmap_alloc_specials(&virtual_avail, nptes, &cdstp, &cdst_pte);
	pmap_set_pt_cache_mode(l1pt, (vaddr_t)cdst_pte, nptes);
	pmap_alloc_specials(&virtual_avail, nptes, &memhook, NULL);
	if (msgbufaddr == NULL) {
		pmap_alloc_specials(&virtual_avail,
		    round_page(MSGBUFSIZE) / PAGE_SIZE,
		    (void *)&msgbufaddr, NULL);
	}
#endif


	VPRINTF("nkmempages ");
	/*
	 * Compute the number of pages kmem_arena will have.  This will also
	 * be called by uvm_km_bootstrap later, but that doesn't matter
	 */
	kmeminit_nkmempages();

	/* Get size of buffer cache and set an upper limit */
	buf_setvalimit((VM_MAX_KERNEL_ADDRESS - VM_MIN_KERNEL_ADDRESS) / 8);
	vsize_t bufsz = buf_memcalc();
	buf_setvalimit(bufsz);

	vsize_t kvmsize = (VM_PHYS_SIZE + (ubc_nwins << ubc_winshift) +
	    bufsz + 16 * NCARGS + pager_map_size) +
	    /*(maxproc * UPAGES) + */nkmempages * NBPG;

#ifdef SYSVSHM
	kvmsize += shminfo.shmall;
#endif

	/* Calculate VA address space and roundup to NBSEG tables */
	kvmsize = roundup(kvmsize, NBSEG);

	/*
	 * Initialize `FYI' variables.	Note we're relying on
	 * the fact that BSEARCH sorts the vm_physmem[] array
	 * for us.  Must do this before uvm_pageboot_alloc()
	 * can be called.
	 */
	pmap_limits.avail_start = ptoa(uvm_physseg_get_start(uvm_physseg_get_first()));
	pmap_limits.avail_end = ptoa(uvm_physseg_get_end(uvm_physseg_get_last()));

	/*
	 * Update the naive settings in pmap_limits to the actual KVA range.
	 */
	pmap_limits.virtual_start = vstart;
	pmap_limits.virtual_end = vend;

	VPRINTF("\nlimits: %" PRIxVADDR " - %" PRIxVADDR "\n", vstart, vend);

	const vaddr_t kvmstart = vstart;
	pmap_curmaxkvaddr = vstart + kvmsize;

	VPRINTF("kva   : %" PRIxVADDR " - %" PRIxVADDR "\n", kvmstart,
	    pmap_curmaxkvaddr);

	pmap_md_grow(pmap_kernel()->pm_pdetab, kvmstart, XSEGSHIFT, &kvmsize);

#if defined(EFI_RUNTIME)
	vaddr_t efi_l0va = uvm_pageboot_alloc(Ln_TABLE_SIZE);
	KASSERT((efi_l0va & PAGE_MASK) == 0);

	pmap_t efipm = pmap_efirt();
	efipm->pm_l0_pa = AARCH64_KVA_TO_PA(efi_l0va);
	efipm->pm_pdetab = (pmap_pdetab_t *)efi_l0va;
#endif

	pool_init(&pmap_pmap_pool, PMAP_SIZE, 0, 0, 0, "pmappl",
	    &pool_allocator_nointr, IPL_NONE);

	pool_init(&pmap_pv_pool, sizeof(struct pv_entry), 0, 0, 0, "pvpl",
#ifdef KASAN
	    NULL,
#else
	    &pmap_pv_page_allocator,
#endif
	    IPL_NONE);

	// arm_dcache_align
	pmap_pvlist_lock_init(CACHE_LINE_SIZE);

	VPRINTF("done\n");
}

#if defined(EFI_RUNTIME)
void
pmap_md_activate_efirt(void)
{
	kpreempt_disable();

//	struct cpu_info * const ci = curcpu();
	struct pmap * const pm = pmap_efirt();
	struct pmap_asid_info * const pai = PMAP_PAI(pm, cpu_tlb_info(ci));

	pmap_md_asid_activate(pai->pai_asid, pmap_efirt(), NULL);
}

void
pmap_md_deactivate_efirt(void)
{
	pmap_md_asid_deactivate(pmap_efirt());

	KASSERT((reg_tcr_el1_read() & TCR_EPD0) != 0);

	kpreempt_enable();
}
#endif


void
pmap_md_pdetab_init(struct pmap *pm)
{

	KASSERT(pm != NULL);

	pmap_extract(pmap_kernel(), (vaddr_t)pm->pm_pdetab, &pm->pm_l0_pa);
}

void
pmap_md_pdetab_fini(struct pmap *pm)
{

	KASSERT(pm != NULL);
}

#ifdef PMAP_VIRTUAL_CACHE_ALIASES

static void
pmap_md_vca_page_wbinv(struct vm_page_md *mdpg, bool locked_p)
{
	UVMHIST_FUNC(__func__); UVMHIST_CALLED(pmaphist);
#ifdef needtowrite
	pt_entry_t pte;

	const register_t va = pmap_md_map_ephemeral_page(mdpg, locked_p,
	    VM_PROT_READ, &pte);

	mips_dcache_wbinv_range(va, PAGE_SIZE);

	pmap_md_unmap_ephemeral_page(mdpg, locked_p, va, pte);
#endif
}
#endif


void
pmap_md_page_syncicache(struct vm_page_md *mdpg, const kcpuset_t *onproc)
{
	UVMHIST_FUNC(__func__); UVMHIST_CALLED(pmaphist);

	/*
	 * This handles the PIPT case where we can use the direct map
	 * to sync the icache without worrying about aliasing.
	 */
	struct vm_page * const pg = VM_MD_TO_PAGE(mdpg);
	const paddr_t pa = VM_PAGE_TO_PHYS(pg);
	const vaddr_t kva = pmap_md_direct_map_paddr(pa);
	cpu_icache_sync_range(kva, PAGE_SIZE);
}


bool
pmap_md_ok_to_steal_p(const uvm_physseg_t bank, size_t npgs)
{
#ifdef needtowrite
	if (uvm_physseg_get_avail_start(bank) + npgs >= atop(MIPS_PHYS_MASK + 1)) {
		aprint_debug("%s: seg not enough in KSEG0 for %zu pages\n",
		    __func__, npgs);
		return false;
	}
#endif

	if (uvm_physseg_get_avail_start(bank) + npgs >= atop(physical_start + 1 * 1024 * 1024 * 1024)) {
		aprint_debug("%s: not enough space in direct map for %zu pages (%lx - %lx)\n",
		    __func__, npgs, uvm_physseg_get_avail_start(bank), uvm_physseg_get_avail_end(bank));
		return false;
	}

	return true;
}

#ifdef PMAP_VIRTUAL_CACHE_ALIASES

bool
pmap_md_vca_add(struct vm_page *pg, vaddr_t va, pt_entry_t *ptep)
{
	UVMHIST_FUNC(__func__); UVMHIST_CALLED(pmaphist);
#ifdef needtowrite
	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);
	if (!MIPS_HAS_R4K_MMU || !MIPS_CACHE_VIRTUAL_ALIAS)
		return false;

	/*
	 * There is at least one other VA mapping this page.
	 * Check if they are cache index compatible.
	 */

	KASSERT(VM_PAGEMD_PVLIST_LOCKED_P(mdpg));
	pv_entry_t pv = &mdpg->mdpg_first;
#if defined(PMAP_NO_PV_UNCACHED)
	/*
	 * Instead of mapping uncached, which some platforms
	 * cannot support, remove incompatible mappings from others pmaps.
	 * When this address is touched again, the uvm will
	 * fault it in.  Because of this, each page will only
	 * be mapped with one index at any given time.
	 *
	 * We need to deal with all entries on the list - if the first is
	 * incompatible with the new mapping then they all will be.
	 */
	if (__predict_true(!mips_cache_badalias(pv->pv_va, va))) {
		return false;
	}
	KASSERT(pv->pv_pmap != NULL);
	bool ret = false;
	for (pv_entry_t npv = pv; npv && npv->pv_pmap;) {
		if (npv->pv_va & PV_KENTER) {
			npv = npv->pv_next;
			continue;
		}
		ret = true;
		vaddr_t nva = trunc_page(npv->pv_va);
		pmap_t npm = npv->pv_pmap;
		VM_PAGEMD_PVLIST_UNLOCK(mdpg);
		pmap_remove(npm, nva, nva + PAGE_SIZE);

		/*
		 * pmap_update is not required here as we're the pmap
		 * and we know that the invalidation happened or the
		 * asid has been released (and activation is deferred)
		 *
		 * A deferred activation should NOT occur here.
		 */
		(void)VM_PAGEMD_PVLIST_LOCK(mdpg);

		npv = pv;
	}
	KASSERT(ret == true);

	return ret;
#else	/* !PMAP_NO_PV_UNCACHED */
	if (VM_PAGEMD_CACHED_P(mdpg)) {
		/*
		 * If this page is cached, then all mappings
		 * have the same cache alias so we only need
		 * to check the first page to see if it's
		 * incompatible with the new mapping.
		 *
		 * If the mappings are incompatible, map this
		 * page as uncached and re-map all the current
		 * mapping as uncached until all pages can
		 * share the same cache index again.
		 */
		if (mips_cache_badalias(pv->pv_va, va)) {
			pmap_page_cache(pg, false);
			pmap_md_vca_page_wbinv(mdpg, true);
			*ptep = pte_cached_change(*ptep, false);
			PMAP_COUNT(page_cache_evictions);
		}
	} else {
		*ptep = pte_cached_change(*ptep, false);
		PMAP_COUNT(page_cache_evictions);
	}
	return false;
#endif	/* !PMAP_NO_PV_UNCACHED */
#endif


	return false;
}

void
pmap_md_vca_clean(struct vm_page_md *mdpg, int op)
{
	UVMHIST_FUNC(__func__); UVMHIST_CALLED(pmaphist);
#ifdef needtowrite
	if (!MIPS_HAS_R4K_MMU || !MIPS_CACHE_VIRTUAL_ALIAS)
		return;

	UVMHIST_LOG(pmaphist, "(pg=%p, op=%d)", pg, op, 0, 0);
	KASSERT(VM_PAGEMD_PVLIST_LOCKED_P(VM_PAGE_TO_MD(pg)));

	if (op == PMAP_WB || op == PMAP_WBINV) {
		pmap_md_vca_page_wbinv(mdpg, true);
	} else if (op == PMAP_INV) {
		KASSERT(op == PMAP_INV && false);
		//mips_dcache_inv_range_index(va, PAGE_SIZE);
	}
#endif
}

/*
 * In the PMAP_NO_PV_CACHED case, all conflicts are resolved at mapping
 * so nothing needs to be done in removal.
 */
void
pmap_md_vca_remove(struct vm_page *pg, vaddr_t va, bool dirty, bool last)
{
#ifdef needtowrite
#if !defined(PMAP_NO_PV_UNCACHED)
	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);
	if (!MIPS_HAS_R4K_MMU
	    || !MIPS_CACHE_VIRTUAL_ALIAS
	    || !VM_PAGEMD_UNCACHED_P(mdpg))
		return;

	KASSERT(kpreempt_disabled());
	KASSERT((va & PAGE_MASK) == 0);

	/*
	 * Page is currently uncached, check if alias mapping has been
	 * removed.  If it was, then reenable caching.
	 */
	(void)VM_PAGEMD_PVLIST_READLOCK(mdpg);
	pv_entry_t pv = &mdpg->mdpg_first;
	pv_entry_t pv0 = pv->pv_next;

	for (; pv0; pv0 = pv0->pv_next) {
		if (mips_cache_badalias(pv->pv_va, pv0->pv_va))
			break;
	}
	if (pv0 == NULL)
		pmap_page_cache(pg, true);
	VM_PAGEMD_PVLIST_UNLOCK(mdpg);
#endif
#endif
}

#endif

pd_entry_t *
pmap_l0table(struct pmap *pm)
{

	return pm->pm_pdetab->pde_pde;
}


#define	L1_BLK_MAPPABLE_P(va, pa, size)					\
    ((((va) | (pa)) & L1_OFFSET) == 0 && (size) >= L1_SIZE)

#define	L2_BLK_MAPPABLE_P(va, pa, size)					\
    ((((va) | (pa)) & L2_OFFSET) == 0 && (size) >= L2_SIZE)


vsize_t
pmap_kenter_range(vaddr_t va, paddr_t pa, vsize_t size,
    vm_prot_t prot, u_int flags)
{
	pt_entry_t attr;
	psize_t blocksize;

	vsize_t resid = round_page(size);
	vsize_t mapped = 0;

	while (resid > 0) {
		if (L1_BLK_MAPPABLE_P(va, pa, resid)) {
			blocksize = L1_SIZE;
			attr = L1_BLOCK;
		} else if (L2_BLK_MAPPABLE_P(va, pa, resid)) {
			blocksize = L2_SIZE;
			attr = L2_BLOCK;
		} else {
			blocksize = L3_SIZE;
			attr = L3_PAGE;
		}

		pt_entry_t pte = pte_make_kenter_pa(pa, NULL, prot, flags);
		pte &= ~LX_TYPE;
		attr |= pte;

		pmapboot_enter(va, pa, blocksize, blocksize, attr, NULL);

		va += blocksize;
		pa += blocksize;
		resid -= blocksize;
		mapped += blocksize;
	}

	return mapped;
}

#ifdef MULTIPROCESSOR
void
pmap_md_tlb_info_attach(struct pmap_tlb_info *ti, struct cpu_info *ci)
{
	/* nothing */
}
#endif /* MULTIPROCESSOR */
