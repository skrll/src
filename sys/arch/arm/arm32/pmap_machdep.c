/*	$NetBSD$	*/

/*
 * Copyright 2003 Wasabi Systems, Inc.
 * All rights reserved.
 *
 * Written by Steve C. Woodford for Wasabi Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed for the NetBSD Project by
 *      Wasabi Systems, Inc.
 * 4. The name of Wasabi Systems, Inc. may not be used to endorse
 *    or promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY WASABI SYSTEMS, INC. ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL WASABI SYSTEMS, INC
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Copyright (c) 2002-2003 Wasabi Systems, Inc.
 * Copyright (c) 2001 Richard Earnshaw
 * Copyright (c) 2001-2002 Christopher Gilbert
 * All rights reserved.
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the company nor the name of the author may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*-
 * Copyright (c) 1999 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Charles M. Hannum.
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
 * Copyright (c) 1994-1998 Mark Brinicombe.
 * Copyright (c) 1994 Brini.
 * All rights reserved.
 *
 * This code is derived from software written for Brini by Mark Brinicombe
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Mark Brinicombe.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *
 * RiscBSD kernel project
 *
 * pmap.c
 *
 * Machine dependent vm stuff
 *
 * Created      : 20/09/94
 */

/*
 * armv6 and VIPT cache support by 3am Software Foundry,
 * Copyright (c) 2007 Microsoft
 */

/*
 * Performance improvements, UVM changes, overhauls and part-rewrites
 * were contributed by Neil A. Carson <neil@causality.com>.
 */

/*
 * Overhauled again to speedup the pmap, use MMU Domains so that L1 tables
 * can be shared, and re-work the KVM layout, by Steve Woodford of Wasabi
 * Systems, Inc.
 *
 * There are still a few things outstanding at this time:
 *
 *   - There are some unresolved issues for MP systems:
 *
 *     o The L1 metadata needs a lock, or more specifically, some places
 *       need to acquire an exclusive lock when modifying L1 translation
 *       table entries.
 *
 *     o When one cpu modifies an L1 entry, and that L1 table is also
 *       being used by another cpu, then the latter will need to be told
 *       that a tlb invalidation may be necessary. (But only if the old
 *       domain number in the L1 entry being over-written is currently
 *       the active domain on that cpu). I guess there are lots more tlb
 *       shootdown issues too...
 *
 *     o If the vector_page is at 0x00000000 instead of in kernel VA space,
 *       then MP systems will lose big-time because of the MMU domain hack.
 *       The only way this can be solved (apart from moving the vector
 *       page to 0xffff0000) is to reserve the first 1MB of user address
 *       space for kernel use only. This would require re-linking all
 *       applications so that the text section starts above this 1MB
 *       boundary.
 *
 *     o Tracking which VM space is resident in the cache/tlb has not yet
 *       been implemented for MP systems.
 *
 *     o Finally, there is a pathological condition where two cpus running
 *       two separate processes (not lwps) which happen to share an L1
 *       can get into a fight over one or more L1 entries. This will result
 *       in a significant slow-down if both processes are in tight loops.
 */

/* Include header files */

//XXXNH PMAP_DEV





#include "opt_arm_debug.h"
#include "opt_cpuoptions.h"
#include "opt_ddb.h"
#include "opt_efi.h"
#include "opt_lockdebug.h"
#include "opt_multiprocessor.h"

#define __PMAP_PRIVATE

#include <sys/param.h>
#include <sys/types.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/intr.h>
#include <sys/pool.h>
#include <sys/kmem.h>
#include <sys/cdefs.h>
#include <sys/cpu.h>
#include <sys/sysctl.h>
#include <sys/bus.h>
#include <sys/atomic.h>
#include <sys/kernhist.h>

#include <uvm/uvm.h>
#include <uvm/pmap/pmap_pvt.h>

#include <arm/cpufunc.h>
#include <arm/arm32/pmap_subr.h>
#include <arm/arm32/machdep.h>

#include <arm/locore.h>

__KERNEL_RCSID(0, "$NetBSD$");

#ifdef VERBOSE_INIT_ARM
#define VPRINTF(...)	printf(__VA_ARGS__)
#else
#define VPRINTF(...)	__nothing
#endif

void	pmap_md_init(void);

static void	pmap_md_vca_page_wbinv(struct vm_page_md *, bool);

/*
 * Misc variables
 */
vaddr_t virtual_avail;
vaddr_t virtual_end;

/*
 * Virtual end of direct-mapped memory
 */
vaddr_t pmap_directlimit;

bool
pmap_extract_coherency(pmap_t pm, vaddr_t va, paddr_t *pap, bool *coherentp)
{
	paddr_t pa;

	if (pm == pmap_kernel()) {
		if (pmap_md_direct_mapped_vaddr_p(va)) {
			pa = pmap_md_direct_mapped_vaddr_to_paddr(va);
			*coherentp = true;
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
	if (ptep == NULL || !pte_valid_p(*ptep)) {
		kpreempt_enable();
		return false;
	}
	// XXXNH assume TRE index 0 is NC and !0 is cached.
	*coherentp = (*ptep & L2_S_CACHE_MASK) ? true : false;

	pa = pte_to_paddr(*ptep) | (va & PGOFSET);
	kpreempt_enable();
done:
	if (pap != NULL) {
		*pap = pa;
	}
	return true;
}

int
pmap_fault_fixup(pmap_t pm, vaddr_t va, vm_prot_t ftype, int user)
{
	int rv = 0;

#ifdef needtowrite
	struct l2_dtable *l2;
	struct l2_bucket *l2b;
	paddr_t pa;
	const size_t l1slot = l1pte_index(va);

	UVMHIST_FUNC(__func__); UVMHIST_CALLED(maphist);

	va = trunc_page(va);

	KASSERT(!user || (pm != pmap_kernel()));

	UVMHIST_LOG(maphist, " (pm=%#jx, va=%#jx, ftype=%#jx, user=%jd)",
	    (uintptr_t)pm, va, ftype, user);
	UVMHIST_LOG(maphist, " ti=%#jx pai=%#jx asid=%#jx",
	    (uintptr_t)cpu_tlb_info(curcpu()),
	    (uintptr_t)PMAP_PAI(pm, cpu_tlb_info(curcpu())),
	    (uintptr_t)PMAP_PAI(pm, cpu_tlb_info(curcpu()))->pai_asid, 0);

	pmap_acquire_pmap_lock(pm);

	/*
	 * If there is no l2_dtable for this address, then the process
	 * has no business accessing it.
	 *
	 * Note: This will catch userland processes trying to access
	 * kernel addresses.
	 */
	l2 = pm->pm_l2[L2_IDX(l1slot)];
	if (l2 == NULL) {
		UVMHIST_LOG(maphist, " no l2 for l1slot %#jx", l1slot, 0, 0, 0);
		goto out;
	}

	/*
	 * Likewise if there is no L2 descriptor table
	 */
	l2b = &l2->l2_bucket[L2_BUCKET(l1slot)];
	if (l2b->l2b_kva == NULL) {
		UVMHIST_LOG(maphist, " <-- done (no ptep for l1slot %#jx)",
		    l1slot, 0, 0, 0);
		goto out;
	}

	/*
	 * Check the PTE itself.
	 */
	pt_entry_t * const ptep = &l2b->l2b_kva[l2pte_index(va)];
	pt_entry_t const opte = *ptep;
	if (opte == 0 || (opte & L2_TYPE_MASK) == L2_TYPE_L) {
		UVMHIST_LOG(maphist, " <-- done (empty pde for l1slot %#jx)",
		    l1slot, 0, 0, 0);
		goto out;
	}

#ifndef ARM_HAS_VBAR
	/*
	 * Catch a userland access to the vector page mapped at 0x0
	 */
	if (user && (opte & L2_S_PROT_U) == 0) {
		UVMHIST_LOG(maphist, " <-- done (vector_page)", 0, 0, 0, 0);
		goto out;
	}
#endif

	pa = l2pte_pa(opte);

	if ((ftype & VM_PROT_WRITE) && !l2pte_writable_p(opte)) {
		/*
		 * This looks like a good candidate for "page modified"
		 * emulation...
		 */
		struct pv_entry *pv;
		struct vm_page *pg;

		/* Extract the physical address of the page */
		if ((pg = PHYS_TO_VM_PAGE(pa)) == NULL) {
			UVMHIST_LOG(maphist, " <-- done (mod/ref unmanaged page)", 0, 0, 0, 0);
			goto out;
		}

		struct vm_page_md *md = VM_PAGE_TO_MD(pg);

		/* Get the current flags for this page. */
		pmap_acquire_page_lock(md);
		pv = pmap_find_pv(md, pm, va);
		if (pv == NULL || PV_IS_KENTRY_P(pv->pv_flags)) {
			pmap_release_page_lock(md);
			UVMHIST_LOG(maphist, " <-- done (mod/ref emul: no PV)", 0, 0, 0, 0);
			goto out;
		}

		/*
		 * Do the flags say this page is writable? If not then it
		 * is a genuine write fault. If yes then the write fault is
		 * our fault as we did not reflect the write access in the
		 * PTE. Now we know a write has occurred we can correct this
		 * and also set the modified bit
		 */
		if ((pv->pv_flags & PVF_WRITE) == 0) {
			pmap_release_page_lock(md);
			goto out;
		}

		md->pvh_attrs |= PVF_REF | PVF_MOD;
		pv->pv_flags |= PVF_REF | PVF_MOD;
		if (md->pvh_attrs & PVF_EXEC) {
			md->pvh_attrs &= ~PVF_EXEC;
			PMAPCOUNT(exec_discarded_modfixup);
		}
		pmap_release_page_lock(md);

		/*
		 * Re-enable write permissions for the page.  No need to call
		 * pmap_vac_me_harder(), since this is just a
		 * modified-emulation fault, and the PVF_WRITE bit isn't
		 * changing. We've already set the cacheable bits based on
		 * the assumption that we can write to this page.
		 */
		const pt_entry_t npte =
		    l2pte_set_writable((opte & ~L2_TYPE_MASK) | L2_S_PROTO)
		    | (pm != pmap_kernel() ? L2_XS_nG : 0)
		    | 0;
		l2pte_reset(ptep);
		PTE_SYNC(ptep);
		pmap_tlb_invalidate_addr(pm, va,
		    (ftype & VM_PROT_EXECUTE) ? PVF_EXEC | PVF_REF : PVF_REF);
		l2pte_set(ptep, npte, 0);
		PTE_SYNC(ptep);
		PMAPCOUNT(fixup_mod);
		rv = 1;
		UVMHIST_LOG(maphist, " <-- done (mod/ref emul: changed pte "
		    "from %#jx to %#jx)", opte, npte, 0, 0);
	} else if ((opte & L2_TYPE_MASK) == L2_TYPE_INV) {
		/*
		 * This looks like a good candidate for "page referenced"
		 * emulation.
		 */
		struct vm_page *pg;

		/* Extract the physical address of the page */
		if ((pg = PHYS_TO_VM_PAGE(pa)) == NULL) {
			UVMHIST_LOG(maphist, " <-- done (ref emul: unmanaged page)", 0, 0, 0, 0);
			goto out;
		}

		struct vm_page_md *md = VM_PAGE_TO_MD(pg);

		/* Get the current flags for this page. */
		pmap_acquire_page_lock(md);
		struct pv_entry *pv = pmap_find_pv(md, pm, va);
		if (pv == NULL || PV_IS_KENTRY_P(pv->pv_flags)) {
			pmap_release_page_lock(md);
			UVMHIST_LOG(maphist, " <-- done (ref emul no PV)", 0, 0, 0, 0);
			goto out;
		}

		md->pvh_attrs |= PVF_REF;
		pv->pv_flags |= PVF_REF;

		pt_entry_t npte =
		    l2pte_set_readonly((opte & ~L2_TYPE_MASK) | L2_S_PROTO);
		if (pm != pmap_kernel()) {
			npte |= L2_XS_nG;
		}
		/*
		 * If we got called from prefetch abort, then ftype will have
		 * VM_PROT_EXECUTE set.  Now see if we have no-execute set in
		 * the PTE.
		 */
		if (user && (ftype & VM_PROT_EXECUTE) && (npte & L2_XS_XN)) {
			/*
			 * Is this a mapping of an executable page?
			 */
			if ((pv->pv_flags & PVF_EXEC) == 0) {
				pmap_release_page_lock(md);
				UVMHIST_LOG(maphist, " <-- done (ref emul: no exec)",
				    0, 0, 0, 0);
				goto out;
			}
			/*
			 * If we haven't synced the page, do so now.
			 */
			if ((md->pvh_attrs & PVF_EXEC) == 0) {
				UVMHIST_LOG(maphist, " ref emul: syncicache "
				    "page #%#jx", pa, 0, 0, 0);
				pmap_syncicache_page(md, pa);
				PMAPCOUNT(fixup_exec);
			}
			npte &= ~L2_XS_XN;
		}
		pmap_release_page_lock(md);
		l2pte_reset(ptep);
		PTE_SYNC(ptep);
		pmap_tlb_invalidate_addr(pm, va,
		    (ftype & VM_PROT_EXECUTE) ? PVF_EXEC | PVF_REF : PVF_REF);
		l2pte_set(ptep, npte, 0);
		PTE_SYNC(ptep);
		PMAPCOUNT(fixup_ref);
		rv = 1;
		UVMHIST_LOG(maphist, " <-- done (ref emul: changed pte from "
		    "%#jx to %#jx)", opte, npte, 0, 0);
	} else if (user && (ftype & VM_PROT_EXECUTE) && (opte & L2_XS_XN)) {
		struct vm_page * const pg = PHYS_TO_VM_PAGE(pa);
		if (pg == NULL) {
			UVMHIST_LOG(maphist, " <-- done (unmanaged page)", 0, 0, 0, 0);
			goto out;
		}

		struct vm_page_md * const md = VM_PAGE_TO_MD(pg);

		/* Get the current flags for this page. */
		pmap_acquire_page_lock(md);
		struct pv_entry * const pv = pmap_find_pv(md, pm, va);
		if (pv == NULL || (pv->pv_flags & PVF_EXEC) == 0) {
			pmap_release_page_lock(md);
			UVMHIST_LOG(maphist, " <-- done (no PV or not EXEC)", 0, 0, 0, 0);
			goto out;
		}

		/*
		 * If we haven't synced the page, do so now.
		 */
		if ((md->pvh_attrs & PVF_EXEC) == 0) {
			UVMHIST_LOG(maphist, "syncicache page #%#jx",
			    pa, 0, 0, 0);
			pmap_syncicache_page(md, pa);
		}
		pmap_release_page_lock(md);
		/*
		 * Turn off no-execute.
		 */
		KASSERT(opte & L2_XS_nG);
		l2pte_reset(ptep);
		PTE_SYNC(ptep);
		pmap_tlb_invalidate_addr(pm, va, PVF_EXEC | PVF_REF);
		l2pte_set(ptep, opte & ~L2_XS_XN, 0);
		PTE_SYNC(ptep);
		rv = 1;
		PMAPCOUNT(fixup_exec);
		UVMHIST_LOG(maphist, "exec: changed pte from %#jx to %#jx",
		    opte, opte & ~L2_XS_XN, 0, 0);
	}


#ifndef MULTIPROCESSOR
	/*
	 * If 'rv == 0' at this point, it generally indicates that there is a
	 * stale TLB entry for the faulting address. This happens when two or
	 * more processes are sharing an L1. Since we don't flush the TLB on
	 * a context switch between such processes, we can take domain faults
	 * for mappings which exist at the same VA in both processes. EVEN IF
	 * WE'VE RECENTLY FIXED UP THE CORRESPONDING L1 in pmap_enter(), for
	 * example.
	 *
	 * This is extremely likely to happen if pmap_enter() updated the L1
	 * entry for a recently entered mapping. In this case, the TLB is
	 * flushed for the new mapping, but there may still be TLB entries for
	 * other mappings belonging to other processes in the 1MB range
	 * covered by the L1 entry.
	 *
	 * Since 'rv == 0', we know that the L1 already contains the correct
	 * value, so the fault must be due to a stale TLB entry.
	 *
	 * Since we always need to flush the TLB anyway in the case where we
	 * fixed up the L1, or frobbed the L2 PTE, we effectively deal with
	 * stale TLB entries dynamically.
	 *
	 * However, the above condition can ONLY happen if the current L1 is
	 * being shared. If it happens when the L1 is unshared, it indicates
	 * that other parts of the pmap are not doing their job WRT managing
	 * the TLB.
	 */
	if (rv == 0
	    && true) {
#ifdef DEBUG
		extern int last_fault_code;
#else
		int last_fault_code = ftype & VM_PROT_EXECUTE
		    ? armreg_ifsr_read()
		    : armreg_dfsr_read();
#endif
		printf("fixup: pm %p, va 0x%lx, ftype %d - nothing to do!\n",
		    pm, va, ftype);
		printf("fixup: l2 %p, l2b %p, ptep %p, pte %#x\n",
		    l2, l2b, ptep, opte);

		printf("fixup: pdep %p, pde %#x, ttbcr %#x\n",
		    &pmap_l1_kva(pm)[l1slot], pmap_l1_kva(pm)[l1slot],
		   armreg_ttbcr_read());
		printf("fixup: fsr %#x cpm %p casid %#x contextidr %#x dacr %#x\n",
		    last_fault_code, curcpu()->ci_pmap_cur,
		    curcpu()->ci_pmap_asid_cur,
		    armreg_contextidr_read(), armreg_dacr_read());
#ifdef _ARM_ARCH_7
		if (ftype & VM_PROT_WRITE)
			armreg_ats1cuw_write(va);
		else
			armreg_ats1cur_write(va);
		isb();
		printf("fixup: par %#x\n", armreg_par_read());
#endif
#ifdef DDB
		extern int kernel_debug;

		if (kernel_debug & 2) {
			pmap_release_pmap_lock(pm);
#ifdef UVMHIST
			KERNHIST_DUMP(maphist);
#endif
			cpu_Debugger();
			pmap_acquire_pmap_lock(pm);
		}
#endif
	}
#endif

	rv = 1;

out:
	pmap_release_pmap_lock(pm);
#endif

	return rv;
}

static int		pmap_l1tt_ctor(void *, void *, int);

static struct pool_cache pmap_l1tt_cache;

static void *pmap_l1tt_alloc(struct pool *, int);
static void pmap_l1tt_free(struct pool *, void *);

static struct pool_allocator pmap_l1tt_allocator = {
	.pa_alloc = pmap_l1tt_alloc,
	.pa_free = pmap_l1tt_free,
	.pa_pagesz = L1TT_SIZE,
};


int
pmap_maxproc_set(int nmaxproc)
{
	static const char pmap_l1ttpool_warnmsg[] =
	    "WARNING: l1ttpool limit reached; increase kern.maxproc";

	pool_cache_prime(&pmap_l1tt_cache, nmaxproc);

	/*
	 * Set the hard limit on the pmap_l1tt_cache to the number
	 * of processes the kernel is to support.  Log the limit
	 * reached message max once a minute.
	 */
	pool_cache_sethardlimit(&pmap_l1tt_cache, nmaxproc,
	    pmap_l1ttpool_warnmsg, 60);

	return 0;
}

void
pmap_md_alloc_pdp(pmap_t pm)
{

	vaddr_t va = (vaddr_t)pool_cache_get_paddr(&pmap_l1tt_cache, PR_WAITOK,
	    &pm->pm_l1_pa);

	pm->pm_l1 = (pd_entry_t *)va;
	PTE_SYNC_RANGE(pm->pm_l1, L1TT_SIZE / sizeof(pt_entry_t));

}



void
pmap_md_free_pdp(pmap_t pm)
{
	pool_cache_put_paddr(&pmap_l1tt_cache, (void *)pm->pm_l1, pm->pm_l1_pa);

	pm->pm_l1 = NULL;
	pm->pm_l1_pa = 0;
}

static void *
pmap_l1tt_alloc(struct pool *pp, int flags)
{
	struct pglist plist;
	vaddr_t va;

	const int waitok = flags & PR_WAITOK;

	int error = uvm_pglistalloc(L1TT_SIZE, 0, -1, L1TT_SIZE, 0, &plist, 1,
	    waitok);
	if (error)
		panic("Cannot allocate L1TT physical pages, %d", error);

	struct vm_page *pg = TAILQ_FIRST(&plist);
#if !defined( __HAVE_MM_MD_DIRECT_MAPPED_PHYS)

	/* Allocate a L1 translation table VA */
	va = uvm_km_alloc(kernel_map, L1TT_SIZE, L1TT_SIZE, UVM_KMF_VAONLY);
	if (va == 0)
		panic("Cannot allocate L1TT KVA");

	const vaddr_t eva = va + L1TT_SIZE;
	vaddr_t mva = va;
	while (pg && mva < eva) {
		paddr_t pa = VM_PAGE_TO_PHYS(pg);

		pmap_kenter_pa(mva, pa,
		    VM_PROT_READ|VM_PROT_WRITE, PMAP_KMPAGE|PMAP_PTE);

		mva += PAGE_SIZE;
		pg = TAILQ_NEXT(pg, pageq.queue);
	}
	KASSERTMSG(pg == NULL && mva == eva, "pg %p mva %" PRIxVADDR
	    " eva %" PRIxVADDR, pg, mva, eva);
#else
	bool ok;
	paddr_t pa = VM_PAGE_TO_PHYS(pg);
	va = pmap_direct_mapped_phys(pa, &ok, 0);
	KASSERT(ok);
	KASSERT(va >= KERNEL_BASE);
#endif

	return (void *)va;
}

static void
pmap_l1tt_free(struct pool *pp, void *v)
{
	vaddr_t va = (vaddr_t)v;

#if !defined( __HAVE_MM_MD_DIRECT_MAPPED_PHYS)
	uvm_km_free(kernel_map, va, L1TT_SIZE, UVM_KMF_WIRED);
#else
#if defined(KERNEL_BASE_VOFFSET)
	paddr_t pa = va - KERNEL_BASE_VOFFSET;
#else
	paddr_t pa = va - KERNEL_BASE + physical_start;
#endif
	const paddr_t epa = pa + L1TT_SIZE;

	for (; pa < epa; pa += PAGE_SIZE) {
		struct vm_page *pg = PHYS_TO_VM_PAGE(pa);
		uvm_pagefree(pg);
	}
#endif
}

/*
 * Pool cache constructors for L1 translation tables
 */

static int
pmap_l1tt_ctor(void *arg, void *v, int flags)
{
#ifndef PMAP_INCLUDE_PTE_SYNC
#error not supported
#endif

	memset(v, 0, L1TT_SIZE);
	PTE_SYNC_RANGE(v, L1TT_SIZE / sizeof(pt_entry_t));
	return 0;
}

#ifdef __HAVE_MM_MD_DIRECT_MAPPED_PHYS
vaddr_t
pmap_direct_mapped_phys(paddr_t pa, bool *ok_p, vaddr_t va)
{
	bool ok = false;
	if (physical_start <= pa && pa < physical_end) {
		const vaddr_t newva = pa - physical_start + KERNEL_DIRECTMAP_BASE;
		if (newva >= KERNEL_DIRECTMAP_BASE && newva < pmap_directlimit) {
			va = newva;
			ok = true;
		}
	}
	KASSERT(ok_p);
	*ok_p = ok;
	return va;
}
#endif

struct vm_page *
pmap_md_alloc_poolpage(int flags)
{
	/*
	 * We must make sure that we only allocate pages that can be mapped
	 * via the direct map KVA area.
	 */
	if (arm_poolpage_vmfreelist != VM_FREELIST_DEFAULT)
		return uvm_pagealloc_strat(NULL, 0, NULL, flags,
		    UVM_PGA_STRAT_ONLY, arm_poolpage_vmfreelist);

	return uvm_pagealloc(NULL, 0, NULL, flags);
}

vaddr_t
pmap_md_map_poolpage(paddr_t pa, size_t len)
{

	struct vm_page * const pg = PHYS_TO_VM_PAGE(pa);
	vaddr_t va = pmap_md_pool_phystov(pa);
	KASSERT(cold || pg != NULL);
	if (pg != NULL) {
		struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);
		pv_entry_t pv = &mdpg->mdpg_first;
		vaddr_t last_va = trunc_page(pv->pv_va);

		KASSERT(len == PAGE_SIZE || last_va == pa);
		KASSERT(pv->pv_pmap == NULL);
		KASSERT(pv->pv_next == NULL);
		KASSERT(!VM_PAGEMD_EXECPAGE_P(mdpg));

#ifdef needtowrite
		/*
		 * If this page was last mapped with an address that
		 * might cause aliases, flush the page from the cache.
		 */
		if (MIPS_CACHE_VIRTUAL_ALIAS
		    && mips_cache_badalias(last_va, va)) {
			pmap_md_vca_page_wbinv(mdpg, false);
		}
#endif
		if (0 /* bad alias */)
			pmap_md_vca_page_wbinv(mdpg, false);

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

	KASSERT(VM_PAGEMD_CACHED_P(mdpg));
	KASSERT(!VM_PAGEMD_EXECPAGE_P(mdpg));

	pv_entry_t pv = &mdpg->mdpg_first;

	/* Note last mapped address for future color check */
	pv->pv_va = va;

	KASSERT(pv->pv_pmap == NULL);
	KASSERT(pv->pv_next == NULL);

	return pa;
}


extern size_t kernel_size;
bool
pmap_md_kernel_vaddr_p(vaddr_t va)
{
	if (va >= KERNEL_BASE && va < KERNEL_BASE + kernel_size) {
		return true;
	}

	return false;
}

paddr_t
pmap_md_kernel_vaddr_to_paddr(vaddr_t va)
{

	if (va >= KERNEL_BASE && va < KERNEL_BASE + kernel_size) {

		return KERN_VTOPHYS(va);
	}
	panic("%s: va %#" PRIxVADDR " not direct mapped!", __func__, va);

}


bool
pmap_md_direct_mapped_vaddr_p(vaddr_t va)
{
	if (va >= KERNEL_DIRECTMAP_BASE && va < pmap_directlimit) {
		return true;
	}

	return false;
}

paddr_t
pmap_md_direct_mapped_vaddr_to_paddr(vaddr_t va)
{

	if (va >= KERNEL_DIRECTMAP_BASE && va < pmap_directlimit) {

		return va - KERNEL_DIRECTMAP_BASE + physical_start;
	}
	panic("%s: va %#" PRIxVADDR " not direct mapped!", __func__, va);

}


bool
pmap_md_io_vaddr_p(vaddr_t va)
{

	if (pmap_devmap_find_va(va, PAGE_SIZE)) {
		return true;
	}
	return false;
}


paddr_t
pmap_md_pool_vtophys(vaddr_t va)
{

	KASSERT(va >= KERNEL_DIRECTMAP_BASE && va < pmap_directlimit);

	return va - KERNEL_DIRECTMAP_BASE + physical_start;
}

vaddr_t
pmap_md_pool_phystov(paddr_t pa)
{

	return (pa - physical_start) + KERNEL_DIRECTMAP_BASE;
}

void
pmap_impl_bootstrap(void)
{
	KASSERT(pte_l1_s_cache_mode == pte_l1_s_cache_mode_pt);
	KASSERT(pte_l2_s_cache_mode == pte_l2_s_cache_mode_pt);

	pmap_t pm = pmap_kernel();

	pm->pm_pdetab = (pmap_pdetab_t *)kernel_l1pt.pv_va;
	pm->pm_l1 = (pd_entry_t *)kernel_l1pt.pv_va;
	pm->pm_l1_pa = kernel_l1pt.pv_pa;

#if defined(EFI_RUNTIME)

	pmap_t efipm = pmap_efirt();

	VPRINTF("efirt ");
	memset(efipm, 0, sizeof(*efipm));
	struct pmap_asid_info * const efipai = PMAP_PAI(efipm, cpu_tlb_info(curcpu()));

	efipai->pai_asid = KERNEL_PID;
//	efipm->pm_refs = 1;
//	efipm->pm_stats.wired_count = 0;
//	efipm->pm_stats.resident_count = 1;
	efipm->pm_pdetab = (pmap_pdetab_t *)efirt_l1pt.pv_va;
	efipm->pm_l1 = (pd_entry_t *)efirt_l1pt.pv_va;
	efipm->pm_l1_pa = efirt_l1pt.pv_pa;
#endif

	VPRINTF("common ");
	pmap_bootstrap_common();

	VPRINTF("tlb0 ");
	pmap_tlb_info_init(&pmap_tlb0_info);

#ifdef MULTIPROCESSOR
	VPRINTF("kcpusets ");
	pm->pm_onproc = kcpuset_running;
	pm->pm_active = kcpuset_running;
#endif

	/*
	 * Initialize `FYI' variables.	Note we're relying on
	 * the fact that BSEARCH sorts the vm_physmem[] array
	 * for us.  Must do this before uvm_pageboot_alloc()
	 * can be called.
	 */
	pmap_limits.avail_start = ptoa(uvm_physseg_get_start(uvm_physseg_get_first()));
	pmap_limits.avail_end = ptoa(uvm_physseg_get_end(uvm_physseg_get_last()));

//	pmap_limits.virtual_end = pmap_limits.virtual_start + (vaddr_t)sysmap_size * NBPG;


//	pmap_pvlist_lock_init(arm_dcache_align);
}

void
pmap_impl_bootstrap_l1(void)
{
}

void
pmap_impl_set_virtual_space(vaddr_t vs, vaddr_t ve)
{

	pmap_limits.virtual_start = vs;
	pmap_limits.virtual_end = ve;
}

void
pmap_impl_bootstrap_pools(void)
{

	/*
	 * Initialize the pools.
	 */
	pool_init(&pmap_pmap_pool, PMAP_SIZE, 0, 0, 0, "pmappl",
	    &pool_allocator_nointr, IPL_NONE);
	pool_init(&pmap_pv_pool, sizeof(struct pv_entry), 0, 0, 0, "pvpl",
	    &pmap_pv_page_allocator, IPL_NONE);

	pmap_pvlist_lock_init(arm_dcache_align);
}


#if defined(EFI_RUNTIME)
void
pmap_md_activate_efirt(void)
{
	kpreempt_disable();

//      struct cpu_info * const ci = curcpu();
	struct pmap * const pm = pmap_efirt();
	struct pmap_asid_info * const pai = PMAP_PAI(pm, cpu_tlb_info(ci));

	pmap_md_asid_activate(pai->pai_asid, pmap_efirt(), NULL);
}

void
pmap_md_deactivate_efirt(void)
{
	pmap_md_asid_deactivate(pmap_efirt());

	KASSERT(
	    (armreg_pfr1_read() & ARM_PFR1_SEC_MASK) == 0 ||
	    (armreg_ttbcr_read() & TTBCR_S_PD0) != 0
	);

	kpreempt_enable();
}
#endif

#if defined(MULTIPROCESSOR)
void
pmap_md_tlb_info_attach(struct pmap_tlb_info *ti, struct cpu_info *ci)
{
	/* nothing */
}
#endif

void
pmap_impl_postinit(void)
{
}

uint32_t
pmap_kernel_L1_addr(void)
{

 	return pmap_kernel()->pm_l1_pa;
}

void
pmap_md_init(void)
{

	/*
	 * Initialise the L1 pool and cache.
	 */

	pool_cache_bootstrap(&pmap_l1tt_cache, L1TT_SIZE, L1TT_SIZE,
	    0, 0, "l1ttpl", &pmap_l1tt_allocator, IPL_NONE, pmap_l1tt_ctor,
	     NULL, NULL);

	int error __diagused = pmap_maxproc_set(maxproc);
	KASSERT(error == 0);

//	pmap_tlb_info_evcnt_attach(&pmap_tlb0_info);
}

void
pmap_md_pdetab_init(struct pmap *pm)
{
	KASSERT(pm != NULL);

	pm->pm_l1 = (pd_entry_t *)pm->pm_pdetab;
	pmap_extract(pmap_kernel(), (vaddr_t)pm->pm_l1, &pm->pm_l1_pa);
	PTE_SYNC_RANGE(pm->pm_l1, PMAP_PDETABSIZE);

	/*
	 * Note: The pool cache ensures that the pm_l2[] array is already
	 * initialised to zero.
	 */

//	pmap->pm_pdetab = pmap_md_alloc_pdp(pmap, &pmap->pm_pdetab);

	/* for (int i = 0; i < NPDEPG; ++i) { */
	/* 	pmap->pm_pdetab[i] = pmap_kernel()->pm_pdetab[i]; */
	/* } */

//	pmap->pm_md.md_ptbr =
//	    pmap_md_direct_mapped_vaddr_to_paddr((vaddr_t)pmap->pm_pdetab) >> PAGE_SHIFT;
}


void
pmap_md_pdetab_fini(struct pmap *pm)
{
	KASSERT(pm != NULL);

}


#if comment



// Common?
pmap_fault_fixup
pmap_get_pde_pte
arm32_mmap_flags

// Here
pmap_md_page_syncicache
#endif


//XXXNH see "common" comment above
u_int
arm32_mmap_flags(paddr_t pa)
{
	/*
	 * the upper 8 bits in pmap_enter()'s flags are reserved for MD stuff
	 * and we're using the upper bits in page numbers to pass flags around
	 * so we might as well use the same bits
	 */
	return (u_int)pa & PMAP_MD_MASK;
}



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

vaddr_t
pmap_md_direct_map_paddr(paddr_t pa)
{
	return POOL_PHYSTOV(pa);
}

void
pmap_md_page_syncicache(struct vm_page_md *mdpg, const kcpuset_t *onproc)
{
	UVMHIST_FUNC(__func__); UVMHIST_CALLED(pmaphist);
#ifdef needtowrite
	struct mips_options * const opts = &mips_options;
	if (opts->mips_cpu_flags & CPU_MIPS_I_D_CACHE_COHERENT)
		return;

	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);

	/*
	 * If onproc is empty, we could do a
	 * pmap_page_protect(pg, VM_PROT_NONE) and remove all
	 * mappings of the page and clear its execness.  Then
	 * the next time page is faulted, it will get icache
	 * synched.  But this is easier. :)
	 */
	if (MIPS_HAS_R4K_MMU) {
		if (VM_PAGEMD_CACHED_P(mdpg)) {
			/* This was probably mapped cached by UBC so flush it */
			pt_entry_t pte;
			const register_t tva = pmap_md_map_ephemeral_page(mdpg, false,
			    VM_PROT_READ, &pte);

			UVMHIST_LOG(pmaphist, "  va %#"PRIxVADDR, tva, 0, 0, 0);
			mips_dcache_wbinv_range(tva, PAGE_SIZE);
			mips_icache_sync_range(tva, PAGE_SIZE);

			pmap_md_unmap_ephemeral_page(mdpg, false, tva, pte);
		}
	} else {
		mips_icache_sync_range(MIPS_PHYS_TO_KSEG0(VM_PAGE_TO_PHYS(pg)),
		    PAGE_SIZE);
	}
#ifdef MULTIPROCESSOR
	pv_entry_t pv = &mdpg->mdpg_first;
	const register_t va = (intptr_t)trunc_page(pv->pv_va);
	pmap_tlb_syncicache(va, onproc);
#endif
#endif
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


bool
pmap_md_vca_add(struct vm_page_md *mdpg, vaddr_t va, pt_entry_t *ptep)
{
	UVMHIST_FUNC(__func__); UVMHIST_CALLED(pmaphist);
#ifdef needtowrite
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
			pmap_page_cache(mdpg, false);
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
		pmap_page_cache(mdpg, true);
	VM_PAGEMD_PVLIST_UNLOCK(mdpg);
#endif
#endif
}
