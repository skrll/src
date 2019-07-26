/*	$NetBSD: pmap_segtab.c,v 1.7 2019/03/08 08:12:40 msaitoh Exp $	*/

/*-
 * Copyright (c) 1998, 2001 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Jason R. Thorpe of the Numerical Aerospace Simulation Facility,
 * NASA Ames Research Center and by Chris G. Demetriou.
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
 * Copyright (c) 1992, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * the Systems Programming Group of the University of Utah Computer
 * Science Department and Ralph Campbell.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)pmap.c	8.4 (Berkeley) 1/26/94
 */

#include <sys/cdefs.h>

__KERNEL_RCSID(0, "$NetBSD: pmap_segtab.c,v 1.7 2019/03/08 08:12:40 msaitoh Exp $");

/*
 *	Manages physical address maps.
 *
 *	In addition to hardware address maps, this
 *	module is called upon to provide software-use-only
 *	maps which may or may not be stored in the same
 *	form as hardware maps.  These pseudo-maps are
 *	used to store intermediate results from copy
 *	operations to and from address spaces.
 *
 *	Since the information managed by this module is
 *	also stored by the logical address mapping module,
 *	this module may throw away valid virtual-to-physical
 *	mappings at almost any time.  However, invalidations
 *	of virtual-to-physical mappings must be done as
 *	requested.
 *
 *	In order to cope with hardware architectures which
 *	make virtual-to-physical map invalidates expensive,
 *	this module may delay invalidate or reduced protection
 *	operations until such time as they are actually
 *	necessary.  This module is given full information as
 *	to which processors are currently using which maps,
 *	and to when physical maps must be made correct.
 */

#define __PMAP_PRIVATE

#include "opt_multiprocessor.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/mutex.h>
#include <sys/atomic.h>

#include <uvm/uvm.h>
#include <uvm/pmap/pmap.h>

#if defined(XSEGSHIFT) && XSEGSHIFT == SEGSHIFT
#undef XSEGSHIFT
#undef XSEGLENGTH
#undef NBXSEG
#undef NXSEGPG
#endif

#define MULT_CTASSERT(a,b)	__CTASSERT((a) < (b) || ((a) % (b) == 0))

__CTASSERT(sizeof(pmap_ptpage_t) == NBPG);

#if defined(PMAP_HWPAGEWALKER)
#ifdef _LP64
MULT_CTASSERT(PMAP_PDETABSIZE, NPDEPG);
MULT_CTASSERT(NPDEPG, PMAP_PDETABSIZE);
MULT_CTASSERT(PMAP_PDETABSIZE, NPDEPG);
#endif /* _LP64 */
MULT_CTASSERT(sizeof(pmap_pdetab_t *), sizeof(pd_entry_t));
MULT_CTASSERT(sizeof(pd_entry_t), sizeof(pmap_pdetab_t));

#if 0
#ifdef _LP64
static const bool separate_pdetab_root_p = NPDEPG != PMAP_PDETABSIZE;
#else
static const bool separate_pdetab_root_p = true;
#endif /* _LP64 */
#endif

typedef struct {
	pmap_pdetab_t *free_pdetab0;	/* free list kept locally */
	pmap_pdetab_t *free_pdetab;	/* free list kept locally */
#ifdef DEBUG
	uint32_t nget;
	uint32_t nput;
	uint32_t npage;
#define	PDETAB_ADD(n, v)	(pmap_segtab_info.pdealloc.n += (v))
#else
#define	PDETAB_ADD(n, v)	((void) 0)
#endif /* DEBUG */
} pmap_pdetab_alloc_t;
#endif /* PMAP_HWPAGEWALKER */

#if !defined(PMAP_HWPAGEWALKER) || !defined(PMAP_MAP_POOLPAGE)
#ifdef _LP64
__CTASSERT(NSEGPG >= PMAP_SEGTABSIZE);
__CTASSERT(NSEGPG % PMAP_SEGTABSIZE == 0);
#endif
__CTASSERT(NBPG >= sizeof(pmap_segtab_t));

typedef struct  {
	pmap_segtab_t *free_segtab0;	/* free list kept locally */
	pmap_segtab_t *free_segtab;	/* free list kept locally */
#ifdef DEBUG
	uint32_t nget;
	uint32_t nput;
	uint32_t npage;
#define	SEGTAB_ADD(n, v)	(pmap_segtab_info.segalloc.n += (v))
#else
#define	SEGTAB_ADD(n, v)	((void) 0)
#endif
} pmap_segtab_alloc_t;
#endif /* !PMAP_HWPAGEWALKER || !PMAP_MAP_POOLPAGE */

struct pmap_segtab_info {
#if defined(PMAP_HWPAGEWALKER)
	pmap_pdetab_alloc_t pdealloc;
#endif
#if !defined(PMAP_HWPAGEWALKER) || !defined(PMAP_MAP_POOLPAGE)
	pmap_segtab_alloc_t segalloc;
#endif
#ifdef PMAP_PTP_CACHE
	struct pgflist ptp_pgflist;	/* Keep a list of idle page tables. */
#endif
} pmap_segtab_info = {
#ifdef PMAP_PTP_CACHE
	.ptp_pgflist = LIST_HEAD_INITIALIZER(pmap_segtab_info.ptp_pgflist),
#endif
};

kmutex_t pmap_segtab_lock __cacheline_aligned;

#ifndef PMAP_HWPAGEWALKER
static void
pmap_check_stp(pmap_segtab_t *stp, const char *caller, const char *why)
{
#ifdef DEBUG
	for (size_t i = 0; i < PMAP_SEGTABSIZE; i++) {
		if (stp->seg_tab[i] != 0) {
#ifdef DEBUG_NOISY
			for (size_t j = i; j < PMAP_SEGTABSIZE; j++)
				printf("%s: pm_segtab.seg_tab[%zu] = 0x%p\n",
				       caller, j, stp->seg_tab[j]);
#endif
			panic("%s: pm_segtab.seg_tab[%zu] != 0 (0x%p): %s",
			      caller, i, stp->seg_tab[i], why);
		}
	}
#endif
}
#endif /* PMAP_HWPAGEWALKER */

static inline struct vm_page *
pmap_pte_pagealloc(void)
{
	struct vm_page *pg;

	pg = PMAP_ALLOC_POOLPAGE(UVM_PGA_ZERO|UVM_PGA_USERESERVE);
	if (pg) {
#ifdef UVM_PAGE_TRKOWN
		pg->owner_tag = NULL;
#endif
		UVM_PAGE_OWN(pg, "pmap-ptp");
	}

	return pg;
}

#if defined(PMAP_HWPAGEWALKER) && defined(PMAP_MAP_POOLPAGE)
static vaddr_t
pmap_pde_to_va(pd_entry_t pde)
{
	if (!pte_pde_valid_p(pde))
		return 0;

	paddr_t pa = pte_pde_to_paddr(pde);
	return pmap_md_direct_map_paddr(pa);
}

#ifdef _LP64
static pmap_pdetab_t *
pmap_pde_to_pdetab(pd_entry_t pde)
{
	return (pmap_pdetab_t *) pmap_pde_to_va(pde);
}
#endif

static pmap_ptpage_t *
pmap_pde_to_ptpage(pd_entry_t pde)
{
	return (pmap_ptpage_t *) pmap_pde_to_va(pde);
}
#endif

#ifdef _LP64
__CTASSERT((XSEGSHIFT - SEGSHIFT) % (PGSHIFT-3) == 0);
#endif

static inline pmap_ptpage_t *
pmap_ptpage(struct pmap *pmap, vaddr_t va)
{
#if defined(PMAP_HWPAGEWALKER) && defined(PMAP_MAP_POOLPAGE)
	vaddr_t pdetab_mask = PMAP_PDETABSIZE - 1;
	pmap_pdetab_t *ptb = pmap->pm_pdetab;

	KASSERT(pmap != pmap_kernel() || !pmap_md_direct_mapped_vaddr_p(va));

#ifdef _LP64
	for (size_t segshift = XSEGSHIFT;
	     segshift > SEGSHIFT;
	     segshift -= PGSHIFT - 3, pdetab_mask = NSEGPG - 1) {
		ptb = pmap_pde_to_pdetab(ptb->pde_pde[(va >> segshift) & pdetab_mask]);
		if (ptb == NULL)
			return NULL;
	}
#endif
	return pmap_pde_to_ptpage(ptb->pde_pde[(va >> SEGSHIFT) & pdetab_mask]);
#else
	vaddr_t segtab_mask = PMAP_SEGTABSIZE - 1;
	pmap_segtab_t *stb = pmap->pm_segtab;

	KASSERT(pmap != pmap_kernel() || !pmap_md_direct_mapped_vaddr_p(va));

#ifdef _LP64
	for (size_t segshift = XSEGSHIFT;
	     segshift > SEGSHIFT;
	     segshift -= PGSHIFT - 3, segtab_mask = NSEGPG - 1) {
		stb = stb->seg_seg[(va >> segshift) & segtab_mask];
		if (stb == NULL)
			return NULL;
	}
#endif
	return stb->seg_tab[(va >> SEGSHIFT) & segtab_mask];
#endif
}

#if defined(PMAP_HWPAGEWALKER)
bool
pmap_pdetab_fixup(struct pmap *pmap, vaddr_t va)
{
	struct pmap * const kpm = pmap_kernel();
	pmap_pdetab_t * const kptb = kpm->pm_pdetab;
	pmap_pdetab_t * const uptb = pmap->pm_pdetab;
	size_t idx = PMAP_PDETABSIZE - 1;
#if !defined(PMAP_MAP_POOLPAGE)
	__CTASSERT(PMAP_PDETABSIZE == PMAP_SEGTABSIZE);
	pmap_segtab_t * const kstb = &pmap_kern_segtab;
	pmap_segtab_t * const ustb = pmap->pm_segtab;
#endif

	// Regardless of how many levels deep this page table is, we only
	// need to verify the first level PDEs match up.
#ifdef XSEGSHIFT
	idx &= va >> XSEGSHIFT;
#else
	idx &= va >> SEGSHIFT;
#endif
	if (uptb->pde_pde[idx] != kptb->pde_pde[idx]) {
		pte_pde_set(&uptb->pde_pde[idx], kptb->pde_pde[idx]);
#if !defined(PMAP_MAP_POOLPAGE)
		ustb->seg_seg[idx] = kstb->seg_seg[idx]; // copy KVA of PTP
#endif
		return true;
	}
	return false;
}
#endif /* PMAP_HWPAGEWALKER */


static void
pmap_page_attach(pmap_t pmap, vaddr_t kva, struct vm_page *pg,
	struct pglist *pglist, voff_t off)
{
	struct uvm_object * const uobj = &pmap->pm_uobject;
	if (pg == NULL) {
		paddr_t pa;

		bool ok __diagused = pmap_extract(pmap_kernel(), kva, &pa);
		KASSERT(ok);

		pg = PHYS_TO_VM_PAGE(pa);
		KASSERT(pg != NULL);
	}

	mutex_spin_enter(uobj->vmobjlock);
	TAILQ_INSERT_TAIL(pglist, pg, listq.queue);
	uobj->uo_npages++;
	mutex_spin_exit(uobj->vmobjlock);

	/*
	 * Now set each vm_page that maps this page to point to the
	 * pmap and set the offset to what we want.
	 */
	KASSERT(pg->uobject == NULL);
	pg->uobject = uobj;
	pg->offset = off;
}

static struct vm_page *
pmap_segtab_pageclean(pmap_t pmap, struct pglist *list, vaddr_t va)
{
	struct uvm_object * const uobj = &pmap->pm_uobject;

	paddr_t pa;

	bool ok __diagused = pmap_extract(pmap_kernel(), va, &pa);
	KASSERT(ok);

//	const paddr_t pa = kvtophys(va);
	struct vm_page * const pg = PHYS_TO_VM_PAGE(pa);

	KASSERT(pg->uobject == uobj);

	mutex_spin_enter(uobj->vmobjlock);
	TAILQ_REMOVE(list, pg, listq.queue);
	uobj->uo_npages--;
	mutex_spin_exit(uobj->vmobjlock);

	pg->uobject = NULL;
	pg->offset = 0;

	return pg;
}

#ifndef PMAP_PTP_CACHE
static void
pmap_segtab_pagefree(pmap_t pmap, struct pglist *list, vaddr_t kva, size_t size)
{
#ifdef POOL_VTOPHYS
	if (size == PAGE_SIZE) {
		uvm_pagefree(pmap_segtab_pageclean(pmap, list, kva));
		return;
	}
#endif
	for (size_t i = 0; i < size; i += PAGE_SIZE) {
		(void)pmap_segtab_pageclean(pmap, list, kva + i);
	}

	uvm_km_free(kernel_map, kva, size, UVM_KMF_WIRED);
}
#endif

#if 0
#ifndef PMAP_HWPAGEWALKER
static inline pt_entry_t *
pmap_segmap(struct pmap *pmap, vaddr_t va)
{
	pmap_segtab_t *stp = pmap->pm_segtab;
	KASSERTMSG(pmap != pmap_kernel() || !pmap_md_direct_mapped_vaddr_p(va),
	    "pmap %p va %#" PRIxVADDR, pmap, va);
#ifdef _LP64
	stp = stp->seg_seg[(va >> XSEGSHIFT) & (NSEGPG - 1)];
	if (stp == NULL)
		return NULL;
#endif

	return stp->seg_tab[(va >> SEGSHIFT) & (PMAP_SEGTABSIZE - 1)];
}
#endif /* PMAP_HWPAGEWALKER */
#endif

pt_entry_t *
pmap_pte_lookup(pmap_t pmap, vaddr_t va)
{
	pmap_ptpage_t * const ptp = pmap_ptpage(pmap, va);
	if (ptp == NULL)
		return NULL;

	const size_t pte_idx = pte_index(va);

	return ptp->ptp_ptes + pte_idx;
}

#ifdef _LP64
#if defined(PMAP_HWPAGEWALKER)
static void
pmap_pdetab_free(pmap_pdetab_t *ptb)
{
	/*
	 * Insert the pdetab into the pdetab freelist.
	 */
	mutex_spin_enter(&pmap_segtab_lock);
	ptb->pde_next = pmap_segtab_info.pdealloc.free_pdetab;
	pmap_segtab_info.pdealloc.free_pdetab = ptb;
	PDETAB_ADD(nput, 1);
	mutex_spin_exit(&pmap_segtab_lock);

}
#endif
#endif


static pmap_ptpage_t *
pmap_ptpage_alloc(pmap_t pmap, int flags, paddr_t *pa_p)
{
#ifdef PMAP_MAP_POOLPAGE
	struct vm_page *pg = NULL;
	pmap_ptpage_t *ptp = NULL;
	paddr_t pa;
#ifdef PMAP_PTP_CACHE
	ptp = pmap_pgcache_alloc(&pmap_segtab_info.ptp_flist);
#endif
	if (ptp == NULL) {
		pg = pmap_pte_pagealloc();
		if (pg == NULL) {
			if (flags & PMAP_CANFAIL)
				return NULL;
			panic("%s: cannot allocate page table page ",
			    __func__);
		}
		pa = VM_PAGE_TO_PHYS(pg);
		ptp = (pmap_ptpage_t *)PMAP_MAP_POOLPAGE(pa);
	} else {
		bool ok __diagused = pmap_extract(pmap_kernel(), (vaddr_t)ptp, &pa);
		KASSERT(ok);
	}

	pmap_page_attach(pmap, (vaddr_t)ptp, pg, &pmap->pm_ptp_list, 0);

	*pa_p = pa;
	return ptp;
#else
	vaddr_t kva = uvm_km_alloc(kernel_map, PAGE_SIZE, PAGE_SIZE,
	    UVM_KMF_WIRED|UVM_KMF_WAITVA
	    |(flags & PMAP_CANFAIL ? UVM_KMF_CANFAIL : 0));
	if (kva == 0) {
		if (flags & PMAP_CANFAIL)
			return NULL;
		panic("%s: cannot allocate page table page", __func__);
	}
	pmap_page_attach(pmap, kva, NULL, &pmap->pm_ptp_list, 0);
	return (pmap_ptpage_t *)kva;
#endif
}

static void
pmap_ptpage_free(pmap_t pmap, pmap_ptpage_t *ptp)
{
	const vaddr_t kva = (vaddr_t) ptp;

#ifdef DEBUG
	for (size_t j = 0; j < NPTEPG; j++) {
		if (ptp->ptp_ptes[j])
			panic("%s: pte entry %p not 0 (%#x)",
			    __func__, &ptp->ptp_ptes[j],
			    ptp->ptp_ptes[j]);
	}
#endif
	//pmap_md_vca_clean(pg, (vaddr_t)ptp, NBPG);
#ifdef PMAP_PTP_CACHE
	pmap_segtab_pageclean(pmap, &pmap->pm_ptp_list, kva);
	pmap_segtab_pagecache(&pmap_segtab_info.ptp_flist, ptp);
#else
	pmap_segtab_pagefree(pmap, &pmap->pm_ptp_list, kva, PAGE_SIZE);
#endif /* PMAP_PTP_CACHE */
}


#if !defined(PMAP_HWPAGEWALKER) || !defined(PMAP_MAP_POOLPAGE)
static void
pmap_segtab_free(pmap_segtab_t *stp)
{
	/*
	 * Insert the segtab into the segtab freelist.
	 */
	mutex_spin_enter(&pmap_segtab_lock);
	stp->seg_next = pmap_segtab_info.segalloc.free_segtab;
	pmap_segtab_info.segalloc.free_segtab = stp;
	SEGTAB_ADD(nput, 1);
	mutex_spin_exit(&pmap_segtab_lock);
}
#endif

#if defined(PMAP_HWPAGEWALKER) && defined(PMAP_MAP_POOLPAGE)
static pmap_pdetab_t *
pmap_pdetab_alloc(void)
{
	pmap_pdetab_t *ptb;

 again:
	mutex_spin_enter(&pmap_segtab_lock);
	if (__predict_true((ptb = pmap_segtab_info.pdealloc.free_pdetab) != NULL)) {
		pmap_segtab_info.pdealloc.free_pdetab = ptb->pde_next;
		PDETAB_ADD(nget, 1);
		ptb->pde_next = NULL;
	}
	mutex_spin_exit(&pmap_segtab_lock);

	if (__predict_false(ptb == NULL)) {
		struct vm_page * const ptb_pg = pmap_pte_pagealloc();

		if (__predict_false(ptb_pg == NULL)) {
			/*
			 * XXX What else can we do?  Could we deadlock here?
			 */
			uvm_wait(__func__);
			goto again;
		}

		PDETAB_ADD(npage, 1);
		const paddr_t ptb_pa = VM_PAGE_TO_PHYS(ptb_pg);
		ptb = (pmap_pdetab_t *)PMAP_MAP_POOLPAGE(ptb_pa);

		if (pte_invalid_pde() != 0) {
			for (size_t i = 0; i < NPDEPG; i++) {
				ptb->pde_pde[i] = pte_invalid_pde();
			}
		}
	}

	return ptb;
}
#else
/*
 *	Create and return a physical map.
 *
 *	If the size specified for the map
 *	is zero, the map is an actual physical
 *	map, and may be referenced by the
 *	hardware.
 *
 *	If the size specified is non-zero,
 *	the map will be used in software only, and
 *	is bounded by that size.
 */
static pmap_segtab_t *
pmap_segtab_alloc(void)
{
	pmap_segtab_t *stp;
	bool found_on_freelist = false;

 again:
	mutex_spin_enter(&pmap_segtab_lock);
	if (__predict_true((stp = pmap_segtab_info.segalloc.free_segtab) != NULL)) {
		pmap_segtab_info.segalloc.free_segtab = stp->seg_next;
		SEGTAB_ADD(nget, 1);
		stp->seg_next = NULL;
		found_on_freelist = true;
	}
	mutex_spin_exit(&pmap_segtab_lock);

	if (__predict_false(stp == NULL)) {
		struct vm_page * const stp_pg = pmap_pte_pagealloc();

		if (__predict_false(stp_pg == NULL)) {
			/*
			 * XXX What else can we do?  Could we deadlock here?
			 */
			uvm_wait("pmap_create");
			goto again;
		}
		SEGTAB_ADD(npage, 1);
		const paddr_t stp_pa = VM_PAGE_TO_PHYS(stp_pg);

		stp = (pmap_segtab_t *)PMAP_MAP_POOLPAGE(stp_pa);
		const size_t n = NBPG / sizeof(*stp);
		if (n > 1) {
			/*
			 * link all the segtabs in this page together
			 */
			for (size_t i = 1; i < n - 1; i++) {
				stp[i].seg_next = &stp[i + 1];
			}
			/*
			 * Now link the new segtabs into the free segtab list.
			 */
			mutex_spin_enter(&pmap_segtab_lock);
			stp[n - 1].seg_next = pmap_segtab_info.segalloc.free_segtab;
			pmap_segtab_info.segalloc.free_segtab = stp + 1;
			SEGTAB_ADD(nput, n - 1);
			mutex_spin_exit(&pmap_segtab_lock);
		}
	}

	pmap_check_stp(stp, __func__,
		       found_on_freelist ? "from free list" : "allocated");

	return stp;
}
#endif


#ifndef PMAP_HWPAGEWALKER
static void
pmap_segtab_release(pmap_t pmap, pmap_segtab_t **stp_p, bool free_stp,
	pte_callback_t callback, uintptr_t flags,
	vaddr_t va, vsize_t vinc)
{
	pmap_segtab_t *stp = *stp_p;

	for (size_t i = (va / vinc) & (PMAP_SEGTABSIZE - 1);
	     i < PMAP_SEGTABSIZE;
	     i++, va += vinc) {
#ifdef _LP64
		if (vinc > NBSEG) {
			if (stp->seg_seg[i] != NULL) {
				pmap_segtab_release(pmap, &stp->seg_seg[i],
				    true, callback, flags, va, vinc / NSEGPG);
				KASSERT(stp->seg_seg[i] == NULL);
			}
			continue;
		}
#endif
		KASSERT(vinc == NBSEG);

		/* get pointer to segment map */
		pt_entry_t *pte = stp->seg_tab[i]->ptp_ptes;
		if (pte == NULL)
			continue;

		/*
		 * If our caller want a callback, do so.
		 */
		if (callback != NULL) {
			(*callback)(pmap, va, va + vinc, pte, flags);
		}
#ifdef DEBUG
		for (size_t j = 0; j < NPTEPG; j++) {
			if (!pte_zero_p(pte[j]))
				panic("%s: pte entry %p not 0 (%#"PRIxPTE")",
				    __func__, &pte[j], pte_value(pte[j]));
		}
#endif
		// PMAP_UNMAP_POOLPAGE should handle any VCA issues itself
		paddr_t pa = PMAP_UNMAP_POOLPAGE((vaddr_t)pte);
		struct vm_page *pg = PHYS_TO_VM_PAGE(pa);
#ifdef PMAP_PTP_CACHE
		mutex_spin_enter(&pmap_segtab_lock);
		LIST_INSERT_HEAD(&pmap_segtab_info.ptp_pgflist, pg, listq.list);
		mutex_spin_exit(&pmap_segtab_lock);
#else
		uvm_pagefree(pg);
#endif

		stp->seg_tab[i] = NULL;
	}

	if (free_stp) {
		pmap_check_stp(stp, __func__,
			       vinc == NBSEG ? "release seg" : "release xseg");
		pmap_segtab_free(stp);
		*stp_p = NULL;
	}
}
#endif

/*
 * Allocate the top segment table for the pmap.
 */
void
pmap_segtab_init(pmap_t pmap)
{
#if !defined(PMAP_HWPAGEWALKER) || !defined(PMAP_MAP_POOLPAGE)
	/*
	 * Constantly converting from extracted PA to VA is somewhat expensive
	 * for systems with hardware page walkers and without an inexpensive
	 * way to access arbitrary virtual addresses, so we allocate an extra
	 * root segtab so that it can contain non-virtual addresses.
	 */
	pmap->pm_segtab = pmap_segtab_alloc();
#endif
#ifdef PMAP_HWPAGEWALKER
	pmap->pm_pdetab = pmap_pdetab_alloc();
	pmap_md_pdetab_init(pmap);
#endif
}

void
pmap_segtab_remove_all(pmap_t pmap)
{
	struct pglist *list;
	struct vm_page *pg;

//	pmap_pv_remove_all(pmap);
//	KASSERT(TAILQ_EMPTY(&pmap->pm_pvp_list));

#if defined(_LP64)
#if defined(PMAP_HWPAGEWALKER)
	list = &pmap->pm_pdetab_list;
	while ((pg = TAILQ_FIRST(list)) != TAILQ_END(list)) {
		pmap_pdetab_t * const ptb = (pmap_pdetab_t *) VM_PAGE_TO_MD(pg)->mdpg_first.pv_va;
		pmap_pdetab_free(ptb);
	}
#else
	list = &pmap->pm_segtab_list;
	while ((pg = TAILQ_FIRST(list)) != TAILQ_END(list)) {
		pmap_segtab_t * const stb = (pmap_segtab_t *) VM_PAGE_TO_MD(pg)->mdpg_first.pv_va;
		pmap_segtab_free(stb);
	}
#endif
#endif /* _LP64 */
	list = &pmap->pm_ptp_list;
	while ((pg = TAILQ_FIRST(list)) != TAILQ_END(list)) {
		pmap_ptpage_free(pmap, (pmap_ptpage_t *) VM_PAGE_TO_MD(pg)->mdpg_first.pv_va);
	}
}

/*
 *	Retire the given physical map from service.
 *	Should only be called if the map contains
 *	no valid mappings.
 */
void
pmap_segtab_destroy(pmap_t pmap, pte_callback_t func, uintptr_t flags)
{
	KASSERT(pmap != pmap_kernel());
#if defined(PMAP_HWPAGEWALKER)
#if !defined(PMAP_MAP_POOLPAGE)
	KASSERT((pmap->pm_segtab == NULL) == (pmap->pm_pdetab == NULL));
#endif
	KASSERT(pmap->pm_pdetab == NULL);
#endif
#if !defined(PMAP_HWPAGEWALKER) || !defined(PMAP_MAP_POOLPAGE)
//	KASSERT(pmap->pm_segtab == NULL);

	//XXXNH need the rest of this function converting to pmap_segtab_remove_all
	if (pmap->pm_segtab == NULL)
		return;

#ifdef _LP64
	const vsize_t vinc = NBXSEG;
#else
	const vsize_t vinc = NBSEG;
#endif
	pmap_segtab_release(pmap, &pmap->pm_segtab,
	    func == NULL, func, flags, pmap->pm_minaddr, vinc);
#endif
}

/*
 *	Make a new pmap (vmspace) active for the given process.
 */
void
pmap_segtab_activate(struct pmap *pm, struct lwp *l)
{
	if (l == curlwp) {
		KASSERT(pm == l->l_proc->p_vmspace->vm_map.pmap);
		if (pm == pmap_kernel()) {
#if defined(PMAP_HWPAGEWALKER)
			pmap_md_pdetab_activate(pm, l);
#endif
#if !defined(PMAP_HWPAGEWALKER) || !defined(PMAP_MAP_POOLPAGE)
			l->l_cpu->ci_pmap_user_segtab = PMAP_INVALID_SEGTAB_ADDRESS;
#ifdef _LP64
			l->l_cpu->ci_pmap_user_seg0tab = PMAP_INVALID_SEGTAB_ADDRESS;
#endif
#endif
		} else {
#if defined(PMAP_HWPAGEWALKER)
			pmap_md_pdetab_activate(pm, l);
#endif
#if !defined(PMAP_HWPAGEWALKER) || !defined(PMAP_MAP_POOLPAGE)
			l->l_cpu->ci_pmap_user_segtab = pm->pm_segtab;
#ifdef _LP64
			l->l_cpu->ci_pmap_user_seg0tab = pm->pm_segtab->seg_seg[0];
#endif
#endif
		}
	}
}

/*
 *	Act on the given range of addresses from the specified map.
 *
 *	It is assumed that the start and end are properly rounded to
 *	the page size.
 */
void
pmap_pte_process(pmap_t pmap, vaddr_t sva, vaddr_t eva,
	pte_callback_t callback, uintptr_t flags)
{
#if 0
	printf("%s: %p, %"PRIxVADDR", %"PRIxVADDR", %p, %"PRIxPTR"\n",
	    __func__, pmap, sva, eva, callback, flags);
#endif
	while (sva < eva) {
		vaddr_t lastseg_va = pmap_trunc_seg(sva) + NBSEG;
		if (lastseg_va == 0 || lastseg_va > eva)
			lastseg_va = eva;

		/*
		 * If VA belongs to an unallocated segment,
		 * skip to the next segment boundary.
		 */
		pt_entry_t * const pte = pmap_pte_lookup(pmap, sva);
		if (pte != NULL) {
			/*
			 * Callback to deal with the ptes for this segment.
			 */
			(*callback)(pmap, sva, lastseg_va, pte, flags);
		}
		/*
		 * In theory we could release pages with no entries,
		 * but that takes more effort than we want here.
		 */
		sva = lastseg_va;
	}
}





#if defined(PMAP_HWPAGEWALKER) && defined(PMAP_MAP_POOLPAGE)
static pd_entry_t *
pmap_pdetab_reserve(struct pmap *pmap, vaddr_t va)
#elif defined(PMAP_HWPAGEWALKER)
static pmap_ptpage_t **
pmap_segtab_reserve(struct pmap *pmap, vaddr_t va, pd_entry_t **pde_p)
#else
static pmap_ptpage_t **
pmap_segtab_reserve(struct pmap *pmap, vaddr_t va)
#endif
{
#if defined(PMAP_HWPAGEWALKER)
	pmap_pdetab_t *ptb = pmap->pm_pdetab;
#endif
#if defined(PMAP_HWPAGEWALKER) && defined(PMAP_MAP_POOLPAGE)
	vaddr_t segtab_mask = PMAP_PDETABSIZE - 1;
#ifdef _LP64
	for (size_t segshift = XSEGSHIFT;
	     segshift > SEGSHIFT;
	     segshift -= PGSHIFT - 3, segtab_mask = NSEGPG - 1) {
		pd_entry_t * const pde_p =
		     &ptb->pde_pde[(va >> segshift) & segtab_mask];
		pd_entry_t opde = *pde_p;
		if (__predict_false(!pte_pde_valid_p(opde))) {
			ptb = pmap_pdetab_alloc();
			pd_entry_t npde = pte_pde_pdetab(
			    pmap_md_direct_mapped_vaddr_to_paddr((vaddr_t)ptb),
			    pmap == pmap_kernel());
			opde = pte_pde_cas(pde_p, opde, npde);
			if (__predict_false(pte_pde_valid_p(opde))) {
				pmap_pdetab_free(ptb);
			} else {
				opde = npde;
			}
		}
		ptb = pmap_pde_to_pdetab(opde);
	}
#elif defined(XSEGSHIFT)
	pd_entry_t opde = ptb->pde_pde[(va >> segshift) & segtab_mask];
	KASSERT(pte_pde_valid_p(opde));
	ptb = pmap_pde_to_pdetab(opde);
	segtab_mask = NSEGPG - 1;
#endif /* _LP64 */
	const size_t idx = (va >> SEGSHIFT) & segtab_mask;
	return &ptb->pde_pde[idx];
#else /* PMAP_HWPAGEWALKER && PMAP_MAP_POOLPAGE */
	pmap_segtab_t *stb = pmap->pm_segtab;
	vaddr_t segtab_mask = PMAP_SEGTABSIZE - 1;
#ifdef _LP64
	for (size_t segshift = XSEGSHIFT;
	     segshift > SEGSHIFT;
	     segshift -= PGSHIFT - 3, segtab_mask = NSEGPG - 1) {
		size_t idx = (va >> segshift) & segtab_mask;
		pmap_segtab_t ** const stb_p = &stb->seg_seg[idx];
#if defined(PMAP_HWPAGEWALKER)
		pmap_pdetab_t ** const ptb_p = &ptb->pde_pde[idx];
#endif
		if (__predict_false((stb = *stb_p) == NULL)) {
			stb = pmap_segtab_alloc();
#ifdef MULTIPROCESSOR
			pmap_segtab_t *ostb = atomic_cas_ptr(stb_p, NULL, stb);
			if (__predict_false(ostb != NULL)) {
				pmap_segtab_free(stb);
				stb = ostb;
			}
#else
			*stb_p = stb;
#endif
		}
	}
#elif defined(PMAP_HWPAGEWALKER)
	pmap_segtab_t opde = ptb->pde_pde[(va >> segshift) & segtab_mask];
	KASSERT(pte_pde_valid_p(opde));
	ptb = pmap_pde_to_pdetab(opde);
	segtab_mask = NSEGPG - 1;

#endif /* PMAP_HWPAGEWALKER */
	size_t idx = (va >> SEGSHIFT) & segtab_mask;
#if defined(PMAP_HWPAGEWALKER)
#if defined(XSEGSHIFT) && XSEGSHIFT != SEGSHIFT)
	*pte_p = &pmap->pm_segtab
#else
	*pde_p = &ptb->pde_pde[idx];
#endif
#endif
	return &stb->seg_tab[idx];
#endif
}





/*
 *	Return a pointer for the pte that corresponds to the specified virtual
 *	address (va) in the target physical map, allocating if needed.
 */
pt_entry_t *
pmap_pte_reserve(pmap_t pmap, vaddr_t va, int flags)
{
//	const size_t pte_idx = (va >> PGSHIFT) & (NPTEPG - 1);
	pmap_ptpage_t *ptp;
	paddr_t pa = 0;

#if defined(PMAP_HWPAGEWALKER) && defined(PMAP_MAP_POOLPAGE)
	pd_entry_t * const pde_p = pmap_pdetab_reserve(pmap, va);
	ptp = pmap_pde_to_ptpage(*pde_p);
#elif defined(PMAP_HWPAGEWALKER)
	pmap_ptpage_t ** const ptp_p = pmap_segtab_reserve(pmap, va, &pde_p);
	ptp = *ptp_p;
#else
	pmap_ptpage_t ** const ptp_p = pmap_segtab_reserve(pmap, va);
	ptp = *ptp_p;
#endif

	if (__predict_false(ptp == NULL)) {
		ptp = pmap_ptpage_alloc(pmap, flags, &pa);
		if (__predict_false(ptp == NULL))
			return NULL;

#if defined(PMAP_HWPAGEWALKER)
		pd_entry_t npde = pte_pde_ptpage(pa, pmap == pmap_kernel());
#endif
#if defined(PMAP_HWPAGEWALKER) && defined(PMAP_MAP_POOLPAGE)
		pd_entry_t opde = *pde_p;
		opde = pte_pde_cas(pde_p, opde, npde);
		if (__predict_false(pte_pde_valid_p(opde))) {
			pmap_ptpage_free(pmap, ptp);
			ptp = pmap_pde_to_ptpage(opde);
		}
#else
#ifdef MULTIPROCESSOR
		pmap_ptpage_t *optp = atomic_cas_ptr(ptp_p, NULL, ptp);
		/*
		 * If another thread allocated the segtab needed for this va
		 * free the page we just allocated.
		 */
		if (__predict_false(optp != NULL)) {
			pmap_ptpage_free(pmap, ptp);
			ptp = optp;
#if defined(PMAP_HWPAGEWALKER)
		} else {
			pte_pde_set(pde_p, npde);
#endif
		}
#else /* !MULTIPROCESSOR */
		*ptp_p = ptp;
#endif /* MULTIPROCESSOR */
#endif /* PMAP_HWPAGEWALKER && PMAP_MAP_POOLPAGE */
	}

	const size_t pte_idx = pte_index(va);

	return ptp->ptp_ptes + pte_idx;
}
