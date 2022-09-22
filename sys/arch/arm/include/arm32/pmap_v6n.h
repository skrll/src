/*	$NetBSD$	*/

/*
 * Copyright (c) 2002, 2003 Wasabi Systems, Inc.
 * All rights reserved.
 *
 * Written by Jason R. Thorpe & Steve C. Woodford for Wasabi Systems, Inc.
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
 *	This product includes software developed for the NetBSD Project by
 *	Wasabi Systems, Inc.
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
 * Copyright (c) 1994,1995 Mark Brinicombe.
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
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Mark Brinicombe
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
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef	_ARM32_PMAP_V6N_H_
#define	_ARM32_PMAP_V6N_H_

#if 0
// if system has too much ram to do direct map then it all turns to pot
// let's hope we don't need this
#ifndef __HAVE_MM_MD_DIRECT_MAPPED_PHYS
#error PMAP requires __HAVE_MM_MD_DIRECT_MAPPED_PHYS
#endif
#endif

#define	PMAP_HWPAGEWALKER		1
#define	PMAP_NEED_ALLOC_POOLPAGE	1
#define	PMAP_TLB_MAX			1
#if PMAP_TLB_MAX > 1
#define	PMAP_TLB_NEED_SHOOTDOWN		1
#endif
#define	PMAP_TLB_FLUSH_ASID_ON_RESET	arm_has_tlbiasid_p
#define	PMAP_TLB_NUM_PIDS		256
#define	cpu_set_tlb_info(ci, ti)	((void)((ci)->ci_tlb_info = (ti)))
#if PMAP_TLB_MAX > 1
#define	cpu_tlb_info(ci)		((ci)->ci_tlb_info)
#else
#define	cpu_tlb_info(ci)		(&pmap_tlb0_info)
#endif
#define	pmap_md_tlb_asid_max()		(PMAP_TLB_NUM_PIDS - 1)

#define	PMAP_PDETABSIZE	(L1_TABLE_SIZE / sizeof(pd_entry_t))
#ifdef _LP64
#define	PMAP_INVALID_PDETAB_ADDRESS	((pmap_pdetab_t *)(VM_MIN_KERNEL_ADDRESS - PAGE_SIZE))
#define	PMAP_INVALID_SEGTAB_ADDRESS	((pmap_segtab_t *)(VM_MIN_KERNEL_ADDRESS - PAGE_SIZE))
#else
#define	PMAP_INVALID_PDETAB_ADDRESS	((pmap_pdetab_t *)0xdeadbeef)
#define	PMAP_INVALID_SEGTAB_ADDRESS	((pmap_segtab_t *)0xdeadbeef)
#endif

#define	NPDEPG		(PAGE_SIZE / sizeof(pd_entry_t))

/* ARM short descriptor translation table format */
#define	SEGSHIFT	20			/* LOG2(NBSEG) */
#define	NBSEG		(1 << SEGSHIFT)		/* bytes/segment */
#define	SEGOFSET	(NBSEG - 1)		/* byte offset into segment */

#ifdef _LP64
#define	SEGLENGTH	(PGSHIFT - 3)
#define	XSEGSHIFT	(SEGSHIFT + SEGLENGTH)	/* LOG2(NBXSEG) */
#define	NBXSEG		(1UL << XSEGSHIFT)	/* bytes/xsegment */
#define	XSEGOFSET	(NBXSEG - 1)		/* byte offset into xsegment */
#define	XSEGLENGTH	(PGSHIFT - 3)
#define	NXSEGPG		(1 << XSEGLENGTH)
#else
#define	SEGLENGTH	(31 - SEGSHIFT)
#endif
#define	NSEGPG		(1 << SEGLENGTH)

int	pmap_maxproc_set(int);

#if defined(__PMAP_PRIVATE)
struct vm_page_md;

#include <uvm/uvm_physseg.h>

void	pmap_md_init(void);
void	pmap_md_icache_sync_all(void);
void	pmap_md_icache_sync_range_index(vaddr_t, vsize_t);
void	pmap_md_page_syncicache(struct vm_page_md *, const kcpuset_t *);
bool	pmap_md_vca_add(struct vm_page_md *, vaddr_t, pt_entry_t *);
void	pmap_md_vca_clean(struct vm_page_md *, int);
void	pmap_md_vca_remove(struct vm_page *, vaddr_t, bool, bool);
bool	pmap_md_ok_to_steal_p(const uvm_physseg_t, size_t);
bool	pmap_md_tlb_check_entry(void *, vaddr_t, tlb_asid_t, pt_entry_t);

void	pmap_md_alloc_pdp(pmap_t);
void	pmap_md_free_pdp(pmap_t);

vaddr_t pmap_md_direct_map_paddr(paddr_t);

#ifdef PMAP_CACHE_VIPT
#define	PMAP_VIRTUAL_CACHE_ALIASES
#endif

#ifdef MULTIPROCESSOR
#define	PMAP_NO_PV_UNCACHED
#endif

static inline bool
pmap_md_virtual_cache_aliasing_p(void)
{
#if defined(PMAP_CACHE_VIPT)
	return true;
#else
	return false;
#endif
}

static inline vsize_t
pmap_md_cache_prefer_mask(void)
{
#if defined(PMAP_CACHE_VIPT)
	return arm_cache_prefer_mask;
#else
	return 0;
#endif
}

static inline pt_entry_t *
pmap_md_nptep(pt_entry_t *ptep)
{
	return ptep + PAGE_SIZE / L2_S_SIZE;
}

#if defined(EFI_RUNTIME)
#if 0
static struct pmap	efirt_pmap;

static inline struct pmap *
pmap_efirt(void)
{
	return &efirt_pmap;
}
#endif

void	pmap_activate_efirt(void);
void	pmap_deactivate_efirt(void);
#endif

#endif	/* __PMAP_PRIVATE */


#define	__HAVE_PMAP_MD
struct pmap_md {
	pd_entry_t *		pmd_l1;
	paddr_t			pmd_l1_pa;
	struct l2_dtable *	pmd_l2[L2_SIZE];
};

#define	pm_l1		pm_md.pmd_l1
#define	pm_l1_pa	pm_md.pmd_l1_pa
#define	pm_l2		pm_md.pmd_l2

void pmap_md_pdetab_init(struct pmap *);
void pmap_md_pdetab_fini(struct pmap *);

vaddr_t pmap_md_map_poolpage(paddr_t, size_t);
paddr_t pmap_md_unmap_poolpage(vaddr_t, size_t);
struct vm_page *pmap_md_alloc_poolpage(int);

/*
 * Other hooks for the pool allocator.
 */
paddr_t	pmap_md_pool_vtophys(vaddr_t);
vaddr_t	pmap_md_pool_phystov(paddr_t);
#define	POOL_VTOPHYS(va)	pmap_md_pool_vtophys((vaddr_t)va)
#define	POOL_PHYSTOV(pa)	pmap_md_pool_phystov((paddr_t)pa)

bool	pmap_md_kernel_vaddr_p(vaddr_t);
paddr_t	pmap_md_kernel_vaddr_to_paddr(vaddr_t);
bool	pmap_md_direct_mapped_vaddr_p(vaddr_t);
paddr_t	pmap_md_direct_mapped_vaddr_to_paddr(vaddr_t);
vaddr_t	pmap_md_direct_map_paddr(paddr_t);
bool	pmap_md_io_vaddr_p(vaddr_t);

void   pmap_md_activate_efirt(void);
void   pmap_md_deactivate_efirt(void);

#include <uvm/pmap/vmpagemd.h>
#include <uvm/pmap/pmap.h>
#include <uvm/pmap/pmap_pvt.h>
#include <uvm/pmap/pmap_tlb.h>
#include <uvm/pmap/pmap_synci.h>
#include <uvm/pmap/tlb.h>

#include <uvm/uvm_page.h>

struct pmap_page {
	struct vm_page_md pp_md;
};

#define	PMAP_PAGE_TO_MD(ppage)	(&(ppage)->pp_md)

/*
 * If we have an EXTENDED MMU and the address space is split evenly between
 * user and kernel, we can use the TTBR0/TTBR1 to have separate L1 tables for
 * user and kernel address spaces.
 */
#if (KERNEL_BASE & 0x80000000) == 0
#error ARMv6 or later systems must have a KERNEL_BASE >= 0x80000000
#endif

extern bool arm_has_tlbiasid_p;	/* also in <arm/locore.h> */

#define	PVLIST_EMPTY_P(pg)	VM_PAGEMD_PVLIST_EMPTY_P(VM_PAGE_TO_MD(pg))

static __inline paddr_t
pte_to_paddr(pt_entry_t pte)
{

	return l2pte_pa(pte);
}

static inline bool
pte_valid_p(pt_entry_t pte)
{

	return l2pte_valid_p(pte);
}

static inline bool
pte_referenced_p(pt_entry_t pte)
{
	return false;
}

static inline pt_entry_t
pte_clear_modify(pt_entry_t pte)
{
	return pte;
}

static inline pt_entry_t
pte_clear_reference(pt_entry_t pte)
{
	return pte;
}

static inline int
pmap_md_pagecolor(struct vm_page *pg)
{
	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);
	pv_entry_t pv = &mdpg->mdpg_first;

	return pv->pv_va;
}

static inline void
pmap_md_clean_page(struct vm_page_md *md, bool is_src)
{
}

#ifdef notyet
static inline void
pmap_md_setvirtualend(vaddr_t va)
{
	pmap_limits.virtual_end = va;
}
#endif

static inline size_t
pte_index(vaddr_t va)
{

	return l2pte_index(va);
}

static inline bool
pte_modified_p(pt_entry_t pte)
{
#if 0
	VM_PAGEMD_MODIFIED
	VM_PAGEMD_MODIFIED_P(mdpg))
	/* XXXNH need emulation */
#endif
	return false;
}

static inline bool
pte_wired_p(pt_entry_t pte)
{

	return (pte & L2_S_OSBIT0) != 0;
}


static inline pt_entry_t
pte_wire_entry(pt_entry_t pte)
{

	return pte | L2_S_OSBIT0;
}

static inline pt_entry_t
pte_unwire_entry(pt_entry_t pte)
{

	return pte & ~L2_S_OSBIT0;
}

static inline uint32_t
pte_value(pt_entry_t pte)
{

	return pte;
}

static inline bool
pte_readonly_p(pt_entry_t pte)
{
	//XXXNH check this logic works
	return !l2pte_writable_p(pte);
}

static inline bool
pte_cached_p(pt_entry_t pte)
{
	// XXXNH depends which index into PRRR/NMRR we use
	// assume non-zero means cached for now
	return (pte & L2_S_CACHE_MASK) != 0;
}

static inline bool
pte_deferred_exec_p(pt_entry_t pte)
{

	return false;
}

static inline pt_entry_t
pte_nv_entry(bool kernel_p)
{

	/* Not valid entry */
	return kernel_p ? 0 : 0;
}

static inline pt_entry_t
pte_prot_downgrade(pt_entry_t pte, vm_prot_t prot)
{

	return (pte & ~(L2_S_PROT_W | L2_S_PROT_RO))
	    | (((prot) & (VM_PROT_READ | VM_PROT_WRITE)) == VM_PROT_READ ? L2_S_PROT_RO : L2_S_PROT_W);
}

static inline pt_entry_t
pte_prot_nowrite(pt_entry_t pte)
{

	return l2pte_set_readonly(pte);
}

static inline pt_entry_t
pte_cached_change(pt_entry_t pte, bool cached)
{
	pte &= ~L2_S_CACHE_MASK;
	pte |= (cached ? pte_l2_s_cache_mode : 0);

	return pte;
}

static inline void
pte_set(pt_entry_t *ptep, pt_entry_t pte)
{
	if (pte == 0)
		l2pte_reset(ptep);
	else
		l2pte_set(ptep, pte, *ptep);
}

static inline pd_entry_t
pte_invalid_pde(void)
{

	return 0;
}

// XXXNH only used in _LP64
static inline pd_entry_t
pte_pde_pdetab(paddr_t pa, bool kernel_p)
{

	return 0;
}

static inline pd_entry_t
pte_pde_ptpage(paddr_t pa, bool kernel_p)
{
	const uint8_t dom = kernel_p ? PMAP_DOMAIN_KERNEL : PMAP_DOMAIN_USER;

	return L1_C_PROTO | L1_C_DOM(dom) | pa;
}

static inline bool
pte_pde_valid_p(pd_entry_t pde)
{

	return l1pte_valid_p(pde);
}

static inline paddr_t
pte_pde_to_paddr(pd_entry_t pde)
{

	return l1pte_pa(pde);
}

static inline pd_entry_t
pte_pde_cas(pd_entry_t *pdep, pd_entry_t opde, pt_entry_t npde)
{

#ifdef MULTIPROCESSOR
	return atomic_cas_32(pdep, opde, npde);
#else
	*pdep = npde;
	return 0;
#endif
}

static inline void
pte_pde_set(pd_entry_t *pdep, pd_entry_t npde)
{

	*pdep = npde;
}


#ifdef __PMAP_PRIVATE
struct vm_page_md;

static inline pt_entry_t
pte_make_kenter_pa(paddr_t pa, struct vm_page_md *mdpg, vm_prot_t prot,
    u_int flags)
{
	pt_entry_t pte = pa
	    | L2_S_PROTO
	    | L2_S_PROT(PTE_KERNEL, prot)
	    | ((flags & PMAP_NOCACHE) ? 0 : ((flags & PMAP_PTE)
		? pte_l2_s_cache_mode_pt : pte_l2_s_cache_mode));
	if (prot & VM_PROT_EXECUTE)
	    pte &= ~L2_XS_XN;

	if (flags & ARM32_MMAP_CACHEABLE) {
		pte |= pte_l2_s_cache_mode;
	} else if (flags & ARM32_MMAP_WRITECOMBINE) {
		pte |= pte_l2_s_wc_mode;
	}

	return pte;
}

static inline pt_entry_t
pte_make_enter_efirt(paddr_t pa, vm_prot_t prot, u_int flags)
{
	pt_entry_t pte = pa
	    | L2_S_PROTO
	    | L2_S_PROT(PTE_USER, prot)
	    | pte_l2_s_cache_mode;

	if (prot & VM_PROT_EXECUTE)
	    pte &= ~L2_XS_XN;

//	pte |= L2_S_PROT_U;

	return pte;
}


static inline pt_entry_t
pte_make_enter(paddr_t pa, const struct vm_page_md *mdpg, vm_prot_t prot,
    u_int flags, bool is_kernel_pmap_p)
{
	pt_entry_t npte = pa;
	const bool cached = (flags & PMAP_NOCACHE);

	if ((flags & VM_PROT_ALL) || VM_PAGEMD_REFERENCED_P(mdpg)) {
		/*
		 * - The access type indicates that we don't need
		 *   to do referenced emulation.
		 * OR
		 * - The physical page has already been referenced
		 *   so no need to re-do referenced emulation here.
		 */
		npte |= l2pte_set_readonly(L2_S_PROTO);

		if ((prot & VM_PROT_WRITE) != 0 &&
		    ((flags & VM_PROT_WRITE) != 0 || VM_PAGEMD_MODIFIED_P(mdpg))) {
			/*
			 * This is a writable mapping, and the
			 * page's mod state indicates it has
			 * already been modified. Make it
			 * writable from the outset.
			 */
			npte = l2pte_set_writable(npte);
		}
		if (prot & VM_PROT_EXECUTE)
			npte &= ~L2_XS_XN;
	} else {
		/*
		 * Need to do page referenced emulation.
		 */
		npte |= L2_TYPE_INV;
	}

	if (flags & ARM32_MMAP_WRITECOMBINE) {
		npte |= pte_l2_s_wc_mode;
	} else
		npte |= pte_l2_s_cache_mode;

	if (!cached)
		npte &= ~L2_S_CACHE_MASK;

	/*
	 * Make sure userland mappings get the right permissions
	 */
	if (!is_kernel_pmap_p) {
		npte |= L2_S_PROT_U;
	}

	return npte;
}


static inline void
pmap_impl_pageidlezero_done(struct vm_page *pg)
{
}

static inline void
pmap_impl_copypage_done(struct vm_page *pg)
{
}

static inline void
pmap_md_asid_activate(tlb_asid_t asid, pmap_t pm, struct lwp *l)
{
	KASSERT(kpreempt_disabled());

	struct cpu_info * const ci = curcpu();

	/*
	 * Assume that TTBR1 has only global mappings and TTBR0 only
	 * has non-global mappings.  To prevent speculation from doing
	 * evil things translation table walks using TTBR0 are disabled
	 * whilst there is no active userland pmap. Assert this here.
	 *
	 * In doing so the new CONTEXTIDR (ASID) and new TTBR0 value are
	 * only enable once both are set.
	 */
	KASSERT(
	    (armreg_pfr1_read() & ARM_PFR1_SEC_MASK) == 0 ||
	    (armreg_ttbcr_read() & TTBCR_S_PD0) != 0
	);

	if (pm == pmap_kernel())
		return;

	armreg_contextidr_write(asid);
	armreg_ttbr_write(pm->pm_l1_pa |
	    (ci->ci_mpidr ? TTBR_MPATTR : TTBR_UPATTR));
	isb();

	/*
	 * Now we can reenable tablewalks since the CONTEXTIDR and TTRB0
	 * have been updated.
	 */
	const uint32_t old_ttbcr = armreg_ttbcr_read();
	armreg_ttbcr_write(old_ttbcr & ~TTBCR_S_PD0);
	cpu_cpwait();

	KASSERTMSG(ci->ci_pmap_asid_cur == asid, "%u vs %u",
	    ci->ci_pmap_asid_cur, asid);
	ci->ci_pmap_cur = pm;
}

void
static inline
pmap_md_asid_deactivate(pmap_t pm)
{
	KASSERT(kpreempt_disabled());

	struct cpu_info * const ci = curcpu();
	/*
	 * Disable translation table walks from TTBR0 while no pmap has been
	 * activated.
	 */
	const uint32_t old_ttbcr = armreg_ttbcr_read();
	armreg_ttbcr_write(old_ttbcr | TTBCR_S_PD0);
	isb();

	ci->ci_pmap_cur = pmap_kernel();
	KASSERTMSG(ci->ci_pmap_asid_cur == KERNEL_PID, "ci_pmap_asid_cur %u",
	    ci->ci_pmap_asid_cur);
}

#endif /* __PMAP_PRIVATE */

#endif	/* _ARM32_PMAP_V6N_H_ */
