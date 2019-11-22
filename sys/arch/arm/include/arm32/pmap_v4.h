/*	$NetBSD: pmap.h,v 1.147 2017/05/25 20:42:41 skrll Exp $	*/

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

#ifndef	_ARM32_PMAP_V4_H_
#define	_ARM32_PMAP_V4_H_

struct l1_ttable;
struct l2_dtable;

#define	POOL_VTOPHYS(va)	vtophys((vaddr_t) (va))

struct pmap_page {
	SLIST_HEAD(,pv_entry) pvh_list;		/* pv_entry list */
	int pvh_attrs;				/* page attributes */
	u_int uro_mappings;
	u_int urw_mappings;
	union {
		u_short s_mappings[2];	/* Assume kernel count <= 65535 */
		u_int i_mappings;
	} k_u;
};

/*
 * pmap-specific data store in the vm_page structure.
 */
#define	__HAVE_VM_PAGE_MD
struct vm_page_md {
	struct pmap_page pp;
#define	pvh_list	pp.pvh_list
#define	pvh_attrs	pp.pvh_attrs
#define	uro_mappings	pp.uro_mappings
#define	urw_mappings	pp.urw_mappings
#define	kro_mappings	pp.k_u.s_mappings[0]
#define	krw_mappings	pp.k_u.s_mappings[1]
#define	k_mappings	pp.k_u.i_mappings
};

#define PMAP_PAGE_TO_MD(ppage) container_of((ppage), struct vm_page_md, pp)

/*
 * Set the default color of each page.
 */
#if ARM_MMU_V6 > 0
#define	VM_MDPAGE_PVH_ATTRS_INIT(pg) \
	(pg)->mdpage.pvh_attrs = (pg)->phys_addr & arm_cache_prefer_mask
#else
#define	VM_MDPAGE_PVH_ATTRS_INIT(pg) \
	(pg)->mdpage.pvh_attrs = 0
#endif

#define	VM_MDPAGE_INIT(pg)						\
do {									\
	SLIST_INIT(&(pg)->mdpage.pvh_list);				\
	VM_MDPAGE_PVH_ATTRS_INIT(pg);					\
	(pg)->mdpage.uro_mappings = 0;					\
	(pg)->mdpage.urw_mappings = 0;					\
	(pg)->mdpage.k_mappings = 0;					\
} while (/*CONSTCOND*/0)

#include <uvm/uvm_page.h>


/*
 * Flags that indicate attributes of pages or mappings of pages.
 *
 * The PVF_MOD and PVF_REF flags are stored in the mdpage for each
 * page.  PVF_WIRED, PVF_WRITE, and PVF_NC are kept in individual
 * pv_entry's for each page.  They live in the same "namespace" so
 * that we can clear multiple attributes at a time.
 *
 * Note the "non-cacheable" flag generally means the page has
 * multiple mappings in a given address space.
 */
#define	PVF_MOD		0x01		/* page is modified */
#define	PVF_REF		0x02		/* page is referenced */
#define	PVF_WIRED	0x04		/* mapping is wired */
#define	PVF_WRITE	0x08		/* mapping is writable */
#define	PVF_EXEC	0x10		/* mapping is executable */
#ifdef PMAP_CACHE_VIVT
#define	PVF_UNC		0x20		/* mapping is 'user' non-cacheable */
#define	PVF_KNC		0x40		/* mapping is 'kernel' non-cacheable */
#define	PVF_NC		(PVF_UNC|PVF_KNC)
#endif
#ifdef PMAP_CACHE_VIPT
#define	PVF_NC		0x20		/* mapping is 'kernel' non-cacheable */
#define	PVF_MULTCLR	0x40		/* mapping is multi-colored */
#endif
#define	PVF_COLORED	0x80		/* page has or had a color */
#define	PVF_KENTRY	0x0100		/* page entered via pmap_kenter_pa */
#define	PVF_KMPAGE	0x0200		/* page is used for kmem */
#define	PVF_DIRTY	0x0400		/* page may have dirty cache lines */
#define	PVF_KMOD	0x0800		/* unmanaged page is modified  */
#define	PVF_KWRITE	(PVF_KENTRY|PVF_WRITE)
#define	PVF_DMOD	(PVF_MOD|PVF_KMOD|PVF_KMPAGE)


/*
 * Macros that we need to export
 */
#define	pmap_resident_count(pmap)	((pmap)->pm_stats.resident_count)
#define	pmap_wired_count(pmap)		((pmap)->pm_stats.wired_count)

#define	pmap_is_modified(pg)	\
	(((pg)->mdpage.pvh_attrs & PVF_MOD) != 0)
#define	pmap_is_referenced(pg)	\
	(((pg)->mdpage.pvh_attrs & PVF_REF) != 0)
#define	pmap_is_page_colored_p(md)	\
	(((md)->pvh_attrs & PVF_COLORED) != 0)

#define	pmap_copy(dp, sp, da, l, sa)	/* nothing */

/*
 * Macros to determine if a mapping might be resident in the
 * instruction/data cache and/or TLB
 */
#if ARM_MMU_V7 > 0 && !defined(ARM_MMU_EXTENDED)
/*
 * Speculative loads by Cortex cores can cause TLB entries to be filled even if
 * there are no explicit accesses, so there may be always be TLB entries to
 * flush.  If we used ASIDs then this would not be a problem.
 */
#define	PV_BEEN_EXECD(f)  (((f) & PVF_EXEC) == PVF_EXEC)
#define	PV_BEEN_REFD(f)   (true)
#else
#define	PV_BEEN_EXECD(f)  (((f) & (PVF_REF | PVF_EXEC)) == (PVF_REF | PVF_EXEC))
#define	PV_BEEN_REFD(f)   (((f) & PVF_REF) != 0)
#endif
#define	PV_IS_EXEC_P(f)   (((f) & PVF_EXEC) != 0)
#define	PV_IS_KENTRY_P(f) (((f) & PVF_KENTRY) != 0)
#define	PV_IS_WRITE_P(f)  (((f) & PVF_WRITE) != 0)

#define	PVLIST_EMPTY_P(pg)	SLIST_EMPTY(VM_PAGE_TO_MD(pg)->pvh_list)

/*
 * Functions that we need to export
 */
void	pmap_procwr(struct proc *, vaddr_t, int);
void	pmap_remove_all(pmap_t);
bool	pmap_extract(pmap_t, vaddr_t, paddr_t *);

void	pmap_md_clean_page(struct vm_page_md *, bool);

#define	PMAP_NEED_PROCWR
#define PMAP_GROWKERNEL		/* turn on pmap_growkernel interface */
#define	PMAP_ENABLE_PMAP_KMPAGE	/* enable the PMAP_KMPAGE flag */

#if (ARM_MMU_V6 + ARM_MMU_V7) > 0
#define	PMAP_PREFER(hint, vap, sz, td)	pmap_prefer((hint), (vap), (td))
void	pmap_prefer(vaddr_t, vaddr_t *, int);
#endif

//void	pmap_icache_sync_range(pmap_t, vaddr_t, vaddr_t);

static inline int
pmap_md_pagecolor(struct vm_page *pg)
{
// err this is always 0???
#if (ARM_MMU_V6 + ARM_MMU_V7) > 0
	struct vm_page_md * const md = VM_PAGE_TO_MD(pg);
	//pv_entry_t pv = &mdpg->mdpg_first;

	return md->pvh_attrs & arm_cache_prefer_mask;
#else
	return 0;
#endif
}

/*
 * Track cache/tlb occupancy using the following structure
 */
union pmap_cache_state {
	struct {
		union {
			uint8_t csu_cache_b[2];
			uint16_t csu_cache;
		} cs_cache_u;

		union {
			uint8_t csu_tlb_b[2];
			uint16_t csu_tlb;
		} cs_tlb_u;
	} cs_s;
	uint32_t cs_all;
};
#define	cs_cache_id	cs_s.cs_cache_u.csu_cache_b[0]
#define	cs_cache_d	cs_s.cs_cache_u.csu_cache_b[1]
#define	cs_cache	cs_s.cs_cache_u.csu_cache
#define	cs_tlb_id	cs_s.cs_tlb_u.csu_tlb_b[0]
#define	cs_tlb_d	cs_s.cs_tlb_u.csu_tlb_b[1]
#define	cs_tlb		cs_s.cs_tlb_u.csu_tlb

/*
 * Assigned to cs_all to force cacheops to work for a particular pmap
 */
#define	PMAP_CACHE_STATE_ALL	0xffffffffu
/*
 * The pmap structure itself
 */
struct pmap {
	struct uvm_object	pm_obj;
	kmutex_t		pm_obj_lock;
#define	pm_lock pm_obj.vmobjlock
#ifndef ARM_HAS_VBAR
	pd_entry_t		*pm_pl1vec;
	pd_entry_t		pm_l1vec;
#endif
	struct l2_dtable	*pm_l2[L2_SIZE];
	struct pmap_statistics	pm_stats;
	LIST_ENTRY(pmap)	pm_list;
	struct l1_ttable	*pm_l1;
	union pmap_cache_state	pm_cstate;
	uint8_t			pm_domain;
	bool			pm_activated;
	bool			pm_remove_all;
};

struct pmap_kernel {
	struct pmap		kernel_pmap;
};

/*
 * Real definition of pv_entry.
 */
struct pv_entry {
	SLIST_ENTRY(pv_entry) pv_link;	/* next pv_entry */
	pmap_t		pv_pmap;        /* pmap where mapping lies */
	vaddr_t		pv_va;          /* virtual address for mapping */
	u_int		pv_flags;       /* flags */
};

/*
 * Move this back into pmap.c when pmap_icache_sync_range is pmap agnostic
 */

/*
 * Misc. locking data structures
 */

extern kmutex_t pmap_lock;

static inline void
pmap_acquire_pmap_lock(pmap_t pm)
{
#if defined(MULTIPROCESSOR) && defined(DDB)
	if (__predict_false(db_onproc != NULL))
		return;
#endif

	mutex_enter(pm->pm_lock);
}

static inline void
pmap_release_pmap_lock(pmap_t pm)
{
#if defined(MULTIPROCESSOR) && defined(DDB)
	if (__predict_false(db_onproc != NULL))
		return;
#endif
	mutex_exit(pm->pm_lock);
}

static inline void
pmap_acquire_page_lock(struct vm_page_md *md)
{
	mutex_enter(&pmap_lock);
}

static inline void
pmap_release_page_lock(struct vm_page_md *md)
{
	mutex_exit(&pmap_lock);
}

#ifdef DIAGNOSTIC
static inline int
pmap_page_locked_p(struct vm_page_md *md)
{
	return mutex_owned(&pmap_lock);
}
#endif


/*
 * Useful macros and constants
 */

/* Virtual address to page table entry */
static inline pt_entry_t *
vtopte(vaddr_t va)
{
	pd_entry_t *pdep;
	pt_entry_t *ptep;

	KASSERT(trunc_page(va) == va);

	if (pmap_get_pde_pte(pmap_kernel(), va, &pdep, &ptep) == false)
		return NULL;
	return ptep;
}

/*
 * Virtual address to physical address
 */
static inline paddr_t
vtophys(vaddr_t va)
{
	paddr_t pa;

	if (pmap_extract(pmap_kernel(), va, &pa) == false)
		return 0;	/* XXXSCW: Panic? */

	return pa;
}



static inline void
pmap_impl_pageidlezero_done(struct vm_page *pg)
{
#if defined(PMAP_CACHE_VIPT)
	struct vm_page_md *md = VM_PAGE_TO_MD(dst_pg);
	/*
	 * This page is now cache resident so it now has a page color.
	 * Any contents have been obliterated so clear the EXEC flag.
	 */
	if (!pmap_is_page_colored_p(md)) {
		PMAPCOUNT(vac_color_new);
		md->pvh_attrs |= PVF_COLORED;
	}
	if (PV_IS_EXEC_P(md->pvh_attrs)) {
		md->pvh_attrs &= ~PVF_EXEC;
		PMAPCOUNT(exec_discarded_zero);
	}
#endif
}

static inline void
pmap_impl_copypage_done(struct vm_page *pg)
{
#ifdef PMAP_CACHE_VIPT
	struct vm_page_md *dst_md = VM_PAGE_TO_MD(dst_pg);

	/*
	 * Now that the destination page is in the cache, mark it as colored.
	 * If this was an exec page, discard it.
	 */
	pmap_acquire_page_lock(dst_md);
	if (arm_pcache.cache_type == CACHE_TYPE_PIPT) {
		dst_md->pvh_attrs &= ~arm_cache_prefer_mask;
		dst_md->pvh_attrs |= (dst & arm_cache_prefer_mask);
	}
	if (!pmap_is_page_colored_p(dst_md)) {
		PMAPCOUNT(vac_color_new);
		dst_md->pvh_attrs |= PVF_COLORED;
	}
	dst_md->pvh_attrs |= PVF_DIRTY;
	if (PV_IS_EXEC_P(dst_md->pvh_attrs)) {
		dst_md->pvh_attrs &= ~PVF_EXEC;
		PMAPCOUNT(exec_discarded_copy);
	}
	pmap_release_page_lock(dst_md);
#endif
}

#endif	/* _ARM32_PMAP_V4_H_ */
