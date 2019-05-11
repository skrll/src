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

#include "opt_arm_debug.h"
#include "opt_cpuoptions.h"
#include "opt_pmap_debug.h"
#include "opt_ddb.h"
#include "opt_lockdebug.h"
#include "opt_multiprocessor.h"

#if 0
#ifdef MULTIPROCESSOR
#define _INTR_PRIVATE
#endif
#endif

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
#include <arm/arm32/pmap_common.h>
#include <arm/arm32/machdep.h>

#include <arm/locore.h>

__KERNEL_RCSID(0, "$NetBSD$");

#if 0
//#define PMAP_DEBUG
#ifdef PMAP_DEBUG

/* XXX need to get rid of all refs to this */
int pmap_debug_level = 0;

/*
 * for switching to potentially finer grained debugging
 */
#define	PDB_FOLLOW	0x0001
#define	PDB_INIT	0x0002
#define	PDB_ENTER	0x0004
#define	PDB_REMOVE	0x0008
#define	PDB_CREATE	0x0010
#define	PDB_PTPAGE	0x0020
#define	PDB_GROWKERN	0x0040
#define	PDB_BITS	0x0080
#define	PDB_COLLECT	0x0100
#define	PDB_PROTECT	0x0200
#define	PDB_MAP_L1	0x0400
#define	PDB_BOOTSTRAP	0x1000
#define	PDB_PARANOIA	0x2000
#define	PDB_WIRING	0x4000
#define	PDB_PVDUMP	0x8000
#define	PDB_VAC		0x10000
#define	PDB_KENTER	0x20000
#define	PDB_KREMOVE	0x40000
#define	PDB_EXEC	0x80000

int debugmap = 1;
int pmapdebug = 0;
#define	NPDEBUG(_lev_,_stat_) \
	if (pmapdebug & (_lev_)) \
        	((_stat_))

#else	/* PMAP_DEBUG */
#define NPDEBUG(_lev_,_stat_) /* Nothing */
#endif	/* PMAP_DEBUG */
#endif

#ifdef VERBOSE_INIT_ARM
#define VPRINTF(...)	printf(__VA_ARGS__)
#else
#define VPRINTF(...)	__nothing
#endif



void	pmap_md_init(void);

static void	pmap_md_vca_page_wbinv(struct vm_page *, bool);



#if 0

// Global
pmap_bootstrap(void)

pmap_md_page_syncicache(struct vm_page *pg, const kcpuset_t *onproc)

pmap_procwr(struct proc *p, vaddr_t va, size_t len)

pmap_zero_page(paddr_t dst_pa)
pmap_copy_page(paddr_t src_pa, paddr_t dst_pa)

// Global - hmm
pmap_md_icache_sync_range_index(vaddr_t va, vsize_t len)
pmap_md_icache_sync_all(void)

// Arch specific
pmap_md_alloc_ephemeral_address_space(struct cpu_info *ci)


// Local
pmap_md_map_ephemeral_page(struct vm_page *pg, bool locked_p, int prot,
pmap_md_unmap_ephemeral_page(struct vm_page *pg, bool locked_p, register_t va,
pmap_md_vca_page_wbinv(struct vm_page *pg, bool locked_p)


pmap_md_tlb_info_attach(struct pmap_tlb_info *ti, struct cpu_info *ci)
pmap_md_tlb_check_entry(void *ctx, vaddr_t va, tlb_asid_t asid, pt_entry_t pte)
pmap_md_vca_add(struct vm_page *pg, vaddr_t va, pt_entry_t *ptep)
pmap_md_vca_clean(struct vm_page *pg, int op)
pmap_md_vca_remove(struct vm_page *pg, vaddr_t va, bool dirty, bool last)
pmap_md_pool_vtophys(vaddr_t va)
pmap_md_pool_phystov(paddr_t pa)

#endif

#if 0
/*
 * pmap copy/zero page, and mem(5) hook point
 */
static pt_entry_t *csrc_pte, *cdst_pte;
static vaddr_t csrcp, cdstp;
#ifdef MULTIPROCESSOR
static size_t cnptes;
#define	cpu_csrc_pte(o)	(csrc_pte + cnptes * cpu_number() + ((o) >> L2_S_SHIFT))
#define	cpu_cdst_pte(o)	(cdst_pte + cnptes * cpu_number() + ((o) >> L2_S_SHIFT))
#define	cpu_csrcp(o)	(csrcp + L2_S_SIZE * cnptes * cpu_number() + (o))
#define	cpu_cdstp(o)	(cdstp + L2_S_SIZE * cnptes * cpu_number() + (o))
#else
#define	cpu_csrc_pte(o)	(csrc_pte + ((o) >> L2_S_SHIFT))
#define	cpu_cdst_pte(o)	(cdst_pte + ((o) >> L2_S_SHIFT))
#define	cpu_csrcp(o)	(csrcp + (o))
#define	cpu_cdstp(o)	(cdstp + (o))
#endif


/*
 * We try to map the page tables write-through, if possible.  However, not
 * all CPUs have a write-through cache mode, so on those we have to sync
 * the cache when we frob page tables.
 *
 * We try to evaluate this at compile time, if possible.  However, it's
 * not always possible to do that, hence this run-time var.
 */
int	pmap_needs_pte_sync;

#endif


/*
 * Misc variables
 */
vaddr_t virtual_avail;
vaddr_t virtual_end;
vaddr_t pmap_curmaxkvaddr;




#if 0
paddr_t avail_start;
paddr_t avail_end;

pv_addr_t kernelpages;
pv_addr_t kernel_l1pt;
pv_addr_t systempage;

/*
 * PTE_SYNC_CURRENT:
 *
 *     Make sure the pte is written out to RAM.
 *     We need to do this for one of two cases:
 *       - We're dealing with the kernel pmap
 *       - There is no pmap active in the cache/tlb.
 *       - The specified pmap is 'active' in the cache/tlb.
 */

static inline void
pmap_pte_sync_current(pmap_t pm, pt_entry_t *ptep)
{
	if (PMAP_NEEDS_PTE_SYNC && pmap_is_cached(pm))
		PTE_SYNC(ptep);
	arm_dsb();
}

#ifdef PMAP_INCLUDE_PTE_SYNC
#define	PTE_SYNC_CURRENT(pm, ptep)	pmap_pte_sync_current(pm, ptep)
#else
#define	PTE_SYNC_CURRENT(pm, ptep)	/* nothing */
#endif





#endif













#if 0
/*
 * pmap_kernel() points here
 */
static struct pmap	kernel_pmap_store;
struct pmap * const	kernel_pmap_ptr = &kernel_pmap_store;
#undef pmap_kernel
#define pmap_kernel()	(&kernel_pmap_store)
#ifdef PMAP_NEED_ALLOC_POOLPAGE
int			arm_poolpage_vmfreelist = VM_FREELIST_DEFAULT;
#endif

/*
 * Pool and cache that pmap structures are allocated from.
 * We use a cache to avoid clearing the pm_l2[] array (1KB)
 * in pmap_create().
 */
static struct pool_cache pmap_cache;

/*
 * Pool of PV structures
 */
static struct pool pmap_pv_pool;
static void *pmap_bootstrap_pv_page_alloc(struct pool *, int);
static void pmap_bootstrap_pv_page_free(struct pool *, void *);
static struct pool_allocator pmap_bootstrap_pv_allocator = {
	pmap_bootstrap_pv_page_alloc, pmap_bootstrap_pv_page_free
};

/*
 * Pool and cache of l2_dtable structures.
 * We use a cache to avoid clearing the structures when they're
 * allocated. (196 bytes)
 */
static struct pool_cache pmap_l2dtable_cache;
static vaddr_t pmap_kernel_l2dtable_kva;

/*
 * Pool and cache of L2 page descriptors.
 * We use a cache to avoid clearing the descriptor table
 * when they're allocated. (1KB)
 */
static struct pool_cache pmap_l2ptp_cache;
static vaddr_t pmap_kernel_l2ptp_kva;
static paddr_t pmap_kernel_l2ptp_phys;

#ifdef PMAPCOUNTERS
#define	PMAP_EVCNT_INITIALIZER(name) \
	EVCNT_INITIALIZER(EVCNT_TYPE_MISC, NULL, "pmap", name)

#if defined(PMAP_CACHE_VIPT) && !defined(ARM_MMU_EXTENDED)
static struct evcnt pmap_ev_vac_clean_one =
   PMAP_EVCNT_INITIALIZER("clean page (1 color)");
static struct evcnt pmap_ev_vac_flush_one =
   PMAP_EVCNT_INITIALIZER("flush page (1 color)");
static struct evcnt pmap_ev_vac_flush_lots =
   PMAP_EVCNT_INITIALIZER("flush page (2+ colors)");
static struct evcnt pmap_ev_vac_flush_lots2 =
   PMAP_EVCNT_INITIALIZER("flush page (2+ colors, kmpage)");
EVCNT_ATTACH_STATIC(pmap_ev_vac_clean_one);
EVCNT_ATTACH_STATIC(pmap_ev_vac_flush_one);
EVCNT_ATTACH_STATIC(pmap_ev_vac_flush_lots);
EVCNT_ATTACH_STATIC(pmap_ev_vac_flush_lots2);

static struct evcnt pmap_ev_vac_color_new =
   PMAP_EVCNT_INITIALIZER("new page color");
static struct evcnt pmap_ev_vac_color_reuse =
   PMAP_EVCNT_INITIALIZER("ok first page color");
static struct evcnt pmap_ev_vac_color_ok =
   PMAP_EVCNT_INITIALIZER("ok page color");
static struct evcnt pmap_ev_vac_color_blind =
   PMAP_EVCNT_INITIALIZER("blind page color");
static struct evcnt pmap_ev_vac_color_change =
   PMAP_EVCNT_INITIALIZER("change page color");
static struct evcnt pmap_ev_vac_color_erase =
   PMAP_EVCNT_INITIALIZER("erase page color");
static struct evcnt pmap_ev_vac_color_none =
   PMAP_EVCNT_INITIALIZER("no page color");
static struct evcnt pmap_ev_vac_color_restore =
   PMAP_EVCNT_INITIALIZER("restore page color");

EVCNT_ATTACH_STATIC(pmap_ev_vac_color_new);
EVCNT_ATTACH_STATIC(pmap_ev_vac_color_reuse);
EVCNT_ATTACH_STATIC(pmap_ev_vac_color_ok);
EVCNT_ATTACH_STATIC(pmap_ev_vac_color_blind);
EVCNT_ATTACH_STATIC(pmap_ev_vac_color_change);
EVCNT_ATTACH_STATIC(pmap_ev_vac_color_erase);
EVCNT_ATTACH_STATIC(pmap_ev_vac_color_none);
EVCNT_ATTACH_STATIC(pmap_ev_vac_color_restore);
#endif

static struct evcnt pmap_ev_mappings =
   PMAP_EVCNT_INITIALIZER("pages mapped");
static struct evcnt pmap_ev_unmappings =
   PMAP_EVCNT_INITIALIZER("pages unmapped");
static struct evcnt pmap_ev_remappings =
   PMAP_EVCNT_INITIALIZER("pages remapped");

EVCNT_ATTACH_STATIC(pmap_ev_mappings);
EVCNT_ATTACH_STATIC(pmap_ev_unmappings);
EVCNT_ATTACH_STATIC(pmap_ev_remappings);

static struct evcnt pmap_ev_kernel_mappings =
   PMAP_EVCNT_INITIALIZER("kernel pages mapped");
static struct evcnt pmap_ev_kernel_unmappings =
   PMAP_EVCNT_INITIALIZER("kernel pages unmapped");
static struct evcnt pmap_ev_kernel_remappings =
   PMAP_EVCNT_INITIALIZER("kernel pages remapped");

EVCNT_ATTACH_STATIC(pmap_ev_kernel_mappings);
EVCNT_ATTACH_STATIC(pmap_ev_kernel_unmappings);
EVCNT_ATTACH_STATIC(pmap_ev_kernel_remappings);

static struct evcnt pmap_ev_kenter_mappings =
   PMAP_EVCNT_INITIALIZER("kenter pages mapped");
static struct evcnt pmap_ev_kenter_unmappings =
   PMAP_EVCNT_INITIALIZER("kenter pages unmapped");
static struct evcnt pmap_ev_kenter_remappings =
   PMAP_EVCNT_INITIALIZER("kenter pages remapped");
static struct evcnt pmap_ev_pt_mappings =
   PMAP_EVCNT_INITIALIZER("page table pages mapped");

EVCNT_ATTACH_STATIC(pmap_ev_kenter_mappings);
EVCNT_ATTACH_STATIC(pmap_ev_kenter_unmappings);
EVCNT_ATTACH_STATIC(pmap_ev_kenter_remappings);
EVCNT_ATTACH_STATIC(pmap_ev_pt_mappings);

static struct evcnt pmap_ev_fixup_mod =
   PMAP_EVCNT_INITIALIZER("page modification emulations");
static struct evcnt pmap_ev_fixup_ref =
   PMAP_EVCNT_INITIALIZER("page reference emulations");
static struct evcnt pmap_ev_fixup_exec =
   PMAP_EVCNT_INITIALIZER("exec pages fixed up");
static struct evcnt pmap_ev_fixup_pdes =
   PMAP_EVCNT_INITIALIZER("pdes fixed up");

EVCNT_ATTACH_STATIC(pmap_ev_fixup_mod);
EVCNT_ATTACH_STATIC(pmap_ev_fixup_ref);
EVCNT_ATTACH_STATIC(pmap_ev_fixup_exec);
EVCNT_ATTACH_STATIC(pmap_ev_fixup_pdes);

#ifdef PMAP_CACHE_VIPT
static struct evcnt pmap_ev_exec_mappings =
   PMAP_EVCNT_INITIALIZER("exec pages mapped");
static struct evcnt pmap_ev_exec_cached =
   PMAP_EVCNT_INITIALIZER("exec pages cached");

EVCNT_ATTACH_STATIC(pmap_ev_exec_mappings);
EVCNT_ATTACH_STATIC(pmap_ev_exec_cached);

static struct evcnt pmap_ev_exec_synced =
   PMAP_EVCNT_INITIALIZER("exec pages synced");
static struct evcnt pmap_ev_exec_synced_map =
   PMAP_EVCNT_INITIALIZER("exec pages synced (MP)");

EVCNT_ATTACH_STATIC(pmap_ev_exec_synced);
EVCNT_ATTACH_STATIC(pmap_ev_exec_synced_map);

static struct evcnt pmap_ev_exec_discarded_unmap =
   PMAP_EVCNT_INITIALIZER("exec pages discarded (UM)");
static struct evcnt pmap_ev_exec_discarded_zero =
   PMAP_EVCNT_INITIALIZER("exec pages discarded (ZP)");
static struct evcnt pmap_ev_exec_discarded_copy =
   PMAP_EVCNT_INITIALIZER("exec pages discarded (CP)");
static struct evcnt pmap_ev_exec_discarded_page_protect =
   PMAP_EVCNT_INITIALIZER("exec pages discarded (PP)");
static struct evcnt pmap_ev_exec_discarded_clearbit =
   PMAP_EVCNT_INITIALIZER("exec pages discarded (DG)");
static struct evcnt pmap_ev_exec_discarded_kremove =
   PMAP_EVCNT_INITIALIZER("exec pages discarded (KU)");
static struct evcnt pmap_ev_exec_discarded_modfixup =
   PMAP_EVCNT_INITIALIZER("exec pages discarded (MF)");

EVCNT_ATTACH_STATIC(pmap_ev_exec_discarded_unmap);
EVCNT_ATTACH_STATIC(pmap_ev_exec_discarded_zero);
EVCNT_ATTACH_STATIC(pmap_ev_exec_discarded_copy);
EVCNT_ATTACH_STATIC(pmap_ev_exec_discarded_page_protect);
EVCNT_ATTACH_STATIC(pmap_ev_exec_discarded_clearbit);
EVCNT_ATTACH_STATIC(pmap_ev_exec_discarded_kremove);
EVCNT_ATTACH_STATIC(pmap_ev_exec_discarded_modfixup);
#endif /* PMAP_CACHE_VIPT */

static struct evcnt pmap_ev_updates = PMAP_EVCNT_INITIALIZER("updates");
static struct evcnt pmap_ev_collects = PMAP_EVCNT_INITIALIZER("collects");
static struct evcnt pmap_ev_activations = PMAP_EVCNT_INITIALIZER("activations");

EVCNT_ATTACH_STATIC(pmap_ev_updates);
EVCNT_ATTACH_STATIC(pmap_ev_collects);
EVCNT_ATTACH_STATIC(pmap_ev_activations);

#define	PMAPCOUNT(x)	((void)(pmap_ev_##x.ev_count++))
#else
#define	PMAPCOUNT(x)	((void)0)
#endif

/*
 * mem(5) hook point
 */
vaddr_t memhook;			/* used by mem.c & others */
kmutex_t memlock __cacheline_aligned;	/* used by mem.c & others */
kmutex_t pmap_lock __cacheline_aligned;
extern void *msgbufaddr;
int pmap_kmpages;
/*
 * Flag to indicate if pmap_init() has done its thing
 */
bool pmap_initialized;

#endif

#if 1 || defined(__HAVE_MM_MD_DIRECT_MAPPED_PHYS)
/*
 * Virtual end of direct-mapped memory
 */
vaddr_t pmap_directlimit;
#endif

#if 0
/*
 * Misc. locking data structures
 */

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
 * Metadata for L1 translation tables.
 */

/*
 * The l2_dtable tracks L2_BUCKET_SIZE worth of L1 slots.
 *
 * This is normally 16MB worth L2 page descriptors for any given pmap.
 * Reference counts are maintained for L2 descriptors so they can be
 * freed when empty.
 */
struct l2_bucket {
	pt_entry_t *l2b_kva;		/* KVA of L2 Descriptor Table */
	paddr_t l2b_pa;			/* Physical address of same */
	u_short l2b_l1slot;		/* This L2 table's L1 index */
	u_short l2b_occupancy;		/* How many active descriptors */
};

struct l2_dtable {
	/* The number of L2 page descriptors allocated to this l2_dtable */
	u_int l2_occupancy;

	/* List of L2 page descriptors */
	struct l2_bucket l2_bucket[L2_BUCKET_SIZE];
};

/*
 * Given an L1 table index, calculate the corresponding l2_dtable index
 * and bucket index within the l2_dtable.
 */
#define L2_BUCKET_XSHIFT	(L2_BUCKET_XLOG2 - L1_S_SHIFT)
#define L2_BUCKET_XFRAME	(~(vaddr_t)0 << L2_BUCKET_XLOG2)
#define L2_BUCKET_IDX(l1slot)	((l1slot) >> L2_BUCKET_XSHIFT)
#define L2_IDX(l1slot)		(L2_BUCKET_IDX(l1slot) >> L2_BUCKET_LOG2)
#define L2_BUCKET(l1slot)	(L2_BUCKET_IDX(l1slot) & (L2_BUCKET_SIZE - 1))

__CTASSERT(0x100000000ULL == ((uint64_t)L2_SIZE * L2_BUCKET_SIZE * L1_S_SIZE));
__CTASSERT(L2_BUCKET_XFRAME == ~(L2_BUCKET_XSIZE-1));

/*
 * Given a virtual address, this macro returns the
 * virtual address required to drop into the next L2 bucket.
 */
#define	L2_NEXT_BUCKET_VA(va)	(((va) & L2_BUCKET_XFRAME) + L2_BUCKET_XSIZE)

/*
 * L2 allocation.
 */
#define	pmap_alloc_l2_dtable()		\
	    pool_cache_get(&pmap_l2dtable_cache, PR_NOWAIT)
#define	pmap_free_l2_dtable(l2)		\
	    pool_cache_put(&pmap_l2dtable_cache, (l2))
#define pmap_alloc_l2_ptp(pap)		\
	    ((pt_entry_t *)pool_cache_get_paddr(&pmap_l2ptp_cache,\
	    PR_NOWAIT, (pap)))

/*
 * Real definition of pv_entry.
 */
struct pv_entry {
	SLIST_ENTRY(pv_entry) pv_link;	/* next pv_entry */
	pmap_t		pv_pmap;        /* pmap where mapping lies */
	vaddr_t		pv_va;          /* virtual address for mapping */
	u_int		pv_flags;       /* flags */
};

#if 0
/*
 * Macros to determine if a mapping might be resident in the
 * instruction/data cache and/or TLB
 */
#define	PV_BEEN_EXECD(f)  (((f) & (PVF_REF | PVF_EXEC)) == (PVF_REF | PVF_EXEC))
#define	PV_BEEN_REFD(f)   (((f) & PVF_REF) != 0)
#define	PV_IS_EXEC_P(f)   (((f) & PVF_EXEC) != 0)
#define	PV_IS_KENTRY_P(f) (((f) & PVF_KENTRY) != 0)
#define	PV_IS_WRITE_P(f)  (((f) & PVF_WRITE) != 0)

#endif

/*
 * Local prototypes
 */
static bool		pmap_set_pt_cache_mode(pd_entry_t *, vaddr_t, size_t);
static void		pmap_alloc_specials(vaddr_t *, int, vaddr_t *,
			    pt_entry_t **);
static bool		pmap_is_current(pmap_t) __unused;
static bool		pmap_is_cached(pmap_t);
static void		pmap_enter_pv(struct vm_page_md *, paddr_t, struct pv_entry *,
			    pmap_t, vaddr_t, u_int);
static struct pv_entry *pmap_find_pv(struct vm_page_md *, pmap_t, vaddr_t);
static struct pv_entry *pmap_remove_pv(struct vm_page_md *, paddr_t, pmap_t, vaddr_t);
static u_int		pmap_modify_pv(struct vm_page_md *, paddr_t, pmap_t, vaddr_t,
			    u_int, u_int);

static void		pmap_pinit(pmap_t);
static int		pmap_pmap_ctor(void *, void *, int);

static void		pmap_alloc_l1(pmap_t);
static void		pmap_free_l1(pmap_t);

static struct l2_bucket *pmap_get_l2_bucket(pmap_t, vaddr_t);
static struct l2_bucket *pmap_alloc_l2_bucket(pmap_t, vaddr_t);
static void		pmap_free_l2_bucket(pmap_t, struct l2_bucket *, u_int);
static int		pmap_l2ptp_ctor(void *, void *, int);
static int		pmap_l2dtable_ctor(void *, void *, int);

static void		pmap_vac_me_harder(struct vm_page_md *, paddr_t, pmap_t, vaddr_t);
#ifdef PMAP_CACHE_VIVT
static void		pmap_vac_me_kpmap(struct vm_page_md *, paddr_t, pmap_t, vaddr_t);
static void		pmap_vac_me_user(struct vm_page_md *, paddr_t, pmap_t, vaddr_t);
#endif

static void		pmap_clearbit(struct vm_page_md *, paddr_t, u_int);
#ifdef PMAP_CACHE_VIVT
static bool		pmap_clean_page(struct vm_page_md *, bool);
#endif
#ifdef PMAP_CACHE_VIPT
//static void		pmap_syncicache_page(struct vm_page_md *, paddr_t);
void			pmap_md_page_syncicache(struct vm_page *pg, const kcpuset_t *onproc)
enum pmap_flush_op {
	PMAP_FLUSH_PRIMARY,
	PMAP_FLUSH_SECONDARY,
	PMAP_CLEAN_PRIMARY
};
#endif
static void		pmap_page_remove(struct vm_page_md *, paddr_t);
static void		pmap_pv_remove(paddr_t);

static vaddr_t		kernel_pt_lookup(paddr_t);

pv_addrqh_t pmap_boot_freeq = SLIST_HEAD_INITIALIZER(&pmap_boot_freeq);
/* Function to set the debug level of the pmap code */

#ifdef PMAP_DEBUG
void
pmap_debug(int level)
{
	pmap_debug_level = level;
	printf("pmap_debug: level=%d\n", pmap_debug_level);
}
#endif	/* PMAP_DEBUG */

#ifdef PMAP_CACHE_VIPT
#define PMAP_VALIDATE_MD_PAGE(md)	\
	KASSERTMSG(arm_cache_prefer_mask == 0 || (((md)->pvh_attrs & PVF_WRITE) == 0) == ((md)->urw_mappings + (md)->krw_mappings == 0), \
	    "(md) %p: attrs=%#x urw=%u krw=%u", (md), \
	    (md)->pvh_attrs, (md)->urw_mappings, (md)->krw_mappings);
#endif /* PMAP_CACHE_VIPT */

#ifdef PMAP_CACHE_VIVT
static inline void
pmap_cache_wbinv_page(pmap_t pm, vaddr_t va, bool do_inv, u_int flags)
{
	if (PV_BEEN_EXECD(flags) && pm->pm_cstate.cs_cache_id) {
		cpu_idcache_wbinv_range(va, PAGE_SIZE);
	} else if (PV_BEEN_REFD(flags) && pm->pm_cstate.cs_cache_d) {
		if (do_inv) {
			if (flags & PVF_WRITE)
				cpu_dcache_wbinv_range(va, PAGE_SIZE);
			else
				cpu_dcache_inv_range(va, PAGE_SIZE);
		} else if (flags & PVF_WRITE) {
			cpu_dcache_wb_range(va, PAGE_SIZE);
		}
	}
}

static inline void
pmap_cache_wbinv_all(pmap_t pm, u_int flags)
{
	if (PV_BEEN_EXECD(flags)) {
		if (pm->pm_cstate.cs_cache_id) {
			cpu_idcache_wbinv_all();
			pm->pm_cstate.cs_cache = 0;
		}
	} else if (pm->pm_cstate.cs_cache_d) {
		cpu_dcache_wbinv_all();
		pm->pm_cstate.cs_cache_d = 0;
	}
}
#endif /* PMAP_CACHE_VIVT */

static inline uint8_t
pmap_domain(pmap_t pm)
{
	return pm == pmap_kernel() ? PMAP_DOMAIN_KERNEL : PMAP_DOMAIN_USER;
}

static inline pd_entry_t *
pmap_l1_kva(pmap_t pm)
{
	return pm->pm_l1;
}

static inline bool
pmap_is_current(pmap_t pm)
{
	if (pm == pmap_kernel() || curproc->p_vmspace->vm_map.pmap == pm)
		return true;

	return false;
}

static inline bool
pmap_is_cached(pmap_t pm)
{
	if (pm == pmap_kernel())
		return true;
#ifdef MULTIPROCESSOR
	// Is this pmap active on any CPU?
	if (!kcpuset_iszero(pm->pm_active))
		return true;
#else
	struct pmap_tlb_info * const ti = cpu_tlb_info(curcpu());
	// Is this pmap active?
	if (PMAP_PAI_ASIDVALID_P(PMAP_PAI(pm, ti), ti))
		return true;
#endif

	return false;
}


/*
 * main pv_entry manipulation functions:
 *   pmap_enter_pv: enter a mapping onto a vm_page list
 *   pmap_remove_pv: remove a mapping from a vm_page list
 *
 * NOTE: pmap_enter_pv expects to lock the pvh itself
 *       pmap_remove_pv expects the caller to lock the pvh before calling
 */

/*
 * pmap_enter_pv: enter a mapping onto a vm_page lst
 *
 * => caller should hold the proper lock on pmap_main_lock
 * => caller should have pmap locked
 * => we will gain the lock on the vm_page and allocate the new pv_entry
 * => caller should adjust ptp's wire_count before calling
 * => caller should not adjust pmap's wire_count
 */
static void
pmap_enter_pv(struct vm_page_md *md, paddr_t pa, struct pv_entry *pv, pmap_t pm,
    vaddr_t va, u_int flags)
{
	struct pv_entry **pvp;

	NPDEBUG(PDB_PVDUMP,
	    printf("pmap_enter_pv: pm %p, md %p, flags 0x%x\n", pm, md, flags));

	pv->pv_pmap = pm;
	pv->pv_va = va;
	pv->pv_flags = flags;

	pvp = &SLIST_FIRST(&md->pvh_list);
#ifdef PMAP_CACHE_VIPT
	/*
	 * Insert unmanaged entries, writeable first, at the head of
	 * the pv list.
	 */
	if (__predict_true(!PV_IS_KENTRY_P(flags))) {
		while (*pvp != NULL && PV_IS_KENTRY_P((*pvp)->pv_flags))
			pvp = &SLIST_NEXT(*pvp, pv_link);
	}
	if (!PV_IS_WRITE_P(flags)) {
		while (*pvp != NULL && PV_IS_WRITE_P((*pvp)->pv_flags))
			pvp = &SLIST_NEXT(*pvp, pv_link);
	}
#endif
	SLIST_NEXT(pv, pv_link) = *pvp;		/* add to ... */
	*pvp = pv;				/* ... locked list */
	md->pvh_attrs |= flags & (PVF_REF | PVF_MOD);

	if (pm == pmap_kernel()) {
		PMAPCOUNT(kernel_mappings);
		if (flags & PVF_WRITE)
			md->krw_mappings++;
		else
			md->kro_mappings++;
	} else {
		if (flags & PVF_WRITE)
			md->urw_mappings++;
		else
			md->uro_mappings++;
	}

#ifdef PMAP_CACHE_VIPT
	/*
	 * If this is an exec mapping and its the first exec mapping
	 * for this page, make sure to sync the I-cache.
	 */
	if (PV_IS_EXEC_P(flags)) {
		/*XXX is adding this correct??? */
		if (!PV_IS_EXEC_P(md->pvh_attrs)) {
			pmap_syncicache_page(md, pa);
			PMAPCOUNT(exec_synced_map);
		}
		PMAPCOUNT(exec_mappings);
	}
#endif

	PMAPCOUNT(mappings);

	if (pv->pv_flags & PVF_WIRED)
		++pm->pm_stats.wired_count;
}

/*
 *
 * pmap_find_pv: Find a pv entry
 *
 * => caller should hold lock on vm_page
 */
static inline struct pv_entry *
pmap_find_pv(struct vm_page_md *md, pmap_t pm, vaddr_t va)
{
	struct pv_entry *pv;

	SLIST_FOREACH(pv, &md->pvh_list, pv_link) {
		if (pm == pv->pv_pmap && va == pv->pv_va)
			break;
	}

	return (pv);
}

/*
 * pmap_remove_pv: try to remove a mapping from a pv_list
 *
 * => caller should hold proper lock on pmap_main_lock
 * => pmap should be locked
 * => caller should hold lock on vm_page [so that attrs can be adjusted]
 * => caller should adjust ptp's wire_count and free PTP if needed
 * => caller should NOT adjust pmap's wire_count
 * => we return the removed pv
 */
static struct pv_entry *
pmap_remove_pv(struct vm_page_md *md, paddr_t pa, pmap_t pm, vaddr_t va)
{
	struct pv_entry *pv, **prevptr;

	NPDEBUG(PDB_PVDUMP,
	    printf("pmap_remove_pv: pm %p, md %p, va 0x%08lx\n", pm, md, va));

	prevptr = &SLIST_FIRST(&md->pvh_list); /* prev pv_entry ptr */
	pv = *prevptr;

	while (pv) {
		if (pv->pv_pmap == pm && pv->pv_va == va) {	/* match? */
			NPDEBUG(PDB_PVDUMP, printf("pmap_remove_pv: pm %p, md "
			    "%p, flags 0x%x\n", pm, md, pv->pv_flags));
			if (pv->pv_flags & PVF_WIRED) {
				--pm->pm_stats.wired_count;
			}
			*prevptr = SLIST_NEXT(pv, pv_link);	/* remove it! */
			if (pm == pmap_kernel()) {
				PMAPCOUNT(kernel_unmappings);
				if (pv->pv_flags & PVF_WRITE)
					md->krw_mappings--;
				else
					md->kro_mappings--;
			} else {
				if (pv->pv_flags & PVF_WRITE)
					md->urw_mappings--;
				else
					md->uro_mappings--;
			}

			PMAPCOUNT(unmappings);
#ifdef PMAP_CACHE_VIPT
			/*
			 * If this page has had an exec mapping, then if
			 * this was the last mapping, discard the contents,
			 * otherwise sync the i-cache for this page.
			 */
			if (PV_IS_EXEC_P(md->pvh_attrs)) {
				if (SLIST_EMPTY(&md->pvh_list)) {
					md->pvh_attrs &= ~PVF_EXEC;
					PMAPCOUNT(exec_discarded_unmap);
				} else if (pv->pv_flags & PVF_WRITE) {
					pmap_syncicache_page(md, pa);
					PMAPCOUNT(exec_synced_unmap);
				}
			}
#endif /* PMAP_CACHE_VIPT */
			break;
		}
		prevptr = &SLIST_NEXT(pv, pv_link);	/* previous pointer */
		pv = *prevptr;				/* advance */
	}


	return(pv);				/* return removed pv */
}

/*
 *
 * pmap_modify_pv: Update pv flags
 *
 * => caller should hold lock on vm_page [so that attrs can be adjusted]
 * => caller should NOT adjust pmap's wire_count
 * => caller must call pmap_vac_me_harder() if writable status of a page
 *    may have changed.
 * => we return the old flags
 *
 * Modify a physical-virtual mapping in the pv table
 */
static u_int
pmap_modify_pv(struct vm_page_md *md, paddr_t pa, pmap_t pm, vaddr_t va,
    u_int clr_mask, u_int set_mask)
{
	struct pv_entry *npv;
	u_int flags, oflags;

	KASSERT(!PV_IS_KENTRY_P(clr_mask));
	KASSERT(!PV_IS_KENTRY_P(set_mask));

	if ((npv = pmap_find_pv(md, pm, va)) == NULL)
		return (0);

	NPDEBUG(PDB_PVDUMP,
	    printf("pmap_modify_pv: pm %p, md %p, clr 0x%x, set 0x%x, flags 0x%x\n", pm, md, clr_mask, set_mask, npv->pv_flags));

	/*
	 * There is at least one VA mapping this page.
	 */

	if (clr_mask & (PVF_REF | PVF_MOD)) {
		md->pvh_attrs |= set_mask & (PVF_REF | PVF_MOD);
#if defined(PMAP_CACHE_VIPT) && !defined(ARM_MMU_EXTENDED)
		if ((md->pvh_attrs & (PVF_DMOD|PVF_NC)) != PVF_NC)
			md->pvh_attrs |= PVF_DIRTY;
		KASSERT((md->pvh_attrs & PVF_DMOD) == 0 || (md->pvh_attrs & (PVF_DIRTY|PVF_NC)));
#endif /* PMAP_CACHE_VIPT && !ARM_MMU_EXTENDED */
	}

	oflags = npv->pv_flags;
	npv->pv_flags = flags = (oflags & ~clr_mask) | set_mask;

	if ((flags ^ oflags) & PVF_WIRED) {
		if (flags & PVF_WIRED)
			++pm->pm_stats.wired_count;
		else
			--pm->pm_stats.wired_count;
	}

	if ((flags ^ oflags) & PVF_WRITE) {
		if (pm == pmap_kernel()) {
			if (flags & PVF_WRITE) {
				md->krw_mappings++;
				md->kro_mappings--;
			} else {
				md->kro_mappings++;
				md->krw_mappings--;
			}
		} else {
			if (flags & PVF_WRITE) {
				md->urw_mappings++;
				md->uro_mappings--;
			} else {
				md->uro_mappings++;
				md->urw_mappings--;
			}
		}
	}
#ifdef PMAP_CACHE_VIPT
	if (arm_cache_prefer_mask != 0) {
		if (md->urw_mappings + md->krw_mappings == 0) {
			md->pvh_attrs &= ~PVF_WRITE;
		} else {
			md->pvh_attrs |= PVF_WRITE;
		}
	}
	/*
	 * We have two cases here: the first is from enter_pv (new exec
	 * page), the second is a combined pmap_remove_pv/pmap_enter_pv.
	 * Since in latter, pmap_enter_pv won't do anything, we just have
	 * to do what pmap_remove_pv would do.
	 */
	if ((PV_IS_EXEC_P(flags) && !PV_IS_EXEC_P(md->pvh_attrs))
	    || (PV_IS_EXEC_P(md->pvh_attrs)
		|| (!(flags & PVF_WRITE) && (oflags & PVF_WRITE)))) {
		pmap_syncicache_page(md, pa);
		PMAPCOUNT(exec_synced_remap);
	}
#endif /* PMAP_CACHE_VIPT */

	PMAPCOUNT(remappings);

	return (oflags);
}

#endif






/*
 * Allocate an L1 translation table for the specified pmap.
 * This is called at pmap creation time.
 */
static void
pmap_alloc_l1(pmap_t pm)
{
#ifdef __HAVE_MM_MD_DIRECT_MAPPED_PHYS
	struct vm_page *pg;
	bool ok __diagused;
	for (;;) {
#ifdef PMAP_NEED_ALLOC_POOLPAGE
		pg = pmap_md_alloc_poolpage(UVM_PGA_ZERO);
#else
		pg = uvm_pagealloc(NULL, 0, NULL, UVM_PGA_ZERO);
#endif
		if (pg != NULL)
			break;
		uvm_wait("pmapl1alloc");
	}
	pm->pm_l1_pa = VM_PAGE_TO_PHYS(pg);
	vaddr_t va = pmap_direct_mapped_phys(pm->pm_l1_pa, &ok, 0);
	KASSERT(ok);
	KASSERT(va >= KERNEL_BASE);

#else
	KASSERTMSG(kernel_map != NULL, "pm %p", pm);
	vaddr_t va = uvm_km_alloc(kernel_map, PAGE_SIZE, 0,
	    UVM_KMF_WIRED|UVM_KMF_ZERO);
	KASSERT(va);
	pmap_extract(pmap_kernel(), va, &pm->pm_l1_pa);
#endif
	pm->pm_l1 = (pd_entry_t *)va;
	PTE_SYNC_RANGE(pm->pm_l1, PAGE_SIZE / sizeof(pt_entry_t));
}

/*
 * Free an L1 translation table.
 * This is called at pmap destruction time.
 */
static void
pmap_free_l1(pmap_t pm)
{
#ifdef __HAVE_MM_MD_DIRECT_MAPPED_PHYS
	struct vm_page *pg = PHYS_TO_VM_PAGE(pm->pm_l1_pa);
	uvm_pagefree(pg);
#else
	uvm_km_free(kernel_map, (vaddr_t)pm->pm_l1, PAGE_SIZE, UVM_KMF_WIRED);
#endif
	pm->pm_l1 = NULL;
	pm->pm_l1_pa = 0;
}




#if 0

/*
 * void pmap_free_l2_ptp(pt_entry_t *, paddr_t *)
 *
 * Free an L2 descriptor table.
 */
static inline void
#if defined(PMAP_INCLUDE_PTE_SYNC) && defined(PMAP_CACHE_VIVT)
pmap_free_l2_ptp(bool need_sync, pt_entry_t *l2, paddr_t pa)
#else
pmap_free_l2_ptp(pt_entry_t *l2, paddr_t pa)
#endif
{
#if defined(PMAP_INCLUDE_PTE_SYNC) && defined(PMAP_CACHE_VIVT)
	/*
	 * Note: With a write-back cache, we may need to sync this
	 * L2 table before re-using it.
	 * This is because it may have belonged to a non-current
	 * pmap, in which case the cache syncs would have been
	 * skipped for the pages that were being unmapped. If the
	 * L2 table were then to be immediately re-allocated to
	 * the *current* pmap, it may well contain stale mappings
	 * which have not yet been cleared by a cache write-back
	 * and so would still be visible to the mmu.
	 */
	if (need_sync)
		PTE_SYNC_RANGE(l2, L2_TABLE_SIZE_REAL / sizeof(pt_entry_t));
#endif /* PMAP_INCLUDE_PTE_SYNC && PMAP_CACHE_VIVT */
	pool_cache_put_paddr(&pmap_l2ptp_cache, (void *)l2, pa);
}

/*
 * Returns a pointer to the L2 bucket associated with the specified pmap
 * and VA, or NULL if no L2 bucket exists for the address.
 */
static inline struct l2_bucket *
pmap_get_l2_bucket(pmap_t pm, vaddr_t va)
{
	const size_t l1slot = l1pte_index(va);
	struct l2_dtable *l2;
	struct l2_bucket *l2b;

	if ((l2 = pm->pm_l2[L2_IDX(l1slot)]) == NULL ||
	    (l2b = &l2->l2_bucket[L2_BUCKET(l1slot)])->l2b_kva == NULL)
		return (NULL);

	return (l2b);
}

/*
 * Returns a pointer to the L2 bucket associated with the specified pmap
 * and VA.
 *
 * If no L2 bucket exists, perform the necessary allocations to put an L2
 * bucket/page table in place.
 *
 * Note that if a new L2 bucket/page was allocated, the caller *must*
 * increment the bucket occupancy counter appropriately *before*
 * releasing the pmap's lock to ensure no other thread or cpu deallocates
 * the bucket/page in the meantime.
 */
static struct l2_bucket *
pmap_alloc_l2_bucket(pmap_t pm, vaddr_t va)
{
	const size_t l1slot = l1pte_index(va);
	struct l2_dtable *l2;

	if ((l2 = pm->pm_l2[L2_IDX(l1slot)]) == NULL) {
		/*
		 * No mapping at this address, as there is
		 * no entry in the L1 table.
		 * Need to allocate a new l2_dtable.
		 */
		if ((l2 = pmap_alloc_l2_dtable()) == NULL)
			return (NULL);

		/*
		 * Link it into the parent pmap
		 */
		pm->pm_l2[L2_IDX(l1slot)] = l2;
	}

	struct l2_bucket * const l2b = &l2->l2_bucket[L2_BUCKET(l1slot)];

	/*
	 * Fetch pointer to the L2 page table associated with the address.
	 */
	if (l2b->l2b_kva == NULL) {
		pt_entry_t *ptep;

		/*
		 * No L2 page table has been allocated. Chances are, this
		 * is because we just allocated the l2_dtable, above.
		 */
		if ((ptep = pmap_alloc_l2_ptp(&l2b->l2b_pa)) == NULL) {
			/*
			 * Oops, no more L2 page tables available at this
			 * time. We may need to deallocate the l2_dtable
			 * if we allocated a new one above.
			 */
			if (l2->l2_occupancy == 0) {
				pm->pm_l2[L2_IDX(l1slot)] = NULL;
				pmap_free_l2_dtable(l2);
			}
			return (NULL);
		}

		l2->l2_occupancy++;
		l2b->l2b_kva = ptep;
		l2b->l2b_l1slot = l1slot;

		/*
		 * We know there will be a mapping here, so simply
		 * enter this PTP into the L1 now.
		 */
		pd_entry_t * const pdep = pmap_l1_kva(pm) + l1slot;
		pd_entry_t npde = L1_C_PROTO | l2b->l2b_pa
		    | L1_C_DOM(pmap_domain(pm));
		KASSERT(*pdep == 0);
		l1pte_setone(pdep, npde);
		PDE_SYNC(pdep);
	}

	return (l2b);
}

/*
 * One or more mappings in the specified L2 descriptor table have just been
 * invalidated.
 *
 * Garbage collect the metadata and descriptor table itself if necessary.
 *
 * The pmap lock must be acquired when this is called (not necessary
 * for the kernel pmap).
 */
static void
pmap_free_l2_bucket(pmap_t pm, struct l2_bucket *l2b, u_int count)
{
	KDASSERT(count <= l2b->l2b_occupancy);

	/*
	 * Update the bucket's reference count according to how many
	 * PTEs the caller has just invalidated.
	 */
	l2b->l2b_occupancy -= count;

	/*
	 * Note:
	 *
	 * Level 2 page tables allocated to the kernel pmap are never freed
	 * as that would require checking all Level 1 page tables and
	 * removing any references to the Level 2 page table. See also the
	 * comment elsewhere about never freeing bootstrap L2 descriptors.
	 *
	 * We make do with just invalidating the mapping in the L2 table.
	 *
	 * This isn't really a big deal in practice and, in fact, leads
	 * to a performance win over time as we don't need to continually
	 * alloc/free.
	 */
	if (l2b->l2b_occupancy > 0 || pm == pmap_kernel())
		return;

	/*
	 * There are no more valid mappings in this level 2 page table.
	 * Go ahead and NULL-out the pointer in the bucket, then
	 * free the page table.
	 */
	const size_t l1slot = l2b->l2b_l1slot;
	pt_entry_t * const ptep = l2b->l2b_kva;
	l2b->l2b_kva = NULL;

	pd_entry_t * const pdep = pmap_l1_kva(pm) + l1slot;
	pd_entry_t pde __diagused = *pdep;

	/*
	 * Invalidate the L1 slot.
	 */
	KASSERT((pde & L1_TYPE_MASK) == L1_TYPE_C);
		l1pte_setone(pdep, 0);
		PDE_SYNC(pdep);

	/*
	 * Release the L2 descriptor table back to the pool cache.
	 */
#if defined(PMAP_INCLUDE_PTE_SYNC) && defined(PMAP_CACHE_VIVT)
	pmap_free_l2_ptp(!pmap_is_cached(pm), ptep, l2b->l2b_pa);
#else
	pmap_free_l2_ptp(ptep, l2b->l2b_pa);
#endif

	/*
	 * Update the reference count in the associated l2_dtable
	 */
	struct l2_dtable * const l2 = pm->pm_l2[L2_IDX(l1slot)];
	if (--l2->l2_occupancy > 0)
		return;

	/*
	 * There are no more valid mappings in any of the Level 1
	 * slots managed by this l2_dtable. Go ahead and NULL-out
	 * the pointer in the parent pmap and free the l2_dtable.
	 */
	pm->pm_l2[L2_IDX(l1slot)] = NULL;
	pmap_free_l2_dtable(l2);
}

/*
 * Pool cache constructors for L2 descriptor tables, metadata and pmap
 * structures.
 */
static int
pmap_l2ptp_ctor(void *arg, void *v, int flags)
{
#ifndef PMAP_INCLUDE_PTE_SYNC
	vaddr_t va = (vaddr_t)v & ~PGOFSET;

	/*
	 * The mappings for these page tables were initially made using
	 * pmap_kenter_pa() by the pool subsystem. Therefore, the cache-
	 * mode will not be right for page table mappings. To avoid
	 * polluting the pmap_kenter_pa() code with a special case for
	 * page tables, we simply fix up the cache-mode here if it's not
	 * correct.
	 */
	if (pte_l2_s_cache_mode != pte_l2_s_cache_mode_pt) {
		const struct l2_bucket * const l2b =
		    pmap_get_l2_bucket(pmap_kernel(), va);
		KASSERTMSG(l2b != NULL, "%#lx", va);
		pt_entry_t * const ptep = &l2b->l2b_kva[l2pte_index(va)];
		const pt_entry_t opte = *ptep;

		if ((opte & L2_S_CACHE_MASK) != pte_l2_s_cache_mode_pt) {
			/*
			 * Page tables must have the cache-mode set correctly.
			 */
			const pt_entry_t npte = (opte & ~L2_S_CACHE_MASK)
			    | pte_l2_s_cache_mode_pt;
			l2pte_set(ptep, npte, opte);
			PTE_SYNC(ptep);
			cpu_tlb_flushD_SE(va);
			cpu_cpwait();
		}
	}
#endif

	memset(v, 0, L2_TABLE_SIZE_REAL);
	PTE_SYNC_RANGE(v, L2_TABLE_SIZE_REAL / sizeof(pt_entry_t));
	return (0);
}

static int
pmap_l2dtable_ctor(void *arg, void *v, int flags)
{

	memset(v, 0, sizeof(struct l2_dtable));
	return (0);
}

static int
pmap_pmap_ctor(void *arg, void *v, int flags)
{

	memset(v, 0, sizeof(struct pmap));
	return (0);
}

static void
pmap_pinit(pmap_t pm)
{
#ifndef ARM_HAS_VBAR
	struct l2_bucket *l2b;

	if (vector_page < KERNEL_BASE) {
		/*
		 * Map the vector page.
		 */
		pmap_enter(pm, vector_page, systempage.pv_pa,
		    VM_PROT_READ | VM_PROT_EXECUTE,
		    VM_PROT_READ | VM_PROT_EXECUTE | PMAP_WIRED);
		pmap_update(pm);

		pm->pm_pl1vec = pmap_l1_kva(pm) + l1pte_index(vector_page);
		l2b = pmap_get_l2_bucket(pm, vector_page);
		KASSERTMSG(l2b != NULL, "%#lx", vector_page);
		pm->pm_l1vec = l2b->l2b_pa | L1_C_PROTO |
		    L1_C_DOM(pmap_domain(pm));
	} else
		pm->pm_pl1vec = NULL;
#endif
}

#ifdef PMAP_CACHE_VIVT
/*
 * Since we have a virtually indexed cache, we may need to inhibit caching if
 * there is more than one mapping and at least one of them is writable.
 * Since we purge the cache on every context switch, we only need to check for
 * other mappings within the same pmap, or kernel_pmap.
 * This function is also called when a page is unmapped, to possibly reenable
 * caching on any remaining mappings.
 *
 * The code implements the following logic, where:
 *
 * KW = # of kernel read/write pages
 * KR = # of kernel read only pages
 * UW = # of user read/write pages
 * UR = # of user read only pages
 *
 * KC = kernel mapping is cacheable
 * UC = user mapping is cacheable
 *
 *               KW=0,KR=0  KW=0,KR>0  KW=1,KR=0  KW>1,KR>=0
 *             +---------------------------------------------
 * UW=0,UR=0   | ---        KC=1       KC=1       KC=0
 * UW=0,UR>0   | UC=1       KC=1,UC=1  KC=0,UC=0  KC=0,UC=0
 * UW=1,UR=0   | UC=1       KC=0,UC=0  KC=0,UC=0  KC=0,UC=0
 * UW>1,UR>=0  | UC=0       KC=0,UC=0  KC=0,UC=0  KC=0,UC=0
 */

static const int pmap_vac_flags[4][4] = {
	{-1,		0,		0,		PVF_KNC},
	{0,		0,		PVF_NC,		PVF_NC},
	{0,		PVF_NC,		PVF_NC,		PVF_NC},
	{PVF_UNC,	PVF_NC,		PVF_NC,		PVF_NC}
};

static inline int
pmap_get_vac_flags(const struct vm_page_md *md)
{
	int kidx, uidx;

	kidx = 0;
	if (md->kro_mappings || md->krw_mappings > 1)
		kidx |= 1;
	if (md->krw_mappings)
		kidx |= 2;

	uidx = 0;
	if (md->uro_mappings || md->urw_mappings > 1)
		uidx |= 1;
	if (md->urw_mappings)
		uidx |= 2;

	return (pmap_vac_flags[uidx][kidx]);
}

static inline void
pmap_vac_me_harder(struct vm_page_md *md, paddr_t pa, pmap_t pm, vaddr_t va)
{
	int nattr;

	nattr = pmap_get_vac_flags(md);

	if (nattr < 0) {
		md->pvh_attrs &= ~PVF_NC;
		return;
	}

	if (nattr == 0 && (md->pvh_attrs & PVF_NC) == 0)
		return;

	if (pm == pmap_kernel())
		pmap_vac_me_kpmap(md, pa, pm, va);
	else
		pmap_vac_me_user(md, pa, pm, va);

	md->pvh_attrs = (md->pvh_attrs & ~PVF_NC) | nattr;
}

static void
pmap_vac_me_kpmap(struct vm_page_md *md, paddr_t pa, pmap_t pm, vaddr_t va)
{
	u_int u_cacheable, u_entries;
	struct pv_entry *pv;
	pmap_t last_pmap = pm;

	/*
	 * Pass one, see if there are both kernel and user pmaps for
	 * this page.  Calculate whether there are user-writable or
	 * kernel-writable pages.
	 */
	u_cacheable = 0;
	SLIST_FOREACH(pv, &md->pvh_list, pv_link) {
		if (pv->pv_pmap != pm && (pv->pv_flags & PVF_NC) == 0)
			u_cacheable++;
	}

	u_entries = md->urw_mappings + md->uro_mappings;

	/*
	 * We know we have just been updating a kernel entry, so if
	 * all user pages are already cacheable, then there is nothing
	 * further to do.
	 */
	if (md->k_mappings == 0 && u_cacheable == u_entries)
		return;

	if (u_entries) {
		/*
		 * Scan over the list again, for each entry, if it
		 * might not be set correctly, call pmap_vac_me_user
		 * to recalculate the settings.
		 */
		SLIST_FOREACH(pv, &md->pvh_list, pv_link) {
			/*
			 * We know kernel mappings will get set
			 * correctly in other calls.  We also know
			 * that if the pmap is the same as last_pmap
			 * then we've just handled this entry.
			 */
			if (pv->pv_pmap == pm || pv->pv_pmap == last_pmap)
				continue;

			/*
			 * If there are kernel entries and this page
			 * is writable but non-cacheable, then we can
			 * skip this entry also.
			 */
			if (md->k_mappings &&
			    (pv->pv_flags & (PVF_NC | PVF_WRITE)) ==
			    (PVF_NC | PVF_WRITE))
				continue;

			/*
			 * Similarly if there are no kernel-writable
			 * entries and the page is already
			 * read-only/cacheable.
			 */
			if (md->krw_mappings == 0 &&
			    (pv->pv_flags & (PVF_NC | PVF_WRITE)) == 0)
				continue;

			/*
			 * For some of the remaining cases, we know
			 * that we must recalculate, but for others we
			 * can't tell if they are correct or not, so
			 * we recalculate anyway.
			 */
			pmap_vac_me_user(md, pa, (last_pmap = pv->pv_pmap), 0);
		}

		if (md->k_mappings == 0)
			return;
	}

	pmap_vac_me_user(md, pa, pm, va);
}

static void
pmap_vac_me_user(struct vm_page_md *md, paddr_t pa, pmap_t pm, vaddr_t va)
{
	pmap_t kpmap = pmap_kernel();
	struct pv_entry *pv, *npv = NULL;
	u_int entries = 0;
	u_int writable = 0;
	u_int cacheable_entries = 0;
	u_int kern_cacheable = 0;
	u_int other_writable = 0;

	/*
	 * Count mappings and writable mappings in this pmap.
	 * Include kernel mappings as part of our own.
	 * Keep a pointer to the first one.
	 */
	npv = NULL;
	KASSERT(pmap_page_locked_p(md));
	SLIST_FOREACH(pv, &md->pvh_list, pv_link) {
		/* Count mappings in the same pmap */
		if (pm == pv->pv_pmap || kpmap == pv->pv_pmap) {
			if (entries++ == 0)
				npv = pv;

			/* Cacheable mappings */
			if ((pv->pv_flags & PVF_NC) == 0) {
				cacheable_entries++;
				if (kpmap == pv->pv_pmap)
					kern_cacheable++;
			}

			/* Writable mappings */
			if (pv->pv_flags & PVF_WRITE)
				++writable;
		} else if (pv->pv_flags & PVF_WRITE)
			other_writable = 1;
	}

	/*
	 * Enable or disable caching as necessary.
	 * Note: the first entry might be part of the kernel pmap,
	 * so we can't assume this is indicative of the state of the
	 * other (maybe non-kpmap) entries.
	 */
	if ((entries > 1 && writable) ||
	    (entries > 0 && pm == kpmap && other_writable)) {
		if (cacheable_entries == 0) {
			return;
		}

		for (pv = npv; pv; pv = SLIST_NEXT(pv, pv_link)) {
			if ((pm != pv->pv_pmap && kpmap != pv->pv_pmap) ||
			    (pv->pv_flags & PVF_NC))
				continue;

			pv->pv_flags |= PVF_NC;

			struct l2_bucket * const l2b
			    = pmap_get_l2_bucket(pv->pv_pmap, pv->pv_va);
			KASSERTMSG(l2b != NULL, "%#lx", va);
			pt_entry_t * const ptep
			    = &l2b->l2b_kva[l2pte_index(pv->pv_va)];
			const pt_entry_t opte = *ptep;
			pt_entry_t npte = opte & ~L2_S_CACHE_MASK;

			if ((va != pv->pv_va || pm != pv->pv_pmap)
			    && l2pte_valid_p(opte)) {
#ifdef PMAP_CACHE_VIVT
				pmap_cache_wbinv_page(pv->pv_pmap, pv->pv_va,
				    true, pv->pv_flags);
#endif
				pmap_tlb_invalidate_addr(pv->pv_pmap,
				    pv->pv_va);
			}

			l2pte_set(ptep, npte, opte);
			PTE_SYNC_CURRENT(pv->pv_pmap, ptep);
		}
		cpu_cpwait();
	} else if (entries > cacheable_entries) {
		/*
		 * Turn cacheing back on for some pages.  If it is a kernel
		 * page, only do so if there are no other writable pages.
		 */
		for (pv = npv; pv; pv = SLIST_NEXT(pv, pv_link)) {
			if (!(pv->pv_flags & PVF_NC) || (pm != pv->pv_pmap &&
			    (kpmap != pv->pv_pmap || other_writable)))
				continue;

			pv->pv_flags &= ~PVF_NC;

			struct l2_bucket * const l2b
			    = pmap_get_l2_bucket(pv->pv_pmap, pv->pv_va);
			KASSERTMSG(l2b != NULL, "%#lx", va);
			pt_entry_t * const ptep
			    = &l2b->l2b_kva[l2pte_index(pv->pv_va)];
			const pt_entry_t opte = *ptep;
			pt_entry_t npte = (opte & ~L2_S_CACHE_MASK)
			    | pte_l2_s_cache_mode;

			if (l2pte_valid_p(opte)) {
				pmap_tlb_invalidate_addr(pv->pv_pmap,
				    pv->pv_va);
			}

			l2pte_set(ptep, npte, opte);
			PTE_SYNC_CURRENT(pv->pv_pmap, ptep);
		}
	}
}
#endif

#ifdef PMAP_CACHE_VIPT
static void
pmap_vac_me_harder(struct vm_page_md *md, paddr_t pa, pmap_t pm, vaddr_t va)
{
}
#endif	/* PMAP_CACHE_VIPT */


/*
 * Modify pte bits for all ptes corresponding to the given physical address.
 * We use `maskbits' rather than `clearbits' because we're always passing
 * constants and the latter would require an extra inversion at run-time.
 */
static void
pmap_clearbit(struct vm_page_md *md, paddr_t pa, u_int maskbits)
{
	struct pv_entry *pv;
#ifdef PMAP_CACHE_VIPT
	const bool want_syncicache = PV_IS_EXEC_P(md->pvh_attrs);
	const u_int execbits = (maskbits & PVF_EXEC) ? L2_XS_XN : 0;
#else
	const u_int execbits = 0;
#endif

	NPDEBUG(PDB_BITS,
	    printf("pmap_clearbit: md %p mask 0x%x\n",
	    md, maskbits));

#ifdef PMAP_CACHE_VIPT
	/*
	 * If we might want to sync the I-cache and we've modified it,
	 * then we know we definitely need to sync or discard it.
	 */
	if (want_syncicache) {
		need_syncicache = md->pvh_attrs & PVF_MOD;
	}
#endif
	KASSERT(pmap_page_locked_p(md));

	/*
	 * Clear saved attributes (modify, reference)
	 */
	md->pvh_attrs &= ~(maskbits & (PVF_MOD | PVF_REF));

	if (SLIST_EMPTY(&md->pvh_list)) {
#if defined(PMAP_CACHE_VIPT) && !defined(ARM_MMU_EXTENDED)
		if (need_syncicache) {
			/*
			 * No one has it mapped, so just discard it.  The next
			 * exec remapping will cause it to be synced.
			 */
			md->pvh_attrs &= ~PVF_EXEC;
			PMAPCOUNT(exec_discarded_clearbit);
		}
#endif
		return;
	}

	/*
	 * Loop over all current mappings setting/clearing as appropos
	 */
	SLIST_FOREACH(pv, &md->pvh_list, pv_link) {
		pmap_t pm = pv->pv_pmap;
		const vaddr_t va = pv->pv_va;
		const u_int oflags = pv->pv_flags;
		pv->pv_flags &= ~maskbits;

		pmap_release_page_lock(md);
		pmap_acquire_pmap_lock(pm);

		struct l2_bucket * const l2b = pmap_get_l2_bucket(pm, va);
		if (l2b == NULL) {
			pmap_release_pmap_lock(pm);
			pmap_acquire_page_lock(md);
			continue;
		}
		KASSERTMSG(l2b != NULL, "%#lx", va);

		pt_entry_t * const ptep = &l2b->l2b_kva[l2pte_index(va)];
		const pt_entry_t opte = *ptep;
		pt_entry_t npte = opte | execbits;

		KASSERT((opte & L2_XS_nG) == (pm == pmap_kernel() ? 0 : L2_XS_nG));

		NPDEBUG(PDB_BITS,
		    printf( "%s: pv %p, pm %p, va 0x%08lx, flag 0x%x\n",
			__func__, pv, pm, va, oflags));

		if (maskbits & (PVF_WRITE|PVF_MOD)) {
#ifdef PMAP_CACHE_VIVT
			if ((oflags & PVF_NC)) {
				/*
				 * Entry is not cacheable:
				 *
				 * Don't turn caching on again if this is a
				 * modified emulation. This would be
				 * inconsitent with the settings created by
				 * pmap_vac_me_harder(). Otherwise, it's safe
				 * to re-enable cacheing.
				 *
				 * There's no need to call pmap_vac_me_harder()
				 * here: all pages are losing their write
				 * permission.
				 */
				if (maskbits & PVF_WRITE) {
					npte |= pte_l2_s_cache_mode;
					pv->pv_flags &= ~PVF_NC;
				}
			} else if (l2pte_writable_p(opte)) {
				/*
				 * Entry is writable/cacheable: check if pmap
				 * is current if it is flush it, otherwise it
				 * won't be in the cache
				 */
				pmap_cache_wbinv_page(pm, va,
				    (maskbits & PVF_REF) != 0,
				    oflags|PVF_WRITE);
			}
#endif

			/* make the pte read only */
			npte = l2pte_set_readonly(npte);

			pmap_acquire_page_lock(md);
#ifdef MULTIPROCESSOR
			pv = pmap_find_pv(md, pm, va);
#endif
			if (pv != NULL && (maskbits & oflags & PVF_WRITE)) {
				/*
				 * Keep alias accounting up to date
				 */
				if (pm == pmap_kernel()) {
					md->krw_mappings--;
					md->kro_mappings++;
				} else {
					md->urw_mappings--;
					md->uro_mappings++;
				}
#ifdef PMAP_CACHE_VIPT
				if (arm_cache_prefer_mask != 0) {
					if (md->urw_mappings + md->krw_mappings == 0) {
						md->pvh_attrs &= ~PVF_WRITE;
					} else {
						PMAP_VALIDATE_MD_PAGE(md);
					}
				}
				if (want_syncicache)
					need_syncicache = true;
#endif /* PMAP_CACHE_VIPT */
			}
			pmap_release_page_lock(md);
		}

		if (maskbits & PVF_REF) {
			if (true
			    && (maskbits & (PVF_WRITE|PVF_MOD)) == 0
			    && l2pte_valid_p(npte)) {
#ifdef PMAP_CACHE_VIVT
				/*
				 * Check npte here; we may have already
				 * done the wbinv above, and the validity
				 * of the PTE is the same for opte and
				 * npte.
				 */
				pmap_cache_wbinv_page(pm, va, true, oflags);
#endif
			}

			/*
			 * Make the PTE invalid so that we will take a
			 * page fault the next time the mapping is
			 * referenced.
			 */
			npte &= ~L2_TYPE_MASK;
			npte |= L2_TYPE_INV;
		}

		if (npte != opte) {
			l2pte_reset(ptep);
			PTE_SYNC(ptep);

			/* Flush the TLB entry if a current pmap. */
			pmap_tlb_invalidate_addr(pm, va);

			l2pte_set(ptep, npte, 0);
			PTE_SYNC(ptep);
		}

		pmap_release_pmap_lock(pm);
		pmap_acquire_page_lock(md);

		NPDEBUG(PDB_BITS,
		    printf("pmap_clearbit: pm %p va 0x%lx opte 0x%08x npte 0x%08x\n",
		    pm, va, opte, npte));
	}

#if defined(PMAP_CACHE_VIPT) && !defined(ARM_MMU_EXTENDED)
	/*
	 * If we need to sync the I-cache and we haven't done it yet, do it.
	 */
	if (need_syncicache) {
		pmap_release_page_lock(md);
		pmap_syncicache_page(md, pa);
		pmap_acquire_page_lock(md);
		PMAPCOUNT(exec_synced_clearbit);
	}

	/*
	 * If we are changing this to read-only, we need to call vac_me_harder
	 * so we can change all the read-only pages to cacheable.  We pretend
	 * this as a page deletion.
	 */
	if (need_vac_me_harder) {
		if (md->pvh_attrs & PVF_NC)
			pmap_vac_me_harder(md, pa, NULL, 0);
	}
#endif /* PMAP_CACHE_VIPT && !ARM_MMU_EXTENDED */
}

/*
 * pmap_clean_page()
 *
 * This is a local function used to work out the best strategy to clean
 * a single page referenced by its entry in the PV table. It's used by
 * pmap_copy_page, pmap_zero_page and maybe some others later on.
 *
 * Its policy is effectively:
 *  o If there are no mappings, we don't bother doing anything with the cache.
 *  o If there is one mapping, we clean just that page.
 *  o If there are multiple mappings, we clean the entire cache.
 *
 * So that some functions can be further optimised, it returns 0 if it didn't
 * clean the entire cache, or 1 if it did.
 *
 * XXX One bug in this routine is that if the pv_entry has a single page
 * mapped at 0x00000000 a whole cache clean will be performed rather than
 * just the 1 page. Since this should not occur in everyday use and if it does
 * it will just result in not the most efficient clean for the page.
 */
#ifdef PMAP_CACHE_VIVT
static bool
pmap_clean_page(struct vm_page_md *md, bool is_src)
{
	struct pv_entry *pv;
	pmap_t pm_to_clean = NULL;
	bool cache_needs_cleaning = false;
	vaddr_t page_to_clean = 0;
	u_int flags = 0;

	/*
	 * Since we flush the cache each time we change to a different
	 * user vmspace, we only need to flush the page if it is in the
	 * current pmap.
	 */
	KASSERT(pmap_page_locked_p(md));
	SLIST_FOREACH(pv, &md->pvh_list, pv_link) {
		if (pmap_is_current(pv->pv_pmap)) {
			flags |= pv->pv_flags;
			/*
			 * The page is mapped non-cacheable in
			 * this map.  No need to flush the cache.
			 */
			if (pv->pv_flags & PVF_NC) {
#ifdef DIAGNOSTIC
				KASSERT(!cache_needs_cleaning);
#endif
				break;
			} else if (is_src && (pv->pv_flags & PVF_WRITE) == 0)
				continue;
			if (cache_needs_cleaning) {
				page_to_clean = 0;
				break;
			} else {
				page_to_clean = pv->pv_va;
				pm_to_clean = pv->pv_pmap;
			}
			cache_needs_cleaning = true;
		}
	}

	if (page_to_clean) {
		pmap_cache_wbinv_page(pm_to_clean, page_to_clean,
		    !is_src, flags | PVF_REF);
	} else if (cache_needs_cleaning) {
		pmap_t const pm = curproc->p_vmspace->vm_map.pmap;

		pmap_cache_wbinv_all(pm, flags);
		return true;
	}
	return false;
}
#endif

#ifdef PMAP_CACHE_VIPT
/*
 * Sync a page with the I-cache.  Since this is a VIPT, we must pick the
 * right cache alias to make sure we flush the right stuff.
 */
void
pmap_md_page_syncicache(struct vm_page *pg, const kcpuset_t *onproc)
{
	struct vm_page_md *md = VM_PAGE_TO_MD(pg);
	pmap_t kpm = pmap_kernel();
	const size_t way_size = arm_pcache.icache_type == CACHE_TYPE_PIPT
	    ? PAGE_SIZE
	    : arm_pcache.icache_way_size;

	NPDEBUG(PDB_EXEC, printf("pmap_syncicache_page: md=%p (attrs=%#x)\n",
	    md, md->pvh_attrs));
	/*
	 * No need to clean the page if it's non-cached.
	 */

	pt_entry_t * const ptep = cpu_cdst_pte(0);
	const vaddr_t dstp = cpu_cdstp(0);
#ifdef __HAVE_MM_MD_DIRECT_MAPPED_PHYS
	if (way_size <= PAGE_SIZE) {
		bool ok = false;
		vaddr_t vdstp = pmap_direct_mapped_phys(pa, &ok, dstp);
		if (ok) {
			cpu_icache_sync_range(vdstp, way_size);
			return;
		}
	}
#endif

	/*
	 * We don't worry about the color of the exec page, we map the
	 * same page to pages in the way and then do the icache_sync on
	 * the entire way making sure we are cleaned.
	 */
	const pt_entry_t npte = L2_S_PROTO | pa | pte_l2_s_cache_mode
	    | L2_S_PROT(PTE_KERNEL, VM_PROT_READ|VM_PROT_WRITE);

	for (size_t i = 0, j = 0; i < way_size;
	     i += PAGE_SIZE, j += PAGE_SIZE / L2_S_SIZE) {
		l2pte_reset(ptep + j);
		PTE_SYNC(ptep + j);

		pmap_tlb_invalidate_addr(kpm, dstp + i);
		/*
		 * Set up a PTE with to flush these cache lines.
		 */
		l2pte_set(ptep + j, npte, 0);
	}
	PTE_SYNC_RANGE(ptep, way_size / L2_S_SIZE);

	/*
	 * Flush it.
	 */
	cpu_icache_sync_range(dstp, way_size);

	for (size_t i = 0, j = 0; i < way_size;
	     i += PAGE_SIZE, j += PAGE_SIZE / L2_S_SIZE) {
		/*
		 * Unmap the page(s).
		 */
		l2pte_reset(ptep + j);
		pmap_tlb_invalidate_addr(kpm, dstp + i);
	}
	PTE_SYNC_RANGE(ptep, way_size / L2_S_SIZE);

	md->pvh_attrs |= PVF_EXEC;
	PMAPCOUNT(exec_synced);
}

#endif /* PMAP_CACHE_VIPT */

/*
 * Routine:	pmap_page_remove
 * Function:
 *		Removes this physical page from
 *		all physical maps in which it resides.
 *		Reflects back modify bits to the pager.
 */
static void
pmap_page_remove(struct vm_page_md *md, paddr_t pa)
{
	struct l2_bucket *l2b;
	struct pv_entry *pv;
	pt_entry_t *ptep;
	u_int flags = 0;

	NPDEBUG(PDB_FOLLOW,
	    printf("pmap_page_remove: md %p (0x%08lx)\n", md,
	    pa));

	struct pv_entry **pvp = &SLIST_FIRST(&md->pvh_list);
	pmap_acquire_page_lock(md);
	if (*pvp == NULL) {
#ifdef PMAP_CACHE_VIPT
		/*
		 * We *know* the page contents are about to be replaced.
		 * Discard the exec contents
		 */
		if (PV_IS_EXEC_P(md->pvh_attrs))
			PMAPCOUNT(exec_discarded_page_protect);
		md->pvh_attrs &= ~PVF_EXEC;
		PMAP_VALIDATE_MD_PAGE(md);
#endif
		pmap_release_page_lock(md);
		return;
	}
#if defined(PMAP_CACHE_VIPT) && !defined(ARM_MMU_EXTENDED)
	KASSERT(arm_cache_prefer_mask == 0 || pmap_is_page_colored_p(md));
#endif

	/*
	 * Clear alias counts
	 */
#ifdef PMAP_CACHE_VIVT
	md->k_mappings = 0;
#endif
	md->urw_mappings = md->uro_mappings = 0;

#ifdef PMAP_CACHE_VIVT
	pmap_clean_page(md, false);
#endif

	while ((pv = *pvp) != NULL) {
		pmap_t pm = pv->pv_pmap;

		if (pm == pmap_kernel()) {
#ifdef PMAP_CACHE_VIPT
			/*
			 * If this was unmanaged mapping, it must be preserved.
			 * Move it back on the list and advance the end-of-list
			 * pointer.
			 */
			if (PV_IS_KENTRY_P(pv->pv_flags)) {
				*pvp = pv;
				pvp = &SLIST_NEXT(pv, pv_link);
				continue;
			}
			if (pv->pv_flags & PVF_WRITE)
				md->krw_mappings--;
			else
				md->kro_mappings--;
#endif
			PMAPCOUNT(kernel_unmappings);
		}
		*pvp = SLIST_NEXT(pv, pv_link); /* remove from list */
		PMAPCOUNT(unmappings);

		pmap_release_page_lock(md);
		pmap_acquire_pmap_lock(pm);

		l2b = pmap_get_l2_bucket(pm, pv->pv_va);
		KASSERTMSG(l2b != NULL, "%#lx", pv->pv_va);

		ptep = &l2b->l2b_kva[l2pte_index(pv->pv_va)];

		/*
		 * Update statistics
		 */
		--pm->pm_stats.resident_count;

		/* Wired bit */
		if (pv->pv_flags & PVF_WIRED)
			--pm->pm_stats.wired_count;

		flags |= pv->pv_flags;

		/*
		 * Invalidate the PTEs.
		 */
		l2pte_reset(ptep);
		PTE_SYNC_CURRENT(pm, ptep);

		pmap_tlb_invalidate_addr(pm, pv->pv_va);

		pmap_free_l2_bucket(pm, l2b, PAGE_SIZE / L2_S_SIZE);

		pmap_release_pmap_lock(pm);

		pool_put(&pmap_pv_pool, pv);
		pmap_acquire_page_lock(md);
#ifdef MULTIPROCESSOR
		/*
		 * Restart of the beginning of the list.
		 */
		pvp = &SLIST_FIRST(&md->pvh_list);
#endif
	}
	/*
	 * if we reach the end of the list and there are still mappings, they
	 * might be able to be cached now.  And they must be kernel mappings.
	 */
	if (!SLIST_EMPTY(&md->pvh_list)) {
		pmap_vac_me_harder(md, pa, pmap_kernel(), 0);
	}

#ifdef PMAP_CACHE_VIPT
	/*
	 * Its EXEC cache is now gone.
	 */
	if (PV_IS_EXEC_P(md->pvh_attrs))
		PMAPCOUNT(exec_discarded_page_protect);
	md->pvh_attrs &= ~PVF_EXEC;
	KASSERT(md->urw_mappings == 0);
	KASSERT(md->uro_mappings == 0);
#endif /* PMAP_CACHE_VIPT */
	pmap_release_page_lock(md);

}

/*
 * pmap_t pmap_create(void)
 *
 *      Create a new pmap structure from scratch.
 */
pmap_t
pmap_create(void)
{
	pmap_t pm;

	pm = pool_cache_get(&pmap_cache, PR_WAITOK);

	mutex_init(&pm->pm_obj_lock, MUTEX_DEFAULT, IPL_NONE);
	uvm_obj_init(&pm->pm_obj, NULL, false, 1);
	uvm_obj_setlock(&pm->pm_obj, &pm->pm_obj_lock);

	pm->pm_stats.wired_count = 0;
	pm->pm_stats.resident_count = 1;
#ifdef MULTIPROCESSOR
	kcpuset_create(&pm->pm_active, true);
	kcpuset_create(&pm->pm_onproc, true);
#endif
	pmap_alloc_l1(pm);

	/*
	 * Note: The pool cache ensures that the pm_l2[] array is already
	 * initialised to zero.
	 */

	pmap_pinit(pm);

	return (pm);
}

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

/*
 * int pmap_enter(pmap_t pm, vaddr_t va, paddr_t pa, vm_prot_t prot,
 *      u_int flags)
 *
 *      Insert the given physical page (p) at
 *      the specified virtual address (v) in the
 *      target physical map with the protection requested.
 *
 *      NB:  This is the only routine which MAY NOT lazy-evaluate
 *      or lose information.  That is, this routine must actually
 *      insert this page into the given map NOW.
 */
int
pmap_enter(pmap_t pm, vaddr_t va, paddr_t pa, vm_prot_t prot, u_int flags)
{
	struct l2_bucket *l2b;
	struct vm_page *pg, *opg;
	u_int nflags;
	u_int oflags;
	const bool kpm_p = (pm == pmap_kernel());
#ifdef ARM_HAS_VBAR
	const bool vector_page_p = false;
#else
	const bool vector_page_p = (va == vector_page);
#endif
	struct pmap_page *pp = pmap_pv_tracked(pa);
	struct pv_entry *new_pv = NULL;
	struct pv_entry *old_pv = NULL;
	int error = 0;

	UVMHIST_FUNC(__func__); UVMHIST_CALLED(maphist);

	UVMHIST_LOG(maphist, " (pm %#jx va %#jx pa %#jx prot %#jx",
	    (uintptr_t)pm, va, pa, prot);
	UVMHIST_LOG(maphist, "  flag %#jx", flags, 0, 0, 0);

	KDASSERT((flags & PMAP_WIRED) == 0 || (flags & VM_PROT_ALL) != 0);
	KDASSERT(((va | pa) & PGOFSET) == 0);

	/*
	 * Get a pointer to the page.  Later on in this function, we
	 * test for a managed page by checking pg != NULL.
	 */
	pg = pmap_initialized ? PHYS_TO_VM_PAGE(pa) : NULL;

	/*
	 * if we may need a new pv entry allocate if now, as we can't do it
	 * with the kernel_pmap locked
	 */
	if (pg || pp)
		new_pv = pool_get(&pmap_pv_pool, PR_NOWAIT);

	nflags = 0;
	if (prot & VM_PROT_WRITE)
		nflags |= PVF_WRITE;
	if (prot & VM_PROT_EXECUTE)
		nflags |= PVF_EXEC;
	if (flags & PMAP_WIRED)
		nflags |= PVF_WIRED;

	pmap_acquire_pmap_lock(pm);

	/*
	 * Fetch the L2 bucket which maps this page, allocating one if
	 * necessary for user pmaps.
	 */
	if (kpm_p) {
		l2b = pmap_get_l2_bucket(pm, va);
	} else {
		l2b = pmap_alloc_l2_bucket(pm, va);
	}
	if (l2b == NULL) {
		if (flags & PMAP_CANFAIL) {
			pmap_release_pmap_lock(pm);
			error = ENOMEM;
			goto free_pv;
		}
		panic("pmap_enter: failed to allocate L2 bucket");
	}
	pt_entry_t *ptep = &l2b->l2b_kva[l2pte_index(va)];
	const pt_entry_t opte = *ptep;
	pt_entry_t npte = pa;
	oflags = 0;

	if (opte) {
		/*
		 * There is already a mapping at this address.
		 * If the physical address is different, lookup the
		 * vm_page.
		 */
		if (l2pte_pa(opte) != pa) {
			KASSERT(!pmap_pv_tracked(pa));
			opg = PHYS_TO_VM_PAGE(l2pte_pa(opte));
		} else
			opg = pg;
	} else
		opg = NULL;

	if (pg || pp) {
		KASSERT((pg != NULL) != (pp != NULL));
		struct vm_page_md *md = (pg != NULL) ? VM_PAGE_TO_MD(pg) :
		    PMAP_PAGE_TO_MD(pp);

		/*
		 * This is to be a managed mapping.
		 */
		pmap_acquire_page_lock(md);
		if ((flags & VM_PROT_ALL) || (md->pvh_attrs & PVF_REF)) {
			/*
			 * - The access type indicates that we don't need
			 *   to do referenced emulation.
			 * OR
			 * - The physical page has already been referenced
			 *   so no need to re-do referenced emulation here.
			 */
			npte |= l2pte_set_readonly(L2_S_PROTO);

			nflags |= PVF_REF;

			if ((prot & VM_PROT_WRITE) != 0 &&
			    ((flags & VM_PROT_WRITE) != 0 ||
			     (md->pvh_attrs & PVF_MOD) != 0)) {
				/*
				 * This is a writable mapping, and the
				 * page's mod state indicates it has
				 * already been modified. Make it
				 * writable from the outset.
				 */
				npte = l2pte_set_writable(npte);
				nflags |= PVF_MOD;
			}

			/*
			 * If the page has been cleaned, then the pvh_attrs
			 * will have PVF_EXEC set, so mark it execute so we
			 * don't get an access fault when trying to execute
			 * from it.
			 */
			if (md->pvh_attrs & nflags & PVF_EXEC) {
				npte &= ~L2_XS_XN;
			}
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

		if (pg != NULL && pg == opg) {
			/*
			 * We're changing the attrs of an existing mapping.
			 */
			oflags = pmap_modify_pv(md, pa, pm, va,
			    PVF_WRITE | PVF_EXEC | PVF_WIRED |
			    PVF_MOD | PVF_REF, nflags);

#ifdef PMAP_CACHE_VIVT
			/*
			 * We may need to flush the cache if we're
			 * doing rw-ro...
			 */
			if (pm->pm_cstate.cs_cache_d &&
			    (oflags & PVF_NC) == 0 &&
			    l2pte_writable_p(opte) &&
			    (prot & VM_PROT_WRITE) == 0)
				cpu_dcache_wb_range(va, PAGE_SIZE);
#endif
		} else {
			struct pv_entry *pv;
			/*
			 * New mapping, or changing the backing page
			 * of an existing mapping.
			 */
			if (opg) {
				struct vm_page_md *omd = VM_PAGE_TO_MD(opg);
				paddr_t opa = VM_PAGE_TO_PHYS(opg);

				/*
				 * Replacing an existing mapping with a new one.
				 * It is part of our managed memory so we
				 * must remove it from the PV list
				 */
				pv = pmap_remove_pv(omd, opa, pm, va);
				pmap_vac_me_harder(omd, opa, pm, 0);
				oflags = pv->pv_flags;

#ifdef PMAP_CACHE_VIVT
				/*
				 * If the old mapping was valid (ref/mod
				 * emulation creates 'invalid' mappings
				 * initially) then make sure to frob
				 * the cache.
				 */
				if (!(oflags & PVF_NC) && l2pte_valid_p(opte)) {
					pmap_cache_wbinv_page(pm, va, true,
					    oflags);
				}
#endif
			} else {
				pv = new_pv;
				new_pv = NULL;
				if (pv == NULL) {
					pmap_release_page_lock(md);
					pmap_release_pmap_lock(pm);
					if ((flags & PMAP_CANFAIL) == 0)
						panic("pmap_enter: "
						    "no pv entries");

					pmap_free_l2_bucket(pm, l2b, 0);
					UVMHIST_LOG(maphist, "  <-- done (ENOMEM)",
					    0, 0, 0, 0);
					return (ENOMEM);
				}
			}

			pmap_enter_pv(md, pa, pv, pm, va, nflags);
		}
		pmap_release_page_lock(md);
	} else {
		/*
		 * We're mapping an unmanaged page.
		 * These are always readable, and possibly writable, from
		 * the get go as we don't need to track ref/mod status.
		 */
		npte |= l2pte_set_readonly(L2_S_PROTO);
		if (prot & VM_PROT_WRITE)
			npte = l2pte_set_writable(npte);

		/*
		 * Make sure the vector table is mapped cacheable
		 */
		if ((vector_page_p && !kpm_p)
		    || (flags & ARM32_MMAP_CACHEABLE)) {
			npte |= pte_l2_s_cache_mode;
			npte &= ~L2_XS_XN;	/* and executable */
		} else if (flags & ARM32_MMAP_WRITECOMBINE) {
			npte |= pte_l2_s_wc_mode;
		}
		if (opg) {
			/*
			 * Looks like there's an existing 'managed' mapping
			 * at this address.
			 */
			struct vm_page_md *omd = VM_PAGE_TO_MD(opg);
			paddr_t opa = VM_PAGE_TO_PHYS(opg);

			pmap_acquire_page_lock(omd);
			old_pv = pmap_remove_pv(omd, opa, pm, va);
			pmap_vac_me_harder(omd, opa, pm, 0);
			oflags = old_pv->pv_flags;
			pmap_release_page_lock(omd);

#ifdef PMAP_CACHE_VIVT
			if (!(oflags & PVF_NC) && l2pte_valid_p(opte)) {
				pmap_cache_wbinv_page(pm, va, true, oflags);
			}
#endif
		}
	}

	/*
	 * Make sure userland mappings get the right permissions
	 */
	if (!vector_page_p && !kpm_p) {
		npte |= L2_S_PROT_U;
		npte |= L2_XS_nG;	/* user pages are not global */
	}

	/*
	 * Keep the stats up to date
	 */
	if (opte == 0) {
		l2b->l2b_occupancy += PAGE_SIZE / L2_S_SIZE;
		pm->pm_stats.resident_count++;
	}

	UVMHIST_LOG(maphist, " opte %#x npte %#x", opte, npte, 0, 0);

	/*
	 * If exec protection was requested but the page hasn't been synced,
	 * sync it now and allow execution from it.
	 */
	if ((nflags & PVF_EXEC) && (npte & L2_XS_XN)) {
		struct vm_page_md *md = VM_PAGE_TO_MD(pg);
		npte &= ~L2_XS_XN;
		pmap_syncicache_page(md, pa);
		PMAPCOUNT(exec_synced_map);
	}
	/*
	 * If this is just a wiring change, the two PTEs will be
	 * identical, so there's no need to update the page table.
	 */
	if (npte != opte) {
		l2pte_reset(ptep);
		PTE_SYNC(ptep);
		if (l2pte_valid_p(opte)) {
			pmap_tlb_invalidate_addr(pm, va, oflags);
		}
		l2pte_set(ptep, npte, 0);
		PTE_SYNC(ptep);

	}
#if defined(PMAP_CACHE_VIPT) && defined(DIAGNOSTIC)
	if (pg) {
		struct vm_page_md *md = VM_PAGE_TO_MD(pg);

		pmap_acquire_page_lock(md);
		PMAP_VALIDATE_MD_PAGE(md);
		pmap_release_page_lock(md);
	}
#endif

	pmap_release_pmap_lock(pm);

	return (0);
}

/*
 * pmap_remove()
 *
 * pmap_remove is responsible for nuking a number of mappings for a range
 * of virtual address space in the current pmap. To do this efficiently
 * is interesting, because in a number of cases a wide virtual address
 * range may be supplied that contains few actual mappings. So, the
 * optimisations are:
 *  1. Skip over hunks of address space for which no L1 or L2 entry exists.
 *  2. Build up a list of pages we've hit, up to a maximum, so we can
 *     maybe do just a partial cache clean. This path of execution is
 *     complicated by the fact that the cache must be flushed _before_
 *     the PTE is nuked, being a VAC :-)
 *  3. If we're called after UVM calls pmap_remove_all(), we can defer
 *     all invalidations until pmap_update(), since pmap_remove_all() has
 *     already flushed the cache.
 *  4. Maybe later fast-case a single page, but I don't think this is
 *     going to make _that_ much difference overall.
 */

#define	PMAP_REMOVE_CLEAN_LIST_SIZE	3

void
pmap_remove(pmap_t pm, vaddr_t sva, vaddr_t eva)
{
	SLIST_HEAD(,pv_entry) opv_list;
	struct pv_entry *pv, *npv;

	UVMHIST_FUNC(__func__); UVMHIST_CALLED(maphist);
	UVMHIST_LOG(maphist, " (pm=%#jx, sva=%#jx, eva=%#jx)",
	    (uintptr_t)pm, sva, eva, 0);

	SLIST_INIT(&opv_list);
	/*
	 * we lock in the pmap => pv_head direction
	 */
	pmap_acquire_pmap_lock(pm);

	u_int cleanlist_idx, total, cnt;
	struct {
		vaddr_t va;
		pt_entry_t *ptep;
	} cleanlist[PMAP_REMOVE_CLEAN_LIST_SIZE];
	u_int mappings;

	if (pm->pm_remove_all || !pmap_is_cached(pm)) {
		cleanlist_idx = PMAP_REMOVE_CLEAN_LIST_SIZE + 1;
	} else
		cleanlist_idx = 0;

	total = 0;

	while (sva < eva) {
		/*
		 * Do one L2 bucket's worth at a time.
		 */
		vaddr_t next_bucket = L2_NEXT_BUCKET_VA(sva);
		if (next_bucket > eva)
			next_bucket = eva;

		struct l2_bucket * const l2b = pmap_get_l2_bucket(pm, sva);
		if (l2b == NULL) {
			sva = next_bucket;
			continue;
		}

		pt_entry_t *ptep = &l2b->l2b_kva[l2pte_index(sva)];

		for (mappings = 0;
		     sva < next_bucket;
		     sva += PAGE_SIZE, ptep += PAGE_SIZE / L2_S_SIZE) {
			pt_entry_t opte = *ptep;

			if (opte == 0) {
				/* Nothing here, move along */
				continue;
			}

			u_int flags = PVF_REF;
			paddr_t pa = l2pte_pa(opte);
			struct vm_page * const pg = PHYS_TO_VM_PAGE(pa);

			/*
			 * Update flags. In a number of circumstances,
			 * we could cluster a lot of these and do a
			 * number of sequential pages in one go.
			 */
			if (pg != NULL) {
				struct vm_page_md *md = VM_PAGE_TO_MD(pg);
				struct pv_entry *pv;

				pmap_acquire_page_lock(md);
				pv = pmap_remove_pv(md, pa, pm, sva);
				pmap_vac_me_harder(md, pa, pm, 0);
				pmap_release_page_lock(md);
				if (pv != NULL) {
					if (pm->pm_remove_all == false) {
						flags = pv->pv_flags;
					}
					pool_put(&pmap_pv_pool, pv);
				}
			}
			mappings += PAGE_SIZE / L2_S_SIZE;

			if (!l2pte_valid_p(opte)) {
				/*
				 * Ref/Mod emulation is still active for this
				 * mapping, therefore it is has not yet been
				 * accessed. No need to frob the cache/tlb.
				 */
				l2pte_reset(ptep);
				PTE_SYNC_CURRENT(pm, ptep);
				continue;
			}

			if (pm == pmap_kernel()) {
				l2pte_reset(ptep);
				PTE_SYNC(ptep);
 				pmap_tlb_invalidate_addr(pm, sva, flags);
				continue;
			}
			if (cleanlist_idx < PMAP_REMOVE_CLEAN_LIST_SIZE) {
				/* Add to the clean list. */
				cleanlist[cleanlist_idx].ptep = ptep;
				cleanlist[cleanlist_idx].va =
				    sva | (flags & PVF_EXEC);
				cleanlist_idx++;
			} else if (cleanlist_idx == PMAP_REMOVE_CLEAN_LIST_SIZE) {
				/* Nuke everything if needed. */
#ifdef PMAP_CACHE_VIVT
				pmap_cache_wbinv_all(pm, PVF_EXEC);
#endif
				/*
				 * Roll back the previous PTE list,
				 * and zero out the current PTE.
				 */
				for (cnt = 0;
				     cnt < PMAP_REMOVE_CLEAN_LIST_SIZE; cnt++) {
					l2pte_reset(cleanlist[cnt].ptep);
					PTE_SYNC(cleanlist[cnt].ptep);
				}
				l2pte_reset(ptep);
				PTE_SYNC(ptep);
				cleanlist_idx++;
				pm->pm_remove_all = true;
			} else {
				l2pte_reset(ptep);
				PTE_SYNC(ptep);
				if (pm->pm_remove_all == false) {
					pmap_tlb_invalidate_addr(pm, sva, flags);
				}
			}
		}

		/*
		 * Deal with any left overs
		 */
		if (cleanlist_idx <= PMAP_REMOVE_CLEAN_LIST_SIZE) {
			total += cleanlist_idx;
			for (cnt = 0; cnt < cleanlist_idx; cnt++) {
				l2pte_reset(cleanlist[cnt].ptep);
				PTE_SYNC_CURRENT(pm, cleanlist[cnt].ptep);
				vaddr_t clva = cleanlist[cnt].va;
				pmap_tlb_invalidate_addr(pm, clva, PVF_REF);
			}

			/*
			 * If it looks like we're removing a whole bunch
			 * of mappings, it's faster to just write-back
			 * the whole cache now and defer TLB flushes until
			 * pmap_update() is called.
			 */
			if (total <= PMAP_REMOVE_CLEAN_LIST_SIZE)
				cleanlist_idx = 0;
			else {
				cleanlist_idx = PMAP_REMOVE_CLEAN_LIST_SIZE + 1;
#ifdef PMAP_CACHE_VIVT
				pmap_cache_wbinv_all(pm, PVF_EXEC);
#endif
				pm->pm_remove_all = true;
			}
		}


		pmap_free_l2_bucket(pm, l2b, mappings);
		pm->pm_stats.resident_count -= mappings / (PAGE_SIZE/L2_S_SIZE);
	}

	pmap_release_pmap_lock(pm);
}

#if defined(PMAP_CACHE_VIPT) && !defined(ARM_MMU_EXTENDED)
static struct pv_entry *
pmap_kremove_pg(struct vm_page *pg, vaddr_t va)
{
	struct vm_page_md *md = VM_PAGE_TO_MD(pg);
	paddr_t pa = VM_PAGE_TO_PHYS(pg);
	struct pv_entry *pv;

	KASSERT(arm_cache_prefer_mask == 0 || md->pvh_attrs & (PVF_COLORED|PVF_NC));
	KASSERT((md->pvh_attrs & PVF_KMPAGE) == 0);
	KASSERT(pmap_page_locked_p(md));

	pv = pmap_remove_pv(md, pa, pmap_kernel(), va);
	KASSERTMSG(pv, "pg %p (pa #%lx) va %#lx", pg, pa, va);
	KASSERT(PV_IS_KENTRY_P(pv->pv_flags));

	/*
	 * If we are removing a writeable mapping to a cached exec page,
	 * if it's the last mapping then clear it execness other sync
	 * the page to the icache.
	 */
	if ((md->pvh_attrs & (PVF_NC|PVF_EXEC)) == PVF_EXEC
	    && (pv->pv_flags & PVF_WRITE) != 0) {
		if (SLIST_EMPTY(&md->pvh_list)) {
			md->pvh_attrs &= ~PVF_EXEC;
			PMAPCOUNT(exec_discarded_kremove);
		} else {
			pmap_syncicache_page(md, pa);
			PMAPCOUNT(exec_synced_kremove);
		}
	}
	pmap_vac_me_harder(md, pa, pmap_kernel(), 0);

	return pv;
}
#endif /* PMAP_CACHE_VIPT && !ARM_MMU_EXTENDED */

/*
 * pmap_kenter_pa: enter an unmanaged, wired kernel mapping
 *
 * We assume there is already sufficient KVM space available
 * to do this, as we can't allocate L2 descriptor tables/metadata
 * from here.
 */
void
pmap_kenter_pa(vaddr_t va, paddr_t pa, vm_prot_t prot, u_int flags)
{
#ifdef PMAP_CACHE_VIVT
	struct vm_page *pg = (flags & PMAP_KMPAGE) ? PHYS_TO_VM_PAGE(pa) : NULL;
#endif
#ifdef PMAP_CACHE_VIPT
	struct vm_page *pg = PHYS_TO_VM_PAGE(pa);
	struct vm_page *opg;
#endif
	struct vm_page_md *md = pg != NULL ? VM_PAGE_TO_MD(pg) : NULL;

	UVMHIST_FUNC(__func__);

	if (pmap_initialized) {
		UVMHIST_CALLED(maphist);
		UVMHIST_LOG(maphist, " (va=%#x, pa=%#x, prot=%#x, flags=%#x",
		    va, pa, prot, flags);
	}

	pmap_t kpm = pmap_kernel();
	pmap_acquire_pmap_lock(kpm);
	struct l2_bucket * const l2b = pmap_get_l2_bucket(kpm, va);
	const size_t l1slot __diagused = l1pte_index(va);
	KASSERTMSG(l2b != NULL,
	    "va %#lx pa %#lx prot %d maxkvaddr %#lx: l2 %p l2b %p kva %p",
	    va, pa, prot, pmap_curmaxkvaddr, kpm->pm_l2[L2_IDX(l1slot)],
	    kpm->pm_l2[L2_IDX(l1slot)]
		? &kpm->pm_l2[L2_IDX(l1slot)]->l2_bucket[L2_BUCKET(l1slot)]
		: NULL,
	    kpm->pm_l2[L2_IDX(l1slot)]
		? kpm->pm_l2[L2_IDX(l1slot)]->l2_bucket[L2_BUCKET(l1slot)].l2b_kva
		: NULL);
	KASSERT(l2b->l2b_kva != NULL);

	pt_entry_t * const ptep = &l2b->l2b_kva[l2pte_index(va)];
	const pt_entry_t opte = *ptep;

	if (opte == 0) {
		PMAPCOUNT(kenter_mappings);
		l2b->l2b_occupancy += PAGE_SIZE / L2_S_SIZE;
	} else {
		PMAPCOUNT(kenter_remappings);
#ifdef PMAP_CACHE_VIPT
		opg = PHYS_TO_VM_PAGE(l2pte_pa(opte));
#if !defined(ARM_MMU_EXTENDED) || defined(DIAGNOSTIC)
		struct vm_page_md *omd __diagused = VM_PAGE_TO_MD(opg);
#endif
		if (opg && arm_cache_prefer_mask != 0) {
			KASSERT(opg != pg);
			KASSERT((omd->pvh_attrs & PVF_KMPAGE) == 0);
			KASSERT((flags & PMAP_KMPAGE) == 0);
		}
#endif
		if (l2pte_valid_p(opte)) {
			l2pte_reset(ptep);
			PTE_SYNC(ptep);
#ifdef PMAP_CACHE_VIVT
			cpu_dcache_wbinv_range(va, PAGE_SIZE);
#endif
			cpu_tlb_flushD_SE(va);
			cpu_cpwait();
		}
	}
	pmap_release_pmap_lock(kpm);

	pt_entry_t npte = L2_S_PROTO | pa | L2_S_PROT(PTE_KERNEL, prot)
	    | ((flags & PMAP_NOCACHE)
		? 0
		: ((flags & PMAP_PTE)
		    ? pte_l2_s_cache_mode_pt : pte_l2_s_cache_mode));
	if (prot & VM_PROT_EXECUTE)
		npte &= ~L2_XS_XN;
	l2pte_set(ptep, npte, 0);
	PTE_SYNC(ptep);

	if (pg) {
		if (flags & PMAP_KMPAGE) {
			KASSERT(md->urw_mappings == 0);
			KASSERT(md->uro_mappings == 0);
			KASSERT(md->krw_mappings == 0);
			KASSERT(md->kro_mappings == 0);
#if defined(PMAP_CACHE_VIPT) && !defined(ARM_MMU_EXTENDED)
			KASSERT(pv == NULL);
			KASSERT(arm_cache_prefer_mask == 0 || (va & PVF_COLORED) == 0);
			KASSERT((md->pvh_attrs & PVF_NC) == 0);
			/* if there is a color conflict, evict from cache. */
			if (pmap_is_page_colored_p(md)
			    && ((va ^ md->pvh_attrs) & arm_cache_prefer_mask)) {
				PMAPCOUNT(vac_color_change);
				pmap_flush_page(md, pa, PMAP_FLUSH_PRIMARY);
			} else if (md->pvh_attrs & PVF_MULTCLR) {
				/*
				 * If this page has multiple colors, expunge
				 * them.
				 */
				PMAPCOUNT(vac_flush_lots2);
				pmap_flush_page(md, pa, PMAP_FLUSH_SECONDARY);
			}
			/*
			 * Since this is a KMPAGE, there can be no contention
			 * for this page so don't lock it.
			 */
			md->pvh_attrs &= PAGE_SIZE - 1;
			md->pvh_attrs |= PVF_KMPAGE | PVF_COLORED | PVF_DIRTY
			    | (va & arm_cache_prefer_mask);
#else /* !PMAP_CACHE_VIPT || ARM_MMU_EXTENDED */
			md->pvh_attrs |= PVF_KMPAGE;
#endif
			atomic_inc_32(&pmap_kmpages);
#if defined(PMAP_CACHE_VIPT) && !defined(ARM_MMU_EXTENDED)
		} else if (arm_cache_prefer_mask != 0) {
			if (pv == NULL) {
				pv = pool_get(&pmap_pv_pool, PR_NOWAIT);
				KASSERT(pv != NULL);
			}
			pmap_acquire_page_lock(md);
			pmap_enter_pv(md, pa, pv, pmap_kernel(), va,
			    PVF_WIRED | PVF_KENTRY
			    | (prot & VM_PROT_WRITE ? PVF_WRITE : 0));
			if ((prot & VM_PROT_WRITE)
			    && !(md->pvh_attrs & PVF_NC))
				md->pvh_attrs |= PVF_DIRTY;
			KASSERT((prot & VM_PROT_WRITE) == 0 || (md->pvh_attrs & (PVF_DIRTY|PVF_NC)));
			pmap_vac_me_harder(md, pa, pmap_kernel(), va);
			pmap_release_page_lock(md);
#endif
		}
#if defined(PMAP_CACHE_VIPT) && !defined(ARM_MMU_EXTENDED)
	} else {
		if (pv != NULL)
			pool_put(&pmap_pv_pool, pv);
#endif
	}
#if defined(PMAP_CACHE_VIPT) && !defined(ARM_MMU_EXTENDED)
	KASSERT(md == NULL || !pmap_page_locked_p(md));
#endif
	if (pmap_initialized) {
		UVMHIST_LOG(maphist, "  <-- done (ptep %p: %#x -> %#x)",
		    ptep, opte, npte, 0);
	}

}

void
pmap_kremove(vaddr_t va, vsize_t len)
{
#ifdef UVMHIST
	u_int total_mappings = 0;
#endif

	PMAPCOUNT(kenter_unmappings);

	UVMHIST_FUNC(__func__); UVMHIST_CALLED(maphist);

	UVMHIST_LOG(maphist, " (va=%#x, len=%#x)", va, len, 0, 0);

	const vaddr_t eva = va + len;

	pmap_acquire_pmap_lock(pmap_kernel());

	while (va < eva) {
		vaddr_t next_bucket = L2_NEXT_BUCKET_VA(va);
		if (next_bucket > eva)
			next_bucket = eva;

		pmap_t kpm = pmap_kernel();
		struct l2_bucket * const l2b = pmap_get_l2_bucket(kpm, va);
		KDASSERT(l2b != NULL);

		pt_entry_t * const sptep = &l2b->l2b_kva[l2pte_index(va)];
		pt_entry_t *ptep = sptep;
		u_int mappings = 0;

		while (va < next_bucket) {
			const pt_entry_t opte = *ptep;
			struct vm_page *opg = PHYS_TO_VM_PAGE(l2pte_pa(opte));
			if (opg != NULL) {
				struct vm_page_md *omd = VM_PAGE_TO_MD(opg);

				if (omd->pvh_attrs & PVF_KMPAGE) {
					KASSERT(omd->urw_mappings == 0);
					KASSERT(omd->uro_mappings == 0);
					KASSERT(omd->krw_mappings == 0);
					KASSERT(omd->kro_mappings == 0);
					omd->pvh_attrs &= ~PVF_KMPAGE;
#if defined(PMAP_CACHE_VIPT) && !defined(ARM_MMU_EXTENDED)
					if (arm_cache_prefer_mask != 0) {
						omd->pvh_attrs &= ~PVF_WRITE;
					}
#endif
					atomic_dec_32(&pmap_kmpages);
#if defined(PMAP_CACHE_VIPT) && !defined(ARM_MMU_EXTENDED)
				} else if (arm_cache_prefer_mask != 0) {
					pmap_acquire_page_lock(omd);
					pool_put(&pmap_pv_pool,
					    pmap_kremove_pg(opg, va));
					pmap_release_page_lock(omd);
#endif
				}
			}
			if (l2pte_valid_p(opte)) {
				l2pte_reset(ptep);
				PTE_SYNC(ptep);
#ifdef PMAP_CACHE_VIVT
				cpu_dcache_wbinv_range(va, PAGE_SIZE);
#endif
				cpu_tlb_flushD_SE(va);

				mappings += PAGE_SIZE / L2_S_SIZE;
			}
			va += PAGE_SIZE;
			ptep += PAGE_SIZE / L2_S_SIZE;
		}
		KDASSERTMSG(mappings <= l2b->l2b_occupancy, "%u %u",
		    mappings, l2b->l2b_occupancy);
		l2b->l2b_occupancy -= mappings;
		//PTE_SYNC_RANGE(sptep, (u_int)(ptep - sptep));
#ifdef UVMHIST
		total_mappings += mappings;
#endif
	}
	pmap_release_pmap_lock(pmap_kernel());
	cpu_cpwait();
	UVMHIST_LOG(maphist, "  <--- done (%u mappings removed)",
	    total_mappings, 0, 0, 0);
}

#endif	/* if 0 */


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









#if 0
/*
 * pmap_pv_remove: remove an unmanaged pv-tracked page from all pmaps
 *	that map it
 */

static void
pmap_pv_remove(paddr_t pa)
{
	struct pmap_page *pp;

	pp = pmap_pv_tracked(pa);
	if (pp == NULL)
		panic("pmap_pv_protect: page not pv-tracked: 0x%"PRIxPADDR,
		    pa);

	struct vm_page_md *md = PMAP_PAGE_TO_MD(pp);
	pmap_page_remove(md, pa);
}

void
pmap_pv_protect(paddr_t pa, vm_prot_t prot)
{

	/* the only case is remove at the moment */
	KASSERT(prot == VM_PROT_NONE);
	pmap_pv_remove(pa);
}

void
pmap_protect(pmap_t pm, vaddr_t sva, vaddr_t eva, vm_prot_t prot)
{
	struct l2_bucket *l2b;
	vaddr_t next_bucket;

	NPDEBUG(PDB_PROTECT,
	    printf("pmap_protect: pm %p sva 0x%lx eva 0x%lx prot 0x%x\n",
	    pm, sva, eva, prot));

	if ((prot & VM_PROT_READ) == 0) {
		pmap_remove(pm, sva, eva);
		return;
	}

	if (prot & VM_PROT_WRITE) {
		/*
		 * If this is a read->write transition, just ignore it and let
		 * uvm_fault() take care of it later.
		 */
		return;
	}

	pmap_acquire_pmap_lock(pm);

	u_int clr_mask = PVF_WRITE | ((prot & VM_PROT_EXECUTE) ? 0 : PVF_EXEC);

	while (sva < eva) {
		next_bucket = L2_NEXT_BUCKET_VA(sva);
		if (next_bucket > eva)
			next_bucket = eva;

		l2b = pmap_get_l2_bucket(pm, sva);
		if (l2b == NULL) {
			sva = next_bucket;
			continue;
		}

		pt_entry_t *ptep = &l2b->l2b_kva[l2pte_index(sva)];

		while (sva < next_bucket) {
			const pt_entry_t opte = *ptep;
			if (l2pte_valid_p(opte) && l2pte_writable_p(opte)) {
				struct vm_page *pg;

#ifdef PMAP_CACHE_VIVT
				/*
				 * OK, at this point, we know we're doing
				 * write-protect operation.  If the pmap is
				 * active, write-back the page.
				 */
				pmap_cache_wbinv_page(pm, sva, false,
				    PVF_REF | PVF_WRITE);
#endif

				pg = PHYS_TO_VM_PAGE(l2pte_pa(opte));
				pt_entry_t npte = l2pte_set_readonly(opte);
				l2pte_reset(ptep);
				PTE_SYNC(ptep);
				pmap_tlb_invalidate_addr(pm, sva, PVF_REF);
				l2pte_set(ptep, npte, 0);
				PTE_SYNC(ptep);

				if (pg != NULL) {
					struct vm_page_md *md = VM_PAGE_TO_MD(pg);
					paddr_t pa = VM_PAGE_TO_PHYS(pg);

					pmap_acquire_page_lock(md);
					    pmap_modify_pv(md, pa, pm, sva,
					       clr_mask, 0);
					pmap_vac_me_harder(md, pa, pm, sva);
					pmap_release_page_lock(md);
				}
			}

			sva += PAGE_SIZE;
			ptep += PAGE_SIZE / L2_S_SIZE;
		}
	}


	pmap_release_pmap_lock(pm);
}

void
pmap_icache_sync_range(pmap_t pm, vaddr_t sva, vaddr_t eva)
{
	struct l2_bucket *l2b;
	pt_entry_t *ptep;
	vaddr_t next_bucket;
	vsize_t page_size = trunc_page(sva) + PAGE_SIZE - sva;

	NPDEBUG(PDB_EXEC,
	    printf("pmap_icache_sync_range: pm %p sva 0x%lx eva 0x%lx\n",
	    pm, sva, eva));

	pmap_acquire_pmap_lock(pm);

	while (sva < eva) {
		next_bucket = L2_NEXT_BUCKET_VA(sva);
		if (next_bucket > eva)
			next_bucket = eva;

		l2b = pmap_get_l2_bucket(pm, sva);
		if (l2b == NULL) {
			sva = next_bucket;
			continue;
		}

		for (ptep = &l2b->l2b_kva[l2pte_index(sva)];
		     sva < next_bucket;
		     sva += page_size,
		     ptep += PAGE_SIZE / L2_S_SIZE,
		     page_size = PAGE_SIZE) {
			if (l2pte_valid_p(*ptep)) {
				cpu_icache_sync_range(sva,
				    min(page_size, eva - sva));
			}
		}
	}

	pmap_release_pmap_lock(pm);
}

void
pmap_page_protect(struct vm_page *pg, vm_prot_t prot)
{
	struct vm_page_md *md = VM_PAGE_TO_MD(pg);
	paddr_t pa = VM_PAGE_TO_PHYS(pg);

	NPDEBUG(PDB_PROTECT,
	    printf("pmap_page_protect: md %p (0x%08lx), prot 0x%x\n",
	    md, pa, prot));

	switch(prot) {
	case VM_PROT_READ|VM_PROT_WRITE:
		pmap_acquire_page_lock(md);
		pmap_clearbit(md, pa, PVF_EXEC);
		pmap_release_page_lock(md);
		break;
	case VM_PROT_READ|VM_PROT_WRITE|VM_PROT_EXECUTE:
		break;

	case VM_PROT_READ:
		pmap_acquire_page_lock(md);
		pmap_clearbit(md, pa, PVF_WRITE|PVF_EXEC);
		pmap_release_page_lock(md);
		break;
	case VM_PROT_READ|VM_PROT_EXECUTE:
		pmap_acquire_page_lock(md);
		pmap_clearbit(md, pa, PVF_WRITE);
		pmap_release_page_lock(md);
		break;

	default:
		pmap_page_remove(md, pa);
		break;
	}
}

/*
 * pmap_clear_modify:
 *
 *	Clear the "modified" attribute for a page.
 */
bool
pmap_clear_modify(struct vm_page *pg)
{
	struct vm_page_md *md = VM_PAGE_TO_MD(pg);
	paddr_t pa = VM_PAGE_TO_PHYS(pg);
	bool rv;

	pmap_acquire_page_lock(md);

	if (md->pvh_attrs & PVF_MOD) {
		rv = true;
#if defined(PMAP_CACHE_VIPT) && !defined(ARM_MMU_EXTENDED)
		/*
		 * If we are going to clear the modified bit and there are
		 * no other modified bits set, flush the page to memory and
		 * mark it clean.
		 */
		if ((md->pvh_attrs & (PVF_DMOD|PVF_NC)) == PVF_MOD)
			pmap_flush_page(md, pa, PMAP_CLEAN_PRIMARY);
#endif
		pmap_clearbit(md, pa, PVF_MOD);
	} else {
		rv = false;
	}
	pmap_release_page_lock(md);

	return rv;
}

/*
 * pmap_clear_reference:
 *
 *	Clear the "referenced" attribute for a page.
 */
bool
pmap_clear_reference(struct vm_page *pg)
{
	struct vm_page_md *md = VM_PAGE_TO_MD(pg);
	paddr_t pa = VM_PAGE_TO_PHYS(pg);
	bool rv;

	pmap_acquire_page_lock(md);

	if (md->pvh_attrs & PVF_REF) {
		rv = true;
		pmap_clearbit(md, pa, PVF_REF);
	} else {
		rv = false;
	}
	pmap_release_page_lock(md);

	return rv;
}

/*
 * pmap_is_modified:
 *
 *	Test if a page has the "modified" attribute.
 */
/* See <arm/arm32/pmap.h> */

/*
 * pmap_is_referenced:
 *
 *	Test if a page has the "referenced" attribute.
 */
/* See <arm/arm32/pmap.h> */

#endif



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
#if defined(DEBUG) || 1
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
		arm_isb();
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
#endif

	rv = 1;

out:
	pmap_release_pmap_lock(pm);
#endif

	return rv;
}

#if 0

/*
 * Routine:	pmap_procwr
 *
 * Function:
 *	Synchronize caches corresponding to [addr, addr+len) in p.
 *
 */
void
pmap_procwr(struct proc *p, vaddr_t va, int len)
{
	/* We only need to do anything if it is the current process. */
	if (p == curproc)
		cpu_icache_sync_range(va, len);
}

/*
 * Routine:	pmap_unwire
 * Function:	Clear the wired attribute for a map/virtual-address pair.
 *
 * In/out conditions:
 *		The mapping must already exist in the pmap.
 */
void
pmap_unwire(pmap_t pm, vaddr_t va)
{
	struct l2_bucket *l2b;
	pt_entry_t *ptep, pte;
	struct vm_page *pg;
	paddr_t pa;

	NPDEBUG(PDB_WIRING, printf("pmap_unwire: pm %p, va 0x%08lx\n", pm, va));

	pmap_acquire_pmap_lock(pm);

	l2b = pmap_get_l2_bucket(pm, va);
	KDASSERT(l2b != NULL);

	ptep = &l2b->l2b_kva[l2pte_index(va)];
	pte = *ptep;

	/* Extract the physical address of the page */
	pa = l2pte_pa(pte);

	if ((pg = PHYS_TO_VM_PAGE(pa)) != NULL) {
		/* Update the wired bit in the pv entry for this page. */
		struct vm_page_md *md = VM_PAGE_TO_MD(pg);

		pmap_acquire_page_lock(md);
		(void) pmap_modify_pv(md, pa, pm, va, PVF_WIRED, 0);
		pmap_release_page_lock(md);
	}

	pmap_release_pmap_lock(pm);
}

#if 0
void
pmap_activate(struct lwp *l)
{
	struct cpu_info * const ci = curcpu();
	extern int block_userspace_access;
	pmap_t npm = l->l_proc->p_vmspace->vm_map.pmap;
	struct pmap_asid_info * const pai = PMAP_PAI(npm, cpu_tlb_info(ci));

	UVMHIST_FUNC(__func__); UVMHIST_CALLED(maphist);

	UVMHIST_LOG(maphist, "(l=%#x) pm=%#x", l, npm, 0, 0);

	/*
	 * If activating a non-current lwp or the current lwp is
	 * already active, just return.
	 */
	if (false
	    || l != curlwp
	    || (ci->ci_pmap_cur == npm &&
		(npm == pmap_kernel()
		 /* || PMAP_PAI_ASIDVALID_P(pai, cpu_tlb_info(ci)) */))
	    || false) {
		UVMHIST_LOG(maphist, " <-- (same pmap)", curlwp, l, 0, 0);
		return;
	}


	PMAPCOUNT(activations);
	block_userspace_access = 1;


#ifndef ARM_HAS_VBAR
	/*
	 * For ARM_VECTORS_LOW, we MUST, I repeat, MUST fix up the L1
	 * entry corresponding to 'vector_page' in the incoming L1 table
	 * before switching to it otherwise subsequent interrupts/exceptions
	 * (including domain faults!) will jump into hyperspace.
	 */
	if (npm->pm_pl1vec != NULL) {
		cpu_tlb_flushID_SE((u_int)vector_page);
		cpu_cpwait();
		*npm->pm_pl1vec = npm->pm_l1vec;
		PTE_SYNC(npm->pm_pl1vec);
	}
#endif

	/*
	 * Assume that TTBR1 has only global mappings and TTBR0 only has
	 * non-global mappings.  To prevent speculation from doing evil things
	 * we disable translation table walks using TTBR0 before setting the
	 * CONTEXTIDR (ASID) or new TTBR0 value.  Once both are set, table
	 * walks are reenabled.
	 */
	UVMHIST_LOG(maphist, " acquiring asid", 0, 0, 0, 0);
	const uint32_t old_ttbcr = armreg_ttbcr_read();
	armreg_ttbcr_write(old_ttbcr | TTBCR_S_PD0);
	arm_isb();
	pmap_tlb_asid_acquire(npm, l);
	UVMHIST_LOG(maphist, " setting ttbr pa=%#x asid=%#x", npm->pm_l1_pa, pai->pai_asid, 0, 0);
	cpu_setttb(npm->pm_l1_pa, pai->pai_asid);
	/*
	 * Now we can reenable tablewalks since the CONTEXTIDR and TTRB0 have
	 * been updated.
	 */
	arm_isb();
	if (npm != pmap_kernel()) {
		armreg_ttbcr_write(old_ttbcr & ~TTBCR_S_PD0);
	}
	cpu_cpwait();
	ci->ci_pmap_asid_cur = pai->pai_asid;

	block_userspace_access = 0;

	ci->ci_pmap_cur = npm;
	UVMHIST_LOG(maphist, " <-- done", 0, 0, 0, 0);
}

void
pmap_deactivate(struct lwp *l)
{
	pmap_t pm = l->l_proc->p_vmspace->vm_map.pmap;

	UVMHIST_FUNC(__func__); UVMHIST_CALLED(maphist);

	UVMHIST_LOG(maphist, "(l=%#x) pm=%#x", l, pm, 0, 0);

	kpreempt_disable();
	struct cpu_info * const ci = curcpu();
	/*
	 * Disable translation table walks from TTBR0 while no pmap has been
	 * activated.
	 */
	const uint32_t old_ttbcr = armreg_ttbcr_read();
	armreg_ttbcr_write(old_ttbcr | TTBCR_S_PD0);
	arm_isb();
	pmap_tlb_asid_deactivate(pm);
	cpu_setttb(pmap_kernel()->pm_l1_pa, KERNEL_PID);
	ci->ci_pmap_cur = pmap_kernel();
	ci->ci_pmap_asid_cur = KERNEL_PID;
	kpreempt_enable();
	UVMHIST_LOG(maphist, "  <-- done", 0, 0, 0, 0);
}
#endif


void
pmap_update(pmap_t pm)
{

	if (pm->pm_remove_all) {
		KASSERT(pm != pmap_kernel());

		KASSERTMSG(curcpu()->ci_pmap_cur != pm
		    || pm->pm_pai[0].pai_asid == curcpu()->ci_pmap_asid_cur,
		    "pmap/asid %p/%#x != %s cur pmap/asid %p/%#x", pm,
		    pm->pm_pai[0].pai_asid, curcpu()->ci_data.cpu_name,
		    curcpu()->ci_pmap_cur, curcpu()->ci_pmap_asid_cur);

#ifdef MULTIPROCESSOR
		/*
		 * Finish up the pmap_remove_all() optimisation by flushing
		 * all our ASIDs.
		 */
		// This should be the last CPU with this pmap onproc
		KASSERT(!kcpuset_isotherset(pm->pm_onproc, cpu_index(curcpu())));
		if (kcpuset_isset(pm->pm_onproc, cpu_index(curcpu()))) {
			if (pm != pmap_kernel()) {
				struct cpu_info * const ci = curcpu();
				KASSERT(!cpu_intr_p());
				/*
				 * The bits in pm_onproc that belong to this
				 * TLB can be changed while this TLBs lock is
				 * not held as long as we use atomic ops.
				 */
				kcpuset_atomic_clear(pm->pm_onproc,
				    cpu_index(ci));
			}
		}
		KASSERT(kcpuset_iszero(pm->pm_onproc));
#endif
		struct pmap_asid_info * const pai =
		    PMAP_PAI(pm, cpu_tlb_info(ci));

		tlb_invalidate_asids(pai->pai_asid, pai->pai_asid);

		pm->pm_remove_all = false;
	}

        if (arm_has_mpext_p)
                armreg_bpiallis_write(0);
        else
                armreg_bpiall_write(0);

#if defined(MULTIPROCESSOR) && PMAP_TLB_MAX > 1
	u_int pending = atomic_swap_uint(&pmap->pm_shootdown_pending, 0);
	if (pending && pmap_tlb_shootdown_bystanders(pmap)) {
		PMAP_COUNT(shootdown_ipis);
	}
#endif
	KASSERTMSG(pm == pmap_kernel()
	    || curcpu()->ci_pmap_cur != pm
	    || pm->pm_pai[0].pai_asid == curcpu()->ci_pmap_asid_cur,
	    "pmap/asid %p/%#x != %s cur pmap/asid %p/%#x", pm,
	    pm->pm_pai[0].pai_asid, curcpu()->ci_data.cpu_name,
	    curcpu()->ci_pmap_cur, curcpu()->ci_pmap_asid_cur);

	PMAPCOUNT(updates);

	/*
	 * make sure TLB/cache operations have completed.
	 */
	cpu_cpwait();
}

void
pmap_remove_all(pmap_t pm)
{

	/*
	 * The vmspace described by this pmap is about to be torn down.
	 * Until pmap_update() is called, UVM will only make calls
	 * to pmap_remove(). We can make life much simpler by flushing
	 * the cache now, and deferring TLB invalidation to pmap_update().
	 */
#ifdef PMAP_CACHE_VIVT
	pmap_cache_wbinv_all(pm, PVF_EXEC);
#endif

#ifdef MULTIPROCESSOR
	struct cpu_info * const ci = curcpu();
	// This should be the last CPU with this pmap onproc
	KASSERT(!kcpuset_isotherset(pm->pm_onproc, cpu_index(ci)));
	if (kcpuset_isset(pm->pm_onproc, cpu_index(ci)))
#endif
		pmap_tlb_asid_deactivate(pm);
#ifdef MULTIPROCESSOR
	KASSERT(kcpuset_iszero(pm->pm_onproc));
#endif

	pmap_tlb_asid_release_all(pm);

	pm->pm_remove_all = true;
}

/*
 * Retire the given physical map from service.
 * Should only be called if the map contains no valid mappings.
 */
void
pmap_destroy(pmap_t pm)
{
	u_int count;

	if (pm == NULL)
		return;

	if (pm->pm_remove_all) {
		pmap_tlb_asid_release_all(pm);
		pm->pm_remove_all = false;
	}

	/*
	 * Drop reference count
	 */
	mutex_enter(pm->pm_lock);
	count = --pm->pm_obj.uo_refs;
	mutex_exit(pm->pm_lock);
	if (count > 0) {
		return;
	}

	/*
	 * reference count is zero, free pmap resources and then free pmap.
	 */

#ifndef ARM_HAS_VBAR
	if (vector_page < KERNEL_BASE) {
		KDASSERT(!pmap_is_current(pm));

		/* Remove the vector page mapping */
		pmap_remove(pm, vector_page, vector_page + PAGE_SIZE);
		pmap_update(pm);
	}
#endif

	pmap_free_l1(pm);

#ifdef MULTIPROCESSOR
	kcpuset_destroy(pm->pm_active);
	kcpuset_destroy(pm->pm_onproc);
#endif

	uvm_obj_destroy(&pm->pm_obj, false);
	mutex_destroy(&pm->pm_obj_lock);
	pool_cache_put(&pmap_cache, pm);
}


/*
 * void pmap_reference(pmap_t pm)
 *
 * Add a reference to the specified pmap.
 */
void
pmap_reference(pmap_t pm)
{

	if (pm == NULL)
		return;


	mutex_enter(pm->pm_lock);
	pm->pm_obj.uo_refs++;
	mutex_exit(pm->pm_lock);
}

#if (ARM_MMU_V6 + ARM_MMU_V7) > 0

static struct evcnt pmap_prefer_nochange_ev =
    EVCNT_INITIALIZER(EVCNT_TYPE_MISC, NULL, "pmap prefer", "nochange");
static struct evcnt pmap_prefer_change_ev =
    EVCNT_INITIALIZER(EVCNT_TYPE_MISC, NULL, "pmap prefer", "change");

EVCNT_ATTACH_STATIC(pmap_prefer_change_ev);
EVCNT_ATTACH_STATIC(pmap_prefer_nochange_ev);

void
pmap_prefer(vaddr_t hint, vaddr_t *vap, int td)
{
	vsize_t mask = arm_cache_prefer_mask | (PAGE_SIZE - 1);
	vaddr_t va = *vap;
	vaddr_t diff = (hint - va) & mask;
	if (diff == 0) {
		pmap_prefer_nochange_ev.ev_count++;
	} else {
		pmap_prefer_change_ev.ev_count++;
		if (__predict_false(td))
			va -= mask + 1;
		*vap = va + diff;
	}
}
#endif /* ARM_MMU_V6 | ARM_MMU_V7 */
#endif



#if 0


/*
 * pmap_zero_page()
 *
 * Zero a given physical page by mapping it at a page hook point.
 * In doing the zero page op, the page we zero is mapped cachable, as with
 * StrongARM accesses to non-cached pages are non-burst making writing
 * _any_ bulk data very slow.
 */
#if (ARM_MMU_GENERIC + ARM_MMU_SA1 + ARM_MMU_V6 + ARM_MMU_V7) != 0
void
pmap_zero_page_generic(paddr_t pa)
{
#if defined(PMAP_CACHE_VIPT) || defined(DEBUG)
	struct vm_page *pg = PHYS_TO_VM_PAGE(pa);
	struct vm_page_md *md = VM_PAGE_TO_MD(pg);
#endif
#if defined(PMAP_CACHE_VIPT)
	/* Choose the last page color it had, if any */
	const vsize_t va_offset = md->pvh_attrs & arm_cache_prefer_mask;
#else
	const vsize_t va_offset = 0;
#endif
#if defined(__HAVE_MM_MD_DIRECT_MAPPED_PHYS)
	/*
	 * Is this page mapped at its natural color?
	 * If we have all of memory mapped, then just convert PA to VA.
	 */
	bool okcolor = arm_pcache.dcache_type == CACHE_TYPE_PIPT
	   || va_offset == (pa & arm_cache_prefer_mask);
	const vaddr_t vdstp = okcolor
	    ? pmap_direct_mapped_phys(pa, &okcolor, cpu_cdstp(va_offset))
	    : cpu_cdstp(va_offset);
#else
	const bool okcolor = false;
	const vaddr_t vdstp = cpu_cdstp(va_offset);
#endif
	pt_entry_t * const ptep = cpu_cdst_pte(va_offset);


#ifdef DEBUG
	if (!SLIST_EMPTY(&md->pvh_list))
		panic("pmap_zero_page: page has mappings");
#endif

	KDASSERT((pa & PGOFSET) == 0);

	if (!okcolor) {
		/*
		 * Hook in the page, zero it, and purge the cache for that
		 * zeroed page. Invalidate the TLB as needed.
		 */
		const pt_entry_t npte = L2_S_PROTO | pa | pte_l2_s_cache_mode
		    | L2_S_PROT(PTE_KERNEL, VM_PROT_WRITE);
		l2pte_set(ptep, npte, 0);
		PTE_SYNC(ptep);
		cpu_tlb_flushD_SE(vdstp);
		cpu_cpwait();
#if defined(__HAVE_MM_MD_DIRECT_MAPPED_PHYS) && defined(PMAP_CACHE_VIPT) \
    && !defined(ARM_MMU_EXTENDED)
		/*
		 * If we are direct-mapped and our color isn't ok, then before
		 * we bzero the page invalidate its contents from the cache and
		 * reset the color to its natural color.
		 */
		cpu_dcache_inv_range(vdstp, PAGE_SIZE);
		md->pvh_attrs &= ~arm_cache_prefer_mask;
		md->pvh_attrs |= (pa & arm_cache_prefer_mask);
#endif
	}
	bzero_page(vdstp);
	if (!okcolor) {
		/*
		 * Unmap the page.
		 */
		l2pte_reset(ptep);
		PTE_SYNC(ptep);
		cpu_tlb_flushD_SE(vdstp);
#ifdef PMAP_CACHE_VIVT
		cpu_dcache_wbinv_range(vdstp, PAGE_SIZE);
#endif
	}
#ifdef PMAP_CACHE_VIPT
	/*
	 * This page is now cache resident so it now has a page color.
	 * Any contents have been obliterated so clear the EXEC flag.
	 */
	if (PV_IS_EXEC_P(md->pvh_attrs)) {
		md->pvh_attrs &= ~PVF_EXEC;
		PMAPCOUNT(exec_discarded_zero);
	}
#endif
}
#endif /* (ARM_MMU_GENERIC + ARM_MMU_SA1 + ARM_MMU_V6) != 0 */

#if ARM_MMU_XSCALE == 1
void
pmap_zero_page_xscale(paddr_t pa)
{
#ifdef DEBUG
	struct vm_page *pg = PHYS_TO_VM_PAGE(pa);
	struct vm_page_md *md = VM_PAGE_TO_MD(pg);

	if (!SLIST_EMPTY(&md->pvh_list))
		panic("pmap_zero_page: page has mappings");
#endif

	KDASSERT((pa & PGOFSET) == 0);

	/*
	 * Hook in the page, zero it, and purge the cache for that
	 * zeroed page. Invalidate the TLB as needed.
	 */

	pt_entry_t npte = L2_S_PROTO | pa |
	    L2_S_PROT(PTE_KERNEL, VM_PROT_WRITE) |
	    L2_C | L2_XS_T_TEX(TEX_XSCALE_X);	/* mini-data */
	l2pte_set(cdst_pte, npte, 0);
	PTE_SYNC(cdst_pte);
	cpu_tlb_flushD_SE(cdstp);
	cpu_cpwait();
	bzero_page(cdstp);
	xscale_cache_clean_minidata();
	l2pte_reset(cdst_pte);
	PTE_SYNC(cdst_pte);
}
#endif /* ARM_MMU_XSCALE == 1 */

/* pmap_pageidlezero()
 *
 * The same as above, except that we assume that the page is not
 * mapped.  This means we never have to flush the cache first.  Called
 * from the idle loop.
 */
bool
pmap_pageidlezero(paddr_t pa)
{
	bool rv = true;
#if defined(PMAP_CACHE_VIPT) || defined(DEBUG)
	struct vm_page * const pg = PHYS_TO_VM_PAGE(pa);
	struct vm_page_md *md = VM_PAGE_TO_MD(pg);
#endif
#ifdef PMAP_CACHE_VIPT
	/* Choose the last page color it had, if any */
	const vsize_t va_offset = md->pvh_attrs & arm_cache_prefer_mask;
#else
	const vsize_t va_offset = 0;
#endif
#ifdef __HAVE_MM_MD_DIRECT_MAPPED_PHYS
	bool okcolor = arm_pcache.dcache_type == CACHE_TYPE_PIPT
	   || va_offset == (pa & arm_cache_prefer_mask);
	const vaddr_t vdstp = okcolor
	    ? pmap_direct_mapped_phys(pa, &okcolor, cpu_cdstp(va_offset))
	    : cpu_cdstp(va_offset);
#else
	const bool okcolor = false;
	const vaddr_t vdstp = cpu_cdstp(va_offset);
#endif
	pt_entry_t * const ptep = cpu_cdst_pte(va_offset);


#ifdef DEBUG
	if (!SLIST_EMPTY(&md->pvh_list))
		panic("pmap_pageidlezero: page has mappings");
#endif

	KDASSERT((pa & PGOFSET) == 0);

	if (!okcolor) {
		/*
		 * Hook in the page, zero it, and purge the cache for that
		 * zeroed page. Invalidate the TLB as needed.
		 */
		const pt_entry_t npte = L2_S_PROTO | pa |
		    L2_S_PROT(PTE_KERNEL, VM_PROT_WRITE) | pte_l2_s_cache_mode;
		l2pte_set(ptep, npte, 0);
		PTE_SYNC(ptep);
		cpu_tlb_flushD_SE(vdstp);
		cpu_cpwait();
	}

	uint64_t *ptr = (uint64_t *)vdstp;
	for (size_t i = 0; i < PAGE_SIZE / sizeof(*ptr); i++) {
		if (sched_curcpu_runnable_p() != 0) {
			/*
			 * A process has become ready.  Abort now,
			 * so we don't keep it waiting while we
			 * do slow memory access to finish this
			 * page.
			 */
			rv = false;
			break;
		}
		*ptr++ = 0;
	}

#ifdef PMAP_CACHE_VIVT
	if (rv)
		/*
		 * if we aborted we'll rezero this page again later so don't
		 * purge it unless we finished it
		 */
		cpu_dcache_wbinv_range(vdstp, PAGE_SIZE);
#elif defined(PMAP_CACHE_VIPT)
	/*
	 * This page is now cache resident so it now has a page color.
	 * Any contents have been obliterated so clear the EXEC flag.
	 */
	if (PV_IS_EXEC_P(md->pvh_attrs)) {
		md->pvh_attrs &= ~PVF_EXEC;
		PMAPCOUNT(exec_discarded_zero);
	}
#endif
	/*
	 * Unmap the page.
	 */
	if (!okcolor) {
		l2pte_reset(ptep);
		PTE_SYNC(ptep);
		cpu_tlb_flushD_SE(vdstp);
	}

	return rv;
}

/*
 * pmap_copy_page()
 *
 * Copy one physical page into another, by mapping the pages into
 * hook points. The same comment regarding cachability as in
 * pmap_zero_page also applies here.
 */
#if (ARM_MMU_GENERIC + ARM_MMU_SA1 + ARM_MMU_V6 + ARM_MMU_V7) != 0
void
pmap_copy_page_generic(paddr_t src, paddr_t dst)
{
	struct vm_page * const src_pg = PHYS_TO_VM_PAGE(src);
	struct vm_page_md *src_md = VM_PAGE_TO_MD(src_pg);
#if defined(PMAP_CACHE_VIPT) || defined(DEBUG)
	struct vm_page * const dst_pg = PHYS_TO_VM_PAGE(dst);
	struct vm_page_md *dst_md = VM_PAGE_TO_MD(dst_pg);
#endif
#ifdef PMAP_CACHE_VIPT
	const vsize_t src_va_offset = src_md->pvh_attrs & arm_cache_prefer_mask;
	const vsize_t dst_va_offset = dst_md->pvh_attrs & arm_cache_prefer_mask;
#else
	const vsize_t src_va_offset = 0;
	const vsize_t dst_va_offset = 0;
#endif
#if defined(__HAVE_MM_MD_DIRECT_MAPPED_PHYS)
	/*
	 * Is this page mapped at its natural color?
	 * If we have all of memory mapped, then just convert PA to VA.
	 */
	bool src_okcolor = arm_pcache.dcache_type == CACHE_TYPE_PIPT
	    || src_va_offset == (src & arm_cache_prefer_mask);
	bool dst_okcolor = arm_pcache.dcache_type == CACHE_TYPE_PIPT
	    || dst_va_offset == (dst & arm_cache_prefer_mask);
	const vaddr_t vsrcp = src_okcolor
	    ? pmap_direct_mapped_phys(src, &src_okcolor,
		cpu_csrcp(src_va_offset))
	    : cpu_csrcp(src_va_offset);
	const vaddr_t vdstp = pmap_direct_mapped_phys(dst, &dst_okcolor,
	    cpu_cdstp(dst_va_offset));
#else
	const bool src_okcolor = false;
	const bool dst_okcolor = false;
	const vaddr_t vsrcp = cpu_csrcp(src_va_offset);
	const vaddr_t vdstp = cpu_cdstp(dst_va_offset);
#endif
	pt_entry_t * const src_ptep = cpu_csrc_pte(src_va_offset);
	pt_entry_t * const dst_ptep = cpu_cdst_pte(dst_va_offset);

#ifdef DEBUG
	if (!SLIST_EMPTY(&dst_md->pvh_list))
		panic("pmap_copy_page: dst page has mappings");
#endif

#if defined(PMAP_CACHE_VIPT) && !defined(ARM_MMU_EXTENDED)
	KASSERT(arm_cache_prefer_mask == 0 || src_md->pvh_attrs & (PVF_COLORED|PVF_NC));
#endif
	KDASSERT((src & PGOFSET) == 0);
	KDASSERT((dst & PGOFSET) == 0);

	/*
	 * Clean the source page.  Hold the source page's lock for
	 * the duration of the copy so that no other mappings can
	 * be created while we have a potentially aliased mapping.
	 */
#ifdef PMAP_CACHE_VIVT
	pmap_acquire_page_lock(src_md);
	(void) pmap_clean_page(src_md, true);
	pmap_release_page_lock(src_md);
#endif

	/*
	 * Map the pages into the page hook points, copy them, and purge
	 * the cache for the appropriate page. Invalidate the TLB
	 * as required.
	 */
	if (!src_okcolor) {
		const pt_entry_t nsrc_pte = L2_S_PROTO
		    | src
#if defined(PMAP_CACHE_VIPT) && !defined(ARM_MMU_EXTENDED)
		    | ((src_md->pvh_attrs & PVF_NC) ? 0 : pte_l2_s_cache_mode)
#else // defined(PMAP_CACHE_VIVT) || defined(ARM_MMU_EXTENDED)
		    | pte_l2_s_cache_mode
#endif
		    | L2_S_PROT(PTE_KERNEL, VM_PROT_READ);
		l2pte_set(src_ptep, nsrc_pte, 0);
		PTE_SYNC(src_ptep);
		cpu_tlb_flushD_SE(vsrcp);
		cpu_cpwait();
	}
	if (!dst_okcolor) {
		const pt_entry_t ndst_pte = L2_S_PROTO | dst |
		    L2_S_PROT(PTE_KERNEL, VM_PROT_WRITE) | pte_l2_s_cache_mode;
		l2pte_set(dst_ptep, ndst_pte, 0);
		PTE_SYNC(dst_ptep);
		cpu_tlb_flushD_SE(vdstp);
		cpu_cpwait();
#if defined(__HAVE_MM_MD_DIRECT_MAPPED_PHYS) && defined(PMAP_CACHE_VIPT)
		/*
		 * If we are direct-mapped and our color isn't ok, then before
		 * we bcopy to the new page invalidate its contents from the
		 * cache and reset its color to its natural color.
		 */
		cpu_dcache_inv_range(vdstp, PAGE_SIZE);
		dst_md->pvh_attrs &= ~arm_cache_prefer_mask;
		dst_md->pvh_attrs |= (dst & arm_cache_prefer_mask);
#endif
	}
	bcopy_page(vsrcp, vdstp);
#ifdef PMAP_CACHE_VIVT
	cpu_dcache_inv_range(vsrcp, PAGE_SIZE);
	cpu_dcache_wbinv_range(vdstp, PAGE_SIZE);
#endif
	/*
	 * Unmap the pages.
	 */
	if (!src_okcolor) {
		l2pte_reset(src_ptep);
		PTE_SYNC(src_ptep);
		cpu_tlb_flushD_SE(vsrcp);
		cpu_cpwait();
	}
	if (!dst_okcolor) {
		l2pte_reset(dst_ptep);
		PTE_SYNC(dst_ptep);
		cpu_tlb_flushD_SE(vdstp);
		cpu_cpwait();
	}
#ifdef PMAP_CACHE_VIPT
	/*
	 * Now that the destination page is in the cache, mark it as colored.
	 * If this was an exec page, discard it.
	 */
	pmap_acquire_page_lock(dst_md);
	if (PV_IS_EXEC_P(dst_md->pvh_attrs)) {
		dst_md->pvh_attrs &= ~PVF_EXEC;
		PMAPCOUNT(exec_discarded_copy);
	}
	pmap_release_page_lock(dst_md);
#endif
}
#endif /* (ARM_MMU_GENERIC + ARM_MMU_SA1 + ARM_MMU_V6) != 0 */

#if ARM_MMU_XSCALE == 1
void
pmap_copy_page_xscale(paddr_t src, paddr_t dst)
{
	struct vm_page *src_pg = PHYS_TO_VM_PAGE(src);
	struct vm_page_md *src_md = VM_PAGE_TO_MD(src_pg);
#ifdef DEBUG
	struct vm_page_md *dst_md = VM_PAGE_TO_MD(PHYS_TO_VM_PAGE(dst));

	if (!SLIST_EMPTY(&dst_md->pvh_list))
		panic("pmap_copy_page: dst page has mappings");
#endif

	KDASSERT((src & PGOFSET) == 0);
	KDASSERT((dst & PGOFSET) == 0);

	/*
	 * Clean the source page.  Hold the source page's lock for
	 * the duration of the copy so that no other mappings can
	 * be created while we have a potentially aliased mapping.
	 */
#ifdef PMAP_CACHE_VIVT
	pmap_acquire_page_lock(src_md);
	(void) pmap_clean_page(src_md, true);
	pmap_release_page_lock(src_md);
#endif

	/*
	 * Map the pages into the page hook points, copy them, and purge
	 * the cache for the appropriate page. Invalidate the TLB
	 * as required.
	 */
	const pt_entry_t nsrc_pte = L2_S_PROTO | src
	    | L2_S_PROT(PTE_KERNEL, VM_PROT_READ)
	    | L2_C | L2_XS_T_TEX(TEX_XSCALE_X);	/* mini-data */
	l2pte_set(csrc_pte, nsrc_pte, 0);
	PTE_SYNC(csrc_pte);

	const pt_entry_t ndst_pte = L2_S_PROTO | dst
	    | L2_S_PROT(PTE_KERNEL, VM_PROT_WRITE)
	    | L2_C | L2_XS_T_TEX(TEX_XSCALE_X);	/* mini-data */
	l2pte_set(cdst_pte, ndst_pte, 0);
	PTE_SYNC(cdst_pte);

	cpu_tlb_flushD_SE(csrcp);
	cpu_tlb_flushD_SE(cdstp);
	cpu_cpwait();
	bcopy_page(csrcp, cdstp);
	xscale_cache_clean_minidata();
	l2pte_reset(csrc_pte);
	l2pte_reset(cdst_pte);
	PTE_SYNC(csrc_pte);
	PTE_SYNC(cdst_pte);
}
#endif /* ARM_MMU_XSCALE == 1 */

#endif


#if 0

/*
 * void pmap_virtual_space(vaddr_t *start, vaddr_t *end)
 *
 * Return the start and end addresses of the kernel's virtual space.
 * These values are setup in pmap_bootstrap and are updated as pages
 * are allocated.
 */
void
pmap_virtual_space(vaddr_t *start, vaddr_t *end)
{
	*start = virtual_avail;
	*end = virtual_end;
}

/*
 * Helper function for pmap_grow_l2_bucket()
 */
static inline int
pmap_grow_map(vaddr_t va, paddr_t *pap)
{
	paddr_t pa;

	if (uvm.page_init_done == false) {
#ifdef PMAP_STEAL_MEMORY
		pv_addr_t pv;
		pmap_boot_pagealloc(PAGE_SIZE,
#ifdef PMAP_CACHE_VIPT
		    arm_cache_prefer_mask,
		    va & arm_cache_prefer_mask,
#else
		    0, 0,
#endif
		    &pv);
		pa = pv.pv_pa;
#else
		if (uvm_page_physget(&pa) == false)
			return (1);
#endif	/* PMAP_STEAL_MEMORY */
	} else {
		struct vm_page *pg;
		pg = uvm_pagealloc(NULL, 0, NULL, UVM_PGA_USERESERVE);
		if (pg == NULL)
			return (1);
		pa = VM_PAGE_TO_PHYS(pg);
		/*
		 * This new page must not have any mappings.  Enter it via
		 * pmap_kenter_pa and let that routine do the hard work.
		 */
		struct vm_page_md *md __diagused = VM_PAGE_TO_MD(pg);
		KASSERT(SLIST_EMPTY(&md->pvh_list));
		pmap_kenter_pa(va, pa,
		    VM_PROT_READ|VM_PROT_WRITE, PMAP_KMPAGE|PMAP_PTE);
	}

	if (pap)
		*pap = pa;

	PMAPCOUNT(pt_mappings);
#ifdef DEBUG
	struct l2_bucket * const l2b = pmap_get_l2_bucket(pmap_kernel(), va);
	KDASSERT(l2b != NULL);

	pt_entry_t * const ptep = &l2b->l2b_kva[l2pte_index(va)];
	const pt_entry_t opte = *ptep;
	KDASSERT((opte & L2_S_CACHE_MASK) == pte_l2_s_cache_mode_pt);
#endif
	memset((void *)va, 0, PAGE_SIZE);
	return (0);
}

/*
 * This is the same as pmap_alloc_l2_bucket(), except that it is only
 * used by pmap_growkernel().
 */
static inline struct l2_bucket *
pmap_grow_l2_bucket(pmap_t pm, vaddr_t va)
{
	struct l2_dtable *l2;
	struct l2_bucket *l2b;
	u_short l1slot;
	vaddr_t nva;

	l1slot = l1pte_index(va);

	if ((l2 = pm->pm_l2[L2_IDX(l1slot)]) == NULL) {
		/*
		 * No mapping at this address, as there is
		 * no entry in the L1 table.
		 * Need to allocate a new l2_dtable.
		 */
		nva = pmap_kernel_l2dtable_kva;
		if ((nva & PGOFSET) == 0) {
			/*
			 * Need to allocate a backing page
			 */
			if (pmap_grow_map(nva, NULL))
				return (NULL);
		}

		l2 = (struct l2_dtable *)nva;
		nva += sizeof(struct l2_dtable);

		if ((nva & PGOFSET) < (pmap_kernel_l2dtable_kva & PGOFSET)) {
			/*
			 * The new l2_dtable straddles a page boundary.
			 * Map in another page to cover it.
			 */
			if (pmap_grow_map(nva, NULL))
				return (NULL);
		}

		pmap_kernel_l2dtable_kva = nva;

		/*
		 * Link it into the parent pmap
		 */
		pm->pm_l2[L2_IDX(l1slot)] = l2;
	}

	l2b = &l2->l2_bucket[L2_BUCKET(l1slot)];

	/*
	 * Fetch pointer to the L2 page table associated with the address.
	 */
	if (l2b->l2b_kva == NULL) {
		pt_entry_t *ptep;

		/*
		 * No L2 page table has been allocated. Chances are, this
		 * is because we just allocated the l2_dtable, above.
		 */
		nva = pmap_kernel_l2ptp_kva;
		ptep = (pt_entry_t *)nva;
		if ((nva & PGOFSET) == 0) {
			/*
			 * Need to allocate a backing page
			 */
			if (pmap_grow_map(nva, &pmap_kernel_l2ptp_phys))
				return (NULL);
			PTE_SYNC_RANGE(ptep, PAGE_SIZE / sizeof(pt_entry_t));
		}

		l2->l2_occupancy++;
		l2b->l2b_kva = ptep;
		l2b->l2b_l1slot = l1slot;
		l2b->l2b_pa = pmap_kernel_l2ptp_phys;

		pmap_kernel_l2ptp_kva += L2_TABLE_SIZE_REAL;
		pmap_kernel_l2ptp_phys += L2_TABLE_SIZE_REAL;
	}

	return (l2b);
}


vaddr_t
pmap_growkernel(vaddr_t maxkvaddr)
{
	pmap_t kpm = pmap_kernel();
	int s;

	if (maxkvaddr <= pmap_curmaxkvaddr)
		goto out;		/* we are OK */

	NPDEBUG(PDB_GROWKERN,
	    printf("pmap_growkernel: growing kernel from 0x%lx to 0x%lx\n",
	    pmap_curmaxkvaddr, maxkvaddr));

	KDASSERT(maxkvaddr <= virtual_end);

	/*
	 * whoops!   we need to add kernel PTPs
	 */

	s = splhigh();	/* to be safe */
	mutex_enter(kpm->pm_lock);

	/* Map 1MB at a time */
	size_t l1slot = l1pte_index(pmap_curmaxkvaddr);
	pd_entry_t * const spdep = &kpm->pm_l1[l1slot];
	pd_entry_t *pdep = spdep;
	for (;pmap_curmaxkvaddr < maxkvaddr; pmap_curmaxkvaddr += L1_S_SIZE,
	     pdep++,
	     l1slot++) {
		struct l2_bucket *l2b =
		    pmap_grow_l2_bucket(kpm, pmap_curmaxkvaddr);
		KASSERT(l2b != NULL);

		const pd_entry_t npde = L1_C_PROTO | l2b->l2b_pa
		    | L1_C_DOM(PMAP_DOMAIN_KERNEL);
		l1pte_setone(pdep, npde);
	}
	PDE_SYNC_RANGE(spdep, pdep - spdep);

#ifdef PMAP_CACHE_VIVT
	/*
	 * flush out the cache, expensive but growkernel will happen so
	 * rarely
	 */
	cpu_dcache_wbinv_all();
	cpu_tlb_flushD();
	cpu_cpwait();
#endif

	mutex_exit(kpm->pm_lock);
	splx(s);

out:
	return (pmap_curmaxkvaddr);
}

/************************ Utility routines ****************************/

#ifndef ARM_HAS_VBAR
/*
 * vector_page_setprot:
 *
 *	Manipulate the protection of the vector page.
 */
void
vector_page_setprot(int prot)
{
	struct l2_bucket *l2b;
	pt_entry_t *ptep;

#if defined(CPU_ARMV7) || defined(CPU_ARM11)
	/*
	 * If we are using VBAR to use the vectors in the kernel, then it's
	 * already mapped in the kernel text so no need to anything here.
	 */
	if (vector_page != ARM_VECTORS_LOW && vector_page != ARM_VECTORS_HIGH) {
		KASSERT((armreg_pfr1_read() & ARM_PFR1_SEC_MASK) != 0);
		return;
	}
#endif

	l2b = pmap_get_l2_bucket(pmap_kernel(), vector_page);
	KASSERT(l2b != NULL);

	ptep = &l2b->l2b_kva[l2pte_index(vector_page)];

	const pt_entry_t opte = *ptep;
	const pt_entry_t npte = (opte & ~(L2_S_PROT_MASK|L2_XS_XN))
	    | L2_S_PROT(PTE_KERNEL, prot);
	l2pte_set(ptep, npte, opte);
	PTE_SYNC(ptep);
	cpu_tlb_flushD_SE(vector_page);
	cpu_cpwait();
}
#endif
#endif

#if 0

/*
 * Fetch pointers to the PDE/PTE for the given pmap/VA pair.
 * Returns true if the mapping exists, else false.
 *
 * NOTE: This function is only used by a couple of arm-specific modules.
 * It is not safe to take any pmap locks here, since we could be right
 * in the middle of debugging the pmap anyway...
 *
 * It is possible for this routine to return false even though a valid
 * mapping does exist. This is because we don't lock, so the metadata
 * state may be inconsistent.
 *
 * NOTE: We can return a NULL *ptp in the case where the L1 pde is
 * a "section" mapping.
 */
bool
pmap_get_pde_pte(pmap_t pm, vaddr_t va, pd_entry_t **pdp, pt_entry_t **ptp)
{
	struct l2_dtable *l2;
	pd_entry_t *pdep, pde;
	pt_entry_t *ptep;
	u_short l1slot;

	if (pm->pm_l1 == NULL)
		return false;

	l1slot = l1pte_index(va);
	*pdp = pdep = pmap_l1_kva(pm) + l1slot;
	pde = *pdep;

	if (l1pte_section_p(pde)) {
		*ptp = NULL;
		return true;
	}

	l2 = pm->pm_l2[L2_IDX(l1slot)];
	if (l2 == NULL ||
	    (ptep = l2->l2_bucket[L2_BUCKET(l1slot)].l2b_kva) == NULL) {
		return false;
	}

	*ptp = &ptep[l2pte_index(va)];
	return true;
}

bool
pmap_get_pde(pmap_t pm, vaddr_t va, pd_entry_t **pdp)
{

	if (pm->pm_l1 == NULL)
		return false;

	*pdp = pmap_l1_kva(pm) + l1pte_index(va);

	return true;
}
#endif



#if 0
/************************ Bootstrapping routines ****************************/


/*
 * pmap_bootstrap() is called from the board-specific initarm() routine
 * once the kernel L1/L2 descriptors tables have been set up.
 *
 * This is a somewhat convoluted process since pmap bootstrap is, effectively,
 * spread over a number of disparate files/functions.
 *
 * We are passed the following parameters
 *  - vstart
 *    1MB-aligned start of managed kernel virtual memory.
 *  - vend
 *    1MB-aligned end of managed kernel virtual memory.
 *
 * We use 'kernel_l1pt' to build the metadata (struct l1_ttable and
 * struct l2_dtable) necessary to track kernel mappings.
 */
#define	PMAP_STATIC_L2_SIZE 16
void
pmap_bootstrap(vaddr_t vstart, vaddr_t vend)
{
	static struct l2_dtable static_l2[PMAP_STATIC_L2_SIZE];
	struct l2_dtable *l2;
	struct l2_bucket *l2b;
	pd_entry_t *l1pt = (pd_entry_t *) kernel_l1pt.pv_va;
	pmap_t pm = pmap_kernel();
	pt_entry_t *ptep;
	paddr_t pa;
	vsize_t size;
	int nptes, l2idx, l2next = 0;

	KASSERT(pte_l1_s_cache_mode == pte_l1_s_cache_mode_pt);
	KASSERT(pte_l2_s_cache_mode == pte_l2_s_cache_mode_pt);

	VPRINTF("kpm ");
	/*
	 * Initialise the kernel pmap object
	 */
	curcpu()->ci_pmap_cur = pm;
	pm->pm_l1 = l1pt;
	pm->pm_l1_pa = kernel_l1pt.pv_pa;

#ifdef MULTIPROCESSOR
	VPRINTF("kcpusets ");
	pm->pm_onproc = kcpuset_running;
	pm->pm_active = kcpuset_running;
#endif

	VPRINTF("tlb0 ");
	pmap_tlb_info_init(&pmap_tlb0_info);


	VPRINTF("locks ");
	mutex_init(&pmap_lock, MUTEX_DEFAULT, IPL_NONE);
	mutex_init(&pm->pm_obj_lock, MUTEX_DEFAULT, IPL_NONE);
	uvm_obj_init(&pm->pm_obj, NULL, false, 1);
	uvm_obj_setlock(&pm->pm_obj, &pm->pm_obj_lock);

	VPRINTF("l1pt ");
	/*
	 * Scan the L1 translation table created by initarm() and create
	 * the required metadata for all valid mappings found in it.
	 */
	for (size_t l1slot = 0;
	     l1slot < L1_TABLE_SIZE / sizeof(pd_entry_t);
	     l1slot++) {
		pd_entry_t pde = l1pt[l1slot];

		/*
		 * We're only interested in Coarse mappings.
		 * pmap_extract() can deal with section mappings without
		 * recourse to checking L2 metadata.
		 */
		if ((pde & L1_TYPE_MASK) != L1_TYPE_C)
			continue;

		/*
		 * Lookup the KVA of this L2 descriptor table
		 */
		pa = l1pte_pa(pde);
		ptep = (pt_entry_t *)kernel_pt_lookup(pa);
		if (ptep == NULL) {
			panic("pmap_bootstrap: No L2 for va 0x%x, pa 0x%lx",
			    (u_int)l1slot << L1_S_SHIFT, pa);
		}

		/*
		 * Fetch the associated L2 metadata structure.
		 * Allocate a new one if necessary.
		 */
		if ((l2 = pm->pm_l2[L2_IDX(l1slot)]) == NULL) {
			if (l2next == PMAP_STATIC_L2_SIZE)
				panic("pmap_bootstrap: out of static L2s");
			pm->pm_l2[L2_IDX(l1slot)] = l2 = &static_l2[l2next++];
		}

		/*
		 * One more L1 slot tracked...
		 */
		l2->l2_occupancy++;

		/*
		 * Fill in the details of the L2 descriptor in the
		 * appropriate bucket.
		 */
		l2b = &l2->l2_bucket[L2_BUCKET(l1slot)];
		l2b->l2b_kva = ptep;
		l2b->l2b_pa = pa;
		l2b->l2b_l1slot = l1slot;

		/*
		 * Establish an initial occupancy count for this descriptor
		 */
		for (l2idx = 0;
		    l2idx < (L2_TABLE_SIZE_REAL / sizeof(pt_entry_t));
		    l2idx++) {
			if ((ptep[l2idx] & L2_TYPE_MASK) != L2_TYPE_INV) {
				l2b->l2b_occupancy++;
			}
		}

		/*
		 * Make sure the descriptor itself has the correct cache mode.
		 * If not, fix it, but whine about the problem. Port-meisters
		 * should consider this a clue to fix up their initarm()
		 * function. :)
		 */
		if (pmap_set_pt_cache_mode(l1pt, (vaddr_t)ptep, 1)) {
			printf("pmap_bootstrap: WARNING! wrong cache mode for "
			    "L2 pte @ %p\n", ptep);
		}
	}

	VPRINTF("cache(l1pt) ");
	/*
	 * Ensure the primary (kernel) L1 has the correct cache mode for
	 * a page table. Bitch if it is not correctly set.
	 */
	if (pmap_set_pt_cache_mode(l1pt, kernel_l1pt.pv_va,
		    L1_TABLE_SIZE / L2_S_SIZE)) {
		printf("pmap_bootstrap: WARNING! wrong cache mode for "
		    "primary L1 @ 0x%lx\n", kernel_l1pt.pv_va);
	}

#if 0
#ifdef PMAP_CACHE_VIVT
	cpu_dcache_wbinv_all();
	cpu_tlb_flushID();
	cpu_cpwait();
#endif
#endif

	/*
	 * now we allocate the "special" VAs which are used for tmp mappings
	 * by the pmap (and other modules).  we allocate the VAs by advancing
	 * virtual_avail (note that there are no pages mapped at these VAs).
	 *
	 * Managed KVM space start from wherever initarm() tells us.
	 */
	virtual_avail = vstart;
	virtual_end = vend;

	VPRINTF("specials ");
#ifdef PMAP_CACHE_VIPT
	/*
	 * If we have a VIPT cache, we need one page/pte per possible alias
	 * page so we won't violate cache aliasing rules.
	 */
	virtual_avail = (virtual_avail + arm_cache_prefer_mask) & ~arm_cache_prefer_mask;
	nptes = (arm_cache_prefer_mask >> L2_S_SHIFT) + 1;
	nptes = roundup(nptes, PAGE_SIZE / L2_S_SIZE);
	if (arm_pcache.icache_type != CACHE_TYPE_PIPT
	    && arm_pcache.icache_way_size > nptes * L2_S_SIZE) {
		nptes = arm_pcache.icache_way_size >> L2_S_SHIFT;
		nptes = roundup(nptes, PAGE_SIZE / L2_S_SIZE);
	}
#else
	nptes = PAGE_SIZE / L2_S_SIZE;
#endif
#ifdef MULTIPROCESSOR
	cnptes = nptes;
	nptes *= arm_cpu_max;
#endif
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

	/*
	 * Allocate a range of kernel virtual address space to be used
	 * for L2 descriptor tables and metadata allocation in
	 * pmap_growkernel().
	 */
	size = ((virtual_end - pmap_curmaxkvaddr) + L1_S_OFFSET) / L1_S_SIZE;
	pmap_alloc_specials(&virtual_avail,
	    round_page(size * L2_TABLE_SIZE_REAL) / PAGE_SIZE,
	    &pmap_kernel_l2ptp_kva, NULL);

	size = (size + (L2_BUCKET_SIZE - 1)) / L2_BUCKET_SIZE;
	pmap_alloc_specials(&virtual_avail,
	    round_page(size * sizeof(struct l2_dtable)) / PAGE_SIZE,
	    &pmap_kernel_l2dtable_kva, NULL);


#ifndef ARM_HAS_VBAR
	/* Set up vector page L1 details, if necessary */
	if (vector_page < KERNEL_BASE) {
		pm->pm_pl1vec = pmap_l1_kva(pm) + l1pte_index(vector_page);
		l2b = pmap_get_l2_bucket(pm, vector_page);
		KDASSERT(l2b != NULL);
		pm->pm_l1vec = l2b->l2b_pa | L1_C_PROTO |
		    L1_C_DOM(pmap_domain(pm));
	} else
		pm->pm_pl1vec = NULL;
#endif

	VPRINTF("pools ");
	/*
	 * Initialize the pmap cache
	 */
	pool_cache_bootstrap(&pmap_cache, sizeof(struct pmap), 0, 0, 0,
	    "pmappl", NULL, IPL_NONE, pmap_pmap_ctor, NULL, NULL);

	/*
	 * Initialize the pv pool.
	 */
	pool_init(&pmap_pv_pool, sizeof(struct pv_entry), 0, 0, 0, "pvepl",
	    &pmap_bootstrap_pv_allocator, IPL_NONE);

	/*
	 * Initialize the L2 dtable pool and cache.
	 */
	pool_cache_bootstrap(&pmap_l2dtable_cache, sizeof(struct l2_dtable), 0,
	    0, 0, "l2dtblpl", NULL, IPL_NONE, pmap_l2dtable_ctor, NULL, NULL);

	/*
	 * Initialise the L2 descriptor table pool and cache
	 */
	pool_cache_bootstrap(&pmap_l2ptp_cache, L2_TABLE_SIZE_REAL, 0,
	    L2_TABLE_SIZE_REAL, 0, "l2ptppl", NULL, IPL_NONE,
	    pmap_l2ptp_ctor, NULL, NULL);

	mutex_init(&memlock, MUTEX_DEFAULT, IPL_NONE);

	cpu_dcache_wbinv_all();
}

static bool
pmap_set_pt_cache_mode(pd_entry_t *kl1, vaddr_t va, size_t nptes)
{
	return false;
}

static void
pmap_alloc_specials(vaddr_t *availp, int pages, vaddr_t *vap, pt_entry_t **ptep)
{
	vaddr_t va = *availp;
	struct l2_bucket *l2b;

	if (ptep) {
		l2b = pmap_get_l2_bucket(pmap_kernel(), va);
		if (l2b == NULL)
			panic("pmap_alloc_specials: no l2b for 0x%lx", va);

		if (ptep)
			*ptep = &l2b->l2b_kva[l2pte_index(va)];
	}

	*vap = va;
	*availp = va + (PAGE_SIZE * pages);
}

void
pmap_init(void)
{

	/*
	 * Set the available memory vars - These do not map to real memory
	 * addresses and cannot as the physical memory is fragmented.
	 * They are used by ps for %mem calculations.
	 * One could argue whether this should be the entire memory or just
	 * the memory that is useable in a user process.
	 */
	avail_start = ptoa(VM_PHYSMEM_PTR(0)->start);
	avail_end = ptoa(VM_PHYSMEM_PTR(vm_nphysseg - 1)->end);

	/*
	 * Now we need to free enough pv_entry structures to allow us to get
	 * the kmem_map/kmem_object allocated and inited (done after this
	 * function is finished).  to do this we allocate one bootstrap page out
	 * of kernel_map and use it to provide an initial pool of pv_entry
	 * structures.   we never free this page.
	 */
	pool_setlowat(&pmap_pv_pool, (PAGE_SIZE / sizeof(struct pv_entry)) * 2);

	pmap_tlb_info_evcnt_attach(&pmap_tlb0_info);

	pmap_initialized = true;
}

static vaddr_t last_bootstrap_page = 0;
static void *free_bootstrap_pages = NULL;

static void *
pmap_bootstrap_pv_page_alloc(struct pool *pp, int flags)
{
	extern void *pool_page_alloc(struct pool *, int);
	vaddr_t new_page;
	void *rv;

	if (pmap_initialized)
		return (pool_page_alloc(pp, flags));

	if (free_bootstrap_pages) {
		rv = free_bootstrap_pages;
		free_bootstrap_pages = *((void **)rv);
		return (rv);
	}

	KASSERT(kernel_map != NULL);
	new_page = uvm_km_alloc(kernel_map, PAGE_SIZE, 0,
	    UVM_KMF_WIRED | ((flags & PR_WAITOK) ? 0 : UVM_KMF_NOWAIT));

	KASSERT(new_page > last_bootstrap_page);
	last_bootstrap_page = new_page;
	return ((void *)new_page);
}

static void
pmap_bootstrap_pv_page_free(struct pool *pp, void *v)
{
	extern void pool_page_free(struct pool *, void *);

	if ((vaddr_t)v <= last_bootstrap_page) {
		*((void **)v) = free_bootstrap_pages;
		free_bootstrap_pages = v;
		return;
	}

	if (pmap_initialized) {
		pool_page_free(pp, v);
		return;
	}
}

/*
 * pmap_postinit()
 *
 * This routine is called after the vm and kmem subsystems have been
 * initialised. This allows the pmap code to perform any initialisation
 * that can only be done one the memory allocation is in place.
 */
void
pmap_postinit(void)
{

	pool_cache_setlowat(&pmap_l2ptp_cache, (PAGE_SIZE / L2_TABLE_SIZE_REAL) * 4);
	pool_cache_setlowat(&pmap_l2dtable_cache,
	    (PAGE_SIZE / sizeof(struct l2_dtable)) * 2);

}

/*
 * Note that the following routines are used by board-specific initialisation
 * code to configure the initial kernel page tables.
 */

/*
 * This list exists for the benefit of pmap_map_chunk().  It keeps track
 * of the kernel L2 tables during bootstrap, so that pmap_map_chunk() can
 * find them as necessary.
 *
 * Note that the data on this list MUST remain valid after initarm() returns,
 * as pmap_bootstrap() uses it to contruct L2 table metadata.
 */
SLIST_HEAD(, pv_addr) kernel_pt_list = SLIST_HEAD_INITIALIZER(kernel_pt_list);

static vaddr_t
kernel_pt_lookup(paddr_t pa)
{
	pv_addr_t *pv;

	SLIST_FOREACH(pv, &kernel_pt_list, pv_list) {
		if (pv->pv_pa == (pa & ~PGOFSET))
			return (pv->pv_va | (pa & PGOFSET));
	}
	return (0);
}

/*
 * pmap_map_section:
 *
 *	Create a single section mapping.
 */
void
pmap_map_section(vaddr_t l1pt, vaddr_t va, paddr_t pa, int prot, int cache)
{
	pd_entry_t * const pdep = (pd_entry_t *) l1pt;
	const size_t l1slot = l1pte_index(va);
	pd_entry_t fl;

	KASSERT(((va | pa) & L1_S_OFFSET) == 0);

	switch (cache) {
	case PTE_NOCACHE:
	default:
		fl = 0;
		break;

	case PTE_CACHE:
		fl = pte_l1_s_cache_mode;
		break;

	case PTE_PAGETABLE:
		fl = pte_l1_s_cache_mode_pt;
		break;
	}

	const pd_entry_t npde = L1_S_PROTO | pa |
	    L1_S_PROT(PTE_KERNEL, prot) | fl | L1_S_DOM(PMAP_DOMAIN_KERNEL);
	l1pte_setone(pdep + l1slot, npde);
	PDE_SYNC(pdep + l1slot);
}

/*
 * pmap_map_entry:
 *
 *	Create a single page mapping.
 */
void
pmap_map_entry(vaddr_t l1pt, vaddr_t va, paddr_t pa, int prot, int cache)
{
	pd_entry_t * const pdep = (pd_entry_t *) l1pt;
	const size_t l1slot = l1pte_index(va);
	pt_entry_t npte;
	pt_entry_t *ptep;

	KASSERT(((va | pa) & PGOFSET) == 0);

	switch (cache) {
	case PTE_NOCACHE:
	default:
		npte = 0;
		break;

	case PTE_CACHE:
		npte = pte_l2_s_cache_mode;
		break;

	case PTE_PAGETABLE:
		npte = pte_l2_s_cache_mode_pt;
		break;
	}

	if ((pdep[l1slot] & L1_TYPE_MASK) != L1_TYPE_C)
		panic("pmap_map_entry: no L2 table for VA 0x%08lx", va);

	ptep = (pt_entry_t *) kernel_pt_lookup(l1pte_pa(pdep[l1slot]));
	if (ptep == NULL)
		panic("pmap_map_entry: can't find L2 table for VA 0x%08lx", va);

	npte |= L2_S_PROTO | pa | L2_S_PROT(PTE_KERNEL, prot);
	if (prot & VM_PROT_EXECUTE) {
		npte &= ~L2_XS_XN;
	}
	ptep += l2pte_index(va);
	l2pte_set(ptep, npte, 0);
	PTE_SYNC(ptep);
}

/*
 * pmap_link_l2pt:
 *
 *	Link the L2 page table specified by "l2pv" into the L1
 *	page table at the slot for "va".
 */
void
pmap_link_l2pt(vaddr_t l1pt, vaddr_t va, pv_addr_t *l2pv)
{
	pd_entry_t * const pdep = (pd_entry_t *) l1pt + l1pte_index(va);

	KASSERT((va & ((L1_S_SIZE * (PAGE_SIZE / L2_T_SIZE)) - 1)) == 0);
	KASSERT((l2pv->pv_pa & PGOFSET) == 0);

	const pd_entry_t npde = L1_S_DOM(PMAP_DOMAIN_KERNEL) | L1_C_PROTO
	    | l2pv->pv_pa;

	l1pte_set(pdep, npde);
	PDE_SYNC_RANGE(pdep, PAGE_SIZE / L2_T_SIZE);

	SLIST_INSERT_HEAD(&kernel_pt_list, l2pv, pv_list);
}

/*
 * pmap_map_chunk:
 *
 *	Map a chunk of memory using the most efficient mappings
 *	possible (section, large page, small page) into the
 *	provided L1 and L2 tables at the specified virtual address.
 */
vsize_t
pmap_map_chunk(vaddr_t l1pt, vaddr_t va, paddr_t pa, vsize_t size,
    int prot, int cache)
{
	pd_entry_t * const pdep = (pd_entry_t *) l1pt;
	pt_entry_t f1, f2s, f2l;
	vsize_t resid;

	resid = (size + (PAGE_SIZE - 1)) & ~(PAGE_SIZE - 1);

	if (l1pt == 0)
		panic("pmap_map_chunk: no L1 table provided");

	VPRINTF("pmap_map_chunk: pa=0x%lx va=0x%lx size=0x%lx resid=0x%lx "
	    "prot=0x%x cache=%d\n", pa, va, size, resid, prot, cache);

	switch (cache) {
	case PTE_NOCACHE:
	default:
		f1 = 0;
		f2l = 0;
		f2s = 0;
		break;

	case PTE_CACHE:
		f1 = pte_l1_s_cache_mode;
		f2l = pte_l2_l_cache_mode;
		f2s = pte_l2_s_cache_mode;
		break;

	case PTE_PAGETABLE:
		f1 = pte_l1_s_cache_mode_pt;
		f2l = pte_l2_l_cache_mode_pt;
		f2s = pte_l2_s_cache_mode_pt;
		break;
	}

	size = resid;

	while (resid > 0) {
		const size_t l1slot = l1pte_index(va);
#if (ARM_MMU_V6 + ARM_MMU_V7) > 0
		/* See if we can use a supersection mapping. */
		if (L1_SS_PROTO && L1_SS_MAPPABLE_P(va, pa, resid)) {
			/* Supersection are always domain 0 */
			const pd_entry_t npde = L1_SS_PROTO | pa
			    | ((prot & VM_PROT_EXECUTE) ? 0 : L1_S_V6_XN)
			    | (va & 0x80000000 ? 0 : L1_S_V6_nG)
			    | L1_S_PROT(PTE_KERNEL, prot) | f1;
			VPRINTF("sS");
			l1pte_set(&pdep[l1slot], npde);
			PDE_SYNC_RANGE(&pdep[l1slot], L1_SS_SIZE / L1_S_SIZE);
			va += L1_SS_SIZE;
			pa += L1_SS_SIZE;
			resid -= L1_SS_SIZE;
			continue;
		}
#endif
		/* See if we can use a section mapping. */
		if (L1_S_MAPPABLE_P(va, pa, resid)) {
			const pd_entry_t npde = L1_S_PROTO | pa
			    | ((prot & VM_PROT_EXECUTE) ? 0 : L1_S_V6_XN)
			    | (va & 0x80000000 ? 0 : L1_S_V6_nG)
			    | L1_S_PROT(PTE_KERNEL, prot) | f1
			    | L1_S_DOM(PMAP_DOMAIN_KERNEL);
			VPRINTF("S");
			l1pte_set(&pdep[l1slot], npde);
			PDE_SYNC(&pdep[l1slot]);
			va += L1_S_SIZE;
			pa += L1_S_SIZE;
			resid -= L1_S_SIZE;
			continue;
		}

		/*
		 * Ok, we're going to use an L2 table.  Make sure
		 * one is actually in the corresponding L1 slot
		 * for the current VA.
		 */
		if ((pdep[l1slot] & L1_TYPE_MASK) != L1_TYPE_C)
			panic("%s: no L2 table for VA %#lx", __func__, va);

		pt_entry_t *ptep = (pt_entry_t *) kernel_pt_lookup(l1pte_pa(pdep[l1slot]));
		if (ptep == NULL)
			panic("%s: can't find L2 table for VA %#lx", __func__,
			    va);

		ptep += l2pte_index(va);

		/* See if we can use a L2 large page mapping. */
		if (L2_L_MAPPABLE_P(va, pa, resid)) {
			const pt_entry_t npte = L2_L_PROTO | pa
			    | ((prot & VM_PROT_EXECUTE) ? 0 : L2_XS_L_XN)
			    | (va & 0x80000000 ? 0 : L2_XS_nG)
			    | L2_L_PROT(PTE_KERNEL, prot) | f2l;
			VPRINTF("L");
			l2pte_set(ptep, npte, 0);
			PTE_SYNC_RANGE(ptep, L2_L_SIZE / L2_S_SIZE);
			va += L2_L_SIZE;
			pa += L2_L_SIZE;
			resid -= L2_L_SIZE;
			continue;
		}

		VPRINTF("P");
		/* Use a small page mapping. */
		pt_entry_t npte = L2_S_PROTO | pa
		    | ((prot & VM_PROT_EXECUTE) ? 0 : L2_XS_XN)
		    | (va & 0x80000000 ? 0 : L2_XS_nG)
		    | L2_S_PROT(PTE_KERNEL, prot) | f2s;
		npte &= ((prot & VM_PROT_EXECUTE) ? ~L2_XS_XN : ~0);
		l2pte_set(ptep, npte, 0);
		PTE_SYNC(ptep);
		va += PAGE_SIZE;
		pa += PAGE_SIZE;
		resid -= PAGE_SIZE;
	}
	VPRINTF("\n");
	return (size);
}

/********************** Static device map routines ***************************/

static const struct pmap_devmap *pmap_devmap_table;

/*
 * Register the devmap table.  This is provided in case early console
 * initialization needs to register mappings created by bootstrap code
 * before pmap_devmap_bootstrap() is called.
 */
void
pmap_devmap_register(const struct pmap_devmap *table)
{

	pmap_devmap_table = table;
}

/*
 * Map all of the static regions in the devmap table, and remember
 * the devmap table so other parts of the kernel can look up entries
 * later.
 */
void
pmap_devmap_bootstrap(vaddr_t l1pt, const struct pmap_devmap *table)
{
	int i;

	pmap_devmap_table = table;

	for (i = 0; pmap_devmap_table[i].pd_size != 0; i++) {
		VPRINTF("devmap: %08lx -> %08lx @ %08lx\n",
		    pmap_devmap_table[i].pd_pa,
		    pmap_devmap_table[i].pd_pa +
			pmap_devmap_table[i].pd_size - 1,
		    pmap_devmap_table[i].pd_va);
		pmap_map_chunk(l1pt, pmap_devmap_table[i].pd_va,
		    pmap_devmap_table[i].pd_pa,
		    pmap_devmap_table[i].pd_size,
		    pmap_devmap_table[i].pd_prot,
		    pmap_devmap_table[i].pd_cache);
	}
}

const struct pmap_devmap *
pmap_devmap_find_pa(paddr_t pa, psize_t size)
{
	uint64_t endpa;
	int i;

	if (pmap_devmap_table == NULL)
		return (NULL);

	endpa = (uint64_t)pa + (uint64_t)(size - 1);

	for (i = 0; pmap_devmap_table[i].pd_size != 0; i++) {
		if (pa >= pmap_devmap_table[i].pd_pa &&
		    endpa <= (uint64_t)pmap_devmap_table[i].pd_pa +
			     (uint64_t)(pmap_devmap_table[i].pd_size - 1))
			return (&pmap_devmap_table[i]);
	}

	return (NULL);
}

const struct pmap_devmap *
pmap_devmap_find_va(vaddr_t va, vsize_t size)
{
	int i;

	if (pmap_devmap_table == NULL)
		return (NULL);

	for (i = 0; pmap_devmap_table[i].pd_size != 0; i++) {
		if (va >= pmap_devmap_table[i].pd_va &&
		    va + size - 1 <= pmap_devmap_table[i].pd_va +
				     pmap_devmap_table[i].pd_size - 1)
			return (&pmap_devmap_table[i]);
	}

	return (NULL);
}

/********************** PTE initialization routines **************************/

/*
 * These routines are called when the CPU type is identified to set up
 * the PTE prototypes, cache modes, etc.
 *
 * The variables are always here, just in case modules need to reference
 * them (though, they shouldn't).
 */

pt_entry_t	pte_l1_s_cache_mode;
pt_entry_t	pte_l1_s_wc_mode;
pt_entry_t	pte_l1_s_cache_mode_pt;
pt_entry_t	pte_l1_s_cache_mask;

pt_entry_t	pte_l2_l_cache_mode;
pt_entry_t	pte_l2_l_wc_mode;
pt_entry_t	pte_l2_l_cache_mode_pt;
pt_entry_t	pte_l2_l_cache_mask;

pt_entry_t	pte_l2_s_cache_mode;
pt_entry_t	pte_l2_s_wc_mode;
pt_entry_t	pte_l2_s_cache_mode_pt;
pt_entry_t	pte_l2_s_cache_mask;

pt_entry_t	pte_l1_s_prot_u;
pt_entry_t	pte_l1_s_prot_w;
pt_entry_t	pte_l1_s_prot_ro;
pt_entry_t	pte_l1_s_prot_mask;

pt_entry_t	pte_l2_s_prot_u;
pt_entry_t	pte_l2_s_prot_w;
pt_entry_t	pte_l2_s_prot_ro;
pt_entry_t	pte_l2_s_prot_mask;

pt_entry_t	pte_l2_l_prot_u;
pt_entry_t	pte_l2_l_prot_w;
pt_entry_t	pte_l2_l_prot_ro;
pt_entry_t	pte_l2_l_prot_mask;

pt_entry_t	pte_l1_ss_proto;
pt_entry_t	pte_l1_s_proto;
pt_entry_t	pte_l1_c_proto;
pt_entry_t	pte_l2_s_proto;

void		(*pmap_copy_page_func)(paddr_t, paddr_t);
void		(*pmap_zero_page_func)(paddr_t);

#if (ARM_MMU_GENERIC + ARM_MMU_SA1 + ARM_MMU_V6 + ARM_MMU_V7) != 0
void
pmap_pte_init_generic(void)
{

	pte_l1_s_cache_mode = L1_S_B|L1_S_C;
	pte_l1_s_wc_mode = L1_S_B;
	pte_l1_s_cache_mask = L1_S_CACHE_MASK_generic;

	pte_l2_l_cache_mode = L2_B|L2_C;
	pte_l2_l_wc_mode = L2_B;
	pte_l2_l_cache_mask = L2_L_CACHE_MASK_generic;

	pte_l2_s_cache_mode = L2_B|L2_C;
	pte_l2_s_wc_mode = L2_B;
	pte_l2_s_cache_mask = L2_S_CACHE_MASK_generic;

	/*
	 * If we have a write-through cache, set B and C.  If
	 * we have a write-back cache, then we assume setting
	 * only C will make those pages write-through (except for those
	 * Cortex CPUs which can read the L1 caches).
	 */
	if (cpufuncs.cf_dcache_wb_range == (void *) cpufunc_nullop
#if ARM_MMU_V7 > 0
	    || CPU_ID_CORTEX_P(curcpu()->ci_arm_cpuid)
#endif
#if ARM_MMU_V6 > 0
	    || CPU_ID_ARM11_P(curcpu()->ci_arm_cpuid) /* arm116 errata 399234 */
#endif
	    || false) {
		pte_l1_s_cache_mode_pt = L1_S_B|L1_S_C;
		pte_l2_l_cache_mode_pt = L2_B|L2_C;
		pte_l2_s_cache_mode_pt = L2_B|L2_C;
	} else {
		pte_l1_s_cache_mode_pt = L1_S_C;	/* write through */
		pte_l2_l_cache_mode_pt = L2_C;		/* write through */
		pte_l2_s_cache_mode_pt = L2_C;		/* write through */
	}

	pte_l1_s_prot_u = L1_S_PROT_U_generic;
	pte_l1_s_prot_w = L1_S_PROT_W_generic;
	pte_l1_s_prot_ro = L1_S_PROT_RO_generic;
	pte_l1_s_prot_mask = L1_S_PROT_MASK_generic;

	pte_l2_s_prot_u = L2_S_PROT_U_generic;
	pte_l2_s_prot_w = L2_S_PROT_W_generic;
	pte_l2_s_prot_ro = L2_S_PROT_RO_generic;
	pte_l2_s_prot_mask = L2_S_PROT_MASK_generic;

	pte_l2_l_prot_u = L2_L_PROT_U_generic;
	pte_l2_l_prot_w = L2_L_PROT_W_generic;
	pte_l2_l_prot_ro = L2_L_PROT_RO_generic;
	pte_l2_l_prot_mask = L2_L_PROT_MASK_generic;

	pte_l1_ss_proto = L1_SS_PROTO_generic;
	pte_l1_s_proto = L1_S_PROTO_generic;
	pte_l1_c_proto = L1_C_PROTO_generic;
	pte_l2_s_proto = L2_S_PROTO_generic;

	pmap_copy_page_func = pmap_copy_page_generic;
	pmap_zero_page_func = pmap_zero_page_generic;
}

#if defined(CPU_ARM8)
void
pmap_pte_init_arm8(void)
{

	/*
	 * ARM8 is compatible with generic, but we need to use
	 * the page tables uncached.
	 */
	pmap_pte_init_generic();

	pte_l1_s_cache_mode_pt = 0;
	pte_l2_l_cache_mode_pt = 0;
	pte_l2_s_cache_mode_pt = 0;
}
#endif /* CPU_ARM8 */

#if defined(CPU_ARM9) && defined(ARM9_CACHE_WRITE_THROUGH)
void
pmap_pte_init_arm9(void)
{

	/*
	 * ARM9 is compatible with generic, but we want to use
	 * write-through caching for now.
	 */
	pmap_pte_init_generic();

	pte_l1_s_cache_mode = L1_S_C;
	pte_l2_l_cache_mode = L2_C;
	pte_l2_s_cache_mode = L2_C;

	pte_l1_s_wc_mode = L1_S_B;
	pte_l2_l_wc_mode = L2_B;
	pte_l2_s_wc_mode = L2_B;

	pte_l1_s_cache_mode_pt = L1_S_C;
	pte_l2_l_cache_mode_pt = L2_C;
	pte_l2_s_cache_mode_pt = L2_C;
}
#endif /* CPU_ARM9 && ARM9_CACHE_WRITE_THROUGH */
#endif /* (ARM_MMU_GENERIC + ARM_MMU_SA1 + ARM_MMU_V6) != 0 */

#if defined(CPU_ARM10)
void
pmap_pte_init_arm10(void)
{

	/*
	 * ARM10 is compatible with generic, but we want to use
	 * write-through caching for now.
	 */
	pmap_pte_init_generic();

	pte_l1_s_cache_mode = L1_S_B | L1_S_C;
	pte_l2_l_cache_mode = L2_B | L2_C;
	pte_l2_s_cache_mode = L2_B | L2_C;

	pte_l1_s_cache_mode = L1_S_B;
	pte_l2_l_cache_mode = L2_B;
	pte_l2_s_cache_mode = L2_B;

	pte_l1_s_cache_mode_pt = L1_S_C;
	pte_l2_l_cache_mode_pt = L2_C;
	pte_l2_s_cache_mode_pt = L2_C;

}
#endif /* CPU_ARM10 */

#if defined(CPU_ARM11) && defined(ARM11_CACHE_WRITE_THROUGH)
void
pmap_pte_init_arm11(void)
{

	/*
	 * ARM11 is compatible with generic, but we want to use
	 * write-through caching for now.
	 */
	pmap_pte_init_generic();

	pte_l1_s_cache_mode = L1_S_C;
	pte_l2_l_cache_mode = L2_C;
	pte_l2_s_cache_mode = L2_C;

	pte_l1_s_wc_mode = L1_S_B;
	pte_l2_l_wc_mode = L2_B;
	pte_l2_s_wc_mode = L2_B;

	pte_l1_s_cache_mode_pt = L1_S_C;
	pte_l2_l_cache_mode_pt = L2_C;
	pte_l2_s_cache_mode_pt = L2_C;
}
#endif /* CPU_ARM11 && ARM11_CACHE_WRITE_THROUGH */

#if ARM_MMU_SA1 == 1
void
pmap_pte_init_sa1(void)
{

	/*
	 * The StrongARM SA-1 cache does not have a write-through
	 * mode.  So, do the generic initialization, then reset
	 * the page table cache mode to B=1,C=1, and note that
	 * the PTEs need to be sync'd.
	 */
	pmap_pte_init_generic();

	pte_l1_s_cache_mode_pt = L1_S_B|L1_S_C;
	pte_l2_l_cache_mode_pt = L2_B|L2_C;
	pte_l2_s_cache_mode_pt = L2_B|L2_C;

	pmap_needs_pte_sync = 1;
}
#endif /* ARM_MMU_SA1 == 1*/

#if ARM_MMU_XSCALE == 1
#if (ARM_NMMUS > 1)
static u_int xscale_use_minidata;
#endif

void
pmap_pte_init_xscale(void)
{
	uint32_t auxctl;
	int write_through = 0;

	pte_l1_s_cache_mode = L1_S_B|L1_S_C;
	pte_l1_s_wc_mode = L1_S_B;
	pte_l1_s_cache_mask = L1_S_CACHE_MASK_xscale;

	pte_l2_l_cache_mode = L2_B|L2_C;
	pte_l2_l_wc_mode = L2_B;
	pte_l2_l_cache_mask = L2_L_CACHE_MASK_xscale;

	pte_l2_s_cache_mode = L2_B|L2_C;
	pte_l2_s_wc_mode = L2_B;
	pte_l2_s_cache_mask = L2_S_CACHE_MASK_xscale;

	pte_l1_s_cache_mode_pt = L1_S_C;
	pte_l2_l_cache_mode_pt = L2_C;
	pte_l2_s_cache_mode_pt = L2_C;

#ifdef XSCALE_CACHE_READ_WRITE_ALLOCATE
	/*
	 * The XScale core has an enhanced mode where writes that
	 * miss the cache cause a cache line to be allocated.  This
	 * is significantly faster than the traditional, write-through
	 * behavior of this case.
	 */
	pte_l1_s_cache_mode |= L1_S_XS_TEX(TEX_XSCALE_X);
	pte_l2_l_cache_mode |= L2_XS_L_TEX(TEX_XSCALE_X);
	pte_l2_s_cache_mode |= L2_XS_T_TEX(TEX_XSCALE_X);
#endif /* XSCALE_CACHE_READ_WRITE_ALLOCATE */

#ifdef XSCALE_CACHE_WRITE_THROUGH
	/*
	 * Some versions of the XScale core have various bugs in
	 * their cache units, the work-around for which is to run
	 * the cache in write-through mode.  Unfortunately, this
	 * has a major (negative) impact on performance.  So, we
	 * go ahead and run fast-and-loose, in the hopes that we
	 * don't line up the planets in a way that will trip the
	 * bugs.
	 *
	 * However, we give you the option to be slow-but-correct.
	 */
	write_through = 1;
#elif defined(XSCALE_CACHE_WRITE_BACK)
	/* force write back cache mode */
	write_through = 0;
#elif defined(CPU_XSCALE_PXA250) || defined(CPU_XSCALE_PXA270)
	/*
	 * Intel PXA2[15]0 processors are known to have a bug in
	 * write-back cache on revision 4 and earlier (stepping
	 * A[01] and B[012]).  Fixed for C0 and later.
	 */
	{
		uint32_t id, type;

		id = cpufunc_id();
		type = id & ~(CPU_ID_XSCALE_COREREV_MASK|CPU_ID_REVISION_MASK);

		if (type == CPU_ID_PXA250 || type == CPU_ID_PXA210) {
			if ((id & CPU_ID_REVISION_MASK) < 5) {
				/* write through for stepping A0-1 and B0-2 */
				write_through = 1;
			}
		}
	}
#endif /* XSCALE_CACHE_WRITE_THROUGH */

	if (write_through) {
		pte_l1_s_cache_mode = L1_S_C;
		pte_l2_l_cache_mode = L2_C;
		pte_l2_s_cache_mode = L2_C;
	}

#if (ARM_NMMUS > 1)
	xscale_use_minidata = 1;
#endif

	pte_l1_s_prot_u = L1_S_PROT_U_xscale;
	pte_l1_s_prot_w = L1_S_PROT_W_xscale;
	pte_l1_s_prot_ro = L1_S_PROT_RO_xscale;
	pte_l1_s_prot_mask = L1_S_PROT_MASK_xscale;

	pte_l2_s_prot_u = L2_S_PROT_U_xscale;
	pte_l2_s_prot_w = L2_S_PROT_W_xscale;
	pte_l2_s_prot_ro = L2_S_PROT_RO_xscale;
	pte_l2_s_prot_mask = L2_S_PROT_MASK_xscale;

	pte_l2_l_prot_u = L2_L_PROT_U_xscale;
	pte_l2_l_prot_w = L2_L_PROT_W_xscale;
	pte_l2_l_prot_ro = L2_L_PROT_RO_xscale;
	pte_l2_l_prot_mask = L2_L_PROT_MASK_xscale;

	pte_l1_ss_proto = L1_SS_PROTO_xscale;
	pte_l1_s_proto = L1_S_PROTO_xscale;
	pte_l1_c_proto = L1_C_PROTO_xscale;
	pte_l2_s_proto = L2_S_PROTO_xscale;

	pmap_copy_page_func = pmap_copy_page_xscale;
	pmap_zero_page_func = pmap_zero_page_xscale;

	/*
	 * Disable ECC protection of page table access, for now.
	 */
	auxctl = armreg_auxctl_read();
	auxctl &= ~XSCALE_AUXCTL_P;
	armreg_auxctl_write(auxctl);
}

/*
 * xscale_setup_minidata:
 *
 *	Set up the mini-data cache clean area.  We require the
 *	caller to allocate the right amount of physically and
 *	virtually contiguous space.
 */
void
xscale_setup_minidata(vaddr_t l1pt, vaddr_t va, paddr_t pa)
{
	extern vaddr_t xscale_minidata_clean_addr;
	extern vsize_t xscale_minidata_clean_size; /* already initialized */
	pd_entry_t *pde = (pd_entry_t *) l1pt;
	vsize_t size;
	uint32_t auxctl;

	xscale_minidata_clean_addr = va;

	/* Round it to page size. */
	size = (xscale_minidata_clean_size + L2_S_OFFSET) & L2_S_FRAME;

	for (; size != 0;
	     va += L2_S_SIZE, pa += L2_S_SIZE, size -= L2_S_SIZE) {
		const size_t l1slot = l1pte_index(va);
		pt_entry_t *ptep = (pt_entry_t *) kernel_pt_lookup(l1pte_pa(pde[l1slot]));
		if (ptep == NULL)
			panic("xscale_setup_minidata: can't find L2 table for "
			    "VA 0x%08lx", va);

		ptep += l2pte_index(va);
		pt_entry_t opte = *ptep;
		l2pte_set(ptep,
		    L2_S_PROTO | pa | L2_S_PROT(PTE_KERNEL, VM_PROT_READ)
		    | L2_C | L2_XS_T_TEX(TEX_XSCALE_X), opte);
	}

	/*
	 * Configure the mini-data cache for write-back with
	 * read/write-allocate.
	 *
	 * NOTE: In order to reconfigure the mini-data cache, we must
	 * make sure it contains no valid data!  In order to do that,
	 * we must issue a global data cache invalidate command!
	 *
	 * WE ASSUME WE ARE RUNNING UN-CACHED WHEN THIS ROUTINE IS CALLED!
	 * THIS IS VERY IMPORTANT!
	 */

	/* Invalidate data and mini-data. */
	__asm volatile("mcr p15, 0, %0, c7, c6, 0" : : "r" (0));
	auxctl = armreg_auxctl_read();
	auxctl = (auxctl & ~XSCALE_AUXCTL_MD_MASK) | XSCALE_AUXCTL_MD_WB_RWA;
	armreg_auxctl_write(auxctl);
}

/*
 * Change the PTEs for the specified kernel mappings such that they
 * will use the mini data cache instead of the main data cache.
 */
void
pmap_uarea(vaddr_t va)
{
	vaddr_t next_bucket, eva;

#if (ARM_NMMUS > 1)
	if (xscale_use_minidata == 0)
		return;
#endif

	eva = va + USPACE;

	while (va < eva) {
		next_bucket = L2_NEXT_BUCKET_VA(va);
		if (next_bucket > eva)
			next_bucket = eva;

		struct l2_bucket *l2b = pmap_get_l2_bucket(pmap_kernel(), va);
		KDASSERT(l2b != NULL);

		pt_entry_t * const sptep = &l2b->l2b_kva[l2pte_index(va)];
		pt_entry_t *ptep = sptep;

		while (va < next_bucket) {
			const pt_entry_t opte = *ptep;
			if (!l2pte_minidata_p(opte)) {
				cpu_dcache_wbinv_range(va, PAGE_SIZE);
				cpu_tlb_flushD_SE(va);
				l2pte_set(ptep, opte & ~L2_B, opte);
			}
			ptep += PAGE_SIZE / L2_S_SIZE;
			va += PAGE_SIZE;
		}
		PTE_SYNC_RANGE(sptep, (u_int)(ptep - sptep));
	}
	cpu_cpwait();
}
#endif /* ARM_MMU_XSCALE == 1 */


#if defined(CPU_ARM11MPCORE)

void
pmap_pte_init_arm11mpcore(void)
{

	/* cache mode is controlled by 5 bits (B, C, TEX) */
	pte_l1_s_cache_mask = L1_S_CACHE_MASK_armv6;
	pte_l2_l_cache_mask = L2_L_CACHE_MASK_armv6;
#if defined(ARM11MPCORE_COMPAT_MMU) || defined(ARMV6_EXTENDED_SMALL_PAGE)
	/* use extended small page (without APn, with TEX) */
	pte_l2_s_cache_mask = L2_XS_CACHE_MASK_armv6;
#else
	pte_l2_s_cache_mask = L2_S_CACHE_MASK_armv6c;
#endif

	/* write-back, write-allocate */
	pte_l1_s_cache_mode = L1_S_C | L1_S_B | L1_S_V6_TEX(0x01);
	pte_l2_l_cache_mode = L2_C | L2_B | L2_V6_L_TEX(0x01);
#if defined(ARM11MPCORE_COMPAT_MMU) || defined(ARMV6_EXTENDED_SMALL_PAGE)
	pte_l2_s_cache_mode = L2_C | L2_B | L2_V6_XS_TEX(0x01);
#else
	/* no TEX. read-allocate */
	pte_l2_s_cache_mode = L2_C | L2_B;
#endif
	/*
	 * write-back, write-allocate for page tables.
	 */
	pte_l1_s_cache_mode_pt = L1_S_C | L1_S_B | L1_S_V6_TEX(0x01);
	pte_l2_l_cache_mode_pt = L2_C | L2_B | L2_V6_L_TEX(0x01);
#if defined(ARM11MPCORE_COMPAT_MMU) || defined(ARMV6_EXTENDED_SMALL_PAGE)
	pte_l2_s_cache_mode_pt = L2_C | L2_B | L2_V6_XS_TEX(0x01);
#else
	pte_l2_s_cache_mode_pt = L2_C | L2_B;
#endif

	pte_l1_s_prot_u = L1_S_PROT_U_armv6;
	pte_l1_s_prot_w = L1_S_PROT_W_armv6;
	pte_l1_s_prot_ro = L1_S_PROT_RO_armv6;
	pte_l1_s_prot_mask = L1_S_PROT_MASK_armv6;

#if defined(ARM11MPCORE_COMPAT_MMU) || defined(ARMV6_EXTENDED_SMALL_PAGE)
	pte_l2_s_prot_u = L2_S_PROT_U_armv6n;
	pte_l2_s_prot_w = L2_S_PROT_W_armv6n;
	pte_l2_s_prot_ro = L2_S_PROT_RO_armv6n;
	pte_l2_s_prot_mask = L2_S_PROT_MASK_armv6n;

#else
	/* with AP[0..3] */
	pte_l2_s_prot_u = L2_S_PROT_U_generic;
	pte_l2_s_prot_w = L2_S_PROT_W_generic;
	pte_l2_s_prot_ro = L2_S_PROT_RO_generic;
	pte_l2_s_prot_mask = L2_S_PROT_MASK_generic;
#endif

#ifdef	ARM11MPCORE_COMPAT_MMU
	/* with AP[0..3] */
	pte_l2_l_prot_u = L2_L_PROT_U_generic;
	pte_l2_l_prot_w = L2_L_PROT_W_generic;
	pte_l2_l_prot_ro = L2_L_PROT_RO_generic;
	pte_l2_l_prot_mask = L2_L_PROT_MASK_generic;

	pte_l1_ss_proto = L1_SS_PROTO_armv6;
	pte_l1_s_proto = L1_S_PROTO_armv6;
	pte_l1_c_proto = L1_C_PROTO_armv6;
	pte_l2_s_proto = L2_S_PROTO_armv6c;
#else
	pte_l2_l_prot_u = L2_L_PROT_U_armv6n;
	pte_l2_l_prot_w = L2_L_PROT_W_armv6n;
	pte_l2_l_prot_ro = L2_L_PROT_RO_armv6n;
	pte_l2_l_prot_mask = L2_L_PROT_MASK_armv6n;

	pte_l1_ss_proto = L1_SS_PROTO_armv6;
	pte_l1_s_proto = L1_S_PROTO_armv6;
	pte_l1_c_proto = L1_C_PROTO_armv6;
	pte_l2_s_proto = L2_S_PROTO_armv6n;
#endif

	pmap_copy_page_func = pmap_copy_page_generic;
	pmap_zero_page_func = pmap_zero_page_generic;
	pmap_needs_pte_sync = 1;
}
#endif	/* CPU_ARM11MPCORE */


#if ARM_MMU_V7 == 1
void
pmap_pte_init_armv7(void)
{
	/*
	 * The ARMv7-A MMU is mostly compatible with generic. If the
	 * AP field is zero, that now means "no access" rather than
	 * read-only. The prototypes are a little different because of
	 * the XN bit.
	 */
	pmap_pte_init_generic();

	pmap_needs_pte_sync = 1;

	pte_l1_s_cache_mask = L1_S_CACHE_MASK_armv7;
	pte_l2_l_cache_mask = L2_L_CACHE_MASK_armv7;
	pte_l2_s_cache_mask = L2_S_CACHE_MASK_armv7;

	/*
	 * If the core support coherent walk then updates to translation tables
	 * do not require a clean to the point of unification to ensure
	 * visibility by subsequent translation table walks.  That means we can
	 * map everything shareable and cached and the right thing will happen.
	 */
        if (__SHIFTOUT(armreg_mmfr3_read(), __BITS(23,20))) {
		pmap_needs_pte_sync = 0;

		/*
		 * write-back, no write-allocate, shareable for normal pages.
		 */
		pte_l1_s_cache_mode |= L1_S_V6_S;
		pte_l2_l_cache_mode |= L2_XS_S;
		pte_l2_s_cache_mode |= L2_XS_S;
	}

	/*
	 * Page tables are just all other memory.  We can use write-back since
	 * pmap_needs_pte_sync is 1 (or the MMU can read out of cache).
	 */
	pte_l1_s_cache_mode_pt = pte_l1_s_cache_mode;
	pte_l2_l_cache_mode_pt = pte_l2_l_cache_mode;
	pte_l2_s_cache_mode_pt = pte_l2_s_cache_mode;

	/*
	 * Check the Memory Model Features to see if this CPU supports
	 * the TLBIASID coproc op.
	 */
	if (__SHIFTOUT(armreg_mmfr2_read(), __BITS(16,19)) >= 2) {
		arm_has_tlbiasid_p = true;
	}

	pte_l1_s_prot_u = L1_S_PROT_U_armv7;
	pte_l1_s_prot_w = L1_S_PROT_W_armv7;
	pte_l1_s_prot_ro = L1_S_PROT_RO_armv7;
	pte_l1_s_prot_mask = L1_S_PROT_MASK_armv7;

	pte_l2_s_prot_u = L2_S_PROT_U_armv7;
	pte_l2_s_prot_w = L2_S_PROT_W_armv7;
	pte_l2_s_prot_ro = L2_S_PROT_RO_armv7;
	pte_l2_s_prot_mask = L2_S_PROT_MASK_armv7;

	pte_l2_l_prot_u = L2_L_PROT_U_armv7;
	pte_l2_l_prot_w = L2_L_PROT_W_armv7;
	pte_l2_l_prot_ro = L2_L_PROT_RO_armv7;
	pte_l2_l_prot_mask = L2_L_PROT_MASK_armv7;

	pte_l1_ss_proto = L1_SS_PROTO_armv7;
	pte_l1_s_proto = L1_S_PROTO_armv7;
	pte_l1_c_proto = L1_C_PROTO_armv7;
	pte_l2_s_proto = L2_S_PROTO_armv7;

}
#endif /* ARM_MMU_V7 */

/*
 * return the PA of the current L1 table, for use when handling a crash dump
 */
uint32_t
pmap_kernel_L1_addr(void)
{
	return pmap_kernel()->pm_l1_pa;
}

#if defined(DDB)
/*
 * A couple of ddb-callable functions for dumping pmaps
 */
void pmap_dump(pmap_t);

static pt_entry_t ncptes[64];
static void pmap_dump_ncpg(pmap_t);

void
pmap_dump(pmap_t pm)
{
	struct l2_dtable *l2;
	struct l2_bucket *l2b;
	pt_entry_t *ptep, pte;
	vaddr_t l2_va, l2b_va, va;
	int i, j, k, occ, rows = 0;

	if (pm == pmap_kernel())
		printf("pmap_kernel (%p): ", pm);
	else
		printf("user pmap (%p): ", pm);

	printf("l1 at %p\n", pmap_l1_kva(pm));

	l2_va = 0;
	for (i = 0; i < L2_SIZE; i++, l2_va += 0x01000000) {
		l2 = pm->pm_l2[i];

		if (l2 == NULL || l2->l2_occupancy == 0)
			continue;

		l2b_va = l2_va;
		for (j = 0; j < L2_BUCKET_SIZE; j++, l2b_va += 0x00100000) {
			l2b = &l2->l2_bucket[j];

			if (l2b->l2b_occupancy == 0 || l2b->l2b_kva == NULL)
				continue;

			ptep = l2b->l2b_kva;

			for (k = 0; k < 256 && ptep[k] == 0; k++)
				;

			k &= ~63;
			occ = l2b->l2b_occupancy;
			va = l2b_va + (k * 4096);
			for (; k < 256; k++, va += 0x1000) {
				char ch = ' ';
				if ((k % 64) == 0) {
					if ((rows % 8) == 0) {
						printf(
"          |0000   |8000   |10000  |18000  |20000  |28000  |30000  |38000\n");
					}
					printf("%08lx: ", va);
				}

				ncptes[k & 63] = 0;
				pte = ptep[k];
				if (pte == 0) {
					ch = '.';
				} else {
					occ--;
					switch (pte & 0x0c) {
					case 0x00:
						ch = 'D'; /* No cache No buff */
						break;
					case 0x04:
						ch = 'B'; /* No cache buff */
						break;
					case 0x08:
						if (pte & 0x40)
							ch = 'm';
						else
						   ch = 'C'; /* Cache No buff */
						break;
					case 0x0c:
						ch = 'F'; /* Cache Buff */
						break;
					}

					if ((pte & L2_S_PROT_U) == L2_S_PROT_U)
						ch += 0x20;

					if ((pte & 0xc) == 0)
						ncptes[k & 63] = pte;
				}

				if ((k % 64) == 63) {
					rows++;
					printf("%c\n", ch);
					pmap_dump_ncpg(pm);
					if (occ == 0)
						break;
				} else
					printf("%c", ch);
			}
		}
	}
}

static void
pmap_dump_ncpg(pmap_t pm)
{
	struct vm_page *pg;
	struct vm_page_md *md;
	struct pv_entry *pv;
	int i;

	for (i = 0; i < 63; i++) {
		if (ncptes[i] == 0)
			continue;

		pg = PHYS_TO_VM_PAGE(l2pte_pa(ncptes[i]));
		if (pg == NULL)
			continue;
		md = VM_PAGE_TO_MD(pg);

		printf(" pa 0x%08lx: krw %d kro %d urw %d uro %d\n",
		    VM_PAGE_TO_PHYS(pg),
		    md->krw_mappings, md->kro_mappings,
		    md->urw_mappings, md->uro_mappings);

		SLIST_FOREACH(pv, &md->pvh_list, pv_link) {
			printf("   %c va 0x%08lx, flags 0x%x\n",
			    (pm == pv->pv_pmap) ? '*' : ' ',
			    pv->pv_va, pv->pv_flags);
		}
	}
}
#endif

#ifdef PMAP_STEAL_MEMORY
void
pmap_boot_pageadd(pv_addr_t *newpv)
{
	pv_addr_t *pv, *npv;

	if ((pv = SLIST_FIRST(&pmap_boot_freeq)) != NULL) {
		if (newpv->pv_pa < pv->pv_va) {
			KASSERT(newpv->pv_pa + newpv->pv_size <= pv->pv_pa);
			if (newpv->pv_pa + newpv->pv_size == pv->pv_pa) {
				newpv->pv_size += pv->pv_size;
				SLIST_REMOVE_HEAD(&pmap_boot_freeq, pv_list);
			}
			pv = NULL;
		} else {
			for (; (npv = SLIST_NEXT(pv, pv_list)) != NULL;
			     pv = npv) {
				KASSERT(pv->pv_pa + pv->pv_size < npv->pv_pa);
				KASSERT(pv->pv_pa < newpv->pv_pa);
				if (newpv->pv_pa > npv->pv_pa)
					continue;
				if (pv->pv_pa + pv->pv_size == newpv->pv_pa) {
					pv->pv_size += newpv->pv_size;
					return;
				}
				if (newpv->pv_pa + newpv->pv_size < npv->pv_pa)
					break;
				newpv->pv_size += npv->pv_size;
				SLIST_INSERT_AFTER(pv, newpv, pv_list);
				SLIST_REMOVE_AFTER(newpv, pv_list);
				return;
			}
		}
	}

	if (pv) {
		SLIST_INSERT_AFTER(pv, newpv, pv_list);
	} else {
		SLIST_INSERT_HEAD(&pmap_boot_freeq, newpv, pv_list);
	}
}

void
pmap_boot_pagealloc(psize_t amount, psize_t mask, psize_t match,
	pv_addr_t *rpv)
{
	pv_addr_t *pv, **pvp;
	struct vm_physseg *ps;
	size_t i;

	KASSERT(amount & PGOFSET);
	KASSERT((mask & PGOFSET) == 0);
	KASSERT((match & PGOFSET) == 0);
	KASSERT(amount != 0);

	for (pvp = &SLIST_FIRST(&pmap_boot_freeq);
	     (pv = *pvp) != NULL;
	     pvp = &SLIST_NEXT(pv, pv_list)) {
		pv_addr_t *newpv;
		psize_t off;
		/*
		 * If this entry is too small to satify the request...
		 */
		KASSERT(pv->pv_size > 0);
		if (pv->pv_size < amount)
			continue;

		for (off = 0; off <= mask; off += PAGE_SIZE) {
			if (((pv->pv_pa + off) & mask) == match
			    && off + amount <= pv->pv_size)
				break;
		}
		if (off > mask)
			continue;

		rpv->pv_va = pv->pv_va + off;
		rpv->pv_pa = pv->pv_pa + off;
		rpv->pv_size = amount;
		pv->pv_size -= amount;
		if (pv->pv_size == 0) {
			KASSERT(off == 0);
			KASSERT((vaddr_t) pv == rpv->pv_va);
			*pvp = SLIST_NEXT(pv, pv_list);
		} else if (off == 0) {
			KASSERT((vaddr_t) pv == rpv->pv_va);
			newpv = (pv_addr_t *) (rpv->pv_va + amount);
			*newpv = *pv;
			newpv->pv_pa += amount;
			newpv->pv_va += amount;
			*pvp = newpv;
		} else if (off < pv->pv_size) {
			newpv = (pv_addr_t *) (rpv->pv_va + amount);
			*newpv = *pv;
			newpv->pv_size -= off;
			newpv->pv_pa += off + amount;
			newpv->pv_va += off + amount;

			SLIST_NEXT(pv, pv_list) = newpv;
			pv->pv_size = off;
		} else {
			KASSERT((vaddr_t) pv != rpv->pv_va);
		}
		memset((void *)rpv->pv_va, 0, amount);
		return;
	}

	if (vm_nphysseg == 0)
		panic("pmap_boot_pagealloc: couldn't allocate memory");

	for (pvp = &SLIST_FIRST(&pmap_boot_freeq);
	     (pv = *pvp) != NULL;
	     pvp = &SLIST_NEXT(pv, pv_list)) {
		if (SLIST_NEXT(pv, pv_list) == NULL)
			break;
	}
	KASSERT(mask == 0);
	for (i = 0; i < vm_nphysseg; i++) {
		ps = VM_PHYSMEM_PTR(i);
		if (ps->avail_start == atop(pv->pv_pa + pv->pv_size)
		    && pv->pv_va + pv->pv_size <= ptoa(ps->avail_end)) {
			rpv->pv_va = pv->pv_va;
			rpv->pv_pa = pv->pv_pa;
			rpv->pv_size = amount;
			*pvp = NULL;
			pmap_map_chunk(kernel_l1pt.pv_va,
			     ptoa(ps->avail_start) + (pv->pv_va - pv->pv_pa),
			     ptoa(ps->avail_start),
			     amount - pv->pv_size,
			     VM_PROT_READ|VM_PROT_WRITE,
			     PTE_CACHE);
			ps->avail_start += atop(amount - pv->pv_size);
			/*
			 * If we consumed the entire physseg, remove it.
			 */
			if (ps->avail_start == ps->avail_end) {
				for (--vm_nphysseg; i < vm_nphysseg; i++)
					VM_PHYSMEM_PTR_SWAP(i, i + 1);
			}
			memset((void *)rpv->pv_va, 0, rpv->pv_size);
			return;
		}
	}

	panic("pmap_boot_pagealloc: couldn't allocate memory");
}

vaddr_t
pmap_steal_memory(vsize_t size, vaddr_t *vstartp, vaddr_t *vendp)
{
	pv_addr_t pv;

	pmap_boot_pagealloc(size, 0, 0, &pv);

	return pv.pv_va;
}
#endif /* PMAP_STEAL_MEMORY */

SYSCTL_SETUP(sysctl_machdep_pmap_setup, "sysctl machdep.kmpages setup")
{
	sysctl_createv(clog, 0, NULL, NULL,
			CTLFLAG_PERMANENT,
			CTLTYPE_NODE, "machdep", NULL,
			NULL, 0, NULL, 0,
			CTL_MACHDEP, CTL_EOL);

	sysctl_createv(clog, 0, NULL, NULL,
			CTLFLAG_PERMANENT,
			CTLTYPE_INT, "kmpages",
			SYSCTL_DESCR("count of pages allocated to kernel memory allocators"),
			NULL, 0, &pmap_kmpages, 0,
			CTL_MACHDEP, CTL_CREATE, CTL_EOL);
}

#ifdef PMAP_NEED_ALLOC_POOLPAGE
struct vm_page *
pmap_md_alloc_poolpage(int flags)
{
	/*
	 * On some systems, only some pages may be "coherent" for dma and we
	 * want to prefer those for pool pages (think mbufs) but fallback to
	 * any page if none is available.  But we can only fallback if we
	 * aren't direct mapping memory or all of memory can be direct-mapped.
	 * If that isn't true, pool changes can only come from direct-mapped
	 * memory.
	 */
	if (arm_poolpage_vmfreelist != VM_FREELIST_DEFAULT) {
		return uvm_pagealloc_strat(NULL, 0, NULL, flags,
		    UVM_PGA_STRAT_FALLBACK,
		    arm_poolpage_vmfreelist);
	}

	return uvm_pagealloc(NULL, 0, NULL, flags);
}
#endif




#if defined(MULTIPROCESSOR)
void
pmap_md_tlb_info_attach(struct pmap_tlb_info *ti, struct cpu_info *ci)
{
        /* nothing */
}

int
pic_ipi_shootdown(void *arg)
{
#if PMAP_TLB_NEED_SHOOTDOWN
	pmap_tlb_shootdown_process();
#endif
	return 1;
}
#endif /* MULTIPROCESSOR */











#endif



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




#if 0
#ifdef __HAVE_MM_MD_DIRECT_MAPPED_PHYS

vaddr_t
pmap_map_poolpage(paddr_t pa)
{
	bool ok __diagused;
	vaddr_t va = pmap_direct_mapped_phys(pa, &ok, 0);
	KASSERTMSG(ok, "pa %#lx not direct mappable", pa);
	return va;
}

paddr_t
pmap_unmap_poolpage(vaddr_t va)
{
	KASSERT(va >= KERNEL_BASE);
#ifdef PMAP_CACHE_VIVT
	cpu_idcache_wbinv_range(va, PAGE_SIZE);
#endif
	return va - KERNEL_DIRECTMAP_BASE + physical_start;
}
#endif /* __HAVE_MM_MD_DIRECT_MAPPED_PHYS */













#if 0




#if defined(PMAP_CACHE_VIPT) || defined(DEBUG)
	struct vm_page *pg = PHYS_TO_VM_PAGE(pa);
	struct vm_page_md *md = VM_PAGE_TO_MD(pg);
#endif
#if defined(PMAP_CACHE_VIPT)
	/* Choose the last page color it had, if any */
	const vsize_t va_offset = md->pvh_attrs & arm_cache_prefer_mask;
#else
	const vsize_t va_offset = 0;
#endif
#if defined(__HAVE_MM_MD_DIRECT_MAPPED_PHYS)
	/*
	 * Is this page mapped at its natural color?
	 * If we have all of memory mapped, then just convert PA to VA.
	 */
	bool okcolor = arm_pcache.dcache_type == CACHE_TYPE_PIPT
	   || va_offset == (pa & arm_cache_prefer_mask);
	const vaddr_t vdstp = okcolor
	    ? pmap_direct_mapped_phys(pa, &okcolor, cpu_cdstp(va_offset))
	    : cpu_cdstp(va_offset);
#else
	const bool okcolor = false;
	const vaddr_t vdstp = cpu_cdstp(va_offset);
#endif
	pt_entry_t * const ptep = cpu_cdst_pte(va_offset);


#ifdef DEBUG
	if (!SLIST_EMPTY(&md->pvh_list))
		panic("pmap_zero_page: page has mappings");
#endif

	KDASSERT((pa & PGOFSET) == 0);

	if (!okcolor) {
		/*
		 * Hook in the page, zero it, and purge the cache for that
		 * zeroed page. Invalidate the TLB as needed.
		 */
		const pt_entry_t npte = L2_S_PROTO | pa | pte_l2_s_cache_mode
		    | L2_S_PROT(PTE_KERNEL, VM_PROT_WRITE);
		l2pte_set(ptep, npte, 0);
		PTE_SYNC(ptep);
		cpu_tlb_flushD_SE(vdstp);
		cpu_cpwait();
#if defined(__HAVE_MM_MD_DIRECT_MAPPED_PHYS) && defined(PMAP_CACHE_VIPT)
		/*
		 * If we are direct-mapped and our color isn't ok, then before
		 * we bzero the page invalidate its contents from the cache and
		 * reset the color to its natural color.
		 */
		cpu_dcache_inv_range(vdstp, PAGE_SIZE);
		md->pvh_attrs &= ~arm_cache_prefer_mask;
		md->pvh_attrs |= (pa & arm_cache_prefer_mask);
#endif
	}
	bzero_page(vdstp);
	if (!okcolor) {
		/*
		 * Unmap the page.
		 */
		l2pte_reset(ptep);
		PTE_SYNC(ptep);
		cpu_tlb_flushD_SE(vdstp);
#ifdef PMAP_CACHE_VIVT
		cpu_dcache_wbinv_range(vdstp, PAGE_SIZE);
#endif
	}
#ifdef PMAP_CACHE_VIPT
	/*
	 * This page is now cache resident so it now has a page color.
	 * Any contents have been obliterated so clear the EXEC flag.
	 */
	if (!pmap_is_page_colored_p(md)) {
		PMAPCOUNT(vac_color_new);
		md->pvh_attrs |= PVF_COLORED;
	}





#endif





























static register_t
pmap_md_map_ephemeral_page(struct vm_page *pg, bool locked_p, int prot,
    pt_entry_t *old_pte_p)
{
	const paddr_t pa = VM_PAGE_TO_PHYS(pg);
	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);
	pv_entry_t pv = &mdpg->mdpg_first;
	register_t va = 0;

	UVMHIST_FUNC(__func__); UVMHIST_CALLED(pmaphist);
	UVMHIST_LOG(pmaphist, "(pg=%p, prot=%d, ptep=%p)",
	    pg, prot, old_pte_p, 0);

	KASSERT(!locked_p || VM_PAGEMD_PVLIST_LOCKED_P(mdpg));

#if defined(PMAP_CACHE_VIPT)
	/* Choose the last page color it had, if any */
	const vsize_t va_offset = pv->pv_va & arm_cache_prefer_mask;
#else
	const vsize_t va_offset = 0;
#endif
	const vaddr_t xva =
	    prot & VM_PROT_WRITE ? cpu_cdstp(va_offset) : cpu_csrcp(va_offset);
#if defined(__HAVE_MM_MD_DIRECT_MAPPED_PHYS)
	/*
	 * Is this page mapped at its natural color?
	 * If we have all of memory mapped, then just convert PA to VA.
	 */
	bool okcolor = arm_pcache.dcache_type == CACHE_TYPE_PIPT
	   || va_offset == (pa & arm_cache_prefer_mask);
	const vaddr_t va =
	    okcolor ? pmap_direct_mapped_phys(pa, &okcolor, xva) : xva;
#else
	const bool okcolor = false;
	const vaddr_t va = xva;
#endif
	pt_entry_t * const ptep =
	    prot & VM_PROT_WRITE ? cpu_cdst_pte(va_offset) : cpu_csrc_pte(va_offset);

	KDASSERT((pa & PGOFSET) == 0);

	if (!okcolor) {
		/*
		 * Hook in the page, zero it, and purge the cache for that
		 * zeroed page. Invalidate the TLB as needed.
		 */
		const pt_entry_t npte = L2_S_PROTO | pa | pte_l2_s_cache_mode
		    | L2_S_PROT(PTE_KERNEL, VM_PROT_WRITE);
		l2pte_set(ptep, npte, 0);
		PTE_SYNC(ptep);
		cpu_tlb_flushD_SE(va);
		cpu_cpwait();
#if defined(__HAVE_MM_MD_DIRECT_MAPPED_PHYS) && defined(PMAP_CACHE_VIPT)
		/*
		 * If we are direct-mapped and our color isn't ok, then before
		 * we bzero the page invalidate its contents from the cache and
		 * reset the color to its natural color.
		 */
		cpu_dcache_inv_range(vdstp, PAGE_SIZE);
		md->pvh_attrs &= ~arm_cache_prefer_mask;
		md->pvh_attrs |= (pa & arm_cache_prefer_mask);
#endif





	if (!MIPS_CACHE_VIRTUAL_ALIAS || !mips_cache_badalias(pv->pv_va, pa)) {
#ifdef _LP64
		va = MIPS_PHYS_TO_XKPHYS_CACHED(pa);
#else
		if (pa < MIPS_PHYS_MASK) {
			va = MIPS_PHYS_TO_KSEG0(pa);
		}
#endif
	}
#if defined(PMAP_CACHE_VIPT)
	if (va == 0) {
		/*
		 * Make sure to use a congruent mapping to the last mapped
		 * address so we don't have to worry about virtual aliases.
		 */
		kpreempt_disable(); // paired with the one in unmap
		struct cpu_info * const ci = curcpu();
		if (MIPS_CACHE_VIRTUAL_ALIAS) {
			KASSERT(ci->ci_pmap_dstbase != 0);
			KASSERT(ci->ci_pmap_srcbase != 0);

			const u_int __diagused mask = pmap_page_cache_alias_mask;
			KASSERTMSG((ci->ci_pmap_dstbase & mask) == 0,
			    "%#"PRIxVADDR, ci->ci_pmap_dstbase);
			KASSERTMSG((ci->ci_pmap_srcbase & mask) == 0,
			    "%#"PRIxVADDR, ci->ci_pmap_srcbase);
		}
		vaddr_t nva = (prot & VM_PROT_WRITE
			? ci->ci_pmap_dstbase
			: ci->ci_pmap_srcbase)
		    + pmap_md_cache_indexof(MIPS_CACHE_VIRTUAL_ALIAS
			? pv->pv_va
			: pa);

		va = (intptr_t)nva;
		/*
		 * Now to make and write the new PTE to map the PA.
		 */
		const pt_entry_t npte = pte_make_kenter_pa(pa, mdpg, prot, 0);
		pt_entry_t * const ptep = pmap_pte_lookup(pmap_kernel(), va);
		*old_pte_p = *ptep;		// save
		bool rv __diagused;
		*ptep = npte;			// update page table

		// update the TLB directly making sure we force the new entry
		// into it.
		rv = tlb_update_addr(va, KERNEL_PID, npte, true);
		KASSERTMSG(rv == 1, "va %#"PRIxREGISTER" pte=%#"PRIxPTE" rv=%d",
		    va, pte_value(npte), rv);
	}
	if (MIPS_CACHE_VIRTUAL_ALIAS) {
		/*
		 * If we are forced to use an incompatible alias, flush the
		 * page from the cache so we will copy the correct contents.
		 */
		if (!locked_p)
			(void)VM_PAGEMD_PVLIST_READLOCK(mdpg);
		if (VM_PAGEMD_CACHED_P(mdpg)
		    && mips_cache_badalias(pv->pv_va, va)) {
			register_t ova = (intptr_t)trunc_page(pv->pv_va);
			mips_dcache_wbinv_range_index(ova, PAGE_SIZE);
			/*
			 * If there is no active mapping, remember this new one.
			 */
			if (pv->pv_pmap == NULL)
				pv->pv_va = va;
		}
		if (!locked_p)
			VM_PAGEMD_PVLIST_UNLOCK(mdpg);
	}

	UVMHIST_LOG(pmaphist, " <-- done (va=%#lx)", va, 0, 0, 0);

	return va;
}

static void
pmap_md_unmap_ephemeral_page(struct vm_page *pg, bool locked_p, register_t va,
	pt_entry_t old_pte)
{
	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);
	pv_entry_t pv = &mdpg->mdpg_first;

	UVMHIST_FUNC(__func__); UVMHIST_CALLED(pmaphist);
	UVMHIST_LOG(pmaphist, "(pg=%p, va=%#lx, pte=%#"PRIxPTE")",
	    pg, va, pte_value(old_pte), 0);

	KASSERT(!locked_p || VM_PAGEMD_PVLIST_LOCKED_P(mdpg));

	if (MIPS_CACHE_VIRTUAL_ALIAS) {
		if (!locked_p)
			(void)VM_PAGEMD_PVLIST_READLOCK(mdpg);
		/*
		 * If this page was previously uncached or we had to use an
		 * incompatible alias, flush it from the cache.
		 */
		if (VM_PAGEMD_UNCACHED_P(mdpg)
		    || (pv->pv_pmap != NULL
			&& mips_cache_badalias(pv->pv_va, va))) {
			mips_dcache_wbinv_range(va, PAGE_SIZE);
		}
		if (!locked_p)
			VM_PAGEMD_PVLIST_UNLOCK(mdpg);
	}
	/*
	 * If we had to map using a page table entry, restore it now.
	 */
	if (!pmap_md_direct_mapped_vaddr_p(va)) {
		*pmap_pte_lookup(pmap_kernel(), va) = old_pte;
		if (pte_valid_p(old_pte)) {
			// Update the TLB with the old mapping.
			tlb_update_addr(va, KERNEL_PID, old_pte, 0);
		} else {
			// Invalidate TLB entry if the old pte wasn't valid.
			tlb_invalidate_addr(va, KERNEL_PID);
		}
		kpreempt_enable();	// Restore preemption
	}
	UVMHIST_LOG(pmaphist, " <-- done", 0, 0, 0, 0);
}




















#endif








#if 0


/*
 *	pmap_zero_page zeros the specified page.
 */
void
pmap_zero_page(paddr_t dst_pa)
{
	pt_entry_t dst_pte;

	UVMHIST_FUNC(__func__); UVMHIST_CALLED(pmaphist);
	UVMHIST_LOG(pmaphist, "(pa=%#"PRIxPADDR")", dst_pa, 0, 0, 0);
	PMAP_COUNT(zeroed_pages);

	struct vm_page * const dst_pg = PHYS_TO_VM_PAGE(dst_pa);

	KASSERT(!VM_PAGEMD_EXECPAGE_P(VM_PAGE_TO_MD(dst_pg)));

	const register_t dst_va = pmap_md_map_ephemeral_page(dst_pg, false,
	    VM_PROT_READ|VM_PROT_WRITE, &dst_pte);

	bzero_page(dst_va);

	pmap_md_unmap_ephemeral_page(dst_pg, false, dst_va, dst_pte);

	UVMHIST_LOG(pmaphist, " <-- done", 0, 0, 0, 0);
}

/*
 *	pmap_copy_page copies the specified page.
 */
void
pmap_copy_page(paddr_t src_pa, paddr_t dst_pa)
{
	pt_entry_t src_pte, dst_pte;

	UVMHIST_FUNC(__func__); UVMHIST_CALLED(pmaphist);
	UVMHIST_LOG(pmaphist, "(src_pa=%#lx, dst_pa=%#lx)", src_pa, dst_pa, 0, 0);
	PMAP_COUNT(copied_pages);

	struct vm_page * const src_pg = PHYS_TO_VM_PAGE(src_pa);
	struct vm_page * const dst_pg = PHYS_TO_VM_PAGE(dst_pa);

	const register_t src_va = pmap_md_map_ephemeral_page(src_pg, false,
	    VM_PROT_READ, &src_pte);

	KASSERT(VM_PAGE_TO_MD(dst_pg)->mdpg_first.pv_pmap == NULL);
	KASSERT(!VM_PAGEMD_EXECPAGE_P(VM_PAGE_TO_MD(dst_pg)));
	const register_t dst_va = pmap_md_map_ephemeral_page(dst_pg, false,
	    VM_PROT_READ|VM_PROT_WRITE, &dst_pte);

	bcopy_page(src_va, dst_va);

	pmap_md_unmap_ephemeral_page(dst_pg, false, dst_va, dst_pte);
	pmap_md_unmap_ephemeral_page(src_pg, false, src_va, src_pte);

	UVMHIST_LOG(pmaphist, " <-- done", 0, 0, 0, 0);
}

void
pmap_md_page_syncicache(struct vm_page *pg, const kcpuset_t *onproc)
{
	UVMHIST_FUNC(__func__); UVMHIST_CALLED(pmaphist);
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
			const register_t tva = pmap_md_map_ephemeral_page(pg, false,
			    VM_PROT_READ, &pte);

			UVMHIST_LOG(pmaphist, "  va %#"PRIxVADDR, tva, 0, 0, 0);
			mips_dcache_wbinv_range(tva, PAGE_SIZE);
			mips_icache_sync_range(tva, PAGE_SIZE);

			pmap_md_unmap_ephemeral_page(pg, false, tva, pte);
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
}




#endif
#endif
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
			pmap_md_vca_page_wbinv(pg, false);
		}
#endif
		if (0 /* bad alias */)
			pmap_md_vca_page_wbinv(pg, false);

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



struct vm_page *pmap_md_alloc_poolpage(int);



void
pmap_impl_bootstrap(void)
{
	KASSERT(pte_l1_s_cache_mode == pte_l1_s_cache_mode_pt);
	KASSERT(pte_l2_s_cache_mode == pte_l2_s_cache_mode_pt);

	pmap_t pm = pmap_kernel();

	pm->pm_l1 = (pd_entry_t *)kernel_l1pt.pv_va;
	pm->pm_l1_pa = kernel_l1pt.pv_pa;

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


void
pmap_md_pdetab_activate(pmap_t pm, struct lwp *l)
{
	UVMHIST_FUNC(__func__); UVMHIST_CALLED(maphist);

	/*
	 * Assume that TTBR1 has only global mappings and TTBR0 only
	 * has non-global mappings.  To prevent speculation from doing
	 * evil things we disable translation table walks using TTBR0
	 * before setting the CONTEXTIDR (ASID) or new TTBR0 value.
	 * Once both are set, table walks are reenabled.
	 */
	const uint32_t old_ttbcr = armreg_ttbcr_read();
	armreg_ttbcr_write(old_ttbcr | TTBCR_S_PD0);
	arm_isb();

	pmap_tlb_asid_acquire(pm, l);

	struct cpu_info * const ci = curcpu();
	struct pmap_asid_info * const pai = PMAP_PAI(pm, cpu_tlb_info(ci));

	cpu_setttb(pm->pm_l1_pa, pai->pai_asid);
	/*
	 * Now we can reenable tablewalks since the CONTEXTIDR and TTRB0
	 * have been updated.
	 */
	arm_isb();

	if (pm != pmap_kernel()) {
		armreg_ttbcr_write(old_ttbcr & ~TTBCR_S_PD0);
	}
	cpu_cpwait();

	UVMHIST_LOG(maphist, " pm %#jx pm->pm_l1_pa %08jx asid %ju... done",
	    (uintptr_t)pm, pm->pm_l1_pa, pai->pai_asid, 0);

	KASSERTMSG(ci->ci_pmap_asid_cur == pai->pai_asid, "%u vs %u",
	    ci->ci_pmap_asid_cur, pai->pai_asid);
	ci->ci_pmap_cur = pm;
}

void
pmap_md_pdetab_deactivate(pmap_t pm)
{
	UVMHIST_FUNC(__func__); UVMHIST_CALLED(maphist);

	kpreempt_disable();
	struct cpu_info * const ci = curcpu();
	/*
	 * Disable translation table walks from TTBR0 while no pmap has been
	 * activated.
	 */
	const uint32_t old_ttbcr = armreg_ttbcr_read();
	armreg_ttbcr_write(old_ttbcr | TTBCR_S_PD0);
	arm_isb();
	pmap_tlb_asid_deactivate(pm);
	cpu_setttb(pmap_kernel()->pm_l1_pa, KERNEL_PID);
	arm_isb();

	ci->ci_pmap_cur = pmap_kernel();
	KASSERTMSG(ci->ci_pmap_asid_cur == KERNEL_PID, "ci_pmap_asid_cur %u",
	    ci->ci_pmap_asid_cur);
	kpreempt_enable();
}

pt_entry_t *
pmap_md_pdetab_lookup_ptep(struct pmap *pmap, vaddr_t va)
{
	struct l2_bucket * const l2b = pmap_get_l2_bucket(pmap, va);
	if (l2b == NULL)
		return NULL;

	pt_entry_t *ptep = &l2b->l2b_kva[l2pte_index(va)];

	return ptep;
}
















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
//        pmap_tlb_info_evcnt_attach(&pmap_tlb0_info);
}

void
pmap_md_pdetab_init(struct pmap *pm)
{
	KASSERT(pm != NULL);

        pmap_alloc_l1(pm);

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
pmap_md_pdetab_destroy(struct pmap *pm)
{
	KASSERT(pm != NULL);

	pmap_free_l1(pm);
}




#if 0

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
pmap_md_vca_page_wbinv(struct vm_page *pg, bool locked_p)
{
	UVMHIST_FUNC(__func__); UVMHIST_CALLED(pmaphist);
#ifdef needtowrite
	pt_entry_t pte;

	const register_t va = pmap_md_map_ephemeral_page(pg, locked_p,
	    VM_PROT_READ, &pte);

	mips_dcache_wbinv_range(va, PAGE_SIZE);

	pmap_md_unmap_ephemeral_page(pg, locked_p, va, pte);
#endif
}

void
pmap_md_page_syncicache(struct vm_page *pg, const kcpuset_t *onproc)
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
			const register_t tva = pmap_md_map_ephemeral_page(pg, false,
			    VM_PROT_READ, &pte);

			UVMHIST_LOG(pmaphist, "  va %#"PRIxVADDR, tva, 0, 0, 0);
			mips_dcache_wbinv_range(tva, PAGE_SIZE);
			mips_icache_sync_range(tva, PAGE_SIZE);

			pmap_md_unmap_ephemeral_page(pg, false, tva, pte);
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
			pmap_md_vca_page_wbinv(pg, true);
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
pmap_md_vca_clean(struct vm_page *pg, int op)
{
	UVMHIST_FUNC(__func__); UVMHIST_CALLED(pmaphist);
#ifdef needtowrite
	if (!MIPS_HAS_R4K_MMU || !MIPS_CACHE_VIRTUAL_ALIAS)
		return;

	UVMHIST_LOG(pmaphist, "(pg=%p, op=%d)", pg, op, 0, 0);
	KASSERT(VM_PAGEMD_PVLIST_LOCKED_P(VM_PAGE_TO_MD(pg)));

	if (op == PMAP_WB || op == PMAP_WBINV) {
		pmap_md_vca_page_wbinv(pg, true);
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
	KASSERT(!VM_PAGEMD_PVLIST_LOCKED_P(mdpg));
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

