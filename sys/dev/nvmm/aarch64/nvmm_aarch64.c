/*	$NetBSD$	*/

/*-
 * Copyright (c) 2023 Ryo Shimizu <ryo@nerv.org>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include "opt_arm_debug.h"
#include "opt_console.h"

#include <sys/param.h>

#include <sys/kmem.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/xcall.h>

#include <uvm/uvm.h>

#include <dev/nvmm/nvmm.h>
#include <dev/nvmm/nvmm_internal.h>
#include <dev/nvmm/aarch64/nvmm_aarch64.h>
#include <dev/nvmm/aarch64/nvmm_aarch64_internal.h>

#include <machine/bootconfig.h>

#include <aarch64/cpufunc.h>
#include <aarch64/pmap.h>

#define AARCH64_VMID(mach) (mach->machid + 1) /* avoid 0. 0 is host's VMID */

struct aarch64_machdata {
	uint64_t vttbr_el2;
};

void aarch64_hvc_init(paddr_t);
void aarch64_hvc_vmenter(paddr_t);
void aarch64_hvc_maintain_ipa(uint64_t, uint64_t, uint64_t, uint64_t);
static void nvmm_aarch64_vcpu_setstate(struct nvmm_cpu *vcpu);

struct nvmm_aarch64_state nvmm_aarch64_reset_state = {
	.gprs = {},	/* x0-x31 are all zero */
	.fprs = {},	/* q0-q31 are all zero */
	.sprs = {
		[NVMM_AARCH64_SPR_SCTLR_EL1]	= SCTLR_RES1,
		[NVMM_AARCH64_SPR_SPSR_EL1]	= SPSR_M_EL1H,
	},
	.tids = {}	/* initted in nvmm_aarch64_init_reset_state() */
};

int nvmm_available;

int nvmm_debug;	/* XXX */

SYSCTL_SETUP(sysctl_machdep_nvmm_setup, "sysctl machdep.nvmm setup")
{
	int err;
	const struct sysctlnode *rnode;
	const struct sysctlnode *cnode;

	err = sysctl_createv(clog, 0, NULL, &rnode,
	    CTLFLAG_PERMANENT, CTLTYPE_NODE, "nvmm",
	    SYSCTL_DESCR("nvmm global controls"),
	    NULL, 0, NULL, 0, CTL_MACHDEP, CTL_CREATE, CTL_EOL);

	if (err)
		goto fail;

	/* control debugging printfs */
	err = sysctl_createv(clog, 0, &rnode, &cnode,
	    CTLFLAG_PERMANENT | CTLFLAG_READWRITE, CTLTYPE_INT,
	    "debug", SYSCTL_DESCR("Enable debugging output"),
	    NULL, 0, &nvmm_debug, sizeof(nvmm_debug), CTL_CREATE, CTL_EOL);
	if (err)
		goto fail;

	return;
fail:
	aprint_error("%s: sysctl_createv failed (err = %d)\n", __func__, err);
}


static unsigned int stage2_startlevel;
static unsigned int stage2_concatenate_num;

static void
nvmm_aarch64_init_reset_state(void)
{
	uint64_t val;

	/* XXXXXXXX: For big.LITTLE, the smallest feature must be selected */
	nvmm_aarch64_reset_state.tids[NVMM_AARCH64_TID_MVFR0_EL1] = reg_mvfr0_el1_read();
	nvmm_aarch64_reset_state.tids[NVMM_AARCH64_TID_MVFR1_EL1] = reg_mvfr1_el1_read();
	nvmm_aarch64_reset_state.tids[NVMM_AARCH64_TID_MVFR2_EL1] = reg_mvfr2_el1_read();
	nvmm_aarch64_reset_state.tids[NVMM_AARCH64_TID_ID_AA64PFR0_EL1] = reg_id_aa64pfr0_el1_read();
	nvmm_aarch64_reset_state.tids[NVMM_AARCH64_TID_ID_AA64PFR1_EL1] = reg_id_aa64pfr1_el1_read();

	/* XXXXXXX: pmu not supported yet */
	val = reg_id_aa64dfr0_el1_read();
	val &= ~ID_AA64DFR0_EL1_PMUVER;
	val |= __SHIFTIN(ID_AA64DFR0_EL1_PMUVER_NONE, ID_AA64DFR0_EL1_PMUVER);
	nvmm_aarch64_reset_state.tids[NVMM_AARCH64_TID_ID_AA64DFR0_EL1] = val;

	nvmm_aarch64_reset_state.tids[NVMM_AARCH64_TID_ID_AA64DFR1_EL1] = reg_id_aa64dfr1_el1_read();
	nvmm_aarch64_reset_state.tids[NVMM_AARCH64_TID_ID_AA64AFR0_EL1] = reg_id_aa64afr0_el1_read();
	nvmm_aarch64_reset_state.tids[NVMM_AARCH64_TID_ID_AA64AFR1_EL1] = reg_id_aa64afr1_el1_read();
	nvmm_aarch64_reset_state.tids[NVMM_AARCH64_TID_ID_AA64ISAR0_EL1] = reg_id_aa64isar0_el1_read();
	nvmm_aarch64_reset_state.tids[NVMM_AARCH64_TID_ID_AA64ISAR1_EL1] = reg_id_aa64isar1_el1_read();
	nvmm_aarch64_reset_state.tids[NVMM_AARCH64_TID_ID_AA64MMFR0_EL1] = reg_id_aa64mmfr0_el1_read();
	nvmm_aarch64_reset_state.tids[NVMM_AARCH64_TID_ID_AA64MMFR1_EL1] = reg_id_aa64mmfr1_el1_read();
	nvmm_aarch64_reset_state.tids[NVMM_AARCH64_TID_ID_AA64MMFR2_EL1] = reg_id_aa64mmfr2_el1_read();
}

static bool
nvmm_aarch64_ident(void)
{
	return true;
}

static pd_entry_t *
nvmm_aarch64_pagealloc(void)
{
	struct vm_page *pg;

	for (;;) {
		pg = uvm_pagealloc(NULL, 0, NULL, UVM_PGA_ZERO);
		if (pg != NULL)
			break;
		uvm_wait("nvmm_aarch64_init");
	}
	pg->flags &= ~PG_BUSY;	/* never busy */
	return (pd_entry_t *)VM_PAGE_TO_PHYS(pg);
}

static void
nvmm_aarch64_el2_setup(void)
{
	/*
	 * calculate VTCR_EL2 setting from ID_AA64MMFR0_EL1.PARange
	 */
	struct cpu_info *ci;
	CPU_INFO_ITERATOR cii;
	uint64_t mmfr0_parange, parange;
	uint64_t vtcr_ps, vtcr_options = 0, vtcr_el2;

	/* pick up the smallest PARange among all CPUs */
	vtcr_ps = ID_AA64MMFR0_EL1_PARANGE;
	for (CPU_INFO_FOREACH(cii, ci)) {
		mmfr0_parange = __SHIFTOUT(reg_id_aa64mmfr0_el1_read(),
		    ID_AA64MMFR0_EL1_PARANGE);
		if (vtcr_ps > mmfr0_parange)
			vtcr_ps = mmfr0_parange;
	}
#if NVMM_MAX_RAM <= (4 * 1024 * 1024 * 1024)
	mmfr0_parange = ID_AA64MMFR0_EL1_PARANGE_4G;
#elif NVMM_MAX_RAM <= (64 * 1024 * 1024 * 1024)
	mmfr0_parange = ID_AA64MMFR0_EL1_PARANGE_64G;
#elif NVMM_MAX_RAM <= (1 * 1024 * 1024 * 1024 * 1024)
	mmfr0_parange = ID_AA64MMFR0_EL1_PARANGE_1T;
#elif NVMM_MAX_RAM <= (4 * 1024 * 1024 * 1024 * 1024)
	mmfr0_parange = ID_AA64MMFR0_EL1_PARANGE_4T;
#elif NVMM_MAX_RAM <= (16 * 1024 * 1024 * 1024 * 1024)
	mmfr0_parange = ID_AA64MMFR0_EL1_PARANGE_16T;
#elif NVMM_MAX_RAM <= (256 * 1024 * 1024 * 1024 * 1024)
	mmfr0_parange = ID_AA64MMFR0_EL1_PARANGE_256T
#else
#error Physical addresses of 48bits or more are not supported
#endif
	if (vtcr_ps > mmfr0_parange)
		vtcr_ps = mmfr0_parange;

	switch (vtcr_ps) {
	case ID_AA64MMFR0_EL1_PARANGE_4G:
		parange = 32;
		break;
	case ID_AA64MMFR0_EL1_PARANGE_64G:
		parange = 36;
		break;
	case ID_AA64MMFR0_EL1_PARANGE_1T:
		parange = 40;
		break;
	case ID_AA64MMFR0_EL1_PARANGE_4T:
		parange = 42;
		break;
	case ID_AA64MMFR0_EL1_PARANGE_16T:
		parange = 44;
		break;
	case ID_AA64MMFR0_EL1_PARANGE_256T:
	default:
		vtcr_ps = ID_AA64MMFR0_EL1_PARANGE_256T;
		parange = 48;
		break;
	}

	CTASSERT(PGSHIFT == 12);
	/*
	 * 12: bitwidth of page (4Kpage)
	 *  9: bitwidth of PTE entries per page (4k/sizeof(pte) = 512)
	 *  4: bitwidth of maximum number of concatenated TTBR (16)
	 *
	 * PArange      Initial Concat-
	 * bit           Lookup enated Ln
	 * width          Level table num Behaviour
	 * ------------ ------- --------- ------------------------------------------------
	 * 30(12+9+9)         2         1                       L2[512] -> L3[512] -> page
	 * 31(12+9+9+1)       2         2                   L2c[2][512] -> L3[512] -> page
	 * 32(12+9+9+2)       2         4                   L2c[4][512] -> L3[512] -> page
	 * 33(12+9+9+3)       2         8                   L2c[8][512] -> L3[512] -> page
	 * 34(12+9+9+4)       2        16                  L2c[16][512] -> L3[512] -> page
	 * 35(12+9+9+5)       1         1            L1[32]  -> L2[512] -> L3[512] -> page
	 * 36(12+9+9+6)       1         1            L1[64]  -> L2[512] -> L3[512] -> page
	 * 37(12+9+9+7)       1         1            L1[128] -> L2[512] -> L3[512] -> page
	 * 38(12+9+9+8)       1         1            L1[256] -> L2[512] -> L3[512] -> page
	 * 39(12+9+9+9)       1         1            L1[512] -> L2[512] -> L3[512] -> page
	 * 40(12+9+9+9+1)     1         2        L1c[2][512] -> L2[512] -> L3[512] -> page
	 * 41(12+9+9+9+2)     1         4        L1c[4][512] -> L2[512] -> L3[512] -> page
	 * 42(12+9+9+9+3)     1         8        L1c[8][512] -> L2[512] -> L3[512] -> page
	 * 43(12+9+9+9+4)     1        16       L1c[16][512] -> L2[512] -> L3[512] -> page
	 * 44(12+9+9+9+5)     0         1 L0[32]  -> L1[512] -> L2[512] -> L3[512] -> page
	 * 45(12+9+9+9+6)     0         1 L0[64]  -> L1[512] -> L2[512] -> L3[512] -> page
	 * 46(12+9+9+9+7)     0         1 L0[128] -> L1[512] -> L2[512] -> L3[512] -> page
	 * 47(12+9+9+9+8)     0         1 L0[256] -> L1[512] -> L2[512] -> L3[512] -> page
	 * 48(12+9+9+9+9)     0         1 L0[512] -> L1[512] -> L2[512] -> L3[512] -> page
	 *
	 * L0,L1,L2,L3 are normal Ln table.
	 * L1c and L2c are concatenated Ln table.
	 */
	if (parange <= (12 + 9 + 9 + 4)) {		/* PArange <= 34bit */
		stage2_startlevel = 2;
		if (parange >= 12 + 9 + 9)
			stage2_concatenate_num = 1 << (parange - 12 - 9 - 9);
		else
			stage2_concatenate_num = 1;
	} else if (parange <= (12 + 9 + 9 + 9 + 4)) {	/* PArange <= 43bit */
		stage2_startlevel = 1;
		if (parange >= 12 + 9 + 9 + 9)
			stage2_concatenate_num = 1 << (parange - 12 - 9 - 9 - 9);
		else
			stage2_concatenate_num = 1;
	} else {
		stage2_startlevel = 0;
		stage2_concatenate_num = 0;
	}

#ifdef ARMV81_HAFDBS
	switch (aarch64_hafdbs_enabled) {
	case ID_AA64MMFR1_EL1_HAFDBS_NONE:
		break;
	case ID_AA64MMFR1_EL1_HAFDBS_A:
		vtcr_options |= TCR_HA;
		break;
	case ID_AA64MMFR1_EL1_HAFDBS_AD:
		vtcr_options |= (TCR_HD | TCR_HA);
		break;
	}
#endif

#if 0
	if (__SHIFTOUT(ID_AA64PFR0_EL1_SEL2, reg_id_aa64pfr0_el1_read()) !=
	    ID_AA64PFR0_EL1_SEL2_NONE) {
		vtcr_options |= VTCR_EL2_NSA;
		vtcr_options |= VTCR_EL2_NSW;
	}
#endif

	vtcr_el2 =
	    __BIT(31) |				/* RES1 */
	    vtcr_options |
	    __SHIFTIN(vtcr_ps, VTCR_EL2_PS) |
	    __SHIFTIN(0, VTCR_EL2_TG0) |	/* 4k page */
#ifdef MULTIPROCESSOR
	    __SHIFTIN(3, VTCR_EL2_SH0) |	/* Inner Shareable */
#else
	    __SHIFTIN(0, VTCR_EL2_SH0) |	/* Non-Shareable */
#endif
	    __SHIFTIN(1, VTCR_EL2_ORGN0) |	/* WB WA */
	    __SHIFTIN(1, VTCR_EL2_IRGN0) |	/* WB WA */
	    __SHIFTIN(2 - stage2_startlevel, VTCR_EL2_SL0) |
	    __SHIFTIN(64 - parange, VTCR_EL2_T0SZ);

	printf("%s: VTCR_EL2=%08"PRIx64", vtcr_ps=%"PRIu64", parange=%"PRIu64", stage2_startlevel=%u, stage2_concatenate_num=%u\n",
	    cpu_name(curcpu()), vtcr_el2, vtcr_ps, parange, stage2_startlevel, stage2_concatenate_num);

	/*
	 * Enable EL2 MMU via hvc. The entity is in aarch64_el2_init().
	 * There is a way to do it at the beginning of aarch64/locore_el2,
	 * but the process is complicated, so we do it in this nvmm initialization.
	 *
	 * Once the EL2 MMU is enabled, it is never disabled again.
	 * EL2 VA=PA identity mapping is enabled until reboot.
	 * Allocated page tables are not released by calling nvmm_aarch64_fini().
	 */
	static pd_entry_t *ttbr_pa;
	if (ttbr_pa != NULL)
		return;

	ttbr_pa = nvmm_aarch64_pagealloc();

#ifdef VERBOSE_INIT_ARM
	printf("Creating EL2 page tables\n");
#define PRFUNC	printf
#else
#define PRFUNC	NULL
#endif

#ifdef CONSADDR
	/* XXX: for EL2 uartprintf debugging */
	const pt_entry_t devattr = LX_BLKPAG_ATTR_DEVICE_MEM |
	    LX_BLKPAG_AP_RW | LX_BLKPAG_XN;
	pmapboot_enter_ttbr(CONSADDR, CONSADDR, L2_SIZE, L2_SIZE,
	    devattr, PRFUNC, ttbr_pa, true, nvmm_aarch64_pagealloc);
#endif

	/* EL2 VA=PA identity mapping */
	const pt_entry_t memattr = LX_BLKPAG_ATTR_NORMAL_WB |
	    LX_BLKPAG_AP_RW;
	for (u_int blk = 0; blk < bootconfig.dramblocks; blk++) {
		uint64_t start, end;

		start = trunc_page(bootconfig.dram[blk].address);
		end = round_page(bootconfig.dram[blk].address +
		(uint64_t)bootconfig.dram[blk].pages * PAGE_SIZE);

		pmapboot_enter_range_ttbr(start, start, end - start,
		    memattr, PRFUNC, ttbr_pa, true, nvmm_aarch64_pagealloc);
	}

	/* XXX: no need to flush cache here? EL2 may be cache off... */

	/* calll aarch64_hvc_init(ttbr_pa, vtcr_el2) on all cpus */
	uint64_t where = xc_broadcast(0, (xcfunc_t)aarch64_hvc_init,
	    (void *)ttbr_pa, (void *)vtcr_el2);
	xc_wait(where);

	aarch64_el2_initted = 1;
}

static void
nvmm_aarch64_init(void)
{
	if (nvmm_available == 0)
		return;

	nvmm_aarch64_init_reset_state();
	nvmm_aarch64_el2_setup();
}

static void
nvmm_aarch64_fini(void)
{
}

static void
nvmm_aarch64_capability(struct nvmm_capability *cap)
{
	/*
	 * VMID 0 is for netbsd kernel, and VMID of aarch64 is 8bit.
	 * then, number of max machines is 254.
	 */
	KASSERT(cap->max_machines < 255);

	cap->arch.mach_conf_support = 0;
	cap->arch.vcpu_conf_support = 0;
}

/*
 * Buffers for stage2 concatenated table must be aligned PAGE_SIZE*2^n.
 * Even if NVVMM_MAX_MACHINES=128, it is at most 8Mbytes. (4kpage)
 * It would be better than dynamically allocating them and getting caught up
 * in pmap page table management. (_pmap_pdp_*() in aarch64/pmap.c)
 */
static char stage2table_buf[NVMM_MAX_MACHINES][PAGE_SIZE * 16] __aligned(PAGE_SIZE * 16);

static void
nvmm_aarch64_machine_create(struct nvmm_machine *mach)
{
	struct aarch64_machdata *machdata;
	struct pmap *pm;
	size_t concat_tablesize;

	/* XXX: (*nvmm_impl->machine_create)(mach) should be able to return an error... */
	if (AARCH64_VMID(mach) >= 0x100)
		panic("%s: VMID %d is too large", __func__, AARCH64_VMID(mach));

	/* set aarch64 pmap to stage2 mode */
	pm = mach->vm->vm_map.pmap;
	pm->pm_stage2 = true;
	pm->pm_nvmm = mach;
	pm->pm_st2_startlevel = stage2_startlevel;
	pm->pm_st2_concatenate_num = stage2_concatenate_num;

	if (pm->pm_st2_startlevel > 0) {
		/*
		 * concatenated initial lookup translation table must be aligned
		 * to pagesize*2^n
		 */
		concat_tablesize = PAGE_SIZE * pm->pm_st2_concatenate_num;
#if 1
		pm->pm_st2_table =
		    (pd_entry_t *)stage2table_buf[mach->machid];
		pmap_extract(pmap_kernel(), (vaddr_t)pm->pm_st2_table,
		    &pm->pm_st2_table_pa);
#else
		/* allocate dynamically. Not well tested. */
		struct pglist pglist;
		int error = uvm_pglistalloc(concat_tablesize, 0, ~0UL,
		    concat_tablesize, 0, &pglist, 1, 1);
		if (error != 0) {
			panic("%s: cannot allocate initial lookup page",
			    __func__);
		}
		pm->pm_st2_table_pa = VM_PAGE_TO_PHYS(TAILQ_FIRST(&pglist));
		pm->pm_st2_table =
		    (pd_entry_t *)AARCH64_PA_TO_KVA(pm->pm_st2_table_pa);
		/* XXX: TODO: free pglist in nvmm_aarch64_machine_destroy() */
#endif

		KASSERT((pm->pm_st2_table_pa & (concat_tablesize - 1)) == 0);
		memset(pm->pm_st2_table, 0, concat_tablesize);

		/* XXX */
		if (pm->pm_st2_startlevel != 1)
			panic("Initial lookup from L2 is not supported yet");

		/*
		 * create L0->L1 table entry for pmap. The hardware MMU starts
		 * the lookup from the concatenated L1 table.
		 */
		for (int i = 0; i < pm->pm_st2_concatenate_num; i++) {
			pm->pm_l0table[i] =
			    (pm->pm_st2_table_pa + i * PAGE_SIZE) |
			    LX_TYPE_TBL | LX_VALID | LX_BLKPAG_OS_STAGE2;
		}
	}


	//XXXXXXXXXXXXXXXXXXXXXXXXXXX
//	if (nvmm_debug) {
		printf("%s:%s:%d: pmap pm=%p, l0table_pa=%016lx, st2_stabtlevel=%d, st2_table=%p, st2_table_pa=%016lx (%d concatenated)\n",
		    cpu_name(curcpu()), __func__, __LINE__,
		    mach->vm->vm_map.pmap,
		    pm->pm_l0table_pa,
		    pm->pm_st2_startlevel,
		    mach->vm->vm_map.pmap->pm_st2_table,
		    mach->vm->vm_map.pmap->pm_st2_table_pa,
		    pm->pm_st2_concatenate_num);
//	}

	machdata = kmem_zalloc(sizeof(struct aarch64_machdata), KM_SLEEP);
	mach->machdata = machdata;
}

static void
nvmm_aarch64_machine_destroy(struct nvmm_machine *mach)
{
	struct pmap *pm;

	pm = mach->vm->vm_map.pmap;

	nvmm_aarch64_maintain_ipa(pm->pm_nvmm,
	    NVMM_AARCH64_MAINTAIN_OP_TLBI_ALL, 0, 0);
	pm->pm_nvmm = NULL;

	kmem_free(mach->machdata, sizeof(struct aarch64_machdata));
}

static int
nvmm_aarch64_machine_configure(struct nvmm_machine *mach, uint64_t op,
    void *data)
{
	return 0;
}

static int
nvmm_aarch64_vcpu_create(struct nvmm_machine *mach, struct nvmm_cpu *vcpu)
{
	struct aarch64_machdata *machdata = mach->machdata;
	struct aarch64_cpudata *cpudata;

	/*
	 * XXX: struct aarch64_cpudata is accessed by physical address from EL2,
	 *      and must fit on a single page.
	 */
	CTASSERT(sizeof(*cpudata) < PAGE_SIZE);

	cpudata = (struct aarch64_cpudata *)uvm_km_alloc(kernel_map,
	    roundup(sizeof(*cpudata), PAGE_SIZE), PAGE_SIZE,
	    UVM_KMF_WIRED | UVM_KMF_ZERO);
	vcpu->cpudata = cpudata;

	machdata->vttbr_el2 = cpudata->vttbr_el2 =
	    __SHIFTIN(AARCH64_VMID(mach), VTTBR_VIMD) |
	    __SHIFTIN(mach->vm->vm_map.pmap->pm_st2_table_pa, VTTBR_BADDR);

	if (!pmap_extract(pmap_kernel(), (vaddr_t)cpudata, &cpudata->cpudata_pa))
		panic("cannot resolve PA of cpudata");


	/* Install the RESET state. */
	memcpy(&vcpu->comm->state, &nvmm_aarch64_reset_state,
	    sizeof(vcpu->comm->state));

	/* The default value of MIDR_EL1 is the same as the host */
	vcpu->comm->state.sprs[NVMM_AARCH64_SPR_MIDR_EL1] = reg_midr_el1_read();
	vcpu->comm->state.sprs[NVMM_AARCH64_SPR_MPIDR_EL1] =
	    MPIDR_RES1 | vcpu->cpuid;

	vcpu->comm->state_wanted = NVMM_AARCH64_STATE_ALL;
	vcpu->comm->state_cached = 0;
	nvmm_aarch64_vcpu_setstate(vcpu);

	return 0;
}

static void
nvmm_aarch64_vcpu_destroy(struct nvmm_machine *mach, struct nvmm_cpu *vcpu)
{
	struct aarch64_cpudata *cpudata = vcpu->cpudata;

	uvm_km_free(kernel_map, (vaddr_t)cpudata,
	    roundup(sizeof(*cpudata), PAGE_SIZE), UVM_KMF_WIRED);
}

static int
nvmm_aarch64_vcpu_configure(struct nvmm_cpu *vcpu, uint64_t op, void *data)
{
	return 0;
}

static void
nvmm_aarch64_vcpu_setstate(struct nvmm_cpu *vcpu)
{
	struct nvmm_comm_page *comm = vcpu->comm;
	const struct nvmm_aarch64_state *state = &comm->state;
	struct aarch64_cpudata *cpudata = vcpu->cpudata;
	uint64_t flags;
	bool force_override_spsr = false;

	flags = comm->state_wanted;

	/* sanity check */
	if (flags & NVMM_AARCH64_STATE_SPRS) {
		switch (__SHIFTOUT(state->sprs[NVMM_AARCH64_SPR_SPSR_EL1], SPSR_M)) {
		case SPSR_M_EL1H:
		case SPSR_M_EL1T:
		case SPSR_M_EL0T:
			break;
		case SPSR_M_SYS32:
		case SPSR_M_UND32:
		case SPSR_M_ABT32:
		case SPSR_M_SVC32:
		case SPSR_M_IRQ32:
		case SPSR_M_FIQ32:
		case SPSR_M_USR32:
			/* XXX: TODO: check if CPU core supports aarch32 */
			break;
		default:
			/* XXX: insufficiency API... */
			//errno = EINVAL;
			//return -1;
			force_override_spsr = true;
			break;
		}

		/* XXX: TODO: check if CPU core supports specified endian */

	}

	if (flags & NVMM_AARCH64_STATE_GPRS) {
		memcpy(cpudata->guest.gprs, state->gprs, sizeof(state->gprs));
	}
	if (flags & NVMM_AARCH64_STATE_SPRS) {
		memcpy(cpudata->guest.sprs, state->sprs, sizeof(state->sprs));
		if (force_override_spsr) {
			cpudata->guest.sprs[NVMM_AARCH64_SPR_SPSR_EL1] &= ~SPSR_M;
			cpudata->guest.sprs[NVMM_AARCH64_SPR_SPSR_EL1] |= SPSR_M_EL1H;
		}
	}
	if (flags & NVMM_AARCH64_STATE_TIDS) {
		memcpy(cpudata->guest.tids, state->tids, sizeof(state->tids));
	}
	if (flags & NVMM_AARCH64_STATE_FPRS) {
		memcpy(cpudata->guest.fprs, state->fprs, sizeof(state->fprs));
	}

	comm->state_wanted = 0;
	comm->state_cached |= flags;
}

static void
nvmm_aarch64_vcpu_getstate(struct nvmm_cpu *vcpu)
{
	struct nvmm_comm_page *comm = vcpu->comm;
	struct nvmm_aarch64_state *state = &comm->state;
	const struct aarch64_cpudata *cpudata = vcpu->cpudata;
	uint64_t flags;

	flags = comm->state_wanted;

	if (flags & NVMM_AARCH64_STATE_GPRS) {
		memcpy(state->gprs, cpudata->guest.gprs, sizeof(state->gprs));
	}
	if (flags & NVMM_AARCH64_STATE_SPRS) {
		memcpy(state->sprs, cpudata->guest.sprs, sizeof(state->sprs));
	}
	if (flags & NVMM_AARCH64_STATE_TIDS) {
		memcpy(state->tids, cpudata->guest.tids, sizeof(state->tids));
	}
	if (flags & NVMM_AARCH64_STATE_FPRS) {
		memcpy(state->fprs, cpudata->guest.fprs, sizeof(state->fprs));
	}

	comm->state_wanted = 0;
	comm->state_cached |= flags;
}

//static void
//aarch64_vcpu_state_provide(struct nvmm_cpu *vcpu, uint64_t flags)
//{
//	vcpu->comm->state_wanted = flags;
//	nvmm_aarch64_vcpu_getstate(vcpu);
//}

static void
aarch64_vcpu_state_commit(struct nvmm_cpu *vcpu)
{
	vcpu->comm->state_wanted = vcpu->comm->state_commit;
	vcpu->comm->state_commit = 0;
	nvmm_aarch64_vcpu_setstate(vcpu);
}

static int
nvmm_aarch64_vcpu_inject(struct nvmm_cpu *vcpu)
{
	printf("%s:%d\n", __func__, __LINE__);
	return 0;
}

static int
nvmm_aarch64_vcpu_run(struct nvmm_machine *mach, struct nvmm_cpu *vcpu,
    struct nvmm_vcpu_exit *exit)
{
	struct aarch64_cpudata *cpudata = vcpu->cpudata;

	aarch64_vcpu_state_commit(vcpu);
	vcpu->comm->state_cached = 0;

	/* event commit */
	if (__predict_false(vcpu->comm->event_commit)) {
		vcpu->comm->event_commit = false;
		cpudata->send_event_type = vcpu->comm->event.type;
//		if (nvmm_debug > 0) {
			printf("%s:%s:%d: send event: %u\n", cpu_name(curcpu()), __func__, __LINE__, vcpu->comm->event.type);
//		}
	} else {
		cpudata->send_event_type = 0;
	}

#if 0
	// XXX avoid this by pinning threads to pCPUs?
	struct cpu_info *ci = curcpu();
	int hcpu = cpu_number();

	if (vcpu->hcpu_last != hcpu) {
		vmx_vmwrite(VMCS_HOST_TR_SELECTOR, ci->ci_tss_sel);
		vmx_vmwrite(VMCS_HOST_TR_BASE, (uint64_t)ci->ci_tss);
		vmx_vmwrite(VMCS_HOST_GDTR_BASE, (uint64_t)ci->ci_gdt);
		vmx_vmwrite(VMCS_HOST_GS_BASE, rdmsr(MSR_GSBASE));
		cpudata->gtsc_want_update = true;
		vcpu->hcpu_last = hcpu;
	}
#endif

	// INVALID

	kpreempt_disable();

	while (true) {

		aarch64_dcache_wb_all();	// XXX: currently, it is unstable without this...
		aarch64_hvc_vmenter(cpudata->cpudata_pa);

// XXXNH exit to here?

		if (nvmm_return_needed(vcpu, exit)) {
			break;
		}

		if (exit->reason != NVMM_VCPU_EXIT_NONE) {
			break;
		}
	}

	kpreempt_enable();

	memcpy(exit, &cpudata->exit, sizeof(*exit));

	return 0;
}

void
nvmm_aarch64_maintain_ipa(void *nvmm_mach, uint64_t op, uint64_t ipa, uint64_t va)
{
	if (nvmm_mach == NULL)
		return;

	struct nvmm_machine *mach = nvmm_mach;
	struct aarch64_machdata *machdata = mach->machdata;

	if (machdata != NULL) {
		if (nvmm_debug & 4) {
			printf("%s:%s:%d: op=%08lx, ipa=%016lx, va=%016lx\n", cpu_name(curcpu()), __func__, __LINE__, op, ipa, va);
		}
		aarch64_hvc_maintain_ipa(machdata->vttbr_el2, op, ipa, va);
	}
}

const struct nvmm_impl nvmm_aarch64 = {
	.name = "aarch64",
	.ident = nvmm_aarch64_ident,
	.init = nvmm_aarch64_init,
	.fini = nvmm_aarch64_fini,
	.capability = nvmm_aarch64_capability,
	.mach_conf_max = 0,
	.mach_conf_sizes = NULL,
	.vcpu_conf_max = 0,
	.vcpu_conf_sizes = NULL,
	.state_size = sizeof(struct nvmm_aarch64_state),
	.machine_create = nvmm_aarch64_machine_create,
	.machine_destroy = nvmm_aarch64_machine_destroy,
	.machine_configure = nvmm_aarch64_machine_configure,
	.vcpu_create = nvmm_aarch64_vcpu_create,
	.vcpu_destroy = nvmm_aarch64_vcpu_destroy,
	.vcpu_configure = nvmm_aarch64_vcpu_configure,
	.vcpu_setstate = nvmm_aarch64_vcpu_setstate,
	.vcpu_getstate = nvmm_aarch64_vcpu_getstate,
	.vcpu_inject = nvmm_aarch64_vcpu_inject,
	.vcpu_run = nvmm_aarch64_vcpu_run
};
