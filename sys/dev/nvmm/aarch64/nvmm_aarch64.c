/*	$NetBSD$	*/

/*-
 * Copyright (c) 2022 Ryo Shimizu <ryo@nerv.org>
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
#include <sys/systm.h>
#include <sys/kmem.h>
#include <sys/xcall.h>

#include <uvm/uvm.h>

#include <dev/nvmm/nvmm.h>
#include <dev/nvmm/nvmm_internal.h>
#include <dev/nvmm/aarch64/nvmm_aarch64.h>

#include <machine/bootconfig.h>

#include <aarch64/cpufunc.h>
#include <aarch64/pmap.h>

struct aarch64_machdata {
	void *unused1;
};

void aarch64_hvc_init(paddr_t);
void aarch64_hvc_vmenter(paddr_t);
static void nvmm_aarch64_vcpu_setstate(struct nvmm_cpu *vcpu);

const struct nvmm_aarch64_state nvmm_aarch64_reset_state = {
	.gprs = {
		0
	},
	.sprs = {
		[NVMM_AARCH64_SPR_PC]			= 0,
		[NVMM_AARCH64_SPR_SP_ELx]		= 0,
		[NVMM_AARCH64_SPR_TPIDRRO_EL0]		= 0,
		[NVMM_AARCH64_SPR_TPIDR_EL0]		= 0,
		[NVMM_AARCH64_SPR_AMAIR_EL1]		= 0,
		[NVMM_AARCH64_SPR_CNTKCTL_EL1]		= 0,
		[NVMM_AARCH64_SPR_CONTEXTIDR_EL1]	= 0,
		[NVMM_AARCH64_SPR_CPACR_EL1]		= 0,
		[NVMM_AARCH64_SPR_CSSELR_EL1]		= 0,
		[NVMM_AARCH64_SPR_ELR_EL1]		= 0,
		[NVMM_AARCH64_SPR_ESR_EL1]		= 0,
		[NVMM_AARCH64_SPR_FAR_EL1]		= 0,
		[NVMM_AARCH64_SPR_MAIR_EL1]		= 0,
		[NVMM_AARCH64_SPR_MDSCR_EL1]		= 0,
		[NVMM_AARCH64_SPR_PAR_EL1]		= 0,
		[NVMM_AARCH64_SPR_SCTLR_EL1]		= SCTLR_RES1,
		[NVMM_AARCH64_SPR_SPSR_EL1]		= SPSR_M_EL1H,
		[NVMM_AARCH64_SPR_SP_EL1]		= 0,
		[NVMM_AARCH64_SPR_TCR_EL1]		= 0,
		[NVMM_AARCH64_SPR_TPIDR_EL1]		= 0,
		[NVMM_AARCH64_SPR_TTBR0_EL1]		= 0,
		[NVMM_AARCH64_SPR_TTBR1_EL1]		= 0,
		[NVMM_AARCH64_SPR_VBAR_EL1]		= 0,
	},
	.fprs = {
		0
	}
};

static void __unused
debugdump_state(struct nvmm_aarch64_state *state)
{
	printf("[state=%p]\n", state);
	printf("    x0=%016lx,     x1=%016lx\n", state->gprs[0], state->gprs[1]);
	printf("    x2=%016lx,     x3=%016lx\n", state->gprs[2], state->gprs[3]);
	printf("    x4=%016lx,     x5=%016lx\n", state->gprs[4], state->gprs[5]);
	printf("    x6=%016lx,     x7=%016lx\n", state->gprs[6], state->gprs[7]);
	printf("    x8=%016lx,     x9=%016lx\n", state->gprs[8], state->gprs[9]);
	printf("   x10=%016lx,    x11=%016lx\n", state->gprs[10], state->gprs[11]);
	printf("   x12=%016lx,    x13=%016lx\n", state->gprs[12], state->gprs[13]);
	printf("   x14=%016lx,    x15=%016lx\n", state->gprs[14], state->gprs[15]);
	printf("   x16=%016lx,    x17=%016lx\n", state->gprs[16], state->gprs[17]);
	printf("   x18=%016lx,    x19=%016lx\n", state->gprs[18], state->gprs[19]);
	printf("   x20=%016lx,    x21=%016lx\n", state->gprs[20], state->gprs[21]);
	printf("   x22=%016lx,    x23=%016lx\n", state->gprs[22], state->gprs[23]);
	printf("   x24=%016lx,    x25=%016lx\n", state->gprs[24], state->gprs[25]);
	printf("   x26=%016lx,    x27=%016lx\n", state->gprs[26], state->gprs[27]);
	printf("   x28=%016lx, fp=x29=%016lx\n", state->gprs[28], state->gprs[29]);
	printf("lr=x30=%016lx,     sp=%016lx\n", state->gprs[30], state->gprs[31]);

	printf("    PC=%016lx\n", state->sprs[NVMM_AARCH64_SPR_PC]);
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
nvmm_aarch64_init(void)
{
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
	    LX_S1_BLKPAG_AP_RW | LX_S1_BLKPAG_XN | LX_S1_BLKPAG_AP1_RES1;
	pmapboot_enter_ttbr(CONSADDR, CONSADDR, L2_SIZE, L2_SIZE,
	    devattr, PRFUNC, ttbr_pa, true, nvmm_aarch64_pagealloc);
#endif
 
	/* EL2 VA=PA identity mapping */
	const pt_entry_t memattr = LX_BLKPAG_ATTR_NORMAL_WB |
	    LX_S1_BLKPAG_AP_RW | LX_S1_BLKPAG_AP1_RES1;
	for (u_int blk = 0; blk < bootconfig.dramblocks; blk++) {
		uint64_t start, end;

		start = trunc_page(bootconfig.dram[blk].address);
		end = round_page(bootconfig.dram[blk].address +
		(uint64_t)bootconfig.dram[blk].pages * PAGE_SIZE);

		pmapboot_enter_range_ttbr(start, start, end - start,
		    memattr, PRFUNC, ttbr_pa, true, nvmm_aarch64_pagealloc);
	}

	/*
	 * Once the EL2 MMU is enabled, it is never disabled again.
	 * VA=PA identity mapping is enabled until reboot.
	 * Allocated page tables are not released by calling nvmm_aarch64_fini().
	 */

	/* calll aarch64_hvc_init(ttbr_pa) on all cpus */
	uint64_t where = xc_broadcast(0, (xcfunc_t)aarch64_hvc_init,
	    (void *)ttbr_pa, NULL);
	xc_wait(where);
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

static void
nvmm_aarch64_machine_create(struct nvmm_machine *mach)
{
	struct aarch64_machdata *machdata;
	struct pmap *pm;
	struct pglist pglist;
	size_t concat_tablesize;
	int error;

	pm = mach->vm->vm_map.pmap;

	/* set aarch64 pmap to stage2 mode */
	pm->pm_stage2 = true;


	/*
	 * allocate concatenated translation table. must be n-page aligned.
	 */

	/* The code assumes a 4kbyte page */
	CTASSERT(PAGE_SIZE == L3_SIZE);

	/* XXXXXXXXX: TODO: calculated from ID_AA64MMFR1_EL1.PARANGE */
	pm->pm_startlevel = 1;
	pm->pm_concatenate_num = 2;

	concat_tablesize = L3_SIZE * pm->pm_concatenate_num;
	error = uvm_pglistalloc(concat_tablesize, 0, ~0UL,
	    concat_tablesize, 0, &pglist, 1, 1);
	KASSERTMSG(error == 0, "cannot allocate concatenated page");

	pm->pm_starttable_pa = VM_PAGE_TO_PHYS(TAILQ_FIRST(&pglist));
	pm->pm_starttable = (pd_entry_t *)AARCH64_PA_TO_KVA(pm->pm_starttable_pa);

	KASSERT((pm->pm_starttable_pa & (concat_tablesize - 1)) == 0);
	memset(pm->pm_starttable, 0, concat_tablesize);

	for (int i = 0; i < pm->pm_concatenate_num; i++) {
		/*
		 * create L0->L1 table entry for pmap.
		 * The hardware MMU starts the lookup from the L1 table.
		 */
		pm->pm_l0table[i] = (pm->pm_starttable_pa + i * PAGE_SIZE) |
		    LX_TYPE_TBL | LX_VALID | LX_BLKPAG_OS_STAGE2;

		/*
		 * ...and these pages must be pmap_append_pdp() so that they
		 * are uvm_pagefree()'d by pmap_free_pdp() upon pmap_destroy.
		 * This is somewhat confusing code...
		 */
		pmap_append_pdp(pm, pm->pm_starttable_pa + i * PAGE_SIZE);
	}

	printf("%s:%d: pmap pm=%p, startlevel=%d, starttable=%p, starttable_pa=%016lx (%d concatenated)\n",
	     __func__, __LINE__,
	    mach->vm->vm_map.pmap,
	    pm->pm_startlevel,
	    mach->vm->vm_map.pmap->pm_starttable,
	    mach->vm->vm_map.pmap->pm_starttable_pa,
	    pm->pm_concatenate_num);


	machdata = kmem_zalloc(sizeof(struct aarch64_machdata), KM_SLEEP);
	mach->machdata = machdata;
}

static void
nvmm_aarch64_machine_destroy(struct nvmm_machine *mach)
{
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
	struct aarch64_cpudata *cpudata;

	cpudata = (struct aarch64_cpudata *)uvm_km_alloc(kernel_map,
	    roundup(sizeof(*cpudata), PAGE_SIZE), 0,
	    UVM_KMF_WIRED | UVM_KMF_ZERO);
	vcpu->cpudata = cpudata;

	/* Install the RESET state. */
	memcpy(&vcpu->comm->state, &nvmm_aarch64_reset_state,
	    sizeof(vcpu->comm->state));
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

	flags = comm->state_wanted;

	if (flags & NVMM_AARCH64_STATE_GPRS) {
		memcpy(cpudata->guest.gprs, state->gprs, sizeof(state->gprs));
	}
	if (flags & NVMM_AARCH64_STATE_SPRS) {
		memcpy(cpudata->guest.sprs, state->sprs, sizeof(state->sprs));
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
	struct nvmm_comm_page *comm = vcpu->comm;
	struct aarch64_cpudata *cpudata __unused = vcpu->cpudata;
//	struct aarch64_machdata *machdata = mach->machdata;

	printf("%s:%d\n", __func__, __LINE__);

	aarch64_vcpu_state_commit(vcpu);
	comm->state_cached = 0;


	//XXXX
//	debugdump_state(&cpudata->guest);

	/* XXX: assert NVMM_MAX_MACHINES < (1 << ID_AA64MMFR1.VMIDBITS) */
	cpudata->vttbr_el2 =
	    __SHIFTIN(mach->machid + 10, VTTBR_VIMD) |
	    __SHIFTIN(mach->vm->vm_map.pmap->pm_starttable_pa, VTTBR_BADDR);

	//event commit

	kpreempt_disable();


	vaddr_t cpudata_va = (vaddr_t)cpudata;
	vaddr_t exit_va = (vaddr_t)exit;
	paddr_t cpudata_pa;
	paddr_t exit_pa;
	if (!pmap_extract(pmap_kernel(), cpudata_va, &cpudata_pa))
		panic("cannot resolve PA of cpudata");
	if (!pmap_extract(pmap_kernel(), exit_va, &exit_pa))
		panic("cannot resolve PA of exit");
	cpudata->exit_pa = exit_pa;

	aarch64_dcache_wb_all();
	aarch64_hvc_vmenter(cpudata_pa);

	kpreempt_enable();

	return 0;
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
