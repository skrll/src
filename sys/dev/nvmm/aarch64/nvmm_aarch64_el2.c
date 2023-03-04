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

#include "opt_cpuoptions.h"
#include "opt_multiprocessor.h"

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kmem.h>

#include <uvm/uvm_extern.h>
#include <uvm/uvm_page.h>

#include <dev/nvmm/nvmm.h>
#include <dev/nvmm/nvmm_internal.h>
#include <dev/nvmm/aarch64/nvmm_aarch64.h>

#include <aarch64/cpufunc.h>

int uartprintf(const char * restrict, ...) __printflike(1, 2);
void dump_el2_trapframe(struct trapframe *tf);
void aarch64_el2_mmu_enable(void);
void aarch64_el2_init(struct trapframe *);
void aarch64_el2_vmenter(struct trapframe *);
void aarch64_el2_vmexit_trap(struct trapframe *);
void aarch64_el2_vmexit_irq(struct trapframe *);

void
aarch64_el2_init(struct trapframe *tf)
{
	static int aarch64_el2_initted = 0;

	if (aarch64_el2_initted != 0)
		return;
	aarch64_el2_initted = 1;

	/*
	 * Setup EL0/1 stage2 translate configuration. The activation and
	 * deactivation of stage2 translation itself is done in VMENTER/VMEXIT.
	 * But VTCR_EL2 is a fixed value, so it should be set here.
	 */
	const uint64_t mmfr0_parange = __SHIFTOUT(reg_id_aa64mmfr0_el1_read(),
	    ID_AA64MMFR0_EL1_PARANGE);
	int parange = aarch64_parange();
	int startlevel = 0;	/* starting level of translation lookup table */
	uint64_t vtcr_options = 0;

	/*
	 * 12: bitwidth of page (4Kpage)
	 *  9: bitwidth of PTE entries per page (4k/sizeof(pte) = 512)
	 *  4: maximum number of concatenated TTBR (16)
	 */
	if (parange <= (12 + 9 + 9 + 4))		/* PArange <= 34bit */
		startlevel = 2;
	else if (parange <= (12 + 9 + 9 + 9 + 4))	/* PArange <= 43bit */
		startlevel = 1;
	else
		startlevel = 0;

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
	if (__SHIFTOUT(ID_AA64PFR0_EL1_SEL2, reg_id_aa64pfr0_el1_read()) !=
	    ID_AA64PFR0_EL1_SEL2_NONE) {
		vtcr_options |= VTCR_EL2_NSA;
		vtcr_options |= VTCR_EL2_NSW;
	}
	reg_vtcr_el2_write(
	    __BIT(31) |				/* RES1 */
	    vtcr_options |
	    __SHIFTIN(mmfr0_parange, VTCR_EL2_PS) |
	    __SHIFTIN(0, VTCR_EL2_TG0) |	/* 4k page */
#ifdef MULTIPROCESSOR
	    __SHIFTIN(3, VTCR_EL2_SH0) |	/* Inner Shareable */
#else
	    __SHIFTIN(0, VTCR_EL2_SH0) |	/* Non-Shareable */
#endif
	    __SHIFTIN(1, VTCR_EL2_ORGN0) |	/* WB WA */
	    __SHIFTIN(1, VTCR_EL2_IRGN0) |	/* WB WA */
	    __SHIFTIN(2 - startlevel, VTCR_EL2_SL0) |
	    __SHIFTIN(64 - parange, VTCR_EL2_T0SZ)
	);


	/*
	 * Enable *EL2* MMU (VA=PA).
	 * Not a stage2 translation for EL0/1.
	 *
	 * If EL2 is not run with MMU enable, cache does not work on EL2
	 * or exclusive load/store cannot be used.
	 *
	 * if the MMU is already enabled, do nothing
	 */
	if (reg_sctlr_el2_read() & SCTLR_M)
		return;

#define VIRT_BIT	48
	/* set TCR_EL2 */
	uint64_t tcr_el2 =
	    __BIT(31) | __BIT(23) |	/* RES1 */
	    __SHIFTIN(mmfr0_parange, TCR_EL2_PS) |
	    TCR_EL2_TG0_4KB |
#ifdef MULTIPROCESSOR
	    TCR_EL2_SH0_INNER |
#else
	    TCR_EL2_SH0_NONE |
#endif
	    TCR_EL2_ORGN0_WB_WA |
	    TCR_EL2_IRGN0_WB_WA |
	    __SHIFTIN(64 - VIRT_BIT, TCR_EL2_T0SZ);
	reg_tcr_el2_write(tcr_el2);

	/* MAIR_EL2 = MAIR_EL1 */
	reg_mair_el2_write(reg_mair_el1_read());

	/* x0 = L0 table of PA */
	reg_ttbr0_el2_write(tf->tf_reg[0]);
	aarch64_el2_mmu_enable();
}

static void
vcpu_context_save(struct trapframe *tf, struct nvmm_aarch64_state *state)
{
	state->sprs[NVMM_AARCH64_SPR_PC] = tf->tf_pc;
	state->sprs[NVMM_AARCH64_SPR_SPSR_EL1] = tf->tf_spsr;

	/* XXX: TODO: SP0 and SP1 must be distinguished */
	memcpy(state->gprs, tf->tf_reg, sizeof(state->gprs));

	state->sprs[NVMM_AARCH64_SPR_TPIDRRO_EL0] = reg_tpidrro_el0_read();
	state->sprs[NVMM_AARCH64_SPR_TPIDR_EL0] = reg_tpidr_el0_read();
	state->sprs[NVMM_AARCH64_SPR_AMAIR_EL1] = reg_amair_el1_read();
	state->sprs[NVMM_AARCH64_SPR_CNTKCTL_EL1] = reg_cntkctl_el1_read();
	state->sprs[NVMM_AARCH64_SPR_CONTEXTIDR_EL1] = reg_contextidr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_CPACR_EL1] = reg_cpacr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_CSSELR_EL1] = reg_csselr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_ELR_EL1] = reg_elr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_ESR_EL1] = reg_esr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_FAR_EL1] = reg_far_el1_read();
	state->sprs[NVMM_AARCH64_SPR_MAIR_EL1] = reg_mair_el1_read();
	state->sprs[NVMM_AARCH64_SPR_MDSCR_EL1] = reg_mdscr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_PAR_EL1] = reg_par_el1_read();
	state->sprs[NVMM_AARCH64_SPR_SCTLR_EL1] = reg_sctlr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_SP_EL1] = reg_sp_el1_read();
	state->sprs[NVMM_AARCH64_SPR_TCR_EL1] = reg_tcr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_TPIDR_EL1] = reg_tpidr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_TTBR0_EL1] = reg_ttbr0_el1_read();
	state->sprs[NVMM_AARCH64_SPR_TTBR1_EL1] = reg_ttbr1_el1_read();
	state->sprs[NVMM_AARCH64_SPR_VBAR_EL1] = reg_vbar_el1_read();
}

static void
vcpu_context_load(struct trapframe *tf, const struct nvmm_aarch64_state *state)
{
	tf->tf_pc = state->sprs[NVMM_AARCH64_SPR_PC];
	tf->tf_spsr = state->sprs[NVMM_AARCH64_SPR_SPSR_EL1];

	/* XXX: TODO: SP0 and SP1 must be distinguished */
	memcpy(tf->tf_reg, state->gprs, sizeof(state->gprs));

	reg_tpidrro_el0_write(state->sprs[NVMM_AARCH64_SPR_TPIDRRO_EL0]);
	reg_tpidr_el0_write(state->sprs[NVMM_AARCH64_SPR_TPIDR_EL0]);
	reg_amair_el1_write(state->sprs[NVMM_AARCH64_SPR_AMAIR_EL1]);
	reg_cntkctl_el1_write(state->sprs[NVMM_AARCH64_SPR_CNTKCTL_EL1]);
	reg_contextidr_el1_write(state->sprs[NVMM_AARCH64_SPR_CONTEXTIDR_EL1]);
	reg_cpacr_el1_write(state->sprs[NVMM_AARCH64_SPR_CPACR_EL1]);
	reg_csselr_el1_write(state->sprs[NVMM_AARCH64_SPR_CSSELR_EL1]);
	reg_elr_el1_write(state->sprs[NVMM_AARCH64_SPR_ELR_EL1]);
	reg_esr_el1_write(state->sprs[NVMM_AARCH64_SPR_ESR_EL1]);
	reg_far_el1_write(state->sprs[NVMM_AARCH64_SPR_FAR_EL1]);
	reg_mair_el1_write(state->sprs[NVMM_AARCH64_SPR_MAIR_EL1]);
	reg_mdscr_el1_write(state->sprs[NVMM_AARCH64_SPR_MDSCR_EL1]);
	reg_par_el1_write(state->sprs[NVMM_AARCH64_SPR_PAR_EL1]);
	reg_sctlr_el1_write(state->sprs[NVMM_AARCH64_SPR_SCTLR_EL1]);
	reg_sp_el1_write(state->sprs[NVMM_AARCH64_SPR_SP_EL1]);
	reg_tcr_el1_write(state->sprs[NVMM_AARCH64_SPR_TCR_EL1]);
	reg_tpidr_el1_write(state->sprs[NVMM_AARCH64_SPR_TPIDR_EL1]);
	reg_ttbr0_el1_write(state->sprs[NVMM_AARCH64_SPR_TTBR0_EL1]);
	reg_ttbr1_el1_write(state->sprs[NVMM_AARCH64_SPR_TTBR1_EL1]);
	reg_vbar_el1_write(state->sprs[NVMM_AARCH64_SPR_VBAR_EL1]);
}

static void
aarch64_vmexit_context(struct trapframe *tf, struct aarch64_cpudata *cpudata)
{
	reg_hcr_el2_write(HCR_EL2_RW);
	reg_hstr_el2_write(0);
	reg_vttbr_el2_write(0);

	/* save guest state */
	vcpu_context_save(tf, &cpudata->guest);
	/* load host state */
	vcpu_context_load(tf, &cpudata->host);
}

void
aarch64_el2_vmexit_irq(struct trapframe *tf)
{
	struct aarch64_cpudata *cpudata_pa;
	struct nvmm_aarch64_exit *exit_pa;

	cpudata_pa = (struct aarch64_cpudata *)reg_tpidr_el2_read();
	exit_pa = (struct nvmm_aarch64_exit *)cpudata_pa->exit_pa;
	exit_pa->reason = NVMM_VCPU_EXIT_NONE;

	aarch64_vmexit_context(tf, cpudata_pa);
	reg_tpidr_el2_write(0);
}

void
aarch64_el2_vmexit_trap(struct trapframe *tf)
{
	struct aarch64_cpudata *cpudata_pa;
	struct nvmm_aarch64_exit *exit_pa;
	const uint64_t esr = tf->tf_esr;
	const uint64_t eclass = __SHIFTOUT(esr, ESR_EC);
	uint32_t rw;
	vm_prot_t ftype;

	rw = __SHIFTOUT(esr, ESR_ISS_DATAABORT_WnR); /* 0 if IFSC */

	cpudata_pa = (struct aarch64_cpudata *)reg_tpidr_el2_read();
	exit_pa = (struct nvmm_aarch64_exit *)cpudata_pa->exit_pa;

	switch (eclass) {
	case ESR_EC_INSN_ABT_EL_LOW:
	case ESR_EC_DATA_ABT_EL_LOW:
		if (eclass == ESR_EC_INSN_ABT_EL_LOW)
			ftype = VM_PROT_EXECUTE;
		else if (__SHIFTOUT(esr, ESR_ISS_DATAABORT_CM))
			ftype = VM_PROT_READ;
		else
			ftype = (rw == 0) ? VM_PROT_READ : VM_PROT_WRITE;

		/* XXX: distinguish cache op. ftype is incomplete */

		exit_pa->reason = NVMM_VCPU_EXIT_MEMORY;
		exit_pa->u.mem.gpa = tf->tf_far;
		exit_pa->u.mem.prot = ftype;
		break;

	case ESR_EC_INSN_ABT_EL_CUR:
	case ESR_EC_DATA_ABT_EL_CUR:
		uartprintf("%s: INSN or DATA ABORT occured on EL2?\n", __func__);
		//panic. never occur data abort on EL2...?
	case ESR_EC_UNKNOWN:
	case ESR_EC_SERROR:
	case ESR_EC_WFX:
	case ESR_EC_ILL_STATE:
	case ESR_EC_BTE_A64:
	case ESR_EC_SYS_REG:
	case ESR_EC_SVC_A64:
	case ESR_EC_HVC_A64:
	case ESR_EC_SMC_A64:
	case ESR_EC_PC_ALIGNMENT:
	case ESR_EC_SP_ALIGNMENT:
	case ESR_EC_FP_ACCESS:
	case ESR_EC_FP_TRAP_A64:
	case ESR_EC_BRKPNT_EL0:
	case ESR_EC_BRKPNT_EL1:
	case ESR_EC_SW_STEP_EL0:
	case ESR_EC_SW_STEP_EL1:
	case ESR_EC_WTCHPNT_EL0:
	case ESR_EC_WTCHPNT_EL1:
	case ESR_EC_BKPT_INSN_A64:
		exit_pa->reason = NVMM_VCPU_EXIT_HALTED;
//		exit_pa->u.mem
//		exit_pa->u.insn
//		exit_pa->u.inv
		break;
	}

	aarch64_vmexit_context(tf, cpudata_pa);
	reg_tpidr_el2_write(0);
}

void
aarch64_el2_vmenter(struct trapframe *tf)
{
	struct aarch64_cpudata *cpudata_pa;

	cpudata_pa = (struct aarch64_cpudata *)tf->tf_reg[0];

	if (reg_tpidr_el2_read() != 0)
		uartprintf("panic: %s: tpidr_el2 is not zero: %016lx\n", __func__, reg_tpidr_el2_read());

	reg_tpidr_el2_write((register_t)cpudata_pa);
	uartprintf("VMENTER: PC=%016lx\n", cpudata_pa->guest.sprs[NVMM_AARCH64_SPR_PC]);

	/* save host state */
	vcpu_context_save(tf, &cpudata_pa->host);
	/* load guest state */
	vcpu_context_load(tf, &cpudata_pa->guest);

	uint64_t hcr = HCR_EL2_RW;	/* 64bit */
//	hcr |= HCR_EL2_ID;
//	hcr |= HCR_EL2_CD;
//	hcr |= HCR_EL2_TRVM;
//	hcr |= HCR_EL2_HCD;
//	hcr |= HCR_EL2_TDZ;
//	hcr |= HCR_EL2_TGE;
//	hcr |= HCR_EL2_TVM;
//	hcr |= HCR_EL2_TTLB;
//	hcr |= HCR_EL2_TPU;
//	hcr |= HCR_EL2_TPC;
//	hcr |= HCR_EL2_TSW;
//	hcr |= HCR_EL2_TACR;
//	hcr |= HCR_EL2_TIDCP;
	hcr |= HCR_EL2_TSC;
//	hcr |= HCR_EL2_TID3;
//	hcr |= HCR_EL2_TID2;
//	hcr |= HCR_EL2_TID1;
//	hcr |= HCR_EL2_TID0;
	hcr |= HCR_EL2_TWE;
	hcr |= HCR_EL2_TWI;
//	hcr |= HCR_EL2_DC;
//	hcr |= HCR_EL2_BSU;
//	hcr |= HCR_EL2_FB;
//	hcr |= HCR_EL2_VSE;
//	hcr |= HCR_EL2_VI;
//	hcr |= HCR_EL2_VF;
	hcr |= HCR_EL2_AMO;
	hcr |= HCR_EL2_IMO;
	hcr |= HCR_EL2_FMO;
//	hcr |= HCR_EL2_PTW;
//	hcr |= HCR_EL2_SWIO;
	hcr |= HCR_EL2_VM;

	reg_vttbr_el2_write(cpudata_pa->vttbr_el2);
	reg_hstr_el2_write(0xffff);
	reg_hcr_el2_write(hcr);

	/* XXX */
	asm("dsb ishst");
	asm("ic ialluis");
	asm("tlbi vmalle1is");
	asm("tlbi vmalls12e1is");
	asm("dsb ish");
	asm("isb");
}
