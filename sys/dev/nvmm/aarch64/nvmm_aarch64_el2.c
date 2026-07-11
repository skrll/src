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

#define KERNHIST_PTRKTOX(k)		nvmm_el2_pointer_ktox(k)
#define KERNHIST_PTRXTOK(x)		nvmm_el2_pointer_xtok(x)
#define KERNHIST_CPUNUMBER		aarch64nvmm_cpu_number()
#define KERNHIST_BINTIME_ALLOWED	false

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kmem.h>

#include <uvm/uvm_extern.h>
#include <uvm/uvm_page.h>

#include <dev/nvmm/nvmm.h>
#include <dev/nvmm/nvmm_internal.h>
#include <dev/nvmm/aarch64/nvmm_aarch64.h>
#include <dev/nvmm/aarch64/nvmm_aarch64_internal.h>

#include <aarch64/machdep.h>

int aarch64_el2_initted;

#define IN_RANGE_P(addr,sta,end)	((sta) <= (addr) && (addr) < (end))

static inline void *
nvmm_el2_pointer_ktox(void *ptr)
{
	extern char __kernel_text[];
	extern char _end[];

	vaddr_t kernstart = KERN_PHYSTOV(trunc_page((vaddr_t)__kernel_text));
	vaddr_t kernend = KERN_PHYSTOV(round_page((vaddr_t)_end));
	const vaddr_t va = (vaddr_t)ptr;
	paddr_t pa = -1;

	if (IN_RANGE_P(va, kernstart, kernend)) {
		pa = KERN_VTOPHYS(va);
	} else if (IN_RANGE_P(va, AARCH64_DIRECTMAP_START, AARCH64_DIRECTMAP_END)) {
		pa = AARCH64_KVA_TO_PA(va);
	}

	return (void *)pa;
}

static inline void *
nvmm_el2_pointer_xtok(const void *ptr)
{
	extern char __kernel_text[];
	extern char _end[];

	const paddr_t kernstart = trunc_page((vaddr_t)__kernel_text);
	const paddr_t kernend = round_page((vaddr_t)_end);
	const paddr_t pa = (vaddr_t)ptr;
	vaddr_t va = -1;

	if (IN_RANGE_P(pa, kernstart, kernend)) {
		va = KERN_PHYSTOV(pa);
	}

	return (void *)va;
}

paddr_t
aarch64_gva_to_pa(uint64_t spsr, vaddr_t va)
{
	paddr_t pa = -1;

	if (__SHIFTOUT(spsr, SPSR_M) == SPSR_M_EL0T)
		reg_s12e0r_write(va);
	else
		reg_s12e1r_write(va);
	isb();
	uint64_t par = reg_par_el1_read();
	if ((par & PAR_F) == 0) {
		pa = (par & PAR_PA) + (va & PAR_PA_LOWMASK);
	}
	return pa;
}

paddr_t
aarch64_gva_to_ipa(uint64_t spsr, vaddr_t va)
{
	paddr_t ipa = -1;

	if (__SHIFTOUT(spsr, SPSR_M) == SPSR_M_EL0T)
		reg_s1e0r_write(va);
	else
		reg_s1e1r_write(va);
	isb();
	uint64_t par = reg_par_el1_read();
	if ((par & PAR_F) == 0) {
		ipa = (par & PAR_PA) + (va & PAR_PA_LOWMASK);
	}
	return ipa;
}

int has_cortex_a57_erratum_834220 = 0;	//XXXXXXXXX

paddr_t
aarch64_get_fault_ipa(struct trapframe *tf)
{
	paddr_t ipa;
	const uint64_t esr = tf->tf_esr;
	const uint64_t fsc = __SHIFTOUT(esr, ESR_ISS_DATAABORT_DFSC);

	/* confirm that these fields are common in DATAABORT and INSNABORT */
	CTASSERT(ESR_ISS_DATAABORT_DFSC  == ESR_ISS_INSNABORT_IFSC);
	CTASSERT(ESR_ISS_DATAABORT_S1PTW == ESR_ISS_INSNABORT_S1PTW);
#define FSC_PERM_FAULT(fsc)	\
	((fsc) >= ESR_ISS_FSC_PERM_FAULT_0 && (fsc) <= ESR_ISS_FSC_PERM_FAULT_3)

	if ((tf->tf_esr & ESR_ISS_DATAABORT_S1PTW) == 0 &&
	    (has_cortex_a57_erratum_834220 || FSC_PERM_FAULT(fsc))) {
		ipa = aarch64_gva_to_ipa(tf->tf_spsr, tf->tf_far);
		/*
		 * XXX:
		 * In this case, nvmm_aarch64_maintain_ipa() from pmap(9)
		 * -> aarch64_tlbi_by_vmid_ipa() doesn't work.
		 * Doing aarch64_tlbi_by_vmid_ipa(ipa) or
		 * aarch64_tlbi_by_vmid_ipa(ipa_hpfar_far(reg_hpfar_el2_read(), tf->tf_far))
		 * here does not work. Do TLBI-all here.
		 */
		aarch64_tlbi_by_vmid();
	} else {
		ipa = ipa_hpfar_far(reg_hpfar_el2_read(), tf->tf_far);
	}
	return ipa;
}

void
aarch64_el2_init(struct trapframe *tf)
{
	/*
	 * Run once (i.e. when aarch64_el2_initted == 0)
	 */
	if (aarch64_el2_initted != 0)
		return;

	struct nvmm_aarch64_el2_init * ni = (struct nvmm_aarch64_el2_init *)tf->tf_reg[0];

	/*
	 * tpidr_el2 is a pointer to struct cpu_info
	 */
	reg_tpidr_el2_write((uint64_t)ni->ni_cpuinfo);

	/*
	 * VTCR_EL2 is a fixed value and should be initialized here.
	 */
	reg_vtcr_el2_write(ni->ni_vtcr_el2);

	/*
	 * Enable *EL2* MMU with identity mapping, i.e. VA==PA.
	 * Not a stage2 translation for EL0/1.
	 *
	 * Ensure EL2 has the same view of MAIR as EL1.
	 *
	 * If EL2 is not run with MMU enabled, cache does not work on EL2
	 * or exclusive load/store cannot be used.
	 *
	 * If the MMU is already enabled, do nothing. Perhaps we should
	 * mark the CPU as not available to NVMM?
	 */
	if (reg_sctlr_el2_read() & SCTLR_M) {
		uartprintf("%s:%d: MMU is already enabled?!\n",
		    __func__, __LINE__);
		return;
	}

	uint64_t mmfr0_parange =
	    __SHIFTOUT(reg_id_aa64mmfr0_el1_read(), ID_AA64MMFR0_EL1_PARANGE);
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

	/* x0 := L0 table of PA */
	reg_ttbr0_el2_write(ni->ni_ttbr0_el2);
	isb();

	if (reg_mpidr_el1_read() == 0x81000000) {
		uartprintf("ni %p ci %p ttbr0_el2 %lx vtcr_el2 %lx\n", ni,  ni->ni_cpuinfo, ni->ni_ttbr0_el2, ni->ni_vtcr_el2);
	}

	aarch64_tlbi_all_local_el2();

	/* enable Icache, Dcache, MMU! */
	reg_sctlr_el2_write(reg_sctlr_el2_read() |
	     SCTLR_I | SCTLR_C | SCTLR_M);
	isb();
}

static void
vcpu_context_save(struct trapframe *tf, struct nvmm_aarch64_state *state)
{
	NVMMHIST_FUNC();
	NVMMHIST_CALLARGSN(nvmmdebug, 10, "tf %#jx state #%jx",
	    (uintptr_t)tf, (uintptr_t)state, 0, 0);

	KASSERTMSG((reg_daif_read() & DAIF_MASK) == DAIF_MASK,
	    "DAIF=%" __PRIxBITS, __SHIFTOUT(reg_daif_read(), DAIF_MASK));

	state->sprs[NVMM_AARCH64_SPR_PC] = tf->tf_pc;
	state->sprs[NVMM_AARCH64_SPR_SPSR_EL1] = tf->tf_spsr;

	memcpy(state->gprs, tf->tf_reg, sizeof(state->gprs));
	nvmm_aarch64_save_fpregs(state->fprs);	/* q0-q31 */

	/*
	 * exception
	 * from       gprs[NVMM_AARCH64_GPR_X31]
	 * ---------- --------------------------
	 * EL0T       sp_el0
	 * EL1T       sp_el0
	 * EL1H       sp_el1
	 * SYS32      sp_el1
	 * UND32      sp_el1
	 * ABT32      sp_el1
	 * SVC32      sp_el1
	 * IRQ32      sp_el1
	 * FIQ32      sp_el1
	 * USR32      sp_el0
	 */
	state->sprs[NVMM_AARCH64_SPR_SP_EL0] = reg_sp_el0_read();
	state->sprs[NVMM_AARCH64_SPR_SP_EL1] = reg_sp_el1_read();
	if (SPSR_USER_P(tf->tf_spsr) ||
	    __SHIFTOUT(SPSR_M, tf->tf_spsr) == SPSR_M_EL1T) {
		state->gprs[NVMM_AARCH64_GPR_X31] = state->sprs[NVMM_AARCH64_SPR_SP_EL0];
	} else {
		state->gprs[NVMM_AARCH64_GPR_X31] = state->sprs[NVMM_AARCH64_SPR_SP_EL1];
	}

	NVMMHIST_LOGN(nvmmdebug, 40,
	    "pc =              %#18jx  "
	    "spsr =            %#18jx  "
	    "sp_el0 =          %#18jx  "
	    "sp_el1 =          %#18jx",
	    state->sprs[NVMM_AARCH64_SPR_PC],
	    state->sprs[NVMM_AARCH64_SPR_SPSR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_SP_EL0],
	    state->sprs[NVMM_AARCH64_SPR_SP_EL1]);

	state->sprs[NVMM_AARCH64_SPR_TPIDRRO_EL0] = reg_tpidrro_el0_read();
	state->sprs[NVMM_AARCH64_SPR_TPIDR_EL0] = reg_tpidr_el0_read();
	state->sprs[NVMM_AARCH64_SPR_AMAIR_EL1] = reg_amair_el1_read();
	state->sprs[NVMM_AARCH64_SPR_CNTKCTL_EL1] = reg_cntkctl_el1_read();

	NVMMHIST_LOGN(nvmmdebug, 40,
	    "tpidrro_el0 =     %#18jx  "
	    "tpidr_el0 =       %#18jx  "
	    "amair_el1 =       %#18jx  "
	    "cntkctl_el1 =     %#18jx",
	    state->sprs[NVMM_AARCH64_SPR_TPIDRRO_EL0],
	    state->sprs[NVMM_AARCH64_SPR_TPIDR_EL0],
	    state->sprs[NVMM_AARCH64_SPR_AMAIR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_CNTKCTL_EL1]);

	state->sprs[NVMM_AARCH64_SPR_CONTEXTIDR_EL1] = reg_contextidr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_CPACR_EL1] = reg_cpacr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_CSSELR_EL1] = reg_csselr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_ELR_EL1] = reg_elr_el1_read();

	NVMMHIST_LOGN(nvmmdebug, 40,
	    "contextidr_el1 =  %#18jx  "
	    "cpacr_el1 =       %#18jx  "
	    "csselr_el1 =      %#18jx  "
	    "elr_el1 =         %#18jx",
	    state->sprs[NVMM_AARCH64_SPR_CONTEXTIDR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_CPACR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_CSSELR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_ELR_EL1]);

	state->sprs[NVMM_AARCH64_SPR_ESR_EL1] = reg_esr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_FAR_EL1] = reg_far_el1_read();
	state->sprs[NVMM_AARCH64_SPR_FPCR] = reg_fpcr_read();
	state->sprs[NVMM_AARCH64_SPR_FPSR] = reg_fpsr_read();

	NVMMHIST_LOGN(nvmmdebug, 40,
	    "esr_el1 =         %#18jx  "
	    "far_el1 =         %#18jx  "
	    "fpcr =            %#18jx  "
	    "fpsr =            %#18jx",
	    state->sprs[NVMM_AARCH64_SPR_ESR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_FAR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_FPCR],
	    state->sprs[NVMM_AARCH64_SPR_FPSR]);

	state->sprs[NVMM_AARCH64_SPR_MAIR_EL1] = reg_mair_el1_read();
	state->sprs[NVMM_AARCH64_SPR_MDSCR_EL1] = reg_mdscr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_MIDR_EL1] = reg_vpidr_el2_read();
	state->sprs[NVMM_AARCH64_SPR_MPIDR_EL1] = reg_vmpidr_el2_read();

	NVMMHIST_LOGN(nvmmdebug, 40,
	    "mair_el1 =        %#18jx  "
	    "mdscr_el1 =       %#18jx  "
	    "midr_el1 =        %#18jx  "
	    "mpidr_el1 =       %#18jx",
	    state->sprs[NVMM_AARCH64_SPR_MAIR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_MDSCR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_MIDR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_MPIDR_EL1]);

	state->sprs[NVMM_AARCH64_SPR_PAR_EL1] = reg_par_el1_read();
	state->sprs[NVMM_AARCH64_SPR_SCTLR_EL1] = reg_sctlr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_TCR_EL1] = reg_tcr_el1_read();
	state->sprs[NVMM_AARCH64_SPR_TPIDR_EL1] = reg_tpidr_el1_read();

	NVMMHIST_LOGN(nvmmdebug, 40,
	    "par_el1 =         %#18jx  "
	    "sctlr_el1 =       %#18jx  "
	    "tcr_el1 =         %#18jx  "
	    "tpidr_el1 =       %#18jx",
	    state->sprs[NVMM_AARCH64_SPR_PAR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_SCTLR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_TCR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_TPIDR_EL1]);

	state->sprs[NVMM_AARCH64_SPR_TTBR0_EL1] = reg_ttbr0_el1_read();
	state->sprs[NVMM_AARCH64_SPR_TTBR1_EL1] = reg_ttbr1_el1_read();
	state->sprs[NVMM_AARCH64_SPR_VBAR_EL1] = reg_vbar_el1_read();

	NVMMHIST_LOGN(nvmmdebug, 40,
	    "ttbr0_el1 =       %#18jx  "
	    "ttbr1_el1 =       %#18jx  "
	    "vbar_el1 =        %#18jx",
	    state->sprs[NVMM_AARCH64_SPR_TTBR0_EL1],
	    state->sprs[NVMM_AARCH64_SPR_TTBR1_EL1],
	    state->sprs[NVMM_AARCH64_SPR_VBAR_EL1], 0);

	state->sprs[NVMM_AARCH64_SPR_CNTV_CTL_EL0] = reg_cntv_ctl_el0_read();
	state->sprs[NVMM_AARCH64_SPR_CNTV_CVAL_EL0] = reg_cntv_cval_el0_read();

	NVMMHIST_LOGN(nvmmdebug, 40,
	    "cntv_ctl_el0 =    %#18jx  "
	    "cntv_cval_el0 =   %#18jx",
	    state->sprs[NVMM_AARCH64_SPR_CNTV_CTL_EL0],
	    state->sprs[NVMM_AARCH64_SPR_CNTV_CVAL_EL0], 0, 0);

	NVMMHIST_LOGN(nvmmdebug, 10, "<--- done", 0, 0, 0, 0);
}

static void
vcpu_context_load(struct trapframe *tf, const struct nvmm_aarch64_state *state)
{
	NVMMHIST_FUNC();
	NVMMHIST_CALLARGSN(nvmmdebug, 10, "tf %#jx state #%jx", (uintptr_t)tf, (uintptr_t)state, 0, 0);

	KASSERTMSG((reg_daif_read() & DAIF_MASK) == DAIF_MASK,
	    "DAIF=%" __PRIxBITS, __SHIFTOUT(reg_daif_read(), DAIF_MASK));

	tf->tf_pc = state->sprs[NVMM_AARCH64_SPR_PC];
	tf->tf_spsr = state->sprs[NVMM_AARCH64_SPR_SPSR_EL1];

	memcpy(tf->tf_reg, state->gprs, sizeof(state->gprs));
	nvmm_aarch64_load_fpregs(state->fprs);	/* q0-q31 */

	if (SPSR_USER_P(tf->tf_spsr) ||
	    __SHIFTOUT(SPSR_M, tf->tf_spsr) == SPSR_M_EL1T) {
		reg_sp_el0_write(state->gprs[NVMM_AARCH64_GPR_X31]);
		reg_sp_el1_write(state->sprs[NVMM_AARCH64_SPR_SP_EL1]);
	} else {
		reg_sp_el0_write(state->sprs[NVMM_AARCH64_SPR_SP_EL0]);
		reg_sp_el1_write(state->gprs[NVMM_AARCH64_GPR_X31]);
	}

	// XXXNH
	// PMU regs
	// physical timer regs - always virtualise?
	// mdccint_el1
	// OSDLR_El1
	// OSLAR_EL1
	// PMSELR_EL0
	// VTCR_EL2???

	// VHE:
	// AMAIR_EL12
	// CNTKCTL_EL12
	// CNTV_CVAL_EL12
	// CONTEXTIDR_EL12
	// CPACR_EL12
	// ELR_EL12
	// ESR_EL12
	// FAR_EL12
	// MAIR_EL12
	// SCTRL_EL12
	// SPSR_EL12
	// TCR_EL12
	// TTBR0_EL12
	// TTBR0_EL12
	// VBAR_EL12
	// VTTBR_EL2
	// AFSR[012]_EL12
	// PAUTH: AP*KEY*_EL1

	reg_tpidrro_el0_write(state->sprs[NVMM_AARCH64_SPR_TPIDRRO_EL0]);
	reg_tpidr_el0_write(state->sprs[NVMM_AARCH64_SPR_TPIDR_EL0]);
	reg_amair_el1_write(state->sprs[NVMM_AARCH64_SPR_AMAIR_EL1]);
	reg_cntkctl_el1_write(state->sprs[NVMM_AARCH64_SPR_CNTKCTL_EL1]);

	NVMMHIST_LOGN(nvmmdebug, 40,
	    "tpidrro_el0 =     %#18jx  "
	    "tpidr_el0 =       %#18jx  "
	    "amair_el1 =       %#18jx  "
	    "cntkctl_el1 =     %#18jx",
	    state->sprs[NVMM_AARCH64_SPR_TPIDRRO_EL0],
	    state->sprs[NVMM_AARCH64_SPR_TPIDR_EL0],
	    state->sprs[NVMM_AARCH64_SPR_AMAIR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_CNTKCTL_EL1]);

	reg_contextidr_el1_write(state->sprs[NVMM_AARCH64_SPR_CONTEXTIDR_EL1]);
	reg_cpacr_el1_write(state->sprs[NVMM_AARCH64_SPR_CPACR_EL1]);
	reg_csselr_el1_write(state->sprs[NVMM_AARCH64_SPR_CSSELR_EL1]);

	NVMMHIST_LOGN(nvmmdebug, 40,
	    "contextidr_el1 =  %#18jx  "
	    "cpacr_el1 =       %#18jx  "
	    "csselr_el1 =      %#18jx",
	    state->sprs[NVMM_AARCH64_SPR_CONTEXTIDR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_CPACR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_CSSELR_EL1], 0);

// XXXNH
	reg_elr_el1_write(state->sprs[NVMM_AARCH64_SPR_ELR_EL1]);
	reg_esr_el1_write(state->sprs[NVMM_AARCH64_SPR_ESR_EL1]);

	NVMMHIST_LOGN(nvmmdebug, 40,
	    "elr_el1 =         %#18jx  "
	    "esr_el1 =         %#18jx",
	    state->sprs[NVMM_AARCH64_SPR_ELR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_ESR_EL1], 0, 0);

	reg_far_el1_write(state->sprs[NVMM_AARCH64_SPR_FAR_EL1]);
	reg_fpcr_write(state->sprs[NVMM_AARCH64_SPR_FPCR]);
	reg_fpsr_write(state->sprs[NVMM_AARCH64_SPR_FPSR]);
	reg_mair_el1_write(state->sprs[NVMM_AARCH64_SPR_MAIR_EL1]);

	NVMMHIST_LOGN(nvmmdebug, 40,
	    "far_el1 =         %#18jx  "
	    "fpcr =            %#18jx  "
	    "fpsr =            %#18jx  "
	    "mair_el1 =        %#18jx",
	    state->sprs[NVMM_AARCH64_SPR_FAR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_FPCR],
	    state->sprs[NVMM_AARCH64_SPR_FPSR],
	    state->sprs[NVMM_AARCH64_SPR_MAIR_EL1]);

	reg_mdscr_el1_write(state->sprs[NVMM_AARCH64_SPR_MDSCR_EL1]);
	reg_par_el1_write(state->sprs[NVMM_AARCH64_SPR_PAR_EL1]);
	reg_sctlr_el1_write(state->sprs[NVMM_AARCH64_SPR_SCTLR_EL1]);
	reg_tcr_el1_write(state->sprs[NVMM_AARCH64_SPR_TCR_EL1]);

	NVMMHIST_LOGN(nvmmdebug, 40,
	    "mdscr_el1 =       %#18jx  "
	    "par_el1 =         %#18jx  "
	    "sctlr_el1 =       %#18jx  "
	    "tcr_el1 =         %#18jx",
	    state->sprs[NVMM_AARCH64_SPR_MDSCR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_PAR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_SCTLR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_TCR_EL1]);

	reg_tpidr_el1_write(state->sprs[NVMM_AARCH64_SPR_TPIDR_EL1]);
	reg_ttbr0_el1_write(state->sprs[NVMM_AARCH64_SPR_TTBR0_EL1]);
	reg_ttbr1_el1_write(state->sprs[NVMM_AARCH64_SPR_TTBR1_EL1]);
	reg_vbar_el1_write(state->sprs[NVMM_AARCH64_SPR_VBAR_EL1]);

	NVMMHIST_LOGN(nvmmdebug, 40,
	    "tpidr_el1 =       %#18jx  "
	    "ttbr0_el1 =       %#18jx  "
	    "ttbr1_el1 =       %#18jx  "
	    "vbar_el1 =        %#18jx",
	    state->sprs[NVMM_AARCH64_SPR_TPIDR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_TTBR0_EL1],
	    state->sprs[NVMM_AARCH64_SPR_TTBR1_EL1],
	    state->sprs[NVMM_AARCH64_SPR_VBAR_EL1]);

	reg_vpidr_el2_write(state->sprs[NVMM_AARCH64_SPR_MIDR_EL1]);
	reg_vmpidr_el2_write(state->sprs[NVMM_AARCH64_SPR_MPIDR_EL1]);
	reg_cntv_ctl_el0_write(state->sprs[NVMM_AARCH64_SPR_CNTV_CTL_EL0]);
	reg_cntv_cval_el0_write(state->sprs[NVMM_AARCH64_SPR_CNTV_CVAL_EL0]);

	NVMMHIST_LOGN(nvmmdebug, 40,
	    "midr_el1 =        %#18jx  "
	    "mpidr_el1 =       %#18jx  "
	    "cntv_ctl_el0 =    %#18jx  "
	    "cntv_cval_el0 =   %#18jx",
	    state->sprs[NVMM_AARCH64_SPR_MIDR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_MPIDR_EL1],
	    state->sprs[NVMM_AARCH64_SPR_CNTV_CTL_EL0],
	    state->sprs[NVMM_AARCH64_SPR_CNTV_CVAL_EL0]);

	NVMMHIST_LOGN(nvmmdebug, 10, "<--- done", 0, 0, 0, 0);
}

static void
aarch64_vmexit_context(struct trapframe *tf, struct aarch64_cpudata *cpudata)
{
	NVMMHIST_FUNC();
	NVMMHIST_CALLARGSN(nvmmdebug, 10, "tf %#jx cpudata %#jx",
	    (uintptr_t)tf, (uintptr_t)cpudata, 0, 0);

	struct cpu_info * const ci = aarch64nvmm_curcpu();

	reg_mdcr_el2_write(0);
	reg_hcr_el2_write(HCR_EL2_RW);
	reg_hstr_el2_write(0);
	reg_vttbr_el2_write(0);
	isb();

	NVMMHIST_LOGN(nvmmdebug, 20,
	    "mair_el2 = %#jx vtcr_el2 = %#jx ttbr0_el2 = %#jx sctlr_el2 = %#jx",
	    reg_mair_el2_read(), reg_vtcr_el2_read(),
	    reg_ttbr0_el2_read(), reg_sctlr_el2_read());

	/* save guest state */
	vcpu_context_save(tf, &cpudata->guest);
	/* load host state */
	vcpu_context_load(tf, &cpudata->host);

	ci->ci_invm = false;

	NVMMHIST_LOGN(nvmmdebug, 10, "<--- done (host loaded %#jx, guest saved %#jx)",
	    (uintptr_t)&cpudata->host, (uintptr_t)&cpudata->guest, 0, 0);
}

void
aarch64_el2_vmexit_irq(struct trapframe *tf)
{
	NVMMHIST_FUNC();
	NVMMHIST_CALLARGS(nvmmdebug, "tf %#jx", (uintptr_t)tf, 0, 0, 0);

	struct cpu_info * const ci = aarch64nvmm_curcpu();
	struct aarch64_cpudata * const cpudata = ci->ci_cpudata;

	/*
	 * Interrupts arrive here when running as a guest (with
	 *
	 *     HCR_EL2.{A,I,F}MO = {1,1,1}.
	 *
	 * Some of these interrupts are generated by the guest directly, e.g.
	 * timer interrupts.
	 * Timer interrupts can be easily checked and an appropriate exit code
	 * supplied. Userland can then mark the interrupt as asserted in the
	 * virtual interrupt controller (GIC) and a virtual interrupt injected.
	 */
	cpudata->exit.reason = NVMM_VCPU_EXIT_NONE;

	uint64_t cntv_ctl = reg_cntv_ctl_el0_read();
	const bool enabled = (cntv_ctl & CNTCTL_ENABLE) != 0;
	const bool fired = (cntv_ctl & CNTCTL_ISTATUS) != 0;
	const bool masked = (cntv_ctl & CNTCTL_IMASK) != 0;
	if (enabled && !masked && fired) {
		NVMMHIST_LOG(nvmmdebug, "NVMM_VCPU_EXIT_IRQ "
		    "(cntv_ctl_el0 0x%01jx cntpct_el0 %#jx cntv_cval_el0 %#jx)",
		     cntv_ctl, reg_cntpct_el0_read(), reg_cntv_cval_el0_read(),
		     0);

		cpudata->exit.reason = NVMM_VCPU_EXIT_IRQ;
		cpudata->exit.exitstate.vtimer = 1;
                reg_cntv_ctl_el0_write(cntv_ctl | CNTCTL_IMASK);
	}

	aarch64_vmexit_context(tf, cpudata);
}

#define SYSREG_ENC(op0, op1, CRn, CRm, op2)		\
    (((op0) << 19) | ((op1) << 16) | ((CRn) << 12) | ((CRm) << 8) | ((op2) << 5))

struct sysreg_table {
	uint32_t code;
	int id;
	bool writable;
	const char name[32];	/* debug. don't use "const char *" because el2 running on PA */
};

/* must be sorted by code */
const struct sysreg_table sysreg_tid[] = {
	/*         op0 op1 CRn CRm op2 ID                                   writable?  name             */
	{ SYSREG_ENC(3, 0,  0,  3, 0), NVMM_AARCH64_TID_MVFR0_EL1,		false, "MVFR0_EL1"	},
	{ SYSREG_ENC(3, 0,  0,  3, 1), NVMM_AARCH64_TID_MVFR1_EL1,		false, "MVFR1_EL1"	},
	{ SYSREG_ENC(3, 0,  0,  3, 2), NVMM_AARCH64_TID_MVFR2_EL1,		false, "MVFR2_EL1"	},
	{ SYSREG_ENC(3, 0,  0,  4, 0), NVMM_AARCH64_TID_ID_AA64PFR0_EL1,	false, "ID_AA64PFR0_EL1" },
	{ SYSREG_ENC(3, 0,  0,  4, 1), NVMM_AARCH64_TID_ID_AA64PFR1_EL1,	false, "ID_AA64PFR1_EL1" },
	{ SYSREG_ENC(3, 0,  0,  5, 0), NVMM_AARCH64_TID_ID_AA64DFR0_EL1,	false, "ID_AA64DFR0_EL1" },
	{ SYSREG_ENC(3, 0,  0,  5, 1), NVMM_AARCH64_TID_ID_AA64DFR1_EL1,	false, "ID_AA64DFR1_EL1" },
	{ SYSREG_ENC(3, 0,  0,  5, 4), NVMM_AARCH64_TID_ID_AA64AFR0_EL1,	false, "ID_AA64AFR0_EL1" },
	{ SYSREG_ENC(3, 0,  0,  5, 5), NVMM_AARCH64_TID_ID_AA64AFR1_EL1,	false, "ID_AA64AFR1_EL1" },
	{ SYSREG_ENC(3, 0,  0,  6, 0), NVMM_AARCH64_TID_ID_AA64ISAR0_EL1,	false, "ID_AA64ISAR0_EL1" },
	{ SYSREG_ENC(3, 0,  0,  6, 1), NVMM_AARCH64_TID_ID_AA64ISAR1_EL1,	false, "ID_AA64ISAR1_EL1" },
	{ SYSREG_ENC(3, 0,  0,  7, 0), NVMM_AARCH64_TID_ID_AA64MMFR0_EL1,	false, "ID_AA64MMFR0_EL1" },
	{ SYSREG_ENC(3, 0,  0,  7, 1), NVMM_AARCH64_TID_ID_AA64MMFR1_EL1,	false, "ID_AA64MMFR1_EL1" },
	{ SYSREG_ENC(3, 0,  0,  7, 2), NVMM_AARCH64_TID_ID_AA64MMFR2_EL1,	false, "ID_AA64MMFR2_EL1" },
};

static const struct sysreg_table *
sysreg_bsearch(const struct sysreg_table *table, size_t tablenum, uint32_t code)
{
	const struct sysreg_table *base = table;

	for (size_t lim = tablenum; lim != 0; lim /= 2) {
		const struct sysreg_table *p = base + (lim / 2);
		if (p->code == code) {
			return p;
		}
		if (code > p->code) {
			base = p + 1;
			lim--;
		}
	}
	return NULL;
}

static int
emul_sysreg_rw(struct trapframe *tf)
{
	NVMMHIST_FUNC();
	NVMMHIST_CALLARGS(nvmmdebug, "tf %#jx (esr %#jx)", (uintptr_t)tf, tf->tf_esr, 0, 0);

	const uint64_t esr = tf->tf_esr;
	const uint64_t op0 = __SHIFTOUT(esr, ESR_ISS_SYSREG_OP0);
	const uint64_t op2 = __SHIFTOUT(esr, ESR_ISS_SYSREG_OP2);
	const uint64_t op1 = __SHIFTOUT(esr, ESR_ISS_SYSREG_OP1);
	const uint64_t CRn = __SHIFTOUT(esr, ESR_ISS_SYSREG_CRN);
	const uint64_t Rt = __SHIFTOUT(esr, ESR_ISS_SYSREG_RT);
	const uint64_t CRm = __SHIFTOUT(esr, ESR_ISS_SYSREG_CRM);
	const uint64_t dir = __SHIFTOUT(esr, ESR_ISS_SYSREG_DIRECTION);

	uint32_t code = SYSREG_ENC(op0, op1, CRn, CRm, op2);
	const struct sysreg_table *tid =
	    sysreg_bsearch(sysreg_tid, __arraycount(sysreg_tid), code);
	if (tid == NULL) {
		NVMMHIST_LOG(nvmmdebug, "<-- done (unsupported)", 0, 0, 0, 0);
		return -1;
	}

	struct cpu_info * const ci = aarch64nvmm_curcpu();
	struct aarch64_cpudata * const cpudata = ci->ci_cpudata;

	if (dir == 0) {
		/* write access */
		if (tid->writable) {
			if (Rt == 31) {
				cpudata->guest.tids[tid->id] = 0;
			} else {
				cpudata->guest.tids[tid->id] = tf->tf_reg[Rt];
			}
		} else {
			NVMMHIST_LOG(nvmmdebug, "<-- done (cannot write)", 0, 0, 0, 0);
			// XXX: TODO: inject undefined
			return -1;
		}
	} else {
		/* read access */
		NVMMHIST_LOG(nvmmdebug, "read sysreg trapped", 0, 0, 0, 0);
		if (Rt != 31)
			tf->tf_reg[Rt] = cpudata->guest.tids[tid->id];
	}
	tf->tf_pc += 4;

	return 0;
}

void
aarch64_el2_vmexit_trap(struct trapframe *tf)
{
	NVMMHIST_FUNC();
	NVMMHIST_CALLARGSN(nvmmdebug, 10, "tf %#jx", (uintptr_t)tf, 0, 0, 0);

	struct cpu_info * const ci = aarch64nvmm_curcpu();
	struct aarch64_cpudata * const cpudata = ci->ci_cpudata;
	struct nvmm_aarch64_exit *exit = &cpudata->exit;
	const uint64_t esr = tf->tf_esr;
	const uint64_t eclass = __SHIFTOUT(esr, ESR_EC);
	bool do_vmexit = true;
	vm_prot_t ftype;

	exit->esr = esr;
	exit->insn = 0;

	switch (eclass) {
	case ESR_EC_INSN_ABT_EL_LOW:
		/*
		 * Abort on instruction fetch.
		 * this requires PROT_EXEC and implicit PROT_READ.
		 */
		ftype = VM_PROT_READ | VM_PROT_EXECUTE;
		exit->reason = NVMM_VCPU_EXIT_MEMORY;
		exit->u.mem.gpa = aarch64_get_fault_ipa(tf);
		exit->u.mem.prot = ftype;
		break;
	case ESR_EC_DATA_ABT_EL_LOW:
		/* Abort on data load or store */
		if (__SHIFTOUT(esr, ESR_ISS_DATAABORT_CM)) {
			/* cache maintainance op should be treated as read */
			ftype = VM_PROT_READ;
		} else {
			uint64_t rw = __SHIFTOUT(esr, ESR_ISS_DATAABORT_WnR);
			ftype = (rw == 0) ? VM_PROT_READ : VM_PROT_WRITE;
		}
		exit->reason = NVMM_VCPU_EXIT_MEMORY;
		exit->u.mem.gpa = aarch64_get_fault_ipa(tf);
		exit->u.mem.prot = ftype;
		break;
	case ESR_EC_INSN_ABT_EL_CUR:
	case ESR_EC_DATA_ABT_EL_CUR:
		/* el2sync_el2h() should be called, so it shouldn't come here... */
		NVMMHIST_LOG(nvmmdebug, "INSN or DATA ABORT occured on EL2?", 0, 0, 0, 0);
		exit->reason = NVMM_VCPU_EXIT_HALTED;
		break;
	case ESR_EC_UNKNOWN:
	case ESR_EC_SERROR:
	case ESR_EC_ILL_STATE:
	case ESR_EC_BTE_A64:
	case ESR_EC_SVC_A64:
	case ESR_EC_PC_ALIGNMENT:
	case ESR_EC_SP_ALIGNMENT:
	case ESR_EC_FP_ACCESS:
	case ESR_EC_FP_TRAP_A64:
	case ESR_EC_BRKPNT_EL_LOW:
	case ESR_EC_BRKPNT_EL_CUR:
	case ESR_EC_SW_STEP_EL_LOW:
	case ESR_EC_SW_STEP_EL_CUR:
	case ESR_EC_WTCHPNT_EL_LOW:
	case ESR_EC_WTCHPNT_EL_CUR:
	case ESR_EC_BKPT_INSN_A64:
	default:
		NVMMHIST_LOG(nvmmdebug, "PC=%#jx ESR_EL2=%#jx (eclass=%#jx)", tf->tf_pc, esr, eclass, 0);
		dump_el2_trapframe(tf);
		exit->reason = NVMM_VCPU_EXIT_HALTED;
		break;
	case ESR_EC_SYS_REG:
		/* system register trap */
		if (emul_sysreg_rw(tf) == 0) {
			do_vmexit = false;
		} else {
			if (__SHIFTOUT(esr, ESR_ISS_SYSREG_DIRECTION) == 0) {
				NVMMHIST_LOG(nvmmdebug, "NVMM_VCPU_EXIT_MSR", 0, 0, 0, 0);
				exit->reason = NVMM_VCPU_EXIT_MSR;
			} else {
				NVMMHIST_LOG(nvmmdebug, "NVMM_VCPU_EXIT_MRS", 0, 0, 0, 0);
				exit->reason = NVMM_VCPU_EXIT_MRS;
			}
		}
		break;
	case ESR_EC_HVC_A64:
		exit->reason = NVMM_VCPU_EXIT_HVC;
		break;
	case ESR_EC_SMC_A64:
		exit->reason = NVMM_VCPU_EXIT_SMC;
		break;
	case ESR_EC_WFX:
		exit->reason = ((esr & ESR_ISS_WFX_TRAP_INSN) == 0) ?
		    NVMM_VCPU_EXIT_WFI : NVMM_VCPU_EXIT_WFE;
		break;
	}

// XXXNH used by the MMIO emulation code. hmm.
#if 1
	if (eclass != ESR_EC_INSN_ABT_EL_LOW &&
	    eclass != ESR_EC_INSN_ABT_EL_CUR) {
		// XXXNH defer to userland
		/*
		 * read an instruction from PC.
		 * if instruction abort, it cannot be read.
		 */
		uint32_t *pa = (uint32_t *)aarch64_gva_to_pa(tf->tf_spsr, tf->tf_pc);
		if (pa != (void *)-1)
			exit->insn = le32toh(*pa);
	}
#endif

	if (do_vmexit) {
		aarch64_vmexit_context(tf, cpudata);
	}
}

void
aarch64_el2_vmenter(struct trapframe *tf)
{
	NVMMHIST_FUNC();
	NVMMHIST_CALLARGSN(nvmmdebug, 10, "tf %#jx state #%jx", (uintptr_t)tf, 0, 0, 0);

	NVMMHIST_LOGN(nvmmdebug, 20,
	    "mair_el2 = %#jx vtcr_el2 = %#jx ttbr0_el2 = %#jx sctlr_el2 = %#jx",
	    reg_mair_el2_read(), reg_vtcr_el2_read(),
	    reg_ttbr0_el2_read(), reg_sctlr_el2_read());

	KASSERTMSG((reg_daif_read() & DAIF_MASK) == DAIF_MASK,
	    "DAIF=%" __PRIxBITS, __SHIFTOUT(reg_daif_read(), DAIF_MASK));

	struct cpu_info * const ci = aarch64nvmm_curcpu();
	ci->ci_cpudata = (struct aarch64_cpudata *)tf->tf_reg[0];
	struct aarch64_cpudata * const cpudata = ci->ci_cpudata;

	KASSERTMSG((reg_daif_read() & DAIF_MASK) == DAIF_MASK,
	    "DAIF=%" __PRIxBITS, __SHIFTOUT(reg_daif_read(), DAIF_MASK));

	if (cpudata == NULL)
		uartprintf("panic: %s: cpudata is NULL\n", __func__);

	/* save host state */
	vcpu_context_save(tf, &cpudata->host);
	/* load guest state */
	vcpu_context_load(tf, &cpudata->guest);

	uint64_t hcr = HCR_EL2_RW;	/* 64bit */

//	hcr |= HCR_EL2_ID;		/* stage2 IC disable */
//	hcr |= HCR_EL2_CD;		/* stage2 DC disable */
//	hcr |= HCR_EL2_TRVM;		/* trap EL1 reads SCTLR_EL1,TTBR0_EL1,TTBR1_EL1,TCR_EL1,ESR_EL1,FAR_EL1,AFSR0_EL1,AFSR1_EL1,MAIR_EL1,AMAIR_EL1,CONTEXTIDR_EL1 */
//	hcr |= HCR_EL2_HCD;		/* hvc disable (if EL3 is not implemented) */
//	hcr |= HCR_EL2_TDZ;		/* trap DC ZVA */
//	hcr |= HCR_EL2_TGE;		/* trap EL0/EL1 general exceptions */
//	hcr |= HCR_EL2_TVM;		/* trap EL1 writes SCTLR_EL1,TTBR0_EL1,TTBR1_EL1,TCR_EL1,ESR_EL1,FAR_EL1,AFSR0_EL1,AFSR1_EL1,MAIR_EL1,AMAIR_EL1,CONTEXTIDR_EL1 */
//	hcr |= HCR_EL2_TTLB;		/* trap EL1 TLB op */
//	hcr |= HCR_EL2_TPU;		/* trap EL0/EL1 cache op: IC IVAU,IC IALLU,IC IALLUIS,DC CVAU */
//	hcr |= HCR_EL2_TPC;		/* trap EL0/EL1 cache op: DC IVAC,DC CIVAC,DC CVAC */
//	hcr |= HCR_EL2_TSW;		/* trap EL0/EL1 cache op: DC ISW,DC CSW,DC CISW */
	hcr |= HCR_EL2_TACR;		/* trap EL1 accessing ACTLR_EL1 */ // XXXNH yes
	hcr |= HCR_EL2_TIDCP;		/* trap IMPLEMENTATION DEFINED system registers */ // XXXNH yes
	hcr |= HCR_EL2_TSC;		/* trap SMC */
	hcr |= HCR_EL2_TID3;		/* trap ID group3 regs: ID_PFR*_EL1,ID_DFR*_EL1,ID_AFR*_EL1,ID_MMFR*_EL1,ID_ISAR*_EL1,MVFR*_EL1,ID_AA64PFR*_EL1,ID_AA64DFR*_EL1,ID_AA64ISAR*_EL1,ID_AA64MMFR*_EL1,ID_AA64AFR*_EL1 */
//	hcr |= HCR_EL2_TID2;		/* trap ID group2 regs: CTR_EL0,CCSIDR_EL1,CLIDR_EL1,CSSELR_EL1 */
//	hcr |= HCR_EL2_TID1;		/* trap ID group1 regs: AIDR_EL1,REVIDR_EL1 */
//	hcr |= HCR_EL2_TID0;		/* trap ID group0 regs: none (aarch32:FPSID,JIDR) */
	hcr |= HCR_EL2_TWE;		/* trap WFE */
	hcr |= HCR_EL2_TWI;		/* trap WFI */
	hcr |= HCR_EL2_DC;		/* default cacheable */
	hcr |= HCR_EL2_AMO;		/* trap SError/AsyncAbort */
	hcr |= HCR_EL2_IMO;		/* trap Physical IRQ */
	hcr |= HCR_EL2_FMO;		/* trap Physical FIQ */
	hcr |= HCR_EL2_PTW;		/* Protect table walk */
//	hcr |= HCR_EL2_SWIO;		/* override DC ISW to DC CISW */
	hcr |= HCR_EL2_VM;		/* enable Stage2 translation */
#ifdef MULTIPROCESSOR
	/* barrier shareability upgrade */
	hcr |= __SHIFTIN(1, HCR_EL2_BSU);	/* upgrade to inner-shareable */
	hcr |= HCR_EL2_FB;		/* force broadcast TLBI VMALLE1,TLBI VAE1,TLBI ASIDE1,TLBI VAAE1,TLBI VALE1,TLBI VAALE1,IC IALLU */
#endif

	if (__predict_false(cpudata->send_event_type != NVMM_VCPU_EVENT_NONE)) {
		/* XXXNH: This is the point where the guest-visible interrupt/exception
		 * delivery path is wired up for the next entry. On AArch64 this is not
		 * a direct "inject vector N" operation like x86 VM-entry injection;
		 * instead it enables the virtual interrupt machinery so the guest can
		 * observe the event through the exception path, ideally via an
		 * emulated interrupt-controller-like source.
		 */
		switch (cpudata->send_event_type) {
		case NVMM_VCPU_EVENT_SYNC:
			NVMMHIST_LOG(nvmmdebug, "NVMM_VCPU_EVENT_SYNC: PC=%#jx LR=%#jx VBAR_EL1=0x%016lx",
			    cpudata->guest.sprs[NVMM_AARCH64_SPR_PC],
			    cpudata->guest.gprs[NVMM_AARCH64_GPR_X30],
			    cpudata->guest.sprs[NVMM_AARCH64_SPR_VBAR_EL1],
			    0);
			break;
		case NVMM_VCPU_EVENT_SERROR:
			NVMMHIST_LOG(nvmmdebug, "virtual serror", 0, 0, 0, 0);
			hcr |= HCR_EL2_VSE;
			break;
		case NVMM_VCPU_EVENT_IRQ:
			NVMMHIST_LOG(nvmmdebug, "virtual irq", 0, 0, 0, 0);
			hcr |= HCR_EL2_VI;
			break;
		case NVMM_VCPU_EVENT_FIQ:
			NVMMHIST_LOG(nvmmdebug, "virtual fiq", 0, 0, 0, 0);
			hcr |= HCR_EL2_VF;
			break;
		default:
			NVMMHIST_LOG(nvmmdebug, "type %#jx", cpudata->send_event_type, 0, 0, 0);
		}
		cpudata->send_event_type = NVMM_VCPU_EVENT_NONE;
	}

	reg_vttbr_el2_write(cpudata->vttbr_el2);
	reg_hstr_el2_write(0xffff);
	reg_hcr_el2_write(hcr);
	isb();

	// XXXNH MDCR_EL2
	reg_mdcr_el2_write(MDCR_EL2_TPM);
	isb();

	ci->ci_invm = true;

	NVMMHIST_LOGN(nvmmdebug, 10, "pc=%#jx lr=%#jx x0=%#jx hcr_el2=%#jx",
	    cpudata->guest.sprs[NVMM_AARCH64_SPR_PC],
	    cpudata->guest.gprs[NVMM_AARCH64_GPR_X30],
	    cpudata->guest.gprs[NVMM_AARCH64_GPR_X0],
	    hcr);
}

/* do TLB and CACHE operation with vm's VTTBR_EL2 */
void
aarch64_el2_maintain_ipa(struct trapframe *tf)
{
	NVMMHIST_FUNC();
	NVMMHIST_CALLARGSN(nvmmdebug, 10, "tf %#jx", (uintptr_t)tf, 0, 0, 0);

	uint64_t vttbr_el2, op, ipa, va __unused;

	// Doesn't this get a mach *
	// XXXNH ipa and va?!?
	/* void aarch64_hvc_maintain_ipa(vttbr_el2, op, addr) */
	vttbr_el2 = tf->tf_reg[0];
	op = tf->tf_reg[1];
	ipa = tf->tf_reg[2];
	va = tf->tf_reg[3];

	reg_vttbr_el2_write(vttbr_el2);
	reg_hcr_el2_write(HCR_EL2_RW | HCR_EL2_VM);
	isb();

	if (op & NVMM_AARCH64_MAINTAIN_OP_TLBI_ALL) {
		NVMMHIST_LOGN(nvmmdebug, 10, "TLBI ALL VMID=%#jx",
		    __SHIFTOUT(reg_vttbr_el2_read(), VTTBR_VIMD), 0, 0, 0);

		aarch64_tlbi_by_vmid();
	} else if (op & NVMM_AARCH64_MAINTAIN_OP_TLBI) {
		NVMMHIST_LOGN(nvmmdebug, 10, "TLBI IPA VMID=%#jx IPA=%#jx",
		    __SHIFTOUT(reg_vttbr_el2_read(), VTTBR_VIMD), ipa, 0, 0);

		aarch64_tlbi_by_vmid_ipa(ipa);
	}

	if (op & NVMM_AARCH64_MAINTAIN_OP_ICACHE_SYNC) {
		NVMMHIST_LOGN(nvmmdebug, 10, "ICACHE", 0, 0 , 0, 0);

#if 0
		asm volatile("ic ialluis" ::: "memory");
		asm volatile("dsb sy" ::: "memory");
#endif
		/* XXX */

		// We need to do invalidate Icache for the guest VA (far_el2),
		// However, However, the guest vCPU is lost in this context,
		// so VA is not known...
		//
		// In nvme.c:vmm_do_vcpu_run(), from vcpu_run() back with
		// NVMM_VCPU_EXIT_MEMORY to uvm_fault(), the VA (not equal IPA)
		// in this context must be stored somewhere.
	}

	reg_hcr_el2_write(HCR_EL2_RW);
	reg_vttbr_el2_write(0);
	isb();
}
