/*	$NetBSD: locore_el2.S,v 1.16 2026/09/09 06:04:56 skrll Exp $	*/

/*-
 * Copyright (c) 2012-2014 Andrew Turner
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD: head/sys/arm64/arm64/locore.S 316755 2017-04-13 11:56:27Z andrew $
 */

#include "nvmm.h"

#include <aarch64/asm.h>
#include "assym.h"

RCSID("$NetBSD: locore_el2.S,v 1.16 2026/09/09 06:04:56 skrll Exp $")

/*
 * For use in #include "locore_el2.S".
 */

	.global drop_to_el1
	.text

drop_to_el1_inline:
	mov	x8, lr
	bl	drop_to_el1
	mov	lr, x8
	b	drop_to_el1_inline_done

	/*
	 * If we are started in EL2, configure the required hypervisor
	 * registers and drop to EL1 or, if available, enable E2 Host
	 * for FEAT_VHE capable PEs.
	 *
	 * x27 is cpuindex (0,1,2,...,MAXCPU-1)
	 */
drop_to_el1:
	mrs	x1, CurrentEL
	lsr	x1, x1, #2
	cmp	x1, #0x2
	b.eq	in_el2

	/* Not in EL2, nothing to do, leave. */
	ret

in_el2:
	/* Check for EL2 host mode */
	mrs	x2, hcr_el2
	isb
	tbz	x2, #34, not_el2_host_mode	/* HCR_E2H not set? */

	/*
	 * EL2 host mode enabled, nothing to do, leave. EL2 system registers
	 * will be configured via their _EL1 aliases
	 */
	ret

not_el2_host_mode:
	/* Default HCR_EL2 to EL1 as AArch64 */
	mov	x2, #HCR_EL2_RW
	mrs	x1, id_aa64mmfr1_el1

	/* Extract the VH field (bits 11:8) */
	ubfx    x1, x1, #8, #4

	cmp	x1, #0
	b.eq    vhe_not_supported

	/* 1. Mirror memory attribute types from EL1 to EL2 */
	mrs	x0, mair_el1
	msr	mair_el2, x0

	/* 2. Configure CPTR_EL2 safely for VHE Host Mode */
	mov	x0, #CPACR_FPEN_ALL
	msr	cptr_el2, x0

	/* 3. Initialize TCR_EL2 using standard TCR configuration values */
	ldr	x0, tcr_setting
	msr	tcr_el2, x0
	isb

	/* Enable VHE */
	orr	x2, x2, #HCR_EL2_E2H
	orr	x2, x2, #HCR_EL2_TGE
	msr	hcr_el2, x2
	isb

	/* Don't trap to EL2 on access to various registers. */
	mov	x2, #CPTR_EL2_VHE_FPEN_NONE
	msr	cptr_el2, x2

	/*
	 * Enable physical/virtual timer access from EL0
	 */
	mov	x2, #(CNTHCTL_EL2_VHE_EL0PCTEN | CNTHCTL_EL2_VHE_EL0VCTEN)
	orr	x2, x2, #(CNTHCTL_EL2_VHE_EL0PTEN  | CNTHCTL_EL2_VHE_EL0VTEN)
	msr	cnthctl_el2, x2
	isb

	adrl	x0, _C_LABEL(e2h_enabled)
	mov	x1, #1
	str	w1, [x0]

#if NNVMM > 0
	/* nvmm_available = 1 */
	adrl	x0, _C_LABEL(nvmm_available)
	str	w1, [x0]			/* still 1 from above */
#endif
	b	vhe_return

vhe_not_supported:
	msr	hcr_el2, x2
	isb

#ifdef ARMV80_NVHE
	/* Mirror the Virtualization Process ID Register. */
	mrs	x2, midr_el1
	msr	vpidr_el2, x2

	/* Mirror the Virtualization Multiprocess ID Register. */
	mrs	x2, mpidr_el1
	msr	vmpidr_el2, x2

	/* Set the bits that need to be 1 in SCTLR_EL2. */
	ldr	x2, .Lsctlr_el2_res1
	mrs	x1, sctlr_el2
	and	x1, x1, #(SCTLR_EE | SCTLR_E0E)	/* keep SCTLR_EL2.{EE,E0E} */
	orr	x2, x2, x1
	msr	sctlr_el2, x2

	/* Set the bits that need to be 1 in SCTLR_EL1. */
	ldr	x2, .Lsctlr_el1_res1
	mrs	x1, sctlr_el1
	and	x1, x1, #(SCTLR_EE | SCTLR_E0E)	/* keep SCTLR_EL1.{EE,E0E} */
	orr	x2, x2, x1
	msr	sctlr_el1, x2

	/* Don't trap to EL2 on FP instructions. */
	mrs	x2, cpacr_el1
	bic	x2, x2, #CPACR_FPEN
	orr	x2, x2, #CPACR_FPEN_ALL
	msr	cpacr_el1, x2
	isb

	/* Don't trap to EL2 on access to various registers. */
	mov	x2, #CPTR_EL2_NVHE_RES1
	msr	cptr_el2, x2

	/* Don't trap to EL2 on CP15 traps. */
	msr	hstr_el2, xzr

	/* Enable access to the physical timers at EL1. */
	mov	x2, #(CNTHCTL_NVHE_EL1PCTEN | CNTHCTL_NVHE_EL1PCEN)
	msr	cnthctl_el2, x2

	/* Set the counter offset to a known value. */
	msr	cntvoff_el2, xzr

	/* Set the hypervisor trap vectors. */
	adr	x2, hyp_vectors
	msr	vbar_el2, x2

	/* Configure GICv3 CPU interface if available */
	mrs	x2, id_aa64pfr0_el1

	/* Extract GIC bits from the register */
	and	x2, x2, #ID_AA64PFR0_EL1_GIC
	lsr	x2, x2, #ID_AA64PFR0_EL1_GIC_SHIFT
	/* GIC[3:0] == 0001 - GIC CPU interface via special regs. supported */
	cmp	x2, #ID_AA64PFR0_EL1_GIC_CPUIF_EN
	b.ne	2f

	mrs	x2, icc_sre_el2			/* Enable... */
	orr	x2, x2, #ICC_SRE_EL2_EN		/*  access from insecure EL1 */
	orr	x2, x2, #ICC_SRE_EL2_SRE	/*  system registers */
	msr	icc_sre_el2, x2
2:

#if NNVMM > 0
	/* nvmm_available = 1 */
	adrl	x0, _C_LABEL(nvmm_available)
	mov	x1, #1
	str	w1, [x0]

	/*
	 * Set new stackpointer for EL2 trap handlers.
	 * sp = sp_el2 = &el2stack + cpuindex * PAGE_SIZE;
	 *
	 * cpuindex is [0, MAXCPUS - 1] and el2stack is placed after
	 * cpu0's page.
	 */
	mov	x1, #PAGE_SIZE
	mul	x1, x1, x27
	adrl	x0, el2stack
	add	sp, x0, x1

	/* set current VMID zero */
	msr	vttbr_el2, xzr

	/*
	 * NVMM uses tpidr_el2 as struct cpu_info *, but is initialised in
	 * aarch64_el2_init.
	 */
	msr	tpidr_el2, xzr
#endif

#endif	/* ARMV80_NVHE */

	/*
	 * Prepare to leave EL2 permanently.
	 * Copy our current active stack pointer directly to SP_EL1 so the
	 * receiving EL1 kernel has a valid stack execution context immediately.
	 */
	mov	x0, sp
	msr	sp_el1, x0

	/*
	 * DAIF = {1,1,1,1} thanks to boot protocol, so it's safe to set {elr,spsr}_el2
	 * Set the address to return to.
	 */
	msr	elr_el2, lr
	mov	x2, #(SPSR_F | SPSR_I | SPSR_A | SPSR_A64_D | SPSR_M_EL1H)
	msr	spsr_el2, x2

	/* Use DPRINTSREG for BP and CPU_DPRINTSREG for APs */
	cmp	x27, #0
	b.ne	3f

	DPRINT("\n")
	DPRINTSREG("SCTLR_EL2        = ", sctlr_el2)
	DPRINTSREG("TCR_EL2          = ", tcr_el2)
	DPRINTSREG("HCR_EL2          = ", hcr_el2)
	DPRINTSREG("CNTHCTL_EL2      = ", cnthctl_el2)
	DPRINTSREG("VTTBR_EL2        = ", vttbr_el2)
	DPRINTSREG("MAIR_EL2         = ", mair_el2)
	DPRINTSREG("TPIDR_EL2        = ", tpidr_el2)
	DPRINTSREG("DAIF             = ", daif)

	DPRINTREG("SP (EL2)         = ", sp)

	mrs	x2, id_aa64pfr0_el1
	/* Extract GIC bits from the register */
	and	x2, x2, #ID_AA64PFR0_EL1_GIC
	lsr	x2, x2, #ID_AA64PFR0_EL1_GIC_SHIFT
	/* GIC[3:0] == 0001 - GIC CPU interface via special regs. supported */
	cmp	x2, #ID_AA64PFR0_EL1_GIC_CPUIF_EN
	b.ne	4f

	DPRINTSREG("ICC_SRE_EL2      = ", icc_sre_el2)
	DPRINTSREG("ICH_HCR_EL2      = ", ich_hcr_el2)
	DPRINTSREG("ICH_VTR_EL2      = ", ich_vtr_el2)

4:
	/*
	 * An isb is not required before eret if SCTLR_EL2.EOS=1 (RES1)
	 */
	eret

3:
	CPU_DPRINTSREG("SCTLR_EL2        = ", sctlr_el2)
	CPU_DPRINTSREG("TCR_EL2          = ", tcr_el2)
	CPU_DPRINTSREG("HCR_EL2          = ", hcr_el2)
	CPU_DPRINTSREG("CNTHCTL_EL2      = ", cnthctl_el2)
	CPU_DPRINTSREG("VTTBR_EL2        = ", vttbr_el2)
	CPU_DPRINTSREG("MAIR_EL2         = ", mair_el2)
	CPU_DPRINTSREG("TPIDR_EL2        = ", tpidr_el2)
	CPU_DPRINTSREG("DAIF             = ", daif)

	CPU_DPRINTREG("SP (EL2)         = ", sp)

	mrs	x2, id_aa64pfr0_el1
	/* Extract GIC bits from the register */
	and	x2, x2, #ID_AA64PFR0_EL1_GIC
	lsr	x2, x2, #ID_AA64PFR0_EL1_GIC_SHIFT
	/* GIC[3:0] == 0001 - GIC CPU interface via special regs. supported */
	cmp	x2, #ID_AA64PFR0_EL1_GIC_CPUIF_EN
	b.ne	5f

	CPU_DPRINTSREG("ICC_SRE_EL2      = ", icc_sre_el2)
	CPU_DPRINTSREG("ICH_HCR_EL2      = ", ich_hcr_el2)
	CPU_DPRINTSREG("ICH_VTR_EL2      = ", ich_vtr_el2)

5:
	/*
	 * An isb is not required before eret if
	 * SCTLR_EL2.EOS=1 [bit 11] (RES1)
	 */
	// BP will eret to drop_to_el1 call site in drop_to_el1_inline
	// APs will eret to drop_to_el1 call site in cpu_mpstart
	eret


vhe_return:
	/*
	 * Prepare to leave EL2 permanently.
	 * Copy our current active stack pointer directly to SP_EL1 so the
	 * receiving EL1 kernel has a valid stack execution context immediately.
	 */
	mov	x0, sp
	msr	sp_el1, x0

	mov	x9, lr

	/* Use DPRINTSREG for BP and CPU_DPRINTSREG for APs */
	cmp	x27, #0
	b.ne	3f

	DPRINT("\n")
	DPRINTSREG("SCTLR_EL2        = ", sctlr_el2)
	DPRINTSREG("TCR_EL2          = ", tcr_el2)
	DPRINTSREG("HCR_EL2          = ", hcr_el2)
	DPRINTSREG("VTTBR_EL2        = ", vttbr_el2)
	DPRINTSREG("MAIR_EL2         = ", mair_el2)
	DPRINTSREG("TPIDR_EL2        = ", tpidr_el2)
	DPRINTSREG("DAIF             = ", daif)

	DPRINTREG("SP (EL2)         = ", sp)

	mrs	x2, id_aa64pfr0_el1
	/* Extract GIC bits from the register */
	and	x2, x2, #ID_AA64PFR0_EL1_GIC
	lsr	x2, x2, #ID_AA64PFR0_EL1_GIC_SHIFT
	/* GIC[3:0] == 0001 - GIC CPU interface via special regs. supported */
	cmp	x2, #ID_AA64PFR0_EL1_GIC_CPUIF_EN
	b.ne	4f

	DPRINTSREG("ICC_SRE_EL2      = ", icc_sre_el2)
	DPRINTSREG("ICH_HCR_EL2      = ", ich_hcr_el2)
	DPRINTSREG("ICH_VTR_EL2      = ", ich_vtr_el2)

4:
	mov	lr, x9
	ret
3:
	CPU_DPRINTSREG("SCTLR_EL2        = ", sctlr_el2)
	CPU_DPRINTSREG("TCR_EL2          = ", tcr_el2)
	CPU_DPRINTSREG("HCR_EL2          = ", hcr_el2)
	CPU_DPRINTSREG("VTTBR_EL2        = ", vttbr_el2)
	CPU_DPRINTSREG("MAIR_EL2         = ", mair_el2)
	CPU_DPRINTSREG("TPIDR_EL2        = ", tpidr_el2)
	CPU_DPRINTSREG("DAIF             = ", daif)

	CPU_DPRINTREG("SP (EL2)         = ", sp)

	mrs	x2, id_aa64pfr0_el1
	/* Extract GIC bits from the register */
	and	x2, x2, #ID_AA64PFR0_EL1_GIC
	lsr	x2, x2, #ID_AA64PFR0_EL1_GIC_SHIFT
	/* GIC[3:0] == 0001 - GIC CPU interface via special regs. supported */
	cmp	x2, #ID_AA64PFR0_EL1_GIC_CPUIF_EN
	b.ne	5f

	CPU_DPRINTSREG("ICC_SRE_EL2      = ", icc_sre_el2)
	CPU_DPRINTSREG("ICH_HCR_EL2      = ", ich_hcr_el2)
	CPU_DPRINTSREG("ICH_VTR_EL2      = ", ich_vtr_el2)

5:
	ret


#if NNVMM > 0
/*
 * aarch64_e2h_vmenter(struct aarch64_cpudata *cpudata)
 */
ENTRY_NP(aarch64_e2h_vmenter)
	stp	x0, x1, [sp, #-TF_SIZE]!		// not needed callee saved only
	stp	x2, x3, [sp, #TF_X2]			// ...
	stp	x4, x5, [sp, #TF_X4]
	stp	x6, x7, [sp, #TF_X6]
	stp	x8, x9, [sp, #TF_X8]
	stp	x10, x11, [sp, #TF_X10]
	stp	x12, x13, [sp, #TF_X12]
	stp	x14, x15, [sp, #TF_X14]
	stp	x16, x17, [sp, #TF_X16]
	stp	x18, x19, [sp, #TF_X18]			// x19 ...
	stp	x20, x21, [sp, #TF_X20]
	stp	x22, x23, [sp, #TF_X22]
	stp	x24, x25, [sp, #TF_X24]
	stp	x26, x27, [sp, #TF_X26]
	stp	x28, x29, [sp, #TF_X28]			// ... X28 and FP and LR are callee saved

//	CTASSERT_ADJACENT(TF_PC, TF_SPSR)
	mov 	x21, lr
	mov     x22, #SPSR_M_EL1H		// What is it really? Maybe it doesn't matter.
	stp	x21, x22, [sp, #TF_PC]

	add	x8, sp, #TF_SIZE

//	CTASSERT_ADJACENT(TF_X30, TF_SP)
	stp	lr, x8, [sp, #TF_X30]

#if 0
	CTASSERT_ADJACENT(TF_ESR, TF_FAR)
	mrs	x23, esr_el2
	mrs	x24, far_el2
	stp	x23, x24, [sp, #TF_ESR]
#endif

	mov	x1, x0 				/* second argument = cpudata */
	mov	x0, sp				/* first argument = tf */

	msr	daifset, #__SHIFTOUT(DAIF_D | DAIF_A | DAIF_I | DAIF_F, DAIF_MASK)

	/*
	 * Once the guest system registers are all in place after
	 * aarch64_el2_vmenter_context call into el2_enter_guest which
	 * will load the general registers from the trapframe and eret
	  to the guest.
	 */
	adr 	lr, el2_enter_guest

	b	aarch64_el2_vmenter_context
END(aarch64_e2h_vmenter)
#endif

	.align 3
.Lsctlr_el1_res1:
	.quad SCTLR_RES1
.Lsctlr_el2_res1:
	.quad (								\
	    __BIT(28) | /* nTLSMD when FEAT_LSMAOC */			\
	    __BIT(23) | /* SPAN */					\
	    __BIT(22) | /* EIS */					\
	    __BIT(18) | /* nTWE */					\
	    __BIT(16) | /* nTWI */					\
	    __BIT(11) | /* EOS */					\
	     __BIT(5) | /* CP15BEN */					\
	     __BIT(4)   /* SA0 */					\
	)

drop_to_el1_inline_done:
	nop

#if NNVMM > 0
	.pushsection .bss
	.align PGSHIFT
	.space	PAGE_SIZE
el2stack:
#ifdef MULTIPROCESSOR
	.space	PAGE_SIZE * (MAXCPUS - 1)
#endif
	.popsection
#endif /* NNVMM */
