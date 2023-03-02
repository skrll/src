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

int uartprintf(const char * restrict, ...);
void dump_el2_trapframe(struct trapframe *tf);
void aarch64_el2_mmu_enable(paddr_t);
void aarch64_el2_init(struct trapframe *);
void aarch64_el2_vmenter(struct trapframe *);
void aarch64_el2_vmexit(struct trapframe *);
void aarch64_el2_vmexit_irq(struct trapframe *);

static void __unused
xxx_hexdump(void *addr, unsigned int len)
{
	uint8_t *p = (uint8_t *)addr;
	unsigned int i;
	for (i = 0; i < len; i++) {
		if ((i & 15) == 0)
			uartprintf("%p:", &p[i]);
		uartprintf(" %02x", p[i]);
		if ((i & 15) == 15)
			uartprintf("\n");
	}
	if ((i & 15) != 0)
		uartprintf("\n");
}

void
aarch64_el2_init(struct trapframe *tf)
{
	/* for EL0/1 stage2 translate configuration */
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

	uartprintf("%s: startlevel=%d, parange=%d\n", __func__, startlevel, parange);


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
	    __BIT(31) |
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
	 * Enable EL2 MMU (VA=PA).
	 * Not a stage2 translation for EL0/1.
	 *
	 * If EL2 is not run with MMU enable, cache does not work on EL2
	 * or exclusive load/store cannot be used.
	 */
	aarch64_el2_mmu_enable(tf->tf_reg[0]);
}

static void
vcpu_context_save(struct trapframe *tf, struct nvmm_aarch64_state *state)
{
	state->sprs[NVMM_AARCH64_SPR_PC] = tf->tf_pc;
	state->sprs[NVMM_AARCH64_SPR_SPSR_EL1] = tf->tf_spsr;

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

	memcpy(tf->tf_reg, state->gprs, sizeof(state->gprs));

#if 0
	uartprintf("load: PC   = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_PC]);
	uartprintf("load: SPSR = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_SPSR_EL1]);

	uartprintf("load: TPIDRRO_EL0    = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_TPIDRRO_EL0]);
	uartprintf("load: TPIDR_EL0      = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_TPIDR_EL0]);
	uartprintf("load: AMAIR_EL1      = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_AMAIR_EL1]);
	uartprintf("load: CNTKCTL_EL1    = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_CNTKCTL_EL1]);
	uartprintf("load: CONTEXTIDR_EL1 = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_CONTEXTIDR_EL1]);
	uartprintf("load: CPACR_EL1      = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_CPACR_EL1]);
	uartprintf("load: CSSELR_EL1     = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_CSSELR_EL1]);
	uartprintf("load: ELR_EL1        = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_ELR_EL1]);
	uartprintf("load: ESR_EL1        = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_ESR_EL1]);
	uartprintf("load: FAR_EL1        = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_FAR_EL1]);
	uartprintf("load: MAIR_EL1       = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_MAIR_EL1]);
	uartprintf("load: MDSCR_EL1      = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_MDSCR_EL1]);
	uartprintf("load: PAR_EL1        = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_PAR_EL1]);
	uartprintf("load: SCTLR_EL1      = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_SCTLR_EL1]);
	uartprintf("load: SP_EL1         = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_SP_EL1]);
	uartprintf("load: TCR_EL1        = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_TCR_EL1]);
	uartprintf("load: TPIDR_EL1      = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_TPIDR_EL1]);
	uartprintf("load: TTBR0_EL1      = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_TTBR0_EL1]);
	uartprintf("load: TTBR1_EL1      = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_TTBR1_EL1]);
	uartprintf("load: VBAR_EL1       = 0x%016x\n", state->sprs[NVMM_AARCH64_SPR_VBAR_EL1]);
#endif

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

void
aarch64_el2_vmexit_irq(struct trapframe *tf)
{
	struct aarch64_cpudata *cpudata_pa;
	struct nvmm_aarch64_exit *exit_pa;

//	dump_el2_trapframe(tf);

	cpudata_pa = (struct aarch64_cpudata *)reg_tpidr_el2_read();
	exit_pa = (struct nvmm_aarch64_exit *)cpudata_pa->exit_pa;

	exit_pa->reason = NVMM_VCPU_EXIT_HALTED;
//	exit_pa->u.mem
//	exit_pa->u.insn
//	exit_pa->u.inv

	reg_hcr_el2_write(HCR_EL2_RW);
	reg_hstr_el2_write(0);
//	reg_vtcr_el2_write(0);

//	uartprintf("%s: save guest\n", __func__);
	vcpu_context_save(tf, &cpudata_pa->guest);
//	uartprintf("%s: load host\n", __func__);
	vcpu_context_load(tf, &cpudata_pa->host);
//	uartprintf("%s: load done\n", __func__);
}

void
aarch64_el2_vmexit(struct trapframe *tf)
{
	struct aarch64_cpudata *cpudata_pa;
	struct nvmm_aarch64_exit *exit_pa;
	const uint32_t esr = tf->tf_esr;
	const uint32_t eclass = __SHIFTOUT(esr, ESR_EC);
	uint32_t rw;
	vm_prot_t ftype;

	rw = __SHIFTOUT(esr, ESR_ISS_DATAABORT_WnR); /* 0 if IFSC */

	cpudata_pa = (struct aarch64_cpudata *)reg_tpidr_el2_read();
	exit_pa = (struct nvmm_aarch64_exit *)cpudata_pa->exit_pa;

	switch (eclass) {
	case ESR_EC_INSN_ABT_EL_LOW:
	case ESR_EC_DATA_ABT_EL_LOW:
//		dump_el2_trapframe(tf);
//		uartprintf("hpfar_el2:%016x (%016x)\n", reg_hpfar_el2_read(), reg_hpfar_el2_read() << 8);
//		uartprintf("sctlr_el1:%016x\n", reg_sctlr_el1_read());
//		uartprintf("tcr_el1:%016x\n", reg_tcr_el1_read());

		if (eclass == ESR_EC_INSN_ABT_EL_LOW)
			ftype = VM_PROT_EXECUTE;
		else if (__SHIFTOUT(esr, ESR_ISS_DATAABORT_CM))
			ftype = VM_PROT_READ;
		else
			ftype = (rw == 0) ? VM_PROT_READ : VM_PROT_WRITE;
		exit_pa->reason = NVMM_VCPU_EXIT_MEMORY;
		exit_pa->u.mem.gpa = tf->tf_far;
		exit_pa->u.mem.prot = ftype;
		break;

	case ESR_EC_INSN_ABT_EL_CUR:
	case ESR_EC_DATA_ABT_EL_CUR:
		uartprintf("%s: INSN or DATA ABORT occured on EL2?\n", __func__);
		//panic. never occur data abort on EL2...
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


	reg_hcr_el2_write(HCR_EL2_RW);
	reg_hstr_el2_write(0);
	reg_vttbr_el2_write(0);

	/* save guest state */
	vcpu_context_save(tf, &cpudata_pa->guest);
	/* lost host state */
	vcpu_context_load(tf, &cpudata_pa->host);
}

void
aarch64_el2_vmenter(struct trapframe *tf)
{
	struct aarch64_cpudata *cpudata_pa;
//	struct nvmm_aarch64_exit *exit_pa;

	cpudata_pa = (struct aarch64_cpudata *)tf->tf_reg[0];
//	exit_pa = (struct aarch64_exit *)cpudata_pa->exit_pa;

#if 0
	uartprintf("%s: cpudata_pa=%p\n", __func__, cpudata_pa);

	uartprintf("    x0=%016lx,     x1=%016lx\n", cpudata_pa->guest.gprs[0], cpudata_pa->guest.gprs[1]);
	uartprintf("    x2=%016lx,     x3=%016lx\n", cpudata_pa->guest.gprs[2], cpudata_pa->guest.gprs[3]);
	uartprintf("    x4=%016lx,     x5=%016lx\n", cpudata_pa->guest.gprs[4], cpudata_pa->guest.gprs[5]);
	uartprintf("    x6=%016lx,     x7=%016lx\n", cpudata_pa->guest.gprs[6], cpudata_pa->guest.gprs[7]);
	uartprintf("    x8=%016lx,     x9=%016lx\n", cpudata_pa->guest.gprs[8], cpudata_pa->guest.gprs[9]);
	uartprintf("   x10=%016lx,    x11=%016lx\n", cpudata_pa->guest.gprs[10], cpudata_pa->guest.gprs[11]);
	uartprintf("   x12=%016lx,    x13=%016lx\n", cpudata_pa->guest.gprs[12], cpudata_pa->guest.gprs[13]);
	uartprintf("   x14=%016lx,    x15=%016lx\n", cpudata_pa->guest.gprs[14], cpudata_pa->guest.gprs[15]);
	uartprintf("   x16=%016lx,    x17=%016lx\n", cpudata_pa->guest.gprs[16], cpudata_pa->guest.gprs[17]);
	uartprintf("   x18=%016lx,    x19=%016lx\n", cpudata_pa->guest.gprs[18], cpudata_pa->guest.gprs[19]);
	uartprintf("   x20=%016lx,    x21=%016lx\n", cpudata_pa->guest.gprs[20], cpudata_pa->guest.gprs[21]);
	uartprintf("   x22=%016lx,    x23=%016lx\n", cpudata_pa->guest.gprs[22], cpudata_pa->guest.gprs[23]);
	uartprintf("   x24=%016lx,    x25=%016lx\n", cpudata_pa->guest.gprs[24], cpudata_pa->guest.gprs[25]);
	uartprintf("   x26=%016lx,    x27=%016lx\n", cpudata_pa->guest.gprs[26], cpudata_pa->guest.gprs[27]);
	uartprintf("   x28=%016lx, fp=x29=%016lx\n", cpudata_pa->guest.gprs[28], cpudata_pa->guest.gprs[29]);
	uartprintf("lr=x30=%016lx,     sp=%016lx\n", cpudata_pa->guest.gprs[30], cpudata_pa->guest.gprs[31]);
	uartprintf("    PC=%016lx\n", cpudata_pa->guest.sprs[NVMM_AARCH64_SPR_PC]);
#endif

	reg_tpidr_el2_write((register_t)cpudata_pa);

//	uartprintf("%s: save host\n", __func__);
	vcpu_context_save(tf, &cpudata_pa->host);
//	uartprintf("%s: load guest\n", __func__);
	vcpu_context_load(tf, &cpudata_pa->guest);
//	uartprintf("%s: load done\n", __func__);

	//set VTTBR_EL2 from cpudata->vttbr_el2
	//enable VTCR_EL2

	uint64_t hcr = HCR_EL2_RW;	/* 64bit */
	hcr |= HCR_EL2_ID;
	hcr |= HCR_EL2_CD;
//	hcr |= HCR_EL2_TRVM;
	hcr |= HCR_EL2_HCD;	/* disable hvc */
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
	reg_hcr_el2_write(hcr);
	reg_hstr_el2_write(0xffff);

//	uartprintf("%s: VTTBR_EL2 = %016lx\n", __func__, cpudata_pa->vttbr_el2);
	reg_vttbr_el2_write(cpudata_pa->vttbr_el2);
//	uartprintf("vttbr_el2:%016x\n", reg_vttbr_el2_read());
//	uartprintf("vtcr_el2:%016x\n", reg_vtcr_el2_read());

	asm("dsb ishst");
	asm("ic ialluis");
	asm("tlbi vmalle1is");
	asm("tlbi vmalls12e1is");
	asm("dsb ish");
	asm("isb");
}
