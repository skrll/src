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

#ifndef _NVMM_AARCH64_H_
#define _NVMM_AARCH64_H_

#include <aarch64/reg.h>

/* Generic. */
#define NVMM_VCPU_EXIT_NONE		0x0000000000000000ULL
#define NVMM_VCPU_EXIT_STOPPED		0xfffffffffffffffeULL
#define NVMM_VCPU_EXIT_INVALID		0xffffffffffffffffULL
/* aarch64: operations. */
#define NVMM_VCPU_EXIT_MEMORY		0x0000000000000001ULL
/* aarch64: changes in VCPU state. */
#define NVMM_VCPU_EXIT_SHUTDOWN		0x0000000000001000ULL
#define NVMM_VCPU_EXIT_IRQ		0x0000000000001001ULL
#define NVMM_VCPU_EXIT_HALTED		0x0000000000001003ULL
/* aarch64: instructions. */
#define NVMM_VCPU_EXIT_MSR		0x0000000000002000ULL
#define NVMM_VCPU_EXIT_MRS		0x0000000000002001ULL
#define NVMM_VCPU_EXIT_WFI		0x0000000000002010ULL
#define NVMM_VCPU_EXIT_WFE		0x0000000000002011ULL
#define NVMM_VCPU_EXIT_HVC		0x0000000000002020ULL
#define NVMM_VCPU_EXIT_SMC		0x0000000000002021ULL

struct cpu_info;

struct nvmm_aarch64_el2_init {
	struct cpu_info	*ni_cpuinfo;
	uint64_t	 ni_ttbr0_el2;
	uint64_t	 ni_vtcr_el2;
};

struct nvmm_aarch64_exit_memory {
	gpaddr_t gpa;
	int prot;
};

struct nvmm_aarch64_exit {
	uint64_t reason;
	union {
		struct nvmm_aarch64_exit_memory mem;
	} u;
	uint64_t esr;
	uint32_t insn;		// Used by the MMIO emulation code
        struct {
		uint64_t evt_pending:1;
		uint64_t vtimer:1;
		uint64_t rsvd:62;
        } exitstate;
};

// XXXNH make a bit mask for {SERROR,IRQ,FIQ} so >1 event
// XXXNH can be sent.
struct nvmm_aarch64_event {
	u_int type;
#define NVMM_VCPU_EVENT_NONE	0
#define NVMM_VCPU_EVENT_SYNC	1	// probably not
#define NVMM_VCPU_EVENT_SERROR	2
#define NVMM_VCPU_EVENT_IRQ	3
#define NVMM_VCPU_EVENT_FIQ	4
	uint64_t esr;	// SYNC only
};

#define NVMM_AARCH64_GPR_X0		0
#define NVMM_AARCH64_GPR_X1		1
#define NVMM_AARCH64_GPR_X2		2
#define NVMM_AARCH64_GPR_X3		3
#define NVMM_AARCH64_GPR_X4		4
#define NVMM_AARCH64_GPR_X5		5
#define NVMM_AARCH64_GPR_X6		6
#define NVMM_AARCH64_GPR_X7		7
#define NVMM_AARCH64_GPR_X8		8
#define NVMM_AARCH64_GPR_X9		9
#define NVMM_AARCH64_GPR_X10		10
#define NVMM_AARCH64_GPR_X11		11
#define NVMM_AARCH64_GPR_X12		12
#define NVMM_AARCH64_GPR_X13		13
#define NVMM_AARCH64_GPR_X14		14
#define NVMM_AARCH64_GPR_X15		15
#define NVMM_AARCH64_GPR_X16		16
#define NVMM_AARCH64_GPR_X17		17
#define NVMM_AARCH64_GPR_X18		18
#define NVMM_AARCH64_GPR_X19		19
#define NVMM_AARCH64_GPR_X20		20
#define NVMM_AARCH64_GPR_X21		21
#define NVMM_AARCH64_GPR_X22		22
#define NVMM_AARCH64_GPR_X23		23
#define NVMM_AARCH64_GPR_X24		24
#define NVMM_AARCH64_GPR_X25		25
#define NVMM_AARCH64_GPR_X26		26
#define NVMM_AARCH64_GPR_X27		27
#define NVMM_AARCH64_GPR_X28		28
#define NVMM_AARCH64_GPR_X29		29
#define NVMM_AARCH64_GPR_X30		30
#define NVMM_AARCH64_GPR_X31		31	/* SP_EL0 or SP_EL1 */
#define NVMM_AARCH64_NGPR		32

#define NVMM_AARCH64_SPR_PC		0	/* ELR_EL2 */
#define NVMM_AARCH64_SPR_SPSR_EL1	1
/* when VMENTER, X31 has priority according to SPSR */
#define NVMM_AARCH64_SPR_SP_EL0		2
#define NVMM_AARCH64_SPR_SP_EL1		3

#define NVMM_AARCH64_SPR_AMAIR_EL1	4
#define NVMM_AARCH64_SPR_CNTKCTL_EL1	5
#define NVMM_AARCH64_SPR_CONTEXTIDR_EL1	6
#define NVMM_AARCH64_SPR_CPACR_EL1	6
#define NVMM_AARCH64_SPR_CSSELR_EL1	8
#define NVMM_AARCH64_SPR_ELR_EL1	9
#define NVMM_AARCH64_SPR_ESR_EL1	10
#define NVMM_AARCH64_SPR_FAR_EL1	11
#define NVMM_AARCH64_SPR_FPCR		12
#define NVMM_AARCH64_SPR_FPSR		13
#define NVMM_AARCH64_SPR_MAIR_EL1	14
#define NVMM_AARCH64_SPR_MDSCR_EL1	15
#define NVMM_AARCH64_SPR_MIDR_EL1	16	/* VMIDR_EL2 */
#define NVMM_AARCH64_SPR_MPIDR_EL1	17	/* VMPIDR_EL2 */
#define NVMM_AARCH64_SPR_PAR_EL1	18
#define NVMM_AARCH64_SPR_SCTLR_EL1	19
#define NVMM_AARCH64_SPR_TCR_EL1	20
#define NVMM_AARCH64_SPR_TPIDRRO_EL0	21
#define NVMM_AARCH64_SPR_TPIDR_EL0	22
#define NVMM_AARCH64_SPR_TPIDR_EL1	23
#define NVMM_AARCH64_SPR_TTBR0_EL1	24
#define NVMM_AARCH64_SPR_TTBR1_EL1	25
#define NVMM_AARCH64_SPR_VBAR_EL1	26
#define NVMM_AARCH64_SPR_CNTV_CTL_EL0	27
#define NVMM_AARCH64_SPR_CNTV_CVAL_EL0	28
#define NVMM_AARCH64_NSPR		64

#define NVMM_AARCH64_TID_MVFR0_EL1		0	/* HDR_EL2.TID3 */
#define NVMM_AARCH64_TID_MVFR1_EL1		1	/* HDR_EL2.TID3 */
#define NVMM_AARCH64_TID_MVFR2_EL1		2	/* HDR_EL2.TID3 */
#define NVMM_AARCH64_TID_ID_AA64PFR0_EL1	3	/* HDR_EL2.TID3 */
#define NVMM_AARCH64_TID_ID_AA64PFR1_EL1	4	/* HDR_EL2.TID3 */
#define NVMM_AARCH64_TID_ID_AA64DFR0_EL1	5	/* HDR_EL2.TID3 */
#define NVMM_AARCH64_TID_ID_AA64DFR1_EL1	6	/* HDR_EL2.TID3 */
#define NVMM_AARCH64_TID_ID_AA64AFR0_EL1	7	/* HDR_EL2.TID3 */
#define NVMM_AARCH64_TID_ID_AA64AFR1_EL1	8	/* HDR_EL2.TID3 */
#define NVMM_AARCH64_TID_ID_AA64ISAR0_EL1	9	/* HDR_EL2.TID3 */
#define NVMM_AARCH64_TID_ID_AA64ISAR1_EL1	10	/* HDR_EL2.TID3 */
#define NVMM_AARCH64_TID_ID_AA64MMFR0_EL1	11	/* HDR_EL2.TID3 */
#define NVMM_AARCH64_TID_ID_AA64MMFR1_EL1	12	/* HDR_EL2.TID3 */
#define NVMM_AARCH64_TID_ID_AA64MMFR2_EL1	13	/* HDR_EL2.TID3 */
#define NVMM_AARCH64_NTID			32

#define NVMM_AARCH64_FPR_V0		0
#define NVMM_AARCH64_FPR_V1		1
#define NVMM_AARCH64_FPR_V2		2
#define NVMM_AARCH64_FPR_V3		3
#define NVMM_AARCH64_FPR_V4		4
#define NVMM_AARCH64_FPR_V5		5
#define NVMM_AARCH64_FPR_V6		6
#define NVMM_AARCH64_FPR_V7		7
#define NVMM_AARCH64_FPR_V8		8
#define NVMM_AARCH64_FPR_V9		9
#define NVMM_AARCH64_FPR_V10		10
#define NVMM_AARCH64_FPR_V11		11
#define NVMM_AARCH64_FPR_V12		12
#define NVMM_AARCH64_FPR_V13		13
#define NVMM_AARCH64_FPR_V14		14
#define NVMM_AARCH64_FPR_V15		15
#define NVMM_AARCH64_FPR_V16		16
#define NVMM_AARCH64_FPR_V17		17
#define NVMM_AARCH64_FPR_V18		18
#define NVMM_AARCH64_FPR_V19		19
#define NVMM_AARCH64_FPR_V20		20
#define NVMM_AARCH64_FPR_V21		21
#define NVMM_AARCH64_FPR_V22		22
#define NVMM_AARCH64_FPR_V23		23
#define NVMM_AARCH64_FPR_V24		24
#define NVMM_AARCH64_FPR_V25		25
#define NVMM_AARCH64_FPR_V26		26
#define NVMM_AARCH64_FPR_V27		27
#define NVMM_AARCH64_FPR_V28		28
#define NVMM_AARCH64_FPR_V29		29
#define NVMM_AARCH64_FPR_V30		30
#define NVMM_AARCH64_FPR_V31		31
#define NVMM_AARCH64_NFPR		32

/* flags */
#define NVMM_AARCH64_STATE_GPRS		__BIT(0)
#define NVMM_AARCH64_STATE_SPRS		__BIT(1)
#define NVMM_AARCH64_STATE_TIDS		__BIT(2)
#define NVMM_AARCH64_STATE_FPRS		__BIT(16)
#define NVMM_AARCH64_STATE_ALL		\
    (NVMM_AARCH64_STATE_GPRS | NVMM_AARCH64_STATE_SPRS | NVMM_AARCH64_STATE_TIDS | NVMM_AARCH64_STATE_FPRS)

struct nvmm_aarch64_state {
	uint64_t gprs[NVMM_AARCH64_NGPR];
	uint64_t sprs[NVMM_AARCH64_NSPR];
	uint64_t tids[NVMM_AARCH64_NTID];		/* XXX: read only. no need to include in host context */
	union fpelem fprs[NVMM_AARCH64_NFPR];
	// struct nvmm_x64_state_intr intr;
};


struct nvmm_cap_md {
	uint64_t mach_conf_support;
	uint64_t vcpu_conf_support;
};

#define nvmm_vcpu_exit nvmm_aarch64_exit
#define nvmm_vcpu_event nvmm_aarch64_event
#define nvmm_vcpu_state nvmm_aarch64_state

#define NVMM_EL2_HVC_INIT		0x0000
#define NVMM_EL2_HVC_VMENTER		0x0001
#define NVMM_EL2_HVC_MAINTAIN_IPA	0x0100

u_int nvmm_aarch64_vmid(void *);

void nvmm_aarch64_maintain_ipa(void *, uint64_t, uint64_t, uint64_t);
#define NVMM_AARCH64_MAINTAIN_OP_TLBI		__BIT(0)	/* none */
#define NVMM_AARCH64_MAINTAIN_OP_TLBI_ALL	__BIT(1)	/* ipa */
#define NVMM_AARCH64_MAINTAIN_OP_ICACHE_SYNC	__BIT(2)	/* va */

#endif /* _NVMM_AARCH64_H_ */
