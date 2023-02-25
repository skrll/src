/*	$NetBSD$	*/

/*-
 * Copyright (c) 202e Ryo Shimizu <ryo@nerv.org>
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

#ifndef _NVMM_AARCH64_INTERNAL_H_
#define _NVMM_AARCH64_INTERNAL_H_


static inline struct cpu_info *
aarch64nvmm_curcpu(void)
{
	return (struct cpu_info *)reg_tpidr_el2_read();
}

static inline u_int
aarch64nvmm_cpu_number(void)
{
        return aarch64nvmm_curcpu()->ci_index;
}

struct aarch64_cpudata {
	uint64_t cpudata_pa;
	uint64_t vttbr_el2;
	uint64_t send_event_type;
	uint64_t send_event_esr;
	bool evt_pending;

	struct nvmm_aarch64_exit exit;
	struct nvmm_aarch64_state host;
	struct nvmm_aarch64_state guest;
};

char *uartputs(const char *);
int uartprintf(const char * restrict, ...) __printflike(1, 2);

paddr_t aarch64_gva_to_pa(uint64_t, vaddr_t);
paddr_t aarch64_gva_to_ipa(uint64_t, vaddr_t);
paddr_t aarch64_get_fault_ipa(struct trapframe *);
static inline paddr_t
ipa_hpfar_far(vaddr_t hpfar, vaddr_t far)
{
	return ((__SHIFTOUT(hpfar, HPFAR_EL2_FIPA) << HPFAR_EL2_FIPA_BITSHIFT) &
	    ~PAGE_MASK) | (far & PAGE_MASK);
}

void dump_el2_trapframe(struct trapframe *);
void aarch64_el2_init(struct trapframe *);
void aarch64_el2_vmenter(struct trapframe *);
void aarch64_el2_vmexit_trap(struct trapframe *tf);
void aarch64_el2_vmexit_irq(struct trapframe *tf);
void aarch64_el2_maintain_ipa(struct trapframe *tf);

void nvmm_aarch64_load_fpregs(const union fpelem *);
void nvmm_aarch64_save_fpregs(union fpelem *);

extern int aarch64_el2_initted;
extern int nvmm_uartdebug;

#endif /* _NVMM_AARCH64_INTERNAL_H_ */
