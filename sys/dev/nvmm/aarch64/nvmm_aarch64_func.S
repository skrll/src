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

#include <aarch64/asm.h>
#include <aarch64/locore.h>

#include "assym.h"

RCSID("$NetBSD$")

	ARMV8_DEFINE_OPTIONS
	.text

/*
 * called from EL1
 */

/* void aarch64_hvc_init(...) */
ENTRY_NP(aarch64_hvc_init)
	hvc	#NVMM_EL2_HVC_INIT
	ret
END(aarch64_hvc_init)

/* aarch64_hvc_vmenter(x0:cpudata) */
ENTRY_NP(aarch64_hvc_vmenter)
	hvc	#NVMM_EL2_HVC_VMENTER
	ret
END(aarch64_hvc_vmenter)

/* aarch64_hvc_maintain_ipa(x0:cpudata, x1:op, x2:va) */
ENTRY_NP(aarch64_hvc_maintain_ipa)
	hvc	#NVMM_EL2_HVC_MAINTAIN_IPA
	ret
END(aarch64_hvc_vmenter)


/*
 * called from EL2
 */
	.balign	128
ENTRY_NP(nvmm_aarch64_load_fpregs)
	ldp	q0, q1, [x0, #(0 * 16)]
	ldp	q2, q3, [x0, #(2 * 16)]
	ldp	q4, q5, [x0, #(4 * 16)]
	ldp	q6, q7, [x0, #(6 * 16)]
	ldp	q8, q9, [x0, #(8 * 16)]
	ldp	q10, q11, [x0, #(10 * 16)]
	ldp	q12, q13, [x0, #(12 * 16)]
	ldp	q14, q15, [x0, #(14 * 16)]
	ldp	q16, q17, [x0, #(16 * 16)]
	ldp	q18, q19, [x0, #(18 * 16)]
	ldp	q20, q21, [x0, #(20 * 16)]
	ldp	q22, q23, [x0, #(22 * 16)]
	ldp	q24, q25, [x0, #(24 * 16)]
	ldp	q26, q27, [x0, #(26 * 16)]
	ldp	q28, q29, [x0, #(28 * 16)]
	ldp	q30, q31, [x0, #(30 * 16)]
	ret
END(nvmm_aarch64_load_fpregs)

ENTRY_NP(nvmm_aarch64_save_fpregs)
	stp	q0, q1, [x0, #(0 * 16)]
	stp	q2, q3, [x0, #(2 * 16)]
	stp	q4, q5, [x0, #(4 * 16)]
	stp	q6, q7, [x0, #(6 * 16)]
	stp	q8, q9, [x0, #(8 * 16)]
	stp	q10, q11, [x0, #(10 * 16)]
	stp	q12, q13, [x0, #(12 * 16)]
	stp	q14, q15, [x0, #(14 * 16)]
	stp	q16, q17, [x0, #(16 * 16)]
	stp	q18, q19, [x0, #(18 * 16)]
	stp	q20, q21, [x0, #(20 * 16)]
	stp	q22, q23, [x0, #(22 * 16)]
	stp	q24, q25, [x0, #(24 * 16)]
	stp	q26, q27, [x0, #(26 * 16)]
	stp	q28, q29, [x0, #(28 * 16)]
	stp	q30, q31, [x0, #(30 * 16)]
	ret
END(nvmm_aarch64_save_fpregs)
