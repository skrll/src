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

#include "nvmm.h"

#include <aarch64/asm.h>
#include <aarch64/locore.h>

#include "assym.h"

RCSID("$NetBSD$")

	ARMV8_DEFINE_OPTIONS
	.text

.macro CTASSERT	expr
	.if !(\expr)
	.err
	.endif
.endm

.macro CTASSERT_CONTIGUOUS	p1, p2
	CTASSERT((\p1) + 8 == (\p2))
.endm

#define	VECT_RETURN	\
	.balign 128;	\
	1:	ERET

#define	VECT_HALT	\
	.balign 128;	\
	1:	wfi;	\
		b	1b

#if NNVMM == 0

	.balign 2048
ENTRY_NBTI(hyp_vectors)
	VECT_HALT	/* Synchronous EL2t */
	VECT_HALT	/* IRQ EL2t */
	VECT_HALT	/* FIQ EL2t */
	VECT_HALT	/* Error EL2t */

	VECT_HALT	/* Synchronous EL2h */
	VECT_HALT	/* IRQ EL2h */
	VECT_HALT	/* FIQ EL2h */
	VECT_HALT	/* Error EL2h */

	VECT_HALT	/* Synchronous 64-bit EL1 */
	VECT_HALT	/* IRQ 64-bit EL1 */
	VECT_HALT	/* FIQ 64-bit EL1 */
	VECT_HALT	/* Error 64-bit EL1 */

	VECT_HALT	/* Synchronous 32-bit EL1 */
	VECT_HALT	/* IRQ 32-bit EL1 */
	VECT_HALT	/* FIQ 32-bit EL1 */
	VECT_HALT	/* Error 32-bit EL1 */
END(hyp_vectors)

#else /* NNVMM == 0 */

	.balign 2048
ENTRY_NBTI(hyp_vectors)
	VECT_RETURN	/* Synchronous EL2t */
	VECT_RETURN	/* IRQ EL2t */
	VECT_RETURN	/* FIQ EL2t */
	VECT_RETURN	/* Error EL2t */

	VECT_RETURN	/* Synchronous EL2h */
	VECT_RETURN	/* IRQ EL2h */
	VECT_RETURN	/* FIQ EL2h */
	VECT_RETURN	/* Error EL2h */

	.balign 128	/* Synchronous 64-bit EL1 */
	CTASSERT(TF_X0 == 0)
	stp	x0, x1, [sp], #-TF_SIZE
	stp	x2, x3, [sp, #TF_X2]
	stp	x4, x5, [sp, #TF_X4]
	stp	x6, x7, [sp, #TF_X6]
	stp	x8, x9, [sp, #TF_X8]
	stp	x10, x11, [sp, #TF_X10]
	stp	x12, x13, [sp, #TF_X12]
	stp	x14, x15, [sp, #TF_X14]
	stp	x16, x17, [sp, #TF_X16]
	stp	x18, x19, [sp, #TF_X18]
	stp	x20, x21, [sp, #TF_X20]
	stp	x22, x23, [sp, #TF_X22]
	stp	x24, x25, [sp, #TF_X24]
	stp	x26, x27, [sp, #TF_X26]
	stp	x28, x29, [sp, #TF_X28]

	CTASSERT_CONTIGUOUS(TF_X30, TF_SP)
	mrs	x20, sp_el1
	stp	x30, x20, [sp, #TF_X30]

	CTASSERT_CONTIGUOUS(TF_PC, TF_SPSR)
	mrs	x21, elr_el2
	mrs	x22, spsr_el2
	stp	x21, x22, [sp, #TF_PC]

	CTASSERT_CONTIGUOUS(TF_ESR, TF_FAR)
	mrs	x23, esr_el2
	mrs	x24, far_el2
	stp	x23, x24, [sp, #TF_ESR]
	adr	x30, el2sync_el1_exit	/* return address */
	mov	x0, sp			/* el2sync_el1(tf) */
	b	el2sync_el1

	VECT_RETURN	/* IRQ 64-bit EL1 */
	VECT_RETURN	/* FIQ 64-bit EL1 */
	VECT_RETURN	/* Error 64-bit EL1 */

	VECT_RETURN	/* Synchronous 32-bit EL1 */
	VECT_RETURN	/* IRQ 32-bit EL1 */
	VECT_RETURN	/* FIQ 32-bit EL1 */
	VECT_RETURN	/* Error 32-bit EL1 */

	.balign 64
ENTRY_NP(el2sync_el1_exit)
	ldp	x8, x9, [sp, #TF_X8]
	ldp	x10, x11, [sp, #TF_X10]
	ldp	x12, x13, [sp, #TF_X12]
	ldp	x14, x15, [sp, #TF_X14]
	ldp	x16, x17, [sp, #TF_X16]
	ldp	x18, x19, [sp, #TF_X18]
	ldp	x20, x21, [sp, #TF_X20]
	ldp	x22, x23, [sp, #TF_X22]
	ldp	x24, x25, [sp, #TF_X24]
	ldp	x26, x27, [sp, #TF_X26]
	ldp	x28, x29, [sp, #TF_X28]

	CTASSERT_CONTIGUOUS(TF_X30, TF_SP)
	ldp	x30, x0, [sp, #TF_X30]
	msr	sp_el1, x0

	CTASSERT_CONTIGUOUS(TF_PC, TF_SPSR)
	ldp	x0, x1, [sp, #TF_PC]
	msr	elr_el2, x0
	msr	spsr_el2, x1

	CTASSERT_CONTIGUOUS(TF_ESR, TF_FAR)
	ldp	x0, x1, [sp, #TF_ESR]
	msr	esr_el2, x0
	msr	far_el2, x1

	ldp	x2, x3, [sp, #TF_X2]
	ldp	x4, x5, [sp, #TF_X4]
	ldp	x6, x7, [sp, #TF_X6]
	CTASSERT(TF_X0 == 0)
	ldp	x0, x1, [sp], #TF_SIZE
	isb
	ERET
END(el2sync_el1_exit)

#endif /* NNVMM == 0 */

