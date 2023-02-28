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

#include "opt_multiprocessor.h"

#include <aarch64/asm.h>

#include "assym.h"

RCSID("$NetBSD$")

	ARMV8_DEFINE_OPTIONS
	.text

ENTRY_NP(aarch64_el2_mmu_enable)
	/* if the MMU is already enabled, do nothing */
	mrs	x1, sctlr_el2
	and	x1, x1, #SCTLR_M
	cbz	x1, 1f
	ret
1:

	/* install new TTBR */
	msr	ttbr0_el2, x0

	/* invalidate all TLB */
	dsb	ishst
#ifdef MULTIPROCESSOR
	tlbi	alle2is
#else
	tlbi	alle2
#endif
	dsb	ish
	isb

	mrs	x1, mair_el1
	msr	mair_el2, x1
	isb

	ldr	x1, .Ltcr_el2_setting
	msr	tcr_el2, x1
	isb

	mrs	x1, sctlr_el2
	orr	x1, x1, #SCTLR_I	/* ICache on */
	orr	x1, x1, #SCTLR_C	/* DCache on */
	orr	x1, x1, #SCTLR_M	/* MMU enable */
	msr	sctlr_el2, x1

	ret
END(aarch64_el2_mmu_enable)

	.align 3
.Ltcr_el2_setting:
#define VIRT_BIT	48
	.quad (						\
	   __BIT(31) | __BIT(23) |	/* RES1 */	\
	   __SHIFTIN(64 - VIRT_BIT, TCR_T0SZ) |		\
	   TCR_TG0_4KB |				\
	   TCR_ORGN0_WB_WA |				\
	   TCR_IRGN0_WB_WA |				\
	   TCR_SH0_INNER				\
	)
