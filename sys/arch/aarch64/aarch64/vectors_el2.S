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

#define	VECT_EMPTY	\
	.align 7;	\
	1:	b	1b

	.align 11
ENTRY_NBTI(hyp_vectors)
#if NNVMM > 0
	VECT_EMPTY	/* Synchronous EL2t */
	VECT_EMPTY	/* IRQ EL2t */
	VECT_EMPTY	/* FIQ EL2t */
	VECT_EMPTY	/* Error EL2t */

	VECT_EMPTY	/* Synchronous EL2h */
	VECT_EMPTY	/* IRQ EL2h */
	VECT_EMPTY	/* FIQ EL2h */
	VECT_EMPTY	/* Error EL2h */

	VECT_EMPTY	/* Synchronous 64-bit EL1 */
	VECT_EMPTY	/* IRQ 64-bit EL1 */
	VECT_EMPTY	/* FIQ 64-bit EL1 */
	VECT_EMPTY	/* Error 64-bit EL1 */

	VECT_EMPTY	/* Synchronous 32-bit EL1 */
	VECT_EMPTY	/* IRQ 32-bit EL1 */
	VECT_EMPTY	/* FIQ 32-bit EL1 */
	VECT_EMPTY	/* Error 32-bit EL1 */




#else /* NNVMM > 0 */
	VECT_EMPTY	/* Synchronous EL2t */
	VECT_EMPTY	/* IRQ EL2t */
	VECT_EMPTY	/* FIQ EL2t */
	VECT_EMPTY	/* Error EL2t */

	VECT_EMPTY	/* Synchronous EL2h */
	VECT_EMPTY	/* IRQ EL2h */
	VECT_EMPTY	/* FIQ EL2h */
	VECT_EMPTY	/* Error EL2h */

	VECT_EMPTY	/* Synchronous 64-bit EL1 */
	VECT_EMPTY	/* IRQ 64-bit EL1 */
	VECT_EMPTY	/* FIQ 64-bit EL1 */
	VECT_EMPTY	/* Error 64-bit EL1 */

	VECT_EMPTY	/* Synchronous 32-bit EL1 */
	VECT_EMPTY	/* IRQ 32-bit EL1 */
	VECT_EMPTY	/* FIQ 32-bit EL1 */
	VECT_EMPTY	/* Error 32-bit EL1 */
#endif /* NNVMM > 0 */
END(hyp_vectors)
