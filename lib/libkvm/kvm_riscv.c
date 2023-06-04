/*	$NetBSD: kvm_riscv.c,v 1.4 2024/10/12 12:19:16 skrll Exp $	*/

/*-
 * Copyright (c) 2014 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Matt Thomas of 3am Software Foundry.
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * OR1K machine dependent routines for kvm.
 */

#include <sys/param.h>

#include <sys/bitops.h>
#include <sys/exec.h>
#include <sys/types.h>

#include <uvm/uvm_extern.h>

#include <db.h>
#include <limits.h>
#include <kvm.h>
#include <stdlib.h>
#include <unistd.h>

#include "kvm_private.h"

#include <sys/kcore.h>
#include <machine/kcore.h>
#include <machine/pmap.h>
#include <machine/pte.h>
#include <machine/sysreg.h>
#include <machine/vmparam.h>

__RCSID("$NetBSD: kvm_riscv.c,v 1.4 2024/10/12 12:19:16 skrll Exp $");

void
_kvm_freevtop(kvm_t *kd)
{
	if (kd->vmst != 0)
		free(kd->vmst);
}

/*ARGSUSED*/
int
_kvm_initvtop(kvm_t *kd)
{

	return 0;
}

/*
 * Translate a KVA to a PA
 */
int
_kvm_kvatop(kvm_t *kd, vaddr_t va, paddr_t *pa)
{
	if (ISALIVE(kd)) {
		_kvm_err(kd, 0, "vatop called in live kernel!");
		return 0;
	}

	const cpu_kcore_hdr_t * const cpu_kh = kd->cpu_data;
	const u_long satp = cpu_kh->kh_satp;

	size_t topbit = sizeof(long) * NBBY - 1;

	paddr_t pte_addr = __SHIFTOUT(satp, SATP_PPN) << PGSHIFT;
	const uint8_t mode = __SHIFTOUT(satp, SATP_MODE);
	u_int levels = 1;

	switch (mode) {
#ifdef _LP64
	case SATP_MODE_SV39:
	case SATP_MODE_SV48:
		topbit = (39 - 1) + (mode - 8) * SEGLENGTH;
		levels = mode - 6;
		break;
#else
	case SATP_MODE_SV32:
		topbit = 32;
		levels = 2;
		break;
#endif
	default:
		goto fail;
	}
#if 0
	(*pr)("topbit = %zu\n", topbit);

	(*pr)("satp   = 0x%" PRIxREGISTER "\n", satp);
#endif
	/*
	 * Real kernel virtual address: do the translation.
	 */

	const u_int page_shift = 12;
//	const size_t page_size = 1 << page_shift;
	const uint64_t page_mask = __BITS(page_shift - 1, 0);
	const uint64_t page_addr = __BITS(topbit, page_shift);
	const u_int pte_shift = page_shift - ilog2(sizeof(long));

	/* restrict va to the valid VA bits */
	va &= page_mask;

	u_int addr_shift = page_shift + (levels - 1) * pte_shift;


	for (;;) {
		pt_entry_t pte;

		/* now index into the pte table */
		const uint64_t idx_mask = __BITS(addr_shift + pte_shift - 1,
						 addr_shift);
		pte_addr += 8 * __SHIFTOUT(va, idx_mask);

		/* Find and read the PTE. */
		if (_kvm_pread(kd, kd->pmfd, &pte, sizeof(pte),
		    _kvm_pa2off(kd, pte_addr)) != sizeof(pte)) {
			_kvm_syserr(kd, 0, "could not read pte");
			goto fail;
		}

		/* Find and read the L2 PTE. */
		if ((pte & PTE_V) == 0) {
			_kvm_err(kd, 0, "invalid translation (invalid pte)");
			goto fail;
		}
#if 0
		if ((pte & LX_TYPE) == LX_TYPE_BLK) {
			const size_t blk_size = 1 << addr_shift;
			const uint64_t blk_mask = __BITS(addr_shift - 1, 0);

			*pa = (pte & page_addr & ~blk_mask) | (va & blk_mask);
			return blk_size - (va & blk_mask);
		}
		if (--levels == 0) {
			*pa = (pte & page_addr) | (va & page_mask);
			return page_size - (va & page_mask);
		}
#endif
		/*
		 * Read next level of page table
		 */

		pte_addr = pte & page_addr;
		addr_shift -= pte_shift;
	}

fail:
	/* No hit -- no translation */
	*pa = ~0UL;

	return 0;
}

off_t
_kvm_pa2off(kvm_t *kd, paddr_t pa)
{
	cpu_kcore_hdr_t	*cpu_kh;
	phys_ram_seg_t	*ram;
	off_t		off;
	void		*e;

	cpu_kh = kd->cpu_data;
	e = (char *) kd->cpu_data + kd->cpu_dsize;
	ram = (void *)((char *)(void *)cpu_kh + ALIGN(sizeof *cpu_kh));
	off = kd->dump_off;
	do {
		if (pa >= ram->start && (pa - ram->start) < ram->size) {
			return off + (pa - ram->start);
		}
		ram++;
		off += ram->size;
	} while ((void *) ram < e && ram->size);

	_kvm_err(kd, 0, "pa2off failed for pa %#" PRIxPADDR "\n", pa);
	return (off_t) -1;
}

/*
 * Machine-dependent initialization for ALL open kvm descriptors,
 * not just those for a kernel crash dump.  Some architectures
 * have to deal with these NOT being constants!  (i.e. m68k)
 */
int
_kvm_mdopen(kvm_t *kd)
{
	kd->min_uva = VM_MIN_ADDRESS;
	kd->max_uva = VM_MAXUSER_ADDRESS;

	return (0);
}
