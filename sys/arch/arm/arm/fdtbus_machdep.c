/*	$NetBSD$	*/

/*-
 * Copyright (c) 2017 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Nick Hudson
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

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>

#include <sys/bus.h>
#include <sys/device.h>
#include <sys/kmem.h>

#include <dev/fdt/fdtvar.h>

#include <arm/fdt/arm_fdtvar.h>

bool
bus_space_is_equal(bus_space_tag_t t1, bus_space_tag_t t2)
{
	//return memcmp(&h1, &h2, sizeof(h1)) == 0;

	/* Rely on iobase checks for comparison */
	return true;
}

/*
 * Translate memory address if needed.
 */
static int
arm_fdtbus_bs_map(void *cookie, bus_addr_t addr, bus_size_t size, int flag,
    bus_space_handle_t *bshp)
{
	// XXX Really should not use simplebus and just just fdtbus_softc

	struct fdtbus_cookie *fc = cookie;
	struct fdtbus_simplebus *fbus = fc->fc_bus;
	struct fdt_softc *sc = container_of(fbus, struct fdt_softc, sc_fbus);
	const int shift = fc->fc_shift;

	uint64_t fdta = addr;

	struct fdt_softc *rsc = fdtbus_decode_range(sc, &fdta, size);
	bus_space_tag_t bst = rsc->sc_fbus.fbus_pshift_bst[shift];

	aprint_debug("\n%s: tag %p/%zu mapping addr %" PRIxBUSADDR
	    " / %" PRIxBUSSIZE " -> %" PRIx64 "\n",  __func__, bst,
	     fc->fc_shift, addr, size, fdta);

	addr = fdta;
	return bus_space_map(bst, addr, size, flag, bshp);
}


static paddr_t
arm_fdtbus_bs_mmap(void *cookie, bus_addr_t addr, off_t offset, int prot,
    int flags)
{
	struct fdtbus_cookie *fc = cookie;
	struct fdtbus_simplebus *fbus = fc->fc_bus;
	const int shift = fc->fc_shift;
	struct fdt_softc *sc = container_of(fbus, struct fdt_softc, sc_fbus);
	uint64_t fdta = addr;

	struct fdt_softc *rsc = fdtbus_decode_range(sc, &fdta, PAGE_SIZE);
	bus_space_tag_t bst = rsc->sc_fbus.fbus_pshift_bst[shift];

	addr = fdta;
	return bus_space_mmap(bst, addr, offset, prot, flags);
}
void
fdtbus_create_bus(struct fdt_softc *sc)
{
	struct fdtbus_simplebus *fbus = &sc->sc_fbus;
	struct fdt_attach_args *faa = &sc->sc_fbus.fbus_faa;

	/* Default to parent tags */
	for (size_t i = 0; i < FDTBUS_NREGSHIFT; i++) {
		KASSERTMSG(faa->faa_shift_bst[i] == fbus->fbus_pshift_bst[i],
		    "faa_bst %p pbst %p", faa->faa_shift_bst[i],
		    fbus->fbus_pshift_bst[i]);
	}
	KASSERTMSG(faa->faa_dmat == fbus->fbus_pdmat, "faa_dmat %p pdmat %p",
	    faa->faa_dmat, fbus->fbus_pdmat);


	/*
	 * We create tags when we have a bus with #address-cells
	 * and #size-cells equal to 1.  These tags use our map/mmap
	 * methods that decode to parent address
	 */

	if (fbus->fbus_nranges >= 0) {
		for (size_t i = 0; i < FDTBUS_NREGSHIFT; i++) {
			struct fdtbus_cookie *fc = kmem_alloc(sizeof(*fc),
			    KM_SLEEP);

			bus_space_tag_t bst = NULL;
			if (fbus->fbus_pshift_bst[i]) {
				bst = kmem_alloc(sizeof(*bst), KM_SLEEP);
				memcpy(bst, fbus->fbus_pshift_bst[i],
				    sizeof(*bst));

				bst->bs_cookie = fc;
				bst->bs_map = arm_fdtbus_bs_map;
				bst->bs_mmap = arm_fdtbus_bs_mmap;

				fc->fc_bus = &sc->sc_fbus;
				fc->fc_shift = i;
			}

			faa->faa_shift_bst[i] = bst;
		}
	}
	if (fbus->fbus_ndmaranges > 0) {
		struct arm32_bus_dma_tag *dmat =
		    kmem_alloc(sizeof(*dmat), KM_SLEEP);
		memcpy(dmat, fbus->fbus_pdmat, sizeof(*dmat));

		struct arm32_dma_range *drs =
		    kmem_alloc(sizeof(*drs) * fbus->fbus_ndmaranges, KM_SLEEP);

		for (size_t i = 0; i < fbus->fbus_ndmaranges; i++) {
			struct fdtbus_range *fr = fbus->fbus_dmaranges;
			struct arm32_dma_range *dr = &drs[i];

			dr->dr_sysbase = fr->fr_paddr;
			dr->dr_busbase = fr->fr_caddr;
			dr->dr_len = fr->fr_size;
			dr->dr_flags = 0;
#if 0
			if (dr->dr_len < physmem * PAGE_SIZE) {
				dr->dr_len = physmem * PAGE_SIZE;
			}
#endif
		}
		dmat->_ranges = drs;
		dmat->_nranges = fbus->fbus_ndmaranges;

		faa->faa_dmat = dmat;
	}
	for (size_t i = 0; i < FDTBUS_NREGSHIFT; i++) {
		if (i == 1)
			continue;
		printf("%s: faa->faa_shift_bst[%zu] %p "
		    "fbus->fbus_pshift_bst[%zu] %p\n", __func__, i,
		    faa->faa_shift_bst[i], i, fbus->fbus_pshift_bst[i]);
	}
	printf("%s: faa->faa_dmat         %p "
	    "fbus->fbus_pdmat         %p\n", __func__, faa->faa_dmat,
	    fbus->fbus_pdmat);
}
