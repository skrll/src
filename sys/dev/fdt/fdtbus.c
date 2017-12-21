/* $NetBSD: fdtbus.c,v 1.15 2017/08/27 19:13:31 jmcneill Exp $ */

/*-
 * Copyright (c) 2015 Jared D. McNeill <jmcneill@invisible.ca>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: fdtbus.c,v 1.15 2017/08/27 19:13:31 jmcneill Exp $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/kmem.h>

#include <sys/bus.h>

#include <dev/ofw/openfirm.h>

#include <dev/fdt/fdtvar.h>

#include <libfdt.h>

#include "locators.h"

#define	FDT_MAX_PATH	256

struct fdt_node {
	device_t	n_bus;
	device_t	n_dev;
	int		n_phandle;
	char		*n_name;

	u_int		n_order;

	TAILQ_ENTRY(fdt_node) n_nodes;
};

static TAILQ_HEAD(, fdt_node) fdt_nodes =
    TAILQ_HEAD_INITIALIZER(fdt_nodes);

static int	fdt_match(device_t, cfdata_t, void *);
static void	fdt_attach(device_t, device_t, void *);
static int	fdt_scan_submatch(device_t, cfdata_t, const int *, void *);
static void	fdt_scan_bus(struct fdt_softc *);
static void	fdt_scan(struct fdt_softc *, int);
static void	fdt_add_node(struct fdt_node *);
static u_int	fdt_get_order(int);

static const char * const fdtbus_compatible[] =
    { "simple-bus", NULL };

CFATTACH_DECL_NEW(fdt, sizeof(struct fdt_softc),
    fdt_match, fdt_attach, NULL, NULL);

void fdtbus_attach(int phandle, struct fdt_softc *sc);

static bool
fdtbus_check_range(struct fdtbus_range *range, int nranges, uint64_t *addrp,
    uint64_t size)
{
	KASSERTMSG(nranges >= 0, "%p: ranges %d", range, nranges);

	uint64_t addr = *addrp;
	/* For each range. */
	for (size_t i = 0; i < nranges; i++) {
		uint64_t cba = range[i].fr_caddr;
		uint64_t pba = range[i].fr_paddr;
		uint64_t cl = range[i].fr_size;

		aprint_debug("%s: checking range addr %llx / %llx\n", __func__,
		    cba, cba + cl);
		/* Try next, if we're not in the range. */
		if (addr < cba || (addr + size) >= (cba + cl))
			continue;

		addr -= cba;
		addr += pba;

		aprint_debug("%s: decoded             %llx / %llx\n", __func__,
		    addr, size);

		*addrp = addr;
		return true;
	}
	return false;
}

struct fdt_softc *
fdtbus_decode_range(struct fdt_softc *sc, uint64_t *paddr, uint64_t size)
{
	struct fdt_softc *psc = sc->sc_parent;
	if (psc == NULL)
		return sc;

	struct fdtbus_simplebus *fbus = &sc->sc_fbus;

	if (fbus->fbus_nranges > 0) {
		bool match = fdtbus_check_range(fbus->fbus_ranges,
		    fbus->fbus_nranges, paddr, size);
		if (match)
			return fdtbus_decode_range(psc, paddr, size);
	}
	if (fbus->fbus_ndmaranges > 0) {
		bool match = fdtbus_check_range(fbus->fbus_dmaranges,
		    fbus->fbus_ndmaranges, paddr, size);
		if (match)
			return fdtbus_decode_range(psc, paddr, size);
	}
	return fdtbus_decode_range(psc, paddr, size);
}


static int
fdt_range(struct fdt_softc *sc, int phandle, const char *name,
    struct fdtbus_range **rangesp, int *nrangesp)
{
	struct fdtbus_range *ranges = NULL;
	int nranges = 0;

	const size_t elen =
	    (sc->sc_pcells + sc->sc_acells + sc->sc_scells) * sizeof(uint32_t);

	int len;
	const uint8_t *buf = fdt_getprop(fdtbus_get_data(),
	    fdtbus_phandle2offset(phandle), name, &len);

	if (len == -1) {
		nranges = -1;
	} else if (len > 0 && len >= elen && (len % elen) == 0) {
		aprint_debug_dev(sc->sc_dev, "%s:\n", name);

		nranges = len / elen;
		ranges = kmem_zalloc(nranges * sizeof(*ranges), KM_SLEEP);

		size_t count = 0;
		while (len > 0) {
			uint64_t cba = fdtbus_get_cells(buf, sc->sc_acells);
			buf += sc->sc_acells * sizeof(uint32_t);
			len -= sc->sc_acells * sizeof(uint32_t);

			uint64_t pba = fdtbus_get_cells(buf, sc->sc_pcells);
			buf += sc->sc_pcells * sizeof(uint32_t);
			len -= sc->sc_pcells * sizeof(uint32_t);

			uint64_t cs = fdtbus_get_cells(buf, sc->sc_scells);
			buf += sc->sc_scells * sizeof(uint32_t);
			len -= sc->sc_scells * sizeof(uint32_t);

			ranges[count].fr_caddr = cba;
			ranges[count].fr_paddr = pba;
			ranges[count].fr_size = cs;

			aprint_debug_dev(sc->sc_dev,
			    "%zu: %llx - %llx -> %llx\n", count, cba, cba + cs,
			    pba);
			count++;
		}
		KASSERT(nranges == count);
	} else {
		aprint_debug_dev(sc->sc_dev, "%s: len %d entry length %zu\n",
		    name, len, elen);
	}
	*nrangesp = nranges;
	*rangesp = ranges;

	return 0;
}

static int
fdt_match(device_t parent, cfdata_t cf, void *aux)
{
	const struct fdt_attach_args *faa = aux;
	const int phandle = faa->faa_phandle;
	int match;

	/* Check compatible string */
	match = of_match_compatible(phandle, fdtbus_compatible);
	if (match)
		return match;

	/* Some nodes have no compatible string */
	if (!of_hasprop(phandle, "compatible")) {
		if (OF_finddevice("/clocks") == phandle)
			return 1;
		if (OF_finddevice("/chosen") == phandle)
			return 1;
	}

	/* Always match the root node */
	return OF_finddevice("/") == phandle;
}

static void
fdt_attach(device_t parent, device_t self, void *aux)
{
	struct fdt_softc *sc = device_private(self);
	const struct fdt_attach_args *faa = aux;
	const int phandle = faa->faa_phandle;
	struct fdt_node *node;
	char *model, *name;
	int len, child;

	sc->sc_dev = self;
	sc->sc_fbus.fbus_phandle = phandle;
	sc->sc_parent = NULL;

	if (phandle != OF_peer(0)) {
		/* we're attaching to a parent fdtbus */
		sc->sc_parent = device_private(parent);
	}

	/* maybe not always */
	memcpy(sc->sc_fbus.fbus_pshift_bst, faa->faa_shift_bst,
	     sizeof(sc->sc_fbus.fbus_pshift_bst));
	sc->sc_fbus.fbus_pdmat = faa->faa_dmat;
	sc->sc_fbus.fbus_faa = *faa;

	aprint_naive("\n");
	len = OF_getproplen(phandle, "model");
	if (len > 0) {
		model = kmem_zalloc(len, KM_SLEEP);
		if (OF_getprop(phandle, "model", model, len) == len) {
			aprint_normal(": %s\n", model);
		} else {
			aprint_normal("\n");
		}
		kmem_free(model, len);
	} else {
		aprint_normal("\n");
	}

	sc->sc_pcells = fdtbus_get_addr_cells(OF_parent(phandle));
	sc->sc_acells = fdtbus_get_addr_cells(phandle);
	sc->sc_scells = fdtbus_get_size_cells(phandle);

	fdt_range(sc, phandle, "ranges", &sc->sc_fbus.fbus_ranges,
	    &sc->sc_fbus.fbus_nranges);
	fdt_range(sc, phandle, "dma-ranges", &sc->sc_fbus.fbus_dmaranges,
	    &sc->sc_fbus.fbus_ndmaranges);

	aprint_debug_dev(sc->sc_dev, "ranges %d dma_ranges %d\n",
	    sc->sc_fbus.fbus_nranges, sc->sc_fbus.fbus_ndmaranges);

	/*
	 * Create bus_space and bus_dma tags appropriate for platform
	 */
	fdtbus_create_bus(sc);

	for (child = OF_child(phandle); child; child = OF_peer(child)) {
		if (!fdtbus_status_okay(child))
			continue;

		len = OF_getproplen(child, "name");
		if (len <= 0)
			continue;

		name = kmem_zalloc(len, KM_SLEEP);
		if (OF_getprop(child, "name", name, len) != len)
			continue;

		/* Add the node to our device list */
		node = kmem_alloc(sizeof(*node), KM_SLEEP);
		node->n_bus = self;
		node->n_dev = NULL;
		node->n_phandle = child;
		node->n_name = name;
		node->n_order = fdt_get_order(node->n_phandle);
		fdt_add_node(node);
	}

	/* Scan and attach all known busses in the tree. */
	fdt_scan_bus(sc);

	/* Only the root bus should scan for devices */
	if (OF_finddevice("/") != faa->faa_phandle)
		return;

	aprint_debug_dev(sc->sc_dev, "  order   phandle   bus    path\n");
	aprint_debug_dev(sc->sc_dev, "  =====   =======   ===    ====\n");
	TAILQ_FOREACH(node, &fdt_nodes, n_nodes) {
		char buf[FDT_MAX_PATH];
		const char *path = buf;
		if (!fdtbus_get_path(node->n_phandle, buf, sizeof(buf)))
			path = node->n_name;
		aprint_debug_dev(sc->sc_dev, "   %04x   0x%04x    %s   %s\n",
		    node->n_order & 0xffff, node->n_phandle,
		    device_xname(node->n_bus), path);
	}

	/* Scan devices */
	for (int pass = 0; pass <= FDTCF_PASS_DEFAULT; pass++)
		fdt_scan(sc, pass);
}

static void
fdt_init_attach_args(struct fdt_softc *sc, struct fdt_node *node,
    bool quiet, struct fdt_attach_args *faa)
{
	*faa = sc->sc_fbus.fbus_faa;
	faa->faa_phandle = node->n_phandle;
	faa->faa_name = node->n_name;
	faa->faa_quiet = quiet;
}

static void
fdt_scan_bus(struct fdt_softc *sc)
{
	struct fdt_node *node;
	struct fdt_attach_args faa;
	cfdata_t cf;

	TAILQ_FOREACH(node, &fdt_nodes, n_nodes) {
		if (node->n_bus != sc->sc_dev)
			continue;
		if (node->n_dev != NULL)
			continue;

		fdt_init_attach_args(sc, node, true, &faa);

		/*
		 * Only attach busses to nodes where this driver is the best
		 * match.
		 */
		cf = config_search_loc(NULL, node->n_bus, NULL, NULL, &faa);
		if (cf == NULL || strcmp(cf->cf_name, "fdt") != 0)
			continue;

		/*
		 * Attach the bus.
		 */
		node->n_dev = config_found(node->n_bus, &faa, fdtbus_print);
	}
}

static int
fdt_scan_submatch(device_t parent, cfdata_t cf, const int *locs, void *aux)
{
	if (locs[FDTCF_PASS] != FDTCF_PASS_DEFAULT &&
	    locs[FDTCF_PASS] != cf->cf_loc[FDTCF_PASS])
		return 0;

	return config_stdsubmatch(parent, cf, locs, aux);
}

static void
fdt_scan(struct fdt_softc *sc, int pass)
{
	struct fdt_node *node;
	struct fdt_attach_args faa;
	const int locs[FDTCF_NLOCS] = {
		[FDTCF_PASS] = pass
	};
	bool quiet = pass != FDTCF_PASS_DEFAULT;

	TAILQ_FOREACH(node, &fdt_nodes, n_nodes) {
		if (node->n_dev != NULL)
			continue;

		struct fdt_softc *csc = device_private(node->n_bus);

		fdt_init_attach_args(csc, node, quiet, &faa);

		/*
		 * Attach the device.
		 */
		node->n_dev = config_found_sm_loc(node->n_bus, "fdt", locs,
		    &faa, fdtbus_print, fdt_scan_submatch);
	}
}

static void
fdt_add_node(struct fdt_node *new_node)
{
	struct fdt_node *node;

	TAILQ_FOREACH(node, &fdt_nodes, n_nodes)
		if (node->n_order > new_node->n_order) {
			TAILQ_INSERT_BEFORE(node, new_node, n_nodes);
			return;
		}
	TAILQ_INSERT_TAIL(&fdt_nodes, new_node, n_nodes);
}

static u_int
fdt_get_order(int phandle)
{
	u_int val = UINT_MAX;
	int child;

	of_getprop_uint32(phandle, "phandle", &val);

	for (child = OF_child(phandle); child; child = OF_peer(child)) {
		u_int child_val = fdt_get_order(child);
		if (child_val < val)
			val = child_val;
	}

	return val;
}

int
fdtbus_print(void *aux, const char *pnp)
{
	const struct fdt_attach_args * const faa = aux;
	char buf[FDT_MAX_PATH];
	const char *name = buf;
	int len;

	if (pnp && faa->faa_quiet)
		return QUIET;

	/* Skip "not configured" for nodes w/o compatible property */
	if (pnp && OF_getproplen(faa->faa_phandle, "compatible") <= 0)
		return QUIET;

	if (!fdtbus_get_path(faa->faa_phandle, buf, sizeof(buf)))
		name = faa->faa_name;

	if (pnp) {
		aprint_normal("%s at %s", name, pnp);
		const char *compat = fdt_getprop(fdtbus_get_data(),
		    fdtbus_phandle2offset(faa->faa_phandle), "compatible",
		    &len);
		while (len > 0) {
			aprint_debug(" <%s>", compat);
			len -= (strlen(compat) + 1);
			compat += (strlen(compat) + 1);
		}
	} else
		aprint_debug(" (%s)", name);

	return UNCONF;
}

bus_space_tag_t
fdtbus_get_node_bst(int phandle, int shift)
{
	struct fdt_node *node;

	aprint_debug("%s: 0x%04x\n", __func__, phandle);

	TAILQ_FOREACH(node, &fdt_nodes, n_nodes) {
		aprint_debug("%s: 0x%04x %s", __func__, node->n_phandle,
		    device_xname(node->n_bus));
		if (node->n_phandle == phandle) {
			struct fdt_softc *sc = device_private(node->n_bus);

			aprint_debug("... found\n");

			return sc->sc_fbus.fbus_faa.faa_shift_bst[shift];
		}
		aprint_debug("... nope\n");
	}
	aprint_debug("%s: ... not found\n", __func__);
	return NULL;
}
