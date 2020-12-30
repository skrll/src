/* $NetBSD: fdtbus.c,v 1.35 2020/09/20 11:25:36 jmcneill Exp $ */

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
__KERNEL_RCSID(0, "$NetBSD: fdtbus.c,v 1.35 2020/09/20 11:25:36 jmcneill Exp $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/kmem.h>
#include <sys/cpu.h>

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
	const char	*n_name;
	struct fdt_attach_args n_faa;

	int		n_cfpass;
	cfdata_t	n_cf;

	u_int		n_order;

	bool		n_pinctrl_init;

	TAILQ_ENTRY(fdt_node) n_nodes;
};

static TAILQ_HEAD(, fdt_node) fdt_nodes =
    TAILQ_HEAD_INITIALIZER(fdt_nodes);
static bool fdt_need_rescan = false;

static int	fdt_match(device_t, cfdata_t, void *);
static void	fdt_attach(device_t, device_t, void *);
static int	fdt_rescan(device_t, const char *, const int *);
static void	fdt_childdet(device_t, device_t);

static int	fdt_scan_submatch(device_t, cfdata_t, const int *, void *);
static void	fdt_scan_best(struct fdt_softc *, struct fdt_node *);
static void	fdt_scan(struct fdt_softc *, int);
static void	fdt_add_node(struct fdt_node *);
static u_int	fdt_get_order(int);
static void	fdt_pre_attach(struct fdt_node *);
static void	fdt_post_attach(struct fdt_node *);

static const char * const fdtbus_compatible[] =
    { "simple-bus", NULL };

CFATTACH_DECL2_NEW(simplebus, sizeof(struct fdt_softc),
    fdt_match, fdt_attach, NULL, NULL, fdt_rescan, fdt_childdet);

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

		aprint_debug("%s: checking range addr %" PRIx64 " / %" PRIx64
		    "\n", __func__, cba, cba + cl);
		/* Try next, if we're not in the range. */
		if (addr < cba || (addr + size) >= (cba + cl))
			continue;

		addr -= cba;
		addr += pba;

		aprint_debug("%s: decoded             %" PRIx64 " / %" PRIx64
		    "\n", __func__, addr, size);

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
			    "%zu: %" PRIx64 " - %" PRIx64 " -> %" PRIx64 "\n",
			    count, cba, cba + cs, pba);
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
	const char *descr, *model;

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

	descr = fdtbus_get_string(phandle, "model");
	if (descr)
		aprint_normal(": %s\n", descr);
	else
		aprint_normal("\n");

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

	for (int child = OF_child(phandle); child; child = OF_peer(child)) {
		if (!fdtbus_status_okay(child))
			continue;

		int len = OF_getproplen(child, "name");
		if (len <= 0)
			continue;

		char *name = kmem_zalloc(len, KM_SLEEP);
		if (OF_getprop(child, "name", name, len) != len)
			continue;

		/* Add the node to our device list */
		struct fdt_node *node = kmem_alloc(sizeof(*node), KM_SLEEP);
		node->n_bus = self;
		node->n_dev = NULL;
		node->n_phandle = child;
		node->n_name = name;
		node->n_order = fdt_get_order(node->n_phandle);
		fdt_add_node(node);
	}

	/* Find all child nodes */
	fdt_add_bus(self, phandle, &sc->sc_fbus.fbus_faa);

	/* Only the root bus should scan for devices */
	if (OF_finddevice("/") != faa->faa_phandle)
		return;

	/* Set hw.model if available */
	model = fdtbus_get_string(phandle, "compatible");
	if (model)
		cpu_setmodel("%s", model);
	else if (descr)
		cpu_setmodel("%s", descr);

	/* Scan devices */
	fdt_rescan(self, NULL, NULL);
}

static int
fdt_rescan(device_t self, const char *ifattr, const int *locs)
{
	struct fdt_softc *sc = device_private(self);
	struct fdt_node *node;
	int pass;

	TAILQ_FOREACH(node, &fdt_nodes, n_nodes)
		fdt_scan_best(sc, node);

	pass = 0;
	fdt_need_rescan = false;
	do {
		fdt_scan(sc, pass);
		if (fdt_need_rescan == true) {
			pass = 0;
			TAILQ_FOREACH(node, &fdt_nodes, n_nodes) {
				if (node->n_cfpass == -1)
					fdt_scan_best(sc, node);
			}
			fdt_need_rescan = false;
		} else {
			pass++;
		}
	} while (pass <= FDTCF_PASS_DEFAULT);

	return 0;
}

static void
fdt_childdet(device_t parent, device_t child)
{
	struct fdt_node *node;

	TAILQ_FOREACH(node, &fdt_nodes, n_nodes)
		if (node->n_dev == child) {
			node->n_dev = NULL;
			break;
		}
}

static void
fdt_init_attach_args(const struct fdt_attach_args *faa_tmpl, struct fdt_node *node,
    bool quiet, struct fdt_attach_args *faa)
{
	*faa = *faa_tmpl;
	faa->faa_phandle = node->n_phandle;
	faa->faa_name = node->n_name;
	faa->faa_quiet = quiet;
	faa->faa_dmat = node->n_faa.faa_dmat;
}

static bool
fdt_add_bus_stdmatch(void *arg, int child)
{
	return fdtbus_status_okay(child);
}

void
fdt_add_bus(device_t bus, const int phandle, struct fdt_attach_args *faa)
{
	fdt_add_bus_match(bus, phandle, faa, fdt_add_bus_stdmatch, NULL);
}

void
fdt_add_bus_match(device_t bus, const int phandle, struct fdt_attach_args *faa,
    bool (*fn)(void *, int), void *fnarg)
{
	int child;

	for (child = OF_child(phandle); child; child = OF_peer(child)) {
		if (fn && !fn(fnarg, child))
			continue;

		fdt_add_child(bus, child, faa, fdt_get_order(child));
	}
}

static int
fdt_dma_translate(int phandle, struct fdt_dma_range **ranges, u_int *nranges)
{
	const uint8_t *data;
	int len, n;

	const int parent = OF_parent(phandle);
	if (parent == -1)
		return 1;	/* done searching */

	data = fdtbus_get_prop(phandle, "dma-ranges", &len);
	if (data == NULL)
		return 1;	/* no dma-ranges property, stop searching */

	if (len == 0)
		return 0;	/* dma-ranges property is empty, keep going */

	const int addr_cells = fdtbus_get_addr_cells(phandle);
	const int size_cells = fdtbus_get_size_cells(phandle);
	const int paddr_cells = fdtbus_get_addr_cells(parent);
	if (addr_cells == -1 || size_cells == -1 || paddr_cells == -1)
		return 1;

	const int entry_size = (addr_cells + paddr_cells + size_cells) * 4;

	*nranges = len / entry_size;
	*ranges = kmem_alloc(sizeof(struct fdt_dma_range) * *nranges, KM_SLEEP);
	for (n = 0; len >= entry_size; n++, len -= entry_size) {
		const uint64_t cba = fdtbus_get_cells(data, addr_cells);
		data += addr_cells * 4;
		const uint64_t pba = fdtbus_get_cells(data, paddr_cells);
		data += paddr_cells * 4;
		const uint64_t cl = fdtbus_get_cells(data, size_cells);
		data += size_cells * 4;

		(*ranges)[n].dr_sysbase = pba;
		(*ranges)[n].dr_busbase = cba;
		(*ranges)[n].dr_len = cl;
	}

	return 1;
}

static bus_dma_tag_t
fdt_get_dma_tag(struct fdt_node *node)
{
	struct fdt_dma_range *ranges = NULL;
	u_int nranges = 0;
	int parent;

	parent = OF_parent(node->n_phandle);
	while (parent != -1) {
		if (fdt_dma_translate(parent, &ranges, &nranges) != 0)
			break;
		parent = OF_parent(parent);
	}

	return fdtbus_dma_tag_create(node->n_phandle, ranges, nranges);
}

void
fdt_add_child(device_t bus, const int child, struct fdt_attach_args *faa,
    u_int order)
{
	struct fdt_node *node;

	/* Add the node to our device list */
	node = kmem_zalloc(sizeof(*node), KM_SLEEP);
	node->n_bus = bus;
	node->n_dev = NULL;
	node->n_phandle = child;
	node->n_name = fdtbus_get_string(child, "name");
	node->n_cfpass = -1;
	node->n_cf = NULL;
	node->n_order = order;
	node->n_faa = *faa;
	node->n_faa.faa_phandle = child;
	node->n_faa.faa_name = node->n_name;
	node->n_faa.faa_dmat = fdt_get_dma_tag(node);

	fdt_add_node(node);
	fdt_need_rescan = true;
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
fdt_scan_best(struct fdt_softc *sc, struct fdt_node *node)
{
	struct fdt_attach_args faa;
	cfdata_t cf, best_cf;
	int match, best_match, best_pass;

	best_cf = NULL;
	best_match = 0;
	best_pass = FDTCF_PASS_DEFAULT;

	for (int pass = 0; pass <= FDTCF_PASS_DEFAULT; pass++) {
		const int locs[FDTCF_NLOCS] = {
			[FDTCF_PASS] = pass
		};
		fdt_init_attach_args(&sc->sc_fbus.fbus_faa, node, true, &faa);
		cf = config_search_loc(fdt_scan_submatch, node->n_bus, "fdt", locs, &faa);
		if (cf == NULL)
			continue;
		match = config_match(node->n_bus, cf, &faa);
		if (match > best_match) {
			best_match = match;
			best_cf = cf;
			best_pass = pass;
		}
	}

	node->n_cf = best_cf;
	node->n_cfpass = best_pass;
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
		if (node->n_cfpass != pass || node->n_dev != NULL)
			continue;

		fdt_init_attach_args(&sc->sc_fbus.fbus_faa, node, quiet, &faa);

		if (quiet && node->n_cf == NULL) {
			/*
			 * No match for this device, skip it.
			 */
			continue;
		}

		/*
		 * Attach the device.
		 */
		fdt_pre_attach(node);

		if (quiet) {
			node->n_dev = config_attach_loc(node->n_bus, node->n_cf, locs,
			    &faa, fdtbus_print);
		} else {
			/*
			 * Default pass.
			 */
			node->n_dev = config_found_sm_loc(node->n_bus, "fdt", locs,
			    &faa, fdtbus_print, fdt_scan_submatch);
		}

		if (node->n_dev != NULL)
			fdt_post_attach(node);
	}
}

static void
fdt_pre_attach(struct fdt_node *node)
{
	const char *cfgname;
	int error;

	node->n_pinctrl_init = fdtbus_pinctrl_has_config(node->n_phandle, "init");

	cfgname = node->n_pinctrl_init ? "init" : "default";

	aprint_debug_dev(node->n_bus, "set %s config for %s\n", cfgname, node->n_name);

	error = fdtbus_pinctrl_set_config(node->n_phandle, cfgname);
	if (error != 0 && error != ENOENT)
		aprint_debug_dev(node->n_bus,
		    "failed to set %s config on %s: %d\n",
		    cfgname, node->n_name, error);
}

static void
fdt_post_attach(struct fdt_node *node)
{
	char buf[FDT_MAX_PATH];
	prop_dictionary_t dict;
	int error;

	dict = device_properties(node->n_dev);
	if (fdtbus_get_path(node->n_phandle, buf, sizeof(buf)))
		prop_dictionary_set_string(dict, "fdt-path", buf);

	if (node->n_pinctrl_init) {
		aprint_debug_dev(node->n_bus, "set default config for %s\n", node->n_name);
		error = fdtbus_pinctrl_set_config(node->n_phandle, "default");
		if (error != 0 && error != ENOENT)
			aprint_debug_dev(node->n_bus,
			    "failed to set default config on %s: %d\n",
			    node->n_name, error);
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

void
fdt_remove_byhandle(int phandle)
{
	struct fdt_node *node;

	TAILQ_FOREACH(node, &fdt_nodes, n_nodes) {
		if (node->n_phandle == phandle) {
			TAILQ_REMOVE(&fdt_nodes, node, n_nodes);
			return;
		}
	}
}

void
fdt_remove_bycompat(const char *compatible[])
{
	struct fdt_node *node, *next;

	TAILQ_FOREACH_SAFE(node, &fdt_nodes, n_nodes, next) {
		if (of_match_compatible(node->n_phandle, compatible)) {
			TAILQ_REMOVE(&fdt_nodes, node, n_nodes);
		}
	}
}

int
fdt_find_with_property(const char *prop, int *pindex)
{
	struct fdt_node *node;
	int index = 0;

	TAILQ_FOREACH(node, &fdt_nodes, n_nodes) {
		if (index++ < *pindex)
			continue;
		if (of_hasprop(node->n_phandle, prop)) {
			*pindex = index;
			return node->n_phandle;
		}
	}

	return -1;
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
