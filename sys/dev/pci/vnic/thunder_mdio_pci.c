/*	$NetBSD$	*/

/*-
 * Copyright (c) 2021 The NetBSD Foundation, Inc.
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

#include <net/if.h>
#include <net/if_media.h>

#include <dev/mii/miivar.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcidevs.h>

#include "thunder_mdio_var.h"

static int	thunder_mdio_pci_probe(device_t, cfdata_t, void *);
static void	thunder_mdio_pci_attach(device_t, device_t, void *);
static int	thunder_mdio_pci_detach(device_t, int);

//XXXNH better name than 'cnmdio'
CFATTACH_DECL3_NEW(cnmdio, sizeof(struct thunder_mdio_softc),
    thunder_mdio_pci_probe, thunder_mdio_pci_attach, thunder_mdio_pci_detach, NULL, NULL, NULL,
    DVF_DETACH_SHUTDOWN);

#define REG_BASE_RID 0

static int
thunder_mdio_pci_probe(device_t dev, cfdata_t cf, void *aux)
{
	const struct pci_attach_args *pa = aux;

	if (PCI_VENDOR(pa->pa_id) == PCI_VENDOR_CAVIUM &&
	    PCI_PRODUCT(pa->pa_id) == PCI_PRODUCT_CAVIUM_THUNDERX_SMI_MDIO)
		return 1;

	return 0;
}

static void
thunder_mdio_pci_attach(device_t parent, device_t dev, void *aux)
{
	struct thunder_mdio_softc *sc = device_private(dev);
	const struct pci_attach_args *pa = aux;
	int ret;

	/*
	 * Map the device.  All devices support memory-mapped acccess.
	 */
	bool memh_valid;
	bus_space_tag_t memt;
	bus_space_handle_t memh;

	const int reg = PCI_BAR(REG_BASE_RID);
	const pcireg_t memtype = pci_mapreg_type(pa->pa_pc, pa->pa_tag, reg);
	switch (memtype) {
	case PCI_MAPREG_TYPE_MEM | PCI_MAPREG_MEM_TYPE_32BIT:
	case PCI_MAPREG_TYPE_MEM | PCI_MAPREG_MEM_TYPE_64BIT:
		memh_valid = (pci_mapreg_map(pa, reg, memtype, 0, &memt, &memh,
		    NULL, NULL) == 0);
		break;
	default:
		memh_valid = false;
		break;
	}

	if (memh_valid) {
		sc->sc_memt = memt;
		sc->sc_memh = memh;
	} else {
		aprint_error_dev(sc->dev,
		    "unable to map device registers\n");
		return;
	}

	/* Call core attach */
	ret = thunder_mdio_attach(dev);
	if (ret != 0)
		return;
#if 0

	/*
	 * Register device to this node/xref.
	 * Thanks to that we will be able to retrieve device_t structure
	 * while holding only node reference acquired from FDT.
	 */
	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);
#endif

	return;
}

static int
thunder_mdio_pci_detach(device_t dev, int flags)
{

	thunder_mdio_detach(dev, flags);

	return (0);
}

