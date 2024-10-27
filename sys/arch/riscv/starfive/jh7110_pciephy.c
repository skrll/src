/* $NetBSD: jh7110_pciephy.c,v 1.3 2025/01/01 17:35:44 skrll Exp $ */

/*-
 * Copyright (c) 2024 The NetBSD Foundation, Inc.
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
__KERNEL_RCSID(0, "$NetBSD: jh7110_pciephy.c,v 1.3 2025/01/01 17:35:44 skrll Exp $");

#include <sys/param.h>

#include <dev/fdt/fdtvar.h>
#include <dev/fdt/syscon.h>


struct jh7110_pciephy_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	int			sc_phandle;

	struct syscon *		sc_sys_syscon;
	bus_size_t		sc_phy_connect;

	struct syscon *		sc_stg_syscon;
	bus_size_t		sc_stg_pcie_mode;
	bus_size_t		sc_stg_pcie_usb;
};

/* Register definitions */

#define PCIE_KVCO_LEVEL			0x28
#define  PCEI_PHY_KVCO_FINE_TUNE_LEVEL	0x91

#define PCIE_USB3_PHY_PLL_CTL		0x7c

#define PCIE_KVCO_TUNE_SIGNAL		0x80
#define	 PCIE_KVO_FINE_TUNE_SIGNALS	0x0c

#define USB_PDRSTN_SPLIT		__BIT(17)

#define PCIE_PHY_MODE			__BIT(20)
#define PCIE_PHY_MODE_MASK		__BITS(21, 20)
#define PCIE_USB3_BUS_WIDTH_MASK	__BITS(3, 2)
#define PCIE_USB3_PHY_ENABLE		__BIT(4)
#define PCIE_USB3_BUS_WIDTH		__BIT(3)


#if 0
#define PCIE_USB3_PHY_ENABLE		__BIT(4)
#define PHY_KVCO_FINE_TUNE_SIGNALS	0xc

#define USB_PDRSTN_SPLIT		__BIT(17)

#define PCIE_PHY_MODE			__BIT(20)
#define PCIE_PHY_MODE_MASK		__BITS(21, 20)
#define PCIE_USB3_BUS_WIDTH_MASK	__BITS(3, 2)
#define PCIE_USB3_BUS_WIDTH		__BIT(3)
#define PCIE_USB3_RATE_MASK		__BITS(6, 5)
#define PCIE_USB3_RX_STANDBY_MASK	__BIT(7)
#define PCIE_USB3_PHY_ENABLE		__BIT(4)
#endif

#define RD4(sc, reg)							       \
	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define WR4(sc, reg, val)						       \
	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))
#define CLR4(sc, reg, mask)						       \
	WR4((sc), (reg), RD4((sc), (reg)) & ~(mask))


static void *
jh7110pciephy_acquire(device_t dev, const void *data, size_t len)
{
	struct jh7110_pciephy_softc * const sc = device_private(dev);

	if (len != 0) {
		aprint_verbose("phy acquire with len %zu", len);
		return NULL;
	}

	return sc;
}

static void
jh7110pciephy_release(device_t dev, void *data)
{
}

static int
jh7110pciephy_enable(device_t dev, void *priv, bool enable)
{
	struct jh7110_pciephy_softc * const sc = device_private(dev);
	uint32_t val;

	syscon_lock(sc->sc_stg_syscon);

	val = syscon_read_4(sc->sc_stg_syscon, sc->sc_stg_pcie_mode);
	val &= ~PCIE_PHY_MODE_MASK;
	syscon_write_4(sc->sc_stg_syscon, sc->sc_stg_pcie_mode, val);

	val = syscon_read_4(sc->sc_stg_syscon, sc->sc_stg_pcie_usb);
	val &= ~(PCIE_USB3_BUS_WIDTH_MASK | PCIE_USB3_PHY_ENABLE);
	val |= PCIE_USB3_BUS_WIDTH;
	syscon_write_4(sc->sc_stg_syscon, sc->sc_stg_pcie_usb, val);

	syscon_unlock(sc->sc_stg_syscon);

	syscon_lock(sc->sc_sys_syscon);

	val = syscon_read_4(sc->sc_sys_syscon, sc->sc_phy_connect);
	val &= ~USB_PDRSTN_SPLIT;
	syscon_write_4(sc->sc_sys_syscon, sc->sc_phy_connect, val);

	syscon_unlock(sc->sc_sys_syscon);

	val = RD4(sc, PCIE_USB3_PHY_PLL_CTL);
	val &= ~PCIE_USB3_PHY_ENABLE;
	WR4(sc, PCIE_USB3_PHY_PLL_CTL, val);

	return 0;
}

const struct fdtbus_phy_controller_func jh7110pciephy_funcs = {
	.acquire = jh7110pciephy_acquire,
	.release = jh7110pciephy_release,
	.enable = jh7110pciephy_enable,
};

/* Compat string(s) */
static const struct device_compatible_entry compat_data[] = {
	{ .compat = "starfive,jh7110-pcie-phy" },
	DEVICE_COMPAT_EOL
};

static int
jh7110_pciephy_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
jh7110_pciephy_attach(device_t parent, device_t self, void *aux)
{
	struct jh7110_pciephy_softc *sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	const bus_space_tag_t bst = faa->faa_bst;
	bus_addr_t addr;
	bus_size_t size;
	int error;

	error = fdtbus_get_reg(phandle, 0, &addr, &size);
	if (error) {
		aprint_error(": couldn't get registers\n");
		return;
	}
	error = bus_space_map(bst, addr, size, 0, &sc->sc_bsh);
	if (error) {
		aprint_error(": couldn't map %#" PRIxBUSADDR ": %d", addr,
		    error);
		return;
	}

	int len;
	const char *sys_syscon = "starfive,sys-syscon";
	const u_int *sys_syscon_data =
	    fdtbus_get_prop(phandle, sys_syscon, &len);
	if (sys_syscon_data != NULL) {
		if (len != 2 * sizeof(uint32_t)) {
			aprint_error(": %s has wrong length (%d)\n",
			    sys_syscon, len);
			return;
		}
		const int sys_syscon_phandle =
		    fdtbus_get_phandle_from_native(be32dec(&sys_syscon_data[0]));
		sc->sc_sys_syscon = fdtbus_syscon_lookup(sys_syscon_phandle);
		if (sc->sc_sys_syscon == NULL) {
			aprint_error(": couldn't get sys-syscon\n");
			return;
		}
		sc->sc_phy_connect = be32dec(&sys_syscon_data[1]);
	}

	const char *stg_syscon = "starfive,stg-syscon";
	const u_int *stg_syscon_data =
	    fdtbus_get_prop(phandle, stg_syscon, &len);
	if (stg_syscon_data != NULL) {
		if (len != 3 * sizeof(uint32_t)) {
			aprint_error(": %s has wrong length (%d)\n",
			    stg_syscon, len);
			return;
		}
		const int stg_syscon_phandle =
		    fdtbus_get_phandle_from_native(be32dec(&stg_syscon_data[0]));

		sc->sc_stg_syscon = fdtbus_syscon_lookup(stg_syscon_phandle);
		if (sc->sc_stg_syscon == NULL) {
			aprint_error(": couldn't get stg-syscon\n");
			return;
		}

		sc->sc_stg_pcie_mode = be32dec(&sys_syscon_data[1]);
		sc->sc_stg_pcie_usb = be32dec(&sys_syscon_data[2]);
	}

	sc->sc_dev = self;
	sc->sc_phandle = phandle;
	sc->sc_bst = bst;

	aprint_naive("\n");
	aprint_normal(": JH7110 PCIe PHY\n");

	WR4(sc, PCIE_KVCO_LEVEL, PCEI_PHY_KVCO_FINE_TUNE_LEVEL);
	WR4(sc, PCIE_KVCO_TUNE_SIGNAL, PCIE_KVO_FINE_TUNE_SIGNALS);

	fdtbus_register_phy_controller(self, faa->faa_phandle,
	    &jh7110pciephy_funcs);
}

CFATTACH_DECL_NEW(jh7110_pciephy, sizeof(struct jh7110_pciephy_softc),
	jh7110_pciephy_match, jh7110_pciephy_attach, NULL, NULL);
