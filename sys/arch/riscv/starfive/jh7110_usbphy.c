/* $NetBSD$ */

/*-
 * Copyright (c) 2023 The NetBSD Foundation, Inc.
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


#include <dev/fdt/fdtvar.h>


struct jh7110_usbphy_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	int			sc_phandle;

	struct clk		*sc_clk_usb125m;
	struct clk		*sc_clk_app125m;
};

#define USB_125M_CLK_RATE		125000000
#define USB_LS_KEEPALIVE_OFF		0x4
#define USB_LS_KEEPALIVE_ENABLE		__BIT(4)



#define	RD4(sc, reg)							       \
	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	WR4(sc, reg, val)						       \
	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))


/* Register definitions */


static void
jh7110_usbphy_init(struct jh7110_usbphy_softc *sc)
{
	int error;

	error = clk_set_rate(sc->sc_clk_usb125m, USB_125M_CLK_RATE);
	if (error) {
		aprint_error(": couldn't set clock usb125m: %d\n", error);
		return;
	}

	error = clk_enable(sc->sc_clk_usb125m);
	if (error) {
		aprint_error(": couldn't enable clock usb125m: %d\n", error);
		return;
	}

	error = clk_enable(sc->sc_clk_app125m);
	if (error) {
		aprint_error(": couldn't enable clock app125m: %d\n", error);
		return;
	}

	/* Enable the LS speed keep-alive signal for host mode */
	uint32_t val = RD4(sc, USB_LS_KEEPALIVE_OFF);
	val |= USB_LS_KEEPALIVE_ENABLE;
	WR4(sc, USB_LS_KEEPALIVE_OFF, val);
}

/* Compat string(s) */
static const struct device_compatible_entry compat_data[] = {
	{ .compat = "starfive,jh7110-usb-phy" },
	DEVICE_COMPAT_EOL
};

static int
jh7110_usbphy_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
jh7110_usbphy_attach(device_t parent, device_t self, void *aux)
{
	struct jh7110_usbphy_softc *sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	sc->sc_clk_usb125m = fdtbus_clock_get(phandle, "125m");
	if (sc->sc_clk_usb125m == NULL) {
		aprint_error(": couldn't get clock 125m\n");
		return;
	}
	sc->sc_clk_app125m = fdtbus_clock_get(phandle, "app_125m");
	if (sc->sc_clk_app125m == NULL) {
		aprint_error(": couldn't get clock app_125m\n");
		return;
	}

	sc->sc_dev = self;
	sc->sc_phandle = phandle;
	sc->sc_bst = faa->faa_bst;
	int error = bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh);
	if (error) {
		aprint_error(": couldn't map %#" PRIxBUSADDR ": %d", addr, error);
		return;
	}

	aprint_naive("\n");
	aprint_normal(": USB PHY\n");

	jh7110_usbphy_init(sc);
}

CFATTACH_DECL_NEW(jh7110_usbphy, sizeof(struct jh7110_usbphy_softc),
	jh7110_usbphy_match, jh7110_usbphy_attach, NULL, NULL);
