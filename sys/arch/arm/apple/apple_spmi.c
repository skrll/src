/*	$NetBSD$	*/
/*	$OpenBSD: aplspmi.c,v 1.2 2022/04/06 18:59:26 naddy Exp $	*/

/*-
 * Copyright (c) 2022 The NetBSD Foundation, Inc.
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

/*
 * Copyright (c) 2021 Mark Kettenis <kettenis@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */


#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>

#include <dev/fdt/fdtvar.h>

/*
 * This driver is based on preliminary device tree bindings and will
 * almost certainly need changes once the official bindings land in
 * mainline Linux.  Support for these preliminary bindings will be
 * dropped as soon as official bindings are available.
 */

#define SPMI_STAT		0x00
#define  SPMI_STAT_RXEMPTY		__BIT(24)
#define  SPMI_STAT_TXEMPTY		__BIT(8)
#define SPMI_CMD		0x04
#define  SPMI_CMD_ADDR_MASK		__BITS(23, 16)
#define  SPMI_CMD_ADDR(x)		__SHIFTIN((x), SPMI_CMD_ADDR_MASK)
#define  SPMI_CMD_LAST			__BIT(15)
#define  SPMI_CMD_SID_MASK		__BITS(14, 8)
#define  SPMI_CMD_SID(x)		__SHIFTIN((x), SPMI_CMD_SID_MASK)
#define  SPMI_CMD_OPC_MASK		__BITS(7, 0)
#define SPMI_RESP		0x08
#define SPMI_INTEN(i)		(0x20 + (i) * 4)
#define SPMI_INTSTAT(i)		(0x60 + (i) * 4)

#define SPMI_READ(sc, reg)							\
	(bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg)))
#define SPMI_WRITE(sc, reg, val)						\
	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

struct apple_spmi_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;

	struct spmi_controller	sc_tag;
};

int	apple_spmi_print(void *, const char *);
int	apple_spmi_cmd_read(void *, uint8_t, uint8_t, uint16_t, void *, size_t);
int	apple_spmi_cmd_write(void *, uint8_t, uint8_t, uint16_t,
	    const void *, size_t);


#if 0

int
apple_spmi_print(void *aux, const char *pnp)
{
	struct spmi_attach_args *sa = aux;

	if (pnp != NULL)
		printf("\"%s\" at %s", sa->sa_name, pnp);
	printf(" sid 0x%x", sa->sa_sid);

	return UNCONF;
}
#endif


int
apple_spmi_read_resp(struct apple_spmi_softc *sc, uint32_t *resp)
{
	int retry;

	for (retry = 1000; retry > 0; retry--) {
		if ((SPMI_READ(sc, SPMI_STAT) & SPMI_STAT_RXEMPTY) == 0)
			break;
		delay(1);
	}
	if (retry == 0)
		return ETIMEDOUT;

	*resp = SPMI_READ(sc, SPMI_RESP);
	return 0;
}

int
apple_spmi_cmd_read(void *cookie, uint8_t sid, uint8_t cmd, uint16_t addr,
    void *buf, size_t len)
{
	struct apple_spmi_softc *sc = cookie;
	uint8_t *cbuf = buf;
	uint32_t resp;
	int error;

	if (len == 0 || len > 8)
		return EINVAL;

	SPMI_WRITE(sc, SPMI_CMD, SPMI_CMD_SID(sid) | cmd | SPMI_CMD_ADDR(addr) |
	    (len - 1) | SPMI_CMD_LAST);

	error = apple_spmi_read_resp(sc, &resp);
	if (error)
		return error;

	while (len > 0) {
		error = apple_spmi_read_resp(sc, &resp);
		if (error)
			return error;
		memcpy(cbuf, &resp, MIN(len, 4));
		cbuf += MIN(len, 4);
		len -= MIN(len, 4);
	}

	return 0;
}

int
apple_spmi_cmd_write(void *cookie, uint8_t sid, uint8_t cmd, uint16_t addr,
    const void *buf, size_t len)
{
	struct apple_spmi_softc *sc = cookie;
	const uint8_t *cbuf = buf;
	uint32_t data, resp;

	if (len == 0 || len > 8)
		return EINVAL;

	SPMI_WRITE(sc, SPMI_CMD, SPMI_CMD_SID(sid) | cmd | SPMI_CMD_ADDR(addr) |
	    (len - 1) | SPMI_CMD_LAST);

	while (len > 0) {
		memcpy(&data, cbuf, MIN(len, 4));
		SPMI_WRITE(sc, SPMI_CMD, data);
		cbuf += MIN(len, 4);
		len -= MIN(len, 4);
	}

	return apple_spmi_read_resp(sc, &resp);
}



static const struct device_compatible_entry compat_data[] = {
	{ .compat = "apple,spmi" },
	DEVICE_COMPAT_EOL
};

static int
apple_iic_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
apple_iic_attach(device_t parent, device_t self, void *aux)
{
	struct apple_iic_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;


void
apple_spmi_attach(device_t *parent, device_t *self, void *aux)
{
	struct apple_spmi_softc *sc = (struct apple_spmi_softc *)self;
	struct fdt_attach_args *faa = aux;
	struct spmi_attach_args sa;
	char name[32];
	uint32_t reg[2];
	int node;

	bus_addr_t addr;
	bus_size_t size;

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;

	int error = fdtbus_get_reg(phandle, 0, &addr, &size);
	if (error) {
		aprint_error(": unable to get device registers\n");
		return;
	}

	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh)) {
		aprint_error(": unable to map device\n");
		return;
	}

	aprint_naive("\n");
	aprint_normal(": Apple SPMI\n");



	sc->sc_tag.sc_cookie = sc;
	sc->sc_tag.sc_cmd_read = apple_spmi_cmd_read;
	sc->sc_tag.sc_cmd_write = apple_spmi_cmd_write;


	fdtbus_register_spmi_controller(&sc->sc_spmi, phandle);

	fdtbus_attach_spmibus(self, phandle, &sc->sc_spmi, spmibus_print);

#if 0
	for (node = OF_child(faa->fa_node); node; node = OF_peer(node)) {
		if (OF_getpropintarray(node, "reg", reg,
		    sizeof(reg)) != sizeof(reg))
			continue;

		memset(name, 0, sizeof(name));
		if (OF_getprop(node, "compatible", name, sizeof(name)) == -1)
			continue;
		if (name[0] == '\0')
			continue;

		memset(&sa, 0, sizeof(sa));
		sa.sa_tag = &sc->sc_tag;
		sa.sa_sid = reg[0];
		sa.sa_name = name;
		sa.sa_node = node;
		config_found(self, &sa, apple_spmi_print);
	}
#endif

}



CFATTACH_DECL_NEW(apple_spmi, sizeof(struct apple_spmi_softc),
    apple_spmi_match, apple_spmi_attach, NULL, NULL);
