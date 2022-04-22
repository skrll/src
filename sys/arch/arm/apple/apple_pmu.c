/*	$NetBSD$	*/
/*	$OpenBSD: aplpmu.c,v 1.5 2022/04/06 18:59:26 naddy Exp $	*/

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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/malloc.h>

#include <machine/bus.h>
#include <machine/fdt.h>

#include <dev/clock_subr.h>
#include <dev/fdt/spmivar.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_misc.h>
#include <dev/ofw/fdt.h>

extern void (*cpuresetfn)(void);
extern void (*powerdownfn)(void);

/*
 * This driver is based on preliminary device tree bindings and will
 * almost certainly need changes once the official bindings land in
 * mainline Linux.  Support for these preliminary bindings will be
 * dropped as soon as official bindings are available.
 */

/*
 * Apple's "sera" PMU contains an RTC that provides time in 32.16
 * fixed-point format as well as a time offset in 33.15 fixed-point
 * format.  The sum of the two gives us a standard Unix timestamp with
 * sub-second resolution.  The time itself is read-only.  To set the
 * time we need to adjust the time offset.
 */
#define SERA_TIME		0xd002
#define SERA_TIME_OFFSET	0xd100
#define SERA_TIME_LEN		6

#define SERA_POWERDOWN		0x9f0f
#define SERA_POWERDOWN_MAGIC	0x08

struct apple_pmu_nvmem {
	struct apple_pmu_softc	*an_sc;
	struct nvmem_device	an_nd;
	bus_addr_t		an_base;
	bus_size_t		an_size;
};

struct apple_pmu_softc {
	device_t		sc_dev;
	spmi_tag_t		sc_tag;
	int8_t			sc_sid;

	struct todr_chip_handle sc_todr;
	uint64_t		sc_offset;
};

struct apple_pmu_softc *apple_pmu_sc;


	apple_pmu_gettime(struct todr_chip_handle *, struct timeval *);
int	apple_pmu_settime(struct todr_chip_handle *, struct timeval *);
void	apple_pmu_powerdown(void);
int	apple_pmu_nvmem_read(void *, bus_addr_t, void *, bus_size_t);
int	apple_pmu_nvmem_write(void *, bus_addr_t, const void *, bus_size_t);


int
apple_pmu_gettime(struct todr_chip_handle *handle, struct timeval *tv)
{
	struct apple_pmu_softc *sc = handle->cookie;
	uint8_t data[8] = {};
	uint64_t time;
	int error;

	error = spmi_cmd_read(sc->sc_tag, sc->sc_sid, SPMI_CMD_EXT_READL,
	    SERA_TIME, &data, SERA_TIME_LEN);
	if (error)
		return error;
	time = le64dec(data) + (sc->sc_offset << 1);

	tv->tv_sec = (time >> 16);
	tv->tv_usec = (((time & 0xffff) * 1000000) >> 16);
	return 0;
}

int
apple_pmu_settime(struct todr_chip_handle *handle, struct timeval *tv)
{
	struct apple_pmu_softc *sc = handle->cookie;
	uint8_t data[8] = {};
	uint64_t time;
	int error;

	error = spmi_cmd_read(sc->sc_tag, sc->sc_sid, SPMI_CMD_EXT_READL,
	    SERA_TIME, &data, SERA_TIME_LEN);
	if (error)
		return error;

	time = ((uint64_t)tv->tv_sec << 16);
	time |= ((uint64_t)tv->tv_usec << 16) / 1000000;
	sc->sc_offset = ((time - le64dec(data)) >> 1);

	htolem64(data, sc->sc_offset);
	return spmi_cmd_write(sc->sc_tag, sc->sc_sid, SPMI_CMD_EXT_WRITEL,
	    SERA_TIME_OFFSET, &data, SERA_TIME_LEN);
}

void
apple_pmu_powerdown(void)
{
	struct apple_pmu_softc * const sc = an->an_sc;
	uint8_t data = SERA_POWERDOWN_MAGIC;

	spmi_cmd_write(sc->sc_tag, sc->sc_sid, SPMI_CMD_EXT_WRITEL,
	    SERA_POWERDOWN, &data, sizeof(data));

	cpuresetfn();
}

int
apple_pmu_nvmem_read(void *cookie, bus_addr_t addr, void *data, bus_size_t size)
{
	struct apple_pmu_nvmem * const an = cookie;
	struct apple_pmu_softc * const sc = an->an_sc;

	if (addr >= an->an_size || addr + size > an->an_size)
		return EINVAL;

	return spmi_cmd_read(sc->sc_tag, sc->sc_sid, SPMI_CMD_EXT_READL,
	    an->an_base + addr, data, size);
}

int
apple_pmu_nvmem_write(void *cookie, bus_addr_t addr, const void *data,
    bus_size_t size)
{
	struct apple_pmu_nvmem * const an = cookie;
	struct apple_pmu_softc * const sc = an->an_sc;

	if (addr >= an->an_size || addr + size > an->an_size)
		return EINVAL;

	return spmi_cmd_write(sc->sc_tag, sc->sc_sid, SPMI_CMD_EXT_WRITEL,
	    an->an_base + addr, data, size);
}

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "apple,sera-pmu" },
	{ .compat = "apple,spmi-pmu" },
	DEVICE_COMPAT_EOL
};


static int
apple_pmu_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
apple_pmu_attach(device_t parent, device_t self, void *aux)
{
	struct apple_pmu_softc * const sc = device_private(self);
	struct spmi_attach_args *sa = aux;
	uint8_t data[8] = {};
	int error, node;

	sc->sc_tag = sa->sa_tag;
	sc->sc_sid = sa->sa_sid;

	if (OF_is_compatible(sa->sa_node, "apple,sera-pmu")) {
		error = spmi_cmd_read(sc->sc_tag, sc->sc_sid,
		    SPMI_CMD_EXT_READL, SERA_TIME_OFFSET,
		    &data, SERA_TIME_LEN);
		if (error) {
			printf(": can't read offset\n");
			return;
		}
		sc->sc_offset = le64dec(data);

		sc->sc_todr.cookie = sc;
		sc->sc_todr.todr_gettime = apple_pmu_gettime;
		sc->sc_todr.todr_settime = apple_pmu_settime;
		todr_attach(&sc->sc_todr);

		apple_pmu_sc = sc;
		powerdownfn = apple_pmu_powerdown;
	}

	printf("\n");

	for (node = OF_child(sa->sa_node); node; node = OF_peer(node)) {
		struct apple_pmu_nvmem *an;
		uint32_t reg[2];

		if (!OF_is_compatible(node, "apple,spmi-pmu-nvmem"))
			continue;

		if (OF_getpropintarray(node, "reg", reg,
		    sizeof(reg)) != sizeof(reg))
			continue;

		an = malloc(sizeof(*an), M_DEVBUF, M_WAITOK);
		an->an_sc = sc;
		an->an_base = reg[0];
		an->an_size = reg[1];
		an->an_nd.nd_node = node;
		an->an_nd.nd_cookie = an;
		an->an_nd.nd_read = apple_pmu_nvmem_read;
		an->an_nd.nd_write = apple_pmu_nvmem_write;
		nvmem_register(&an->an_nd);
	}
}

CFATTACH_DECL_NEW(apple_pmu, sizeof(struct apple_pmu_softc),
    apple_pmu_match, apple_pmu_attach, NULL, NULL);
