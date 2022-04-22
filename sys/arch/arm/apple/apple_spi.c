/*	$NetBSD$	*/
/*	$OpenBSD: aplspi.c,v 1.4 2022/04/06 18:59:26 naddy Exp $	*/

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

#include <sys/bus.h>
#include <sys/device.h>
#include <sys/kernel.h>

#if 0
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/malloc.h>
#include <sys/mutex.h>

#include <machine/bus.h>
#include <machine/fdt.h>

#include <dev/spi/spivar.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_clock.h>
#include <dev/ofw/ofw_gpio.h>
#include <dev/ofw/ofw_pinctrl.h>
#include <dev/ofw/fdt.h>
#endif

#include <dev/spi/spivar.h>

#include <dev/fdt/fdtvar.h>

#define SPL_CTRL		0x00
#define  SPL_CTRL_RUN		__BIT(0)
#define  SPL_CTRL_TX_RESET	__BIT(2)
#define  SPL_CTRL_RX_RESET	__BIT(3)
#define  SPL_CTRL_EN		(SPL_CTRL_RUN | SPL_CTRL_TX_RESET | SPL_CTRL_RX_RESET)
#define SPI_CONFIG		0x04
#define  SPI_CONFIG_EN		__BIT(18)
#define  SPI_CONFIG_PIOEN	__BIT(5)
#define SPI_STATUS		0x08
#define  SPI_STATUS_RXDONE	__BITS(0)
#define  SPI_STATUS_RXTX	__BITS(1)
#define  SPI_STATUS_TXDONE	__BITS(2)
#define SPI_PIN			0x0c
#define  SPI_PIN_CS		__BIT(1)
#define SPI_TXDATA		0x10
#define SPI_RXDATA		0x20
#define SPI_CLKDIV		0x30
#define  SPI_CLKDIV_MIN		2
#define  SPI_CLKDIV_MAX		2047
#define SPI_RXCNT		0x34
#define SPI_CLKIDLE		0x38
#define SPI_TXCNT		0x4c
#define SPI_AVAIL		0x10c
#define  SPI_AVAIL_RX_MASK	__BITS(31, 24)
#define  SPI_AVAIL_RX(avail)	__SHIFTOUT(avail, SPI_AVAIL_RX_MASK)
#define  SPI_AVAIL_TX_MASK	__BITS(15, 8)
#define  SPI_AVAIL_TX(avail)	__SHIFTOUT(avail, SPI_AVAIL_TX_MASK)
#define SPI_IE_XFER		0x130
#define SPI_IF_XFER		0x134
#define  SPI_XFER_RXDONE	__BIT(0)
#define  SPI_XFER_TXDONE	__BIT(1)
#define SPI_IE_FIFO		0x138
#define SPI_IF_FIFO		0x13c
#define  SPI_FIFO_RXTHRESH	__BIT(4)
#define  SPI_FIFO_TXTHRESH	__BIT(5)
#define  SPI_FIFO_RXFULL	__BIT(8)
#define  SPI_FIFO_TXEMPTY	__BIT(9)
#define  SPI_FIFO_RXUNDERRUN	__BIT(16)
#define  SPI_FIFO_TXOVERFLOW	__BIT(17)
#define SPI_SHIFTCFG		0x150
#define  SPI_SHIFTCFG_OVERRIDE_CS	__BIT(24)
#define SPI_PINCFG		0x154
#define  SPI_PINCFG_KEEP_CS	__BIT(1)
#define  SPI_PINCFG_CS_IDLE_VAL	__BIT(9)

#define SPI_FIFO_SIZE		16

struct apple_spi_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	int			sc_node;

	struct clk *		sc_clk;
	uint32_t		sc_pfreq;

	void *			sc_ih;

	struct spi_controller	sc_spi;
	kmutex_t                sc_mutex;

	SIMPLEQ_HEAD(,spi_transfer)
				sc_q;

	struct spi_transfer	*sc_transfer;
	struct spi_chunk	*sc_wchunk;
	struct spi_chunk	*sc_rchunk;
	bool		sc_running;
};

int	 apple_spi_acquire_bus(void *, int);
void	 apple_spi_release_bus(void *, int);

void	 apple_spi_set_cs(struct apple_spi_softc *, int, int);
int	 apple_spi_wait_state(struct apple_spi_softc *, uint32_t, uint32_t);

void	 apple_spi_scan(struct apple_spi_softc *);

#define SPI_READ(sc, reg)						\
	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define SPI_WRITE(sc, reg, val)						\
	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))
#define SPI_SET(sc, reg, bits)						\
	SPI_WRITE((sc), (reg), SPI_READ((sc), (reg)) | (bits))
#define SPI_CLR(sc, reg, bits)						\
	SPI_WRITE((sc), (reg), SPI_READ((sc), (reg)) & ~(bits))

static uint32_t
apple_spi_clkdiv(struct apple_spi_softc *sc, uint32_t freq)
{
	uint32_t div = 0;

	while ((freq * div) < sc->sc_pfreq)
		div++;
	if (div < SPI_CLKDIV_MIN)
		div = SPI_CLKDIV_MIN;
	if (div > SPI_CLKDIV_MAX)
		div = SPI_CLKDIV_MAX;

	return div << 1;
}

static int
apple_spi_configure(void *cookie, int slave, int mode, int speed)
{
	struct apple_spi_softc * const sc = cookie;

	if (slave >= sc->sc_spi.sct_nslaves)
		return EINVAL;

	if (speed <= 0)
		return EINVAL;

	SPI_WRITE(sc, SPL_CTRL, 0);

	SPI_WRITE(sc, SPI_CLKDIV, apple_spi_clkdiv(sc, speed));
	SPI_WRITE(sc, SPI_CLKIDLE, 0);

	SPI_WRITE(sc, SPI_CONFIG, SPI_CONFIG_EN);
	SPI_WRITE(sc, SPL_CTRL, SPL_CTRL_EN);
	SPI_READ(sc, SPI_CONFIG);

	return 0;
}

void
apple_spi_set_cs(struct apple_spi_softc *sc, int cs, int on)
{
	KASSERT(cs == 0);
	SPI_WRITE(sc, SPI_PIN, on ? 0 : SPI_PIN_CS);
}


static void
apple_spi_send(struct apple_spi_softc * const sc)
{
	const uint32_t avail = SPI_READ(sc, SPI_AVAIL);
	int count = SPI_FIFO_SIZE - SPI_AVAIL_TX(avail);
	struct spi_chunk *chunk;

	while ((chunk = sc->sc_wchunk) != NULL) {
		while (chunk->chunk_wresid) {
			if (count == 0)
				return;

			uint32_t data = chunk->chunk_wptr ?
			    *chunk->chunk_wptr++ : '\0';
			SPI_WRITE(sc, SPI_TXDATA, data);
			chunk->chunk_wresid--;
			count--;
		}
		sc->sc_wchunk = sc->sc_wchunk->chunk_next;
	}
}

static void
apple_spi_recv(struct apple_spi_softc * const sc)
{
	const uint32_t avail = SPI_READ(sc, SPI_AVAIL);
	int count = SPI_AVAIL_RX(avail);
	struct spi_chunk *chunk;

	while ((chunk = sc->sc_rchunk) != NULL) {
		while (chunk->chunk_rresid) {
			if (count == 0)
				return;

			uint32_t data = SPI_READ(sc, SPI_RXDATA);
			if (chunk->chunk_rptr) {
				*chunk->chunk_rptr++ = data & 0xff;
			}
			chunk->chunk_rresid--;
			count--;
		}
		sc->sc_rchunk = sc->sc_rchunk->chunk_next;
	}
}


static int
apple_spi_intr_locked(struct apple_spi_softc * const sc)
{

	uint32_t xfer_status = SPI_READ(sc, SPI_IF_XFER);
	uint32_t fifo_status = SPI_READ(sc, SPI_IF_FIFO);
	const uint32_t xfer_done = SPI_XFER_RXDONE | SPI_XFER_TXDONE;

	if (ISSET(xfer_status, xfer_done)) {
		if (sc->sc_wchunk != NULL) {
			apple_spi_send(sc);
		} else {
//			SPI_WRITE(sc, SPI_CS, sc->sc_CS);
			apple_spi_recv(sc);
			sc->sc_rchunk = sc->sc_wchunk = NULL;
			struct spi_transfer *st = sc->sc_transfer;
			sc->sc_transfer = NULL;
			KASSERT(st != NULL);
			spi_done(st, 0);
			sc->sc_running = false;
		}
	// RX Fifo needs reading.
	} else if (ISSET(fifo_status, SPI_FIFO_RXFULL)) {
		apple_spi_recv(sc);
		apple_spi_send(sc);
	}

	return ISSET(xfer_status, xfer_done);
}


static int
apple_spi_intr(void *cookie)
{
	struct apple_spi_softc * const sc = cookie;

	mutex_enter(&sc->sc_mutex);
	int done = apple_spi_intr_locked(sc);
	mutex_exit(&sc->sc_mutex);

	return done;
}


static void
apple_spi_start(struct apple_spi_softc * const sc)
{
	struct spi_transfer *st;
//	uint32_t cs;

	while ((st = spi_transq_first(&sc->sc_q)) != NULL) {

		spi_transq_dequeue(&sc->sc_q);

		KASSERT(sc->sc_transfer == NULL);
		sc->sc_transfer = st;
		sc->sc_rchunk = sc->sc_wchunk = st->st_chunks;

		// XXXNH wchunk vs rchunk ?!?
		SPI_WRITE(sc, SPI_TXCNT, sc->sc_wchunk->chunk_wresid);
		SPI_WRITE(sc, SPI_RXCNT, sc->sc_rchunk->chunk_rresid);
		SPI_WRITE(sc, SPI_CONFIG, SPI_CONFIG_EN | SPI_CONFIG_PIOEN);

		if (!cold)
			return;

		for (;;) {
			apple_spi_intr_locked(sc);

			if (ISSET(st->st_flags, SPI_F_DONE))
				break;
		}
	}

	sc->sc_running = false;
}

#if 0

	uint32_t avail, data, status;
	int rsplen;
	int count;

	apple_spi_set_cs(sc, sc->sc_cs, 1);
	delay(sc->sc_cs_delay);

	SPI_WRITE(sc, SPI_TXCNT, len);
	SPI_WRITE(sc, SPI_RXCNT, len);
	SPI_WRITE(sc, SPI_CONFIG, SPI_CONFIG_EN | SPI_CONFIG_PIOEN);

	rsplen = len;
#if 0
	while (len > 0 || rsplen > 0) {
		avail = SPI_READ(sc, SPI_AVAIL);
		count = SPI_AVAIL_RX(avail);
		while (rsplen > 0 && count > 0) {
			data = SPI_READ(sc, SPI_RXDATA);
			if (in)
				*in++ = data;
			rsplen--;

			avail = SPI_READ(sc, SPI_AVAIL);
			count = SPI_AVAIL_RX(avail);
		}

		count = SPI_FIFO_SIZE - SPI_AVAIL_TX(avail);
		while (len > 0 && count > 0) {
			data = out ? *out++ : 0;
			SPI_WRITE(sc, SPI_TXDATA, data);
			len--;
			count--;
		}
	}
#endif
	SPI_WRITE(sc, SPI_CONFIG, SPI_CONFIG_EN);
	status = SPI_READ(sc, SPI_STATUS);
	SPI_WRITE(sc, SPI_STATUS, status);

	if (!ISSET(flags, SPI_KEEP_CS))
		apple_spi_set_cs(sc, sc->sc_cs, 0);

	return 0;
}
#endif

static int
apple_spi_transfer(void *cookie, struct spi_transfer *st)
{
	struct apple_spi_softc * const sc = cookie;

	mutex_enter(&sc->sc_mutex);
	spi_transq_enqueue(&sc->sc_q, st);
	if (sc->sc_running == false) {
		apple_spi_start(sc);
	}
	mutex_exit(&sc->sc_mutex);

	return 0;
}


static const struct device_compatible_entry compat_data[] = {
	{ .compat = "apple,spi" },
	DEVICE_COMPAT_EOL
};

static int
apple_spi_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}


static void
apple_spi_attach(device_t parent, device_t self, void *aux)
{
	struct apple_spi_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;

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

	sc->sc_clk = fdtbus_clock_get_index(phandle, 0);
	if (sc->sc_clk == NULL) {
		aprint_error(": couldn't get clock\n");
		return;
	}

	sc->sc_pfreq = clk_get_rate(sc->sc_clk);

	sc->sc_ih = fdtbus_intr_establish_xname(phandle, 0, IPL_VM, 0,
	    apple_spi_intr, sc, device_xname(self));
	if (sc->sc_ih == NULL) {
		aprint_error(": unable to establish interrupt\n");
		return;
	}


	aprint_naive("\n");
	aprint_normal(": Apple SPI\n");

//	aprint_normal_dev(self, "interrupting on %s\n", intrstr);

	/* Configure CS# pin for manual control. */
	SPI_WRITE(sc, SPI_PIN, SPI_PIN_CS);
	SPI_CLR(sc, SPI_SHIFTCFG, SPI_SHIFTCFG_OVERRIDE_CS);
	SPI_CLR(sc, SPI_PINCFG, SPI_PINCFG_CS_IDLE_VAL);
	SPI_SET(sc, SPI_PINCFG, SPI_PINCFG_KEEP_CS);

	sc->sc_spi.sct_cookie = sc;
	sc->sc_spi.sct_configure = apple_spi_configure;
	sc->sc_spi.sct_transfer = apple_spi_transfer;
	sc->sc_spi.sct_nslaves = 1;

	spibus_attach(self, &sc->sc_spi);

#if 0
	sc->sc_tag.sc_cookie = sc;
	sc->sc_tag.sc_config = apple_spi_config;
	sc->sc_tag.sc_transfer = apple_spi_transfer;
	sc->sc_tag.sc_acquire_bus = apple_spi_acquire_bus;
	sc->sc_tag.sc_release_bus = apple_spi_release_bus;

	mtx_init(&sc->sc_mtx, IPL_TTY);

	apple_spi_scan(sc);

#endif
}

CFATTACH_DECL_NEW(apple_spi, sizeof(struct apple_spi_softc),
    apple_spi_match, apple_spi_attach, NULL, NULL);
