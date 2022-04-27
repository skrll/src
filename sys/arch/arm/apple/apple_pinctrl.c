/*	$NetBSD: apple_pinctrl.c,v 1.1 2022/04/27 07:59:18 skrll Exp $	*/
/*	$OpenBSD: aplpinctrl.c,v 1.4 2022/04/06 18:59:26 naddy Exp $	*/

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
__KERNEL_RCSID(0, "$NetBSD: apple_pinctrl.c,v 1.1 2022/04/27 07:59:18 skrll Exp $");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/kmem.h>

#include <dev/fdt/fdtvar.h>

#include <arm/pic/picvar.h>

#define APPLE_PIN(pinmux)	__SHIFTOUT((pinmux), __BITS(15, 0))
#define APPLE_FUNC(pinmux)	__SHIFTOUT((pinmux), __BITS(31, 16))

#define GPIO_PIN(pin)		((pin) * 4)
#define  GPIO_PIN_GROUP_MASK	__BITS(18, 16)
#define  GPIO_PIN_INPUT_ENABLE	__BIT(9)
#define  GPIO_PIN_FUNC_MASK	__BITS(6, 5)
#define  GPIO_PIN_MODE_MASK	__BITS(3, 1)
#define  GPIO_PIN_MODE_INPUT	 __SHIFTIN(0, GPIO_PIN_MODE_MASK);
#define  GPIO_PIN_MODE_OUTPUT	 __SHIFTIN(1, GPIO_PIN_MODE_MASK);
#define  GPIO_PIN_MODE_IRQ_HI	 __SHIFTIN(2, GPIO_PIN_MODE_MASK);
#define  GPIO_PIN_MODE_IRQ_LO	 __SHIFTIN(3, GPIO_PIN_MODE_MASK);
#define  GPIO_PIN_MODE_IRQ_UP	 __SHIFTIN(4, GPIO_PIN_MODE_MASK);
#define  GPIO_PIN_MODE_IRQ_DN	 __SHIFTIN(5, GPIO_PIN_MODE_MASK);
#define  GPIO_PIN_MODE_IRQ_ANY	 __SHIFTIN(6, GPIO_PIN_MODE_MASK);
#define  GPIO_PIN_MODE_IRQ_OFF	 __SHIFTIN(7, GPIO_PIN_MODE_MASK);
#define  GPIO_PIN_DATA		__BIT(0)
#define GPIO_IRQ(grp, pin)	(0x800 + (grp) * 64 + ((pin) >> 5) * 4)

#define PINCTRL_READ(sc, reg)						\
	(bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg)))
#define PINCTRL_WRITE(sc, reg, val)					\
	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))
#define PINCTRL_SET(sc, reg, bits)					\
	PINCTRL_WRITE((sc), (reg), PINCTRL_READ((sc), (reg)) | (bits))
#define PINCTRL_CLR(sc, reg, bits)					\
	PINCTRL_WRITE((sc), (reg), PINCTRL_READ((sc), (reg)) & ~(bits))


//static int apple_pinctrl_intr(void *);

struct apple_pinctrl_softc {
	device_t sc_dev;
	int sc_phandle;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	u_int sc_npins;

#if 0
	struct pic_softc sc_pic;
#endif

#if 0
	// Other NetBSD
	void *gpio_is;
	void *gpio_is_high;

	int sc_unit;
	int sc_irqbase;
	uint32_t gpio_enable_mask;
	uint32_t gpio_edge_mask;
	uint32_t gpio_level_mask;
#if NGPIO > 0
	struct gpio_chipset_tag gpio_chipset;
	gpio_pin_t gpio_pins[32];
#endif

	kmutex_t gpio_lock;
#endif


#if 0
	// OpenBSD
	int			sc_npins;
	struct gpio_controller	sc_gc;

	void			*sc_ih;
	struct intrhand		**sc_handler;
	struct interrupt_controller sc_ic;
#endif
};

struct apple_gpio_pin {
	int		pin_no;
	u_int		pin_flags;
	bool		pin_actlo;
};



#if notyet
static struct fdtbus_interrupt_controller_func apple_pinctrl_intr_funcs = {
	.establish = apple_pinctrl_establish,
	.disestablish = apple_pinctrl_disestablish,
	.intrstr = apple_pinctrl_intrstr
};
#endif


static const struct device_compatible_entry compat_data[] = {
	{ .compat = "apple,pinctrl" },
	DEVICE_COMPAT_EOL
};


#if 0

int
apple_pinctrl_pinctrl(uint32_t phandle, void *cookie)
{
	struct apple_pinctrl_softc *sc = cookie;
	uint32_t *pinmux;
	int node, len, i;
	uint16_t pin, func;
	uint32_t reg;

	node = OF_getnodebyphandle(phandle);
	if (node == 0)
		return -1;

	len = OF_getproplen(node, "pinmux");
	if (len <= 0)
		return -1;

	pinmux = malloc(len, M_TEMP, M_WAITOK);
	OF_getpropintarray(node, "pinmux", pinmux, len);

	for (i = 0; i < len / sizeof(uint32_t); i++) {
		pin = APPLE_PIN(pinmux[i]);
		func = APPLE_FUNC(pinmux[i]);
		reg = PINCTRL_READ(sc, GPIO_PIN(pin));
		reg &= ~GPIO_PIN_FUNC_MASK;
		reg |= (func << GPIO_PIN_FUNC_SHIFT) & GPIO_PIN_FUNC_MASK;
		PINCTRL_WRITE(sc, GPIO_PIN(pin), reg);
	}

	free(pinmux, M_TEMP, len);
	return 0;
}
#endif

static void
apple_gpio_pin_ctl(void *cookie, int pin, int flags)
{
	struct apple_pinctrl_softc * const sc = cookie;

	KASSERT(pin < sc->sc_npins);

	uint32_t reg = PINCTRL_READ(sc, GPIO_PIN(pin));
	reg &= ~GPIO_PIN_FUNC_MASK;
	reg &= ~GPIO_PIN_MODE_MASK;

	if (flags & (GPIO_PIN_OUTPUT | GPIO_PIN_INPUT)) {
		if (flags & GPIO_PIN_INPUT) {
			/* for safety INPUT will override output */
			reg |= GPIO_PIN_MODE_INPUT;
		} else {
			reg |= GPIO_PIN_MODE_OUTPUT;
		}
	}
	PINCTRL_WRITE(sc, GPIO_PIN(pin), reg);
}

#if 0
int
apple_pinctrl_intr(void *arg)
{
	struct apple_pinctrl_softc *sc = arg;
	struct intrhand *ih;
	uint32_t status, pending;
	int base, pin, s;

	for (base = 0; base < sc->sc_npins; base += 32) {
		status = PINCTRL_READ(sc, GPIO_IRQ(0, base));
		pending = status;

		while (pending) {
			pin = ffs(pending) - 1;
			ih = sc->sc_handler[base + pin];

			if (ih) {
				s = splraise(ih->ih_ipl);
				if (ih->ih_func(ih->ih_arg))
					ih->ih_count.ec_count++;
				splx(s);
			}

			pending &= ~(1 << pin);
		}

		PINCTRL_WRITE(sc, GPIO_IRQ(0, base), status);
	}

	return 1;
}

void *
apple_pinctrl_intr_establish(void *cookie, int *cells, int ipl,
    struct cpu_info *ci, int (*func)(void *), void *arg, char *name)
{
	struct apple_pinctrl_softc *sc = cookie;
	struct intrhand *ih;
	uint32_t pin = cells[0];
	uint32_t type = IST_NONE;
	uint32_t reg;

	KASSERT(pin < sc->sc_npins);
	KASSERT(sc->sc_handler[pin] == NULL);

	if (ci != NULL && !CPU_IS_PRIMARY(ci))
		return NULL;

	switch (cells[1]) {
	case 1:
		type = IST_EDGE_RISING;
		break;
	case 2:
		type = IST_EDGE_FALLING;
		break;
	case 3:
		type = IST_EDGE_BOTH;
		break;
	case 4:
		type = IST_LEVEL_HIGH;
		break;
	case 8:
		type = IST_LEVEL_LOW;
		break;
	}

	ih = malloc(sizeof(*ih), M_DEVBUF, M_WAITOK);
	ih->ih_func = func;
	ih->ih_arg = arg;
	ih->ih_irq = pin;
	ih->ih_type = type;
	ih->ih_ipl = ipl;
	ih->ih_name = name;
	ih->ih_sc = sc;

	if (name != NULL)
		evcount_attach(&ih->ih_count, name, &ih->ih_irq);

	sc->sc_handler[pin] = ih;

	reg = PINCTRL_READ(sc, GPIO_PIN(pin));
	reg &= ~GPIO_PIN_DATA;
	reg &= ~GPIO_PIN_FUNC_MASK;
	reg &= ~GPIO_PIN_MODE_MASK;
	switch (type) {
	case IST_NONE:
		reg |= GPIO_PIN_MODE_IRQ_OFF
		break;
	case IST_EDGE_RISING:
		reg |= GPIO_PIN_MODE_IRQ_UP
		break;
	case IST_EDGE_FALLING:
		reg |= GPIO_PIN_MODE_IRQ_DN
		break;
	case IST_EDGE_BOTH:
		reg |= GPIO_PIN_MODE_IRQ_ANY
		break;
	case IST_LEVEL_HIGH:
		reg |= GPIO_PIN_MODE_IRQ_HI
		break;
	case IST_LEVEL_LOW:
		reg |= GPIO_PIN_MODE_IRQ_LO
		break;
	}
	reg |= GPIO_PIN_INPUT_ENABLE;
	reg &= ~GPIO_PIN_GROUP_MASK;
	PINCTRL_WRITE(sc, GPIO_PIN(pin), reg);

	return ih;
}

void
apple_pinctrl_intr_disestablish(void *cookie)
{
	struct intrhand *ih = cookie;
	struct apple_pinctrl_softc * const sc = ih->ih_sc;
	uint32_t reg;
	int s;

	s = splhigh();

	reg = PINCTRL_READ(sc, GPIO_PIN(ih->ih_irq));
	reg &= ~GPIO_PIN_MODE_MASK;
	reg |= GPIO_PIN_MODE_IRQ_OFF
	PINCTRL_WRITE(sc, GPIO_PIN(ih->ih_irq), reg);

	sc->sc_handler[ih->ih_irq] = NULL;
	if (ih->ih_name)
		evcount_detach(&ih->ih_count);
	free(ih, M_DEVBUF, sizeof(*ih));

	splx(s);
}

void
apple_pinctrl_intr_enable(void *cookie)
{
	struct intrhand *ih = cookie;
	struct apple_pinctrl_softc * const sc = ih->ih_sc;
	uint32_t reg;
	int s;

	s = splhigh();
	reg = PINCTRL_READ(sc, GPIO_PIN(ih->ih_irq));
	reg &= ~GPIO_PIN_MODE_MASK;
	switch (ih->ih_type) {
	case IST_NONE:
		reg |= GPIO_PIN_MODE_IRQ_OFF
		break;
	case IST_EDGE_RISING:
		reg |= GPIO_PIN_MODE_IRQ_UP
		break;
	case IST_EDGE_FALLING:
		reg |= GPIO_PIN_MODE_IRQ_DN
		break;
	case IST_EDGE_BOTH:
		reg |= GPIO_PIN_MODE_IRQ_ANY
		break;
	case IST_LEVEL_HIGH:
		reg |= GPIO_PIN_MODE_IRQ_HI
		break;
	case IST_LEVEL_LOW:
		reg |= GPIO_PIN_MODE_IRQ_LO
		break;
	}
	PINCTRL_WRITE(sc, GPIO_PIN(ih->ih_irq), reg);
	splx(s);
}

void
apple_pinctrl_intr_disable(void *cookie)
{
	struct intrhand *ih = cookie;
	struct apple_pinctrl_softc *sc = ih->ih_sc;
	uint32_t reg;
	int s;

	s = splhigh();
	reg = PINCTRL_READ(sc, GPIO_PIN(ih->ih_irq));
	reg &= ~GPIO_PIN_MODE_MASK;
	reg |= GPIO_PIN_MODE_IRQ_OFF
	PINCTRL_WRITE(sc, GPIO_PIN(ih->ih_irq), reg);
	splx(s);
}

#endif

static void *
apple_pinctrl_gpio_acquire(device_t dev, const void *data, size_t len, int flags)
{
	struct apple_pinctrl_softc * const sc = device_private(dev);
	struct apple_gpio_pin *pin;
	const u_int *gpio = data;

	if (len != 12)
		return NULL;

	const u_int pinno = be32toh(gpio[1]);
	const bool actlo = be32toh(gpio[2]) & 1;	// Why hasn't NetBSD got GPIO_ACTIVE_LOW here?

	if (pinno >= sc->sc_npins)
		return NULL;

	pin = kmem_alloc(sizeof(*pin), KM_SLEEP);
	pin->pin_no = pinno;
	pin->pin_flags = flags;
	pin->pin_actlo = actlo;

	apple_gpio_pin_ctl(sc, pin->pin_no, pin->pin_flags);

	return pin;
}

static void
apple_pinctrl_gpio_release(device_t dev, void *priv)
{
	struct apple_pinctrl_softc * const sc = device_private(dev);
	struct apple_gpio_pin *pin = priv;

	apple_gpio_pin_ctl(sc, pin->pin_no, GPIO_PIN_INPUT);
	kmem_free(pin, sizeof(*pin));
}



static int
apple_pinctrl_gpio_read(device_t dev, void *priv, bool raw)
{
	struct apple_pinctrl_softc * const sc = device_private(dev);
	struct apple_gpio_pin *pin = priv;

	KASSERT(pin->pin_no < sc->sc_npins);

	uint32_t reg = PINCTRL_READ(sc, GPIO_PIN(pin->pin_no));
	int val = __SHIFTOUT(reg, GPIO_PIN_DATA);
	if (!raw && pin->pin_actlo)
		val = !val;

	return val;
}

static void
apple_pinctrl_gpio_write(device_t dev, void *priv, int val, bool raw)
{
	struct apple_pinctrl_softc * const sc = device_private(dev);
	struct apple_gpio_pin *pin = priv;

	KASSERT(pin->pin_no < sc->sc_npins);

	if (!raw && pin->pin_actlo)
		val = !val;

	if (val)
		PINCTRL_SET(sc, GPIO_PIN(pin->pin_no), GPIO_PIN_DATA);
	else
		PINCTRL_CLR(sc, GPIO_PIN(pin->pin_no), GPIO_PIN_DATA);
}


static int
apple_pinctrl_set_config(device_t dev, const void *data, size_t len)
{
	struct apple_pinctrl_softc * const sc = device_private(dev);

	if (len != 4)
		return -1;

	int pins_len;
	const int phandle = fdtbus_get_phandle_from_native(be32dec(data));
	const u_int *pins = fdtbus_get_prop(phandle, "pinmux", &pins_len);

	if (pins == NULL)
		return -1;

	const u_int npins = pins_len / sizeof(uint32_t);

	for (u_int i = 0; i < npins; i++) {
		uint32_t pinmux = be32dec(&pins[i]);
		u_int pinno = APPLE_PIN(pinmux);
		u_int func = APPLE_FUNC(pinmux);

		uint32_t reg = PINCTRL_READ(sc, GPIO_PIN(pinno));
		reg &= ~GPIO_PIN_FUNC_MASK;
		reg |= __SHIFTIN(func, GPIO_PIN_FUNC_MASK);

		PINCTRL_WRITE(sc, GPIO_PIN(pinno), reg);
	}

	return 0;
}



static struct fdtbus_gpio_controller_func apple_pinctrl_gpio_funcs = {
	.acquire = apple_pinctrl_gpio_acquire,
	.release = apple_pinctrl_gpio_release,
	.read = apple_pinctrl_gpio_read,
	.write = apple_pinctrl_gpio_write
};

static struct fdtbus_pinctrl_controller_func apple_pinctrl_funcs = {
	.set_config = apple_pinctrl_set_config,
};








static int
apple_pinctrl_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
apple_pinctrl_attach(device_t parent, device_t self, void *aux)
{
	struct apple_pinctrl_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;
//	char intrstr[128];
//	int error;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;

	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	if (of_getprop_uint32(phandle, "apple,npins", &sc->sc_npins)) {
		aprint_error(": couldn't get number of pins\n");
		return;
	}

	if (!of_hasprop(phandle, "gpio-controller")) {
		aprint_error(": no gpio controller");
		return;
	}

	aprint_naive("\n");
	aprint_normal(": Apple Pinctrl\n");



#if 0
	sc->sc_unit = -1;
	sc->sc_irqbase = PIC_IRQBASE_ALLOC;

	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr))) {
		aprint_error_dev(self, "failed to decode interrupt\n");
		return;
	}


	// 7 interrupts



	// XXXNH multiple interrupts?!?
	sc->gpio_is = fdtbus_intr_establish_xname(phandle, 0, IPL_HIGH, 0,
	    pic_handle_intr, &sc->sc_pic, device_xname(self));
	if (sc->gpio_is == NULL) {
		aprint_error_dev(self, "couldn't establish interrupt on %s\n",
		    intrstr);
		return;
	}
	aprint_normal_dev(self, "interrupting on %s\n", intrstr);

	if (!fdtbus_intr_str(phandle, 1, intrstr, sizeof(intrstr))) {
		aprint_error_dev(self, "failed to decode interrupt\n");
		return;
	}
	sc->gpio_is_high = fdtbus_intr_establish_xname(phandle, 1, IPL_HIGH, 0,
	    pic_handle_intr, &sc->sc_pic, device_xname(self));
	if (sc->gpio_is_high == NULL) {
		aprint_error_dev(self, "couldn't establish interrupt on %s\n",
		    intrstr);
		return;
	}
	aprint_normal_dev(self, "interrupting on %s\n", intrstr);


#endif





	fdtbus_register_gpio_controller(self, phandle, &apple_pinctrl_gpio_funcs);

	for (int child = OF_child(phandle); child; child = OF_peer(child)) {
		if (!of_hasprop(child, "pinmux"))
			continue;
		fdtbus_register_pinctrl_config(self, child,
		    &apple_pinctrl_funcs);

	}

#if 0

	error = fdtbus_register_interrupt_controller(self, phandle,
	    &apple_pinctrl_funcs);
	if (error) {
		aprint_error(": couldn't register with fdtbus: %d\n", error);
		return;
	}



#endif
}


CFATTACH_DECL_NEW(apple_pinctrl, sizeof(struct apple_pinctrl_softc),
    apple_pinctrl_match, apple_pinctrl_attach, NULL, NULL);

