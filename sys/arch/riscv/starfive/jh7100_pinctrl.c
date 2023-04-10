/* $NetBSD: jh7100_pinctrl.c,v 1.3 2024/09/18 08:31:50 skrll Exp $ */

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
__KERNEL_RCSID(0, "$NetBSD: jh7100_pinctrl.c,v 1.3 2024/09/18 08:31:50 skrll Exp $");

#include <sys/param.h>

#include <sys/kmem.h>

#include <dev/fdt/fdtvar.h>

#define JH7100_GPIO_MAX_IRQS 1	// XXXNH

struct jh7100_gpio_irq {
	int	(*jgi_func)(void *);
	void	 *jgi_arg;
	bool	  jgi_mpsafe;
//	int	  jgi_bank;
//	int	  jgi_num;
};


struct jh7100_pinctrl_softc {
	device_t		 sc_dev;
	bus_space_tag_t		 sc_bst;
	bus_space_handle_t	 sc_gpio_bsh;
	bus_space_handle_t	 sc_padctl_bsh;
	int			 sc_phandle;

	kmutex_t		 sc_lock;
	u_int			 sc_padctl_gpio;

	void			*sc_ih;
//	struct jh7100_gpio_jgi	 sc_jgi[JH7100_GPIO_MAX_IRQS];
};

struct jh7100_pinctrl_gpio_pin {
	struct jh7100_pinctrl_softc	*pin_sc;
	u_int				 pin_no;
	bool				 pin_actlo;
};


#define GPIORD4(sc, reg)						       \
	bus_space_read_4((sc)->sc_bst, (sc)->sc_gpio_bsh, (reg))
#define GPIOWR4(sc, reg, val)						       \
	bus_space_write_4((sc)->sc_bst, (sc)->sc_gpio_bsh, (reg), (val))


#define	GPIO_ENABLE			0x0000
#define	 GPIO_ENABLE_IRQS		__BIT(0)


#define GPIO_DIN(pin)			(0x0048 + (((pin) / 32) * 4))

#define GPO_DOUT_CFG(pin)		(0x0050 + ((pin) * 8))
#define  GPO_DOUT_REVERSE		__BIT(31)
#define  GPO_DOUT_MASK			__BITS(30,  0)
#define GPO_DOEN_CFG(pin)		(0x0054 + ((pin) * 8))
#define  GPO_DOEN_REVERSE		__BIT(31)
#define  GPO_DOEN_MASK			__BITS(30,  0)
#define  GPO_ENABLE			0
#define  GPO_DISABLE			1
#define GPI_DIN(din)			(0x0250 + ((din) * 4))
#define  GPI_NONE			0xff


#define PCTLRD4(sc, reg)						       \
	bus_space_read_4((sc)->sc_bst, (sc)->sc_padctl_bsh, (reg))
#define PCTLWR4(sc, reg, val)						       \
	bus_space_write_4((sc)->sc_bst, (sc)->sc_padctl_bsh, (reg), (val))

#define PAD_GPIO(pin)			(0x0000 + (((pin) / 2) * 4))
#define PAD_SHIFT(pin)			((pin % 2) * 16)
#define  PAD_SLEW_RATE_MASK		__BITS(11, 9)
#define  PAD_BIAS_STRONG_PULLUP		__BIT(8)
#define  PAD_INPUT_ENABLE		__BIT(7)
#define  PAD_INPUT_SCHMITT_ENABLE	__BIT(6)
#define  PAD_BIAS_DISABLE		__BIT(5)
#define  PAD_BIAS_PULLDOWN		__BIT(4)
#define  PAD_DRIVE_STRENGTH_MASK	__BITS( 3, 0)

#define  PAD_BIAS_MASK 			(PAD_BIAS_STRONG_PULLUP |	       \
					 PAD_BIAS_DISABLE |		       \
					 PAD_BIAS_PULLDOWN)

#define PAD_DS_MAX			__SHIFTOUT_MASK(PAD_DRIVE_STRENGTH_MASK)
#define GPIO_DS_TO_MA(ds)		((ds) * 7 + 14)
#define GPIO_DS_MIN			GPIO_DS_TO_MA(0)
#define GPIO_DS_MAX			GPIO_DS_TO_MA(PAD_DS_MAX)

#define PAD_FUNC_SHARE(pad)		(GPIO_NPINS + (pad))
#define IO_PADSHARE_SEL			0x01a0

#define GPIO_NPINS			64


/* Device Tree encoding */
#define DT_GPIOMUX_DOUT_MASK	__BITS(31, 24)
#define DT_GPIOMUX_DOEN_MASK	__BITS(23, 16)
#define DT_GPIOMUX_DIN_MASK	__BITS(15,  8)
#define DT_GPIOMUX_DOUTREV_MASK	__BIT(7)
#define DT_GPIOMUX_DOENREV_MASK	__BIT(6)
#define DT_GPIOMUX_GPIO_MASK	__BITS( 5,  0)

#define DT_PAD_GPIO(x)		((x) & (GPIO_NPINS - 1))
#define DT_PAD_FUNC_SHARE(x)	((x) - GPIO_NPINS)


static const struct device_compatible_entry compat_data[] = {
	{ .compat = "starfive,jh7100-pinctrl" },
	DEVICE_COMPAT_EOL
};


static inline void
jh7100_padctl_rmw(struct jh7100_pinctrl_softc * const sc, u_int pad_no,
    uint16_t val, uint16_t mask)
{
	const bus_size_t regoff = PAD_GPIO(pad_no);
	const u_int shift = PAD_SHIFT(pad_no);
	const uint32_t regmask = mask << shift;
	const uint32_t regval = val << shift;

	mutex_enter(&sc->sc_lock);
	uint32_t reg = PCTLRD4(sc, regoff);
	uint32_t oreg = reg;
	reg &= ~regmask;
	reg |= regval;
	PCTLWR4(sc, regoff, reg);
	mutex_exit(&sc->sc_lock);

	aprint_debug_dev(sc->sc_dev, "pad %d (pin %d) %08x -> %08x (%#"
	    PRIxBUSSIZE ")\n", pad_no, pad_no - sc->sc_padctl_gpio,
	    oreg, reg, regoff);
}


static void
jh7100_gpio_pin_ctl(void *priv, int pin, int flags)
{
	struct jh7100_pinctrl_softc * const sc = priv;
printf("%s\n", __func__);
	if (flags & GPIO_PIN_OUTPUT) {
		GPIOWR4(sc, GPO_DOEN_CFG(pin), GPO_ENABLE);
        } else {

		GPIOWR4(sc, GPO_DOEN_CFG(pin), GPO_DISABLE);
	}
}


#if 0
static int
jh7100_gpio_intr(void *priv)
{
	struct jh7100_pinctrl_softc * const sc = priv;

	int ret = 0;

	while ((bit = ffs32(status)) != 0) {
		status &= ~__BIT(bit - 1);
		jgi = &sc->sc_jgi[bit - 1];
		if (jgi->jgi_func == NULL)
			continue;
		if (!jgi->jgi_mpsafe)
			KERNEL_LOCK(1, curlwp);
		ret |= jgi->jgi_func(jgi->jgi_arg);
		if (!jgi->jgi_mpsafe)
			KERNEL_UNLOCK_ONE(curlwp);
	}

	return ret;
}




static void *
jh7100_intr_enable(struct jh7100_pinctrl_softc *sc,
    const struct jh7100_pinctrl_pins *pin_def, u_int mode, bool mpsafe,
    int (*func)(void *), void *arg)
{
	uint32_t val;
	struct jh7100_pinctrl_jgi *jgi;

	if (pin_def->functions[pin_def->jgi_func] == NULL ||
	    strcmp(pin_def->functions[pin_def->jgi_func], "irq") != 0)
		return NULL;

	KASSERT(pin_def->jgi_num < SUNXI_GPIO_MAX_EINT);

	mutex_enter(&sc->sc_lock);

	jgi = &sc->sc_jgi[pin_def->jgi_bank][pin_def->jgi_num];
	if (jgi->jgi_func != NULL) {
		mutex_exit(&sc->sc_lock);
		return NULL;	/* in use */
	}

	/* Set function */
	if (jh7100_pinctrl_setfunc(sc, pin_def, "irq") != 0) {
		mutex_exit(&sc->sc_lock);
		return NULL;
	}

	jgi->jgi_func = func;
	jgi->jgi_arg = arg;
	jgi->jgi_mpsafe = mpsafe;
	jgi->jgi_bank = pin_def->jgi_bank;
	jgi->jgi_num = pin_def->jgi_num;

	/* Configure jgi mode */
	val = GPIO_READ(sc, SUNXI_GPIO_INT_CFG(jgi->jgi_bank, jgi->jgi_num));
	val &= ~SUNXI_GPIO_INT_MODEMASK(jgi->jgi_num);
	val |= __SHIFTIN(mode, SUNXI_GPIO_INT_MODEMASK(jgi->jgi_num));
	GPIO_WRITE(sc, SUNXI_GPIO_INT_CFG(jgi->jgi_bank, jgi->jgi_num), val);

	val = SUNXI_GPIO_INT_DEBOUNCE_CLK_SEL;
	GPIO_WRITE(sc, SUNXI_GPIO_INT_DEBOUNCE(jgi->jgi_bank), val);

	/* Enable jgi */
	val = GPIO_READ(sc, SUNXI_GPIO_INT_CTL(jgi->jgi_bank));
	val |= __BIT(jgi->jgi_num);
	GPIO_WRITE(sc, SUNXI_GPIO_INT_CTL(jgi->jgi_bank), val);

	mutex_exit(&sc->sc_lock);

	return jgi;
}

static void
jh7100_intr_disable(struct jh7100_pinctrl_softc *sc, struct jh7100_pinctrl_jgi *jgi)
{
	uint32_t val;

	KASSERT(jgi->jgi_func != NULL);

	mutex_enter(&sc->sc_lock);

	/* Disable jgi */
	val = GPIO_READ(sc, SUNXI_GPIO_INT_CTL(jgi->jgi_bank));
	val &= ~__BIT(jgi->jgi_num);
	GPIO_WRITE(sc, SUNXI_GPIO_INT_CTL(jgi->jgi_bank), val);
	GPIO_WRITE(sc, SUNXI_GPIO_INT_STATUS(jgi->jgi_bank), __BIT(jgi->jgi_num));

	jgi->jgi_func = NULL;
	jgi->jgi_arg = NULL;
	jgi->jgi_mpsafe = false;

	mutex_exit(&sc->sc_lock);
}

static void *
jh7100_fdt_intr_establish(device_t dev, u_int *specifier, int ipl, int flags,
    int (*func)(void *), void *arg, const char *xname)
{
	struct jh7100_pinctrl_softc * const sc = device_private(dev);
	bool mpsafe = (flags & FDT_INTR_MPSAFE) != 0;
	const struct jh7100_pinctrl_pins *pin_def;
	u_int mode;

	if (ipl != IPL_VM) {
		aprint_error_dev(dev, "%s: wrong IPL %d (expected %d)\n",
		    __func__, ipl, IPL_VM);
		return NULL;
	}

	/* 1st cell is the bank */
	/* 2nd cell is the pin */
	/* 3rd cell is flags */
	const u_int port = be32toh(specifier[0]);
	const u_int pin = be32toh(specifier[1]);
	const u_int type = be32toh(specifier[2]) & 0xf;

	switch (type) {
	case FDT_INTR_TYPE_POS_EDGE:
		mode = SUNXI_GPIO_INT_MODE_POS_EDGE;
		break;
	case FDT_INTR_TYPE_NEG_EDGE:
		mode = SUNXI_GPIO_INT_MODE_NEG_EDGE;
		break;
	case FDT_INTR_TYPE_DOUBLE_EDGE:
		mode = SUNXI_GPIO_INT_MODE_DOUBLE_EDGE;
		break;
	case FDT_INTR_TYPE_HIGH_LEVEL:
		mode = SUNXI_GPIO_INT_MODE_HIGH_LEVEL;
		break;
	case FDT_INTR_TYPE_LOW_LEVEL:
		mode = SUNXI_GPIO_INT_MODE_LOW_LEVEL;
		break;
	default:
		aprint_error_dev(dev, "%s: unsupported irq type 0x%x\n",
		    __func__, type);
		return NULL;
	}

	pin_def = jh7100_pinctrl_lookup(sc, port, pin);
	if (pin_def == NULL)
		return NULL;

	return jh7100_intr_enable(sc, pin_def, mode, mpsafe, func, arg);
}

static void
jh7100_fdt_intr_disestablish(device_t dev, void *ih)
{
	struct jh7100_pinctrl_softc * const sc = device_private(dev);
	struct jh7100_pinctrl_jgi * const jgi = ih;

	jh7100_intr_disable(sc, jgi);
}

static bool
jh7100_fdt_intrstr(device_t dev, u_int *specifier, char *buf, size_t buflen)
{
	struct jh7100_pinctrl_softc * const sc = device_private(dev);
	const struct jh7100_pinctrl_pins *pin_def;

	/* 1st cell is the bank */
	/* 2nd cell is the pin */
	/* 3rd cell is flags */
	if (!specifier)
		return false;
	const u_int port = be32toh(specifier[0]);
	const u_int pin = be32toh(specifier[1]);

	pin_def = jh7100_pinctrl_lookup(sc, port, pin);
	if (pin_def == NULL)
		return false;

	snprintf(buf, buflen, "GPIO %s", pin_def->name);

	return true;
}

static struct fdtbus_interrupt_controller_func jh7100_pinctrl_intrfuncs = {
	.establish = jh7100_fdt_intr_establish,
	.disestablish = jh7100_fdt_intr_disestablish,
	.intrstr = jh7100_fdt_intrstr,
};

static void *
jh7100_pinctrl_intr_establish(void *vsc, int pin, int ipl, int irqmode,
    int (*func)(void *), void *arg)
{
	struct jh7100_pinctrl_softc * const sc = vsc;
	bool mpsafe = (irqmode & GPIO_INTR_MPSAFE) != 0;
	int type = irqmode & GPIO_INTR_MODE_MASK;
	const struct jh7100_pinctrl_pins *pin_def;
	u_int mode;

	switch (type) {
	case GPIO_INTR_POS_EDGE:
		mode = SUNXI_GPIO_INT_MODE_POS_EDGE;
		break;
	case GPIO_INTR_NEG_EDGE:
		mode = SUNXI_GPIO_INT_MODE_NEG_EDGE;
		break;
	case GPIO_INTR_DOUBLE_EDGE:
		mode = SUNXI_GPIO_INT_MODE_DOUBLE_EDGE;
		break;
	case GPIO_INTR_HIGH_LEVEL:
		mode = SUNXI_GPIO_INT_MODE_HIGH_LEVEL;
		break;
	case GPIO_INTR_LOW_LEVEL:
		mode = SUNXI_GPIO_INT_MODE_LOW_LEVEL;
		break;
	default:
		aprint_error_dev(sc->sc_dev, "%s: unsupported irq type 0x%x\n",
				 __func__, type);
		return NULL;
	}

	if (pin < 0 || pin >= sc->sc_padconf->npins)
		return NULL;
	pin_def = &sc->sc_padconf->pins[pin];

	return jh7100_intr_enable(sc, pin_def, mode, mpsafe, func, arg);
}

static void
jh7100_pinctrl_intr_disestablish(void *vsc, void *ih)
{
	struct jh7100_pinctrl_softc * const sc = vsc;
	struct jh7100_pinctrl_jgi * const jgi = ih;

	jh7100_intr_disable(sc, jgi);
}

static bool
jh7100_pinctrl_intrstr(void *vsc, int pin, int irqmode, char *buf, size_t buflen)
{
	struct jh7100_pinctrl_softc * const sc = vsc;
	const struct jh7100_pinctrl_pins *pin_def;

	if (pin < 0 || pin >= sc->sc_padconf->npins)
		return NULL;
	pin_def = &sc->sc_padconf->pins[pin];

	snprintf(buf, buflen, "GPIO %s", pin_def->name);

	return true;
}













#endif

static int
jh7100_parse_slew_rate(int phandle)
{
	int slew_rate;

	if (of_getprop_uint32(phandle, "slew-rate", &slew_rate) == 0)
		return slew_rate;

	return -1;
}



static void
jh7100_pinctrl_pin_properties(struct jh7100_pinctrl_softc *sc, int phandle,
    uint16_t *val, uint16_t *mask)
{
	*mask = 0;
	*val = 0;

	const int bias = fdtbus_pinctrl_parse_bias(phandle, NULL);
	const int drive_strength = fdtbus_pinctrl_parse_drive_strength(phandle);
	const int slew_rate = jh7100_parse_slew_rate(phandle);

	switch (bias) {
	case 0:
		*mask |= PAD_BIAS_MASK;
		*val  |= PAD_BIAS_DISABLE;
		break;
	case GPIO_PIN_PULLUP:
		*mask |= PAD_BIAS_MASK;
		break;
	case GPIO_PIN_PULLDOWN:
		*mask |= PAD_BIAS_MASK;
		*val  |= PAD_BIAS_PULLDOWN;
		break;
	case -1:
	default:
		break;
	}

	switch (drive_strength) {
	case GPIO_DS_MIN ... GPIO_DS_MAX: {
		const u_int ds = (drive_strength - 14) / 7;
		*mask |=  PAD_DRIVE_STRENGTH_MASK;
		*val  |=  __SHIFTIN(ds, PAD_DRIVE_STRENGTH_MASK);
		break;
	    }
	case -1:
		break;
	default:
		aprint_error_dev(sc->sc_dev, "phandle %d invalid drive "
		"strength %d\n", phandle, drive_strength);
	}

	if (of_hasprop(phandle, "input-enable")) {
		*mask |=  PAD_INPUT_ENABLE;
		*val  |=  PAD_INPUT_ENABLE;
	}
	if (of_hasprop(phandle, "input-disable")) {
		*mask |=  PAD_INPUT_ENABLE;
		*val  &= ~PAD_INPUT_ENABLE;
	}
	if (of_hasprop(phandle, "input-schmitt-enable")) {
		*mask |=  PAD_INPUT_SCHMITT_ENABLE;
		*val  |=  PAD_INPUT_SCHMITT_ENABLE;
	}
	if (of_hasprop(phandle, "input-schmitt-disable")) {
		*mask |=  PAD_INPUT_SCHMITT_ENABLE;
		*val  &= ~PAD_INPUT_SCHMITT_ENABLE;
	}

	switch (slew_rate) {
	case 0 ... __SHIFTOUT_MASK(PAD_SLEW_RATE_MASK):
		*mask |=  PAD_SLEW_RATE_MASK;
		*val  |= __SHIFTIN(slew_rate, PAD_SLEW_RATE_MASK);
		break;
	case -1:
		break;
	default:
		aprint_error_dev(sc->sc_dev, "invalid slew rate\n");
	}

	if (of_hasprop(phandle, "starfive,strong-pull-up")) {
		*mask |=  PAD_BIAS_MASK;
		*val  |=  PAD_BIAS_STRONG_PULLUP;
	}
}



static void
jh7100_pinctrl_set_config_group(struct jh7100_pinctrl_softc *sc, int group)
{
	int pins_len, pinmux_len;
	const u_int *pins = fdtbus_get_prop(group, "pins", &pins_len);
	const u_int *pinmux = fdtbus_get_prop(group, "pinmux", &pinmux_len);
	size_t plen;
	const u_int *parray;

	aprint_debug_dev(sc->sc_dev, "set_config: group   %d\n", group);

	if (pins == NULL && pinmux == NULL) {
		aprint_debug_dev(sc->sc_dev, "group %d neither 'pins' nor "
		    "'pinmux' exist\n", group);
		return;
	} else if (pins != NULL && pinmux != NULL) {
		aprint_debug_dev(sc->sc_dev, "group %d both 'pins' and "
		    "'pinmux' exist\n", group);
		return;
	}

	if (pins != NULL) {
		KASSERT(pinmux == NULL);
		plen = pins_len;
		parray = pins;
	}
	if (pinmux != NULL) {
		KASSERT(pins == NULL);
		plen = pinmux_len;
		parray = pinmux;
	}
	const size_t npins = plen / sizeof(uint32_t);

	uint16_t val, mask;
	jh7100_pinctrl_pin_properties(sc, group, &val, &mask);

	aprint_debug_dev(sc->sc_dev, "set_config: group   %d, len %zu "
	    "value %#6x mask %#6x\n", group, plen, val, mask);

	for (size_t i = 0; i < npins; i++) {
		uint32_t p = be32dec(&parray[i]);
		u_int pin_no;

		if (pins != NULL) {
			pin_no = p;
			aprint_debug_dev(sc->sc_dev, "set_config: group   %d"
			    ", gpio %d doen %#x\n", group, pin_no,
			    GPIORD4(sc, GPO_DOEN_CFG(pin_no)));
			GPIOWR4(sc, GPO_DOEN_CFG(pin_no), GPO_DISABLE);
			jh7100_padctl_rmw(sc, pin_no,
			    val, mask);
		}
		if (pinmux != NULL) {
			pin_no = __SHIFTOUT(p, DT_GPIOMUX_GPIO_MASK);
			u_int dout = __SHIFTOUT(p, DT_GPIOMUX_DOUT_MASK);
			u_int doen = __SHIFTOUT(p, DT_GPIOMUX_DOEN_MASK);
			u_int din = __SHIFTOUT(p, DT_GPIOMUX_DIN_MASK);
			u_int doutrev = __SHIFTOUT(p, DT_GPIOMUX_DOUTREV_MASK);
			u_int doenrev = __SHIFTOUT(p, DT_GPIOMUX_DOENREV_MASK);

			uint32_t doutval =
			    __SHIFTIN(doutrev, GPO_DOUT_REVERSE) |
			    __SHIFTIN(dout, GPO_DOUT_MASK);
			uint32_t doenval =
			    __SHIFTIN(doenrev, GPO_DOEN_REVERSE) |
			    __SHIFTIN(doen, GPO_DOEN_MASK);

			aprint_debug_dev(sc->sc_dev, "set_config: group   %d"
			    ", gpio %d dout %#x/%#x doen %#x/%#x din %#x/%#x\n",
			    group, pin_no,
			    doutval, GPIORD4(sc, GPO_DOUT_CFG(pin_no)),
			    doenval, GPIORD4(sc, GPO_DOEN_CFG(pin_no)),
			    din != GPI_NONE ? pin_no + 2 : 0, GPIORD4(sc, GPI_DIN(din)));

			mutex_enter(&sc->sc_lock);
			GPIOWR4(sc, GPO_DOUT_CFG(pin_no), doutval);
			GPIOWR4(sc, GPO_DOEN_CFG(pin_no), doenval);
			if (din != GPI_NONE) {
				/*
				 *   0 is digital 0,
				 *   1 is digital 1,
				 *   2 is PAD_GPIO[0]
				 * ...
				 *  65 is PAD_GPIO[63]
				 */
				GPIOWR4(sc, GPI_DIN(din), pin_no + 2);
			}
			mutex_exit(&sc->sc_lock);
			jh7100_padctl_rmw(sc, sc->sc_padctl_gpio + pin_no,
			    val, mask);
		}
	}
}

static int
jh7100_pinctrl_set_config(device_t dev, const void *data, size_t len)
{
	struct jh7100_pinctrl_softc * const sc = device_private(dev);

	if (len != 4)
		return -1;

	const int phandle = fdtbus_get_phandle_from_native(be32dec(data));
	aprint_debug_dev(sc->sc_dev, "set_config: phandle %d\n", phandle);

	for (int child = OF_child(phandle); child; child = OF_peer(child)) {
		jh7100_pinctrl_set_config_group(sc, child);
	}

	return 0;
}

static struct fdtbus_pinctrl_controller_func jh7100_pinctrl_funcs = {
	.set_config = jh7100_pinctrl_set_config,
};





static void *
jh7100_pinctrl_gpio_acquire(device_t dev, const void *data, size_t len, int flags)
{
	struct jh7100_pinctrl_softc * const sc = device_private(dev);

printf("%s: data %p size %zu flags %x\n", __func__, data, len, flags);
	if (len != 12)
		return NULL;

	const u_int *gpio = data;
	const u_int pin_no = be32toh(gpio[1]);
	const bool actlo = be32toh(gpio[2]) & 1;

	if (pin_no >= GPIO_NPINS)
		return NULL;

	// XXXNH twiddle something??
	struct jh7100_pinctrl_gpio_pin *pin =
	    kmem_zalloc(sizeof(*pin), KM_SLEEP);
	pin->pin_sc = sc;
	pin->pin_no = pin_no;
	pin->pin_actlo = actlo;

	jh7100_gpio_pin_ctl(sc, pin->pin_no, /*pin->pin_ */ flags);

	return pin;
}

static void
jh7100_pinctrl_gpio_release(device_t dev, void *priv)
{
	struct jh7100_pinctrl_softc * const sc = device_private(dev);
	struct jh7100_pinctrl_gpio_pin *pin = priv;

	KASSERT(sc == pin->pin_sc);
	// XXXNH untwiddle something?
	kmem_free(pin, sizeof(*pin));
}

static int
jh7100_pinctrl_gpio_read(device_t dev, void *priv, bool raw)
{
	struct jh7100_pinctrl_softc * const sc = device_private(dev);
	struct jh7100_pinctrl_gpio_pin *pin = priv;
	const u_int pin_no = pin ->pin_no;
	const uint32_t bank = GPIORD4(sc, GPIO_DIN(pin_no));
	const uint32_t mask = pin_no % (sizeof(bank) * NBBY);

	int val = __SHIFTOUT(bank, mask);
	if (!raw && pin->pin_actlo)
		val = !val;

	return val;
}

static void
jh7100_pinctrl_gpio_write(device_t dev, void *priv, int val, bool raw)
{
	struct jh7100_pinctrl_softc * const sc = device_private(dev);
	struct jh7100_pinctrl_gpio_pin *pin = priv;
	const u_int pin_no = pin ->pin_no;

	if (!raw && pin->pin_actlo)
		val = !val;

	mutex_enter(&sc->sc_lock);
	GPIOWR4(sc, GPO_DOUT_CFG(pin_no), val);
	mutex_exit(&sc->sc_lock);
}

static struct fdtbus_gpio_controller_func jh7100_pinctrl_gpio_funcs = {
	.acquire = jh7100_pinctrl_gpio_acquire,
	.release = jh7100_pinctrl_gpio_release,
	.read = jh7100_pinctrl_gpio_read,
	.write = jh7100_pinctrl_gpio_write,
};


static int
jh7100_pinctrl_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
jh7100_pinctrl_attach(device_t parent, device_t self, void *aux)
{
	struct jh7100_pinctrl_softc *sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;

	sc->sc_dev = self;
	sc->sc_phandle = phandle;
	sc->sc_bst = faa->faa_bst;

	if (!of_hasprop(phandle, "gpio-controller")) {
		aprint_error(": no gpio controller\n");
		return;
	}

	if (fdtbus_get_reg_byname(phandle, "gpio", &addr, &size) != 0 ||
	    bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_gpio_bsh) != 0) {
		aprint_error(": couldn't map gpio registers\n");
		return;
	}
	if (fdtbus_get_reg_byname(phandle, "padctl", &addr, &size) != 0 ||
	    bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_padctl_bsh) != 0) {
		aprint_error(": couldn't map padctl registers\n");
		return;
	}

	// XXXNH Clocks
	// XXXNH interrupt-controller
	// XXXNH interrupts
#if 0
	char intrstr[128];
	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr))) {
		aprint_error_dev(self, "failed to decode interrupt\n");
		return;
	}
	sc->sc_ih = fdtbus_intr_establish_xname(phandle, 0, IPL_VM,
	    FDT_INTR_MPSAFE, jh7100_gpio_intr, sc, device_xname(self));
	if (sc->sc_ih == NULL) {
		aprint_error_dev(self, "failed to establish interrupt on %s\n",
		    intrstr);
		return;
	}
	aprint_normal_dev(self, "interrupting on %s\n", intrstr);
#endif



	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_VM);

	aprint_naive("\n");
	aprint_normal(": Pin Controller\n");

	u_int sel;
	int ret;
	ret = of_getprop_uint32(phandle, "starfive,signal-group",
	    &sel);
	if (ret < 0) {
		sel = PCTLRD4(sc, IO_PADSHARE_SEL);
	} else {
		PCTLWR4(sc, IO_PADSHARE_SEL, sel);
	}

	switch (sel) {
	case 0:
		// invalid gpio
		sc->sc_padctl_gpio = -1;
		break;
	case 1:
		sc->sc_padctl_gpio = PAD_GPIO(0);
		break;
	case 2:
		sc->sc_padctl_gpio = PAD_FUNC_SHARE(72);
		break;
	case 3:
		sc->sc_padctl_gpio = PAD_FUNC_SHARE(70);
		break;
	case 4 ... 6:
		sc->sc_padctl_gpio = PAD_FUNC_SHARE(0);
		break;
	default:
		aprint_error_dev(sc->sc_dev, "invalid signal group %u\n", sel);
		return;
	}

	aprint_verbose_dev(self, "selector %d\n", sel);

	fdtbus_register_gpio_controller(sc->sc_dev, sc->sc_phandle,
	    &jh7100_pinctrl_gpio_funcs);

	// XXXNH groups? new fdtbus_register_pinctrlgroup_config?
	for (int child = OF_child(phandle); child; child = OF_peer(child)) {
		fdtbus_register_pinctrl_config(self, child,
		    &jh7100_pinctrl_funcs);
	}
}

CFATTACH_DECL_NEW(jh7100_pinctrl, sizeof(struct jh7100_pinctrl_softc),
	jh7100_pinctrl_match, jh7100_pinctrl_attach, NULL, NULL);
