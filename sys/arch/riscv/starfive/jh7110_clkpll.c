/* $NetBSD$ */

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
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>

#include <sys/bus.h>
#include <sys/device.h>

#include <dev/clk/clk_backend.h>

#include <dev/fdt/fdtvar.h>
#include <dev/fdt/syscon.h>

/*
 * This driver is about to register JH7110 PLL clock generator and
 * support ops.
 *
 * The JH7110 have three PLL clock, PLL0, PLL1 and PLL2.
 * Each PLL clocks work in integer mode or fraction mode by some dividers,
 * and the configuration registers and dividers are set in several syscon
 * registers.
 *
 * The formula for calculating frequency is:
 * Fvco = Fref * (NI + NF) / M / Q1
 * Fref: OSC source clock rate
 *
 * NI: integer frequency dividing ratio of feedback divider,
 *     set by fbdiv[11:0].
 * NF: fractional frequency dividing ratio, set by frac[23:0].
 *     NF = frac[23:0] / 2^24 = 0 ~ 0.999.
 * M:  frequency dividing ratio of pre-divider, set by prediv[5:0].
 * Q1: frequency dividing ratio of post divider,
 *     set by 2^postdiv1[1:0], eg. 1, 2, 4 or 8.
 */

struct jh7110_clkpll;

struct jh7110_clkpll_softc {
	device_t		sc_dev;
	int			sc_syscon_phandle;
	const struct syscon *	sc_syscon;
#if 0
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
#endif
	int			sc_phandle;
	struct clk_domain	sc_clkdom;

	struct jh7110_clkpll *	sc_clk;
	size_t			sc_nclks;

};


/* Register definitions */
/* this driver expects a 24MHz input frequency from the oscillator */
#define KHZ 				1000UL
#define MHZ 				1000000UL
#define JH7110_PLL_OSC_RATE		(24 * MHZ)

#define JH7110_PLL0_PD_OFFSET		0x18
#define JH7110_PLL0_DACPD_MASK		__BIT(24)
#define JH7110_PLL0_DSMPD_MASK		__BIT(25)
#define JH7110_PLL0_FBDIV_OFFSET	0x1c
#define JH7110_PLL0_FBDIV_MASK		__BITS(11, 0)
#define JH7110_PLL0_FRAC_OFFSET		0x20
#define JH7110_PLL0_PREDIV_OFFSET	0x24

#define JH7110_PLL1_PD_OFFSET		0x24
#define JH7110_PLL1_DACPD_MASK		__BIT(15)
#define JH7110_PLL1_DSMPD_MASK		__BIT(16)
#define JH7110_PLL1_FBDIV_OFFSET	0x24
#define JH7110_PLL1_FBDIV_MASK		__BITS(28, 17)
#define JH7110_PLL1_FRAC_OFFSET		0x28
#define JH7110_PLL1_PREDIV_OFFSET	0x2c

#define JH7110_PLL2_PD_OFFSET		0x2c
#define JH7110_PLL2_DACPD_MASK		__BIT(15)
#define JH7110_PLL2_DSMPD_MASK		__BIT(16)
#define JH7110_PLL2_FBDIV_OFFSET	0x2c
#define JH7110_PLL2_FBDIV_MASK		__BITS(28, 17)
#define JH7110_PLL2_FRAC_OFFSET		0x30
#define JH7110_PLL2_PREDIV_OFFSET	0x34

#define JH7110_PLL_FRAC_MASK		__BITS(23, 0)
#define JH7110_PLL_POSTDIV1_MASK	__BITS(29, 28)
#define JH7110_PLL_PREDIV_MASK		__BITS(5, 0)

/* PLL clocks */
#define JH7110_PLLCLK_PLL0_OUT		0
#define JH7110_PLLCLK_PLL1_OUT		1
#define JH7110_PLLCLK_PLL2_OUT		2
#define JH7110_PLLCLK_NPLLS		3

static inline void
jh7110_syscon_update(struct jh7110_clkpll_softc *sc, bus_size_t off,
    uint32_t clr, uint32_t set)
{
	syscon_lock(sc->sc_syscon);
	const uint32_t old = syscon_read_4(sc->sc_syscon, off);
	const uint32_t new = (old & ~clr) | set;
	if (old != new) {
		syscon_write_4(sc->sc_syscon, off, new);
	}
	syscon_unlock(sc->sc_syscon);
}





struct jh7110_clkpll_preset {
	u_long		jcpp_frequency;
	uint32_t	jcpp_fbdivider;
	uint8_t		jcpp_prediv;
};


static const struct jh7110_clkpll_preset jh7110_pll0_presets[] = {
	{
		.jcpp_frequency = 375 * MHZ,
		.jcpp_fbdivider = 125,
		.jcpp_prediv = 8,
	}, {
		.jcpp_frequency = 500 * MHZ,
		.jcpp_fbdivider = 125,
		.jcpp_prediv = 6,
	}, {
		.jcpp_frequency = 625 * MHZ,
		.jcpp_fbdivider = 625,
		.jcpp_prediv = 24,
	}, {
		.jcpp_frequency = 750 * MHZ,
		.jcpp_fbdivider = 125,
		.jcpp_prediv = 4,
	}, {
		.jcpp_frequency = 875 * MHZ,
		.jcpp_fbdivider = 875,
		.jcpp_prediv = 24,
	}, {
		.jcpp_frequency = 1000 * MHZ,
		.jcpp_fbdivider = 125,
		.jcpp_prediv = 3,
	}, {
		.jcpp_frequency = 1250 * MHZ,
		.jcpp_fbdivider = 625,
		.jcpp_prediv = 12,
	}, {
		.jcpp_frequency = 1375 * MHZ,
		.jcpp_fbdivider = 1375,
		.jcpp_prediv = 24,
	}, {
		.jcpp_frequency = 1500 * MHZ,
		.jcpp_fbdivider = 125,
		.jcpp_prediv = 2,
	},
};

static const struct jh7110_clkpll_preset jh7110_pll1_presets[] = {
	{
		.jcpp_frequency = 1066 * MHZ,
		.jcpp_fbdivider = 533,
		.jcpp_prediv = 12,
	}, {
		.jcpp_frequency = 1200 * MHZ,
		.jcpp_fbdivider = 50,
		.jcpp_prediv = 1,
	}, {
		.jcpp_frequency = 1400 * MHZ,
		.jcpp_fbdivider = 350,
		.jcpp_prediv = 6,
	}, {
		.jcpp_frequency = 1600 * MHZ,
		.jcpp_fbdivider = 200,
		.jcpp_prediv = 3,
	},
};

static const struct jh7110_clkpll_preset jh7110_pll2_presets[] = {
	{
		.jcpp_frequency = 1188 * MHZ,
		.jcpp_fbdivider = 99,
		.jcpp_prediv = 2,
	}, {
		.jcpp_frequency = 1228800 * KHZ,
		.jcpp_fbdivider = 256,
		.jcpp_prediv = 5,
	},
};




struct jh7110_clkpll {
	struct clk				jcp_clk;
	const struct jh7110_clkpll_preset *	jcp_presets;
	size_t					jcp_npresets;

	// pd.{dacpd,dsmpd},
	// fbdiv.{fbdiv}
	// frac.{frac} ... always the same
	// prediv.{prediv} ... always the same
	struct {
		bus_size_t			jcpr_pd;
		bus_size_t			jcpr_fbdiv;
		bus_size_t			jcpr_frac;
		bus_size_t			jcpr_prediv;
	} jcp_regs;
	struct {
		uint32_t			jcpm_dacpd;
		uint32_t			jcpm_dsmpd;
		uint32_t			jcpm_fbdiv;
	} jcp_masks;
};

#define _JH7110_CLKPLL(_id, _name, _presets)				       \
	[_id] = {							       \
		.jcp_clk = {						       \
			.name = (_name),				       \
		},							       \
		.jcp_presets = (_presets),				       \
		.jcp_npresets = __arraycount(_presets),			       \
		.jcp_regs = {						       \
			.jcpr_pd = JH7110_PLL##_id##_PD_OFFSET,		       \
			.jcpr_fbdiv = JH7110_PLL##_id##_FBDIV_OFFSET,	       \
			.jcpr_frac = JH7110_PLL##_id##_FRAC_OFFSET,	       \
			.jcpr_prediv = JH7110_PLL##_id##_PREDIV_OFFSET,	       \
		},							       \
		.jcp_masks = {						       \
			.jcpm_dacpd = JH7110_PLL##_id##_DACPD_MASK,	       \
			.jcpm_dsmpd = JH7110_PLL##_id##_DSMPD_MASK,	       \
			.jcpm_fbdiv = JH7110_PLL##_id##_FBDIV_MASK,	       \
		},							       \
	}

#define JH7110_CLKPLL(_id, _name, _presets)				       \
	   _JH7110_CLKPLL(_id, _name, _presets)


#if 0
static int
jh7100_usb_init(struct jh7110_clkpll_softc *sc, const u_int *syscon_data )
{
	enum usb_dr_mode mode = USB_DR_MODE_HOST;

	switch (mode) {
	case USB_DR_MODE_HOST:
		jh7110_syscon_update(sc, JH7100_USB0,
		    JH7100_USB0_MODE_STRAP_MASK, JH7100_USB0_MODE_STRAP_HOST);
		jh7110_syscon_update(sc, JH7100_USB7,
		    JH7100_USB7_PLL_
struct jh7110_pll_preset {
	unsigned long freq;
	u32 frac;		/* frac value should be decimals multiplied by 2^24 */
	unsigned fbdiv    : 12;	/* fbdiv value should be 8 to 4095 */
	unsigned prediv   :  6;
	unsigned postdiv1 :  2;
	unsigned mode     :  1;
};


EN, JH7100_USB7_PLL_EN);
		jh7110_syscon_update(sc, JH7100_USB7,
		    JH7100_USB7_EQ_EN, JH7100_USB7_EQ_EN);
		jh7110_syscon_update(sc, JH7100_USB7,
		    JH7100_USB7_SSRX_SEL, JH7100_USB7_SSRX_SEL);
		jh7110_syscon_update(sc, JH7100_USB7,
		    JH7100_USB7_SSTX_SEL, JH7100_USB7_SSTX_SEL);
		jh7110_syscon_update(sc, JH7100_USB3,
		    JH7100_USB3_UTMI_IDDIG, JH7100_USB3_UTMI_IDDIG);
		break;
	default:
		break;
	}

	return 0;
}

static int
jh7110_clkpll_init(struct jh7110_clkpll_softc *sc, const u_int *syscon_data)
{
	enum usb_dr_mode mode = USB_DR_MODE_HOST;
	bus_size_t usb_mode = be32dec(&syscon_data[1]);

	jh7110_syscon_update(sc, usb_mode, JH7110_MISC_CFG_MASK,
	    JH7110_SUSPENDM_BYPS | JH7110_PLL_EN | JH7110_REFCLK_MODE);

	switch (mode) {
	case USB_DR_MODE_HOST:
		jh7110_syscon_update(sc, usb_mode, JH7110_STRAP_MASK, JH7110_STRAP_HOST);
		jh7110_syscon_update(sc, usb_mode, JH7110_SUSPENDM_MASK, JH7110_SUSPENDM_HOST);
		break;

	case USB_DR_MODE_PERIPHERAL:
		jh7110_syscon_update(sc, usb_mode, JH7110_STRAP_MASK, JH7110_STRAP_DEVICE);
		jh7110_syscon_update(sc, usb_mode, JH7110_SUSPENDM_MASK, 0);
		break;
	default:
		break;
	}

	return 0;
}
#endif

#if 0
struct jh7110_clkpll_config {
	int (*jhuc_init)(struct jh7110_clkpll_softc *, const u_int *);
	const char *jhuc_syscon;
	size_t jhuc_sclen;
};

struct jh7110_clkpll_config jh7100_usb_data = {
	.jhuc_init = jh7100_usb_init,
	.jhuc_syscon = "starfive,syscon",
	.jhuc_sclen = 1 * sizeof(uint32_t),
};

struct jh7110_clkpll_config jh7110_clkpll_data = {
	.jhuc_init = jh7110_clkpll_init,
	.jhuc_syscon = "starfive,stg-syscon",
	.jhuc_sclen = 2 * sizeof(uint32_t),
};
#endif

#if 0
static void
jh7110_clkpll_update(struct jh7110_clkpll_softc * const sc,
    struct jh7110_clkpll *jcp, uint32_t set, uint32_t clr)
{
	// lock
	uint32_t val = RD4(sc, jcc->jcc_reg);
	uint32_t before = val;
	val &= ~clr;
	val |=  set;
	WR4(sc, jcc->jcc_reg, val);
	aprint_verbose("%s: %04" PRIxBUSADDR " = %#10x(was %#10x) called from %p\n",
	    __func__, jcc->jcc_reg, val, before, __builtin_return_address(0));
}

/*
 * FIXED_FACTOR operations
 */

static u_int
jh7110_clkpll_fixed_factor_get_parent_rate(struct clk *clk)
{
	struct clk *clk_parent = clk_get_parent(clk);
	if (clk_parent == NULL)
		return 0;
aprint_verbose("%s: %u (rate)\n", __func__, clk_get_rate(clk_parent));

	return clk_get_rate(clk_parent);
}

u_int
jh7110_clkpll_fixed_factor_get_rate(struct jh7110_clkpll_softc *sc,
    struct jh7110_clkpll *jcp)
{
	KASSERT(jcc->jcc_type == JH71X0CLK_FIXED_FACTOR);

	struct jh7110_clkpll_fixed_factor * const jcff = &jcc->jcc_ffactor;
	struct clk *clk = &jcc->jcp_clk;

	uint64_t rate = jh7110_clkpll_fixed_factor_get_parent_rate(clk);
	if (rate == 0)
		return 0;

	uint64_t tmp = rate;

	rate *= jcff->jcff_mult;
	rate /= jcff->jcff_div;

aprint_verbose("%s: %u (rate) (%"PRIu64") (%"PRIu64" * %u / %u)\n", __func__,
    (u_int)rate, rate, tmp, jcff->jcff_mult, jcff->jcff_div);

	return rate;
}

static int
jh7110_clkpll_fixed_factor_set_parent_rate(struct clk *clk, u_int rate)
{
	struct clk *clk_parent = clk_get_parent(clk);
	if (clk_parent == NULL)
		return ENXIO;

aprint_verbose("%s: %u (set parent)\n", __func__, rate);

	return clk_set_rate(clk_parent, rate);
}

int
jh7110_clkpll_fixed_factor_set_rate(struct jh7110_clkpll_softc *sc,
    struct jh7110_clkpll *jcp, u_int rate)
{
	KASSERT(jcc->jcc_type == JH71X0CLK_FIXED_FACTOR);

	struct jh7110_clkpll_fixed_factor * const jcff = &jcc->jcc_ffactor;
	struct clk *clk = &jcc->jcp_clk;


	uint64_t tmp = rate;
	tmp *= jcff->jcff_div;
	tmp /= jcff->jcff_mult;

aprint_verbose("%s: %u (set parent)\n", __func__, (u_int)tmp);

	return jh7110_clkpll_fixed_factor_set_parent_rate(clk, tmp);
}

const char *
jh7110_clkpll_fixed_factor_get_parent(struct jh7110_clkpll_softc *sc,
    struct jh7110_clkpll *jcp)
{
	KASSERT(jcc->jcc_type == JH71X0CLK_FIXED_FACTOR);

	struct jh7110_clkpll_fixed_factor * const jcff = &jcc->jcc_ffactor;

aprint_verbose("%s: '%s' has parent '%s'\n", __func__, jcc->jcp_clk.name, jcff->jcff_parent);

	return jcff->jcff_parent;
}


/*
 * MUX operations
 */

int
jh7110_clkpll_mux_set_parent(struct jh7110_clkpll_softc *sc,
    struct jh7110_clkpll *jcp, const char *name)
{
	KASSERT(jcc->jcc_type == JH71X0CLK_MUX);

	struct jh7110_clkpll_mux * const jcm = &jcc->jcc_mux;

	aprint_verbose("%s: '%s' setting parent to'%s'\n", __func__, jcc->jcp_clk.name, name);

	size_t i;
	for (i = 0; i < jcm->jcm_nparents; i++) {
		if (jcm->jcm_parents[i] != NULL &&
		    strcmp(jcm->jcm_parents[i], name) == 0)
			break;
	}
	if (i >= jcm->jcm_nparents)
		return EINVAL;

	KASSERT(i <= __SHIFTOUT_MASK(JH71X0_CLK_MUX_MASK));

	uint32_t val = RD4(sc, jcc->jcc_reg);
	uint32_t before = val;
	val &= ~JH71X0_CLK_MUX_MASK;
	val |= __SHIFTIN(i, JH71X0_CLK_MUX_MASK);
	aprint_verbose("%s: %04" PRIxBUSADDR    " = %08x(%08x)\n", __func__,
		jcc->jcc_reg, val, before);
	WR4(sc, jcc->jcc_reg, val);

	return 0;
}


const char *
jh7110_clkpll_mux_get_parent(struct jh7110_clkpll_softc *sc,
    struct jh7110_clkpll *jcp)
{
	KASSERT(jcc->jcc_type == JH71X0CLK_MUX);

	uint32_t val = RD4(sc, jcc->jcc_reg);
	size_t pindex = __SHIFTOUT(val, JH71X0_CLK_MUX_MASK);

	if (pindex >= jcc->jcc_mux.jcm_nparents)
		return NULL;

aprint_verbose("%s: '%s' has parent '%s'\n", __func__, jcc->jcp_clk.name, jcc->jcc_mux.jcm_parents[pindex]);

	return jcc->jcc_mux.jcm_parents[pindex];
}


/*
 * GATE operations
 */

int
jh7110_clkpll_gate_enable(struct jh7110_clkpll_softc *sc,
    struct jh7110_clkpll *jcp, int enable)
{
	KASSERT(jcc->jcc_type == JH71X0CLK_GATE);

	jh7110_clkpll_update(sc, jcc,
	    (enable ? JH71X0_CLK_ENABLE : 0), JH71X0_CLK_ENABLE);

aprint_verbose("%s: '%s' has been %s\n", __func__, jcc->jcp_clk.name, enable ? "enabled" : "disabled");

	return 0;
}

const char *
jh7110_clkpll_gate_get_parent(struct jh7110_clkpll_softc *sc,
    struct jh7110_clkpll *jcp)
{
	KASSERT(jcc->jcc_type == JH71X0CLK_GATE);

	struct jh7110_clkpll_gate *jcc_gate = &jcc->jcc_gate;

aprint_verbose("%s: '%s' has parent '%s'\n", __func__, jcc->jcp_clk.name, jcc_gate->jcg_parent);
	return jcc_gate->jcg_parent;
}


/*
 * DIVIDER operations
 */

u_int
jh7110_clkpll_div_get_rate(struct jh7110_clkpll_softc *sc,
    struct jh7110_clkpll *jcp)
{
	KASSERT(jcc->jcc_type == JH71X0CLK_DIV);

	struct clk * const clk = &jcc->jcp_clk;
	struct clk * const clk_parent = clk_get_parent(clk);

	if (clk_parent == NULL)
		return 0;

aprint_verbose("\n%s: getting parent '%s' rate of '%s' = ...\n", __func__, clk_parent->name, clk->name);
	u_int rate = clk_get_rate(clk_parent);
  aprint_verbose("%s: got     parent '%s' rate of '%s' = %u\n", __func__, clk_parent->name, clk->name, rate);
	if (rate == 0)
		return 0;

	uint32_t val = RD4(sc, jcc->jcc_reg);
	uint32_t div = __SHIFTOUT(val, JH71X0_CLK_DIV_MASK);

aprint_verbose("%s: '%s' %u (rate) val %#08x div %d\n", __func__, clk->name,
    div != 0 ? rate / div : 0, val, div);

	return rate / div;
}
#endif

#if 0
int
jh7110_clkpll_div_set_rate(struct jh7110_clkpll_softc *sc,
    struct jh7110_clkpll *jcp, u_int new_rate)
{
	KASSERT(jcc->jcc_type == JH71X0CLK_DIV);

	struct jh7110_clkpll_div * const jcc_div = &jcc->jcc_div;
	struct clk * const clk = &jcc->jcp_clk;
	struct clk * const clk_parent = clk_get_parent(clk);

aprint_verbose("%s: clk '%s' new_rate %u\n", __func__, clk->name,
    new_rate);

	if (clk_parent == NULL)
		return ENXIO;

	if (jcc_div->jcd_maxdiv == 0)
		return ENXIO;

aprint_verbose("%s: getting parent (%s) rate of %s\n", __func__, clk_parent->name, clk->name);
	u_int parent_rate = clk_get_rate(clk_parent);
	if (parent_rate == 0) {
		return (new_rate == 0) ? 0 : ERANGE;
	}
aprint_verbose("%s: got     parent (%p) rate of %p\n", __func__, clk_parent, clk);
	u_int ratio = howmany(parent_rate, new_rate);
	u_int div = uimin(ratio, jcc_div->jcd_maxdiv);

	KASSERT(div <= __SHIFTOUT_MASK(JH71X0_CLK_DIV_MASK));

aprint_verbose("%s: clk '%s' new_rate %u %u/%u (ratio of %u:%u)\n",
    __func__, clk->name, new_rate, div, ratio, parent_rate, new_rate);

	jh7110_clkpll_update(sc, jcc,
	    __SHIFTIN(div, JH71X0_CLK_DIV_MASK), JH71X0_CLK_DIV_MASK);

	return 0;
}

const char *
jh7110_clkpll_div_get_parent(struct jh7110_clkpll_softc *sc,
    struct jh7110_clkpll *jcp)
{
	KASSERT(jcc->jcc_type == JH71X0CLK_DIV);

	struct jh7110_clkpll_div *jcc_div = &jcc->jcc_div;

	return jcc_div->jcd_parent;
}
#endif



static struct clk *
jh7110_clkpll_get(void *priv, const char *name)
{
	struct jh7110_clkpll_softc * const sc = priv;

	for (u_int id = 0; id < sc->sc_nclks; id++) {
		struct jh7110_clkpll * const jcp = &sc->sc_clk[id];

		if (strcmp(name, jcp->jcp_clk.name) == 0) {
			return &jcp->jcp_clk;
		}
	}

	return NULL;
}

static void
jh7110_clkpll_put(void *priv, struct clk *clk)
{
}

static int
jh7110_clkpll_set_rate(void *priv, struct clk *clk, u_int rate)
{
//	struct jh7110_clkpll_softc * const sc = priv;
	struct jh7110_clkpll * const jcp =
	    container_of(clk, struct jh7110_clkpll, jcp_clk);

	struct clk *clk_parent = clk_get_parent(clk);
	if (clk_parent == NULL) {
		aprint_debug("%s: no parent for %s\n", __func__,
		    jcp->jcp_clk.name);
		return ENXIO;
	}

	if (clk_get_rate(clk_parent) != JH7110_PLL_OSC_RATE) {
		return EINVAL;
	}
printf("%s: trying to set rate %u KHz for %s", __func__, rate / 1000, clk->name);
	bool found = false;
	for (size_t idx = 0; idx < jcp->jcp_npresets; idx++) {
		if (jcp->jcp_presets[idx].jcpp_frequency == rate) {
			found = true;
			break;
		}
	}

	if (!found)
		return ENXIO;


	// XXXNH

	return 0;
}

static u_int
jh7110_clkpll_get_rate(void *priv, struct clk *clk)
{
	struct jh7110_clkpll_softc * const sc = priv;
	struct jh7110_clkpll * const jcp =
	    container_of(clk, struct jh7110_clkpll, jcp_clk);

	syscon_lock(sc->sc_syscon);

	uint32_t pd_reg = syscon_read_4(sc->sc_syscon, jcp->jcp_regs.jcpr_pd);
	uint32_t fbdiv_reg = syscon_read_4(sc->sc_syscon, jcp->jcp_regs.jcpr_fbdiv);
	uint32_t frac_reg = syscon_read_4(sc->sc_syscon, jcp->jcp_regs.jcpr_frac);
	uint32_t prediv_reg = syscon_read_4(sc->sc_syscon, jcp->jcp_regs.jcpr_prediv);

aprint_verbose("%s: '%s' reg: pd %#10x fbdiv %#10x frac %#10x prediv %#10x\n",
    __func__, clk->name, pd_reg, fbdiv_reg, frac_reg, prediv_reg);

#if 0
	KASSERT(__SHIFTOUT(pd_reg, jcp->jcp_masks.jcpm_dacpd) == 1);
	KASSERT(__SHIFTOUT(pd_reg, jcp->jcp_masks.jcpm_dsmpd) == 1);
#endif
	uint8_t dacpd = __SHIFTOUT(pd_reg, jcp->jcp_masks.jcpm_dacpd);
	uint8_t dsmpd = __SHIFTOUT(pd_reg, jcp->jcp_masks.jcpm_dsmpd);
	uint32_t fbdiv = __SHIFTOUT(fbdiv_reg, jcp->jcp_masks.jcpm_fbdiv);
	uint32_t frac = __SHIFTOUT(frac_reg, JH7110_PLL_FRAC_MASK);
	uint32_t postdiv = __SHIFTOUT(frac_reg, JH7110_PLL_POSTDIV1_MASK);
	uint32_t prediv = __SHIFTOUT(prediv_reg, JH7110_PLL_PREDIV_MASK);

	syscon_unlock(sc->sc_syscon);

	u_long parent_rate = JH7110_PLL_OSC_RATE;
	u_int rate;

	/*
	 * dacpd = dsmpd = 0: fraction mode
	 * dacpd = dsmpd = 1: integer mode, frac value ignored
	 *
	 * rate = parent * (fbdiv + frac/2^24) / prediv / 2^postdiv1
	 *      = (parent * fbdiv + parent * frac / 2^24) / (prediv * 2^postdiv1)
	 */
	if (dacpd == 0 && dsmpd == 0)
		rate = parent_rate * frac / (1UL << 24);
	else if (dacpd == 1 && dsmpd == 1)
		rate = 0;
	else
		return 0;
	rate += parent_rate * fbdiv;
	rate /= prediv << postdiv;

aprint_verbose("%s: get rate for '%s' = %u (%u/%u/%u)\n",
    __func__, clk->name, rate, fbdiv, prediv, postdiv);

	return rate;
}

#if 0
static int
jh7110_clkpll_enable(void *priv, struct clk *clk)
{
	struct jh7110_clkpll_softc * const sc = priv;
	struct jh7110_clkpll * const jcp =
	    container_of(clk, struct jh7110_clkpll, jcp_clk);

aprint_verbose("%s: enabling '%s' (%p/%p)\n", __func__, clk->name, clk, jcc);

	struct clk * const clk_parent = clk_get_parent(clk);
	if (clk_parent != NULL) {
		int error = clk_enable(clk_parent);
		if (error != 0)
			return error;
	}

	switch (jcc->jcc_type) {
	case JH71X0CLK_GATE:
		jh7110_clkpll_update(sc, jcc, JH71X0_CLK_ENABLE, 0);
		break;

	case JH71X0CLK_DIV: {
		struct jh7110_clkpll_div * const jcc_div = &jcc->jcc_div;
		if (jcc_div->jcd_flags & JH71X0CLKC_DIV_GATE) {
			jh7110_clkpll_update(sc, jcc, JH71X0_CLK_ENABLE, 0);
		}
		break;
	    }

	case JH71X0CLK_MUX: {
		struct jh7110_clkpll_mux * const jcc_mux = &jcc->jcc_mux;
		if (jcc_mux->jcm_flags & JH71X0CLKC_MUX_GATE) {
			jh7110_clkpll_update(sc, jcc, JH71X0_CLK_ENABLE, 0);
		}
		break;
	    }

	case JH71X0CLK_FIXED_FACTOR:
	case JH71X0CLK_INV:
	case JH71X0CLK_MUXDIV:
		break;

	default:
		aprint_verbose("%s: type %d\n", __func__, jcc->jcc_type);
		return ENXIO;
	}
	return 0;
}

static int
jh7110_clkpll_disable(void *priv, struct clk *clk)
{
	struct jh7110_clkpll_softc * const sc = priv;
	struct jh7110_clkpll * const jcp =
	    container_of(clk, struct jh7110_clkpll, jcp_clk);

	switch (jcc->jcc_type) {
	case JH71X0CLK_GATE:
		jh7110_clkpll_update(sc, jcc, 0, JH71X0_CLK_ENABLE);
		break;

	case JH71X0CLK_DIV: {
		struct jh7110_clkpll_div * const jcc_div = &jcc->jcc_div;
		if (jcc_div->jcd_flags & JH71X0CLKC_DIV_GATE) {
			jh7110_clkpll_update(sc, jcc, 0, JH71X0_CLK_ENABLE);
		}
		break;
	    }

	case JH71X0CLK_MUX: {
		struct jh7110_clkpll_mux * const jcc_mux = &jcc->jcc_mux;
		if (jcc_mux->jcm_flags & JH71X0CLKC_MUX_GATE) {
			jh7110_clkpll_update(sc, jcc, 0, JH71X0_CLK_ENABLE);
		}
		break;
	    }

	case JH71X0CLK_FIXED_FACTOR:
	case JH71X0CLK_INV:
	case JH71X0CLK_MUXDIV:
		break;

	default:
		return ENXIO;
	}
	return 0;
}

#endif

#if 0
static struct jh7110_clkpll *
jh7110_clkpll_clock_find(struct jh7110_clkpll_softc *sc, const char *name)
{
	for (size_t id = 0; id < sc->sc_nclks; id++) {
		struct jh7110_clkpll * const jcp = &sc->sc_clk[id];

		if (jcp->jcp_clk.name == NULL)
			continue;
		if (strcmp(jcp->jcp_clk.name, name) == 0)
			return jcp;
	}

	return NULL;
}
#endif


#if 0

static int
jh7110_clkpll_set_parent(void *priv, struct clk *clk,
    struct clk *clk_parent)
{
	struct jh7110_clkpll_softc * const sc = priv;
	struct jh7110_clkpll * const jcp =
	    container_of(clk, struct jh7110_clkpll, jcp_clk);

aprint_verbose("%s: '%s' (%p/%p)\n", __func__, clk->name, clk, jcp);

	if (jcc->jcc_ops->jcco_setparent == NULL)
		return EINVAL;

	return jcc->jcc_ops->jcco_setparent(sc, jcc, clk_parent->name);
}
#endif

#if 0

static struct clk *
jh7110_clkpll_get_parent(void *priv, struct clk *clk)
{
	struct jh7110_clkpll_softc * const sc = priv;
	struct jh7110_clkpll * const jcp =
	    container_of(clk, struct jh7110_clkpll, jcp_clk);

	if (jcc->jcc_ops->jcco_getparent == NULL)
		return NULL;

	const char *parent = jcc->jcc_ops->jcco_getparent(sc, jcc);
	if (parent == NULL)
		return NULL;

	struct jh7110_clkpll *jcp_parent = jh7110_clkpll_clock_find(sc, parent);
	if (jcc_parent != NULL)
		return &jcc_parent->jcp_clk;

	/* No parent in this domain, try FDT */
	return fdtbus_clock_get(sc->sc_phandle, parent);
}
#endif


#if 1
static struct jh7110_clkpll jh7110_clkplls[JH7110_PLLCLK_NPLLS] = {
	JH7110_CLKPLL(JH7110_PLLCLK_PLL0_OUT, "pll0_out", jh7110_pll0_presets),
	JH7110_CLKPLL(JH7110_PLLCLK_PLL1_OUT, "pll1_out", jh7110_pll1_presets),
	JH7110_CLKPLL(JH7110_PLLCLK_PLL2_OUT, "pll2_out", jh7110_pll2_presets),
};

#else
struct jh7110_clkpll jh7110_clkplls[JH7110_PLLCLK_NPLLS] = {
	JH7110_CLKPLL(0, "pll0_out", jh7110_pll0_presets),
	JH7110_CLKPLL(1, "pll1_out", jh7110_pll1_presets),
	JH7110_CLKPLL(2, "pll2_out", jh7110_pll2_presets),
};
#endif



const struct clk_funcs jh7110_clkpll_funcs = {
	.get = jh7110_clkpll_get,
	.put = jh7110_clkpll_put,
	.set_rate = jh7110_clkpll_set_rate,
	.get_rate = jh7110_clkpll_get_rate,
#if 0
	.enable = jh7110_clkpll_enable,
	.disable = jh7110_clkpll_disable,
	.set_parent = jh7110_clkpll_set_parent,
	.get_parent = jh7110_clkpll_get_parent,
#endif

};







static struct clk *
jh7110_pllclk_clock_decode(device_t dev, int phandle, const void *data,
    size_t len)
{
	struct jh7110_clkpll_softc * const sc = device_private(dev);

	if (len != 4) {
		return NULL;
	}

	u_int id = be32dec(data);
	if (id >= sc->sc_nclks) {
		return NULL;
	}

	return &sc->sc_clk[id].jcp_clk;
}

static const struct fdtbus_clock_controller_func jh7110_pllclk_fdtclock_funcs = {
	.decode = jh7110_pllclk_clock_decode
};


/* Compat string(s) */
static const struct device_compatible_entry compat_data[] = {
	{ .compat = "starfive,jh7110-pll" },
	DEVICE_COMPAT_EOL
};

static int
jh7110_clkpll_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
jh7110_clkpll_attach(device_t parent, device_t self, void *aux)
{
	struct jh7110_clkpll_softc *sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;

	sc->sc_dev = self;
	sc->sc_phandle = phandle;
#if 0
	sc->sc_bst = faa->faa_bst;
#endif
	sc->sc_syscon_phandle = OF_parent(sc->sc_phandle);
	sc->sc_syscon = fdtbus_syscon_lookup(sc->sc_syscon_phandle);
	if (sc->sc_syscon == NULL) {
		aprint_error(": couldn't get syscon registers\n");
		return;
	}

	sc->sc_clkdom.name = device_xname(self);
	sc->sc_clkdom.funcs = &jh7110_clkpll_funcs;
	sc->sc_clkdom.priv = sc;

	sc->sc_clk = jh7110_clkplls;
	sc->sc_nclks = __arraycount(jh7110_clkplls);

	for (size_t id = 0; id < sc->sc_nclks; id++) {
		sc->sc_clk[id].jcp_clk.domain = &sc->sc_clkdom;
		// Names already populated.
		clk_attach(&sc->sc_clk[id].jcp_clk);
	}

	aprint_naive("\n");
	aprint_normal(": JH7110 PLL clock controller\n");

#if 0
	for (size_t id = 0; id < sc->sc_nclks; id++) {
		struct clk * const clk = &sc->sc_clk[id].jcp_clk;

		aprint_debug_dev(self, "id %zu [%s]: %u Hz\n", id,
		    clk->name ? clk->name : "<none>", clk_get_rate(clk));
	}

	const struct jh7110_clkpll_config *jhuc =
	    of_compatible_lookup(sc->sc_phandle, compat_data)->data;

	int len;
	const u_int *syscon_data =
	    fdtbus_get_prop(phandle, jhuc->jhuc_syscon, &len);
	if (syscon_data == NULL) {
		aprint_error(": couldn't get '%s' property\n",
		    jhuc->jhuc_syscon);
		return;
	}
	if (len != jhuc->jhuc_sclen) {
		aprint_error(": incorrect syscon data (len = %u)\n",
		    len);
		return;
	}

	int syscon_phandle =
	    fdtbus_get_phandle_from_native(be32dec(&syscon_data[0]));

	sc->sc_syscon = fdtbus_syscon_lookup(syscon_phandle);
	if (sc->sc_syscon == NULL) {
		aprint_error(": couldn't get syscon\n");
		return;
	}

#endif
	fdtbus_register_clock_controller(self, phandle,
	    &jh7110_pllclk_fdtclock_funcs);
}


CFATTACH_DECL_NEW(jh7110_clkpll, sizeof(struct jh7110_clkpll_softc),
	jh7110_clkpll_match, jh7110_clkpll_attach, NULL, NULL);
