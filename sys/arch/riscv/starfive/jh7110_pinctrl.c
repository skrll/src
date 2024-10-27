/* $NetBSD: jh7110_pinctrl.c,v 1.1 2024/11/11 19:23:18 skrll Exp $ */

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
__KERNEL_RCSID(0, "$NetBSD: jh7110_pinctrl.c,v 1.1 2024/11/11 19:23:18 skrll Exp $");

#include <sys/param.h>

#include <sys/kmem.h>

#include <dev/fdt/fdtvar.h>

struct jh7110_pinctrl_softc;
struct jh7110_pinctrl_data {
//	const struct pinctrl_pin_desc *pins;
	u_int		jpd_npins;
	u_int		jpd_ngpios;
//	u_int gcsc_base;

	bus_size_t	jpd_dout;
	uint32_t	jpd_dout_mask;
	bus_size_t	jpd_doen;
	uint32_t	jpd_doen_mask;
	bus_size_t	jpd_gpi;
	uint32_t	jpd_gpi_mask;
	bus_size_t	jpd_gin;

	bus_size_t	jpd_gpioin;

//	const struct jh7110_gpio_irq_reg *irq_reg;

//	u_int nsaved_regs;

#if 0
	/* generic pinmux */
	/* gpio chip */
	int (*jh7110_get_padcfgsc_base)(struct jh7110_pinctrl *sfp,
				      u_int pin);
	void (*jh7110_gpio_irq_handler)(struct irq_desc *desc);
	int (*jh7110_gpio_init_hw)(struct gpio_chip *gc);
#endif
};

struct jh7110_pinctrl_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	int			sc_phandle;

	kmutex_t		sc_lock;

	const struct jh7110_pinctrl_data *
				sc_jpd;
};

struct jh7110_pinctrl_gpio_pin {
	struct jh7110_pinctrl_softc	*pin_sc;
	u_int				 pin_no;
	bool				 pin_actlo;
};


// https://doc-en.rvspace.org/JH7110/TRM/JH7110_TRM/sys_iomux_cfg.html

/* SYS registers */
#define JH7110_SYS_DOEN			0x0000
#define JH7110_SYS_DOUT			0x0040
#define JH7110_SYS_GPI			0x0080
#define JH7110_SYS_GPIOIN		0x0118

#define JH7110_SYS_NGPIO		64
#define JH7110_SYS_NPIN			96

//#define JH7110_SYS_GC_BASE		0

/* AON registers */
#define JH7110_AON_DOEN			0x0000
#define JH7110_AON_DOUT			0x0004
#define JH7110_AON_GPI			0x0008
#define JH7110_AON_GPIOIN		0x002c

#define JH7110_AON_NGPIO		4
#define JH7110_AON_NPIN			20

//#define JH7110_AON_GC_BASE		64		// XXXNH not needed atm
//#define JH7110_AON_REGS_NUM		37		// XXXNH not needed atm

// XXXNH rename
#define GPOUT_LOW			0
#define GPOUT_HIGH			1

#define  GPI_NONE			0xff


#define RD4(sc, reg)						       \
	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define WR4(sc, reg, val)					       \
	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

/* pad control bits */
#define JH7110_PADCFG_IE	__BIT(0)
#define JH7110_PADCFG_DS_MASK	__BITS(1, 2)
#define JH7110_PADCFG_DS_2MA	__SHIFTIN(0, JH7110_PADCFG_DS_MASK)
#define JH7110_PADCFG_DS_4MA	__SHIFTIN(1, JH7110_PADCFG_DS_MASK)
#define JH7110_PADCFG_DS_8MA	__SHIFTIN(2, JH7110_PADCFG_DS_MASK)
#define JH7110_PADCFG_DS_12MA	__SHIFTIN(3, JH7110_PADCFG_DS_MASK)
#define JH7110_PADCFG_PU	__BIT(3)
#define JH7110_PADCFG_PD	__BIT(4)
#define JH7110_PADCFG_BIAS_MASK	(JH7110_PADCFG_PD | JH7110_PADCFG_PU)
#define JH7110_PADCFG_SLEW	__BIT(5)
#define JH7110_PADCFG_SMT	__BIT(6)
#define JH7110_PADCFG_POS	__BIT(7)

#if 0
struct jh7110_gpio_irq_reg {
	u_int is_reg_base;
	u_int ic_reg_base;
	u_int ibe_reg_base;
	u_int iev_reg_base;
	u_int ie_reg_base;
	u_int ris_reg_base;
	u_int mis_reg_base;
};
#endif

/* SYS pins */
#define JH7110_SYS_PAD_GPIO0               0
#define JH7110_SYS_PAD_GPIO1               1
#define JH7110_SYS_PAD_GPIO2               2
#define JH7110_SYS_PAD_GPIO3               3
#define JH7110_SYS_PAD_GPIO4               4
#define JH7110_SYS_PAD_GPIO5               5
#define JH7110_SYS_PAD_GPIO6               6
#define JH7110_SYS_PAD_GPIO7               7
#define JH7110_SYS_PAD_GPIO8               8
#define JH7110_SYS_PAD_GPIO9               9
#define JH7110_SYS_PAD_GPIO10              10
#define JH7110_SYS_PAD_GPIO11              11
#define JH7110_SYS_PAD_GPIO12              12
#define JH7110_SYS_PAD_GPIO13              13
#define JH7110_SYS_PAD_GPIO14              14
#define JH7110_SYS_PAD_GPIO15              15
#define JH7110_SYS_PAD_GPIO16              16
#define JH7110_SYS_PAD_GPIO17              17
#define JH7110_SYS_PAD_GPIO18              18
#define JH7110_SYS_PAD_GPIO19              19
#define JH7110_SYS_PAD_GPIO20              20
#define JH7110_SYS_PAD_GPIO21              21
#define JH7110_SYS_PAD_GPIO22              22
#define JH7110_SYS_PAD_GPIO23              23
#define JH7110_SYS_PAD_GPIO24              24
#define JH7110_SYS_PAD_GPIO25              25
#define JH7110_SYS_PAD_GPIO26              26
#define JH7110_SYS_PAD_GPIO27              27
#define JH7110_SYS_PAD_GPIO28              28
#define JH7110_SYS_PAD_GPIO29              29
#define JH7110_SYS_PAD_GPIO30              30
#define JH7110_SYS_PAD_GPIO31              31
#define JH7110_SYS_PAD_GPIO32              32
#define JH7110_SYS_PAD_GPIO33              33
#define JH7110_SYS_PAD_GPIO34              34
#define JH7110_SYS_PAD_GPIO35              35
#define JH7110_SYS_PAD_GPIO36              36
#define JH7110_SYS_PAD_GPIO37              37
#define JH7110_SYS_PAD_GPIO38              38
#define JH7110_SYS_PAD_GPIO39              39
#define JH7110_SYS_PAD_GPIO40              40
#define JH7110_SYS_PAD_GPIO41              41
#define JH7110_SYS_PAD_GPIO42              42
#define JH7110_SYS_PAD_GPIO43              43
#define JH7110_SYS_PAD_GPIO44              44
#define JH7110_SYS_PAD_GPIO45              45
#define JH7110_SYS_PAD_GPIO46              46
#define JH7110_SYS_PAD_GPIO47              47
#define JH7110_SYS_PAD_GPIO48              48
#define JH7110_SYS_PAD_GPIO49              49
#define JH7110_SYS_PAD_GPIO50              50
#define JH7110_SYS_PAD_GPIO51              51
#define JH7110_SYS_PAD_GPIO52              52
#define JH7110_SYS_PAD_GPIO53              53
#define JH7110_SYS_PAD_GPIO54              54
#define JH7110_SYS_PAD_GPIO55              55
#define JH7110_SYS_PAD_GPIO56              56
#define JH7110_SYS_PAD_GPIO57              57
#define JH7110_SYS_PAD_GPIO58              58
#define JH7110_SYS_PAD_GPIO59              59
#define JH7110_SYS_PAD_GPIO60              60
#define JH7110_SYS_PAD_GPIO61              61
#define JH7110_SYS_PAD_GPIO62              62
#define JH7110_SYS_PAD_GPIO63              63
#define JH7110_SYS_PAD_SD0_CLK             64
#define JH7110_SYS_PAD_SD0_CMD             65
#define JH7110_SYS_PAD_SD0_DATA0           66
#define JH7110_SYS_PAD_SD0_DATA1           67
#define JH7110_SYS_PAD_SD0_DATA2           68
#define JH7110_SYS_PAD_SD0_DATA3           69
#define JH7110_SYS_PAD_SD0_DATA4           70
#define JH7110_SYS_PAD_SD0_DATA5           71
#define JH7110_SYS_PAD_SD0_DATA6           72
#define JH7110_SYS_PAD_SD0_DATA7           73
#define JH7110_SYS_PAD_SD0_STRB            74
#define JH7110_SYS_PAD_GMAC1_MDC           75
#define JH7110_SYS_PAD_GMAC1_MDIO          76
#define JH7110_SYS_PAD_GMAC1_RXD0          77
#define JH7110_SYS_PAD_GMAC1_RXD1          78
#define JH7110_SYS_PAD_GMAC1_RXD2          79
#define JH7110_SYS_PAD_GMAC1_RXD3          80
#define JH7110_SYS_PAD_GMAC1_RXDV          81
#define JH7110_SYS_PAD_GMAC1_RXC           82
#define JH7110_SYS_PAD_GMAC1_TXD0          83
#define JH7110_SYS_PAD_GMAC1_TXD1          84
#define JH7110_SYS_PAD_GMAC1_TXD2          85
#define JH7110_SYS_PAD_GMAC1_TXD3          86
#define JH7110_SYS_PAD_GMAC1_TXEN          87
#define JH7110_SYS_PAD_GMAC1_TXC           88
#define JH7110_SYS_PAD_QSPI_SCLK           89
#define JH7110_SYS_PAD_QSPI_CS0            90
#define JH7110_SYS_PAD_QSPI_DATA0          91
#define JH7110_SYS_PAD_QSPI_DATA1          92
#define JH7110_SYS_PAD_QSPI_DATA2          93
#define JH7110_SYS_PAD_QSPI_DATA3          94


/* AON pins */
#define JH7110_AON_PAD_TESTEN              0
#define JH7110_AON_PAD_RGPIO0              1
#define JH7110_AON_PAD_RGPIO1              2
#define JH7110_AON_PAD_RGPIO2              3
#define JH7110_AON_PAD_RGPIO3              4
#define JH7110_AON_PAD_RSTN                5
// doesn't match the register layout in AON IOMUX CFG SAIF SYSCFG'
#define JH7110_AON_PAD_GMAC0_MDC           6
#define JH7110_AON_PAD_GMAC0_MDIO          7
#define JH7110_AON_PAD_GMAC0_RXD0          8
#define JH7110_AON_PAD_GMAC0_RXD1          9
#define JH7110_AON_PAD_GMAC0_RXD2          10
#define JH7110_AON_PAD_GMAC0_RXD3          11
#define JH7110_AON_PAD_GMAC0_RXDV          12
#define JH7110_AON_PAD_GMAC0_RXC           13
#define JH7110_AON_PAD_GMAC0_TXD0          14
#define JH7110_AON_PAD_GMAC0_TXD1          15
#define JH7110_AON_PAD_GMAC0_TXD2          16
#define JH7110_AON_PAD_GMAC0_TXD3          17
#define JH7110_AON_PAD_GMAC0_TXEN          18
#define JH7110_AON_PAD_GMAC0_TXC           19


#if 0
// https://doc-en.rvspace.org/JH7110/TRM/JH7110_TRM/sys_iomux_cfg.html#sys_iomux_cfg__section_gtq_t3b_xsb

static const struct pinctrl_pin_desc jh7110_sys_pins[] = {
	PINCTRL_PIN(PAD_GPIO0,		"GPIO0"),
	PINCTRL_PIN(PAD_GPIO1,		"GPIO1"),
	PINCTRL_PIN(PAD_GPIO2,		"GPIO2"),
	PINCTRL_PIN(PAD_GPIO3,		"GPIO3"),
	PINCTRL_PIN(PAD_GPIO4,		"GPIO4"),
	PINCTRL_PIN(PAD_GPIO5,		"GPIO5"),
	PINCTRL_PIN(PAD_GPIO6,		"GPIO6"),
	PINCTRL_PIN(PAD_GPIO7,		"GPIO7"),
	PINCTRL_PIN(PAD_GPIO8,		"GPIO8"),
	PINCTRL_PIN(PAD_GPIO9,		"GPIO9"),
	PINCTRL_PIN(PAD_GPIO10,		"GPIO10"),
	PINCTRL_PIN(PAD_GPIO11,		"GPIO11"),
	PINCTRL_PIN(PAD_GPIO12,		"GPIO12"),
	PINCTRL_PIN(PAD_GPIO13,		"GPIO13"),
	PINCTRL_PIN(PAD_GPIO14,		"GPIO14"),
	PINCTRL_PIN(PAD_GPIO15,		"GPIO15"),
	PINCTRL_PIN(PAD_GPIO16,		"GPIO16"),
	PINCTRL_PIN(PAD_GPIO17,		"GPIO17"),
	PINCTRL_PIN(PAD_GPIO18,		"GPIO18"),
	PINCTRL_PIN(PAD_GPIO19,		"GPIO19"),
	PINCTRL_PIN(PAD_GPIO20,		"GPIO20"),
	PINCTRL_PIN(PAD_GPIO21,		"GPIO21"),
	PINCTRL_PIN(PAD_GPIO22,		"GPIO22"),
	PINCTRL_PIN(PAD_GPIO23,		"GPIO23"),
	PINCTRL_PIN(PAD_GPIO24,		"GPIO24"),
	PINCTRL_PIN(PAD_GPIO25,		"GPIO25"),
	PINCTRL_PIN(PAD_GPIO26,		"GPIO26"),
	PINCTRL_PIN(PAD_GPIO27,		"GPIO27"),
	PINCTRL_PIN(PAD_GPIO28,		"GPIO28"),
	PINCTRL_PIN(PAD_GPIO29,		"GPIO29"),
	PINCTRL_PIN(PAD_GPIO30,		"GPIO30"),
	PINCTRL_PIN(PAD_GPIO31,		"GPIO31"),
	PINCTRL_PIN(PAD_GPIO32,		"GPIO32"),
	PINCTRL_PIN(PAD_GPIO33,		"GPIO33"),
	PINCTRL_PIN(PAD_GPIO34,		"GPIO34"),
	PINCTRL_PIN(PAD_GPIO35,		"GPIO35"),
	PINCTRL_PIN(PAD_GPIO36,		"GPIO36"),
	PINCTRL_PIN(PAD_GPIO37,		"GPIO37"),
	PINCTRL_PIN(PAD_GPIO38,		"GPIO38"),
	PINCTRL_PIN(PAD_GPIO39,		"GPIO39"),
	PINCTRL_PIN(PAD_GPIO40,		"GPIO40"),
	PINCTRL_PIN(PAD_GPIO41,		"GPIO41"),
	PINCTRL_PIN(PAD_GPIO42,		"GPIO42"),
	PINCTRL_PIN(PAD_GPIO43,		"GPIO43"),
	PINCTRL_PIN(PAD_GPIO44,		"GPIO44"),
	PINCTRL_PIN(PAD_GPIO45,		"GPIO45"),
	PINCTRL_PIN(PAD_GPIO46,		"GPIO46"),
	PINCTRL_PIN(PAD_GPIO47,		"GPIO47"),
	PINCTRL_PIN(PAD_GPIO48,		"GPIO48"),
	PINCTRL_PIN(PAD_GPIO49,		"GPIO49"),
	PINCTRL_PIN(PAD_GPIO50,		"GPIO50"),
	PINCTRL_PIN(PAD_GPIO51,		"GPIO51"),
	PINCTRL_PIN(PAD_GPIO52,		"GPIO52"),
	PINCTRL_PIN(PAD_GPIO53,		"GPIO53"),
	PINCTRL_PIN(PAD_GPIO54,		"GPIO54"),
	PINCTRL_PIN(PAD_GPIO55,		"GPIO55"),
	PINCTRL_PIN(PAD_GPIO56,		"GPIO56"),
	PINCTRL_PIN(PAD_GPIO57,		"GPIO57"),
	PINCTRL_PIN(PAD_GPIO58,		"GPIO58"),
	PINCTRL_PIN(PAD_GPIO59,		"GPIO59"),
	PINCTRL_PIN(PAD_GPIO60,		"GPIO60"),
	PINCTRL_PIN(PAD_GPIO61,		"GPIO61"),
	PINCTRL_PIN(PAD_GPIO62,		"GPIO62"),
	PINCTRL_PIN(PAD_GPIO63,		"GPIO63"),
	PINCTRL_PIN(PAD_SD0_CLK,	"SD0_CLK"),
	PINCTRL_PIN(PAD_SD0_CMD,	"SD0_CMD"),
	PINCTRL_PIN(PAD_SD0_DATA0,	"SD0_DATA0"),
	PINCTRL_PIN(PAD_SD0_DATA1,	"SD0_DATA1"),
	PINCTRL_PIN(PAD_SD0_DATA2,	"SD0_DATA2"),
	PINCTRL_PIN(PAD_SD0_DATA3,	"SD0_DATA3"),
	PINCTRL_PIN(PAD_SD0_DATA4,	"SD0_DATA4"),
	PINCTRL_PIN(PAD_SD0_DATA5,	"SD0_DATA5"),
	PINCTRL_PIN(PAD_SD0_DATA6,	"SD0_DATA6"),
	PINCTRL_PIN(PAD_SD0_DATA7,	"SD0_DATA7"),
	PINCTRL_PIN(PAD_SD0_STRB,	"SD0_STRB"),
	PINCTRL_PIN(PAD_GMAC1_MDC,	"GMAC1_MDC"),
	PINCTRL_PIN(PAD_GMAC1_MDIO,	"GMAC1_MDIO"),
	PINCTRL_PIN(PAD_GMAC1_RXD0,	"GMAC1_RXD0"),
	PINCTRL_PIN(PAD_GMAC1_RXD1,	"GMAC1_RXD1"),
	PINCTRL_PIN(PAD_GMAC1_RXD2,	"GMAC1_RXD2"),
	PINCTRL_PIN(PAD_GMAC1_RXD3,	"GMAC1_RXD3"),
	PINCTRL_PIN(PAD_GMAC1_RXDV,	"GMAC1_RXDV"),
	PINCTRL_PIN(PAD_GMAC1_RXC,	"GMAC1_RXC"),
	PINCTRL_PIN(PAD_GMAC1_TXD0,	"GMAC1_TXD0"),
	PINCTRL_PIN(PAD_GMAC1_TXD1,	"GMAC1_TXD1"),
	PINCTRL_PIN(PAD_GMAC1_TXD2,	"GMAC1_TXD2"),
	PINCTRL_PIN(PAD_GMAC1_TXD3,	"GMAC1_TXD3"),
	PINCTRL_PIN(PAD_GMAC1_TXEN,	"GMAC1_TXEN"),
	PINCTRL_PIN(PAD_GMAC1_TXC,	"GMAC1_TXC"),
	PINCTRL_PIN(PAD_QSPI_SCLK,	"QSPI_SCLK"),
	PINCTRL_PIN(PAD_QSPI_CS0,	"QSPI_CS0"),
	PINCTRL_PIN(PAD_QSPI_DATA0,	"QSPI_DATA0"),
	PINCTRL_PIN(PAD_QSPI_DATA1,	"QSPI_DATA1"),
	PINCTRL_PIN(PAD_QSPI_DATA2,	"QSPI_DATA2"),
	PINCTRL_PIN(PAD_QSPI_DATA3,	"QSPI_DATA3"),
};


// https://doc-en.rvspace.org/JH7110/TRM/JH7110_TRM/aon_iomux_cfg.html#aon_iomux_cfg__section_xl4_w3b_xsb
static const struct pinctrl_pin_desc jh7110_aon_pins[] = {
	PINCTRL_PIN(PAD_TESTEN,		"TESTEN"),
	PINCTRL_PIN(PAD_RGPIO0,		"RGPIO0"),
	PINCTRL_PIN(PAD_RGPIO1,		"RGPIO1"),
	PINCTRL_PIN(PAD_RGPIO2,		"RGPIO2"),
	PINCTRL_PIN(PAD_RGPIO3,		"RGPIO3"),
	PINCTRL_PIN(PAD_RSTN,		"RSTN"),
// RTC_D
	PINCTRL_PIN(PAD_GMAC0_MDC,	"GMAC0_MDC"),
	PINCTRL_PIN(PAD_GMAC0_MDIO,	"GMAC0_MDIO"),
	PINCTRL_PIN(PAD_GMAC0_RXD0,	"GMAC0_RXD0"),
	PINCTRL_PIN(PAD_GMAC0_RXD1,	"GMAC0_RXD1"),
	PINCTRL_PIN(PAD_GMAC0_RXD2,	"GMAC0_RXD2"),
	PINCTRL_PIN(PAD_GMAC0_RXD3,	"GMAC0_RXD3"),
	PINCTRL_PIN(PAD_GMAC0_RXDV,	"GMAC0_RXDV"),
	PINCTRL_PIN(PAD_GMAC0_RXC,	"GMAC0_RXC"),
	PINCTRL_PIN(PAD_GMAC0_TXD0,	"GMAC0_TXD0"),
	PINCTRL_PIN(PAD_GMAC0_TXD1,	"GMAC0_TXD1"),
	PINCTRL_PIN(PAD_GMAC0_TXD2,	"GMAC0_TXD2"),
	PINCTRL_PIN(PAD_GMAC0_TXD3,	"GMAC0_TXD3"),
	PINCTRL_PIN(PAD_GMAC0_TXEN,	"GMAC0_TXEN"),
	PINCTRL_PIN(PAD_GMAC0_TXC,	"GMAC0_TXC"),
};

#endif

struct jh7110_func_sel {
    uint16_t	jfs_funcreg;
    uint16_t	jfs_max;
    uint32_t	jfs_mask;
};

#define JH7110_FS(_reg, _mask, _max)					\
    {									\
	.jfs_funcreg = (_reg),						\
	.jfs_max = (_max),						\
	.jfs_mask = (_mask),						\
    }

// https://doc-en.rvspace.org/JH7110/TRM/JH7110_TRM/sys_iomux_cfg.html#sys_iomux_cfg__section_fw2_v3b_xsb
static const struct jh7110_func_sel jh7110_sys_func_sel[] = {
	[JH7110_SYS_PAD_GMAC1_RXC] = JH7110_FS(0x29c, __BITS( 1, 0), 1),
	[JH7110_SYS_PAD_GPIO10]    = JH7110_FS(0x29c, __BITS( 4, 2), 3),
	[JH7110_SYS_PAD_GPIO11]    = JH7110_FS(0x29c, __BITS( 7, 5), 3),
	[JH7110_SYS_PAD_GPIO12]    = JH7110_FS(0x29c, __BITS(10, 8), 3),
	[JH7110_SYS_PAD_GPIO13]    = JH7110_FS(0x29c, __BITS(13,11), 3),
	[JH7110_SYS_PAD_GPIO14]    = JH7110_FS(0x29c, __BITS(16,14), 3),
	[JH7110_SYS_PAD_GPIO15]    = JH7110_FS(0x29c, __BITS(19,17), 3),
	[JH7110_SYS_PAD_GPIO16]    = JH7110_FS(0x29c, __BITS(22,20), 3),
	[JH7110_SYS_PAD_GPIO17]    = JH7110_FS(0x29c, __BITS(25,23), 3),
	[JH7110_SYS_PAD_GPIO18]    = JH7110_FS(0x29c, __BITS(28,26), 3),
	[JH7110_SYS_PAD_GPIO19]    = JH7110_FS(0x29c, __BITS(31,29), 3),

	[JH7110_SYS_PAD_GPIO20]    = JH7110_FS(0x2a0, __BITS( 2, 0), 3),
	[JH7110_SYS_PAD_GPIO21]    = JH7110_FS(0x2a0, __BITS( 5, 3), 3),
	[JH7110_SYS_PAD_GPIO22]    = JH7110_FS(0x2a0, __BITS( 8, 6), 3),
	[JH7110_SYS_PAD_GPIO23]    = JH7110_FS(0x2a0, __BITS(11, 9), 3),
	[JH7110_SYS_PAD_GPIO24]    = JH7110_FS(0x2a0, __BITS(14,12), 3),
	[JH7110_SYS_PAD_GPIO25]    = JH7110_FS(0x2a0, __BITS(17,15), 3),
	[JH7110_SYS_PAD_GPIO26]    = JH7110_FS(0x2a0, __BITS(20,18), 3),
	[JH7110_SYS_PAD_GPIO27]    = JH7110_FS(0x2a0, __BITS(23,21), 3),
	[JH7110_SYS_PAD_GPIO28]    = JH7110_FS(0x2a0, __BITS(26,24), 3),
	[JH7110_SYS_PAD_GPIO29]    = JH7110_FS(0x2a0, __BITS(29,27), 3),

	[JH7110_SYS_PAD_GPIO30]    = JH7110_FS(0x2a4, __BITS( 2, 0), 3),
	[JH7110_SYS_PAD_GPIO31]    = JH7110_FS(0x2a4, __BITS( 5, 3), 3),
	[JH7110_SYS_PAD_GPIO32]    = JH7110_FS(0x2a4, __BITS( 8, 6), 3),
	[JH7110_SYS_PAD_GPIO33]    = JH7110_FS(0x2a4, __BITS(11, 9), 3),
	[JH7110_SYS_PAD_GPIO34]    = JH7110_FS(0x2a4, __BITS(14,12), 3),
	[JH7110_SYS_PAD_GPIO35]    = JH7110_FS(0x2a4, __BITS(17,15), 3),
	[JH7110_SYS_PAD_GPIO36]    = JH7110_FS(0x2a4, __BITS(19,17), 3),
	[JH7110_SYS_PAD_GPIO37]    = JH7110_FS(0x2a4, __BITS(23,20), 3),
	[JH7110_SYS_PAD_GPIO38]    = JH7110_FS(0x2a4, __BITS(26,23), 3),
	[JH7110_SYS_PAD_GPIO39]    = JH7110_FS(0x2a4, __BITS(28,26), 3),
	[JH7110_SYS_PAD_GPIO40]    = JH7110_FS(0x2a4, __BITS(31,29), 3),

	[JH7110_SYS_PAD_GPIO41]    = JH7110_FS(0x2a8, __BITS( 2, 0), 3),
	[JH7110_SYS_PAD_GPIO42]    = JH7110_FS(0x2a8, __BITS( 5, 3), 3),
	[JH7110_SYS_PAD_GPIO43]    = JH7110_FS(0x2a8, __BITS( 8, 6), 3),
	[JH7110_SYS_PAD_GPIO44]    = JH7110_FS(0x2a8, __BITS(11, 9), 3),
	[JH7110_SYS_PAD_GPIO45]    = JH7110_FS(0x2a8, __BITS(14,12), 3),
	[JH7110_SYS_PAD_GPIO46]    = JH7110_FS(0x2a8, __BITS(17,15), 3),
	[JH7110_SYS_PAD_GPIO47]    = JH7110_FS(0x2a8, __BITS(20,18), 3),
	[JH7110_SYS_PAD_GPIO48]    = JH7110_FS(0x2a8, __BITS(23,21), 3),
	[JH7110_SYS_PAD_GPIO49]    = JH7110_FS(0x2a8, __BITS(26,24), 3),
	[JH7110_SYS_PAD_GPIO50]    = JH7110_FS(0x2a8, __BITS(29,27), 3),
	[JH7110_SYS_PAD_GPIO51]    = JH7110_FS(0x2a8, __BITS(31,30), 3),

	[JH7110_SYS_PAD_GPIO52]    = JH7110_FS(0x2ac, __BITS( 1, 0), 3),
	[JH7110_SYS_PAD_GPIO53]    = JH7110_FS(0x2ac, __BITS( 3, 2), 3),
	[JH7110_SYS_PAD_GPIO54]    = JH7110_FS(0x2ac, __BITS( 5, 4), 3),
	[JH7110_SYS_PAD_GPIO55]    = JH7110_FS(0x2ac, __BITS( 8, 6), 3),
	[JH7110_SYS_PAD_GPIO56]    = JH7110_FS(0x2ac, __BITS(11, 9), 3),
	[JH7110_SYS_PAD_GPIO57]    = JH7110_FS(0x2ac, __BITS(14,12), 3),
	[JH7110_SYS_PAD_GPIO58]    = JH7110_FS(0x2ac, __BITS(17,15), 3),
	[JH7110_SYS_PAD_GPIO59]    = JH7110_FS(0x2ac, __BITS(20,18), 3),
	[JH7110_SYS_PAD_GPIO60]    = JH7110_FS(0x2ac, __BITS(23,21), 3),
	[JH7110_SYS_PAD_GPIO61]    = JH7110_FS(0x2ac, __BITS(26,24), 3),
	[JH7110_SYS_PAD_GPIO62]    = JH7110_FS(0x2ac, __BITS(29,27), 3),
	[JH7110_SYS_PAD_GPIO63]    = JH7110_FS(0x2ac, __BITS(31,30), 3),

	[JH7110_SYS_PAD_GPIO6]     = JH7110_FS(0x2b0, __BITS( 1, 0), 3),
	[JH7110_SYS_PAD_GPIO7]     = JH7110_FS(0x2b0, __BITS( 4, 2), 3),
	[JH7110_SYS_PAD_GPIO8]     = JH7110_FS(0x2b0, __BITS( 7, 5), 3),
	[JH7110_SYS_PAD_GPIO9]     = JH7110_FS(0x2b0, __BITS(10, 8), 3),
};

static void
jh7110_set_function(struct jh7110_pinctrl_softc *sc, u_int pin_no,
    u_int func)
{
	if (pin_no >= __arraycount(jh7110_sys_func_sel))
		return;

	const struct jh7110_func_sel * const jfs =
	    &jh7110_sys_func_sel[pin_no];

	if (func > jfs->jfs_max)
		return;

	if (jfs->jfs_funcreg == 0)
		return;

	uint32_t funcold, funcval;
	mutex_enter(&sc->sc_lock);
	funcold = RD4(sc, jfs->jfs_funcreg);

	funcval = funcold & ~jfs->jfs_mask;
	funcval |= __SHIFTIN(func, jfs->jfs_mask);

	WR4(sc, jfs->jfs_funcreg, funcval);
	mutex_exit(&sc->sc_lock);
}

#if 0
//
static const struct jh7110_vin_group_sel
	jh7110_sys_vin_group_sel[ARRAY_SIZE(jh7110_sys_pins)] = {
	[PAD_GPIO6]     = { 0x2b4, 21, 0 },
	[PAD_GPIO7]     = { 0x2b4, 18, 0 },
	[PAD_GPIO8]     = { 0x2b4, 15, 0 },
	[PAD_GPIO9]     = { 0x2b0, 11, 0 },
	[PAD_GPIO10]    = { 0x2b0, 20, 0 },
	[PAD_GPIO11]    = { 0x2b0, 23, 0 },
	[PAD_GPIO12]    = { 0x2b0, 26, 0 },
	[PAD_GPIO13]    = { 0x2b0, 29, 0 },
	[PAD_GPIO14]    = { 0x2b4,  0, 0 },
	[PAD_GPIO15]    = { 0x2b4,  3, 0 },
	[PAD_GPIO16]    = { 0x2b4,  6, 0 },
	[PAD_GPIO17]    = { 0x2b4,  9, 0 },
	[PAD_GPIO18]    = { 0x2b4, 12, 0 },
	[PAD_GPIO19]    = { 0x2b0, 14, 0 },
	[PAD_GPIO20]    = { 0x2b0, 17, 0 },

	[PAD_GPIO21]    = { 0x2b4, 21, 1 },
	[PAD_GPIO22]    = { 0x2b4, 18, 1 },
	[PAD_GPIO23]    = { 0x2b4, 15, 1 },
	[PAD_GPIO24]    = { 0x2b0, 11, 1 },
	[PAD_GPIO25]    = { 0x2b0, 20, 1 },
	[PAD_GPIO26]    = { 0x2b0, 23, 1 },
	[PAD_GPIO27]    = { 0x2b0, 26, 1 },
	[PAD_GPIO28]    = { 0x2b0, 29, 1 },
	[PAD_GPIO29]    = { 0x2b4,  0, 1 },
	[PAD_GPIO30]    = { 0x2b4,  3, 1 },
	[PAD_GPIO31]    = { 0x2b4,  6, 1 },
	[PAD_GPIO32]    = { 0x2b4,  9, 1 },
	[PAD_GPIO33]    = { 0x2b4, 12, 1 },
	[PAD_GPIO34]    = { 0x2b0, 14, 1 },
	[PAD_GPIO35]    = { 0x2b0, 17, 1 },

	[PAD_GPIO36]    = { 0x2b4, 21, 2 },
	[PAD_GPIO37]    = { 0x2b4, 18, 2 },
	[PAD_GPIO38]    = { 0x2b4, 15, 2 },
	[PAD_GPIO39]    = { 0x2b0, 11, 2 },
	[PAD_GPIO40]    = { 0x2b0, 20, 2 },
	[PAD_GPIO41]    = { 0x2b0, 23, 2 },
	[PAD_GPIO42]    = { 0x2b0, 26, 2 },
	[PAD_GPIO43]    = { 0x2b0, 29, 2 },
	[PAD_GPIO44]    = { 0x2b4,  0, 2 },
	[PAD_GPIO45]    = { 0x2b4,  3, 2 },
	[PAD_GPIO46]    = { 0x2b4,  6, 2 },
	[PAD_GPIO47]    = { 0x2b4,  9, 2 },
	[PAD_GPIO48]    = { 0x2b4, 12, 2 },
	[PAD_GPIO49]    = { 0x2b0, 14, 2 },
	[PAD_GPIO50]    = { 0x2b0, 17, 2 },
};


static void
jh7110_set_vin_group(struct jh7110_pinctrl_softc *sc, u_int pin_no)
{
	if (pin_no >= __arraycount(jh7110_sys_func_sel))
		return;

	const struct jh7110_func_sel * const jfs =
	    &jh7110_sys_func_sel[pin_no];

	const struct jh7110_vin_group_sel *gs = &jh7110_sys_vin_group_sel[pin];
	unsigned long flags;
	void __iomem *reg;
	u32 mask;
	u32 grp;

	if (!gs->offset)
		return;

	reg = sfp->base + gs->offset;
	grp = gs->group << gs->shift;
	mask = 0x3U << gs->shift;

	raw_spin_lock_irqsave(&sfp->lock, flags);
	grp |= readl_relaxed(reg) & ~mask;
	writel_relaxed(grp, reg);
	raw_spin_unlock_irqrestore(&sfp->lock, flags);
}


#endif


static void
jh7110_set_gpiomux(struct jh7110_pinctrl_softc * const sc, u_int pin_no,
    u_int din, u_int dout, u_int doen)
{
	const struct jh7110_pinctrl_data * const jpd = sc->sc_jpd;
	const u_int offset = 4 * (pin_no / 4);
	const u_int shift = 8 * (pin_no % 4);
	const uint32_t dout_mask = jpd->jpd_dout_mask << shift;
	const uint32_t doen_mask = jpd->jpd_doen_mask << shift;
	const bus_size_t dout_reg = jpd->jpd_dout + offset;
	const bus_size_t doen_reg = jpd->jpd_doen + offset;
	uint32_t doutval, doutold;
	uint32_t doenval, doenold;
	uint32_t dinval, dinold;

	mutex_enter(&sc->sc_lock);
	doutold = RD4(sc, dout_reg);
	doutval = doutold & ~dout_mask;
	doutval |= __SHIFTIN(dout, dout_mask);

	doenold = RD4(sc, doen_reg);
	doenval = doenold & ~doen_mask;
	doenval |= __SHIFTIN(doen, doen_mask);

	WR4(sc, dout_reg, doutval);
	WR4(sc, doen_reg, doenval);
	if (din != GPI_NONE) {
		const u_int din_offset = 4 * (din / 4);
		const u_int din_shift = 8 * (din % 4);
		const uint32_t din_mask = jpd->jpd_gpi_mask << din_shift;
		const bus_size_t din_reg = jpd->jpd_gpi + din_offset;

		dinold = RD4(sc, din_reg);
		dinval = dinold & ~din_mask;
		/*
		 * The register value indicates the selected GPIO number + 2
		 * (GPIO2 - GPIO63, GPIO0 and GPIO1 are not available) for the
		 * input signal.
		 */
		dinval |= __SHIFTIN(pin_no + 2, din_mask);
		WR4(sc, din_reg, dinval);
	}
	mutex_exit(&sc->sc_lock);

	aprint_debug_dev(sc->sc_dev, "set_config: "
	    "gpio %d dout %#x/%#x doen %#x/%#x din %#x/%#x\n",
	    pin_no,
	    doutval, doutold,
	    doenval, doenold,
	    din != GPI_NONE ? dinval : 0,
	    din != GPI_NONE ? dinold : 0);
}


static const struct jh7110_pinctrl_data jh7110_aon_pinctrl_data = {
//	.jpd_pins	= jh7110_aon_pins,
//	.jpd_npins	= ARRAY_SIZE(jh7110_aon_pins),
	.jpd_npins	= JH7110_AON_NPIN,
	.jpd_ngpios	= JH7110_AON_NGPIO,
//	.gc	= JH7110_AON_GC_BASE,
	.jpd_doen	= JH7110_AON_DOEN,
	.jpd_doen_mask	= __BITS(2, 0),
	.jpd_dout	= JH7110_AON_DOUT,
	.jpd_dout_mask	= __BITS(3, 0),
	.jpd_gpi	= JH7110_AON_GPI,
	.jpd_gpi_mask	= __BITS(3, 0),
	.jpd_gpioin	= JH7110_AON_GPIOIN,

//	.irq_reg		   = &jh7110_aon_irq_reg,
//	.nsaved_regs		   = JH7110_AON_REGS_NUM,
//	.jh7110_get_padcfgsc_base  = jh7110_aon_get_padcfgsc_base,
//	.jh7110_gpio_irq_handler = jh7110_aon_irq_handler,
//	.jh7110_gpio_init_hw	 = jh7110_aon_init_hw,
};



static const struct jh7110_pinctrl_data jh7110_sys_pinctrl_data = {
//	.pins		= jh7110_sys_pins,
//	.npins		= ARRAY_SIZE(jh7110_sys_pins),
	.jpd_npins	= JH7110_SYS_NPIN,
	.jpd_ngpios	= JH7110_SYS_NGPIO,
//	.jpd_gc	= JH7110_SYS_GC_BASE,

	.jpd_doen	= JH7110_SYS_DOEN,
	.jpd_doen_mask	= __BITS(5, 0),
	.jpd_dout	= JH7110_SYS_DOUT,
	.jpd_dout_mask	= __BITS(6, 0),
	.jpd_gpi	= JH7110_SYS_GPI,
	.jpd_gpi_mask	= __BITS(6, 0),
	.jpd_gpioin	= JH7110_SYS_GPIOIN,

//	.irq		   = &jh7110_sys_irq_reg,
//	.nsaved_regs		   = JH7110_SYS_REGS_NUM,
//	.jh7110_set_one_pin_mux  = jh7110_sys_set_one_pin_mux,
//	.jh7110_get_padcfgsc_base  = jh7110_sys_get_padcfg_base,
//	.jh7110_gpio_irq_handler = jh7110_sys_irq_handler,
//	.jh7110_gpio_init_hw	 = jh7110_sys_init_hw,
};


static int
jh7110_set_pinmux(struct jh7110_pinctrl_softc *sc, u_int pin,
    u_int din, u_int dout, u_int doen, u_int func)
{
	const struct jh7110_pinctrl_data * const jpd = sc->sc_jpd;

	if (pin < jpd->jpd_ngpios && func == 0)
		jh7110_set_gpiomux(sc, pin, din, dout, doen);
	return 0;

	if (sc->sc_jpd == &jh7110_aon_pinctrl_data)
		return 0;

	if (pin < jpd->jpd_npins)
		jh7110_set_function(sc, pin, func);

#if 0
	if (pin < jpd->jpd_ngpios && func == 2)
		jh7110_set_vin_group(sc, pin);
#endif
	return 0;
}

/* Device Tree encoding */
#define DT_PINMUX_DIN_MASK	__BITS(31, 24)
#define DT_PINMUX_DOUT_MASK	__BITS(23, 16)
#define DT_PINMUX_DOEN_MASK	__BITS(15, 10)
#define DT_PINMUX_FUNC_MASK	__BITS( 9,  8)
#define DT_PINMUX_PIN_MASK	__BITS( 7,  0)

static int
jh7110_parse_slew_rate(int phandle)
{
	int slew_rate;

	if (of_getprop_uint32(phandle, "slew-rate", &slew_rate) == 0)
                return slew_rate;

	return -1;
}

static void
jh7110_pinctrl_pin_properties(struct jh7110_pinctrl_softc *sc, int phandle,
    uint16_t *val, uint16_t *mask)
{
	*mask = 0;
	*val = 0;

	const int bias = fdtbus_pinctrl_parse_bias(phandle, NULL);
	const int drive_strength = fdtbus_pinctrl_parse_drive_strength(phandle);
	const int slew_rate = jh7110_parse_slew_rate(phandle);

#define JH7110_PADCFG_POS	__BIT(7)
#define JH7110_PADCFG_SMT	__BIT(6)
#define JH7110_PADCFG_SLEW	__BIT(5)

	switch (bias) {
	case 0:
		*mask |= JH7110_PADCFG_BIAS_MASK;
		break;
	case GPIO_PIN_PULLUP:
		*mask |= JH7110_PADCFG_BIAS_MASK;
		*val  |= JH7110_PADCFG_PU;
		break;
	case GPIO_PIN_PULLDOWN:
		*mask |= JH7110_PADCFG_BIAS_MASK;
		*val  |= JH7110_PADCFG_PD;
		break;
	case -1:
	default:
		break;
	}

	switch (drive_strength) {
	case 2:
		*mask |=  JH7110_PADCFG_DS_MASK;
		*val  |=  JH7110_PADCFG_DS_2MA;
		break;
	case 4:
		*mask |=  JH7110_PADCFG_DS_MASK;
		*val  |=  JH7110_PADCFG_DS_4MA;
		break;
	case 8:
		*mask |=  JH7110_PADCFG_DS_MASK;
		*val  |=  JH7110_PADCFG_DS_8MA;
		break;
	case 12:
		*mask |=  JH7110_PADCFG_DS_MASK;
		*val  |=  JH7110_PADCFG_DS_12MA;
		break;
	case -1:
		break;
	default:
		aprint_error_dev(sc->sc_dev, "phandle %d invalid drive "
		"strength %d\n", phandle, drive_strength);
	}

	if (of_hasprop(phandle, "input-enable")) {
		*mask |= JH7110_PADCFG_IE;
		*val  |= JH7110_PADCFG_IE;
	}
	if (of_hasprop(phandle, "input-disable")) {
		*mask |=  JH7110_PADCFG_IE;
		*val  &= ~JH7110_PADCFG_IE;
	}
	if (of_hasprop(phandle, "input-schmitt-enable")) {
		*mask |=  JH7110_PADCFG_SMT;
		*val  |=  JH7110_PADCFG_SMT;
	}
	if (of_hasprop(phandle, "input-schmitt-disable")) {
		*mask |=  JH7110_PADCFG_SMT;
		*val  &= ~JH7110_PADCFG_SMT;
	}

	switch (slew_rate) {
	case 0:
		*mask |=  JH7110_PADCFG_SLEW;
		*val  &= ~JH7110_PADCFG_SLEW;
		break;
	case 1:
		*mask |=  JH7110_PADCFG_SLEW;
		*val  |=  JH7110_PADCFG_SLEW;
		break;
	case -1:
		break;
	default:
		aprint_error_dev(sc->sc_dev, "invalid slew rate");
	}
}


static void
jh7110_pinctrl_set_config_group(struct jh7110_pinctrl_softc *sc, int group)
{
	int pins_len, pinmux_len;
	const u_int *pins = fdtbus_get_prop(group, "pins", &pins_len);
	const u_int *pinmux = fdtbus_get_prop(group, "pinmux", &pinmux_len);
	size_t plen;
	const u_int *parray;

	aprint_debug_dev(sc->sc_dev, "set_config: group   %d\n", group);
	// XXXNH check binding to see if 'pins' makes sense.
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
		plen = pins_len;
		parray = pins;
	}
	if (pinmux != NULL) {
		plen = pinmux_len;
		parray = pinmux;
	}
	const size_t npins = plen / sizeof(uint32_t);

	aprint_debug_dev(sc->sc_dev, "set_config: group   %d, len %zu\n",
	    group, plen);

	uint16_t val, mask;
	jh7110_pinctrl_pin_properties(sc, group, &val, &mask);

	for (size_t i = 0; i < npins; i++) {
		uint32_t p = be32dec(&parray[i]);
		u_int pin_no;

#if 0
		if (pins != NULL) {
			pin_no = p;
			aprint_debug_dev(sc->sc_dev, "set_config: group   %d"
			    ", gpio %d doen %#x\n", group, pin_no,
			    RD4(sc, GPO_DOEN_CFG(pin_no)));
			WR4(sc, GPO_DOEN_CFG(pin_no), GPO_DISABLE);
			jh7110_padctl_rmw(sc, pin_no,
			    val, mask);
		}
#endif
		if (pinmux != NULL) {
			u_int din = __SHIFTOUT(p, DT_PINMUX_DIN_MASK);
			u_int dout = __SHIFTOUT(p, DT_PINMUX_DOUT_MASK);
			u_int doen = __SHIFTOUT(p, DT_PINMUX_DOEN_MASK);
			u_int func = __SHIFTOUT(p, DT_PINMUX_FUNC_MASK);
			pin_no = __SHIFTOUT(p, DT_PINMUX_PIN_MASK);
			jh7110_set_pinmux(sc, pin_no, din, dout,
			    doen, func);
		}
	}
}

static int
jh7110_pinctrl_set_config(device_t dev, const void *data, size_t len)
{
	struct jh7110_pinctrl_softc * const sc = device_private(dev);

	if (len != sizeof(uint32_t))
		return -1;

	const int phandle = fdtbus_get_phandle_from_native(be32dec(data));
	aprint_debug_dev(sc->sc_dev, "set_config: phandle %d\n", phandle);

	for (int child = OF_child(phandle); child; child = OF_peer(child)) {
		jh7110_pinctrl_set_config_group(sc, child);
	}

	return 0;
}

static struct fdtbus_pinctrl_controller_func jh7110_pinctrl_funcs = {
	.set_config = jh7110_pinctrl_set_config,
};


static void *
jh7110_pinctrl_gpio_acquire(device_t dev, const void *data, size_t len, int flags)
{
	struct jh7110_pinctrl_softc * const sc = device_private(dev);

	if (len != 3 * sizeof(uint32_t))
		return NULL;

	const u_int *gpio = data;
	const u_int pin_no = be32toh(gpio[1]);
	const bool actlo = be32toh(gpio[2]) & 1;

	// XXXNH twiddle something??
	struct jh7110_pinctrl_gpio_pin *pin =
	    kmem_zalloc(sizeof(*pin), KM_SLEEP);
	pin->pin_sc = sc;
	pin->pin_no = pin_no;
	pin->pin_actlo = actlo;

	return pin;
}

static void
jh7110_pinctrl_gpio_release(device_t dev, void *priv)
{
	struct jh7110_pinctrl_softc * const sc = device_private(dev);
	struct jh7110_pinctrl_gpio_pin *pin = priv;

	KASSERT(sc == pin->pin_sc);
	// XXXNH untwiddle something?
	kmem_free(pin, sizeof(*pin));
}

static int
jh7110_pinctrl_gpio_read(device_t dev, void *priv, bool raw)
{
	struct jh7110_pinctrl_softc * const sc = device_private(dev);
	struct jh7110_pinctrl_gpio_pin *pin = priv;

	const u_int pin_no = pin ->pin_no;
	const u_int pins_per_bank = 32;
	const size_t banksz = sizeof(uint32_t);
	const bus_size_t offset = ((pin_no) / pins_per_bank) * banksz;
	const uint32_t mask = __BIT(pin_no % pins_per_bank);
	const uint32_t bank = RD4(sc, sc->sc_jpd->jpd_gpioin + offset);

	aprint_verbose_dev(sc->sc_dev, "pin_no %3d banksz %2zu "
	    "offset %#5" PRIxBUSSIZE " mask %2x gpioinreg %#"PRIxBUSSIZE" "
	    "bank %#"PRIx32"\n",
	    pin_no, banksz, offset, mask,
	    sc->sc_jpd->jpd_gpioin + offset, bank);

	int val = __SHIFTOUT(bank, mask);
	if (!raw && pin->pin_actlo)
		val = !val;

	return val;
}


static void
jh7110_pinctrl_gpio_write(device_t dev, void *priv, int val, bool raw)
{
	struct jh7110_pinctrl_softc * const sc = device_private(dev);
	struct jh7110_pinctrl_gpio_pin *pin = priv;

	const u_int pin_no = pin ->pin_no;
	const u_int pins_per_bank = 4;
	const size_t banksz = sizeof(uint32_t);
	const u_int bits_per_bank = banksz * NBBY;
	const u_int bits_per_pin = bits_per_bank / pins_per_bank;
	const bus_size_t offset = ((pin_no) / pins_per_bank) * banksz;
	const u_int shift = bits_per_pin * (pin_no % pins_per_bank);
	const uint32_t mask = sc->sc_jpd->jpd_dout_mask << shift;

	if (!raw && pin->pin_actlo)
		val = !val;

	mutex_enter(&sc->sc_lock);
	uint32_t bank = RD4(sc, sc->sc_jpd->jpd_dout + offset);
	uint32_t obank = bank;
	bank &= ~mask;
	bank |= __SHIFTIN(val != 0 ? GPOUT_HIGH : GPOUT_LOW, mask);
	WR4(sc, sc->sc_jpd->jpd_dout + offset, bank);
	mutex_exit(&sc->sc_lock);

	aprint_verbose_dev(sc->sc_dev, "pin_no %3d banksz %2zu bpb %2u "
	    "bpp %2u offset %#5" PRIxBUSSIZE " mask %2x "
	    "doutreg %#"PRIxBUSSIZE" bank %#"PRIx32"/%#"PRIx32"\n",
	    pin_no, banksz, bits_per_bank, bits_per_pin, offset, mask,
	    sc->sc_jpd->jpd_dout_mask + offset, obank, bank);

}


static struct fdtbus_gpio_controller_func jh7110_pinctrl_gpio_funcs = {
	.acquire = jh7110_pinctrl_gpio_acquire,
	.release = jh7110_pinctrl_gpio_release,
	.read = jh7110_pinctrl_gpio_read,
	.write = jh7110_pinctrl_gpio_write,
};


static const struct device_compatible_entry compat_data[] = {
	{ .compat = "starfive,jh7110-sys-pinctrl", .data = &jh7110_sys_pinctrl_data },
	{ .compat = "starfive,jh7110-aon-pinctrl", .data = &jh7110_aon_pinctrl_data },
	DEVICE_COMPAT_EOL
};


static int
jh7110_pinctrl_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
jh7110_pinctrl_attach(device_t parent, device_t self, void *aux)
{
	struct jh7110_pinctrl_softc *sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;
	int error;

	sc->sc_dev = self;
	sc->sc_phandle = phandle;
	sc->sc_bst = faa->faa_bst;
#if 0
	if (!of_hasprop(phandle, "gpio-controller")) {
		aprint_error(": no gpio controller");
		return;
	}
#endif
	error = fdtbus_get_reg(phandle, 0, &addr, &size);
	if (error) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	error = bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh);
	if (error) {
		aprint_error(": couldn't map %#" PRIxBUSADDR ": %d", addr,
		    error);
		return;
	}

	// check for pins / npins
#if 0
	/* enable clocks */
	struct clk *clk;
	fdtbus_clock_assign(phandle);
	for (u_int c = 0; (clk = fdtbus_clock_get_index(phandle, c)) != NULL; c++) {
		if (clk_enable(clk) != 0) {
			aprint_error(": couldn't enable clock #%d\n", c);
			// XXXNH cleanup
			return;
		}
	}
	/* de-assert resets */
	struct fdtbus_reset *rst;
	for (u_int r = 0; (rst = fdtbus_reset_get_index(phandle, r)) != NULL; r++) {
		if (fdtbus_reset_deassert(rst) != 0) {
			aprint_error(": couldn't de-assert reset #%d\n", r);
			// XXXNH cleanup
			return;
		}
	}
#endif
	sc->sc_jpd = of_compatible_lookup(phandle, compat_data)->data;

	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_VM);

	aprint_naive("\n");
	aprint_normal(": Pin Controller\n");

	fdtbus_register_gpio_controller(sc->sc_dev, sc->sc_phandle,
	    &jh7110_pinctrl_gpio_funcs);

	for (int child = OF_child(phandle); child; child = OF_peer(child)) {
		fdtbus_register_pinctrl_config(self, child,
		    &jh7110_pinctrl_funcs);
        }
}

CFATTACH_DECL_NEW(jh7110_pinctrl, sizeof(struct jh7110_pinctrl_softc),
	jh7110_pinctrl_match, jh7110_pinctrl_attach, NULL, NULL);
