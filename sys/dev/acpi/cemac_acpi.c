/* $NetBSD$ */

/*-
 * Copyright (c) 2024 Nick Hudson
 * All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>

#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/device.h>
#include <sys/rndsource.h>

#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_ether.h>
#include <net/if_media.h>

#include <dev/mii/miivar.h>

#include <dev/acpi/acpireg.h>
#include <dev/acpi/acpivar.h>
#include <dev/acpi/acpi_intr.h>

#include <dev/cadence/if_cemacvar.h>

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "BCM6E4F" },	/* Cadence Gigabit Ethernet */
	DEVICE_COMPAT_EOL
};


static int
cemac_acpi_match(device_t parent, cfdata_t cf, void *aux)
{
	struct acpi_attach_args *aa = aux;

	return acpi_compatible_match(aa, compat_data);
}


#define RP1_SYSINFO_BASE				0x00000000

#define RP1_CLOCKS_MAIN_BASE				0x00018000

#define RP1_BUSFABRIC_MONITOR_BASE			0x000c0000

#define RP1_SYS_RIO0_BASE				0x000e0000
#define RP1_SYS_RIO1_BASE				0x000e4000
#define RP1_SYS_RIO2_BASE				0x000e8000

#define RP1_ETH_BASE					0x00100000




#define FC0_REF_KHZ			0x0021c
#define FC0_MIN_KHZ			0x00220
#define FC0_MAX_KHZ			0x00224
#define FC0_DELAY			0x00228
#define FC0_INTERVAL			0x0022c
#define FC0_SRC				0x00230
#define FC0_STATUS			0x00234
#define FC0_RESULT			0x00238
#define FC_SIZE				0x20
#define FC_COUNT			8
#define FC_NUM(idx, off)		((idx) * 32 + (off))

#define FC0_STATUS_DONE			__BIT(4)
#define FC0_STATUS_RUNNING		__BIT(8)


#define FC_TIMEOUT_NS			100000000

// 100000000 NS
// 100000 US
// 100 MS

#define clockman_read(sc, off) \
    bus_space_read_4((sc)->sc_iot, (sc)->sc_rp1, RP1_CLOCKS_MAIN_BASE + off)


#define clockman_write(sc, off, val) \
    bus_space_write_4((sc)->sc_iot, (sc)->sc_rp1, RP1_CLOCKS_MAIN_BASE + (off), val)


static unsigned long
clockman_measure_clock(struct cemac_softc *sc, unsigned int fc0_src)
{
	struct cemac_softc *clockman = sc;
	unsigned long fc0_ref_rate = 50000;
	unsigned long result;
//	ktime_t timeout;
	unsigned int fc_idx, fc_offset, fc_src;
	unsigned int i;

	fc_idx = fc0_src / 32;
	fc_src = fc0_src % 32;

	/* fc_src == 0 is invalid. */
	if (!fc_src || fc_idx >= FC_COUNT)
		return 0;

	fc_offset = fc_idx * FC_SIZE;

	/* Ensure the frequency counter is idle. */
//	timeout = ktime_add_ns(ktime_get(), FC_TIMEOUT_NS);
	i = 0;
	while (clockman_read(clockman, fc_offset + FC0_STATUS) & FC0_STATUS_RUNNING) {
		if (i > 100) {
			printf("%s: running timeout\n", __func__);
			return 0;
		}
		delay(1000);
	}

//	spin_lock(&clockman->regs_lock);
	clockman_write(clockman, fc_offset + FC0_REF_KHZ, fc0_ref_rate);
	clockman_write(clockman, fc_offset + FC0_MIN_KHZ, 0);
	clockman_write(clockman, fc_offset + FC0_MAX_KHZ, 0x1ffffff);
	clockman_write(clockman, fc_offset + FC0_INTERVAL, 8);
	clockman_write(clockman, fc_offset + FC0_DELAY, 7);
	clockman_write(clockman, fc_offset + FC0_SRC, fc_src);
//	spin_unlock(&clockman->regs_lock);

	/* Ensure the frequency counter is idle. */
//	timeout = ktime_add_ns(ktime_get(), FC_TIMEOUT_NS);
	i = 0;
	while (!(clockman_read(clockman, fc_offset + FC0_STATUS) & FC0_STATUS_DONE)) {
		if (i > 100) {
			printf("%s: done timeout\n", __func__);
			return 0;
		}
		delay(1000);
	}

	result = clockman_read(clockman, fc_offset + FC0_RESULT);

	/* Disable FC0 */
//	spin_lock(&clockman->regs_lock);
	clockman_write(clockman, fc_offset + FC0_SRC, 0);
//	spin_unlock(&clockman->regs_lock);

	return result;
}




static void
cemac_acpi_attach(device_t parent, device_t self, void *aux)
{
	struct cemac_softc * const sc = device_private(self);
	struct acpi_attach_args *aa = aux;
	ACPI_HANDLE handle = aa->aa_node->ad_handle;
	struct acpi_resources res;
	struct acpi_mem *mem;
	struct acpi_mem *mem2;
	struct acpi_irq *irq;
	char *phy_mode;
	ACPI_STATUS rv;
	int error;
	void *ih;

	sc->sc_dev = self;

	rv = acpi_resource_parse(sc->sc_dev, handle, "_CRS",
	    &res, &acpi_resource_parse_ops_default);
	if (ACPI_FAILURE(rv))
		return;

	mem = acpi_res_mem(&res, 0);
	if (mem == NULL) {
		aprint_error_dev(self, "couldn't find mem resource\n");
		goto done;
	}

	irq = acpi_res_irq(&res, 0);
	if (irq == NULL) {
		aprint_error_dev(self, "couldn't find irq resource\n");
		goto done;
	}

	sc->sc_iot = aa->aa_memt;
	error = bus_space_map(sc->sc_iot, mem->ar_base, mem->ar_length,
	    0, &sc->sc_ioh);
	if (error != 0) {
		aprint_error_dev(self, "couldn't map registers\n");
		goto done;
	}

printf("%s: base  %#018lx\n", __func__, mem->ar_base);
//1f000e0000/1f000ebfff
//1f00100000
//1f00100000
// vs 0x400e0000 in PDF

	mem2 = acpi_res_mem(&res, 1);
	if (mem2 == NULL) {
		aprint_error_dev(self, "couldn't find mem2 resource\n");
	} else {
printf("%s: rp1   %#018lx\n", __func__, mem2->ar_base);
		error = bus_space_map(sc->sc_iot,
		    mem2->ar_base, mem2->ar_length, 0, &sc->sc_rp1);
		if (error) {
			aprint_error_dev(self, "couldn't map GPIO RIO registers\n");
			goto done;
		}
printf("%s: pci   %#018lx\n", __func__, 0x1000120000);
		error = bus_space_map(sc->sc_iot, 0x1000120000, 0x9310, 0, &sc->sc_pci);
		if (error) {
			aprint_error_dev(self, "couldn't map PCI registers\n");
			goto done;
		}

	}

#if 0
	sc->sc_dmat = BUS_DMA_TAG_VALID(aa->aa_dmat64) ?
	    aa->aa_dmat64 : aa->aa_dmat;
#endif
	sc->sc_dmat = aa->aa_dmat;

	// Read PHY number
	sc->sc_phy_id = MII_PHY_ANY;

	rv = acpi_dsd_string(handle, "phy-mode", &phy_mode);
	if (ACPI_FAILURE(rv)) {
		aprint_error_dev(self, "missing 'phy-mode' property\n");
		goto done;
	}
#if 0
	if (strcmp(phy_mode, "rgmii-rxid") == 0)
		sc->sc_phy_mode = GENET_PHY_MODE_RGMII_RXID;
	else if (strcmp(phy_mode, "rgmii-txid") == 0)
		sc->sc_phy_mode = GENET_PHY_MODE_RGMII_TXID;
	else if (strcmp(phy_mode, "rgmii-id") == 0)
		sc->sc_phy_mode = GENET_PHY_MODE_RGMII_ID;
	else if (strcmp(phy_mode, "rgmii") == 0)
		sc->sc_phy_mode = GENET_PHY_MODE_RGMII;
	else {
		aprint_error(": unsupported phy-mode '%s'\n", phy_mode);
		goto done;
	}
#endif


#define CLK_CTRL_ENABLE			__BIT(11)

#define CLK_SYS_CTRL			0x00014
#define CLK_SYS_DIV_INT			0x00018
#define CLK_SYS_SEL			0x00020

#define CLK_ETH_CTRL			0x00064
#define CLK_ETH_DIV_INT			0x00068
#define CLK_ETH_SEL			0x00070

#define CLK_ETH_TSU_CTRL		0x00134
#define CLK_ETH_TSU_DIV_INT		0x00138
#define CLK_ETH_TSU_SEL			0x00140

	uint32_t val;

	val = bus_space_read_4(sc->sc_iot, sc->sc_rp1, RP1_CLOCKS_MAIN_BASE + CLK_ETH_CTRL);
	printf("%s: ETH_CTRL    %#010x before\n", __func__, val);


	val = bus_space_read_4(sc->sc_iot, sc->sc_rp1, RP1_CLOCKS_MAIN_BASE + CLK_ETH_TSU_CTRL);
	printf("%s: ETH_TSU_CTL %#010x before\n", __func__, val);

	bus_space_write_4(sc->sc_iot, sc->sc_rp1, RP1_CLOCKS_MAIN_BASE + CLK_ETH_TSU_CTRL, val | CLK_CTRL_ENABLE);

	printf("%s: ETH_TSU_CTL %#010x after \n", __func__,
	    bus_space_read_4(sc->sc_iot, sc->sc_rp1, RP1_CLOCKS_MAIN_BASE + CLK_ETH_TSU_CTRL));



	val = bus_space_read_4(sc->sc_iot, sc->sc_rp1, RP1_CLOCKS_MAIN_BASE + CLK_SYS_CTRL);
	printf("%s: SYS_CTRL %#010x before\n", __func__, val);

	bus_space_write_4(sc->sc_iot, sc->sc_rp1, RP1_CLOCKS_MAIN_BASE + CLK_SYS_CTRL, val | CLK_CTRL_ENABLE);

	printf("%s: SYS_CTRL %#010x after \n", __func__,
	    bus_space_read_4(sc->sc_iot, sc->sc_rp1, RP1_CLOCKS_MAIN_BASE + CLK_SYS_CTRL));




	unsigned long chz;

	chz = clockman_measure_clock(sc, FC_NUM(4, 6));	// RP1_CLK_ETH
	printf("%s: ETH Hz     %ld\n", __func__, chz);

	chz = clockman_measure_clock(sc, FC_NUM(5, 7));
	printf("%s: ETH_TSU Hz %ld\n", __func__, chz);


#if 0


	.clk_init = macb_clk_init,
	.init = macb_init,




static const struct macb_usrio_config macb_default_usrio = {
	.mii = MACB_BIT(MII),
	.rmii = MACB_BIT(RMII),
	.rgmii = GEM_BIT(RGMII),
	.refclk = MACB_BIT(CLKEN),
};

	[RP1_CLK_ETH] = REGISTER_CLK(
				.name = "clk_eth",
				.parents = {"pll_sys_sec",
					    "pll_sys",
					    "pll_video_sec",
					    "clksrc_gp0",
					    "clksrc_gp1",
					    "clksrc_gp2",
					    "clksrc_gp3",
					    "clksrc_gp4",
					    "clksrc_gp5"},
				.num_std_parents = 0,
				.num_aux_parents = 9,
				.ctrl_reg = CLK_ETH_CTRL,
				.div_int_reg = CLK_ETH_DIV_INT,
				.sel_reg = CLK_ETH_SEL,
				.div_int_max = DIV_INT_8BIT_MAX,
				.max_freq = 125 * MHz,
				.fc0_src = FC_NUM(4, 6),
				),

	[RP1_CLK_ETH_TSU] = REGISTER_CLK(
				.name = "clk_eth_tsu",
				.parents = {"xosc",
					    "pll_video_sec",
					    "clksrc_gp0",
					    "clksrc_gp1",
					    "clksrc_gp2",
					    "clksrc_gp3",
					    "clksrc_gp4",
					    "clksrc_gp5"},
				.num_std_parents = 0,
				.num_aux_parents = 8,
				.ctrl_reg = CLK_ETH_TSU_CTRL,
				.div_int_reg = CLK_ETH_TSU_DIV_INT,
				.sel_reg = CLK_ETH_TSU_SEL,
				.div_int_max = DIV_INT_8BIT_MAX,
				.max_freq = 50 * MHz,
				.fc0_src = FC_NUM(5, 7),
				),

#define RP1_CLK_ETH_TSU			29



		rp1_clocks: clocks@18000 {
			compatible = "raspberrypi,rp1-clocks";
			#clock-cells = <1>;
			reg = <0xc0 0x40018000 0x0 0x10038>;
			clocks = <&clk_xosc>;

			assigned-clocks = <&rp1_clocks RP1_PLL_SYS_CORE>,
					  <&rp1_clocks RP1_PLL_AUDIO_CORE>,
					  // RP1_PLL_VIDEO_CORE and dividers are now managed by VEC,DPI drivers
					  <&rp1_clocks RP1_PLL_SYS>,
					  <&rp1_clocks RP1_PLL_SYS_SEC>,
					  <&rp1_clocks RP1_PLL_AUDIO>,
					  <&rp1_clocks RP1_PLL_AUDIO_SEC>,
					  <&rp1_clocks RP1_CLK_SYS>,
					  <&rp1_clocks RP1_PLL_SYS_PRI_PH>,
					  // RP1_CLK_SLOW_SYS is used for the frequency counter (FC0)
					  <&rp1_clocks RP1_CLK_SLOW_SYS>,
					  <&rp1_clocks RP1_CLK_SDIO_TIMER>,
					  <&rp1_clocks RP1_CLK_SDIO_ALT_SRC>,
					  <&rp1_clocks RP1_CLK_ETH_TSU>;

			assigned-clock-rates = <1000000000>, // RP1_PLL_SYS_CORE
					       <1536000000>, // RP1_PLL_AUDIO_CORE
					       <200000000>,  // RP1_PLL_SYS
					       <125000000>,  // RP1_PLL_SYS_SEC
					       <61440000>,   // RP1_PLL_AUDIO
					       <192000000>,  // RP1_PLL_AUDIO_SEC
					       <200000000>,  // RP1_CLK_SYS
					       <100000000>,  // RP1_PLL_SYS_PRI_PH
					       // Must match the XOSC frequency
					       <50000000>, // RP1_CLK_SLOW_SYS
					       <1000000>, // RP1_CLK_SDIO_TIMER
					       <200000000>, // RP1_CLK_SDIO_ALT_SRC
					       <50000000>; // RP1_CLK_ETH_TSU
		};


	macb_pclk: macb_pclk {
		compatible = "fixed-clock";
		#clock-cells = <0>;
		clock-output-names = "pclk";
		clock-frequency = <200000000>;
	};
	macb_hclk: macb_hclk {
		compatible = "fixed-clock";
		#clock-cells = <0>;
		clock-output-names = "hclk";
		clock-frequency = <200000000>;
	};

&rp1_gpio {
	gpio-line-names =
...
	    ???"ETH_RST_N", // GPIO32
...
}

&rp1_eth {
	status = "okay";
	phy-handle = <&phy1>;
	phy-reset-gpios = <&rp1_gpio 32 GPIO_ACTIVE_LOW>;
	phy-reset-duration = <5>;

	phy1: ethernet-phy@1 {
		reg = <0x1>;
		brcm,powerdown-enable;
	};
};



include/dt-bindings/clock/rp1.h:#define RP1_CLK_ETH_TSU                 29


                                               <50000000>; // RP1_CLK_ETH_TSU





		rp1_eth: ethernet@100000 {
			reg = <0xc0 0x40100000  0x0 0x4000>;
			compatible = "cdns,macb";
			#address-cells = <1>;
			#size-cells = <0>;
			interrupts = <RP1_INT_ETH IRQ_TYPE_LEVEL_HIGH>;
			clocks = <&macb_pclk &macb_hclk &rp1_clocks RP1_CLK_ETH_TSU>;
			clock-names = "pclk", "hclk", "tsu_clk";
			phy-mode = "rgmii-id";
			cdns,aw2w-max-pipe = /bits/ 8 <8>;
			cdns,ar2r-max-pipe = /bits/ 8 <8>;
			cdns,use-aw2b-fill;
			local-mac-address = [00 00 00 00 00 00];
			status = "disabled";
		};

#endif



#if 0

static const struct clk_ops rp1_clk_ops = {
	.is_prepared = rp1_clock_is_on,
	.prepare = rp1_clock_on,
	.unprepare = rp1_clock_off,
	.recalc_rate = rp1_clock_recalc_rate,
	.get_parent = rp1_clock_get_parent,
	.set_parent = rp1_clock_set_parent,
	.set_rate_and_parent = rp1_clock_set_rate_and_parent,
	.set_rate = rp1_clock_set_rate,
	.determine_rate = rp1_clock_determine_rate,
	.debug_init = rp1_clk_debug_init,
};
#endif


#define CLK_ETH_TSU_CTRL		0x00134
#define CLK_ETH_TSU_DIV_INT		0x00138
#define CLK_ETH_TSU_SEL			0x00140


	aprint_normal("%s", device_xname(self));

        ih = acpi_intr_establish(self, (uint64_t)(uintptr_t)handle, IPL_NET,
	    true, cemac_intr, sc, device_xname(self));
	if (ih == NULL) {
		aprint_error_dev(self, "couldn't establish interrupt\n");
		goto done;
	}

	sc->sc_phyno = 1;
	sc->cemac_flags = CEMAC_FLAG_GEM;
	cemac_attach_common(sc);

//        aprint_normal_dev(self, "interrupting on %s\n", intrstr);

done:
	acpi_resource_cleanup(&res);
}

CFATTACH_DECL_NEW(cemac_acpi, sizeof(struct cemac_softc),
    cemac_acpi_match, cemac_acpi_attach, NULL, NULL);
