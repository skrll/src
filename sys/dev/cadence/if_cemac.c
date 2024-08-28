/*	$NetBSD: if_cemac.c,v 1.46 2025/10/12 23:30:13 thorpej Exp $	*/

/*
 * Copyright (c) 2015  Genetec Corporation.  All rights reserved.
 * Written by Hashimoto Kenichi for Genetec Corporation.
 *
 * Based on arch/arm/at91/at91emac.c
 *
 * Copyright (c) 2007 Embedtronics Oy
 * All rights reserved.
 *
 * Copyright (c) 2004 Jesse Off
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
 * Cadence EMAC/GEM ethernet controller IP driver
 * used by arm/at91, arm/zynq SoC
 */

/*
 * Lock order:
 *
 *	IFNET_LOCK -> sc_mcast_lock
 *	IFNET_LOCK -> sc_intr_lock
 */


#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: if_cemac.c,v 1.46 2025/10/12 23:30:13 thorpej Exp $");

#include <sys/param.h>
#include <sys/types.h>

#include <sys/bus.h>
#include <sys/device.h>
#include <sys/kernel.h>
#include <sys/proc.h>
#include <sys/systm.h>
#include <sys/time.h>

#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_types.h>
#include <net/if_media.h>
#include <net/if_ether.h>
#include <net/bpf.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#ifdef INET
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/ip.h>
#include <netinet/if_inarp.h>
#endif

#include <dev/cadence/cemacreg.h>
#include <dev/cadence/if_cemacvar.h>

#ifndef CEMAC_WATCHDOG_TIMEOUT
#define CEMAC_WATCHDOG_TIMEOUT 5
#endif
static int cemac_watchdog_timeout = CEMAC_WATCHDOG_TIMEOUT;

#define DEFAULT_MDCDIV	32


#if 0

static const struct macb_config default_gem_config = {
	.caps = MACB_CAPS_GIGABIT_MODE_AVAILABLE |
		MACB_CAPS_JUMBO |
		MACB_CAPS_GEM_HAS_PTP,
	.dma_burst_length = 16,
	.clk_init = macb_clk_init,
	.init = macb_init,
	.usrio = &macb_default_usrio,
	.jumbo_max_len = 10240,
};

static const struct macb_usrio_config macb_default_usrio = {
	.mii = MACB_BIT(MII),
	.rmii = MACB_BIT(RMII),
	.rgmii = GEM_BIT(RGMII),
	.refclk = MACB_BIT(CLKEN),
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
#endif




static uint32_t
cemac_rd4(const char *fn, unsigned ln, struct cemac_softc *sc,
    bus_size_t off)
{
	uint32_t val = bus_space_read_4(sc->sc_iot, sc->sc_ioh, off);
	printf("%s:%u: reg %#06lx val %#010x (read)\n", fn, ln, off, val);

	return val;
}

static void
cemac_wr4(const char *fn, unsigned ln, struct cemac_softc *sc,
    bus_size_t off, uint32_t val)
{
	printf("%s:%u: reg %#06lx val %#010x (write)\n", fn, ln, off, val);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, off, val);
}

#define CEMAC_READ(x) \
	cemac_rd4(__func__, __LINE__, sc, (x))

#define CEMAC_WRITE(x, y) \
	cemac_wr4(__func__, __LINE__, sc, (x), (y))

#if 1
#define CEMAC_GEM_WRITE(x, y)						      \
    do {								      \
	if (ISSET(sc->cemac_flags, CEMAC_FLAG_GEM)) {			      \
		printf("%s: reg %#06x val %#010x (gem)\n", __func__, (GEM_##x), (y)); \
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, (GEM_##x), (y));    \
	} else {								      \
		printf("%s: reg %#06x val %#010x (eth)\n", __func__, (ETH_##x), (y)); \
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, (ETH_##x), (y));    \
	} \
    } while(0)
#else
#define CEMAC_GEM_WRITE(x, y)						      \
    do {								      \
	if (ISSET(sc->cemac_flags, CEMAC_FLAG_GEM)) {			      \
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, (GEM_##x), (y));    \
	} else {							      \
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, (ETH_##x), (y));    \
	} \
    } while(0)
#endif

static void	cemac_init(struct cemac_softc *);
static int	cemac_gctx(struct cemac_softc *);
static int	cemac_mediachange(struct ifnet *);
static void	cemac_mediastatus(struct ifnet *, struct ifmediareq *);
static int	cemac_mii_readreg(device_t, int, int, uint16_t *);
static int	cemac_mii_writereg(device_t, int, int, uint16_t);
static void	cemac_statchg(struct ifnet *);
static void	cemac_tick(void *);
static int	cemac_ifioctl(struct ifnet *, u_long, void *);
static void	cemac_ifstart(struct ifnet *);
static void	cemac_ifstart_locked(struct ifnet *);
static void	cemac_ifwatchdog(struct ifnet *);
static int	cemac_ifinit(struct ifnet *);
static void	cemac_ifstop(struct ifnet *, int);
static void	cemac_setaddr(struct ifnet *);

//#define	CEMAC_DEBUG 10
#undef CEMAC_DEBUG
#ifdef	CEMAC_DEBUG
int cemac_debug = CEMAC_DEBUG;
#define	DPRINTFN(n, fmt)	if (cemac_debug >= (n)) printf fmt
#else
#define	DPRINTFN(n, fmt)
#endif

/*
 * Perform an interface watchdog reset.
 */
static void
cemac_handle_reset_work(struct work *work, void *arg)
{
	struct cemac_softc * const sc = arg;
	struct ifnet * const ifp = &sc->sc_ethercom.ec_if;

	printf("%s: watchdog timeout -- resetting\n", ifp->if_xname);

	/* Don't want ioctl operations to happen */
	IFNET_LOCK(ifp);

	/* reset the interface. */
	cemac_ifinit(ifp);

	IFNET_UNLOCK(ifp);

	/*
	 * There are still some upper layer processing which call
	 * ifp->if_start(). e.g. ALTQ or one CPU system
	 */
	/* Try to get more packets going. */
	ifp->if_start(ifp);

	atomic_store_relaxed(&sc->sc_reset_pending, 0);
}


void
cemac_attach_common(struct cemac_softc *sc)
{
	aprint_naive("\n");
	uint32_t mid = CEMAC_READ(GEM_MID);
	uint16_t mid_id = __SHIFTOUT(mid, __BITS(31, 16));
	uint16_t mid_rev = __SHIFTOUT(mid, __BITS(15, 0));
	if (mid_id >= 0x2) {
		sc->cemac_flags |= CEMAC_FLAG_GEM;

		// Read DCFG6 for number of queues
	} else {
		// 1 queue
	}


	// XXXNH clock enable needs to be done by earlier code?

	if (ISSET(sc->cemac_flags, CEMAC_FLAG_GEM)) {
		char buf[128];

		aprint_normal(": Cadence Gigabit Ethernet Controller %x/%x\n",
		    mid_id, mid_rev);

		snprintb(buf, sizeof(buf), GEM_DCFG1_BITS, CEMAC_READ(GEM_DCFG1));
		aprint_normal_dev(sc->sc_dev, "DCFG1 %s\n", buf);
		snprintb(buf, sizeof(buf), GEM_DCFG2_BITS, CEMAC_READ(GEM_DCFG2));
		aprint_normal_dev(sc->sc_dev, "DCFG2 %s\n", buf);
		snprintb(buf, sizeof(buf), GEM_DCFG3_BITS, CEMAC_READ(GEM_DCFG3));
		aprint_normal_dev(sc->sc_dev, "DCFG3 %s\n", buf);
		snprintb(buf, sizeof(buf), GEM_DCFG4_BITS, CEMAC_READ(GEM_DCFG4));
		aprint_normal_dev(sc->sc_dev, "DCFG4 %s\n", buf);
		snprintb(buf, sizeof(buf), GEM_DCFG5_BITS, CEMAC_READ(GEM_DCFG5));
		aprint_normal_dev(sc->sc_dev, "DCFG5 %s\n", buf);
	} else
		aprint_normal(": Cadence Ethernet Controller %x/%x\n",
		    mid_id, mid_rev);

#if 0
	// XXXNH not yet?
	/* configure emac: */
	CEMAC_WRITE(ETH_CTL, 0);		// disable everything
	CEMAC_WRITE(ETH_IDR, -1);		// disable interrupts
	CEMAC_WRITE(ETH_RBQP, 0);		// clear receive
	CEMAC_WRITE(ETH_TBQP, 0);		// clear transmit
#endif
	// ETH_CFG_{SPD,FD,GEN} should go in mii
	if (ISSET(sc->cemac_flags, CEMAC_FLAG_GEM)) {
		// gem_mdc_clk_div
		if (1) {
			CEMAC_WRITE(ETH_CFG,
			    GEM_CFG_CLK_96 | GEM_CFG_DBW_128);
		} else {
			CEMAC_WRITE(ETH_CFG,
			    GEM_CFG_CLK_64 | GEM_CFG_GEN | ETH_CFG_SPD | ETH_CFG_FD);
		}
	} else {
		CEMAC_WRITE(ETH_CFG,
		    ETH_CFG_CLK_32 | ETH_CFG_SPD | ETH_CFG_FD | ETH_CFG_BIG);
	}

#if 0
		// XXXNH not yet?

	//CEMAC_WRITE(ETH_TCR, 0);		// send nothing
	//(void)CEMAC_READ(ETH_ISR);
	uint32_t u;

	u = CEMAC_READ(ETH_TSR);
	CEMAC_WRITE(ETH_TSR, (u & (ETH_TSR_UND | ETH_TSR_COMP | ETH_TSR_BNQ
				  | ETH_TSR_IDLE | ETH_TSR_RLE
				  | ETH_TSR_COL | ETH_TSR_OVR)));
	u = CEMAC_READ(ETH_RSR);
	CEMAC_WRITE(ETH_RSR, (u & (ETH_RSR_OVR | ETH_RSR_REC | ETH_RSR_BNA)));
#endif

	CEMAC_WRITE(ETH_CTL, ETH_CTL_MPE);	// Management port enable

	/* Fetch the Ethernet address from property if set. */
	if (! ether_getaddr(sc->sc_dev, sc->sc_enaddr)) {
		static const uint8_t hardcoded[ETHER_ADDR_LEN] = {
			0x00, 0x0d, 0x10, 0x81, 0x0c, 0x94
		};
		memcpy(sc->sc_enaddr, hardcoded, ETHER_ADDR_LEN);
	}

	cemac_init(sc);
}

static int
cemac_gctx(struct cemac_softc *sc)
{
	uint32_t tsr;

	tsr = CEMAC_READ(ETH_TSR);
	if (!ISSET(sc->cemac_flags, CEMAC_FLAG_GEM)) {
		// no space left
		if (!(tsr & ETH_TSR_BNQ))
			return 0;
	} else {
		if (tsr & GEM_TSR_TXGO)
			return 0;
	}
	CEMAC_WRITE(ETH_TSR, tsr);

	// free sent frames
	while (sc->txqc > (ISSET(sc->cemac_flags, CEMAC_FLAG_GEM) ? 0 :
		(tsr & ETH_TSR_IDLE ? 0 : 1))) {
		int bi = sc->txqi % TX_QLEN;

		DPRINTFN(3,("%s: TDSC[%i].Addr 0x%08x\n",
			__FUNCTION__, bi, sc->TDSC[bi].Addr));
		DPRINTFN(3,("%s: TDSC[%i].Info 0x%08x\n",
			__FUNCTION__, bi, sc->TDSC[bi].Info));

		bus_dmamap_sync(sc->sc_dmat, sc->txq[bi].m_dmamap, 0,
		    sc->txq[bi].m->m_pkthdr.len, BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->sc_dmat, sc->txq[bi].m_dmamap);
		m_freem(sc->txq[bi].m);
		DPRINTFN(2,("%s: freed idx #%i mbuf %p (txqc=%i)\n",
		    __FUNCTION__, bi, sc->txq[bi].m, sc->txqc));
		sc->txq[bi].m = NULL;
		sc->txqi = (bi + 1) % TX_QLEN;
		sc->txqc--;
	}

	// mark we're free
	if (sc->sc_txbusy) {
		sc->sc_txbusy = false;
		/* Disable transmit-buffer-free interrupt */
		/*CEMAC_WRITE(ETH_IDR, ETH_ISR_TBRE);*/
	}

	return 1;
}

int
cemac_intr(void *arg)
{
	struct cemac_softc * const sc = arg;
	struct ifnet * const ifp = &sc->sc_ethercom.ec_if;
	uint32_t imr, isr, ctl;
#ifdef	CEMAC_DEBUG
	uint32_t rsr;
#endif
	int bi;

	mutex_enter(sc->sc_intr_lock);
	if (sc->sc_stopping) {
		mutex_exit(sc->sc_intr_lock);
		return 0;
	}

	imr = ~CEMAC_READ(ETH_IMR);
	if (!(imr & (ETH_ISR_RCOM | ETH_ISR_TBRE | ETH_ISR_TIDLE |
	    ETH_ISR_RBNA | ETH_ISR_ROVR | ETH_ISR_TCOM))) {
		// interrupt not enabled, can't be us
		mutex_exit(sc->sc_intr_lock);
		return 0;
	}

	isr = CEMAC_READ(ETH_ISR);
	CEMAC_WRITE(ETH_ISR, isr);
	isr &= imr;

	if (isr == 0) {
		mutex_exit(sc->sc_intr_lock);
		return 0;
	}

#ifdef	CEMAC_DEBUG
	rsr = CEMAC_READ(ETH_RSR);		// get receive status register
#endif
	DPRINTFN(2, ("%s: isr=0x%08X rsr=0x%08X imr=0x%08X\n", __FUNCTION__,
	    isr, rsr, imr));

	net_stat_ref_t nsr = IF_STAT_GETREF(ifp);
	// out of receive buffers
	if (isr & ETH_ISR_RBNA) {
		// clear interrupt
		CEMAC_WRITE(ETH_RSR, ETH_RSR_BNA);

		ctl = CEMAC_READ(ETH_CTL);
		// disable receiver
		CEMAC_WRITE(ETH_CTL, ctl & ~ETH_CTL_RE);
		// clear BNA bit
		CEMAC_WRITE(ETH_RSR, ETH_RSR_BNA);
		// re-enable receiver
		CEMAC_WRITE(ETH_CTL, ctl |  ETH_CTL_RE);

		if_statinc_ref(ifp, nsr, if_ierrors);
		if_statinc_ref(ifp, nsr, if_ipackets);
		DPRINTFN(1,("%s: out of receive buffers\n", __FUNCTION__));
	}
	if (isr & ETH_ISR_ROVR) {
		// clear interrupt
		CEMAC_WRITE(ETH_RSR, ETH_RSR_OVR);
		if_statinc_ref(ifp, nsr, if_ierrors);
		if_statinc_ref(ifp, nsr, if_ipackets);
		DPRINTFN(1,("%s: receive overrun\n", __FUNCTION__));
	}

	// packet has been received!
	if (isr & ETH_ISR_RCOM) {
		uint32_t nfo;
		DPRINTFN(2,("#2 RDSC[%i].INFO=0x%08X\n", sc->rxqi % RX_QLEN,
		    sc->RDSC[sc->rxqi % RX_QLEN].Info));
		while (sc->RDSC[(bi = sc->rxqi % RX_QLEN)].Addr & ETH_RDSC_F_USED) {
			int fl, csum;
			struct mbuf *m;

			nfo = sc->RDSC[bi].Info;
			fl = (nfo & ETH_RDSC_I_LEN) - 4;
			DPRINTFN(2,("## nfo=0x%08X\n", nfo));

			MGETHDR(m, M_DONTWAIT, MT_DATA);
			if (m != NULL)
				MCLGET(m, M_DONTWAIT);
			if (m != NULL && (m->m_flags & M_EXT)) {
				bus_dmamap_sync(sc->sc_dmat,
				    sc->rxq[bi].m_dmamap, 0, MCLBYTES,
				    BUS_DMASYNC_POSTREAD);
				bus_dmamap_unload(sc->sc_dmat,
				    sc->rxq[bi].m_dmamap);
				m_set_rcvif(sc->rxq[bi].m, ifp);
				sc->rxq[bi].m->m_pkthdr.len =
				    sc->rxq[bi].m->m_len = fl;
				switch (nfo & ETH_RDSC_I_CHKSUM) {
				case ETH_RDSC_I_CHKSUM_IP:
					csum = M_CSUM_IPv4;
					break;
				case ETH_RDSC_I_CHKSUM_UDP:
					csum = M_CSUM_IPv4 | M_CSUM_UDPv4 |
					    M_CSUM_UDPv6;
					break;
				case ETH_RDSC_I_CHKSUM_TCP:
					csum = M_CSUM_IPv4 | M_CSUM_TCPv4 |
					    M_CSUM_TCPv6;
					break;
				default:
					csum = 0;
					break;
				}
				sc->rxq[bi].m->m_pkthdr.csum_flags = csum;
				DPRINTFN(2,("received %u bytes packet\n", fl));
				if_percpuq_enqueue(ifp->if_percpuq,
						   sc->rxq[bi].m);
				if (mtod(m, intptr_t) & 3)
					m_adj(m, mtod(m, intptr_t) & 3);
				sc->rxq[bi].m = m;
				bus_dmamap_load(sc->sc_dmat,
				    sc->rxq[bi].m_dmamap, m->m_ext.ext_buf,
					MCLBYTES, NULL, BUS_DMA_NOWAIT);
				bus_dmamap_sync(sc->sc_dmat,
				    sc->rxq[bi].m_dmamap, 0, MCLBYTES,
				    BUS_DMASYNC_PREREAD);
				sc->RDSC[bi].Info = 0;
				sc->RDSC[bi].Addr =
				    sc->rxq[bi].m_dmamap->dm_segs[0].ds_addr
				    | (bi == (RX_QLEN-1) ? ETH_RDSC_F_WRAP : 0);
			} else {
				/* Drop packets until we can get replacement
				 * empty mbufs for the RXDQ.
				 */
				m_freem(m);
				if_statinc_ref(ifp, nsr, if_ierrors);
			}
			sc->rxqi++;
		}
	}

	IF_STAT_PUTREF(ifp);

	if (cemac_gctx(sc) > 0)
		if_schedule_deferred_start(ifp);
#if 0 // reloop
	irq = CEMAC_READ(IntStsC);
	if ((irq & (IntSts_RxSQ | IntSts_ECI)) != 0)
		goto begin;
#endif
	mutex_exit(sc->sc_intr_lock);

	return 1;
}


static int
cemac_ifflags_cb(struct ethercom *ec)
{
	struct ifnet * const ifp = &ec->ec_if;
	struct cemac_softc * const sc = ifp->if_softc;
	int ret = 0;

	KASSERT(IFNET_LOCKED(ifp));
	mutex_enter(sc->sc_mcast_lock);

	u_short change = ifp->if_flags ^ sc->sc_if_flags;
	sc->sc_if_flags = ifp->if_flags;

	if ((change & ~(IFF_CANTCHANGE | IFF_DEBUG)) != 0) {
		ret = ENETRESET;
	} else if ((change & IFF_PROMISC) != 0) {
		if ((sc->sc_if_flags & IFF_RUNNING) != 0)
			cemac_setaddr(ifp);
	}
	mutex_exit(sc->sc_mcast_lock);

	return ret;
}




#if 0
static u32 gem_mdc_clk_div(struct macb *bp)
{
	u32 config;
	unsigned long pclk_hz = clk_get_rate(bp->pclk);

	if (pclk_hz <= 20000000)
		config = GEM_BF(CLK, GEM_CLK_DIV8);
	else if (pclk_hz <= 40000000)
		config = GEM_BF(CLK, GEM_CLK_DIV16);
	else if (pclk_hz <= 80000000)
		config = GEM_BF(CLK, GEM_CLK_DIV32);
	else if (pclk_hz <= 120000000)
		config = GEM_BF(CLK, GEM_CLK_DIV48);
	else if (pclk_hz <= 160000000)
		config = GEM_BF(CLK, GEM_CLK_DIV64);
//                          200000000
	else if (pclk_hz <= 240000000)
		config = GEM_BF(CLK, GEM_CLK_DIV96);
	else if (pclk_hz <= 320000000)
		config = GEM_BF(CLK, GEM_CLK_DIV128);
	else
		config = GEM_BF(CLK, GEM_CLK_DIV224);

	return config;
}

static u32 macb_mdc_clk_div(struct macb *bp)
{
        u32 config;
        unsigned long pclk_hz;

        if (macb_is_gem(bp))
                return gem_mdc_clk_div(bp);

	// XXXNH We're GEM!!!

        pclk_hz = clk_get_rate(bp->pclk);
        if (pclk_hz      <= 20000000)
                config = MACB_BF(CLK, MACB_CLK_DIV8);
        else if (pclk_hz <= 40000000)
                config = MACB_BF(CLK, MACB_CLK_DIV16);
        else if (pclk_hz <= 80000000)
                config = MACB_BF(CLK, MACB_CLK_DIV32);
        else
                config = MACB_BF(CLK, MACB_CLK_DIV64);

        return config;
}
#endif


#define RP1_SYS_RIO0_BASE				0x000e0000
#define RP1_SYS_RIO1_BASE				0x000e4000
#define RP1_SYS_RIO2_BASE				0x000e8000


#define RP1_RIO_OUT 0x00
#define RP1_SET_OFFSET 0x2000
#define RP1_CLR_OFFSET 0x3000




static void
rp1_pin_write(struct cemac_softc *sc, bus_size_t offset, uint32_t val)
{

	mutex_enter(sc->sc_intr_lock);

        /*
         * Issuing 6 pipelined writes to the RC's Slot Control register will stall the
         * peripheral bus inside 2712 if the link is in L1. This acts as a lightweight
         * "fence" operation preventing back-to-back writes arriving at RP1 on a wake.
         */
	bus_space_write_4(sc->sc_iot, sc->sc_pci, 0xc0, 0); // 1
	bus_space_write_4(sc->sc_iot, sc->sc_pci, 0xc0, 0); // 2
	bus_space_write_4(sc->sc_iot, sc->sc_pci, 0xc0, 0); // 3
	bus_space_write_4(sc->sc_iot, sc->sc_pci, 0xc0, 0); // 4
	bus_space_write_4(sc->sc_iot, sc->sc_pci, 0xc0, 0); // 5
	bus_space_write_4(sc->sc_iot, sc->sc_pci, 0xc0, 0); // 6

	bus_space_write_4(sc->sc_iot, sc->sc_rp1, offset, val);

	mutex_exit(sc->sc_intr_lock);
}





static void
cemac_init(struct cemac_softc *sc)
{
	bus_dma_segment_t segs;
	int rsegs, err, i;
	struct ifnet * const ifp = &sc->sc_ethercom.ec_if;
	struct mii_data * const mii = &sc->sc_mii;
	uint32_t u;
#if 0
	int mdcdiv = DEFAULT_MDCDIV;
#endif
	callout_init(&sc->cemac_tick_ch, CALLOUT_MPSAFE);
	callout_setfunc(&sc->cemac_tick_ch, cemac_tick, sc);

	sc->sc_intr_lock = mutex_obj_alloc(MUTEX_DEFAULT, IPL_NET);


//   pin->rio + RP1_RIO_OUT + (value ? RP1_SET_OFFSET : RP1_CLR_OFFSET));

	rp1_pin_write(sc,
	    RP1_SYS_RIO0_BASE +			/* rio base */
	    0x4000 +				/* bank 1 rio_offset */
	    RP1_RIO_OUT +
	    RP1_SET_OFFSET,
	    __BIT(32 - 28));

	{

	uint16_t id1;
	uint16_t id2;
	cemac_mii_readreg(sc->sc_dev, 1, MII_PHYIDR1, &id1);
	cemac_mii_readreg(sc->sc_dev, 1, MII_PHYIDR2, &id2);

	KASSERTMSG(id1 == 0x600d && id2 == 0x84a2, "id1/id2 %#06x/%#06x", id1, id2);

	}



#if 0


	.clk_init = macb_clk_init,
	.init = macb_init,



static const struct macb_usrio_config macb_default_usrio = {
	.mii = MACB_BIT(MII),
	.rmii = MACB_BIT(RMII),
	.rgmii = GEM_BIT(RGMII),
	.refclk = MACB_BIT(CLKEN),
};





#endif

	// macb_init...
#if 0
	if (!(bp->caps & MACB_CAPS_USRIO_DISABLED)) {
		val = 0;
		if (phy_interface_mode_is_rgmii(bp->phy_interface))
			val = bp->usrio->rgmii;
		else if (bp->phy_interface == PHY_INTERFACE_MODE_RMII &&
			 (bp->caps & MACB_CAPS_USRIO_DEFAULT_IS_MII_GMII))
			val = bp->usrio->rmii;
		else if (!(bp->caps & MACB_CAPS_USRIO_DEFAULT_IS_MII_GMII))
			val = bp->usrio->mii;

		if (bp->caps & MACB_CAPS_USRIO_HAS_CLKEN)
			val |= bp->usrio->refclk;

		macb_or_gem_writel(bp, USRIO, val);
	}

	/* Set MII management clock divider */
                config = MACB_BF(CLK, MACB_CLK_DIV64);

	val = macb_mdc_clk_div(bp);
	val |= macb_dbw(bp);
	if (bp->phy_interface == PHY_INTERFACE_MODE_SGMII)
		val |= GEM_BIT(SGMIIEN) | GEM_BIT(PCSSEL);
	macb_writel(bp, NCFGR, val);
#endif





	// macb_mii_init


//	macb_configure_caps

	// ok...


	CEMAC_WRITE(ETH_CTL, 0);	// disable everything
	CEMAC_WRITE(ETH_IDR, -1);	// disable interrupts
	CEMAC_WRITE(ETH_RBQP, 0);	// clear receive
	CEMAC_WRITE(ETH_TBQP, 0);	// clear transmit

	// ETH_CFG_{SPD,FD,GEN} should go in mii
	if (ISSET(sc->cemac_flags, CEMAC_FLAG_GEM)) {
		// gem_mdc_clk_div
		// macb_dbw
		if (1) {
			// pclk_hz is 200000000 so gem_mdc_clk_div selects 96
			CEMAC_WRITE(ETH_CFG,
			    GEM_CFG_CLK_96 | GEM_CFG_DBW_128);
		} else {
			CEMAC_WRITE(ETH_CFG,
			    GEM_CFG_CLK_64 | GEM_CFG_GEN | ETH_CFG_SPD | ETH_CFG_FD);
		}
	} else {
		CEMAC_WRITE(ETH_CFG,
		    ETH_CFG_CLK_32 | ETH_CFG_SPD | ETH_CFG_FD | ETH_CFG_BIG);
	}

	printf("%s: ETH_CFG %#010x\n", __func__, CEMAC_READ(ETH_CFG));
	if (ISSET(sc->cemac_flags, CEMAC_FLAG_GEM)) {
		CEMAC_WRITE(GEM_USER_IO,
		    __BIT(0) /* GEN_RGMII */
		);
	}

	if (ISSET(sc->cemac_flags, CEMAC_FLAG_GEM)) {
		CEMAC_WRITE(GEM_DMA_CFG,
		    __SHIFTIN((MCLBYTES + 63) / 64, GEM_DMA_CFG_RX_BUF_SIZE) |
		    __SHIFTIN(3, GEM_DMA_CFG_RX_PKTBUF_MEMSZ_SEL) |
		    GEM_DMA_CFG_TX_PKTBUF_MEMSZ_SEL |
		    __SHIFTIN(16, GEM_DMA_CFG_AHB_FIXED_BURST_LEN) |
		    GEM_DMA_CFG_DISC_WHEN_NO_AHB);
	}
//	CEMAC_WRITE(ETH_TCR, 0);			// send nothing
//	(void)CEMAC_READ(ETH_ISR);
	u = CEMAC_READ(ETH_TSR);
	CEMAC_WRITE(ETH_TSR, (u & (ETH_TSR_UND | ETH_TSR_COMP | ETH_TSR_BNQ
		    | ETH_TSR_IDLE | ETH_TSR_RLE
		    | ETH_TSR_COL | ETH_TSR_OVR)));
	u = CEMAC_READ(ETH_RSR);
	CEMAC_WRITE(ETH_RSR, (u & (ETH_RSR_OVR | ETH_RSR_REC | ETH_RSR_BNA)));

#if 0
	if (device_cfdata(sc->sc_dev)->cf_flags)
		mdcdiv = device_cfdata(sc->sc_dev)->cf_flags;
#endif
	/* set ethernet address */
	CEMAC_GEM_WRITE(SA1L, (sc->sc_enaddr[3] << 24)
	    | (sc->sc_enaddr[2] << 16) | (sc->sc_enaddr[1] << 8)
	    | (sc->sc_enaddr[0]));
	CEMAC_GEM_WRITE(SA1H, (sc->sc_enaddr[5] << 8)
	    | (sc->sc_enaddr[4]));
	CEMAC_GEM_WRITE(SA2L, 0);
	CEMAC_GEM_WRITE(SA2H, 0);
	CEMAC_GEM_WRITE(SA3L, 0);
	CEMAC_GEM_WRITE(SA3H, 0);
	CEMAC_GEM_WRITE(SA4L, 0);
	CEMAC_GEM_WRITE(SA4H, 0);

	CEMAC_WRITE(ETH_CTL, ETH_CTL_MPE);      // Management port enable

	char wqname[MAXCOMLEN];
	snprintf(wqname, sizeof(wqname), "%sReset", device_xname(sc->sc_dev));
	int error = workqueue_create(&sc->sc_reset_wq, wqname,
	    cemac_handle_reset_work, sc, PRI_NONE, IPL_SOFTCLOCK,
	    WQ_MPSAFE);
	if (error) {
		aprint_error_dev(sc->sc_dev,
		    "unable to create reset workqueue\n");
		return;
	}

	/* Allocate memory for receive queue descriptors */
	sc->rbqlen = roundup(ETH_DSC_SIZE * (RX_QLEN + 1) * 2, PAGE_SIZE);
	DPRINTFN(1,("%s: rbqlen=%i\n", __FUNCTION__, sc->rbqlen));

	// see EMAC errata why forced to 16384 byte boundary
	err = bus_dmamem_alloc(sc->sc_dmat, sc->rbqlen, 0,
	    MAX(16384, PAGE_SIZE), &segs, 1, &rsegs, BUS_DMA_WAITOK);
	if (err == 0) {
		DPRINTFN(1,("%s: -> bus_dmamem_map\n", __FUNCTION__));
		err = bus_dmamem_map(sc->sc_dmat, &segs, 1, sc->rbqlen,
		    &sc->rbqpage, (BUS_DMA_WAITOK | BUS_DMA_COHERENT));
	}
	if (err == 0) {
		DPRINTFN(1,("%s: -> bus_dmamap_create\n", __FUNCTION__));
		err = bus_dmamap_create(sc->sc_dmat, sc->rbqlen, 1,
		    sc->rbqlen, MAX(16384, PAGE_SIZE), BUS_DMA_WAITOK,
		    &sc->rbqpage_dmamap);
	}
	if (err == 0) {
		DPRINTFN(1,("%s: -> bus_dmamap_load\n", __FUNCTION__));
		err = bus_dmamap_load(sc->sc_dmat, sc->rbqpage_dmamap,
		    sc->rbqpage, sc->rbqlen, NULL, BUS_DMA_WAITOK);
	}
	if (err != 0)
		panic("%s: Cannot get DMA memory", device_xname(sc->sc_dev));

	sc->rbqpage_dsaddr = sc->rbqpage_dmamap->dm_segs[0].ds_addr;
	memset(sc->rbqpage, 0, sc->rbqlen);

	/* Allocate memory for transmit queue descriptors */
	sc->tbqlen = roundup(ETH_DSC_SIZE * (TX_QLEN + 1) * 2, PAGE_SIZE);
	DPRINTFN(1,("%s: tbqlen=%i\n", __FUNCTION__, sc->tbqlen));

	// see EMAC errata why forced to 16384 byte boundary
	err = bus_dmamem_alloc(sc->sc_dmat, sc->tbqlen, 0,
	    MAX(16384, PAGE_SIZE), &segs, 1, &rsegs, BUS_DMA_WAITOK);
	if (err == 0) {
		DPRINTFN(1,("%s: -> bus_dmamem_map\n", __FUNCTION__));
		err = bus_dmamem_map(sc->sc_dmat, &segs, 1, sc->tbqlen,
		    &sc->tbqpage, (BUS_DMA_WAITOK | BUS_DMA_COHERENT));
	}
	if (err == 0) {
		DPRINTFN(1,("%s: -> bus_dmamap_create\n", __FUNCTION__));
		err = bus_dmamap_create(sc->sc_dmat, sc->tbqlen, 1,
		    sc->tbqlen, MAX(16384, PAGE_SIZE), BUS_DMA_WAITOK,
		    &sc->tbqpage_dmamap);
	}
	if (err == 0) {
		DPRINTFN(1,("%s: -> bus_dmamap_load\n", __FUNCTION__));
		err = bus_dmamap_load(sc->sc_dmat, sc->tbqpage_dmamap,
		    sc->tbqpage, sc->tbqlen, NULL, BUS_DMA_WAITOK);
	}
	if (err != 0)
		panic("%s: Cannot get DMA memory", device_xname(sc->sc_dev));

	sc->tbqpage_dsaddr = sc->tbqpage_dmamap->dm_segs[0].ds_addr;
	memset(sc->tbqpage, 0, sc->tbqlen);

	/* Set up pointers to start of each queue in kernel addr space.
	 * Each descriptor queue or status queue entry uses 2 words
	 */
	sc->RDSC = (void *)sc->rbqpage;
	sc->TDSC = (void *)sc->tbqpage;

	/* init TX queue */
	for (i = 0; i < TX_QLEN; i++) {
		sc->TDSC[i].Addr = 0;
		sc->TDSC[i].Info = ETH_TDSC_I_USED |
		    (i == (TX_QLEN - 1) ? ETH_TDSC_I_WRAP : 0);
	}

	/* Populate the RXQ with mbufs */
	sc->rxqi = 0;
	for (i = 0; i < RX_QLEN; i++) {
		struct mbuf *m;

		err = bus_dmamap_create(sc->sc_dmat, MCLBYTES, 1, MCLBYTES,
		    PAGE_SIZE, BUS_DMA_WAITOK, &sc->rxq[i].m_dmamap);
		if (err) {
			panic("%s: dmamap_create failed: %i\n", __FUNCTION__,
			    err);
		}
		MGETHDR(m, M_WAIT, MT_DATA);
		MCLGET(m, M_WAIT);
		sc->rxq[i].m = m;
		if (mtod(m, intptr_t) & 3) {
			m_adj(m, mtod(m, intptr_t) & 3);
		}
		err = bus_dmamap_load(sc->sc_dmat, sc->rxq[i].m_dmamap,
		    m->m_ext.ext_buf, MCLBYTES, NULL,
		    BUS_DMA_WAITOK);
		if (err) {
			panic("%s: dmamap_load failed: %i\n", __FUNCTION__,
			    err);
		}
		sc->RDSC[i].Addr = sc->rxq[i].m_dmamap->dm_segs[0].ds_addr
		    | (i == (RX_QLEN-1) ? ETH_RDSC_F_WRAP : 0);
		sc->RDSC[i].Info = 0;
		bus_dmamap_sync(sc->sc_dmat, sc->rxq[i].m_dmamap, 0,
		    MCLBYTES, BUS_DMASYNC_PREREAD);
	}

	/* prepare transmit queue */
	for (i = 0; i < TX_QLEN; i++) {
		err = bus_dmamap_create(sc->sc_dmat, MCLBYTES, 1, MCLBYTES, 0,
		    (BUS_DMA_WAITOK | BUS_DMA_ALLOCNOW),
		    &sc->txq[i].m_dmamap);
		if (err)
			panic("ARGH #1");
		sc->txq[i].m = NULL;
	}

	/* Program each queue's start addr, cur addr, and len registers
	 * with the physical addresses.
	 */
	CEMAC_WRITE(ETH_RBQP, (uint32_t)sc->rbqpage_dsaddr);
	CEMAC_WRITE(ETH_TBQP, (uint32_t)sc->tbqpage_dsaddr);

	sc->sc_mcast_lock = mutex_obj_alloc(MUTEX_DEFAULT, IPL_SOFTNET);
//	sc->sc_intr_lock = mutex_obj_alloc(MUTEX_DEFAULT, IPL_NET);

	/* Divide HCLK by 32 for MDC clock */

#if 0
	bp->phylink_sgmii_pcs.ops = &macb_phylink_pcs_ops;
	bp->phylink_sgmii_pcs.neg_mode = true;
	bp->phylink_usx_pcs.ops = &macb_phylink_usx_pcs_ops;
	bp->phylink_usx_pcs.neg_mode = true;

	//XXXNH this is true
	if (macb_is_gem(bp) && (bp->caps & MACB_CAPS_GIGABIT_MODE_AVAILABLE)) {
		bp->phylink_config.mac_capabilities |= MAC_1000FD;
		if (!(bp->caps & MACB_CAPS_NO_GIGABIT_HALF))
			bp->phylink_config.mac_capabilities |= MAC_1000HD;

		__set_bit(PHY_INTERFACE_MODE_GMII,
			  bp->phylink_config.supported_interfaces);
		phy_interface_set_rgmii(bp->phylink_config.supported_interfaces);

		if (bp->caps & MACB_CAPS_PCS)
			__set_bit(PHY_INTERFACE_MODE_SGMII,
				  bp->phylink_config.supported_interfaces);

		if (bp->caps & MACB_CAPS_HIGH_SPEED) {
			__set_bit(PHY_INTERFACE_MODE_10GBASER,
				  bp->phylink_config.supported_interfaces);
			bp->phylink_config.mac_capabilities |= MAC_10000FD;
		}
	}



#endif


#if 1

//[    2.223449] macb:macb_mdio_read_c22: phy 0x0001 reg 0x0002  = 0x600d
//[    2.258871] macb:macb_mdio_read_c22: phy 0x0001 reg 0x0003  = 0x84a2

// pin conf?
// XXXNH !!!

//	phy-reset-gpios = <&rp1_gpio 32 GPIO_ACTIVE_LOW>;
// XXXNH switch to CLR/SET as active low???
	printf("%s: gpio reset start\n", __func__);
	// const struct rp1_iobank_desc rp1_iobanks[RP1_NUM_BANKS] = {
	// ...
	// 	{ 28,  6, 0x4000, 0x411c, 0x4124, 0x4000, 0x4004 },

	//[    2.938569] rp1_set_value: write 32 bank 1 rio 00000000a26d316c+0(s/c 2000) offset 0x0004 value 1


	rp1_pin_write(sc,
	    RP1_SYS_RIO0_BASE +			/* rio base */
	    0x4000 +				/* bank 1 rio_offset*/
	    RP1_RIO_OUT + RP1_SET_OFFSET,
	    __BIT(32 - 28));

	DELAY(5 * 1000);

	rp1_pin_write(sc,
	    RP1_SYS_RIO0_BASE +			/* rio base */
	    0x4000 +				/* bank 1 rio_offset*/
	    RP1_RIO_OUT + RP1_CLR_OFFSET,
	    __BIT(32 - 28));
printf("%s: gpio reset done\n", __func__);
	DELAY(5 * 1000);




#endif


/*
 * [    2.223449] macb:macb_mdio_read_c22: phy 0x0001 reg 0x0002  = 0x600d
 * [    2.229829] macb:hw_readl_native:  readl(0x0008) = 0x00000006
 * [    2.235595] macb:hw_writel_native: writel(0x0034) = 0x608e0000
 * [    2.241455] macb:hw_readl_native:  readl(0x0008) = 0x00000002
 * [    2.247330] macb:hw_readl_native:  readl(0x0008) = 0x00000006
 * [    2.253103] macb:hw_readl_native:  readl(0x0034) = 0x608e84a2
 * [    2.258871] macb:macb_mdio_read_c22: phy 0x0001 reg 0x0003  = 0x84a2
 */

	uint16_t id1;
	uint16_t id2;
	cemac_mii_readreg(sc->sc_dev, 1, MII_PHYIDR1, &id1);
	cemac_mii_readreg(sc->sc_dev, 1, MII_PHYIDR2, &id2);

	KASSERTMSG(id1 == 0x600d && id2 == 0x84a2, "id1/id2 %#06x/%#06x", id1, id2);

	sc->sc_ethercom.ec_mii = mii;
	// XXXNH 0 vs IFM_MASK
	ifmedia_init(&mii->mii_media, IFM_IMASK, cemac_mediachange,
	    cemac_mediastatus);
	mii->mii_ifp = ifp;
	mii->mii_readreg = cemac_mii_readreg;
	mii->mii_writereg = cemac_mii_writereg;
	mii->mii_statchg = cemac_statchg;
	ifmedia_init(&mii->mii_media, IFM_IMASK, cemac_mediachange,
	    cemac_mediastatus);
	mii_attach(sc->sc_dev, mii, 0xffffffff, sc->sc_phyno, MII_OFFSET_ANY, 0);
	ifmedia_set(&mii->mii_media, IFM_ETHER | IFM_AUTO);


#if 0
	// enable / disable interrupts
	CEMAC_WRITE(ETH_IDR, -1);
	CEMAC_WRITE(ETH_IER, ETH_ISR_RCOM | ETH_ISR_TBRE | ETH_ISR_TIDLE
	    | ETH_ISR_RBNA | ETH_ISR_ROVR | ETH_ISR_TCOM);
//	(void)CEMAC_READ(ETH_ISR); // why

	// enable transmitter / receiver
	CEMAC_WRITE(ETH_CTL, ETH_CTL_TE | ETH_CTL_RE | ETH_CTL_ISR
	    | ETH_CTL_CSR | ETH_CTL_MPE);
#endif
	/*
	 * We can support hardware checksumming.
	 */
	ifp->if_capabilities |=
	    IFCAP_CSUM_IPv4_Tx | IFCAP_CSUM_IPv4_Rx |
	    IFCAP_CSUM_TCPv4_Tx | IFCAP_CSUM_TCPv4_Rx |
	    IFCAP_CSUM_UDPv4_Tx | IFCAP_CSUM_UDPv4_Rx |
	    IFCAP_CSUM_TCPv6_Tx | IFCAP_CSUM_TCPv6_Rx |
	    IFCAP_CSUM_UDPv6_Tx | IFCAP_CSUM_UDPv6_Rx;

	/*
	 * We can support 802.1Q VLAN-sized frames.
	 */
	sc->sc_ethercom.ec_capabilities |= ETHERCAP_VLAN_MTU;

	strcpy(ifp->if_xname, device_xname(sc->sc_dev));
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_extflags = IFEF_MPSAFE;
	ifp->if_ioctl = cemac_ifioctl;
	ifp->if_start = cemac_ifstart;
	ifp->if_watchdog = cemac_ifwatchdog;
	ifp->if_init = cemac_ifinit;
	ifp->if_stop = cemac_ifstop;
	ifp->if_softc = sc;
	IFQ_SET_READY(&ifp->if_snd);
	if_attach(ifp);
	if_deferred_start_init(ifp, NULL);
	ether_ifattach(ifp, (sc)->sc_enaddr);
	ether_set_ifflags_cb(&sc->sc_ethercom, cemac_ifflags_cb);
}

static int
cemac_mediachange(struct ifnet *ifp)
{
	if (ifp->if_flags & IFF_UP)
		cemac_ifinit(ifp);
	return 0;
}

static void
cemac_mediastatus(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct cemac_softc * const sc = ifp->if_softc;

	mii_pollstat(&sc->sc_mii);
	ifmr->ifm_active = sc->sc_mii.mii_media_active;
	ifmr->ifm_status = sc->sc_mii.mii_media_status;
}


//XXXNH clause 45?

static int
cemac_mii_readreg(device_t self, int phy, int reg, uint16_t *val)
{
	struct cemac_softc * const sc = device_private(self);

	KASSERTMSG(phy <= __SHIFTOUT_MASK(ETH_MAN_PHYA), "phy %d", phy);
	KASSERTMSG(reg <= __SHIFTOUT_MASK(ETH_MAN_REGA), "reg %d", phy);

	while (!(CEMAC_READ(ETH_SR) & ETH_SR_IDLE))
		;

	CEMAC_WRITE(ETH_MAN, (ETH_MAN_HIGH | ETH_MAN_RW_RD
			     | __SHIFTIN(phy, ETH_MAN_PHYA)
			     | __SHIFTIN(reg, ETH_MAN_REGA)
			     | ETH_MAN_CODE_IEEE802_3));
	while (!(CEMAC_READ(ETH_SR) & ETH_SR_IDLE))
		;

	*val = CEMAC_READ(ETH_MAN) & ETH_MAN_DATA;

printf("%s: phy %3d reg %#06x val %#010x\n", __func__, phy, reg, *val);

	if (reg == 1)
		    *val = 0xc0de;
	return 0;
}

static int
cemac_mii_writereg(device_t self, int phy, int reg, uint16_t val)
{
	struct cemac_softc * const sc = device_private(self);

	while (!(CEMAC_READ(ETH_SR) & ETH_SR_IDLE))
		;

	KASSERTMSG(phy <= __SHIFTOUT_MASK(ETH_MAN_PHYA), "phy %d", phy);
	KASSERTMSG(reg <= __SHIFTOUT_MASK(ETH_MAN_REGA), "reg %d", phy);

printf("%s: phy %3d reg %#06x val = %#010x\n", __func__, phy, reg, val);

	CEMAC_WRITE(ETH_MAN, (ETH_MAN_HIGH | ETH_MAN_RW_WR
			     | __SHIFTIN(phy, ETH_MAN_PHYA)
			     | __SHIFTIN(reg, ETH_MAN_REGA)
			     | ETH_MAN_CODE_IEEE802_3
			     | (val & ETH_MAN_DATA)));
	while (!(CEMAC_READ(ETH_SR) & ETH_SR_IDLE))
		;

	return 0;
}


static void
cemac_statchg(struct ifnet *ifp)
{
	struct cemac_softc * const sc = ifp->if_softc;
	struct mii_data *mii = &sc->sc_mii;
	uint32_t reg;

	/*
	 * We must keep the MAC and the PHY in sync as
	 * to the status of full-duplex!
	 */
	reg = CEMAC_READ(ETH_CFG);
	reg &= ~ETH_CFG_FD;
	if (sc->sc_mii.mii_media_active & IFM_FDX)
		reg |= ETH_CFG_FD;

	reg &= ~ETH_CFG_SPD;
	if (ISSET(sc->cemac_flags, CEMAC_FLAG_GEM))
		reg &= ~GEM_CFG_GEN;
	switch (IFM_SUBTYPE(mii->mii_media_active)) {
	case IFM_10_T:
		break;
	case IFM_100_TX:
		reg |= ETH_CFG_SPD;
		break;
	case IFM_1000_T:
		reg |= ETH_CFG_SPD | GEM_CFG_GEN;
		break;
	default:
		break;
	}
	CEMAC_WRITE(ETH_CFG, reg);
}

static bool
cemac_watchdog_check(struct cemac_softc * const sc)
{

	KASSERT(mutex_owned(sc->sc_intr_lock));

	if (!sc->sc_tx_sending)
		return true;

	if (time_uptime - sc->sc_tx_lastsent <= cemac_watchdog_timeout)
		return true;

	return false;
}

static bool
cemac_watchdog_tick(struct ifnet *ifp)
{
	struct cemac_softc * const sc = ifp->if_softc;

	KASSERT(mutex_owned(sc->sc_intr_lock));

	if (!sc->sc_trigger_reset && cemac_watchdog_check(sc))
		return true;

	if (atomic_swap_uint(&sc->sc_reset_pending, 1) == 0)
		workqueue_enqueue(sc->sc_reset_wq, &sc->sc_reset_work, NULL);

	return false;
}


static void
cemac_tick(void *arg)
{
	struct cemac_softc * const sc = arg;
	struct ifnet * const ifp = &sc->sc_ethercom.ec_if;

	mutex_enter(sc->sc_intr_lock);
	if (sc->sc_stopping) {
		mutex_exit(sc->sc_intr_lock);
		return;
	}

	if (ISSET(sc->cemac_flags, CEMAC_FLAG_GEM))
		if_statadd(ifp, if_collisions,
		    CEMAC_READ(GEM_SCOL) + CEMAC_READ(GEM_MCOL));
	else
		if_statadd(ifp, if_collisions,
		    CEMAC_READ(ETH_SCOL) + CEMAC_READ(ETH_MCOL));

	/* These misses are ok, they will happen if the RAM/CPU can't keep up */
	if (!ISSET(sc->cemac_flags, CEMAC_FLAG_GEM)) {
		uint32_t misses = CEMAC_READ(ETH_DRFC);
		if (misses > 0)
			aprint_normal_ifnet(ifp, "%d rx misses\n", misses);
	}

	mii_tick(&sc->sc_mii);

	const bool ok = cemac_watchdog_tick(ifp);
	if (ok)
		callout_schedule(&sc->cemac_tick_ch, hz);

	mutex_exit(sc->sc_intr_lock);
}


static int
cemac_ifioctl(struct ifnet *ifp, u_long cmd, void *data)
{
	struct cemac_softc * const sc = ifp->if_softc;
	int error;

 	switch (cmd) {
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		break;
 	default:
		KASSERT(IFNET_LOCKED(ifp));
	}

	const int s = splnet();
	error = ether_ioctl(ifp, cmd, data);
	splx(s);

	if (error == ENETRESET) {
 		error = 0;

		if (cmd == SIOCADDMULTI || cmd == SIOCDELMULTI) {
			mutex_enter(sc->sc_mcast_lock);
			if ((sc->sc_if_flags & IFF_RUNNING) != 0)
				cemac_setaddr(ifp);

			mutex_exit(sc->sc_mcast_lock);
 		}
 	}

	return error;
}



static void
cemac_ifstart(struct ifnet *ifp)
{
	struct cemac_softc * const sc = ifp->if_softc;
	KASSERT(if_is_mpsafe(ifp));

	mutex_enter(sc->sc_intr_lock);
	if (!sc->sc_stopping) {
		cemac_ifstart_locked(ifp);
	}
	mutex_exit(sc->sc_intr_lock);
}

static void
cemac_ifstart_locked(struct ifnet *ifp)
{
	struct cemac_softc * const sc = ifp->if_softc;
	struct mbuf *m;
	bus_dma_segment_t *segs;
	int bi, err, nsegs;

	KASSERT(mutex_owned(sc->sc_intr_lock));

start:
	if (cemac_gctx(sc) == 0) {
		/* Enable transmit-buffer-free interrupt */
		CEMAC_WRITE(ETH_IER, ETH_ISR_TBRE);
		sc->sc_txbusy = true;
		return;
	}

	IFQ_POLL(&ifp->if_snd, m);
	if (m == NULL) {
		return;
	}

	bi = (sc->txqi + sc->txqc) % TX_QLEN;
	if ((err = bus_dmamap_load_mbuf(sc->sc_dmat, sc->txq[bi].m_dmamap, m,
		BUS_DMA_NOWAIT)) ||
		sc->txq[bi].m_dmamap->dm_segs[0].ds_addr & 0x3 ||
		sc->txq[bi].m_dmamap->dm_nsegs > 1) {
		/* Copy entire mbuf chain to new single */
		struct mbuf *mn;

		if (err == 0)
			bus_dmamap_unload(sc->sc_dmat, sc->txq[bi].m_dmamap);

		MGETHDR(mn, M_DONTWAIT, MT_DATA);
		if (mn == NULL)
			return;
		if (m->m_pkthdr.len > MHLEN) {
			MCLGET(mn, M_DONTWAIT);
			if ((mn->m_flags & M_EXT) == 0) {
				m_freem(mn);
				return;
			}
		}
		m_copydata(m, 0, m->m_pkthdr.len, mtod(mn, void *));
		mn->m_pkthdr.len = mn->m_len = m->m_pkthdr.len;
		IFQ_DEQUEUE(&ifp->if_snd, m);
		m_freem(m);
		m = mn;
		bus_dmamap_load_mbuf(sc->sc_dmat, sc->txq[bi].m_dmamap, m,
		    BUS_DMA_NOWAIT);
	} else {
		IFQ_DEQUEUE(&ifp->if_snd, m);
	}

	bpf_mtap(ifp, m, BPF_D_OUT);

	nsegs = sc->txq[bi].m_dmamap->dm_nsegs;
	segs = sc->txq[bi].m_dmamap->dm_segs;
	if (nsegs > 1)
		panic("#### ARGH #2");

	sc->txq[bi].m = m;
	sc->txqc++;

	DPRINTFN(2,("%s: start sending idx #%i mbuf %p (txqc=%i, phys %p), "
	    "len=%u\n", __FUNCTION__, bi, sc->txq[bi].m, sc->txqc,
	     (void *)segs->ds_addr, (unsigned)m->m_pkthdr.len));
#ifdef	DIAGNOSTIC
	if (sc->txqc > TX_QLEN)
		panic("%s: txqc %i > %i", __FUNCTION__, sc->txqc, TX_QLEN);
#endif

	bus_dmamap_sync(sc->sc_dmat, sc->txq[bi].m_dmamap, 0,
	    sc->txq[bi].m_dmamap->dm_mapsize, BUS_DMASYNC_PREWRITE);

	if (ISSET(sc->cemac_flags, CEMAC_FLAG_GEM)) {
		sc->TDSC[bi].Addr = segs->ds_addr;
		sc->TDSC[bi].Info =
		    __SHIFTIN(m->m_pkthdr.len, ETH_TDSC_I_LEN) |
		    ETH_TDSC_I_LAST_BUF |
		    (bi == (TX_QLEN - 1) ? ETH_TDSC_I_WRAP : 0);

		DPRINTFN(3,("%s: TDSC[%i].Addr 0x%08x\n",
			__FUNCTION__, bi, sc->TDSC[bi].Addr));
		DPRINTFN(3,("%s: TDSC[%i].Info 0x%08x\n",
			__FUNCTION__, bi, sc->TDSC[bi].Info));

		uint32_t ctl = CEMAC_READ(ETH_CTL) | GEM_CTL_STARTTX;
		CEMAC_WRITE(ETH_CTL, ctl);
		DPRINTFN(3,("%s: ETH_CTL 0x%08x\n", __FUNCTION__,
		    CEMAC_READ(ETH_CTL)));
	} else {
		CEMAC_WRITE(ETH_TAR, segs->ds_addr);
		CEMAC_WRITE(ETH_TCR, m->m_pkthdr.len);
	}
	sc->sc_tx_lastsent = time_uptime;

	if (IFQ_IS_EMPTY(&ifp->if_snd) == 0)
		goto start;

	return;
}

static void
cemac_ifwatchdog(struct ifnet *ifp)
{
	struct cemac_softc * const sc = ifp->if_softc;

	if ((ifp->if_flags & IFF_RUNNING) == 0)
		return;
	aprint_error_ifnet(ifp, "device timeout, CTL = 0x%08x, CFG = 0x%08x\n",
	    CEMAC_READ(ETH_CTL), CEMAC_READ(ETH_CFG));
}

static int
cemac_ifinit(struct ifnet *ifp)
{
	struct cemac_softc * const sc = ifp->if_softc;
	uint32_t dma, cfg;

	ASSERT_SLEEPABLE();
	KASSERT(IFNET_LOCKED(ifp));

	/* Cancel pending I/O and flush buffers. */
	cemac_ifstop(ifp, 0);

	if (ISSET(sc->cemac_flags, CEMAC_FLAG_GEM)) {
		if (ifp->if_capenable &
		    (IFCAP_CSUM_IPv4_Tx |
			IFCAP_CSUM_TCPv4_Tx | IFCAP_CSUM_UDPv4_Tx |
			IFCAP_CSUM_TCPv6_Tx | IFCAP_CSUM_UDPv6_Tx)) {
			dma = CEMAC_READ(GEM_DMA_CFG);
			dma |= GEM_DMA_CFG_CHKSUM_GEN_OFFLOAD_EN;
			CEMAC_WRITE(GEM_DMA_CFG, dma);
		}
		if (ifp->if_capenable &
		    (IFCAP_CSUM_IPv4_Rx |
			IFCAP_CSUM_TCPv4_Rx | IFCAP_CSUM_UDPv4_Rx |
			IFCAP_CSUM_TCPv6_Rx | IFCAP_CSUM_UDPv6_Rx)) {
			cfg = CEMAC_READ(ETH_CFG);
			cfg |= GEM_CFG_RXCOEN;
			CEMAC_WRITE(ETH_CFG, cfg);
		}
	}

	// enable interrupts
	CEMAC_WRITE(ETH_IDR, -1);
	CEMAC_WRITE(ETH_IER, ETH_ISR_RCOM | ETH_ISR_TBRE | ETH_ISR_TIDLE
	    | ETH_ISR_RBNA | ETH_ISR_ROVR | ETH_ISR_TCOM);

	// enable transmitter / receiver
	CEMAC_WRITE(ETH_CTL, ETH_CTL_TE | ETH_CTL_RE | ETH_CTL_ISR
	    | ETH_CTL_CSR | ETH_CTL_MPE);

	// mii_lock?
	mii_mediachg(&sc->sc_mii);

	callout_reset(&sc->cemac_tick_ch, hz, cemac_tick, sc);
	ifp->if_flags |= IFF_RUNNING;

	mutex_enter(sc->sc_intr_lock);
	sc->sc_stopping = false;
	mutex_exit(sc->sc_intr_lock);

	return 0;
}

static void
cemac_ifstop(struct ifnet *ifp, int disable)
{
	struct cemac_softc * const sc = ifp->if_softc;

	ASSERT_SLEEPABLE();
	KASSERT(IFNET_LOCKED(ifp));

	ifp->if_flags &= ~IFF_RUNNING;

	mutex_enter(sc->sc_mcast_lock);
	sc->sc_if_flags = ifp->if_flags;
	mutex_exit(sc->sc_mcast_lock);

	mutex_enter(sc->sc_intr_lock);
	sc->sc_stopping = true;
	mutex_exit(sc->sc_intr_lock);

#if 1
	CEMAC_WRITE(ETH_CTL, 0);	// disable everything
	CEMAC_WRITE(ETH_IDR, -1);	// disable interrupts
//	CEMAC_WRITE(ETH_RBQP, 0);	// clear receive
	if (ISSET(sc->cemac_flags, CEMAC_FLAG_GEM)) {
		CEMAC_WRITE(ETH_CFG,
		    GEM_CFG_CLK_64 | ETH_CFG_SPD | ETH_CFG_FD | ETH_CFG_BIG);
	} else {
		CEMAC_WRITE(ETH_CFG,
		    ETH_CFG_CLK_32 | ETH_CFG_SPD | ETH_CFG_FD | ETH_CFG_BIG);
	}
//	CEMAC_WRITE(ETH_TCR, 0);			// send nothing
//	(void)CEMAC_READ(ETH_ISR);
	uint32_t u = CEMAC_READ(ETH_TSR);
	CEMAC_WRITE(ETH_TSR, (u & (ETH_TSR_UND | ETH_TSR_COMP | ETH_TSR_BNQ
				  | ETH_TSR_IDLE | ETH_TSR_RLE
				  | ETH_TSR_COL | ETH_TSR_OVR)));
	u = CEMAC_READ(ETH_RSR);
	CEMAC_WRITE(ETH_RSR, (u & (ETH_RSR_OVR | ETH_RSR_REC | ETH_RSR_BNA)));
#endif

	callout_halt(&sc->cemac_tick_ch, NULL);

	/* Down the MII. */
	mii_down(&sc->sc_mii);

	ifp->if_flags &= ~IFF_RUNNING;
	sc->sc_txbusy = false;
	sc->sc_mii.mii_media_status &= ~IFM_ACTIVE;

	// XXXNH
	// reset rings

	// XXXNH
//	sc->sc_mii.mii_media_status &= ~IFM_ACTIVE;
}

static void
cemac_setaddr(struct ifnet *ifp)
{
	struct cemac_softc * const sc = ifp->if_softc;
	struct ethercom *ec = &sc->sc_ethercom;
	struct ether_multi *enm;
	struct ether_multistep step;
	uint8_t ias[3][ETHER_ADDR_LEN];
	uint32_t h, nma = 0, hashes[2] = { 0, 0 };
	uint32_t ctl = CEMAC_READ(ETH_CTL);
	uint32_t cfg = CEMAC_READ(ETH_CFG);

	KASSERT(mutex_owned(sc->sc_mcast_lock));

	/* disable receiver temporarily */
	CEMAC_WRITE(ETH_CTL, ctl & ~ETH_CTL_RE);

	cfg &= ~(ETH_CFG_MTI | ETH_CFG_UNI | ETH_CFG_CAF | ETH_CFG_UNI);

	if (sc->sc_if_flags & IFF_PROMISC) {
		cfg |=	ETH_CFG_CAF;
	} else {
		cfg &= ~ETH_CFG_CAF;
	}

	// ETH_CFG_BIG?

	ETHER_LOCK(ec);
	ec->ec_flags &= ~ETHER_F_ALLMULTI;

	ETHER_FIRST_MULTI(step, ec, enm);
	while (enm != NULL) {
		if (memcmp(enm->enm_addrlo, enm->enm_addrhi, ETHER_ADDR_LEN)) {
			/*
			 * We must listen to a range of multicast addresses.
			 * For now, just accept all multicasts, rather than
			 * trying to set only those filter bits needed to match
			 * the range.  (At this time, the only use of address
			 * ranges is for IP multicast routing, for which the
			 * range is big enough to require all bits set.)
			 */
			cfg |= ETH_CFG_MTI;
			hashes[0] = 0xffffffffUL;
			hashes[1] = 0xffffffffUL;
			nma = 0;
			ec->ec_flags |= ETHER_F_ALLMULTI;
			break;
		}

		if (nma < 3) {
			/* We can program 3 perfect address filters for mcast */
			memcpy(ias[nma], enm->enm_addrlo, ETHER_ADDR_LEN);
		} else {
			/*
			 * XXX: Datasheet is not very clear here, I'm not sure
			 * if I'm doing this right.  --joff
			 */
			h = ether_crc32_le(enm->enm_addrlo, ETHER_ADDR_LEN);

			/* Just want the 6 most-significant bits. */
			h = h >> 26;
#if 0
			hashes[h / 32] |=  (1 << (h % 32));
#else
			hashes[0] = 0xffffffffUL;
			hashes[1] = 0xffffffffUL;
#endif
			cfg |= ETH_CFG_MTI;
		}
		ETHER_NEXT_MULTI(step, enm);
		nma++;
	}
	ETHER_UNLOCK(ec);

	// program...
	DPRINTFN(1,("%s: en0 %02x:%02x:%02x:%02x:%02x:%02x\n", __FUNCTION__,
		sc->sc_enaddr[0], sc->sc_enaddr[1], sc->sc_enaddr[2],
		sc->sc_enaddr[3], sc->sc_enaddr[4], sc->sc_enaddr[5]));
	CEMAC_GEM_WRITE(SA1L, (sc->sc_enaddr[3] << 24)
	    | (sc->sc_enaddr[2] << 16) | (sc->sc_enaddr[1] << 8)
	    | (sc->sc_enaddr[0]));
	CEMAC_GEM_WRITE(SA1H, (sc->sc_enaddr[5] << 8)
	    | (sc->sc_enaddr[4]));
	if (nma > 0) {
		DPRINTFN(1,("%s: en1 %02x:%02x:%02x:%02x:%02x:%02x\n",
		    __FUNCTION__,
		    ias[0][0], ias[0][1], ias[0][2],
		    ias[0][3], ias[0][4], ias[0][5]));
		CEMAC_WRITE(ETH_SA2L, (ias[0][3] << 24)
		    | (ias[0][2] << 16) | (ias[0][1] << 8)
		    | (ias[0][0]));
		CEMAC_WRITE(ETH_SA2H, (ias[0][4] << 8)
		    | (ias[0][5]));
	}
	if (nma > 1) {
		DPRINTFN(1,("%s: en2 %02x:%02x:%02x:%02x:%02x:%02x\n",
		    __FUNCTION__,
		    ias[1][0], ias[1][1], ias[1][2],
		    ias[1][3], ias[1][4], ias[1][5]));
		CEMAC_WRITE(ETH_SA3L, (ias[1][3] << 24)
		    | (ias[1][2] << 16) | (ias[1][1] << 8)
		    | (ias[1][0]));
		CEMAC_WRITE(ETH_SA3H, (ias[1][4] << 8)
		    | (ias[1][5]));
	}
	if (nma > 2) {
		DPRINTFN(1,("%s: en3 %02x:%02x:%02x:%02x:%02x:%02x\n",
		    __FUNCTION__,
		    ias[2][0], ias[2][1], ias[2][2],
		    ias[2][3], ias[2][4], ias[2][5]));
		CEMAC_WRITE(ETH_SA4L, (ias[2][3] << 24)
		    | (ias[2][2] << 16) | (ias[2][1] << 8)
		    | (ias[2][0]));
		CEMAC_WRITE(ETH_SA4H, (ias[2][4] << 8)
		    | (ias[2][5]));
	}
	CEMAC_GEM_WRITE(HSH, hashes[0]);
	CEMAC_GEM_WRITE(HSL, hashes[1]);
	CEMAC_WRITE(ETH_CFG, cfg);
	CEMAC_WRITE(ETH_CTL, ctl | ETH_CTL_RE);
}
