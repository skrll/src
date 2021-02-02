/*	$NetBSD$	*/

/*-
 * Copyright (c) 2015 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed by Semihalf under
 * the sponsorship of the FreeBSD Foundation.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>

#include <sys/bus.h>
#include <sys/device.h>
#include <sys/kmem.h>
#include <sys/mutex.h>
//#include <sys/reboot.h>

#include <net/if.h>
#include <net/if_ether.h>
#include <net/if_media.h>

//#include <dev/mii/mii.h>
//#include <dev/mii/mdio.h>
#include <dev/mii/miivar.h>
//#include <dev/mii/miidevs.h>

#include "thunder_mdio_var.h"

#define	SMI_CMD				0x00
#define	 SMI_CMD_PHY_REG_ADR_SHIFT	(0)
#define	 SMI_CMD_PHY_REG_ADR_MASK	(0x1FUL << SMI_CMD_PHY_REG_ADR_SHIFT)
#define	 SMI_CMD_PHY_ADR_SHIFT		(8)
#define	 SMI_CMD_PHY_ADR_MASK		(0x1FUL << SMI_CMD_PHY_ADR_SHIFT)
#define	 SMI_CMD_PHY_OP_MASK		(0x3UL << 16)
#define	 SMI_CMD_PHY_OP_C22_READ	(0x1UL << 16)
#define	 SMI_CMD_PHY_OP_C22_WRITE	(0x0UL << 16)
#define	 SMI_CMD_PHY_OP_C45_READ	(0x3UL << 16)
#define	 SMI_CMD_PHY_OP_C45_WRITE	(0x1UL << 16)
#define	 SMI_CMD_PHY_OP_C45_ADDR	(0x0UL << 16)

#define	SMI_WR_DAT			0x08
#define	 SMI_WR_DAT_PENDING		(1UL << 17)
#define	 SMI_WR_DAT_VAL			(1UL << 16)
#define	 SMI_WR_DAT_DAT_MASK		(0xFFFFUL << 0)

#define	SMI_RD_DAT			0x10
#define	 SMI_RD_DAT_PENDING		(1UL << 17)
#define	 SMI_RD_DAT_VAL			(1UL << 16)
#define	 SMI_RD_DAT_DAT_MASK		(0xFFFFUL << 0)

#define	SMI_CLK				0x18
#define	 SMI_CLK_PREAMBLE		(1UL << 12)
#define	 SMI_CLK_MODE			(1UL << 24)

#define	SMI_EN				0x20
#define	 SMI_EN_EN			(1UL << 0)	/* Enabele interface */

#define	SMI_DRV_CTL			0x28

static int thunder_mdio_read(device_t, int, int, uint16_t *);
static int thunder_mdio_write(device_t, int, int, uint16_t);
static void thunder_mdio_statchg(struct ifnet *);

#if 0
static int thunder_mdio_media_change(struct ifnet *);
#endif

int thunder_mdio_phy_connect(device_t, int, int);
int thunder_mdio_phy_disconnect(device_t, int, int);
int thunder_mdio_media_status(device_t, int, int *, int *, int *);

#if 0
static device_method_t thunder_mdio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_detach,	thunder_mdio_detach),
	/* LMAC interface */
	DEVMETHOD(lmac_media_status,	thunder_mdio_media_status),
	DEVMETHOD(lmac_media_change,	thunder_mdio_media_change),
	DEVMETHOD(lmac_phy_connect,	thunder_mdio_phy_connect),
	DEVMETHOD(lmac_phy_disconnect,	thunder_mdio_phy_disconnect),
	/* MII interface */
	DEVMETHOD(miibus_readreg,	thunder_mdio_read),
	DEVMETHOD(miibus_writereg,	thunder_mdio_write),

	/* End */
	DEVMETHOD_END
};

DEFINE_CLASS_0(thunder_mdio, thunder_mdio_driver, thunder_mdio_methods,
    sizeof(struct thunder_mdio_softc));

DRIVER_MODULE(miibus, thunder_mdio, miibus_driver, miibus_devclass, 0, 0);
MODULE_VERSION(thunder_mdio, 1);
MODULE_DEPEND(thunder_mdio, ether, 1, 1, 1);
MODULE_DEPEND(thunder_mdio, miibus, 1, 1, 1);
MODULE_DEPEND(thunder_mdio, mrmlbus, 1, 1, 1);

MALLOC_DEFINE(M_THUNDER_MDIO, "ThunderX MDIO",
    "Cavium ThunderX MDIO dynamic memory");
#endif


//XXXNH IPL_NET?
#define	MDIO_LOCK_INIT(sc, name)			\
    mutex_init(&(sc)->mtx, MUTEX_DEFAULT, IPL_NET);

#define	MDIO_LOCK_DESTROY(sc)				\
    mutex_destroy(&(sc)->mtx)

#define	MDIO_LOCK(sc)	mutex_enter(&(sc)->mtx)
#define	MDIO_UNLOCK(sc)	mutex_exit(&(sc)->mtx)

#define	MDIO_LOCK_ASSERT(sc)				\
    mutex_owned(&(sc)->mtx)

#define	mdio_reg_read(sc, reg)				\
    bus_space_read_8((sc)->sc_memt, (sc)->sc_memh, (reg))

#define	mdio_reg_write(sc, reg, val)			\
    bus_space_write_8((sc)->sc_memt, (sc)->sc_memh, (reg), (val))

int
thunder_mdio_attach(device_t dev)
{
	struct thunder_mdio_softc *sc;

	sc = device_private(dev);
	sc->dev = dev;

	TAILQ_INIT(&sc->phy_desc_head);
	MDIO_LOCK_INIT(sc, "ThunderX MDIO lock");

	/* Enable SMI/MDIO interface */
	mdio_reg_write(sc, SMI_EN, SMI_EN_EN);

	return (0);
}

int
thunder_mdio_detach(device_t dev, int flags)
{
#if 0
	struct thunder_mdio_softc *sc;

	sc = device_private(dev);

// 	if (sc->reg_base != NULL) {
// 		bus_release_resource(dev, SYS_RES_MEMORY, REG_BASE_RID,
// 		    sc->reg_base);
// 	}
#endif
	return (0);
}

static __inline void
thunder_mdio_set_mode(struct thunder_mdio_softc *sc,
    enum thunder_mdio_mode mode)
{
	uint64_t smi_clk;

	if (sc->mode == mode)
		return;

	/* Set mode, IEEE CLAUSE 22 or IEEE CAUSE 45 */
	smi_clk = mdio_reg_read(sc, SMI_CLK);
	if (mode == MODE_IEEE_C22)
		smi_clk &= ~SMI_CLK_MODE;
	else
		smi_clk |= SMI_CLK_MODE;
	/* Enable sending 32 bit preable on SMI transactions */
	smi_clk |= SMI_CLK_PREAMBLE;
	/* Saved setings */
	mdio_reg_write(sc, SMI_CLK, smi_clk);
	sc->mode = mode;
}

static int
thunder_mdio_c45_addr(struct thunder_mdio_softc *sc, int phy, int reg)
{
	uint64_t smi_cmd, smi_wr_dat;
	ssize_t timeout;

	thunder_mdio_set_mode(sc, MODE_IEEE_C45);

	/* Prepare data for transmission */
	mdio_reg_write(sc, SMI_WR_DAT, reg & SMI_WR_DAT_DAT_MASK);
	/*
	 * Assemble command
	 */
	smi_cmd = 0;
	/* Set opcode */
	smi_cmd |= SMI_CMD_PHY_OP_C45_WRITE;

	/* Set PHY address */
	smi_cmd |= ((phy << SMI_CMD_PHY_ADR_SHIFT) & SMI_CMD_PHY_ADR_MASK);
	/* Set PHY register offset */
	smi_cmd |= ((reg << SMI_CMD_PHY_REG_ADR_SHIFT) &
	    SMI_CMD_PHY_REG_ADR_MASK);

	mdio_reg_write(sc, SMI_CMD, smi_cmd);
	for (timeout = 1000; timeout > 0; timeout--) {
		smi_wr_dat = mdio_reg_read(sc, SMI_WR_DAT);
		if (smi_wr_dat & SMI_WR_DAT_PENDING)
			DELAY(1000);
		else
			break;
	}

	if (timeout <= 0)
		return (EIO);
	else {
		/* Return 0 on success */
		return (0);
	}
}

static int
thunder_mdio_read(device_t dev, int phy, int reg, uint16_t *val)
{
	struct thunder_mdio_softc *sc;
	uint64_t smi_cmd, smi_rd_dat;
	ssize_t timeout;
	int err;

	sc = device_private(dev);

	/* XXX Always C22 - for <= 1Gbps only */
	thunder_mdio_set_mode(sc, MODE_IEEE_C22);

	/*
	 * Assemble command
	 */
	smi_cmd = 0;
	/* Set opcode */
	if (sc->mode == MODE_IEEE_C22)
		smi_cmd |= SMI_CMD_PHY_OP_C22_READ;
	else {
		smi_cmd |= SMI_CMD_PHY_OP_C45_READ;
		err = thunder_mdio_c45_addr(sc, phy, reg);
		if (err != 0)
			return (err);

		reg = (reg >> 16) & 0x1F;
	}

	/* Set PHY address */
	smi_cmd |= ((phy << SMI_CMD_PHY_ADR_SHIFT) & SMI_CMD_PHY_ADR_MASK);
	/* Set PHY register offset */
	smi_cmd |= ((reg << SMI_CMD_PHY_REG_ADR_SHIFT) &
	    SMI_CMD_PHY_REG_ADR_MASK);

	mdio_reg_write(sc, SMI_CMD, smi_cmd);
	for (timeout = 1000; timeout > 0; timeout--) {
		smi_rd_dat = mdio_reg_read(sc, SMI_RD_DAT);
		if (smi_rd_dat & SMI_RD_DAT_PENDING)
			DELAY(1000);
		else
			break;
	}

	if (smi_rd_dat & SMI_RD_DAT_VAL) {
		*val = (smi_rd_dat & SMI_RD_DAT_DAT_MASK);
		return 0;
	} else {
		return ETIMEDOUT;
	}
}

static int
thunder_mdio_write(device_t dev, int phy, int reg, uint16_t data)
{
	struct thunder_mdio_softc *sc;
	uint64_t smi_cmd, smi_wr_dat;
	ssize_t timeout;

	sc = device_private(dev);

	/* XXX Always C22 - for <= 1Gbps only */
	thunder_mdio_set_mode(sc, MODE_IEEE_C22);

	/* Prepare data for transmission */
	mdio_reg_write(sc, SMI_WR_DAT, data & SMI_WR_DAT_DAT_MASK);
	/*
	 * Assemble command
	 */
	smi_cmd = 0;
	/* Set opcode */
	if (sc->mode == MODE_IEEE_C22)
		smi_cmd |= SMI_CMD_PHY_OP_C22_WRITE;
	else
		smi_cmd |= SMI_CMD_PHY_OP_C45_WRITE;

	/* Set PHY address */
	smi_cmd |= ((phy << SMI_CMD_PHY_ADR_SHIFT) & SMI_CMD_PHY_ADR_MASK);
	/* Set PHY register offset */
	smi_cmd |= ((reg << SMI_CMD_PHY_REG_ADR_SHIFT) &
	    SMI_CMD_PHY_REG_ADR_MASK);

	mdio_reg_write(sc, SMI_CMD, smi_cmd);
	for (timeout = 1000; timeout > 0; timeout--) {
		smi_wr_dat = mdio_reg_read(sc, SMI_WR_DAT);
		if (smi_wr_dat & SMI_WR_DAT_PENDING)
			DELAY(1000);
		else
			break;
	}

	if (timeout <= 0)
		return ETIMEDOUT;
	else {
		/* Return 0 on success */
		return (0);
	}
}


static void
thunder_mdio_statchg(struct ifnet *ifp)
{
	/* Will never be called by if_media */

}

static __inline struct phy_desc *
get_phy_desc(struct thunder_mdio_softc *sc, int lmacid)
{
	struct phy_desc *pd = NULL;

	MDIO_LOCK_ASSERT(sc);
	TAILQ_FOREACH(pd, &sc->phy_desc_head, phy_desc_list) {
		if (pd->lmacid == lmacid)
			break;
	}

	return (pd);
}

int
thunder_mdio_media_status(device_t dev, int lmacid, int *link, int *duplex,
    int *speed)
{
	struct thunder_mdio_softc *sc;
	struct mii_data *mii_sc;
	struct phy_desc *pd;

	sc = device_private(dev);

	MDIO_LOCK(sc);
	pd = get_phy_desc(sc, lmacid);
	if (pd == NULL) {
		/* Panic when invariants are enabled, fail otherwise. */
		KASSERTMSG(false, "%s: no PHY descriptor for LMAC%d",
		    __func__, lmacid);
		MDIO_UNLOCK(sc);
		return (ENXIO);
	}
	mii_sc = device_private(pd->miibus);

	mii_tick(mii_sc);
	if ((mii_sc->mii_media_status & (IFM_ACTIVE | IFM_AVALID)) ==
	    (IFM_ACTIVE | IFM_AVALID)) {
		/* Link is up */
		*link = 1;
	} else
		*link = 0;

	switch (IFM_SUBTYPE(mii_sc->mii_media_active)) {
	case IFM_10_T:
		*speed = 10;
		break;
	case IFM_100_TX:
		*speed = 100;
		break;
	case IFM_1000_T:
		*speed = 1000;
		break;
	default:
		/* IFM_NONE */
		*speed = 0;
	}

	if ((IFM_OPTIONS(mii_sc->mii_media_active) & IFM_FDX) != 0)
		*duplex = 1;
	else
		*duplex = 0;

	MDIO_UNLOCK(sc);

	return (0);
}

#if 0
static int
thunder_mdio_media_change(struct ifnet *ifp)
{
	// XXXNH
	return (EIO);
}
#endif

int
thunder_mdio_phy_connect(device_t dev, int lmacid, int phy)
{
	//XXXNH dev is phy device_t
	struct thunder_mdio_softc *sc;
	struct phy_desc *pd;
	int err;

	sc = device_private(dev);

	MDIO_LOCK(sc);
	pd = get_phy_desc(sc, lmacid);
	MDIO_UNLOCK(sc);
	if (pd == NULL) {
		pd = kmem_zalloc(sizeof(*pd), KM_SLEEP);
		struct mii_data *mii = &pd->mii;

		//mii->mii_ifp = ifp;
		mii->mii_readreg = thunder_mdio_read;
		mii->mii_writereg = thunder_mdio_write;
		mii->mii_statchg = thunder_mdio_statchg;
		//sc->sc_ec.ec_mii = mii;

//		sc->sc_ethercom.ec_mii = &sc->sc_mii;
//		ifmedia_init_with_lock(&mii->mii_media, IFM_IMASK, wm_gmii_mediachange,
//		    wm_gmii_mediastatus, sc->sc_core_lock);
//
// 		ifmedia_init(&mii->mii_media, IFM_IMASK,
// 		    thunder_mdio_media_change, thunder_mdio_media_status);
//
// 		pd->ifp = if_alloc(IFT_ETHER);
// 		if (pd->ifp == NULL) {
// 			kmem_free(pd, sizeof(*pd));
// 			return (ENOMEM);
// 		}
		pd->lmacid = lmacid;
	}

	mii_attach(dev, &pd->miibus, 0xffffffff, phy, MII_OFFSET_ANY, 0);

	if (err != 0) {
		device_printf(dev, "Could not attach PHY%d\n", phy);
		//if_free(pd->ifp);
		kmem_free(pd, sizeof(*pd));
		return (ENXIO);
	}

	MDIO_LOCK(sc);
	TAILQ_INSERT_TAIL(&sc->phy_desc_head, pd, phy_desc_list);
	MDIO_UNLOCK(sc);

	return (0);
}

int
thunder_mdio_phy_disconnect(device_t dev, int lmacid, int phy)
{
	struct thunder_mdio_softc *sc;
	struct phy_desc *pd;

	sc = device_private(dev);
	MDIO_LOCK(sc);

	pd = get_phy_desc(sc, lmacid);
	if (pd == NULL) {
		MDIO_UNLOCK(sc);
		return (EINVAL);
	}

	/* Remove this PHY descriptor from the list */
	TAILQ_REMOVE(&sc->phy_desc_head, pd, phy_desc_list);

	/* Detach miibus */
//	bus_generic_detach(dev);
//	device_delete_child(dev, pd->miibus);

	MDIO_UNLOCK(sc);
	/* Free memory under phy descriptor */
	kmem_free(pd, sizeof(*pd));

	return (0);
}

__strong_alias(LMAC_PHY_CONNECT,thunder_mdio_phy_connect);
__strong_alias(LMAC_PHY_DISCONNECT,thunder_mdio_phy_disconnect);
__strong_alias(LMAC_MEDIA_STATUS,thunder_mdio_media_status);
