/*	$NetBSD: octeon_pipvar.h,v 1.3 2020/06/18 13:52:08 simonb Exp $	*/

/*
 * Copyright (c) 2007 Internet Initiative Japan, Inc.
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
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _OCTEON_PIPVAR_H_
#define _OCTEON_PIPVAR_H_


/* XXX */
struct octpip_softc {
	int			sc_port;
	bus_space_tag_t		sc_regt;
	bus_space_handle_t	sc_regh;
	int			sc_tag_type;
	int			sc_receive_group;
	size_t			sc_ip_offset;
#ifdef CNMAC_DEBUG
	struct evcnt		sc_ev_pipbeperr;
	struct evcnt		sc_ev_pipfeperr;
	struct evcnt		sc_ev_pipskprunt;
	struct evcnt		sc_ev_pipbadtag;
	struct evcnt		sc_ev_pipprtnxa;
	struct evcnt		sc_ev_pippktdrp;
#endif
};

/* XXX */
struct octpip_attach_args {
	int			aa_port;
	bus_space_tag_t		aa_regt;
	int			aa_tag_type;
	int			aa_receive_group;
	size_t			aa_ip_offset;
};

void		octpip_init(struct octpip_attach_args *,
		    struct octpip_softc **);
void		octpip_gbl_ctl_debug(struct octpip_softc *);
int		octpip_port_config(struct octpip_softc *);
void		octpip_prt_cfg_enable(struct octpip_softc *, uint64_t, int);
void		octpip_stats(struct octpip_softc *, struct ifnet *, int);
#ifdef CNMAC_DEBUG
void		octpip_int_enable(struct octpip_softc *, int);
void		octpip_dump(void);
void		octpip_dump_regs(void);
void		octpip_dump_stats(void);
#endif /* CNMAC_DEBUG */

#ifdef CNMAC_DEBUG
void		octpip_int_enable(struct octpip_softc *, int);
uint64_t	octpip_int_summary(struct octpip_softc *);
#endif /* CNMAC_DEBUG */

#endif /* _OCTEON_PIPVAR_H_ */
