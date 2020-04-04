/*	$NetBSD$	*/

/*
 * Copyright (c) 1996, 1997, 1998, 2001 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Jason R. Thorpe of the Numerical Aerospace Simulation Facility,
 * NASA Ames Research Center.
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

#ifndef _HPCMIPS_BUS_DEFS_H_
#define _HPCMIPS_BUS_DEFS_H_

/*
 * Bus address and size types
 */
typedef paddr_t bus_addr_t;
typedef psize_t bus_size_t;
#define	PRIxBUSADDR	PRIxPADDR
#define	PRIxBUSSIZE	PRIxPSIZE

typedef u_long	bus_space_handle_t;

#include <mips/locore.h>


#ifndef BUS_SPACE_MD_CALLS

#define	BUS_SPACE_MAP_CACHEABLE		0x01
#define	BUS_SPACE_MAP_LINEAR		0x02
#define	BUS_SPACE_MAP_PREFETCHABLE     	0x04

#define	BUS_SPACE_BARRIER_READ	0x01
#define	BUS_SPACE_BARRIER_WRITE	0x02

#ifndef BUS_SPACE_MD_TYPES
typedef struct bus_space_tag *bus_space_tag_t;
#endif

#define BUS_SPACE_ALIGNED_POINTER(p, t) ALIGNED_POINTER(p, t)

/*
 * bus space operaion table
 */
struct bus_space_ops {
	/* mapping/unmapping */
	int	  (*bs_map)(bus_space_tag_t, bus_addr_t, bus_size_t,
		      int, bus_space_handle_t *);
	void	  (*bs_unmap)(bus_space_tag_t, bus_space_handle_t, bus_size_t);
	int	  (*bs_subregion)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, bus_size_t, bus_space_handle_t *);

	/* allocation/deallocation */
	int	  (*bs_alloc)(bus_space_tag_t, bus_addr_t,
		      bus_addr_t, bus_size_t, bus_size_t, bus_size_t,
		      int, bus_addr_t *, bus_space_handle_t *);
	void	  (*bs_free)(bus_space_tag_t, bus_space_handle_t, bus_size_t);

	/* get kernel virtual address */
	void *	  (*bs_vaddr)(bus_space_tag_t, bus_space_handle_t);

	/* mmap bus space for user */
	paddr_t	  (*bs_mmap)(bus_space_tag_t, bus_addr_t, off_t, int, int);

	/* barrier */
	void	  (*bs_barrier)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, bus_size_t, int);

	/* probe */
	int	  (*bs_peek)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, size_t, void *);
	int	  (*bs_poke)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, size_t, u_int32_t);

	/* read (single) */
	u_int8_t  (*bs_r_1)(bus_space_tag_t, bus_space_handle_t, bus_size_t);
	u_int16_t (*bs_r_2)(bus_space_tag_t, bus_space_handle_t, bus_size_t);
	u_int32_t (*bs_r_4)(bus_space_tag_t, bus_space_handle_t, bus_size_t);
	u_int64_t (*bs_r_8)(bus_space_tag_t, bus_space_handle_t, bus_size_t);

	/* read multiple */
	void	  (*bs_rm_1)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int8_t *, bus_size_t);
	void	  (*bs_rm_2)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int16_t *, bus_size_t);
	void	  (*bs_rm_4)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int32_t *, bus_size_t);
	void	  (*bs_rm_8)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int64_t *, bus_size_t);

	/* read region */
	void	  (*bs_rr_1)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int8_t *, bus_size_t);
	void	  (*bs_rr_2)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int16_t *, bus_size_t);
	void	  (*bs_rr_4)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int32_t *, bus_size_t);
	void	  (*bs_rr_8)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int64_t *, bus_size_t);

	/* write (single) */
	void	  (*bs_w_1)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int8_t);
	void	  (*bs_w_2)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int16_t);
	void	  (*bs_w_4)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int32_t);
	void	  (*bs_w_8)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int64_t);

	/* write multiple */
	void	  (*bs_wm_1)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int8_t *, bus_size_t);
	void	  (*bs_wm_2)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int16_t *, bus_size_t);
	void	  (*bs_wm_4)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int32_t *, bus_size_t);
	void	  (*bs_wm_8)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int64_t *, bus_size_t);

	/* write region */
	void	  (*bs_wr_1)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int8_t *, bus_size_t);
	void	  (*bs_wr_2)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int16_t *, bus_size_t);
	void	  (*bs_wr_4)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int32_t *, bus_size_t);
	void	  (*bs_wr_8)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int64_t *, bus_size_t);

#ifdef BUS_SPACE_HAS_REAL_STREAM_METHODS
	/* read (single) stream */
	u_int8_t  (*bs_rs_1)(bus_space_tag_t, bus_space_handle_t, bus_size_t);
	u_int16_t (*bs_rs_2)(bus_space_tag_t, bus_space_handle_t, bus_size_t);
	u_int32_t (*bs_rs_4)(bus_space_tag_t, bus_space_handle_t, bus_size_t);
	u_int64_t (*bs_rs_8)(bus_space_tag_t, bus_space_handle_t, bus_size_t);

	/* read multiple stream */
	void	  (*bs_rms_1)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int8_t *, bus_size_t);
	void	  (*bs_rms_2)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int16_t *, bus_size_t);
	void	  (*bs_rms_4)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int32_t *, bus_size_t);
	void	  (*bs_rms_8)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int64_t *, bus_size_t);

	/* read region stream */
	void	  (*bs_rrs_1)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int8_t *, bus_size_t);
	void	  (*bs_rrs_2)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int16_t *, bus_size_t);
	void	  (*bs_rrs_4)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int32_t *, bus_size_t);
	void	  (*bs_rrs_8)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int64_t *, bus_size_t);

	/* write (single) stream */
	void	  (*bs_ws_1)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int8_t);
	void	  (*bs_ws_2)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int16_t);
	void	  (*bs_ws_4)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int32_t);
	void	  (*bs_ws_8)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int64_t);

	/* write multiple stream */
	void	  (*bs_wms_1)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int8_t *, bus_size_t);
	void	  (*bs_wms_2)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int16_t *, bus_size_t);
	void	  (*bs_wms_4)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int32_t *, bus_size_t);
	void	  (*bs_wms_8)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int64_t *, bus_size_t);

	/* write region stream */
	void	  (*bs_wrs_1)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int8_t *, bus_size_t);
	void	  (*bs_wrs_2)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int16_t *, bus_size_t);
	void	  (*bs_wrs_4)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int32_t *, bus_size_t);
	void	  (*bs_wrs_8)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, const u_int64_t *, bus_size_t);
#endif /* BUS_SPACE_HAS_REAL_STREAM_METHODS */

	/* set multiple */
	void	  (*bs_sm_1)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int8_t, bus_size_t);
	void	  (*bs_sm_2)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int16_t, bus_size_t);
	void	  (*bs_sm_4)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int32_t, bus_size_t);
	void	  (*bs_sm_8)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int64_t, bus_size_t);

	/* set region */
	void	  (*bs_sr_1)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int8_t, bus_size_t);
	void	  (*bs_sr_2)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int16_t, bus_size_t);
	void	  (*bs_sr_4)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int32_t, bus_size_t);
	void	  (*bs_sr_8)(bus_space_tag_t, bus_space_handle_t,
		      bus_size_t, u_int64_t, bus_size_t);

	/* copy */
	void	  (*bs_c_1)(bus_space_tag_t, bus_space_handle_t, bus_size_t,
		      bus_space_handle_t, bus_size_t, bus_size_t);
	void	  (*bs_c_2)(bus_space_tag_t, bus_space_handle_t, bus_size_t,
		      bus_space_handle_t, bus_size_t, bus_size_t);
	void	  (*bs_c_4)(bus_space_tag_t, bus_space_handle_t, bus_size_t,
		      bus_space_handle_t, bus_size_t, bus_size_t);
	void	  (*bs_c_8)(bus_space_tag_t, bus_space_handle_t, bus_size_t,
		      bus_space_handle_t, bus_size_t, bus_size_t);
};

#ifndef BUS_SPACE_MD_TYPES
#ifdef BUS_SPACE_MD_CALLS
typedef struct bus_space *bus_space_tag_t;
#endif /* BUS_SPACE_MD_CALLS */

/*
 *	bus_space_tag_t
 *
 *	bus space tag structure
 */
struct bus_space_tag {
	bus_space_tag_t bs_base;
	struct bus_space_ops bs_ops;
};
#endif /* ! BUS_SPACE_MD_TYPES */



#endif /*  BUS_SPACE_MD_CALLS */

















//#define _MIPS_NEED_BUS_DMA_BOUNCE
#include <mips/bus_dma_defs.h>


extern struct mips_bus_dma_tag hpcmips_default_bus_dma_tag;


#endif /* _HPCMIPS_BUS_DEFS_H_ */
