/*	$NetBSD$	*/

/*-
 * Copyright (c) 2023 Ryo Shimizu <ryo@nerv.org>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/bitops.h>
#include <sys/endian.h>

     #include <sys/types.h>
     #include <sys/param.h>
     #include <sys/time.h>
     #include <sys/uio.h>
     #include <sys/ktrace.h>



#include <assert.h>
#include <err.h>
#include <errno.h>
#include <inttypes.h>

#include <machine/vmparam.h>

#include <arm/armreg.h>

/*
 * The following accesses to MMIO are not supported because of troublesome
 * emulation. It is unlikely that this kind of access will actually occur.
 *
 * In the case where 0x1000 is normal mapped memory and 0x2000 is MMIO,
 *
 *   mov  x0,0x1ff8
 *   ldp  x1,x2,[x0]	// access to 0x1ff8-0x2008
 *
 * Within the fault trap, FAR_EL2 (fault address register) indicates 0x2000,
 * and x0 is 0x1ff8.
 *
 *   mov  x0,0x1ffe
 *   str  x1,[x0]	// unaligned access to 0x1ffe-0x2006
 *
 * in this case too, fault address indicate 0x2000.
 *
 * Also, access to MMIO using fp registers, dc zva, atomic instructions, and
 * additional instructions since ARMv8.1 are not supported at this time.
 */

__CTASSERT((int)NVMM_AARCH64_STATE_GPRS > 0);
__CTASSERT((int)NVMM_AARCH64_STATE_SPRS > 0);
__CTASSERT((int)NVMM_AARCH64_STATE_FPRS > 0);


//XXXNH need to check translation granule?
#define FAULT_ACROSS_A_PAGE(va, gpa)				\
	(((va) & PAGE_MASK) != ((gpa) & PAGE_MASK))
#define ACCESS_WITHIN_A_PAGE(addr, size)			\
	(((addr) & ~PAGE_MASK) == (((addr) + (size) - 1) & ~PAGE_MASK))

static inline bool
endian_eb_p(struct nvmm_vcpu *vcpu)
{
	uint64_t sctlr = vcpu->state->sprs[NVMM_AARCH64_SPR_SCTLR_EL1];
	uint64_t spsr = vcpu->state->sprs[NVMM_AARCH64_SPR_SPSR_EL1];

	if ((spsr & SPSR_A32) != 0)
		return ((spsr & SPSR_A32_E) != 0);

	switch (__SHIFTOUT(spsr, SPSR_M)) {
	case SPSR_M_EL1H:
	case SPSR_M_EL1T:
		return ((sctlr & SCTLR_EE) != 0);
	case SPSR_M_EL0T:
		return ((sctlr & SCTLR_E0E) != 0);
	}

	warnx("%s:%d: unsupported guest mode: SPSR_EL1: 0x%"PRIx64,
	    __func__, __LINE__, spsr);
	abort();
}

static void
nvmm_assist_mem_write(struct nvmm_machine *mach, struct nvmm_vcpu *vcpu,
    gpaddr_t gpa, uint64_t regdata, size_t size)
{
	struct nvmm_mem mem;
	bool guest_eb = endian_eb_p(vcpu);

	assert(size <= sizeof(uint64_t));

	/*
	 * regdata = 0x0123456789abcdef
	 * guest is LE, should be written:
	 *   size 1: gpa[0]      = ef
	 *   size 2: gpa[0,1]    = ef,cd
	 *   size 4: gpa[0,..,3] = ef,cd,ab,89
	 *   size 8: gpa[0,..,7] = ef,cd,ab,89,67,45,23,01
	 * guest is BE, should be written:
	 *   size 1: gpa[0]      = ef
	 *   size 2: gpa[0,1]    = cd,ef
	 *   size 4: gpa[0,..,3] = 89,ab,cd,ef
	 *   size 8: gpa[0,..,7] = 01,23,45,67,89,ab,cd,ef
	 */
	if (guest_eb)
		HTOBE64(regdata);
	else
		HTOLE64(regdata);

	mem.mach = mach;
	mem.vcpu = vcpu;
	mem.gpa = gpa;
	mem.write = true;
	mem.size = size;
	// XXXNH needs to be host endianness?
	if (guest_eb)
		mem.data = (uint8_t *)&regdata + 8 - size;
	else
		mem.data = (uint8_t *)&regdata;
	(*vcpu->cbs.mem)(&mem);
}

static uint64_t
nvmm_assist_mem_read(struct nvmm_machine *mach, struct nvmm_vcpu *vcpu,
    gpaddr_t gpa, size_t size)
{
	struct nvmm_mem mem;
	uint64_t regdata = 0;
	bool guest_eb = endian_eb_p(vcpu);

	assert(size <= sizeof(uint64_t));

	/*
	 * gpa[] = 01,23,45,67,89,ab,cd,ef
	 * guest is LE, should be read in:
	 *   size 1: regdata = 0x0000000000000001
	 *   size 2: regdata = 0x0000000000002301
	 *   size 4: regdata = 0x0000000067452301
	 *   size 8: regdata = 0xefcdab8967452301
	 * guest is BE, should be read in:
	 *   size 1: regdata = 0x0000000000000001
	 *   size 2: regdata = 0x0000000000000123
	 *   size 4: regdata = 0x0000000001234567
	 *   size 8: regdata = 0x0123456789abcdef
	 */
	mem.mach = mach;
	mem.vcpu = vcpu;
	mem.gpa = gpa;
	mem.write = false;
	mem.size = size;
	// XXXNH needs to be host endianness?
	if (guest_eb)
		mem.data = (uint8_t *)&regdata + 8 - size;
	else
		mem.data = (uint8_t *)&regdata;
	(*vcpu->cbs.mem)(&mem);

	if (guest_eb)
		BE64TOH(regdata);
	else
		LE64TOH(regdata);

	return regdata;
}

/*
 * instruction emulator for access of MMIO
 */
#define OPFUNC_DECL(func, a, b, c, d, e, f, g, h)		\
func(struct nvmm_machine *mach, struct nvmm_vcpu *vcpu,		\
    uint64_t a, uint64_t b, uint64_t c, uint64_t d,		\
    uint64_t e, uint64_t f, uint64_t g, uint64_t h)

#define UNUSED0 arg0 __unused
#define UNUSED1 arg1 __unused
#define UNUSED2 arg2 __unused
#define UNUSED3 arg3 __unused
#define UNUSED4 arg4 __unused
#define UNUSED5 arg5 __unused
#define UNUSED6 arg6 __unused
#define UNUSED7 arg7 __unused

#define OP0FUNC(func)						\
	static int						\
	OPFUNC_DECL(func,					\
	    UNUSED0, UNUSED1, UNUSED2, UNUSED3,			\
	    UNUSED4, UNUSED5, UNUSED6, UNUSED7)
#define OP1FUNC(func, a)					\
	static int						\
	OPFUNC_DECL(func, a,					\
	    UNUSED1, UNUSED2, UNUSED3, UNUSED4,			\
	    UNUSED5, UNUSED6, UNUSED7)
#define OP2FUNC(func, a, b)					\
	static int						\
	OPFUNC_DECL(func, a, b,					\
	    UNUSED2, UNUSED3, UNUSED4, UNUSED5,			\
	    UNUSED6, UNUSED7)
#define OP3FUNC(func, a, b, c)					\
	static int						\
	OPFUNC_DECL(func, a, b, c,				\
	    UNUSED3, UNUSED4, UNUSED5, UNUSED6,			\
	    UNUSED7)
#define OP4FUNC(func, a, b, c, d)				\
	static int						\
	OPFUNC_DECL(func, a, b, c, d,				\
	    UNUSED4, UNUSED5, UNUSED6, UNUSED7)
#define OP5FUNC(func, a, b, c, d, e)				\
	static int						\
	OPFUNC_DECL(func, a, b, c, d, e,			\
	    UNUSED5, UNUSED6, UNUSED7)
#define OP6FUNC(func, a, b, c, d, e, f)				\
	static int						\
	OPFUNC_DECL(func, a, b, c, d, e, f,			\
	    UNUSED6, UNUSED7)
#define OP7FUNC(func, a, b, c, d, e, f, g)			\
	static int						\
	OPFUNC_DECL(func, a, b, c, d, e, f, g,			\
	    UNUSED7)
#define OP8FUNC(func, a, b, c, d, e, f, g, h)			\
	static int						\
	OPFUNC_DECL(func, a, b, c, d, e, f, g, h)

#define REG_XZR_P(regno)	((regno) == 31)	/* is this the zero register? */
#define REG_SP_P(regno)		((regno) == 31)	/* is this the stack pointer? */

static inline int64_t
SignExtend(unsigned int bitwidth, uint64_t imm, unsigned int multiply)
{
	_DIAGASSERT(bitwidth <= 32);
	_DIAGASSERT(bitwidth > 0);
	_DIAGASSERT(bitwidth > 0);
	_DIAGASSERT(imm < (1UL << bitwidth));
	_DIAGASSERT(multiply <= 16);

	unsigned int shift = 64 - bitwidth;
	int64_t val = ((int64_t)(imm << shift)) >> shift;

	return val * multiply;
}

static inline unsigned int
nvmm_state_sp_reg(struct nvmm_aarch64_state *state)
{
	uint64_t spsr = state->sprs[NVMM_AARCH64_SPR_SPSR_EL1];

	if ((spsr & SPSR_A32) == 0) {
		if ((spsr & __BIT(0)) == 0)
			return NVMM_AARCH64_SPR_SP_EL0;
	} else {
		warnx("AA32 guest mode unsupported");
		abort();
	}

	return NVMM_AARCH64_SPR_SP_EL1;

}

static inline uint64_t
nvmm_register_n_get(struct nvmm_aarch64_state *state, unsigned int Rn)
{
	if (REG_SP_P(Rn)) {
		return state->sprs[nvmm_state_sp_reg(state)];
	}
	return state->gprs[NVMM_AARCH64_GPR_X0 + Rn];
}

static inline int
nvmm_register_n_set(struct nvmm_aarch64_state *state, unsigned int Rn, uint64_t va)
{
	if (REG_SP_P(Rn)) {
		state->sprs[nvmm_state_sp_reg(state)] = va;
		/* Don't actually need this as it's covered by PC, but... */
		return NVMM_AARCH64_STATE_SPRS;
	}
	state->gprs[NVMM_AARCH64_GPR_X0 + Rn] = va;
	return NVMM_AARCH64_STATE_GPRS;
}


// C6.2.109 DC
/* emul "dc zva,Rt" */
OP1FUNC(op_dc_zva, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	gpaddr_t gpa = exit->u.mem.gpa;
	struct nvmm_mem mem;
	static uint8_t zerodat[2048] __aligned(128);

	if (Rt == 31) {
		warnx("%s: dc zva with x31: PC=%016lx",
		    __func__, state->sprs[NVMM_AARCH64_SPR_PC]);
		return -1;	/* XX */
	}

	uint64_t va = state->gprs[NVMM_AARCH64_GPR_X0 + Rt];
	/*
	 * XXX: The value of dczid_el0 may be different for each CPU,
	 *      so we must refer to the dczid of the vcpu that executed
	 *      this instruction.
	 */
	uint64_t dcz_size = 4 << __SHIFTOUT(reg_dczid_el0_read(), DCZID_BS);


#if 0
	/* Extract sizing parameters directly from the virtualised Guest CPU state */
	uint64_t dczid = state->sprs[NVMM_AARCH64_SPR_DCZID_EL0];

	/* If DCZID_EL0.pZ is set, DC ZVA is prohibited on this core configuration */
	if (dczid & __BIT(4)) { /* DCZID_EL0_pZ_SHIFT */
		return -1;
	}

	uint64_t dcz_size = 4 << __SHIFTOUT(dczid, DCZID_BS);

	/* Strict safety guard protecting host buffer boundary constraints */
	if (dcz_size > sizeof(zerodat)) {
		warnx("%s: Guest requested DC ZVA size %lu exceeds hypervisor buffer limits",
		    __func__, dcz_size);
		return -1;
	}
#endif

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa)) {
		return -1;
	}
	if (!ACCESS_WITHIN_A_PAGE(gpa, dcz_size)) {
		return -1;
	}

	/*
	 * since the write exceeds 8 bytes, callback is called directly
	 * without nvmm_assist_mem_write().
	 */
	mem.mach = mach;
	mem.vcpu = vcpu;
	mem.gpa = gpa;
	mem.write = true;
	mem.size = dcz_size;
	mem.data = zerodat;
	(*vcpu->cbs.mem)(&mem);

	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return 0;
}

/* emul "ldrb Rt,[Rn],#imm9" */
OP3FUNC(op_ldrb_immpostidx, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t regdata;
	int updated = 0;

	/* Maybe the CPU handles this for us. */
	if (Rn == Rt && !REG_SP_P(Rn)) {
		// XXX: Rn and Rt can't be equal. When Rn is SP, Rt can be XZR.
		return -1;
	}

	uint64_t va = nvmm_register_n_get(state, Rn);

	regdata = nvmm_assist_mem_read(mach, vcpu, gpa, 1);
	if (!REG_XZR_P(Rt)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = regdata;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rn are updated? */
}

/* emul "ldrb Rt,[Rn,#imm9]!" */
OP3FUNC(op_ldrb_immpreidx, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t regdata;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);

	regdata = nvmm_assist_mem_read(mach, vcpu, gpa, 1);
	if (!REG_XZR_P(Rt)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = regdata;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rn are updated? */
}

/* emul "ldrh Rt,[Rn],#imm9" */
OP3FUNC(op_ldrh_immpostidx, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t regdata;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, 2))
		return -1;

	regdata = nvmm_assist_mem_read(mach, vcpu, gpa, 2);
	if (!REG_XZR_P(Rt)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = regdata;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rn are updated? */
}

/* emul "ldrh Rt,[Rn],#imm9" */
OP3FUNC(op_ldrh_immpreidx, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t regdata;
	int updated = 0;

	/* Maybe the CPU handles this for us. */
	if (Rn == Rt && !REG_SP_P(Rn)) {
		// XXX: Rn and Rt can't be equal. When Rn is SP, Rt can be XZR.
		return -1;
	}

	uint64_t va = nvmm_register_n_get(state, Rn);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, 2))
		return -1;

	regdata = nvmm_assist_mem_read(mach, vcpu, gpa, 2);
	if (!REG_XZR_P(Rt)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = regdata;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rn are updated? */
}

/* emul "ldrsw Rt,[Rn],#imm9" */
OP3FUNC(op_ldrsw_immpostidx, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t regdata;
	int updated = 0;

	/* Maybe the CPU handles this for us. */
	if (Rn == Rt && !REG_SP_P(Rn)) {
		// XXX: Rn and Rt can't be equal. When Rn is SP, Rt can be XZR.
		return -1;
	}

	uint64_t va = nvmm_register_n_get(state, Rn);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, 4))
		return -1;

	regdata = nvmm_assist_mem_read(mach, vcpu, gpa, 4);
	if (!REG_XZR_P(Rt)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] =
		    SignExtend(32, regdata, 1);
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rn are updated? */
}

/* emul "ldrsw Rt,[Rn,#imm9]!" */
OP3FUNC(op_ldrsw_immpreidx, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t regdata;
	int updated = 0;

	/* Maybe the CPU handles this for us. */
	if (Rn == Rt && !REG_SP_P(Rn)) {
		// XXX: Rn and Rt can't be equal. When Rn is SP, Rt can be XZR.
		return -1;
	}

	uint64_t va = nvmm_register_n_get(state, Rn);
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
	}

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, 4))
		return -1;

	regdata = nvmm_assist_mem_read(mach, vcpu, gpa, 4);
	if (!REG_XZR_P(Rt)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] =
		    SignExtend(32, regdata, 1);
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (imm9 != 0) {
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rn are updated? */
}

/* emul "ldrsb Rt,[Rn],#imm9" */
OP4FUNC(op_ldrsb_immpostidx, opc, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t regdata;
	int updated = 0;

	/* Maybe the CPU handles this for us. */
	if (Rn == Rt && !REG_SP_P(Rn)) {
		// XXX: Rn and Rt can't be equal. When Rn is SP, Rt can be XZR.
		return -1;
	}

	uint64_t va = nvmm_register_n_get(state, Rn);

	regdata = nvmm_assist_mem_read(mach, vcpu, gpa, 1);
	if (!REG_XZR_P(Rt)) {
		regdata = SignExtend(8, regdata, 1);
		if (opc & __BIT(0))
			regdata &= __BITS(31, 0);
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = regdata;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rn are updated? */
}

/* emul "ldrsb Rt,[Rn,#imm9]!" */
OP4FUNC(op_ldrsb_immpreidx, opc, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t regdata;
	int updated = 0;

	/* Maybe the CPU handles this for us. */
	if (Rn == Rt && !REG_SP_P(Rn)) {
		// XXX: Rn and Rt can't be equal. When Rn is SP, Rt can be XZR.
		return -1;
	}

	uint64_t va = nvmm_register_n_get(state, Rn);
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
		updated |= nvmm_register_n_set(state, Rn, va);
	}

	regdata = nvmm_assist_mem_read(mach, vcpu, gpa, 1);
	if (!REG_XZR_P(Rt)) {
		regdata = SignExtend(8, regdata, 1);
		if (opc & __BIT(0))
			regdata &= __BITS(31, 0);
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = regdata;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}

	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rn are updated? */
}

/* emul "ldrsh Rt,[Rn],#imm9" */
OP4FUNC(op_ldrsh_immpostidx, opc, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t regdata;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, 2))
		return -1;

	regdata = nvmm_assist_mem_read(mach, vcpu, gpa, 2);
	if (!REG_XZR_P(Rt)) {
		regdata = SignExtend(16, regdata, 1);
		if (opc & __BIT(0))
			regdata &= __BITS(31, 0);
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = regdata;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rn are updated? */
}

/* emul "ldrsh Rt,[Rn,#imm9]!" */
OP4FUNC(op_ldrsh_immpreidx, opc, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t regdata;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
	}

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, 2))
		return -1;

	regdata = nvmm_assist_mem_read(mach, vcpu, gpa, 2);
	if (!REG_XZR_P(Rt)) {
		regdata = SignExtend(16, regdata, 1);
		if (opc & __BIT(0))
			regdata &= __BITS(31, 0);
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = regdata;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (imm9 != 0) {
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rn are updated? */
}

/* emul "ldr Rt,[Rn],#imm9" */
OP4FUNC(op_ldr_immpostidx, sf, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	size_t bytes = (sf == 0) ? 4 : 8;
	uint64_t regdata;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, bytes))
		return -1;

	regdata = nvmm_assist_mem_read(mach, vcpu, gpa, bytes);
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
	}
	if (!REG_XZR_P(Rt)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = regdata;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (imm9 != 0) {
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rn are updated? */
}

/* emul "ldr Rt,[Rn,#imm9]!" */
OP4FUNC(op_ldr_immpreidx, sf, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	size_t bytes = (sf == 0) ? 4 : 8;
	uint64_t regdata;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
	}

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, bytes))
		return -1;

	regdata = nvmm_assist_mem_read(mach, vcpu, gpa, bytes);
	if (!REG_XZR_P(Rt)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = regdata;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (imm9 != 0) {
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rn are updated? */
}

/* emul "strb Rt,[Rn],#imm9" */
OP3FUNC(op_strb_immpostidx, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t regdata;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);

	regdata = REG_XZR_P(Rt) ? 0 : state->gprs[NVMM_AARCH64_GPR_X0 + Rt];
	regdata &= __BITS(7, 0);
	nvmm_assist_mem_write(mach, vcpu, gpa, regdata, 1);
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rn is updated? */
}

/* emul "strb Rt,[Rn,#imm9]!" */
OP3FUNC(op_strb_immpreidx, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t regdata;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
	}

	regdata = REG_XZR_P(Rt) ? 0 : state->gprs[NVMM_AARCH64_GPR_X0 + Rt];
	regdata &= __BITS(7, 0);
	nvmm_assist_mem_write(mach, vcpu, gpa, regdata, 1);

	if (imm9 != 0) {
		updated |= nvmm_register_n_set(state, Rn, va);
	}

	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rn is updated? */
}

/* emul "strh Rt,[Rn],#imm9" */
OP3FUNC(op_strh_immpostidx, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t regdata;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, 2))
		return -1;

	regdata = REG_XZR_P(Rt) ? 0 : state->gprs[NVMM_AARCH64_GPR_X0 + Rt];
	regdata &= __BITS(15, 0);
	nvmm_assist_mem_write(mach, vcpu, gpa, regdata, 2);
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rn is updated? */
}

/* emul "strh Rt,[Rn,#imm9]!" */
OP3FUNC(op_strh_immpreidx, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t regdata;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
	}

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, 2))
		return -1;

	regdata = REG_XZR_P(Rt) ? 0 : state->gprs[NVMM_AARCH64_GPR_X0 + Rt];
	regdata &= __BITS(15, 0);
	nvmm_assist_mem_write(mach, vcpu, gpa, regdata, 2);
	if (imm9 != 0) {
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rn is updated? */
}

/* emul "str Rt,[Rn],#imm9" */
OP4FUNC(op_str_immpostidx, sf, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	size_t bytes = (sf == 0) ? 4 : 8;
	uint64_t regdata;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, bytes))
		return -1;

	regdata = REG_XZR_P(Rt) ? 0 : state->gprs[NVMM_AARCH64_GPR_X0 + Rt];
	if (sf == 0) {
		regdata &= __BITS(31, 0);
	}
	nvmm_assist_mem_write(mach, vcpu, gpa, regdata, bytes);

	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rn is updated? */
}

/* emul "str Rt,[Rn,#imm9]!" */
OP4FUNC(op_str_immpreidx, sf, imm9, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	size_t bytes = (sf == 0) ? 4 : 8;
	uint64_t regdata;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);
	if (imm9 != 0) {
		va += SignExtend(9, imm9, 1);
	}

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, (sf == 0) ? 4 : 8))
		return -1;

	regdata = REG_XZR_P(Rt) ? 0 : state->gprs[NVMM_AARCH64_GPR_X0 + Rt];
	if (sf == 0) {
		regdata &= __BITS(31, 0);
	}
	nvmm_assist_mem_write(mach, vcpu, gpa, regdata, bytes);
	if (imm9 != 0) {
		updated |= nvmm_register_n_set(state, Rn, va);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rn is updated */
}

/* --- new --- */

/* emul "ldr Rt, [Rn, #imm12]" */
OP4FUNC(op_ldr_immunsigned, sf, imm12, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	size_t bytes = (sf == 0) ? 4 : 8;
	uint64_t regdata;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);
	if (imm12 != 0) {
		va += SignExtend(12, imm12, bytes);
	}

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, bytes))
		return -1;

	regdata = nvmm_assist_mem_read(mach, vcpu, gpa, bytes);
	if (!REG_XZR_P(Rt)) {
		if (sf == 0) {
			regdata &= __BITS(31, 0);
		}
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = regdata;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt is updated? */
}

/* emul "str Rt, [Rn, #imm12]" */
OP4FUNC(op_str_immunsigned, sf, imm12, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	size_t bytes = (sf == 0) ? 4 : 8;
	uint64_t regdata;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);
	if (imm12 != 0) {
		va += SignExtend(12, imm12, bytes);
	}

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, bytes))
		return -1;

	regdata = REG_XZR_P(Rt) ? 0 : state->gprs[NVMM_AARCH64_GPR_X0 + Rt];
	if (sf == 0) {
		regdata &= __BITS(31, 0);
	}
	nvmm_assist_mem_write(mach, vcpu, gpa, regdata, bytes);

	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rn is updated */
}


/* ------ here down ----- */

/* emul "stp Rt,Rt2,[Rn],#imm7" */
OP5FUNC(op_stp_postidx, sf, imm7, Rt2, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t reg1, reg2;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, (sf == 0) ? 8 : 16))
		return -1;

	reg1 = REG_XZR_P(Rt) ? 0 : state->gprs[NVMM_AARCH64_GPR_X0 + Rt];
	reg2 = REG_XZR_P(Rt2) ? 0 : state->gprs[NVMM_AARCH64_GPR_X0 + Rt2];
	if (sf == 0) {
		reg1 &= __BITS(31, 0);
		reg2 &= __BITS(31, 0);
		nvmm_assist_mem_write(mach, vcpu, gpa, reg1, 4);
		nvmm_assist_mem_write(mach, vcpu, gpa + 4, reg2, 4);
	} else {
		nvmm_assist_mem_write(mach, vcpu, gpa, reg1, 8);
		nvmm_assist_mem_write(mach, vcpu, gpa + 8, reg2, 8);
	}
	if (imm7 != 0) {
		va += SignExtend(7, imm7, (sf == 0) ? 4 : 8);
		state->gprs[NVMM_AARCH64_GPR_X0 + Rn] = va;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rn is updated? */
}

/* emul "stp Rt,Rt2,[Rn,#imm7]!" */
OP5FUNC(op_stp_preidx, sf, imm7, Rt2, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t reg1, reg2;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);
	if (imm7 != 0) {
		va += SignExtend(7, imm7, (sf == 0) ? 4 : 8);
		state->gprs[NVMM_AARCH64_GPR_X0 + Rn] = va;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, (sf == 0) ? 8 : 16))
		return -1;

	reg1 = REG_XZR_P(Rt) ? 0 : state->gprs[NVMM_AARCH64_GPR_X0 + Rt];
	reg2 = REG_XZR_P(Rt2) ? 0 : state->gprs[NVMM_AARCH64_GPR_X0 + Rt2];
	if (sf == 0) {
		reg1 &= __BITS(31, 0);
		reg2 &= __BITS(31, 0);
		nvmm_assist_mem_write(mach, vcpu, gpa, reg1, 4);
		nvmm_assist_mem_write(mach, vcpu, gpa + 4, reg2, 4);
	} else {
		nvmm_assist_mem_write(mach, vcpu, gpa, reg1, 8);
		nvmm_assist_mem_write(mach, vcpu, gpa + 8, reg2, 8);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rn is updated? */
}

/* emul "stp Rt,Rt2,[Rn,#imm7]" */
OP5FUNC(op_stp_signed, sf, imm7, Rt2, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t reg1, reg2;

	uint64_t va = nvmm_register_n_get(state, Rn);
	warnx("%s: va=%016"PRIx64", gpa=%016"PRIx64, __func__, va, gpa);
	if (imm7 != 0)
		va += SignExtend(7, imm7, (sf == 0) ? 4 : 8);
	warnx("imm7 = %ld, va=%016"PRIx64"\n", imm7, va);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, (sf == 0) ? 8 : 16))
		return -1;

	reg1 = REG_XZR_P(Rt) ? 0 : state->gprs[NVMM_AARCH64_GPR_X0 + Rt];
	reg2 = REG_XZR_P(Rt2) ? 0 : state->gprs[NVMM_AARCH64_GPR_X0 + Rt2];
	if (sf == 0) {
		reg1 &= __BITS(31, 0);
		reg2 &= __BITS(31, 0);
		nvmm_assist_mem_write(mach, vcpu, gpa, reg1, 4);
		nvmm_assist_mem_write(mach, vcpu, gpa + 4, reg2, 4);
	} else {
		nvmm_assist_mem_write(mach, vcpu, gpa, reg1, 8);
		nvmm_assist_mem_write(mach, vcpu, gpa + 8, reg2, 8);
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return 0;	/* No registers that need to be updated */
}

/* emul "ldp Rt,Rt2,[Rn],#imm7" */
OP5FUNC(op_ldp_postidx, sf, imm7, Rt2, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t reg1, reg2;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, (sf == 0) ? 8 : 16))
		return -1;

	if (sf == 0) {
		reg1 = nvmm_assist_mem_read(mach, vcpu, gpa, 4);
		reg2 = nvmm_assist_mem_read(mach, vcpu, gpa + 4, 4);
	} else {
		reg1 = nvmm_assist_mem_read(mach, vcpu, gpa, 8);
		reg2 = nvmm_assist_mem_read(mach, vcpu, gpa + 8, 8);
	}
	if (imm7 != 0) {
		va += SignExtend(7, imm7, (sf == 0) ? 4 : 8);
		state->gprs[NVMM_AARCH64_GPR_X0 + Rn] = va;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (!REG_XZR_P(Rt)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = reg1;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (!REG_XZR_P(Rt2)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt2] = reg2;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rt2,Rn are updated? */
}

/* emul "ldp Rt,Rt2,[Rn,#imm7]!" */
OP5FUNC(op_ldp_preidx, sf, imm7, Rt2, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t reg1, reg2;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, (sf == 0) ? 8 : 16))
		return -1;

	if (imm7 != 0) {
		va += SignExtend(7, imm7, (sf == 0) ? 4 : 8);
		state->gprs[NVMM_AARCH64_GPR_X0 + Rn] = va;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (sf == 0) {
		reg1 = nvmm_assist_mem_read(mach, vcpu, gpa, 4);
		reg2 = nvmm_assist_mem_read(mach, vcpu, gpa + 4, 4);
	} else {
		reg1 = nvmm_assist_mem_read(mach, vcpu, gpa, 8);
		reg2 = nvmm_assist_mem_read(mach, vcpu, gpa + 8, 8);
	}
	if (!REG_XZR_P(Rt)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = reg1;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (!REG_XZR_P(Rt2)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt2] = reg2;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rt2,Rn are updated? */
}

/* emul "ldp Rt,Rt2,[Rn,#imm7]" */
OP5FUNC(op_ldp_signed, sf, imm7, Rt2, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	uint64_t reg1, reg2;
	int updated = 0;

	uint64_t va = nvmm_register_n_get(state, Rn);
	if (imm7 != 0)
		va += SignExtend(7, imm7, (sf == 0) ? 4 : 8);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, (sf == 0) ? 8 : 16))
		return -1;

	if (sf == 0) {
		reg1 = nvmm_assist_mem_read(mach, vcpu, gpa, 4);
		reg2 = nvmm_assist_mem_read(mach, vcpu, gpa + 4, 4);
	} else {
		reg1 = nvmm_assist_mem_read(mach, vcpu, gpa, 8);
		reg2 = nvmm_assist_mem_read(mach, vcpu, gpa + 8, 8);
	}
	if (!REG_XZR_P(Rt)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = reg1;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (!REG_XZR_P(Rt2)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt2] = reg2;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rt2 are updated? */
}

/* emul "stp Rt,Rt2,[Rn],#imm7" */
OP5FUNC(op_stp_simd_postidx, opc, imm7, Rt2, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	__uint128_t reg1, reg2;
	size_t bytes;
	int updated = 0;

	switch (opc) {
	case 0:
		/* St[12] */
		bytes = 4;
		break;
	case 1:
		/* Dt[12] */
		bytes = 8;
		break;
	case 2:
		/* Qt[12] */
		bytes = 16;
		break;
	case 3:
	default:
		return -1;
	}

	uint64_t va = nvmm_register_n_get(state, Rn);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, bytes * 2))
		return -1;

	reg1 = state->fprs[NVMM_AARCH64_FPR_V0 + Rt].u128[0];
	reg2 = state->fprs[NVMM_AARCH64_FPR_V0 + Rt2].u128[0];

	switch (opc) {
	case 0:
		nvmm_assist_mem_write(mach, vcpu, gpa, reg1, 4);
		nvmm_assist_mem_write(mach, vcpu, gpa + 4, reg2, 4);
		break;
	case 1:
		nvmm_assist_mem_write(mach, vcpu, gpa, reg1, 8);
		nvmm_assist_mem_write(mach, vcpu, gpa + 8, reg2, 8);
		break;
	case 2:
		// XXXNH endian
		nvmm_assist_mem_write(mach, vcpu, gpa, reg1, 8);
		nvmm_assist_mem_write(mach, vcpu, gpa + 8, reg1 >> 64, 8);
		nvmm_assist_mem_write(mach, vcpu, gpa + 16, reg2, 8);
		nvmm_assist_mem_write(mach, vcpu, gpa + 24, reg2 >> 64, 8);
		break;
	case 3:
	default:
		return -1;
	}
	if (imm7 != 0) {
		va += SignExtend(7, imm7, bytes);
		state->gprs[NVMM_AARCH64_GPR_X0 + Rn] = va;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rn is updated? */
}

/* emul "stp Rt,Rt2,[Rn,#imm7]!" */
OP5FUNC(op_stp_simd_preidx, opc, imm7, Rt2, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	__uint128_t reg1, reg2;
	size_t bytes;
	int updated = 0;

	switch (opc) {
	case 0:
		/* St[12] */
		bytes = 4;
		break;
	case 1:
		/* Dt[12] */
		bytes = 8;
		break;
	case 2:
		/* Qt[12] */
		bytes = 16;
		break;
	case 3:
	default:
		return -1;
	}

	uint64_t va = nvmm_register_n_get(state, Rn);
	if (imm7 != 0) {
		va += SignExtend(7, imm7, bytes);
		state->gprs[NVMM_AARCH64_GPR_X0 + Rn] = va;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, bytes * 2))
		return -1;

	reg1 = state->fprs[NVMM_AARCH64_FPR_V0 + Rt].u128[0];
	reg2 = state->fprs[NVMM_AARCH64_FPR_V0 + Rt2].u128[0];

	switch (opc) {
	case 0:
		nvmm_assist_mem_write(mach, vcpu, gpa, reg1, 4);
		nvmm_assist_mem_write(mach, vcpu, gpa + 4, reg2, 4);
		break;
	case 1:
		nvmm_assist_mem_write(mach, vcpu, gpa, reg1, 8);
		nvmm_assist_mem_write(mach, vcpu, gpa + 8, reg2, 8);
		break;
	case 2:
		// XXXNH endian
		nvmm_assist_mem_write(mach, vcpu, gpa, reg1, 8);
		nvmm_assist_mem_write(mach, vcpu, gpa + 8, reg1 >> 64, 8);
		nvmm_assist_mem_write(mach, vcpu, gpa + 16, reg2, 8);
		nvmm_assist_mem_write(mach, vcpu, gpa + 24, reg2 >> 64, 8);
		break;
	default:
		return -1;
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rn is updated? */
}

/* emul "stp Rt,Rt2,[Rn,#imm7]" */
OP5FUNC(op_stp_simd_signed, opc, imm7, Rt2, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	__uint128_t reg1, reg2;
	size_t bytes;

	switch (opc) {
	case 0:
		/* St[12] */
		bytes = 4;
		break;
	case 1:
		/* Dt[12] */
		bytes = 8;
		break;
	case 2:
		/* Qt[12] */
		bytes = 16;
		break;
	case 3:
	default:
		return -1;
	}

	uint64_t va = nvmm_register_n_get(state, Rn);
	if (imm7 != 0)
		va += SignExtend(7, imm7, bytes);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, bytes * 2))
		return -1;
	reg1 = state->fprs[NVMM_AARCH64_FPR_V0 + Rt].u128[0];
	reg2 = state->fprs[NVMM_AARCH64_FPR_V0 + Rt2].u128[0];

	switch (opc) {
	case 0:
		nvmm_assist_mem_write(mach, vcpu, gpa, reg1, 4);
		nvmm_assist_mem_write(mach, vcpu, gpa + 4, reg2, 4);
		break;
	case 1:
		nvmm_assist_mem_write(mach, vcpu, gpa, reg1, 8);
		nvmm_assist_mem_write(mach, vcpu, gpa + 8, reg2, 8);
		break;
	}

	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return 0;	/* No registers that need to be updated */
}

/* emul "ldp Rt,Rt2,[Rn],#imm7" */
OP5FUNC(op_ldp_simd_postidx, opc, imm7, Rt2, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	__uint128_t reg1, reg2;
	int updated = 0;
	size_t bytes;

	switch (opc) {
	case 0:
		/* St[12] */
		bytes = 4;
		break;
	case 1:
		/* Dt[12] */
		bytes = 8;
		break;
	case 2:
		/* Qt[12] */
		bytes = 16;
		break;
	case 3:
	default:
		return -1;
	}

	uint64_t va = nvmm_register_n_get(state, Rn);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, bytes))
		return -1;

	switch (opc) {
	case 0:
		reg1 = nvmm_assist_mem_read(mach, vcpu, gpa, 4);
		reg2 = nvmm_assist_mem_read(mach, vcpu, gpa + 4, 4);
		break;
	case 1:
		reg1 = nvmm_assist_mem_read(mach, vcpu, gpa, 8);
		reg2 = nvmm_assist_mem_read(mach, vcpu, gpa + 8, 8);
		break;
	case 2:
		// XXXNH
	default:
		reg1 = 0;
		reg2 = 0;
	}
	if (imm7 != 0) {
		va += SignExtend(7, imm7, bytes);
		state->gprs[NVMM_AARCH64_GPR_X0 + Rn] = va;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}

	state->fprs[NVMM_AARCH64_FPR_V0 + Rt].u128[0] = reg1;
	state->fprs[NVMM_AARCH64_FPR_V0 + Rt2].u128[0] = reg2;
	updated |= NVMM_AARCH64_STATE_FPRS;

	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;
}

/* emul "ldp Rt,Rt2,[Rn,#imm7]!" */
OP5FUNC(op_ldp_simd_preidx, opc, imm7, Rt2, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	__uint128_t reg1, reg2;
	size_t bytes;
	int updated = 0;

	switch (opc) {
	case 0:
		/* St[12] */
		bytes = 4;
		break;
	case 1:
		/* Dt[12] */
		bytes = 8;
		break;
	case 2:
		/* Qt[12] */
		bytes = 16;
		break;
	case 3:
	default:
		return -1;
	}

	uint64_t va = nvmm_register_n_get(state, Rn);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, bytes))
		return -1;

	if (imm7 != 0) {
		va += SignExtend(7, imm7, bytes);
		state->gprs[NVMM_AARCH64_GPR_X0 + Rn] = va;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}

	reg1 = 0;
	reg2 = 0;
	switch (opc) {
	case 0:
		reg1 = nvmm_assist_mem_read(mach, vcpu, gpa, 4);
		reg2 = nvmm_assist_mem_read(mach, vcpu, gpa + 4, 4);
		break;
	case 1:
		reg1 = nvmm_assist_mem_read(mach, vcpu, gpa, 8);
		reg2 = nvmm_assist_mem_read(mach, vcpu, gpa + 8, 8);
		break;
	case 2:
		// XXXNH
		break;
	case 3:
	default:
		break;
	}
	if (!REG_XZR_P(Rt)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = reg1;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (!REG_XZR_P(Rt2)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt2] = reg2;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rt2,Rn are updated? */
}

/* emul "ldp Rt,Rt2,[Rn,#imm7]" */
OP5FUNC(op_ldp_simd_signed, opc, imm7, Rt2, Rn, Rt)
{
	warnx("%s: pc=%#018lx\n",
	    __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC]);
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	__uint128_t reg1, reg2;
	size_t bytes;
	int updated = 0;

	switch (opc) {
	case 0:
		/* St[12] */
		bytes = 4;
		break;
	case 1:
		/* Dt[12] */
		bytes = 8;
		break;
	case 2:
		/* Qt[12] */
		bytes = 16;
		break;
	case 3:
	default:
		return -1;
	}

	uint64_t va = nvmm_register_n_get(state, Rn);
	if (imm7 != 0)
		va += SignExtend(7, imm7, bytes);

	/* XXX: illegal alignment access is not supported */
	if (FAULT_ACROSS_A_PAGE(va, gpa))
		return -1;
	if (!ACCESS_WITHIN_A_PAGE(gpa, bytes))
		return -1;

	reg1 = 0;
	reg2 = 0;
	switch (opc) {
	case 0:
	case 1:
		reg1 = nvmm_assist_mem_read(mach, vcpu, gpa, bytes);
		reg2 = nvmm_assist_mem_read(mach, vcpu, gpa + bytes, bytes);
		break;
	case 2:
		//XXXNH
		break;
	default:
	}

	if (!REG_XZR_P(Rt)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt] = reg1;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	if (!REG_XZR_P(Rt2)) {
		state->gprs[NVMM_AARCH64_GPR_X0 + Rt2] = reg2;
		updated |= NVMM_AARCH64_STATE_GPRS;
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;

	return updated;	/* Rt,Rt2 are updated? */
}

struct bitmask {
	uint8_t lo;
	uint8_t hi;
};

struct insn_info {
	uint32_t mask;
	uint32_t pattern;
#define INSN_MAXARG	8
	struct bitmask bitinfo[INSN_MAXARG];
	OPFUNC_DECL(int (*opfunc),,,,,,,,);
	bool is_simd;
};

#define BM(_lo, _hi) { .lo = _lo, .hi = _hi}

/* bit positions of arg in opecode. { bitpos, biwidth } */
#define FMT_RT			{ BM( 0,  4) }
#define FMT_IMM9_RN_RT		{ BM(12, 20), BM( 5,  9), BM( 0,  4) }
#define FMT_OPC_IMM9_RN_RT	{ BM(22, 22), BM(12, 20), BM( 5,  9), BM(0, 4) }
#define FMT_SF_IMM9_RN_RT	{ BM(30, 30), BM(12, 20), BM( 5,  9), BM(0, 4) }
#define FMT_SF_IMM7_RT2_RN_RT	{ BM(31, 31), BM(15, 21), BM(10, 14), BM(5, 9), BM( 0, 4) }
#define FMT_SF_IMM12_RN_RT	{ BM(30, 30), BM(10, 21), BM( 5,  9), BM(0, 4) }
#define FMT_OPC_IMM7_RT2_RN_RT	{ BM(30, 31), BM(15, 21), BM(10, 14), BM(5, 9), BM( 0, 4) }

// XXXNH really should do opc, v, l, imm7, Rt2, Rn, Rt, cf Load/store register pair (post-indexed) C4-568


static const struct insn_info insn_tables[] = {
 /* mask,      pattern,    opcode format,               opfunc                  simd? */
 /* ---------  ----------  ---------------------------	----------------------  ----- */
 { 0xffffffe0, 0xd50b7420, FMT_RT,			op_dc_zva,		false },

 /* --- Byte operations (Explicitly checks size == 00 via 0xffe...) --- */
 { 0xffe00c00, 0x38400400, FMT_IMM9_RN_RT,		op_ldrb_immpostidx,	false },
 { 0xffe00c00, 0x38400c00, FMT_IMM9_RN_RT,		op_ldrb_immpreidx,	false },
 { 0xffe00c00, 0x38000400, FMT_IMM9_RN_RT,		op_strb_immpostidx,	false },
 { 0xffe00c00, 0x38000c00, FMT_IMM9_RN_RT,		op_strb_immpreidx,	false },

 /* --- Halfword operations (Explicitly checks size == 01 via 0xffe...) --- */
 { 0xffe00c00, 0x78400400, FMT_IMM9_RN_RT,		op_ldrh_immpostidx,	false },
 { 0xffe00c00, 0x78400c00, FMT_IMM9_RN_RT,		op_ldrh_immpreidx,	false },
 { 0xffe00c00, 0x78000400, FMT_IMM9_RN_RT,		op_strh_immpostidx,	false },
 { 0xffe00c00, 0x78000c00, FMT_IMM9_RN_RT,		op_strh_immpreidx,	false },

 /* --- Signed Word operations (Explicitly checks size == 10, opc == 10 via 0xffe...) --- */
 { 0xffe00c00, 0xb8800400, FMT_IMM9_RN_RT,		op_ldrsw_immpostidx,	false },
 { 0xffe00c00, 0xb8800c00, FMT_IMM9_RN_RT,		op_ldrsw_immpreidx,	false },

 /* --- Signed Byte/Halfword operations (Explicitly checks opc via 0xffa...) --- */
 { 0xffa00c00, 0x38800400, FMT_OPC_IMM9_RN_RT,		op_ldrsb_immpostidx,	false },
 { 0xffa00c00, 0x38800c00, FMT_OPC_IMM9_RN_RT,		op_ldrsb_immpreidx,	false },
 { 0xffa00c00, 0x78800400, FMT_OPC_IMM9_RN_RT,		op_ldrsh_immpostidx,	false },
 { 0xffa00c00, 0x78800c00, FMT_OPC_IMM9_RN_RT,		op_ldrsh_immpreidx,	false },

 /* --- Standard Word/Doubleword operations (Uses 0xbf7... to handle size 10 vs 11 via sf) --- */
 { 0xbf700c00, 0x38400400, FMT_SF_IMM9_RN_RT,		op_ldr_immpostidx,	false },
 { 0xbf700c00, 0x38400c00, FMT_SF_IMM9_RN_RT,		op_ldr_immpreidx,	false },
 { 0xbf700c00, 0x38000400, FMT_SF_IMM9_RN_RT,		op_str_immpostidx,	false },
 { 0xbf700c00, 0x38000c00, FMT_SF_IMM9_RN_RT,		op_str_immpreidx,	false },



/* mask,      pattern,    opcode format,               opfunc                  simd? */
 /* ---------  ----------  ---------------------------	----------------------  ----- */
 /* --- Unsigned Immediate Offset (imm12) Variants --- */
// { 0xbf700000, 0x39400000, FMT_SF_IMM12_RN_RT,		op_ldrb_immunsigned,	false }, // LDRB/LDR(sb) 32/64
// { 0xbf700000, 0x39000000, FMT_SF_IMM12_RN_RT,		op_strb_immunsigned,	false }, // STRB
// { 0xbf700000, 0x79400000, FMT_SF_IMM12_RN_RT,		op_ldrh_immunsigned,	false }, // LDRH/LDR(sh) 32/64
// { 0xbf700000, 0x79000000, FMT_SF_IMM12_RN_RT,		op_strh_immunsigned,	false }, // STRH
 { 0xbf700000, 0xb9400000, FMT_SF_IMM12_RN_RT,		op_ldr_immunsigned,	false }, // LDR (32/64-bit)
 { 0xbf700000, 0xb9000000, FMT_SF_IMM12_RN_RT,		op_str_immunsigned,	false }, // STR (32/64-bit)
// { 0xbf700000, 0xb9800000, FMT_SF_IMM12_RN_RT,		op_ldrsw_immunsigned,	false }, // LDRSW (Signed Word)




 { 0x7fc00000, 0x28800000, FMT_SF_IMM7_RT2_RN_RT,	op_stp_postidx,		false },
 { 0x7fc00000, 0x29800000, FMT_SF_IMM7_RT2_RN_RT,	op_stp_preidx,		false },
 { 0x7fc00000, 0x29000000, FMT_SF_IMM7_RT2_RN_RT,	op_stp_signed,		false },
 { 0x7fc00000, 0x28c00000, FMT_SF_IMM7_RT2_RN_RT,	op_ldp_postidx,		false },
 { 0x7fc00000, 0x29c00000, FMT_SF_IMM7_RT2_RN_RT,	op_ldp_preidx,		false },
 { 0x7fc00000, 0x29400000, FMT_SF_IMM7_RT2_RN_RT,	op_ldp_signed,		false },

 { 0x0ff00000, 0x0c800000, FMT_OPC_IMM7_RT2_RN_RT,      op_stp_simd_postidx,	true },
 { 0x0ff00000, 0x0d800000, FMT_OPC_IMM7_RT2_RN_RT,      op_stp_simd_preidx,	true },
 { 0x0ff00000, 0x0c000000, FMT_OPC_IMM7_RT2_RN_RT,      op_stp_simd_signed,	true },

 { 0x0ff00000, 0x0cc00000, FMT_OPC_IMM7_RT2_RN_RT,      op_ldp_simd_postidx,	true },
 { 0x0ff00000, 0x0dc00000, FMT_OPC_IMM7_RT2_RN_RT,      op_ldp_simd_preidx,	true },
 { 0x0ff00000, 0x0c400000, FMT_OPC_IMM7_RT2_RN_RT,      op_ldp_simd_signed,	true },
};

static int
nvmm_assist_mem_aarch64_emul(struct nvmm_machine *mach, struct nvmm_vcpu *vcpu,
    uint64_t *update)
{
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	const uint32_t insn = exit->insn;
	uint64_t args[INSN_MAXARG] = { 0 };
	unsigned int i, j;
	int emul_ret = -1;

	for (i = 0; i < __arraycount(insn_tables); i++) {
		if ((insn & insn_tables[i].mask) == insn_tables[i].pattern)
			break;
	}
	if (i == __arraycount(insn_tables)) {
		warnx("%s: unsupported instruction: "
		    "inst=0x%08"PRIx32", accessed gpa=0x%016"PRIx64, __func__, insn, gpa);
		errno = ENODEV;
		return -1;
	}

	uint32_t tmp = insn;
	utrace(__func__, &tmp, sizeof(tmp));
	warnx("%s: pc=%#018lx, insn=%#08x gpa=0x%016"PRIx64 "\n", __func__, vcpu->state->sprs[NVMM_AARCH64_SPR_PC], insn, gpa);
	/* extract operands */
	for (j = 0; j < INSN_MAXARG; j++) {
		const uint8_t lo = insn_tables[i].bitinfo[j].lo;
		const uint8_t hi = insn_tables[i].bitinfo[j].hi;

		if (lo == 0 && hi == 0)
			break;

		args[j] = __SHIFTOUT(insn, __BITS(hi, lo));
	}

	if (insn_tables[i].is_simd) {
		int ret = nvmm_vcpu_getstate(mach, vcpu, NVMM_AARCH64_STATE_FPRS);
		if (ret == -1)
			return -1;
	}

	emul_ret = insn_tables[i].opfunc(mach, vcpu,
	    args[0], args[1], args[2], args[3],
	    args[4], args[5], args[6], args[7]);
	if (emul_ret < 0) {
		errno = ENODEV;
		return -1;
	}

	*update = emul_ret | NVMM_AARCH64_STATE_SPRS;

	return 0;
}

static int
nvmm_assist_mem_aarch32_emul(struct nvmm_machine *mach, struct nvmm_vcpu *vcpu,
    uint64_t *update)
{
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	const gpaddr_t gpa = exit->u.mem.gpa;
	const uint32_t insn = exit->insn;

	/* XXX: not supported (yet?) */
	warnx("%s: aarch32 is not supported: "
	    "inst=0x%08"PRIx32", accessed gpa=0x%016"PRIx64, __func__, insn, gpa);
	errno = ENODEV;
	return -1;
}

static int
nvmm_assist_mem_aarch64_esr(struct nvmm_machine *mach, struct nvmm_vcpu *vcpu,
    uint64_t esr, uint64_t *update)
{
	struct nvmm_aarch64_state *state = vcpu->state;
	const struct nvmm_vcpu_exit *exit = vcpu->exit;
	int ret = 0;

	/* sas := [0,1,2,3], size := [1,2,4,8] */
	const uint64_t sas = __SHIFTOUT(esr, ESR_ISS_DATAABORT_SAS);
	const unsigned int size = __BIT(sas);
	const unsigned int regno = __SHIFTOUT(esr, ESR_ISS_DATAABORT_SRT);
	const gpaddr_t gpa = exit->u.mem.gpa;

	/* XXX: access across multiple pages is not supported */
	if (!ACCESS_WITHIN_A_PAGE(gpa, size))
		return -1;

	if (__SHIFTOUT(esr, ESR_ISS_DATAABORT_WnR)) {
		/* store */
		uint64_t regdata = REG_XZR_P(regno) ? 0 :
		    state->gprs[NVMM_AARCH64_GPR_X0 + regno];

		/* Wt or Xt */
		if (__SHIFTOUT(esr, ESR_ISS_DATAABORT_SF) == 0)
			regdata &= __BITS(31, 0);

		nvmm_assist_mem_write(mach, vcpu, gpa, regdata, size);
	} else {
		/* load */
		uint64_t regdata = nvmm_assist_mem_read(mach, vcpu, gpa, size);

		/* sign extend */
		// XXXNH
		if (__SHIFTOUT(esr, ESR_ISS_DATAABORT_SSE)) {
			if (regdata & __BIT(__BIT(sas) * 8 - 1)) {
				regdata |= __BITS(63, 0) << (__BIT(sas) * 8);
			}
		}

		/* resiger width. Wt or Xt? */
		if (__SHIFTOUT(esr, ESR_ISS_DATAABORT_SF) == 0)
			regdata &= __BITS(31, 0);

		if (!REG_XZR_P(regno)) {
			state->gprs[NVMM_AARCH64_GPR_X0 + regno] = regdata;
			*update |= NVMM_AARCH64_STATE_GPRS;
		}
	}
	state->sprs[NVMM_AARCH64_SPR_PC] += 4;
	*update |= NVMM_AARCH64_STATE_SPRS;

	return ret;
}

int
nvmm_assist_mem(struct nvmm_machine *mach, struct nvmm_vcpu *vcpu)
{
	struct nvmm_vcpu_exit *exit = vcpu->exit;
	const uint64_t esr_el2 = exit->esr;
	uint64_t update = 0;
	int ret;

	if (__predict_false(exit->reason != NVMM_VCPU_EXIT_MEMORY)) {
		errno = EINVAL;
		return -1;
	}

	/* XXX */
	if (exit->insn == 0) {
		/* XXX: fetch it ourselves from GPA -> HVA */
		struct nvmm_aarch64_state *state = vcpu->state;
		warnx("%s: instruction is zero: ESR=%016lx, PC=%016lx, GPA=%016lx, prot=0x%x", __func__, exit->esr, state->sprs[NVMM_AARCH64_SPR_PC], exit->u.mem.gpa, exit->u.mem.prot);
		goto assist_failure;
	}

	if (__SHIFTOUT(esr_el2, ESR_ISS_DATAABORT_EA) != 0)
		goto assist_failure;

	/* XXX */
	if (__SHIFTOUT(esr_el2, ESR_ISS_DATAABORT_CM) != 0) {
		warnx("%s: cache op is not supported: inst=%08"PRIx32,
		    __func__, exit->insn);
		goto assist_failure;
	}

	/* XXX */
	if (__SHIFTOUT(esr_el2, ESR_ISS_DATAABORT_AR) != 0) {
		warnx("%s: acquire/release read/write is not supported: "
		    "inst=%08"PRIx32, __func__, exit->insn);
		goto assist_failure;
	}

	/* load x0-x31, and PC, SPSR, SCTLR, ... */
	ret = nvmm_vcpu_getstate(mach, vcpu,
	    NVMM_AARCH64_STATE_SPRS | NVMM_AARCH64_STATE_GPRS);
	if (ret == -1)
		return -1;

	if (__SHIFTOUT(esr_el2, ESR_ISS_DATAABORT_ISV) == 0) {
		if (vcpu->state->sprs[NVMM_AARCH64_SPR_SPSR_EL1] & SPSR_A32)
			ret = nvmm_assist_mem_aarch32_emul(mach, vcpu, &update);
		else
			ret = nvmm_assist_mem_aarch64_emul(mach, vcpu, &update);
		if (ret < 0)
			goto assist_failure;
		return nvmm_vcpu_setstate(mach, vcpu, update);
	}

	ret = nvmm_assist_mem_aarch64_esr(mach, vcpu, esr_el2, &update);
	if (ret == 0)
		return nvmm_vcpu_setstate(mach, vcpu, update);

 assist_failure:
	//XXX DEBUG
	{
		struct nvmm_aarch64_state *state = vcpu->state;
		warnx("%s: unsupported instruction: ESR=%016lx, PC=%016lx, insn=%08x, GPA=%016lx, prot=0x%x", __func__, exit->esr, state->sprs[NVMM_AARCH64_SPR_PC], exit->insn, exit->u.mem.gpa, exit->u.mem.prot);
	}

	/* inject SError to guest */
	vcpu->event->type = NVMM_VCPU_EVENT_SERROR;
	nvmm_vcpu_inject(mach, vcpu);
	return -1;
}

int
nvmm_gva_to_gpa(struct nvmm_machine *mach, struct nvmm_vcpu *vcpu,
    gvaddr_t gva, gpaddr_t *gpa, nvmm_prot_t *prot)
{
	/* XXX: TODO */
	errno = ENODEV;
	return -1;
}
