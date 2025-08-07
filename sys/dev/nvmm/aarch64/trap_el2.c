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

#include "opt_arm_debug.h"

#include <sys/param.h>
#include <sys/types.h>

#include <aarch64/armreg.h>
#include <aarch64/frame.h>
#include <aarch64/machdep.h>

#include <dev/nvmm/nvmm.h>
#include <dev/nvmm/aarch64/nvmm_aarch64_internal.h>

/*
 * EL2 trap handler.
 *
 * This EL2 trap handler works on physical addresses.
 * Keep in mind that the code should not be KVA-dependent.
 */

void el2sync_el_low(struct trapframe *);
void el2irq_el_low(struct trapframe *);
void el2fiq_el_low(struct trapframe *);
void el2error_el_low(struct trapframe *);

struct nvmm_aarch64_state;

/*
 * uartprintf() is a simple printf() for debugging that depends only on
 * uartputc(), uartputs() and strlen(). uartputc() and uartputs() are defined
 * inside locore and are enabled only when the EARLYCONS option is specified.
 * If EARLYCONS is not defined, uartputc() will not work and uartprintf() will
 * not output anything.
 *
 * uartprintf() supports only 'x', 'u', 'd' and 's' formats and padding.
 * Any modifiers ('h','l','k','t','z','q') are ignored, and '\n' will be
 * replaced to '\r\n' internally.
 */
int
uartprintf(const char * restrict fmt, ...)
{
#ifdef EARLYCONS
	va_list ap;
	uint64_t v;
	int count, width, l;
	int flag_prefix;
	char buf[32], *p;
	char ch, padch;

	va_start(ap, fmt);

	count = 0;

	while ((ch = *fmt++) != '\0') {
		if (ch != '%') {
			if (ch == '\n')
				uartputc('\r');
			uartputc(ch);
			count++;
			continue;
		}

		p = &buf[sizeof(buf)];
		*--p = '\0';
		padch = ' ';
		width = 0;
		flag_prefix = 0;

		ch = *fmt++;
		if (ch == '\0')
			break;
		if (ch == '0') {
			padch = '0';
			ch = *fmt++;
			if (ch == '\0')
				break;
			width = 0;
		}
 widthloop:
		switch (ch) {
		case '0' ... '9':
			width *= 10;
			width += ch - '0';
			ch = *fmt++;
			if (ch == '\0')
				break;
			goto widthloop;

		case 'd':
		case 'u':
			v = va_arg(ap, uint64_t);
			do {
				*--p = (v % 10) + '0';
				v /= 10;
				width--;
			} while (v != 0);
			goto output;
		case 'p':
			flag_prefix = 1;
			ch = 'x';
			width -= 2;
			/* FALLTHRU */
		case 'x':
		case 'X':
			v = va_arg(ap, uint64_t);
			do {
				unsigned int x = (v & 15);
				if (x < 10)
					x += '0';
				else
					x += 'A' - 10 + (ch - 'X');
				*--p = x;
				v >>= 4;
				width--;
			} while (v != 0);
 output:
			for (; width > 0; width--) {
				uartputc(padch);
				count++;
			}

			if (flag_prefix) {
				uartputs("0x");
				count += 2;
			}

			uartputs(p);
			count += strlen(p);
			break;
		case 's':
			p = va_arg(ap, char *);
			l = strlen(p);
			width -= l;
			for (; width > 0; width--) {
				uartputc(padch);
				count++;
			}
			uartputs(p);
			count += l;
			break;
		case 'h':
		case 'l':
		case 'j':
		case 't':
		case 'z':
		case 'q':
			ch = *fmt++;
			if (ch == '\0')
				break;
			goto widthloop;
		}
	}

	va_end(ap);

	return count;
#else /* EARLYCONS */
	return 0;
#endif /* EARLYCONS */
}

void
dump_el2_trapframe(struct trapframe *tf)
{
	uartprintf("    pc=%016"PRIxREGISTER",   spsr=%016"PRIxREGISTER"\n",
	    tf->tf_pc, tf->tf_spsr);
	uartprintf("   esr=%016"PRIxREGISTER",    far=%016"PRIxREGISTER"\n",
	    tf->tf_esr, tf->tf_far);
	uartprintf("    x0=%016"PRIxREGISTER",     x1=%016"PRIxREGISTER"\n",
	    tf->tf_reg[0], tf->tf_reg[1]);
	uartprintf("    x2=%016"PRIxREGISTER",     x3=%016"PRIxREGISTER"\n",
	    tf->tf_reg[2], tf->tf_reg[3]);
	uartprintf("    x4=%016"PRIxREGISTER",     x5=%016"PRIxREGISTER"\n",
	    tf->tf_reg[4], tf->tf_reg[5]);
	uartprintf("    x6=%016"PRIxREGISTER",     x7=%016"PRIxREGISTER"\n",
	    tf->tf_reg[6], tf->tf_reg[7]);
	uartprintf("    x8=%016"PRIxREGISTER",     x9=%016"PRIxREGISTER"\n",
	    tf->tf_reg[8], tf->tf_reg[9]);
	uartprintf("   x10=%016"PRIxREGISTER",    x11=%016"PRIxREGISTER"\n",
	    tf->tf_reg[10], tf->tf_reg[11]);
	uartprintf("   x12=%016"PRIxREGISTER",    x13=%016"PRIxREGISTER"\n",
	    tf->tf_reg[12], tf->tf_reg[13]);
	uartprintf("   x14=%016"PRIxREGISTER",    x15=%016"PRIxREGISTER"\n",
	    tf->tf_reg[14], tf->tf_reg[15]);
	uartprintf("   x16=%016"PRIxREGISTER",    x17=%016"PRIxREGISTER"\n",
	    tf->tf_reg[16], tf->tf_reg[17]);
	uartprintf("   x18=%016"PRIxREGISTER",    x19=%016"PRIxREGISTER"\n",
	    tf->tf_reg[18], tf->tf_reg[19]);
	uartprintf("   x20=%016"PRIxREGISTER",    x21=%016"PRIxREGISTER"\n",
	    tf->tf_reg[20], tf->tf_reg[21]);
	uartprintf("   x22=%016"PRIxREGISTER",    x23=%016"PRIxREGISTER"\n",
	    tf->tf_reg[22], tf->tf_reg[23]);
	uartprintf("   x24=%016"PRIxREGISTER",    x25=%016"PRIxREGISTER"\n",
	    tf->tf_reg[24], tf->tf_reg[25]);
	uartprintf("   x26=%016"PRIxREGISTER",    x27=%016"PRIxREGISTER"\n",
	    tf->tf_reg[26], tf->tf_reg[27]);
	uartprintf("   x28=%016"PRIxREGISTER", fp=x29=%016"PRIxREGISTER"\n",
	    tf->tf_reg[28], tf->tf_reg[29]);
	uartprintf("lr=x30=%016"PRIxREGISTER",     sp=%016"PRIxREGISTER"\n",
	    tf->tf_reg[30],  tf->tf_sp);
}

static void
aarch64_guest_backtrace_fp(uint64_t spsr, uint64_t fp)
{
	vaddr_t fp_va;
	uint64_t *fp_pa;

	fp_va = fp;
	uartprintf("trace: fp=%016lx\n", fp_va);

	while (fp_va != 0) {
		fp_pa = (uint64_t *)aarch64_gva_to_pa(spsr, fp_va);
		if (fp_pa == 0)
			break;

		uartprintf("0x%016lx %016lx fp=%016lx\n", fp_pa[1], fp_pa[1], fp_va);
		fp_va = fp_pa[0];
	}

}

static void
aarch64_guest_backtrace_tf(struct trapframe *tf, struct trapframe *gtf)
{
	vaddr_t fp_va;
	uint64_t *fp_pa;

	fp_va = gtf->tf_reg[29];
	uartprintf("trace:\n"
	    "0x%016lx %016lx fp=%016lx\n", gtf->tf_reg[30], gtf->tf_reg[30], fp_va);

	while (fp_va != 0) {
		fp_pa = (uint64_t *)aarch64_gva_to_pa(tf->tf_spsr, fp_va);
		if (fp_pa == 0)
			break;

		uartprintf("0x%016lx %016lx fp=%016lx\n", fp_pa[1], fp_pa[1], fp_va);
		fp_va = fp_pa[0];
	}
}

static void
dump_guest_regs(struct trapframe *tf)
{
	dump_el2_trapframe(tf);
	uartprintf("esr_el1  =%016"PRIxREGISTER"\n", reg_esr_el1_read());
	uartprintf("far_el1  =%016"PRIxREGISTER"\n", reg_far_el1_read());
	uartprintf("elr_el1  =%016"PRIxREGISTER"\n", reg_elr_el1_read());
	uartprintf("hpfar_el2=%016"PRIxREGISTER"\n", reg_hpfar_el2_read());
	uartprintf("far_el2  =%016"PRIxREGISTER"\n", reg_far_el2_read());
	uartprintf("tf_far   =%016"PRIxREGISTER"\n", tf->tf_far);
	aarch64_guest_backtrace_fp(tf->tf_spsr, tf->tf_reg[29]);
	uartprintf("\n");
}

void
el2sync_el_low(struct trapframe *tf)
{
	const uint64_t esr = tf->tf_esr;
	const uint64_t eclass = __SHIFTOUT(esr, ESR_EC);

	// any EL1 can do this?? on, TPIDR_EL2 == 0 only?
	if (eclass == ESR_EC_HVC_A64) {
		if (reg_tpidr_el2_read() == 0) {
			/* hvc #n from host */
			switch (tf->tf_esr & 0xffff) {
			case 0:
				aarch64_el2_init(tf);
				break;
			case 1:
				aarch64_el2_vmenter(tf);
				break;
			case 0x100:
				aarch64_el2_maintain_ipa(tf);
				break;
			default:
				break;
			}
		} else {
			/* XXXXXXXXX: debug: hvc #n from guest. TODO: psci */
			uartprintf("%s: PC=%016"PRIx64" ESR_EL2=0x%08"PRIx64" (eclass=0x%"PRIx64")\n", __func__, tf->tf_pc, esr, eclass);
			dump_el2_trapframe(tf);

			// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX: <DEBUG>
			if ((tf->tf_esr & 0xffff) == 99) {
				/* analyze guest stacktrace */
				struct trapframe *gtf_pa = (struct trapframe *)aarch64_gva_to_pa(tf->tf_spsr, tf->tf_reg[0]);
				uartprintf("guest tf(va) %016lx -> (pa)%p\n", tf->tf_reg[0], gtf_pa);
				dump_el2_trapframe(gtf_pa);
				uartprintf("guest: esr_el1=%016"PRIxREGISTER"\n", reg_esr_el1_read());
				uartprintf("guest: far_el1=%016"PRIxREGISTER"\n", reg_far_el1_read());
				uartprintf("guest: elr_el1=%016"PRIxREGISTER"\n", reg_elr_el1_read());
				aarch64_guest_backtrace_tf(tf, gtf_pa);	/* XXX: dangerous */
			}
			// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX: </DEBUG>

			aarch64_el2_vmexit_trap(tf);
		}
	} else {
		if (nvmm_debug >= 2)
			uartprintf("%s: PC=%016"PRIx64" ESR_EL2=0x%08"PRIx64"(eclass=0x%"PRIx64") FAR_EL2=%016"PRIx64" IPA=%016"PRIx64" HPFAR=%016"PRIx64"(%016"PRIx64") PA=%016"PRIx64"\n",
			    __func__, tf->tf_pc, esr, eclass,
			    tf->tf_far, aarch64_gva_to_ipa(tf->tf_spsr, tf->tf_far),
			    reg_hpfar_el2_read(), ipa_hpfar_far(reg_hpfar_el2_read(), tf->tf_far), aarch64_gva_to_pa(tf->tf_spsr, tf->tf_far));

#if 0
		// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX DEBUG
		if (tf->tf_pc == 0x200)
			aarch64_guest_backtrace_fp(tf->tf_spsr, tf->tf_reg[29]);
#endif

#if 0
		//XXXXXXXXXXXXXXXXXXXXXXXX: HPFAR check DEBUG
		switch (eclass) {
		case ESR_EC_DATA_ABT_EL_LOW:
		case ESR_EC_INSN_ABT_EL_LOW:
			{
				paddr_t ipa0 = aarch64_gva_to_ipa(tf->tf_spsr, tf->tf_far);
				paddr_t ipa1 = ipa_hpfar_far(reg_hpfar_el2_read(), tf->tf_far);
				if (ipa0 != ipa1) {
					uartprintf("%s: invalid HPFAR: PC=%016lx ESR_EL2=0x%08lx(eclass=0x%lx) FAR_EL2=%016lx IPA=%016lx HPFAR=%016lx(%016lx) PA=%016lx\n",
					    __func__, tf->tf_pc, esr, eclass,
					    tf->tf_far, aarch64_gva_to_ipa(tf->tf_spsr, tf->tf_far),
					    reg_hpfar_el2_read(), ipa_hpfar_far(reg_hpfar_el2_read(), tf->tf_far),
					    aarch64_gva_to_pa(tf->tf_spsr, tf->tf_far));
					uartprintf("truly, ipa=%016lx ?\n", aarch64_get_fault_ipa(tf));

					dump_guest_regs(tf);
				}
			}
			break;
		}
#endif


		aarch64_el2_vmexit_trap(tf);
	}

}

void
el2irq_el_low(struct trapframe *tf)
{
	if (nvmm_debug >= 2)
		uartprintf("%s: PC=%016"PRIx64" ESR_EL2=0x%08"PRIx64" ESR_EL1=0x%08"PRIx64"\n", __func__, tf->tf_pc, tf->tf_esr, reg_esr_el1_read());

	aarch64_el2_vmexit_irq(tf);
}

void
el2fiq_el_low(struct trapframe *tf)
{
	if (nvmm_debug >= 2)
		uartprintf("%s: PC=%016"PRIx64" ESR_EL2=0x%08"PRIx64" ESR_EL1=0x%08"PRIx64"\n", __func__, tf->tf_pc, tf->tf_esr, reg_esr_el1_read());

	aarch64_el2_vmexit_irq(tf);
}

void
el2error_el_low(struct trapframe *tf)
{
	uartprintf("%s: PC=%016"PRIx64" ESR_EL2=0x%08"PRIx64" ESR_EL1=0x%08"PRIx64"\n", __func__, tf->tf_pc, tf->tf_esr, reg_esr_el1_read());
	dump_guest_regs(tf);

	aarch64_el2_vmexit_trap(tf);
}

extern u_long kern_vtopdiff;

#define bad_trap_el2(trapfunc)						\
void trapfunc(struct trapframe *);					\
void									\
trapfunc(struct trapframe *tf)						\
{									\
	uint64_t vpc = tf->tf_pc + kern_vtopdiff;			\
	uartprintf("EL2 trap: %s: PC=%016"PRIx64" (->va %016"PRIx64")"	\
	    " ESR=0x%08"PRIx64"\n",					\
	    __func__, tf->tf_pc, vpc, tf->tf_esr);			\
	uartprintf("tpidr_el2: %016"PRIx64"\n", reg_tpidr_el2_read());	\
	dump_el2_trapframe(tf);						\
	for (;;)							\
		asm("wfi");						\
}

bad_trap_el2(el2sync_el2t)
bad_trap_el2(el2irq_el2t)
bad_trap_el2(el2fiq_el2t)
bad_trap_el2(el2error_el2t)

bad_trap_el2(el2sync_el2h)
bad_trap_el2(el2irq_el2h)
bad_trap_el2(el2fiq_el2h)
bad_trap_el2(el2error_el2h)

bad_trap_el2(el2sync32_el_low)
bad_trap_el2(el2irq32_el_low)
bad_trap_el2(el2fiq32_el_low)
bad_trap_el2(el2error32_el_low)
