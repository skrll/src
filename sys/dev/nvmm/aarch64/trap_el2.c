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

#include <aarch64/frame.h>
#include <aarch64/machdep.h>

/*
 * EL2 trap handler.
 *
 * This EL2 trap handler works on physical addresses.
 * Keep in mind that the code should not be KVA-dependent.
 */

void el2sync_el1(struct trapframe *);
char *uartputs(const char *);
int uartprintf(const char * restrict, ...);

/*
 * uartprintf() is a simple printf() for debugging that depends only on
 * uartputc(), uartputs() and strlen(). uartputc() and uartputs() are defined
 * inside locore and are enabled only when the EARLYCONS option is specified.
 * If EARLYCONS is not defined, uartputc() will not work and uartprintf() will
 * not output anything.
 *
 * uartprintf() supports only 'x', 'u', 'd' and 's' formats and padding.
 * Any modifiers ('h','l','k','t','z','q') are ignored, and '\n' will be
 * replaced to '\r\n' by internally.
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

static void
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

void
el2sync_el1(struct trapframe *tf)
{
	const uint32_t esr = tf->tf_esr;
	const uint32_t eclass = __SHIFTOUT(esr, ESR_EC);

	uartprintf("%s: ESR_EL2=0x%08x (eclass=0x%x)\n", __func__, esr, eclass);

	dump_el2_trapframe(tf);

	switch (eclass) {
	case ESR_EC_UNKNOWN:
	case ESR_EC_SERROR:
	case ESR_EC_WFX:
	case ESR_EC_ILL_STATE:
	case ESR_EC_BTE_A64:
	case ESR_EC_SYS_REG:
	case ESR_EC_SVC_A64:
	case ESR_EC_HVC_A64:
	case ESR_EC_SMC_A64:
		break;
	case ESR_EC_INSN_ABT_EL0:
	case ESR_EC_INSN_ABT_EL1:
	case ESR_EC_DATA_ABT_EL0:
	case ESR_EC_DATA_ABT_EL1:
		break;
	case ESR_EC_PC_ALIGNMENT:
	case ESR_EC_SP_ALIGNMENT:
		break;
	case ESR_EC_FP_ACCESS:
	case ESR_EC_FP_TRAP_A64:
		break;
	case ESR_EC_BRKPNT_EL0:
	case ESR_EC_BRKPNT_EL1:
	case ESR_EC_SW_STEP_EL0:
	case ESR_EC_SW_STEP_EL1:
	case ESR_EC_WTCHPNT_EL0:
	case ESR_EC_WTCHPNT_EL1:
	case ESR_EC_BKPT_INSN_A64:
		break;
	default:
		break;
	}

}
