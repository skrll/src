/*	$NetBSD$	*/

/*-
 * Copyright (c) 2012, 2017, 2018 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Matt Thomas of 3am Software Foundry and Nick Hudson.
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

#include "opt_arm_debug.h"
#include "opt_console.h"
#include "opt_cpuoptions.h"
#include "opt_cputypes.h"
#include "opt_fdt.h"
#include "opt_multiprocessor.h"

#include <sys/cdefs.h>

#include <arm/asm.h>
#include <arm/armreg.h>
#include "assym.h"

#define INIT_ARM_STACK_SIZE	8192

#if defined(CONSADDR) && defined(CONADDR)
#error Only one of CONSADDR and CONADDR should be defined
#endif

#if defined(CONSADDR)
#define START_CONSADDR	CONSADDR
#endif
#if defined(CONADDR)
#define START_CONSADDR	CONADDR
#endif

#ifdef VERBOSE_INIT_ARM
#define XPUTC(n)	mov r0, n; bl uartputc
#define VPRINTF(string)	bl generic_vprint; .asciz string; .align 2
#define VPRINTX(regno)	mov r0, regno; bl generic_printx
#else
#define XPUTC(c)	/* nothing */
#define VPRINTF(string)	/* nothing */
#define VPRINTX(regno)	/* nothing */
#endif

#ifdef FDT
#define MD_CPU_HATCH	arm_fdt_cpu_hatch
#endif

/*
 * A generic kernel start routine.
 *
 * At this point, this code has been loaded into SDRAM and the MMU maybe on or
 * maybe off.  Assume identity mapping (VA=PA) if MMU is on.
 *
 * Reference linux boot code (MMU off) and rpi boot protocol
 *
 */

	.text

ENTRY_NP(generic_start)

	// ARMv7 only?!?
#ifdef __ARMEB__
	setend	be			/* force big endian */
#endif

	/* disable IRQs/FIQs. */
	cpsid	if

	adr	r9, generic_start
	ldr	r10, =generic_start
	sub	r10, r10, r9

	ldr	r9, =svcstk
	add	r9, #INIT_ARM_STACK_SIZE * MAXCPUS
	sub	sp, r9, r10

	mov	r4, r0
	mov	r5, r1
	mov	r6, r2
	mov	r7, r3

	// We can now call functions

	VPRINTF("\n\rpc   :")
	VPRINTX(pc)

	VPRINTF("\n\roff  :")
	VPRINTX(r10)

	VPRINTF("\n\rsp   :")
	VPRINTX(sp)

	ldr	r8, =kern_vtopdiff
	sub	r8, r8, r10
	str	r10, [r8]

	ldr	r8, =(L1_S_SIZE - 1)
	ands	r9, r10, r8
	bne	arm_bad_vtopdiff
	/*
	 * store uboot arguments to uboot_args[4]
	 */
	ldr	r8, =uboot_args
	sub	r8, r8, r10

	str	r4, [r8, #(4*0)]
	str	r5, [r8, #(4*1)]
	str	r6, [r8, #(4*2)]
	str	r7, [r8, #(4*3)]

#if defined(FDTBASE)
	/*
	 * ARM boot protocol has FDT address in r2
	 */
	VPRINTF("\n\rfdt  :")
	ldr	r8, =fdt_addr_r
	sub	r8, r8, r10
	str	r6, [r8]
	mov	r9, r6		// Save fdt_addr_r for mapping later

	VPRINTX(r6)
#endif

#ifdef VERBOSE_INIT_ARM
	VPRINTF("\n\rttb0 :")
	mrc	p15, 0, r0, c2, c0, 0	// TTBR0 read
	VPRINTX(r0)
	VPRINTF("\n\rttb1 :")
	mrc	p15, 0, r0, c2, c0, 1	// TTBR1 read
	VPRINTX(r0)
	VPRINTF("\n\rttcr :")
	mrc	p15, 0, r0, c2, c0, 2	// TTBCR read
	VPRINTX(r0)
	VPRINTF("\n\r")
#endif

	// Let's work out what sort of CPU we're running on
	mrc	p15, 0, r0, c0, c0, 0	/* Get MIDR */
	and	r0, #0xf0000
#if defined(_ARM_ARCH_7)
	cmp	r0, #0xf0000
	beq	generic_startv7
#endif
#if defined(_ARM_ARCH_6)
	cmp	r0, #0x60000
	beq	generic_startv6
#endif
arm_cpu_unknown:
	VPRINTF("\n\rpanic: unknown midr (")
	mrc	p15, 0, r0, c0, c0, 0	/* Get MIDR */
	VPRINTX(r0)
	VPRINTF(")\n\r")
1:	b	1b
arm_bad_vtopdiff:
	VPRINTF("\n\rpanic: vtop not L1_FRAME aligned (")
	VPRINTX(r10)
	VPRINTF(")\n\r")
1:	b	 1b
ASEND(generic_start)

	.ltorg

	.section ".init_pagetable", "aw", %nobits
	.p2align  14 /* 16KiB aligned */
	.global ARM_BOOTSTRAP_LxPT
ARM_BOOTSTRAP_LxPT:
TEMP_L1_TABLE:
	.space  L1_TABLE_SIZE
TEMP_L1_TABLE_END:

	.text
	.align  2


	// Use callee saved registers
	R_L1TABLE	.req r4
	R_VA		.req r5
	R_PA		.req r6
	R_NSEC		.req r7
	R_ATTR		.req r8


#if defined(_ARM_ARCH_7)
/*
 * Registers at this point are:
 *
 * r4	argument 0
 * r5	argument 1
 * r6	argument 2
 * r7	argument 3
 * r10	virtual to physical offset
 */
generic_startv7:

	.arch		armv7a
	.arch_extension	sec
	.arch_extension	virt

	VPRINTF("v7   :")

	bl	armv7_init

	/*
	 * Initialise the l1pt for identity mapping of the kernel with caches off.
	 * This will be updated further with additional mappings to form the
	 * bootstrap table.
	 */
	ldr	R_L1TABLE, =TEMP_L1_TABLE
	sub	R_L1TABLE, r10

	// PA of kernel rounded down to nearest L1_S boundary
	adr	R_PA, generic_start
	ldr	r0, =(L1_S_SIZE - 1)
	bic	R_PA, R_PA, r0

	// attribute to map kernel - run without L1_S_CACHEABLE
	ldr	R_ATTR, =(L1_S_PROTO_armv7 | L1_S_APv7_KRW)
	bl	arm_boot_l1pt_init

	/*
	 * Set up a preliminary mapping in the MMU to allow us to run
	 * at KERNEL_BASE with caches off.
	 */
	ldr	R_L1TABLE, =TEMP_L1_TABLE
	sub	R_L1TABLE, r10

	ldr	r0, =KERNEL_BASE	// kernel start VA
	ldr	r1, =TEMP_L1_TABLE_END	// kernel end VA
	add	r1, #L1_S_SIZE		// Make sure we have 1M to grow into

	ldr	r2, =(L1_S_SIZE - 1)
	add	r1, r2			// Round end VA to 1M boumdary step 1

	bic	r0, r2			// round kernel start down to 1M boundary
	add	r1, r2
	bic	r1, r2			// round kernel end up to 1M boundary
	sub	r1, r0			// kernel size plus 1M to grow into
	mov	R_NSEC, r1, lsr #(L1_S_SHIFT)


	ldr	R_VA, =KERNEL_BASE
	// PA of kernel rounded down to nearest L1_S boundary
	adr	R_PA, generic_start
	bic	R_PA, r2

	// attribute to map kernel - run without L1_S_CACHEABLE
	ldr	R_ATTR, =(L1_S_PROTO_armv7 | L1_S_APv7_KRW)
	bl	arm_boot_l1pt_entry

#if defined(FDTBASE)
	VPRINTF("DTB")

	/*
	 * Add DTB identity mapping (1MB) from originaly r2 (but saved in r9)
	 */
	movw	r0, #:lower16:(L1_S_SIZE - 1)		/* align DTB PA to 1M */
	movt	r0, #:upper16:(L1_S_SIZE - 1)
	bic	R_VA, r9, r0
	mov	R_PA, R_VA
	mov	R_NSEC, #1				/* 1MB mapping */

	movw	R_ATTR, #:lower16:(L1_S_PROTO_armv7 | L1_S_APv7_KRW | L1_S_V6_XN)
	movt	R_ATTR, #:upper16:(L1_S_PROTO_armv7 | L1_S_APv7_KRW | L1_S_V6_XN)

	movw	R_L1TABLE, #:lower16:TEMP_L1_TABLE
	movt	R_L1TABLE, #:upper16:TEMP_L1_TABLE
	sub	R_L1TABLE, r10
	bl	arm_boot_l1pt_entry
#endif

#if defined(START_CONSADDR)
	/* If CONSADDR exists add its identity mapping (1MB) */
	VPRINTF("\n\rCONSADDR")
	movw	r0, #:lower16:(L1_S_SIZE - 1)		/* align DTB PA to 1M */
	movt	r0, #:upper16:(L1_S_SIZE - 1)
	ldr	R_VA, =START_CONSADDR
	bic	R_VA, R_VA, r0
	mov	R_PA, R_VA
	mov	R_NSEC, #1

	movw	R_ATTR, #:lower16:(L1_S_PROTO_armv7 | L1_S_APv7_KRW | L1_S_V6_XN)
	movt	R_ATTR, #:upper16:(L1_S_PROTO_armv7 | L1_S_APv7_KRW | L1_S_V6_XN)

	ldr	R_L1TABLE, =TEMP_L1_TABLE
	sub	R_L1TABLE, r10
	bl	arm_boot_l1pt_entry
#endif

	XPUTC(#'M')
//	movw	r0, #:lower16:TEMP_L1_TABLE
//	movt	r0, #:upper16:TEMP_L1_TABLE
//	sub	r0, r10
//	bl	generic_dumpl1pt

	/*
	 * Turn on the MMU.  Return to new enabled address space.
	 */
	movw	r0, #:lower16:TEMP_L1_TABLE
	movt	r0, #:upper16:TEMP_L1_TABLE
	sub	r0, r10

	// Just return to 'start'?
	movw	lr, #:lower16:1f
	movt	lr, #:upper16:1f
	b	armv7_mmuinit
1:

	VPRINTF("go\n\r")

	/*
	 * Jump to start in locore.S, which in turn will call initarm and main.
	 */
	b	start

	/* NOTREACHED */
	.ltorg

#endif

#if defined(_ARM_ARCH_6)


generic_startv6:
	// TODO

	.ltorg

#endif


// Set up a preliminary mapping in the MMU to allow us to run at KERNEL_BASE.
//
// On Entry
//
// R_L1TABLE	is the PA of the temporary L1PT
// R_ATTR	is the attribute bits to set for each section mapping
//
// No R_VA/R_PA/R_NSEC needed here as we use KERNEL_BASE and
// TEMP_L1_TABLE_END to calculate the initial direct VA:PA mapping

// We push r0 to maintain stack alignment only
arm_boot_l1pt_init:
	push	{r0, lr}

	// Start address to clear memory.
	mov	r0, R_L1TABLE

	// Zero the entire table so all virtual addresses are invalid.
	add	r1, r0, #L1_TABLE_SIZE	// Ending address
	mov	r2, #0
	mov	r3, #0
1:	stmia	r0!, {r2-r3}
	stmia	r0!, {r2-r3}		// 16 bytes per loop
	cmp	r0, r1
	blt	1b

	// Calculate the size of the kernel in L1_S_SIZE sections
	ldr	r2, =(L1_S_SIZE - 1)
	ldr	r0, =KERNEL_BASE
	ldr	r1, =TEMP_L1_TABLE_END
	add	r1, r2

	bic	r0, r2		// round kernel start down to L1_S_SIZE boundary
	bic	r1, r2		// round kernel end up to L1_S_SIZE boundary
	sub	r1, r0

	mov	R_NSEC, r1, lsr #(L1_S_SHIFT)

	// identity mapping for size of kernel
	adr	R_VA, generic_start
	bic	R_VA, r2
	mov	R_PA, R_VA

	pop	{r0, lr}

// We push r0 to maintain stack alignment only
arm_boot_l1pt_entry:
	push	{r0, lr}

	VPRINTF("\n\r")
	VPRINTX(R_L1TABLE)

	VPRINTF(" va:")
	VPRINTX(R_VA)

	VPRINTF(" pa:")
	VPRINTX(R_PA)

	VPRINTF(" nsec:")
	VPRINTX(R_NSEC)

	VPRINTF(" attr:")
	VPRINTX(R_ATTR)

	VPRINTF("\n\r")
	lsr	R_VA, R_VA, #L1_S_SHIFT
	orr	R_PA, R_ATTR

2:
	VPRINTX(R_L1TABLE)
	XPUTC('[')
	VPRINTX(R_VA)
	XPUTC(']')
	XPUTC('=')
	VPRINTX(R_PA)

	ldr	r0, [R_L1TABLE, R_VA, lsl #2]
	cmp	r0, #0
	cmpne	r0, R_PA
	bne	arm_boot_overlap

	str	R_PA, [R_L1TABLE, R_VA, lsl #2]
	add	R_VA, R_VA, #1
	add	R_PA, R_PA, #(L1_S_SIZE)

	VPRINTF("\n\r")

	subs	R_NSEC, R_NSEC, #1
	bhi	2b

	pop	{r0, pc}

	.unreq	R_VA
	.unreq	R_PA
	.unreq	R_NSEC
	.unreq	R_ATTR
	.unreq	R_L1TABLE

arm_boot_overlap:
	VPRINTF("\n\rpanic: overlapping mappings\n\r")
3:
	b	3b
#if defined(_ARM_ARCH_7)

//
// SCTLR register initialization values
//
#ifdef __ARMEL__
#define CPU_CONTROL_EX_BEND_SET		0
#else
#define CPU_CONTROL_EX_BEND_SET		CPU_CONTROL_EX_BEND
#endif
#ifdef ARM32_DISABLE_ALIGNMENT_FAULTS
#define CPU_CONTROL_AFLT_ENABLE_CLR	CPU_CONTROL_AFLT_ENABLE
#define CPU_CONTROL_AFLT_ENABLE_SET	0
#else
#define CPU_CONTROL_AFLT_ENABLE_CLR	0
#define CPU_CONTROL_AFLT_ENABLE_SET	CPU_CONTROL_AFLT_ENABLE
#endif
#ifdef ARM_MMU_EXTENDED_XXX
#define CPU_CONTROL_XP_ENABLE_CLR	0
#define CPU_CONTROL_XP_ENABLE_SET	CPU_CONTROL_XP_ENABLE
#else
#define CPU_CONTROL_XP_ENABLE_CLR	CPU_CONTROL_XP_ENABLE
#define CPU_CONTROL_XP_ENABLE_SET	0
#endif


// bits to set in the Control Register
//
#define CPU_CONTROL_SET			( 	\
	CPU_CONTROL_MMU_ENABLE		|	\
	CPU_CONTROL_UNAL_ENABLE		|	\
	CPU_CONTROL_EX_BEND_SET		|	\
	CPU_CONTROL_AFLT_ENABLE_SET	|	\
	CPU_CONTROL_XP_ENABLE_SET	|	\
	0)

// bits to clear in the Control Register
//
#define CPU_CONTROL_CLR			(	\
	CPU_CONTROL_AFLT_ENABLE_CLR	|	\
	CPU_CONTROL_XP_ENABLE_CLR	|	\
	0)


//
// Perform the initialization of the Cortex core required by NetBSD.
//
// Probably best to turn off ic/dc and perform invalidation(?)
//
// registers used:
//
// r0 - r8 and r12

armv7_init:

	.arch		armv7a
	.arch_extension	sec
	.arch_extension	virt

	mov	r4, lr
	mov	r5, sp

	/*
	 * Leave HYP mode and move into supervisor mode with IRQs/FIQs
	 * disabled.
	 */
	mrs	r0, cpsr
	and	r0, r0, #(PSR_MODE)	/* Mode is in the low 5 bits of CPSR */
	teq	r0, #(PSR_HYP32_MODE)	/* Hyp Mode? */
	bne	1f

	XPUTC('h')

	mov	sp, #0

	/* Set CNTVOFF to 0 */
	mov	r1, #0
	mcrr	p15, 4, r1, r1, c14

	/* Ensure that IRQ, and FIQ will be disabled after eret */
	mrs	r0, cpsr
	bic	r0, r0, #(PSR_MODE)
	orr	r0, r0, #(PSR_SVC32_MODE)
	orr	r0, r0, #(I32_bit | F32_bit)
	msr	spsr_cxsf, r0
	/* Exit hypervisor mode */
	adr	lr, 2f
	msr	elr_hyp, lr
	eret

1:
	cpsid	if, #PSR_SVC32_MODE		// SVC32 with no interrupts

2:
	mov	r0, #0
	msr	spsr_sxc, r0			// set SPSR[23:8] to known value

	mov	sp, r5

	XPUTC('A')

	mrc	p15, 0, r0, c1, c0, 0
	tst	r0, #CPU_CONTROL_DC_ENABLE
	blne	armv7_dcache_wbinv_all

	// TeX remap

#define ARMV7_SCTLR_CLEAR	( 	\
    CPU_CONTROL_IC_ENABLE |	\
    CPU_CONTROL_DC_ENABLE |	\
    CPU_CONTROL_MMU_ENABLE |	\
    CPU_CONTROL_BPRD_ENABLE |	\
    CPU_CONTROL_TR_ENABLE |	\
    0)


// CPU_CONTROL_VECRELOC

#define ARMV7_SCTLR_SET	( 	\
    CPU_CONTROL_UNAL_ENABLE |	\
    0)


	mrc	p15, 0, r0, c1, c0, 0
	movw	r1, #:lower16:ARMV7_SCTLR_CLEAR
	movt	r1, #:upper16:ARMV7_SCTLR_CLEAR
	movw	r2, #:lower16:ARMV7_SCTLR_SET
	movt	r2, #:upper16:ARMV7_SCTLR_SET

	mov	r7, r0				// save for printing
	bic	r6, r0, r1			// disable icache/dcache/mmu
	orr	r6, r6, r2			//

	mcr	p15, 0, r6, c1, c0, 0		// SCTLR write
	dsb
	isb

	bl	armv7_dcache_inv_all
	mcr	p15, 0, r0, c7, c5,  0		/* ICIALLU */
	dsb
	isb

	XPUTC(#'B')


	VPRINTF(" sctlr:")
	VPRINTX(r7)
	VPRINTF("/")
	VPRINTX(r6)
	VPRINTF(" ")

	XPUTC(#'C')

	bx	r4				// return

	.ltorg


armv7_mmuinit:
	// Because the MMU may already be on do a typical sequence to set
	// the Translation Table Base(s).
	mov	r4, lr
	mov	r5, r0			// save TTBR


	// XXXNH dsb - Why?
	XPUTC(#'F')
	dsb				// Drain the write buffers.

	XPUTC(#'G')
	mrc	p15, 0, r1, c0, c0, 5	// MPIDR read
	cmp	r1, #0
	orrlt	r5, r5, #TTBR_MPATTR	// MP, cachable (Normal WB)
	orrge	r5, r5, #TTBR_UPATTR	// Non-MP, cacheable, normal WB
	XPUTC(#'0')

	mcr	p15, 0, r5, c2, c0, 0	// TTBR0 write

#if defined(ARM_MMU_EXTENDED)
	// When using split TTBRs, we need to set both since the physical
	// addresses we were/are using might be in either.
	XPUTC(#'1')
	mcr	p15, 0, r5, c2, c0, 1	// TTBR1 write
#endif

	XPUTC(#'H')
#if defined(ARM_MMU_EXTENDED)
	XPUTC(#'1')
	mov	r1, #TTBCR_S_N_1	// make sure TTBCR_S_N is 1
#else
	XPUTC(#'0')
	mov	r1, #0			// make sure TTBCR is 0
#endif
	mcr	p15, 0, r1, c2, c0, 2	// TTBCR write

	XPUTC(#'J')
	mov	r1, #0			// get KERNEL_PID
	mcr	p15, 0, r1, c13, c0, 1	// CONTEXTIDR write



	// XXXNH FreeBSD doesn't do this isb
	isb


	// Set the Domain Access register.  Very important!
	XPUTC(#'K')
	mov	r1, #DOMAIN_DEFAULT
	mcr	p15, 0, r1, c3, c0, 0	// DACR write





#if 0
//XXXNH FreeBSD

288	/*
289	 * Set TEX remap registers
290	 *  - All is set to uncacheable memory
291	 */
292	ldr	r0, =0xAAAAA
293	mcr	CP15_PRRR(r0)
294	mov	r0, #0
295	mcr	CP15_NMRR(r0)
#endif


	// Could wait until reinit/cpu_ctrl update
	XPUTC(#'I')
	mov	r1, #0
	mcr	p15, 0, r1, c8, c7, 0	// TLBIALL (just this core)
	dsb
	isb

	//
	// Enable the MMU, etc.
	//
	XPUTC(#'L')
	XPUTC(#'\n')
	XPUTC(#'\r')
	mrc	p15, 0, r1, c1, c0, 0	// SCTLR read


	movw	r3, #:lower16:CPU_CONTROL_SET
	movt	r3, #:upper16:CPU_CONTROL_SET
	movw	r2, #:lower16:CPU_CONTROL_CLR
	movt	r2, #:upper16:CPU_CONTROL_CLR
	orr	r0, r1, r3
	bic	r0, r0, r2


	mcr	p15, 0, r0, c1, c0, 0	/* SCTLR write */

	dsb
	isb

	mcr	p15, 0, r0, c8, c7, 0	/* TLBIALL - Flush TLB */
	mcr	p15, 0, r0, c7, c5, 6	/* BPIALL - Branch predictor invalidate all */
	dsb
	isb

	VPRINTF("MMU\n\r")
	bx	r4			// return

	.p2align 2


	.text

ENTRY_NP(cpu_mpstart)
#ifndef MULTIPROCESSOR
	//
	// If not MULTIPROCESSOR, drop CPU into power saving state.
	//
3:	wfi
	b	3b
#else
#ifdef __ARMEB__
	setend	be				// switch to BE now
#endif

	/* disable IRQs/FIQs. */
	cpsid	if

	adr	r9, cpu_mpstart
	ldr	r10, =cpu_mpstart
	sub	r10, r10, r9

	ldr	r9, =svcstk
	add	r9, #INIT_ARM_STACK_SIZE * MAXCPUS
	sub	sp, r9, r10

	mrc	p15, 0, r4, c0, c0, 5		// MPIDR get
	and	r4, r4, #7			// get our cpu numder

	mov	r5, r4
	lsl	r5, #13
	sub	sp, sp, r5

	XPUTC('c')
	mov	r0, r4
	add	r0, #'0'
	bl	uartputc
	XPUTC(':')

	mov	r0, sp
	bl	generic_printx

	XPUTC('\n')
	XPUTC('\r')

#if 0
	// We haven't used anything from memory yet so we can invalidate the
	// L1 cache without fear of losing valuable data.  Afterwards, we can
	// flush icache without worrying about anything getting written back
	// to memory.
	bl	armv7_dcache_l1inv_all		// toss-dcache
	bl	armv7_icache_inv_all		// toss i-cache after d-cache

#endif

	// disables and clears caches
	bl	armv7_init


	movw	r0, #:lower16:TEMP_L1_TABLE
	movt	r0, #:upper16:TEMP_L1_TABLE
	sub	r0, r10

	movw	lr, #:lower16:armv7_mpcontinuation
	movt	lr, #:upper16:armv7_mpcontinuation
	b	armv7_mmuinit
ASEND(cpu_mpstart)


/*
 * Now running with real kernel VA via bootstrap tables
 */
armv7_mpcontinuation:
	ldr	r9, =svcstk
	add	r9, #INIT_ARM_STACK_SIZE * MAXCPUS

	mrc	p15, 0, r4, c0, c0, 5		// MPIDR get
	and	r4, r4, #7			// get our cpu numder

	mov	r5, r4
	lsl	r5, #13
	sub	sp, r9, r5

	VPRINTF("go\n\r")

	mrc	p15, 0, r4, c0, c0, 5		// MPIDR get
	and	r4, r4, #7			// get our cpu numder

	mov	r0, r4
	bl	cpu_init_secondary_processor

	movw	r0, #:lower16:cpu_info
	movt	r0, #:upper16:cpu_info		// get pointer to cpu_infos
	ldr	r5, [r0, r4, lsl #2]		// load our cpu_info
	ldr	r6, [r5, #CI_IDLELWP]		// get the idlelwp
	ldr	r7, [r6, #L_PCB]		// now get its pcb
	ldr	sp, [r7, #PCB_KSP]		// finally, we can load our SP
#ifdef TPIDRPRW_IS_CURCPU
	mcr	p15, 0, r5, c13, c0, 4		// squirrel away curcpu()
#elif defined(TPIDRPRW_IS_CURLWP)
	mcr	p15, 0, r6, c13, c0, 4		// squirrel away curlwp()
#else
#error either TPIDRPRW_IS_CURCPU or TPIDRPRW_IS_CURLWP must be defined
#endif
	str	r6, [r5, #CI_CURLWP]		// and note we are running on it

	mov	r0, r5				// pass cpu_info
	mov	r1, r4				// pass cpu_id
	movw	r2, #:lower16:MD_CPU_HATCH	// pass md_cpu_hatch
	movt	r2, #:upper16:MD_CPU_HATCH	// pass md_cpu_hatch
	bl	_C_LABEL(cpu_hatch)
	b	_C_LABEL(idle_loop)		// never to return
ASEND(armv7_mpcontinuation)
#endif	// MULTIPROCESSOR

#endif

ENTRY_NP(generic_vprint)
	push	{r4, lr}

	mov	r4, lr
	b	2f
1:
	bl	uartputc

2:
	ldrb	r0, [r4], #1
	cmp	r0, #0
	bne	1b

	add	lr, r4, #3
	bic	lr, #3

	pop	{r4}
	add	sp, sp, #4
	mov	pc, lr
ASEND(generic_prints)

ENTRY_NP(generic_prints)
	push	{r4, lr}

	mov	r4, r0
1:
	ldrb	r0, [r4], #1
	cmp	r0, #0
	popeq	{r4, pc}

	bl	uartputc
	b	1b
ASEND(generic_prints)

ENTRY_NP(generic_printx)
	push	{r4, r5, r6, lr}
	mov	r5, r0

	mov	r0, #'0'
	bl	uartputc
	mov	r0, #'x'
	bl	uartputc

	// Word size in bits
	mov	r4, #32
1:
	sub	r4, r4, #4		// nibble shift
	lsr	r3, r5, r4		// extract ...
	and	r3, r3, #0xf		// ... nibble

	cmp	r3, #9
	add	r0, r3, #'0'
	addgt	r0, r3, #'a' - 10
	bl	uartputc

	cmp	r4, #0
	bne	1b
	pop	{r4, r5, r6, pc}
ASEND(generic_printx)
