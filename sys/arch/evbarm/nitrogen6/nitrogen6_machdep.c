/*	$NetBSD: nitrogen6_machdep.c,v 1.10 2018/09/21 12:04:09 skrll Exp $	*/

/*-
 * Copyright (c) 2012 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Matt Thomas of 3am Software Foundry.
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
__KERNEL_RCSID(0, "$NetBSD: nitrogen6_machdep.c,v 1.10 2018/09/21 12:04:09 skrll Exp $");

#include "opt_evbarm_boardtype.h"
#include "opt_arm_debug.h"
#include "opt_kgdb.h"
#include "opt_console.h"
#include "com.h"
#include "opt_machdep.h"
#include "opt_imxuart.h"
#include "imxuart.h"
#include "opt_imx.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/atomic.h>
#include <sys/device.h>
#include <sys/kernel.h>
#include <sys/msgbuf.h>
#include <sys/reboot.h>
#include <sys/termios.h>

#include <dev/cons.h>

#include <uvm/uvm_extern.h>

#include <arm/db_machdep.h>
#include <arm/arm32/machdep.h>

#include <machine/autoconf.h>
#include <machine/bootconfig.h>

#include <arm/cortex/a9tmr_var.h>
#include <arm/cortex/scu_reg.h>

#include <arm/imx/imx6var.h>
#include <arm/imx/imxuartreg.h>
#include <arm/imx/imxuartvar.h>

#include <evbarm/nitrogen6/platform.h>

#ifdef VERBOSE_INIT_ARM
#define VPRINTF(...)	printf(__VA_ARGS__)
#else
#define VPRINTF(...)	do { } while (/* CONSTCOND */ 0)
#endif

extern int _end[];
extern int KERNEL_BASE_phys[];
extern int KERNEL_BASE_virt[];

BootConfig bootconfig;
static char bootargs[MAX_BOOT_STRING];
char *boot_args = NULL;

/* filled in before cleaning bss. keep in .data */
u_int uboot_args[4] __attribute__((__section__(".data")));

#ifndef CONADDR
#define CONADDR	(IMX6_AIPS2_BASE + AIPS2_UART1_BASE)
#endif
#ifndef CONSPEED
#define CONSPEED B115200
#endif
#ifndef CONMODE
#define CONMODE ((TTYDEF_CFLAG & ~(CSIZE | CSTOPB | PARENB)) | CS8) /* 8N1 */
#endif

void nitrogen6_setup_iomux(void);
void nitrogen6_device_register(device_t, void *);
void nitrogen6_mpstart(void);
void nitrogen6_platform_early_putchar(char);


#ifdef KGDB
#include <sys/kgdb.h>
#endif

static dev_type_cnputc(earlyconsputc);
static dev_type_cngetc(earlyconsgetc);

static struct consdev earlycons = {
	.cn_putc = earlyconsputc,
	.cn_getc = earlyconsgetc,
	.cn_pollc = nullcnpollc,
};

static void
earlyconsputc(dev_t dev, int c)
{
	uartputc(c);
}

static int
earlyconsgetc(dev_t dev)
{
	return 0;	/* XXX */
}

/*
 * Static device mappings. These peripheral registers are mapped at
 * fixed virtual addresses very early in initarm() so that we can use
 * them while booting the kernel, and stay at the same address
 * throughout whole kernel's life time.
 *
 * We use this table twice; once with bootstrap page table, and once
 * with kernel's page table which we build up in initarm().
 *
 * Since we map these registers into the bootstrap page table using
 * pmap_devmap_bootstrap() which calls pmap_map_chunk(), we map
 * registers segment-aligned and segment-rounded in order to avoid
 * using the 2nd page tables.
 */
static const struct pmap_devmap devmap[] = {
	{
		KERNEL_IO_IOREG_VBASE,
		IMX6_IOREG_PBASE,		/* 0x02000000 */
		IMX6_IOREG_SIZE,
		VM_PROT_READ | VM_PROT_WRITE,
		PTE_NOCACHE,
	},
	{
		KERNEL_IO_ARMCORE_VBASE,
		IMX6_ARMCORE_PBASE,		/* 0x00a00000 */
		IMX6_ARMCORE_SIZE,
		VM_PROT_READ | VM_PROT_WRITE,
		PTE_NOCACHE,
	},
	{ 0, 0, 0, 0, 0 }
};

#ifdef PMAP_NEED_ALLOC_POOLPAGE
static struct boot_physmem bp_highgig = {
	.bp_start = IMX6_MEM_BASE / NBPG,
	.bp_pages = (KERNEL_VM_BASE - KERNEL_BASE) / NBPG,
	.bp_freelist = VM_FREELIST_ISADMA,
	.bp_flags = 0,
};
#endif

void
nitrogen6_platform_early_putchar(char c)
{
#define CONADDR_VA (CONADDR - IMX6_IOREG_PBASE + KERNEL_IO_IOREG_VBASE)
	volatile uint32_t *uartaddr = cpu_earlydevice_va_p() ?
	    (volatile uint32_t *)CONADDR_VA :
	    (volatile uint32_t *)CONADDR;

	int timo = 150000;

	while ((uartaddr[IMX_USR2 / 4] & IMX_USR2_TXDC) == 0) {
		if (--timo == 0)
			break;
	}

	uartaddr[IMX_UTXD / 4] = c;

	timo = 150000;
	while ((uartaddr[IMX_USR2 / 4] & IMX_USR2_TXDC) == 0) {
		if (--timo == 0)
			break;
	}
}



void
nitrogen6_mpstart(void)
{
#ifdef MULTIPROCESSOR
	uint32_t scu_cfg = bus_space_read_4(imx6_armcore_bst,
	    imx6_armcore_bsh, ARMCORE_SCU_BASE + SCU_CFG);
	scu_cfg |= SCU_CTL_SCU_ENA;
	bus_space_write_4(imx6_armcore_bst, imx6_armcore_bsh,
	    ARMCORE_SCU_BASE + SCU_CFG, scu_cfg);
printf("%s: %d\n", __func__, __LINE__);

//#if NARML2CC > 0
//	arml2cc_init(imx6_armcore_bst, imx6_armcore_bsh, ARMCORE_L2C_BASE);
//#endif












#if 0
	bus_space_handle_t scu;
	bus_space_handle_t src;

	uint32_t val;
	int i;

	if (bus_space_map(fdtbus_bs_tag, SCU_PHYSBASE, SCU_SIZE, 0, &scu) != 0)
		panic("Couldn't map the SCU\n");
	if (bus_space_map(fdtbus_bs_tag, SRC_PHYSBASE, SRC_SIZE, 0, &src) != 0)
		panic("Couldn't map the system reset controller (SRC)\n");

	/*
	 * Invalidate SCU cache tags.  The 0x0000ffff constant invalidates all
	 * ways on all cores 0-3.  Per the ARM docs, it's harmless to write to
	 * the bits for cores that are not present.
	 */
	bus_space_write_4(fdtbus_bs_tag, scu, SCU_INV_TAGS_REG, 0x0000ffff);

	/*
	 * Erratum ARM/MP: 764369 (problems with cache maintenance).
	 * Setting the "disable-migratory bit" in the undocumented SCU
	 * Diagnostic Control Register helps work around the problem.
	 */
	val = bus_space_read_4(fdtbus_bs_tag, scu, SCU_DIAG_CONTROL);
	bus_space_write_4(fdtbus_bs_tag, scu, SCU_DIAG_CONTROL, 
	    val | SCU_DIAG_DISABLE_MIGBIT);

	/*
	 * Enable the SCU, then clean the cache on this core.  After these two
	 * operations the cache tag ram in the SCU is coherent with the contents
	 * of the cache on this core.  The other cores aren't running yet so
	 * their caches can't contain valid data yet, but we've initialized
	 * their SCU tag ram above, so they will be coherent from startup.
	 */
	val = bus_space_read_4(fdtbus_bs_tag, scu, SCU_CONTROL_REG);
	bus_space_write_4(fdtbus_bs_tag, scu, SCU_CONTROL_REG, 
	    val | SCU_CONTROL_ENABLE);
	dcache_wbinv_poc_all();

	/*
	 * For each AP core, set the entry point address and argument registers,
	 * and set the core-enable and core-reset bits in the control register.
	 */
	val = bus_space_read_4(fdtbus_bs_tag, src, SRC_CONTROL_REG);
	for (i=1; i < mp_ncpus; i++) {
		bus_space_write_4(fdtbus_bs_tag, src, SRC_GPR0_C1FUNC + 8*i,
		    pmap_kextract((vm_offset_t)mpentry));
		bus_space_write_4(fdtbus_bs_tag, src, SRC_GPR1_C1ARG  + 8*i, 0);

		val |= ((1 << (SRC_CONTROL_C1ENA_SHIFT - 1 + i )) |
		    ( 1 << (SRC_CONTROL_C1RST_SHIFT - 1 + i)));

	}
	bus_space_write_4(fdtbus_bs_tag, src, SRC_CONTROL_REG, val);

	dsb();
	sev();

	bus_space_unmap(fdtbus_bs_tag, scu, SCU_SIZE);
	bus_space_unmap(fdtbus_bs_tag, src, SRC_SIZE);
#endif




#if 0
printf("%s: %d\n", __func__, __LINE__);
	bus_space_tag_t bst = imx6_armcore_bst;
	bus_space_handle_t bsh;
	int error = bus_space_map(bst, ZYNQ7000_CPU1_ENTRY,
	    ZYNQ7000_CPU1_ENTRY_SZ, 0, &bsh);
	if (error)
		panic("%s: Couldn't map OCM", __func__);

printf("%s: %d\n", __func__, __LINE__);
	/* Write start address for CPU1. */
	bus_space_write_4(bst, bsh, 0, KERN_VTOPHYS((vaddr_t)cpu_mpstart));

printf("%s: %d\n", __func__, __LINE__);
	bus_space_unmap(bst, bsh, ZYNQ7000_CPU1_ENTRY_SZ);

printf("%s: %d\n", __func__, __LINE__);
	cpu_idcache_wbinv_all();
	// cache flush
	//        armv7_dcache_l1inv_all();
#endif

printf("%s: %d\n", __func__, __LINE__);
	arm_dsb();
	__asm __volatile("sev" ::: "memory");

printf("%s: %d\n", __func__, __LINE__);
	for (int loop = 0; loop < 16; loop++) {
		if (arm_cpu_hatched == __BITS(arm_cpu_max - 1, 1))
			break;
		a9tmr_delay(10000);
	}
	for (size_t i = 1; i < arm_cpu_max; i++) {
		if ((arm_cpu_hatched & __BIT(i)) == 0) {
		printf("%s: warning: cpu%zu failed to hatch\n",
			    __func__, i);
		}
	}

	VPRINTF(" (%u cpu%s, hatched %#x)",
	    arm_cpu_max, arm_cpu_max ? "s" : "",
	    arm_cpu_hatched);
#endif /* MULTIPROCESSOR */
}



/*
 * u_int initarm(...)
 *
 * Initial entry point on startup. This gets called before main() is
 * entered.
 * It should be responsible for setting up everything that must be
 * in place when main is called.
 * This includes
 *   Taking a copy of the boot configuration structure.
 *   Initialising the physical console so characters can be printed.
 *   Setting up page tables for the kernel
 */
u_int
initarm(void *arg)
{
	psize_t memsize;

	/*
	 * Heads up ... Setup the CPU / MMU / TLB functions
	 */
	if (set_cpufuncs())		// starts PMC counter
		panic("cpu not recognized!");

	cn_tab = &earlycons;

	extern char ARM_BOOTSTRAP_LxPT[];
	pmap_devmap_bootstrap((vaddr_t)ARM_BOOTSTRAP_LxPT, devmap);

	imx6_bootstrap(KERNEL_IO_IOREG_VBASE);

#ifdef MULTIPROCESSOR
	uint32_t scu_cfg = bus_space_read_4(imx6_armcore_bst, imx6_armcore_bsh,
	    ARMCORE_SCU_BASE + SCU_CFG);
	arm_cpu_max = (scu_cfg & SCU_CFG_CPUMAX) + 1;
	membar_producer();
#endif /* MULTIPROCESSOR */

	nitrogen6_setup_iomux();

	consinit();
//XXXNH
	cpu_domains((DOMAIN_CLIENT << (PMAP_DOMAIN_KERNEL*2)) | DOMAIN_CLIENT);

#ifdef NO_POWERSAVE
	cpu_do_powersave = 0;
#endif

	cortex_pmc_ccnt_init();

	printf("\nuboot arg = %#x, %#x, %#x, %#x\n",
	    uboot_args[0], uboot_args[1], uboot_args[2], uboot_args[3]);

	cpu_reset_address = imx6_reset;

	/* Talk to the user */
	printf("\nNetBSD/evbarm (" ___STRING(EVBARM_BOARDTYPE) ") booting ...\n");

#ifdef BOOT_ARGS
	char mi_bootargs[] = BOOT_ARGS;
	parse_mi_bootargs(mi_bootargs);
#endif /* BOOT_ARGS */
	bootargs[0] = '\0';

#ifdef VERBOSE_INIT_ARM
	printf("initarm: Configuring system");
#ifdef MULTIPROCESSOR
	printf(" (%u cpu%s, hatched %#x)",
	    arm_cpu_max, arm_cpu_max ? "s" : "",
	    arm_cpu_hatched);
#endif /* MULTIPROCESSOR */
	printf(", CLIDR=%010o CTR=%#x",
	    armreg_clidr_read(), armreg_ctr_read());
	printf("\n");
#endif /* VERBOSE_INIT_ARM */


#ifdef MEMSIZE
	memsize = MEMSIZE * 1024 * 1024;
#else
	memsize = imx6_memprobe();
#endif

	bootconfig.dramblocks = 1;
	bootconfig.dram[0].address = IMX6_MEM_BASE;
	bootconfig.dram[0].pages = memsize / PAGE_SIZE;

#ifdef __HAVE_MM_MD_DIRECT_MAPPED_PHYS
	const bool mapallmem_p = true;
#ifndef PMAP_NEED_ALLOC_POOLPAGE
	if (memsize > KERNEL_VM_BASE - KERNEL_BASE) {
		printf("%s: dropping RAM size from %luMB to %uMB\n",
		   __func__, (unsigned long) (memsize >> 20),
		   (KERNEL_VM_BASE - KERNEL_BASE) >> 20);
		memsize = KERNEL_VM_BASE - KERNEL_BASE;
	}
#endif
#else /* !__HAVE_MM_MD_DIRECT_MAPPED_PHYS */
	const bool mapallmem_p = false;
#endif /* __HAVE_MM_MD_DIRECT_MAPPED_PHYS */

	arm32_bootmem_init(bootconfig.dram[0].address,
	    memsize, (paddr_t)KERNEL_BASE_phys);
	arm32_kernel_vm_init(KERNEL_VM_BASE, ARM_VECTORS_LOW, 0, devmap,
	    mapallmem_p);

	VPRINTF("initarm_common");

	/* we've a specific device_register routine */
	evbarm_device_register = nitrogen6_device_register;

	const struct boot_physmem *bp = NULL;
	size_t nbp = 0;

#ifdef PMAP_NEED_ALLOC_POOLPAGE
	/*
	 * If we couldn't map all of memory via TTBR1, limit the memory the
	 * kernel can allocate from to be from the highest available 1GB.
	 */
	if (atop(memsize) > bp_highgig.bp_pages) {
		bp_highgig.bp_start += atop(memsize) - bp_highgig.bp_pages;
		arm_poolpage_vmfreelist = bp_highgig.bp_freelist;
		bp = &bp_highgig;
		nbp = 1;
	}
#endif
	u_int sp = initarm_common(KERNEL_VM_BASE, KERNEL_VM_SIZE, bp, nbp);

	VPRINTF("mpstart\n");
	nitrogen6_mpstart();

	return sp;
}

#ifdef CONSDEVNAME
const char consdevname[] = CONSDEVNAME;

#ifndef CONMODE
#define CONMODE	((TTYDEF_CFLAG & ~(CSIZE | CSTOPB | PARENB)) | CS8) /* 8N1 */
#endif
#ifndef CONSPEED
#define CONSPEED	115200
#endif

int consmode = CONMODE;
int consrate = CONSPEED;

#endif /* CONSDEVNAME */

#ifndef IMXUART_FREQ
#define IMXUART_FREQ	80000000
#endif

void
consinit(void)
{
	static int consinit_called = 0;

	if (consinit_called)
		return;

	consinit_called = 1;

#ifdef CONSDEVNAME
# if NIMXUART > 0
	imxuart_set_frequency(IMXUART_FREQ, 2);
# endif
# if (NIMXUART > 0) && defined(IMXUARTCONSOLE)
	if (strcmp(consdevname, CONSDEVNAME) == 0) {
		paddr_t consaddr;

		consaddr = CONADDR;
		imxuart_cnattach(&armv7_generic_bs_tag, consaddr, consrate, consmode);
		return;
	}
# endif /* (NIMXUART > 0) && defined(IMXUARTCONSOLE) */
#endif /* CONSDEVNAME */
}
