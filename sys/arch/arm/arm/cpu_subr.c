/*	$NetBSD$	*/


#include "opt_cputypes.h"
#include "opt_multiprocessor.h"

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/atomic.h>
#include <sys/cpu.h>
#include <sys/reboot.h>

#ifdef VERBOSE_INIT_ARM
#define VPRINTF(...)	printf(__VA_ARGS__)
#else
#define VPRINTF(...)	__nothing
#endif

#ifdef MULTIPROCESSOR
#define NCPUINFO	MAXCPUS
#else
#define NCPUINFO	1
#endif /* MULTIPROCESSOR */

mpidr_t cpu_mpidr[NCPUINFO] = {
	[0 ... NCPUINFO - 1] = ~0,
};

struct cpu_info *cpu_info[NCPUINFO] __read_mostly = {
	[0] = &cpu_info_store[0]
};

#ifdef MULTIPROCESSOR

#define	CPUINDEX_DIVISOR	(sizeof(u_long) * NBBY)

volatile u_long arm_cpu_hatched[howmany(MAXCPUS, CPUINDEX_DIVISOR)] __cacheline_aligned = { 0 };
volatile u_long arm_cpu_mbox[howmany(MAXCPUS, CPUINDEX_DIVISOR)] __cacheline_aligned = { 0 };
u_int arm_cpu_max = 1;

kmutex_t cpu_hatch_lock;

void
cpu_boot_secondary_processors(void)
{
	u_int cpuno;

	if ((boothowto & RB_MD1) != 0)
		return;

	mutex_init(&cpu_hatch_lock, MUTEX_DEFAULT, IPL_NONE);

	VPRINTF("%s: starting secondary processors\n", __func__);

	/* send mbox to have secondary processors do cpu_hatch() */
	for (size_t n = 0; n < __arraycount(arm_cpu_mbox); n++)
		atomic_or_ulong(&arm_cpu_mbox[n], arm_cpu_hatched[n]);

	__asm __volatile ("sev; sev; sev");

	/* wait all cpus have done cpu_hatch() */
	for (cpuno = 1; cpuno < ncpu; cpuno++) {
		if (!cpu_hatched_p(cpuno))
			continue;

		const size_t off = cpuno / CPUINDEX_DIVISOR;
		const u_long bit = __BIT(cpuno % CPUINDEX_DIVISOR);

		while (membar_consumer(), arm_cpu_mbox[off] & bit) {
			__asm __volatile ("wfe");
		}
		/* Add processor to kcpuset */
		kcpuset_set(kcpuset_attached, cpuno);
	}

	VPRINTF("%s: secondary processors hatched\n", __func__);
}

bool
cpu_hatched_p(u_int cpuindex)
{
	const u_int off = cpuindex / CPUINDEX_DIVISOR;
	const u_int bit = cpuindex % CPUINDEX_DIVISOR;

	membar_consumer();
	return (arm_cpu_hatched[off] & __BIT(bit)) != 0;
}

void
cpu_set_hatched(int cpuindex)
{

	const size_t off = cpuindex / CPUINDEX_DIVISOR;
	const u_long bit = __BIT(cpuindex % CPUINDEX_DIVISOR);

	atomic_or_ulong(&arm_cpu_hatched[off], bit);
}

void
cpu_clr_mbox(int cpuindex)
{

	const size_t off = cpuindex / CPUINDEX_DIVISOR;
	const u_long bit = __BIT(cpuindex % CPUINDEX_DIVISOR);

	/* Notify cpu_boot_secondary_processors that we're done */
	atomic_and_ulong(&arm_cpu_mbox[off], ~bit);
	membar_producer();
	__asm __volatile("sev; sev; sev");
}

#endif
