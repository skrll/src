
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kmem.h>

#include <uvm/uvm_extern.h>
#include <uvm/uvm_page.h>

#include <dev/nvmm/nvmm.h>
#include <dev/nvmm/nvmm_internal.h>
#include <dev/nvmm/aarch64/nvmm_aarch64.h>

int uartprintf(const char * restrict, ...);
void aarch64_el2_mmu_enable(paddr_t);
void aarch64_el2_init(paddr_t);
void aarch64_el2_vmrun(struct nvmm_aarch64_state *);

static void __unused
xxx_hexdump(void *addr, unsigned int len)
{
	uint8_t *p = (uint8_t *)addr;
	unsigned int i;
	for (i = 0; i < len; i++) {
		if ((i & 15) == 0)
			uartprintf("%p:", &p[i]);
		uartprintf(" %02x", p[i]);
		if ((i & 15) == 15)
			uartprintf("\n");
	}
	if ((i & 15) != 0)
		uartprintf("\n");
}

void
aarch64_el2_init(paddr_t ttbr)
{
	aarch64_el2_mmu_enable(ttbr);
}

void
aarch64_el2_vmrun(struct nvmm_aarch64_state *state_pa)
{
	uartprintf("%s: state_pa=%p\n", __func__, state_pa);

	uartprintf("    x0=%016lx,     x1=%016lx\n", state_pa->gprs[0], state_pa->gprs[1]);
	uartprintf("    x2=%016lx,     x3=%016lx\n", state_pa->gprs[2], state_pa->gprs[3]);
	uartprintf("    x4=%016lx,     x5=%016lx\n", state_pa->gprs[4], state_pa->gprs[5]);
	uartprintf("    x6=%016lx,     x7=%016lx\n", state_pa->gprs[6], state_pa->gprs[7]);
	uartprintf("    x8=%016lx,     x9=%016lx\n", state_pa->gprs[8], state_pa->gprs[9]);
	uartprintf("   x10=%016lx,    x11=%016lx\n", state_pa->gprs[10], state_pa->gprs[11]);
	uartprintf("   x12=%016lx,    x13=%016lx\n", state_pa->gprs[12], state_pa->gprs[13]);
	uartprintf("   x14=%016lx,    x15=%016lx\n", state_pa->gprs[14], state_pa->gprs[15]);
	uartprintf("   x16=%016lx,    x17=%016lx\n", state_pa->gprs[16], state_pa->gprs[17]);
	uartprintf("   x18=%016lx,    x19=%016lx\n", state_pa->gprs[18], state_pa->gprs[19]);
	uartprintf("   x20=%016lx,    x21=%016lx\n", state_pa->gprs[20], state_pa->gprs[21]);
	uartprintf("   x22=%016lx,    x23=%016lx\n", state_pa->gprs[22], state_pa->gprs[23]);
	uartprintf("   x24=%016lx,    x25=%016lx\n", state_pa->gprs[24], state_pa->gprs[25]);
	uartprintf("   x26=%016lx,    x27=%016lx\n", state_pa->gprs[26], state_pa->gprs[27]);
	uartprintf("   x28=%016lx, fp=x29=%016lx\n", state_pa->gprs[28], state_pa->gprs[29]);
	uartprintf("lr=x30=%016lx,     sp=%016lx\n", state_pa->gprs[30], state_pa->gprs[31]);
	uartprintf("    PC=%016lx\n", state_pa->sprs[NVMM_AARCH64_SPR_PC]);
}
