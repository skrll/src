#ifndef _NVMM_AARCH64_INTERNAL_H_
#define _NVMM_AARCH64_INTERNAL_H_

struct aarch64_cpudata {
	uint64_t cpudata_pa;
	uint64_t vttbr_el2;
	uint64_t send_event_type;

	struct nvmm_aarch64_exit exit;
	struct nvmm_aarch64_state host;
	struct nvmm_aarch64_state guest;
};


char *uartputs(const char *);
int uartprintf(const char * restrict, ...) __printflike(1, 2);

paddr_t aarch64_gva_to_pa(uint64_t, vaddr_t);
paddr_t aarch64_gva_to_ipa(uint64_t, vaddr_t);
paddr_t aarch64_get_fault_ipa(struct trapframe *);
static inline paddr_t
ipa_hpfar_far(vaddr_t hpfar, vaddr_t far)
{
	return ((__SHIFTOUT(hpfar, HPFAR_EL2_FIPA) << HPFAR_EL2_FIPA_BITSHIFT) &
	    ~PAGE_MASK) | (far & PAGE_MASK);
}

void dump_el2_trapframe(struct trapframe *);
void aarch64_el2_init(struct trapframe *);
void aarch64_el2_vmenter(struct trapframe *);
void aarch64_el2_vmexit_trap(struct trapframe *tf);
void aarch64_el2_vmexit_irq(struct trapframe *tf);
void aarch64_el2_maintain_ipa(struct trapframe *tf);

void nvmm_aarch64_load_fpregs(const union fpelem *);
void nvmm_aarch64_save_fpregs(union fpelem *);

extern int aarch64_el2_initted;	/* nvmm_aarch64_el2.c */

//XXX: for debug
extern int nvmm_debug;

#endif /* _NVMM_AARCH64_INTERNAL_H_ */
