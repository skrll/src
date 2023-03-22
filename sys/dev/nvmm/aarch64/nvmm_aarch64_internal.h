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

//XXXXXXXXX decl
int uartprintf(const char * restrict, ...) __printflike(1, 2);
void dump_el2_trapframe(struct trapframe *tf);
void aarch64_el2_init(struct trapframe *);
void aarch64_el2_vmenter(struct trapframe *);
void aarch64_el2_vmexit_trap(struct trapframe *tf);
void aarch64_el2_vmexit_irq(struct trapframe *tf);
void aarch64_el2_maintain_ipa(struct trapframe *tf);

char *uartputs(const char *);
int uartprintf(const char * restrict, ...) __printflike(1, 2);

void nvmm_aarch64_load_fpregs(const union fpelem *);
void nvmm_aarch64_save_fpregs(union fpelem *);

#endif /* _NVMM_AARCH64_INTERNAL_H_ */
