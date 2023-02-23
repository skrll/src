/*	$NetBSD$	*/

/*-
 * Copyright (c) 2022 Ryo Shimizu <ryo@nerv.org>
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

#include <sys/param.h>
#include <sys/kmem.h>
#include <sys/systm.h>
#include <sys/kmem.h>

#include <uvm/uvm_extern.h>
#include <uvm/uvm_page.h>

#include <dev/nvmm/nvmm.h>
#include <dev/nvmm/nvmm_internal.h>
#include <dev/nvmm/aarch64/nvmm_aarch64.h>

struct aarch64_machdata {
	void *unused1;
};

struct aarch64_cpudata {
	void *unused1;

	/* guest state */
	uint64_t gprs[NVMM_AARCH64_NGPR];
	uint64_t sprs[NVMM_AARCH64_NSPR];
	__uint128_t fprs[NVMM_AARCH64_NFPR];
};

static void nvmm_aarch64_vcpu_setstate(struct nvmm_cpu *vcpu);

static void
debugdump_cpudata(struct aarch64_cpudata *cpudata)
{
	printf("    x0=%016lx,     x1=%016lx\n", cpudata->gprs[0], cpudata->gprs[1]);
	printf("    x2=%016lx,     x3=%016lx\n", cpudata->gprs[2], cpudata->gprs[3]);
	printf("    x4=%016lx,     x5=%016lx\n", cpudata->gprs[4], cpudata->gprs[5]);
	printf("    x6=%016lx,     x7=%016lx\n", cpudata->gprs[6], cpudata->gprs[7]);
	printf("    x8=%016lx,     x9=%016lx\n", cpudata->gprs[8], cpudata->gprs[9]);
	printf("   x10=%016lx,    x11=%016lx\n", cpudata->gprs[10], cpudata->gprs[11]);
	printf("   x12=%016lx,    x13=%016lx\n", cpudata->gprs[12], cpudata->gprs[13]);
	printf("   x14=%016lx,    x15=%016lx\n", cpudata->gprs[14], cpudata->gprs[15]);
	printf("   x16=%016lx,    x17=%016lx\n", cpudata->gprs[16], cpudata->gprs[17]);
	printf("   x18=%016lx,    x19=%016lx\n", cpudata->gprs[18], cpudata->gprs[19]);
	printf("   x20=%016lx,    x21=%016lx\n", cpudata->gprs[20], cpudata->gprs[21]);
	printf("   x22=%016lx,    x23=%016lx\n", cpudata->gprs[22], cpudata->gprs[23]);
	printf("   x24=%016lx,    x25=%016lx\n", cpudata->gprs[24], cpudata->gprs[25]);
	printf("   x26=%016lx,    x27=%016lx\n", cpudata->gprs[26], cpudata->gprs[27]);
	printf("   x28=%016lx, fp=x29=%016lx\n", cpudata->gprs[28], cpudata->gprs[29]);
	printf("lr=x30=%016lx,     sp=%016lx\n", cpudata->gprs[30], cpudata->gprs[31]);

	printf("    PC=%016lx\n", cpudata->sprs[NVMM_AARCH64_SPR_PC]);
}

static bool
nvmm_aarch64_ident(void)
{
	printf("%s:%d\n", __func__, __LINE__);
	return true;
}

static void
nvmm_aarch64_init(void)
{
	printf("%s:%d\n", __func__, __LINE__);
}

static void
nvmm_aarch64_fini(void)
{
	printf("%s:%d\n", __func__, __LINE__);
}

static void
nvmm_aarch64_capability(struct nvmm_capability *cap)
{
	printf("%s:%d\n", __func__, __LINE__);

	cap->arch.mach_conf_support = 0;
	cap->arch.vcpu_conf_support = 0;
}

static void
nvmm_aarch64_machine_create(struct nvmm_machine *mach)
{
	struct aarch64_machdata *machdata;

	/* setup aarch64's pmap hooks */
	mach->vm->vm_map.pmap->pm_stage2 = true;

	machdata = kmem_zalloc(sizeof(struct aarch64_machdata), KM_SLEEP);
	mach->machdata = machdata;
}

static void
nvmm_aarch64_machine_destroy(struct nvmm_machine *mach)
{
	printf("%s:%d\n", __func__, __LINE__);

	kmem_free(mach->machdata, sizeof(struct aarch64_machdata));
}

static int
nvmm_aarch64_machine_configure(struct nvmm_machine *mach, uint64_t op,
    void *data)
{
	printf("%s:%d\n", __func__, __LINE__);
	return 0;
}

static int
nvmm_aarch64_vcpu_create(struct nvmm_machine *mach, struct nvmm_cpu *vcpu)
{
	struct aarch64_cpudata *cpudata;

	cpudata = (struct aarch64_cpudata *)uvm_km_alloc(kernel_map,
	    roundup(sizeof(*cpudata), PAGE_SIZE), 0,
	    UVM_KMF_WIRED | UVM_KMF_ZERO);
	vcpu->cpudata = cpudata;

	/* Install the RESET state. */
	memset(&vcpu->comm->state, 0, sizeof(vcpu->comm->state));
	vcpu->comm->state_wanted = NVMM_AARCH64_STATE_ALL;
	vcpu->comm->state_cached = 0;
	nvmm_aarch64_vcpu_setstate(vcpu);

	return 0;
}

static void
nvmm_aarch64_vcpu_destroy(struct nvmm_machine *mach, struct nvmm_cpu *vcpu)
{
	struct aarch64_cpudata * const cpudata = vcpu->cpudata;

	printf("%s:%d\n", __func__, __LINE__);

	uvm_km_free(kernel_map, (vaddr_t)cpudata,
	    roundup(sizeof(*cpudata), PAGE_SIZE), UVM_KMF_WIRED);
}

static int
nvmm_aarch64_vcpu_configure(struct nvmm_cpu *vcpu, uint64_t op, void *data)
{
	printf("%s:%d\n", __func__, __LINE__);
	return 0;
}

static void
nvmm_aarch64_vcpu_setstate(struct nvmm_cpu *vcpu)
{
	struct nvmm_comm_page * const comm = vcpu->comm;
	const struct nvmm_aarch64_state * const state = &comm->state;
	struct aarch64_cpudata * const cpudata = vcpu->cpudata;
	const uint64_t flags = comm->state_wanted;

	if (flags & NVMM_AARCH64_STATE_GPRS) {
		memcpy(cpudata->gprs, state->gprs, sizeof(state->gprs));
	}
	if (flags & NVMM_AARCH64_STATE_SPRS) {
		memcpy(cpudata->sprs, state->sprs, sizeof(state->sprs));
	}
	if (flags & NVMM_AARCH64_STATE_FPRS) {
		memcpy(cpudata->fprs, state->fprs, sizeof(state->fprs));
	}

	comm->state_wanted = 0;
	comm->state_cached |= flags;
}

static void
nvmm_aarch64_vcpu_getstate(struct nvmm_cpu *vcpu)
{
	struct nvmm_comm_page * const comm = vcpu->comm;
	struct nvmm_aarch64_state * const state = &comm->state;
	const struct aarch64_cpudata * const cpudata = vcpu->cpudata;
	const uint64_t flags = comm->state_wanted;

	if (flags & NVMM_AARCH64_STATE_GPRS) {
		memcpy(state->gprs, cpudata->gprs, sizeof(state->gprs));
	}
	if (flags & NVMM_AARCH64_STATE_SPRS) {
		memcpy(state->sprs, cpudata->sprs, sizeof(state->sprs));
	}
	if (flags & NVMM_AARCH64_STATE_FPRS) {
		memcpy(state->fprs, cpudata->fprs, sizeof(state->fprs));
	}

	comm->state_wanted = 0;
	comm->state_cached |= flags;
}

//static void
//aarch64_vcpu_state_provide(struct nvmm_cpu *vcpu, uint64_t flags)
//{
//	vcpu->comm->state_wanted = flags;
//	nvmm_aarch64_vcpu_getstate(vcpu);
//}

static void
aarch64_vcpu_state_commit(struct nvmm_cpu *vcpu)
{
	vcpu->comm->state_wanted = vcpu->comm->state_commit;
	vcpu->comm->state_commit = 0;
	nvmm_aarch64_vcpu_setstate(vcpu);
}

static int
nvmm_aarch64_vcpu_inject(struct nvmm_cpu *vcpu)
{
	printf("%s:%d\n", __func__, __LINE__);
	return 0;
}

static int
nvmm_aarch64_vcpu_run(struct nvmm_machine *mach, struct nvmm_cpu *vcpu,
    struct nvmm_vcpu_exit *exit)
{
	struct nvmm_comm_page * const comm = vcpu->comm;
	struct aarch64_cpudata * const cpudata = vcpu->cpudata;
//	struct aarch64_machdata * const machdata = mach->machdata;


	aarch64_vcpu_state_commit(vcpu);
	comm->state_cached = 0;


	//XXXX
	debugdump_cpudata(cpudata);


	//event commit


	kpreempt_disable();
	/*
		XXXXXXXXXXXXXXXXXXXXXXXXXX
	*/
	kpreempt_enable();

	return 0;
}

const struct nvmm_impl nvmm_aarch64 = {
	.name = "aarch64",
	.ident = nvmm_aarch64_ident,
	.init = nvmm_aarch64_init,
	.fini = nvmm_aarch64_fini,
	.capability = nvmm_aarch64_capability,
	.mach_conf_max = 0,
	.mach_conf_sizes = NULL,
	.vcpu_conf_max = 0,
	.vcpu_conf_sizes = NULL,
	.state_size = sizeof(struct nvmm_aarch64_state),
	.machine_create = nvmm_aarch64_machine_create,
	.machine_destroy = nvmm_aarch64_machine_destroy,
	.machine_configure = nvmm_aarch64_machine_configure,
	.vcpu_create = nvmm_aarch64_vcpu_create,
	.vcpu_destroy = nvmm_aarch64_vcpu_destroy,
	.vcpu_configure = nvmm_aarch64_vcpu_configure,
	.vcpu_setstate = nvmm_aarch64_vcpu_setstate,
	.vcpu_getstate = nvmm_aarch64_vcpu_getstate,
	.vcpu_inject = nvmm_aarch64_vcpu_inject,
	.vcpu_run = nvmm_aarch64_vcpu_run
};
