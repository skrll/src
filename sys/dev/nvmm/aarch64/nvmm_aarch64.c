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
#include <sys/systm.h>

#include <uvm/uvm_extern.h>
#include <uvm/uvm_page.h>

#include <dev/nvmm/nvmm.h>
#include <dev/nvmm/nvmm_internal.h>
#include <dev/nvmm/aarch64/nvmm_aarch64.h>

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
	printf("%s:%d\n", __func__, __LINE__);
}

static void
nvmm_aarch64_machine_destroy(struct nvmm_machine *mach)
{
	printf("%s:%d\n", __func__, __LINE__);
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
	printf("%s:%d\n", __func__, __LINE__);
	return 0;
}

static void
nvmm_aarch64_vcpu_destroy(struct nvmm_machine *mach, struct nvmm_cpu *vcpu)
{
	printf("%s:%d\n", __func__, __LINE__);
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
	printf("%s:%d\n", __func__, __LINE__);
}

static void
nvmm_aarch64_vcpu_getstate(struct nvmm_cpu *vcpu)
{
	printf("%s:%d\n", __func__, __LINE__);
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
	printf("%s:%d\n", __func__, __LINE__);
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
