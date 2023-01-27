#	$NetBSD: Makefile,v 1.1 2018/11/07 07:43:08 maxv Exp $

.include <bsd.own.mk>

.if ${MACHINE_CPU} == "aarch64"
SUBDIR= aarch64
.elif ${MACHINE_ARCH} == "x86_64"
SUBDIR= x86
.endif

INCSDIR= /usr/include/dev/nvmm

INCS=	nvmm.h nvmm_ioctl.h

.include <bsd.kinc.mk>
