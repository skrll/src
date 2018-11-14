#	$NetBSD: Makefile,v 1.1.1.1 2018/11/10 09:38:09 mrg Exp $

SUBDIR+= lib

.include <bsd.own.mk>

.if ${MKGCCCMDS} != "no"
SUBDIR+= .WAIT usr.bin
.endif

.include <bsd.subdir.mk>
