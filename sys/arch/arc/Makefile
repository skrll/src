#	$NetBSD: Makefile,v 1.12 2011/04/03 23:14:12 dyoung Exp $
#	$OpenBSD: Makefile,v 1.5 1997/05/19 10:34:53 pefo Exp $
#	from: @(#)Makefile	8.1 (Berkeley) 6/16/93

# Makefile for arc tags file and boot blocks

# Find where mips source files are for inclusion in tags
.include <../mips/Makefile.inc>

TARC=	${SYSDIR}/arch/arc/tags
SARC=	${SYSDIR}/arch/arc/arc/*.[ch] ${SYSDIR}/arch/arc/dev/*.[ch] \
	${SYSDIR}/arch/arc/dti/*.[ch] ${SYSDIR}/arch/arc/include/*.h \
	${SYSDIR}/arch/arc/isa/*.[ch] \
	${SYSDIR}/arch/arc/pci/*.[ch] ${SYSDIR}/arch/arc/jazz/*.[ch]
AARC=	${SYSDIR}/arch/arc/arc/*.S

# Directories in which to place tags links
DARC=	arc dev dti include isa pci jazz

.include "../../kern/Make.tags.inc"

tags:
	-rm -f ${TARC}
	-echo ${SARC} ${SMIPS} | xargs ctags -wadtf ${TARC}
	-${FINDCOMM} | xargs ctags -wadtf ${TARC}
	egrep "^LEAF(.*)|^[AN]LEAF(.*)|^NON_LEAF(.*)" ${AARC} ${AMIPS} | \
	    ${TOOL_SED} "s;\([^:]*\):\([^(]*\)(\([^, )]*\)\(.*\);\3 \1 /^\2(\3\4$$/;" \
	    >> ${TARC}
	sort -o ${TARC} ${TARC}

links:
	-for i in ${DARC}; do \
	    cd $$i && rm -f tags; ln -s ../tags tags; done


SUBDIR=	compile include stand

.include <bsd.subdir.mk>
