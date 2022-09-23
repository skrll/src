# $NetBSD: Makefile.inc,v 1.5 2022/08/06 21:31:33 riastradh Exp $


#.if defined(LIB) && (${LIB} == "kern" || ${LIB} == "c" || ${LIB} == "pthread" \
#	|| ${LIB} == "rump")



.if defined(LIB) && (${LIB} == "kern" || ${LIB} == "c")

AORDISTDIR:=${NETBSDSRCDIR}/external/mit/aor/dist

.PATH: ${AORDISTDIR}/string/aarch64

# memchr uses SIMD
# memrchr
# memcmp uses SIMD
# memset uses SIMD
# stpcpy
# strlen uses SIMD
# strchr
# strcpy uses SIMD
# strnlen
# strrchr

FUNCS=	memcpy  \
	strcmp  \
	strncmp

.for func in ${FUNCS}
CPPFLAGS.${func}.S+=	-D__${func}_aarch64=${func}
SRCS.string+=	${func}.S
.endfor

SRCS+=		${SRCS.string}

.endif
