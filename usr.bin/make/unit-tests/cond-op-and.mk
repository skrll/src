# $NetBSD: cond-op-and.mk,v 1.4 2020/09/10 22:38:57 rillig Exp $
#
# Tests for the && operator in .if conditions.

.if 0 && 0
.error
.endif

.if 1 && 0
.error
.endif

.if 0 && 1
.error
.endif

.if !(1 && 1)
.error
.endif

# The right-hand side is not evaluated since the left-hand side is already
# false.
.if 0 && ${UNDEF}
.endif

# The && operator may be abbreviated as &.  This is not widely known though
# and is also not documented in the manual page.

.if 0 & 0
.  error
.endif
.if 1 & 0
.  error
.endif
.if 0 & 1
.  error
.endif
.if !(1 & 1)
.  error
.endif

# There is no operator &&&.
.if 0 &&& 0
.  error
.endif

all:
	@:;
