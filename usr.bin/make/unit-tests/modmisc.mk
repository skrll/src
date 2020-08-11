# $Id: modmisc.mk,v 1.41 2020/08/09 09:32:04 rillig Exp $
#
# miscellaneous modifier tests

# do not put any dirs in this list which exist on some
# but not all target systems - an exists() check is below.
path=:/bin:/tmp::/:.:/no/such/dir:.
# strip cwd from path.
MOD_NODOT=S/:/ /g:N.:ts:
# and decorate, note that $'s need to be doubled. Also note that 
# the modifier_variable can be used with other modifiers.
MOD_NODOTX=S/:/ /g:N.:@d@'$$d'@
# another mod - pretend it is more interesting
MOD_HOMES=S,/home/,/homes/,
MOD_OPT=@d@$${exists($$d):?$$d:$${d:S,/usr,/opt,}}@
MOD_SEP=S,:, ,g

all:	modvar modvarloop modsysv mod-HTE emptyvar undefvar
all:	mod-subst
all:	mod-subst-delimiter
all:	mod-subst-chain
all:	mod-regex
all:	mod-loop-varname mod-loop-resolve mod-loop-varname-dollar
all:	mod-subst-dollar mod-loop-dollar
all:	mod-regex-limits
all:	mod-regex-errors
all:	mod-assign
all:	mod-assign-nested
all:	mod-tu-space
all:	mod-quote
all:	mod-break-many-words
all:	mod-remember
all:	mod-gmtime
all:	mod-gmtime-indirect
all:	mod-localtime
all:	mod-hash
all:	mod-range

# See also sysv.mk.
modsysv:
	@echo "The answer is ${libfoo.a:L:libfoo.a=42}"

# Demonstrates modifiers that are given indirectly from a variable.
modvar:
	@echo "path='${path}'"
	@echo "path='${path:${MOD_NODOT}}'"
	@echo "path='${path:S,home,homes,:${MOD_NODOT}}'"
	@echo "path=${path:${MOD_NODOTX}:ts:}"
	@echo "path=${path:${MOD_HOMES}:${MOD_NODOTX}:ts:}"

.for d in ${path:${MOD_SEP}:N.} /usr/xbin
path_$d?= ${d:${MOD_OPT}:${MOD_HOMES}}/
paths+= ${d:${MOD_OPT}:${MOD_HOMES}}
.endfor

modvarloop:
	@echo "path_/usr/xbin=${path_/usr/xbin}"
	@echo "paths=${paths}"
	@echo "PATHS=${paths:tu}"

PATHNAMES=	a/b/c def a.b.c a.b/c a a.a .gitignore a a.a
mod-HTE:
	@echo "dirname of '"${PATHNAMES:Q}"' is '"${PATHNAMES:H:Q}"'"
	@echo "basename of '"${PATHNAMES:Q}"' is '"${PATHNAMES:T:Q}"'"
	@echo "suffix of '"${PATHNAMES:Q}"' is '"${PATHNAMES:E:Q}"'"
	@echo "root of '"${PATHNAMES:Q}"' is '"${PATHNAMES:R:Q}"'"

# When a modifier is applied to the "" variable, the result is discarded.
emptyvar:
	@echo S:${:S,^$,empty,}
	@echo C:${:C,^$,empty,}
	@echo @:${:@var@${var}@}

# The :U modifier turns even the "" variable into something that has a value.
# The resulting variable is empty, but is still considered to contain a
# single empty word. This word can be accessed by the :S and :C modifiers,
# but not by the :@ modifier since it explicitly skips empty words.
undefvar:
	@echo S:${:U:S,^$,empty,}
	@echo C:${:U:C,^$,empty,}
	@echo @:${:U:@var@empty@}

WORDS=		sequences of letters
.if ${WORDS:S,,,} != ${WORDS}
.warning The empty pattern matches something.
.endif
.if ${WORDS:S,e,*,1} != "s*quences of letters"
.warning The :S modifier flag '1' is not applied exactly once.
.endif
.if ${WORDS:S,e,*,} != "s*quences of l*tters"
.warning The :S modifier does not replace every first match per word.
.endif
.if ${WORDS:S,e,*,g} != "s*qu*nc*s of l*tt*rs"
.warning The :S modifier flag 'g' does not replace every occurrence.
.endif
.if ${WORDS:S,^sequ,occurr,} != "occurrences of letters"
.warning The :S modifier fails for a short match anchored at the start.
.endif
.if ${WORDS:S,^of,with,} != "sequences with letters"
.warning The :S modifier fails for an exact match anchored at the start.
.endif
.if ${WORDS:S,^office,does not match,} != ${WORDS}
.warning The :S modifier matches a too long pattern anchored at the start.
.endif
.if ${WORDS:S,f$,r,} != "sequences or letters"
.warning The :S modifier fails for a short match anchored at the end.
.endif
.if ${WORDS:S,s$,,} != "sequence of letter"
.warning The :S modifier fails to replace one occurrence per word.
.endif
.if ${WORDS:S,of$,,} != "sequences letters"
.warning The :S modifier fails for an exact match anchored at the end.
.endif
.if ${WORDS:S,eof$,,} != ${WORDS}
.warning The :S modifier matches a too long pattern anchored at the end.
.endif
.if ${WORDS:S,^of$,,} != "sequences letters"
.warning The :S modifier does not match a word anchored at both ends.
.endif
.if ${WORDS:S,^o$,,} != ${WORDS}
.warning The :S modifier matches a prefix anchored at both ends.
.endif
.if ${WORDS:S,^f$,,} != ${WORDS}
.warning The :S modifier matches a suffix anchored at both ends.
.endif
.if ${WORDS:S,^eof$,,} != ${WORDS}
.warning The :S modifier matches a too long prefix anchored at both ends.
.endif
.if ${WORDS:S,^office$,,} != ${WORDS}
.warning The :S modifier matches a too long suffix anchored at both ends.
.endif

mod-subst:
	@echo $@:
	@echo :${:Ua b b c:S,a b,,:Q}:
	@echo :${:Ua b b c:S,a b,,1:Q}:
	@echo :${:Ua b b c:S,a b,,W:Q}:
	@echo :${:Ua b b c:S,b,,g:Q}:
	@echo :${:U1 2 3 1 2 3:S,1 2,___,Wg:S,_,x,:Q}:
	@echo ${:U12345:S,,sep,g:Q}

# The :S and :C modifiers accept an arbitrary character as the delimiter,
# including characters that are otherwise used as escape characters or
# interpreted in a special way.  This can be used to confuse humans.
mod-subst-delimiter:
	@echo $@:
	@echo ${:U1 2 3:S	2	two	:Q} horizontal tabulator
	@echo ${:U1 2 3:S 2 two :Q} space
	@echo ${:U1 2 3:S!2!two!:Q} exclamation mark
	@echo ${:U1 2 3:S"2"two":Q} double quotes
	# In shell command lines, the hash does not need to be escaped.
	# It needs to be escaped in variable assignment lines though.
	@echo ${:U1 2 3:S#2#two#:Q} hash
	@echo ${:U1 2 3:S$2$two$:Q} dollar
	@echo ${:U1 2 3:S%2%two%:Q} percent
	@echo ${:U1 2 3:S'2'two':Q} apostrophe
	@echo ${:U1 2 3:S(2(two(:Q} opening parenthesis
	@echo ${:U1 2 3:S)2)two):Q} closing parenthesis
	@echo ${:U1 2 3:S121two1:Q} digit
	@echo ${:U1 2 3:S:2:two::Q} colon
	@echo ${:U1 2 3:S<2<two<:Q} less than sign
	@echo ${:U1 2 3:S=2=two=:Q} equal sign
	@echo ${:U1 2 3:S>2>two>:Q} greater than sign
	@echo ${:U1 2 3:S?2?two?:Q} question mark
	@echo ${:U1 2 3:S@2@two@:Q} at
	@echo ${:U1 2 3:Sa2atwoa:Q} letter
	@echo ${:U1 2 3:S[2[two[:Q} opening bracket
	@echo ${:U1 2 3:S\2\two\:Q} backslash
	@echo ${:U1 2 3:S]2]two]:Q} closing bracket
	@echo ${:U1 2 3:S^2^two^:Q} caret
	@echo ${:U1 2 3:S{2{two{:Q} opening brace
	@echo ${:U1 2 3:S|2|two|:Q} vertical line
	@echo ${:U1 2 3:S}2}two}:Q} closing brace
	@echo ${:U1 2 3:S~2~two~:Q} tilde

# The :S and :C modifiers can be chained without a separating ':'.
# This is not documented in the manual page.
# It works because ApplyModifier_Subst scans for the known modifiers g1W
# and then just returns to ApplyModifiers.  There, the colon is optionally
# skipped (see the *st.next == ':' at the end of the loop).
#
# Most other modifiers cannot be chained since their parsers skip until
# the next ':' or '}' or ')'.
mod-subst-chain:
	@echo $@:
	@echo ${:Ua b c:S,a,A,S,b,B,}.
	# There is no 'i' modifier for the :S or :C modifiers.
	# The error message is "make: Unknown modifier 'i'", which is
	# kind of correct, although it is mixing the terms for variable
	# modifiers with the matching modifiers.
	@echo ${:Uvalue:S,a,x,i}.

mod-regex:
	@echo $@:
	@echo :${:Ua b b c:C,a b,,:Q}:
	@echo :${:Ua b b c:C,a b,,1:Q}:
	@echo :${:Ua b b c:C,a b,,W:Q}:
	@echo :${:Uword1 word2:C,****,____,g:C,word,____,:Q}:
	@echo :${:Ua b b c:C,b,,g:Q}:
	@echo :${:U1 2 3 1 2 3:C,1 2,___,Wg:C,_,x,:Q}:

# In the :@ modifier, the name of the loop variable can even be generated
# dynamically.  There's no practical use-case for this, and hopefully nobody
# will ever depend on this, but technically it's possible.
# Therefore, in -dL mode, this is forbidden, see lint.mk.
mod-loop-varname:
	@echo :${:Uone two three:@${:Ubar:S,b,v,}@+${var}+@:Q}:
	# ":::" is a very creative variable name, unlikely in practice
	# The expression ${\:\:\:} would not work since backslashes can only
	# be escaped in the modifiers, but not in the variable name.
	@echo :${:U1 2 3:@:::@x${${:U\:\:\:}}y@}:
	# "@@" is another creative variable name.
	@echo :${:U1 2 3:@\@\@@x${@@}y@}:
	# Even "@" works as a variable name since the variable is installed
	# in the "current" scope, which in this case is the one from the
	# target.
	@echo :$@: :${:U1 2 3:@\@@x${@}y@}: :$@:
	# In extreme cases, even the backslash can be used as variable name.
	# It needs to be doubled though.
	@echo :${:U1 2 3:@\\@x${${:Ux:S,x,\\,}}y@}:

# The :@ modifier resolves the variables a little more often than expected.
# In particular, it resolves _all_ variables from the context, and not only
# the loop variable (in this case v).
#
# The d means direct reference, the i means indirect reference.
RESOLVE=	${RES1} $${RES1}
RES1=		1d${RES2} 1i$${RES2}
RES2=		2d${RES3} 2i$${RES3}
RES3=		3

mod-loop-resolve:
	@echo $@:${RESOLVE:@v@w${v}w@:Q}:

# Until 2020-07-20, the variable name of the :@ modifier could end with one
# or two dollar signs, which were silently ignored.
# There's no point in allowing a dollar sign in that position.
mod-loop-varname-dollar:
	@echo $@:${1 2 3:L:@v$@($v)@:Q}.
	@echo $@:${1 2 3:L:@v$$@($v)@:Q}.
	@echo $@:${1 2 3:L:@v$$$@($v)@:Q}.

# No matter how many dollar characters there are, they all get merged
# into a single dollar by the :S modifier.
#
# As of 2020-08-09, this is because ParseModifierPart sees a '$' and
# calls Var_Parse to expand the variable.  In all other places, the "$$"
# is handled outside of Var_Parse.  Var_Parse therefore considers "$$"
# one of the "really stupid names", skips the first dollar, and parsing
# continues with the next character.  This repeats for the other dollar
# signs, except the one before the delimiter.  That one is handled by
# the code that optionally interprets the '$' as the end-anchor in the
# first part of the :S modifier.  That code doesn't call Var_Parse but
# simply copies the dollar to the result.
mod-subst-dollar:
	@echo $@:${:U1:S,^,$,:Q}:
	@echo $@:${:U2:S,^,$$,:Q}:
	@echo $@:${:U3:S,^,$$$,:Q}:
	@echo $@:${:U4:S,^,$$$$,:Q}:
	@echo $@:${:U5:S,^,$$$$$,:Q}:
	@echo $@:${:U6:S,^,$$$$$$,:Q}:
	@echo $@:${:U7:S,^,$$$$$$$,:Q}:
	@echo $@:${:U8:S,^,$$$$$$$$,:Q}:
	@echo $@:${:U40:S,^,$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$,:Q}:
# This generates no dollar at all:
	@echo $@:${:UU8:S,^,${:U$$$$$$$$},:Q}:
# Here is an alternative way to generate dollar characters.
# It's unexpectedly complicated though.
	@echo $@:${:U:range=5:ts\x24:C,[0-9],,g:Q}:
# In modifiers, dollars are escaped using the backslash, not using another
# dollar sign.  Therefore, creating a dollar sign is pretty simple:
	@echo $@:${:Ugood3:S,^,\$\$\$,:Q}

# Demonstrate that it is possible to generate dollar characters using the
# :@ modifier.
#
# These are edge cases that could have resulted in a parse error as well
# since the $@ at the end could have been interpreted as a variable, which
# would mean a missing closing @ delimiter.
mod-loop-dollar:
	@echo $@:${:U1:@word@${word}$@:Q}:
	@echo $@:${:U2:@word@$${word}$$@:Q}:
	@echo $@:${:U3:@word@$$${word}$$$@:Q}:
	@echo $@:${:U4:@word@$$$${word}$$$$@:Q}:
	@echo $@:${:U5:@word@$$$$${word}$$$$$@:Q}:
	@echo $@:${:U6:@word@$$$$$${word}$$$$$$@:Q}:

mod-regex-limits:
	@echo $@:00-ok:${:U1 23 456:C,..,\0\0,:Q}
	@echo $@:11-missing:${:U1 23 456:C,..,\1\1,:Q}
	@echo $@:11-ok:${:U1 23 456:C,(.).,\1\1,:Q}
	@echo $@:22-missing:${:U1 23 456:C,..,\2\2,:Q}
	@echo $@:22-missing:${:U1 23 456:C,(.).,\2\2,:Q}
	@echo $@:22-ok:${:U1 23 456:C,(.)(.),\2\2,:Q}
	# The :C modifier only handles single-digit capturing groups,
	# which is more than enough for daily use.
	@echo $@:capture:${:UabcdefghijABCDEFGHIJrest:C,(.)(.)(.)(.)(.)(.)(.)(.)(.)(.)(.)(.)(.)(.)(.)(.)(.)(.)(.)(.),\9\8\7\6\5\4\3\2\1\0\10\11\12,}

mod-regex-errors:
	@echo $@: ${UNDEF:Uvalue:C,[,,}

# Just a bit of basic code coverage for the obscure ::= assignment modifiers.
mod-assign:
	@echo $@: ${1 2 3:L:@i@${FIRST::?=$i}@} first=${FIRST}.
	@echo $@: ${1 2 3:L:@i@${LAST::=$i}@} last=${LAST}.
	@echo $@: ${1 2 3:L:@i@${APPENDED::+=$i}@} appended=${APPENDED}.
	@echo $@: ${echo.1 echo.2 echo.3:L:@i@${RAN::!=${i:C,.*,&; & 1>\&2,:S,., ,g}}@} ran:${RAN}.
	# The assignments happen in the global scope and thus are
	# preserved even after the shell command has been run.
	@echo $@: global: ${FIRST:Q}, ${LAST:Q}, ${APPENDED:Q}, ${RAN:Q}.

mod-assign-nested:
	@echo $@: ${1:?${THEN1::=then1${IT1::=t1}}:${ELSE1::=else1${IE1::=e1}}}${THEN1}${ELSE1}${IT1}${IE1}
	@echo $@: ${0:?${THEN2::=then2${IT2::=t2}}:${ELSE2::=else2${IE2::=e2}}}${THEN2}${ELSE2}${IT2}${IE2}
	@echo $@: ${SINK3:Q}
	@echo $@: ${SINK4:Q}
SINK3:=	${1:?${THEN3::=then3${IT3::=t3}}:${ELSE3::=else3${IE3::=e3}}}${THEN3}${ELSE3}${IT3}${IE3}
SINK4:=	${0:?${THEN4::=then4${IT4::=t4}}:${ELSE4::=else4${IE4::=e4}}}${THEN4}${ELSE4}${IT4}${IE4}

mod-tu-space:
	# The :tu and :tl modifiers operate on the variable value
	# as a single string, not as a list of words. Therefore,
	# the adjacent spaces are preserved.
	@echo $@: ${a   b:L:tu:Q}

mod-quote:
	@echo $@: new${.newline:Q}${.newline:Q}line

# Cover the bmake_realloc in brk_string.
mod-break-many-words:
	@echo $@: ${UNDEF:U:range=500:[#]}

# Demonstrate the :_ modifier.
# In the parameterized form, having the variable name on the right side
# of the = assignment operator is confusing. Luckily this modifier is
# only rarely needed.
mod-remember:
	@echo $@: ${1 2 3:L:_:@var@${_}@}
	@echo $@: ${1 2 3:L:@var@${var:_=SAVED:}@}, SAVED=${SAVED}

mod-gmtime:
	@echo $@:
	@echo ${%Y:L:gmtim=1593536400}		# modifier name too short
	@echo ${%Y:L:gmtime=1593536400}		# 2020-07-01T00:00:00Z
	@echo ${%Y:L:gmtimer=1593536400}	# modifier name too long
	@echo ${%Y:L:gm=gm:M*}

mod-gmtime-indirect:
	@echo $@:
	# It's not possible to pass the seconds via a variable expression.
	# Parsing of the :gmtime modifier stops at the '$' and returns to
	# ApplyModifiers.  There, a colon would be skipped but not a dollar.
	# Parsing continues by looking at the next modifier.  Now the ${:U}
	# is expanded and interpreted as a variable modifier, which results
	# in the error message "Unknown modifier '1'".
	@echo ${%Y:L:gmtime=${:U1593536400}}

mod-localtime:
	@echo $@:
	@echo ${%Y:L:localtim=1593536400}	# modifier name too short
	@echo ${%Y:L:localtime=1593536400}	# 2020-07-01T00:00:00Z
	@echo ${%Y:L:localtimer=1593536400}	# modifier name too long

mod-hash:
	@echo $@:
	@echo ${12345:L:has}			# modifier name too short
	@echo ${12345:L:hash}			# ok
	@echo ${12345:L:hash=SHA-256}		# :hash does not accept '='
	@echo ${12345:L:hasX}			# misspelled
	@echo ${12345:L:hashed}			# modifier name too long

mod-range:
	@echo $@:
	@echo ${a b c:L:rang}			# modifier name too short
	@echo ${a b c:L:range}			# ok
	@echo ${a b c:L:rango}			# misspelled
	@echo ${a b c:L:ranger}			# modifier name too long

# To apply a modifier indirectly via another variable, the whole
# modifier must be put into a single variable.
.if ${value:L:${:US}${:U,value,replacement,}} != "S,value,replacement,}"
.warning unexpected
.endif

# Adding another level of indirection (the 2 nested :U expressions) helps.
.if ${value:L:${:U${:US}${:U,value,replacement,}}} != "replacement"
.warning unexpected
.endif

# Multiple indirect modifiers can be applied one after another as long as
# they are separated with colons.
.if ${value:L:${:US,a,A,}:${:US,e,E,}} != "vAluE"
.warning unexpected
.endif

# An indirect variable that evaluates to the empty string is allowed though.
# This makes it possible to define conditional modifiers, like this:
#
# M.little-endian=	S,1234,4321,
# M.big-endian=		# none
.if ${value:L:${:Dempty}S,a,A,} != "vAlue"
.warning unexpected
.endif

# begin mod-shell

.if ${:!echo hello | tr 'l' 'l'!} != "hello"
.warning unexpected
.endif

# The output is truncated at the first null byte.
# Cmd_Exec returns only a string pointer without length information.
.if ${:!echo hello | tr 'l' '\0'!} != "he"
.warning unexpected
.endif

.if ${:!echo!} != ""
.warning A newline at the end of the output must be stripped.
.endif

.if ${:!echo;echo!} != " "
.warning Only a single newline at the end of the output is stripped.
.endif

.if ${:!echo;echo;echo;echo!} != "   "
.warning Other newlines in the output are converted to spaces.
.endif

# end mod-shell
