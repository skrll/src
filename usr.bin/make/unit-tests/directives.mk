# $NetBSD: directives.mk,v 1.1 2020/07/27 20:46:17 rillig Exp $
#
# Tests for parsing directives, in the same order as in the manual page.
#
# Each test group has 10 lines, to keep the expected file stable.
#
# no tests for .error since it exits immediately, see ParseMessage.


.info begin .export tests
.expor				# misspelled
.export				# oops: missing argument
.export VARNAME
.exporting works		# oops: misspelled





.info begin .export-env tests
.export-en			# oops: misspelled
.export-env
.export-environment		# oops: misspelled






.info begin .export-literal tests
.export-litera			# oops: misspelled
.export-literal			# oops: missing argument
.export-literal VARNAME
.export-literally		# oops: misspelled





.info begin .info tests
.inf				# misspelled
.info				# oops: message should be "missing parameter"
.info message
.info		indented message
.information
.information message		# oops: misspelled



.info begin .undef tests
.unde				# misspelled
.undef				# oops: missing argument
.undefined			# oops: misspelled
.undef VARNAME





.info begin .unexport tests
.unexpor			# misspelled
.unexport			# oops: missing argument
.unexport VARNAME		# ok
.unexporting works		# oops: misspelled





.info begin .unexport-env tests
.unexport-en			# misspelled
.unexport-env			# ok
.unexport-environment		# oops: misspelled






.info begin .warning tests
.warn				# misspelled
.warnin				# misspelled
.warning			# oops: should be "missing argument"
.warning message		# ok
.warnings			# misspelled
.warnings messages		# oops



.info begin .elif misspellings tests, part 1
.if 1
.elif 1				# ok
.elsif 1			# oops: misspelled
.elseif 1			# oops: misspelled
.endif




.info begin .elif misspellings tests, part 2
.if 0
.elif 0				# ok
.elsif 0			# oops: misspelled
.elseif 0			# oops: misspelled
.endif




.info begin .elif misspellings tests, part 3
.if 0
.elsif 0			# oops: misspelled
.endif
.if 0
.elseif 0			# oops: misspelled
.endif



.info end of the tests

all:
	@:
