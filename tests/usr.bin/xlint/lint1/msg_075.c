/*	$NetBSD: msg_075.c,v 1.2 2021/01/08 21:25:03 rillig Exp $	*/
# 3 "msg_075.c"

// Test for message: overflow in hex escape [75]

char str[] = "\x12345678123456781234567812345678";
