/*	$NetBSD: c99_init_designator.c,v 1.1 2021/06/20 18:09:48 rillig Exp $	*/
# 3 "c99_init_designator.c"

/*
 * Test initialization of structs or unions using designators.
 *
 * See init.c, 'struct designator' and 'struct designation'.
 *
 * C99 6.7.8p6, 6.7.8p7
 */

struct point {
	int x;
	int y;
};

/*
 * Before cgram.y 1.230 from 2021-06-20, the grammar allowed either of the
 * operators '.' or '->' to be used for the designators and had extra code
 * to ensure that only '.' was actually used.
 */
struct point origin = {
    .x = 0,
    ->y = 0,			/* expect: syntax error '->' */
};

/* Ensure that the parser can recover from the parse error. */
struct point pythagoras = { 3, 4 };
