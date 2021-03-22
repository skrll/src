/*	$NetBSD: msg_193.c,v 1.7 2021/03/21 15:44:57 rillig Exp $	*/
# 3 "msg_193.c"

// Test for message: statement not reached [193]

/*
 * Test the reachability of statements in a function.
 *
 *	if
 *	if-else
 *	if-else-if-else
 *	for
 *	while
 *	do-while
 *	switch
 *	break
 *	continue
 *	goto
 *	return
 *
 *	constant expression
 *	system-dependent constant expression
 */

extern void
reachable(void);
extern void
unreachable(void);

void
test_statement(void)
{
	reachable();
	reachable();
}

void
test_compound_statement(void)
{
	reachable();
	{
		reachable();
		reachable();
	}
	reachable();
}

void
test_if_statement(void)
{
	if (1)
		reachable();
	reachable();
	if (0)
		unreachable();		/* expect: 193 */
	reachable();
}

void
test_if_compound_statement(void)
{
	if (1) {
		reachable();
	}
	if (1) {
		{
			{
				reachable();
			}
		}
	}

	if (0) {
		unreachable();		/* expect: 193 */
	}
	if (0) {
		{
			{
				unreachable();	/* expect: 193 */
			}
		}
	}
}

void
test_if_without_else(void)
{
	if (1)
		reachable();
	reachable();

	if (0)
		unreachable();		/* expect: 193 */
	reachable();
}

void
test_if_with_else(void)
{
	if (1)
		reachable();
	else
		unreachable();		/* expect: 193 */
	reachable();

	if (0)
		unreachable();		/* expect: 193 */
	else
		reachable();
	reachable();
}

void
test_if_else_if_else(void)
{
	if (1)
		reachable();
	else if (1)			/* expect: 193 */
		unreachable();
	else
		unreachable();		/* expect: 193 */

	if (0)
		unreachable();		/* expect: 193 */
	else if (1)
		reachable();
	else
		unreachable();		/* expect: 193 */

	if (0)
		unreachable();		/* expect: 193 */
	else if (0)
		unreachable();		/* expect: 193 */
	else
		reachable();
}

void
test_if_return(void)
{
	if (1)
		return;
	unreachable();			/* expect: 193 */
}

void
test_if_else_return(void)
{
	if (1)
		reachable();
	else
		return;			/* expect: 193 */
	reachable();
}

void
test_for_forever(void)
{
	for (;;)
		reachable();
	unreachable();			/* expect: 193 */
}

void
test_for_true(void)
{
	for (; 1;)
		reachable();
	unreachable();			/* expect: 193 */
}

void
test_for_false(void)
{
	for (; 0;)
		unreachable();		/* expect: 193 */
	reachable();
}

void
test_for_break(void)
{
	for (;;) {
		reachable();
		break;
		unreachable();		/* expect: 193 */
	}
	reachable();
}

void
test_for_if_break(void)
{
	for (;;) {
		reachable();
		if (0) {
			unreachable();	/* expect: 193 */
			break;
			unreachable();	/* expect: 193 */
		}
		if (1) {
			reachable();
			break;
			unreachable();	/* expect: 193 */
		}
		unreachable();		/* TODO: expect: 193 */
	}
	reachable();
}

void
test_for_continue(void)
{
	for (;;) {
		reachable();
		continue;
		unreachable();		/* expect: 193 */
	}
	unreachable();			/* expect: 193 */
}

void
test_for_if_continue(void)
{
	for (;;) {
		reachable();
		if (0) {
			unreachable();	/* expect: 193 */
			continue;
			unreachable();	/* expect: 193 */
		}
		if (1) {
			reachable();
			continue;
			unreachable();	/* expect: 193 */
		}
		unreachable();		/* TODO: expect: 193 */
	}
	unreachable();			/* expect: 193 */
}

void
test_for_return(void)
{
	for (;;) {
		reachable();
		return;
		unreachable();		/* expect: 193 */
	}
	unreachable();			/* expect: 193 */
}

void
test_for_if_return(void)
{
	for (;;) {
		reachable();
		if (0) {
			unreachable();	/* expect: 193 */
			return;
			unreachable();	/* expect: 193 */
		}
		if (1) {
			reachable();
			return;
			unreachable();	/* expect: 193 */
		}
		unreachable();		/* TODO: expect: 193 */
	}
	unreachable();			/* expect: 193 */
}

void
test_while_true(void)
{
	while (1)
		reachable();
	unreachable();			/* expect: 193 */
}

void
test_while_false(void)
{
	while (0)
		unreachable();		/* expect: 193 */
	reachable();
}

void
test_while_break(void)
{
	while (1) {
		reachable();
		break;
		unreachable();		/* expect: 193 */
	}
	reachable();
}

void
test_while_if_break(void)
{
	while (1) {
		reachable();
		if (0) {
			unreachable();	/* expect: 193 */
			break;
			unreachable();	/* expect: 193 */
		}
		if (1) {
			reachable();
			break;
			unreachable();	/* expect: 193 */
		}
		unreachable();		/* TODO: expect: 193 */
	}
	reachable();
}

void
test_while_continue(void)
{
	while (1) {
		reachable();
		continue;
		unreachable();		/* expect: 193 */
	}
	unreachable();			/* expect: 193 */
}

void
test_while_if_continue(void)
{
	while (1) {
		reachable();
		if (0) {
			unreachable();	/* expect: 193 */
			continue;
			unreachable();	/* expect: 193 */
		}
		if (1) {
			reachable();
			continue;
			unreachable();	/* expect: 193 */
		}
		unreachable();		/* TODO: expect: 193 */
	}
	unreachable();			/* expect: 193 */
}

void
test_while_return(void)
{
	while (1) {
		reachable();
		return;
		unreachable();		/* expect: 193 */
	}
	unreachable();			/* expect: 193 */
}

void
test_while_if_return(void)
{
	while (1) {
		reachable();
		if (0) {
			unreachable();	/* expect: 193 */
			return;
			unreachable();	/* expect: 193 */
		}
		if (1) {
			reachable();
			return;
			unreachable();	/* expect: 193 */
		}
		unreachable();		/* TODO: expect: 193 */
	}
	unreachable();			/* expect: 193 */
}

void
test_do_while_true(void)
{
	do {
		reachable();
	} while (1);
	unreachable();			/* expect: 193 */
}

void
test_do_while_false(void)
{
	do {
		reachable();
	} while (0);
	reachable();
}

void
test_do_while_break(void)
{
	do {
		reachable();
		break;
		unreachable();		/* expect: 193 */
	} while (1);
	reachable();
}

void
test_do_while_if_break(void)
{
	do {
		reachable();
		if (0) {
			unreachable();	/* expect: 193 */
			break;
			unreachable();	/* expect: 193 */
		}
		if (1) {
			reachable();
			break;
			unreachable();	/* expect: 193 */
		}
		unreachable();		/* TODO: expect: 193 */
	} while (1);
	reachable();
}

void
test_do_while_continue(void)
{
	do {
		reachable();
		continue;
		unreachable();		/* expect: 193 */
	} while (1);
	unreachable();			/* expect: 193 */
}

void
test_do_while_if_continue(void)
{
	do {
		reachable();
		if (0) {
			unreachable();	/* expect: 193 */
			continue;
			unreachable();	/* expect: 193 */
		}
		if (1) {
			reachable();
			continue;
			unreachable();	/* expect: 193 */
		}
		unreachable();		/* TODO: expect: 193 */
	} while (1);
	unreachable();			/* expect: 193 */
}

void
test_do_while_return(void)
{
	do {
		reachable();
		return;
		unreachable();		/* expect: 193 */
	} while (1);
	unreachable();			/* expect: 193 */
}

void
test_do_while_if_return(void)
{
	do {
		reachable();
		if (0) {
			unreachable();	/* expect: 193 */
			return;
			unreachable();	/* expect: 193 */
		}
		if (1) {
			reachable();
			return;
			unreachable();	/* expect: 193 */
		}
		unreachable();		/* TODO: expect: 193 */
	} while (1);
	unreachable();			/* expect: 193 */
}

/* TODO: switch */

/* TODO: goto */

/* TODO: system-dependent constant expression (see tn_system_dependent) */
