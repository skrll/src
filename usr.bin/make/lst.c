/* $NetBSD: lst.c,v 1.74 2020/09/30 06:27:02 rillig Exp $ */

/*
 * Copyright (c) 1988, 1989, 1990, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Adam de Boor.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "make.h"

MAKE_RCSID("$NetBSD: lst.c,v 1.74 2020/09/30 06:27:02 rillig Exp $");

/* Allocate and initialize a list node.
 *
 * The fields 'prev' and 'next' must be initialized by the caller.
 */
static ListNode *
LstNodeNew(void *datum)
{
    ListNode *node = bmake_malloc(sizeof *node);
    node->priv_useCount = 0;
    node->priv_deleted = FALSE;
    node->datum = datum;
    return node;
}

static Boolean
LstIsEmpty(List *list)
{
    return list->first == NULL;
}

/* Create and initialize a new, empty list. */
List *
Lst_Init(void)
{
    List *list = bmake_malloc(sizeof *list);

    list->first = NULL;
    list->last = NULL;
    list->priv_isOpen = FALSE;
    list->priv_lastAccess = Unknown;

    return list;
}

/* Duplicate an entire list, usually by copying the datum pointers.
 * If copyProc is given, that function is used to create the new datum from the
 * old datum, usually by creating a copy of it. */
List *
Lst_Copy(List *list, LstCopyProc copyProc)
{
    List *newList;
    ListNode *node;

    newList = Lst_Init();

    for (node = list->first; node != NULL; node = node->next) {
	void *datum = copyProc != NULL ? copyProc(node->datum) : node->datum;
	Lst_Append(newList, datum);
    }

    return newList;
}

/* Free a list and all its nodes. The list data itself are not freed though. */
void
Lst_Free(List *list)
{
    ListNode *node;
    ListNode *next;

    for (node = list->first; node != NULL; node = next) {
	next = node->next;
	free(node);
    }

    free(list);
}

/* Destroy a list and free all its resources. The freeProc is called with the
 * datum from each node in turn before the node is freed. */
void
Lst_Destroy(List *list, LstFreeProc freeProc)
{
    ListNode *node;
    ListNode *next;

    for (node = list->first; node != NULL; node = next) {
	next = node->next;
	freeProc(node->datum);
	free(node);
    }

    free(list);
}

/*
 * Functions to modify a list
 */

/* Insert a new node with the given piece of data before the given node in the
 * given list. */
void
Lst_InsertBefore(List *list, ListNode *node, void *datum)
{
    ListNode *newNode;

    assert(!LstIsEmpty(list));
    assert(datum != NULL);

    newNode = LstNodeNew(datum);
    newNode->prev = node->prev;
    newNode->next = node;

    if (node->prev != NULL) {
	node->prev->next = newNode;
    }
    node->prev = newNode;

    if (node == list->first) {
	list->first = newNode;
    }
}

/* Add a piece of data at the start of the given list. */
void
Lst_Prepend(List *list, void *datum)
{
    ListNode *node;

    assert(datum != NULL);

    node = LstNodeNew(datum);
    node->prev = NULL;
    node->next = list->first;

    if (list->first == NULL) {
	list->first = node;
	list->last = node;
    } else {
	list->first->prev = node;
	list->first = node;
    }
}

/* Add a piece of data at the end of the given list. */
void
Lst_Append(List *list, void *datum)
{
    ListNode *node;

    assert(datum != NULL);

    node = LstNodeNew(datum);
    node->prev = list->last;
    node->next = NULL;

    if (list->last == NULL) {
	list->first = node;
	list->last = node;
    } else {
	list->last->next = node;
	list->last = node;
    }
}

/* Remove the given node from the given list.
 * The datum stored in the node must be freed by the caller, if necessary. */
void
Lst_Remove(List *list, ListNode *node)
{
    /*
     * unlink it from the list
     */
    if (node->next != NULL) {
	node->next->prev = node->prev;
    }
    if (node->prev != NULL) {
	node->prev->next = node->next;
    }

    /*
     * if either the first or last of the list point to this node,
     * adjust them accordingly
     */
    if (list->first == node) {
	list->first = node->next;
    }
    if (list->last == node) {
	list->last = node->prev;
    }

    /*
     * Sequential access stuff. If the node we're removing is the current
     * node in the list, reset the current node to the previous one. If the
     * previous one was non-existent (prev == NULL), we set the
     * end to be Unknown, since it is.
     */
    if (list->priv_isOpen && list->priv_curr == node) {
	list->priv_curr = list->priv_prev;
	if (list->priv_curr == NULL) {
	    list->priv_lastAccess = Unknown;
	}
    }

    /*
     * note that the datum is unmolested. The caller must free it as
     * necessary and as expected.
     */
    if (node->priv_useCount == 0) {
	free(node);
    } else {
	node->priv_deleted = TRUE;
    }
}

/* Replace the datum in the given node with the new datum. */
void
LstNode_Set(ListNode *node, void *datum)
{
    assert(datum != NULL);

    node->datum = datum;
}

/* Replace the datum in the given node to NULL.
 * Having NULL values in a list is unusual though. */
void
LstNode_SetNull(ListNode *node)
{
    node->datum = NULL;
}


/*
 * Functions for entire lists
 */

/* Return the first node from the list for which the match function returns
 * TRUE, or NULL if none of the nodes matched. */
ListNode *
Lst_Find(List *list, LstFindProc match, const void *matchArgs)
{
    return Lst_FindFrom(list, Lst_First(list), match, matchArgs);
}

/* Return the first node from the list, starting at the given node, for which
 * the match function returns TRUE, or NULL if none of the nodes matches.
 *
 * The start node may be NULL, in which case nothing is found. */
ListNode *
Lst_FindFrom(List *list, ListNode *node, LstFindProc match, const void *matchArgs)
{
    ListNode *tln;

    assert(list != NULL);
    assert(match != NULL);

    for (tln = node; tln != NULL; tln = tln->next) {
	if (match(tln->datum, matchArgs))
	    return tln;
    }

    return NULL;
}

/* Return the first node that contains the given datum, or NULL. */
ListNode *
Lst_FindDatum(List *list, const void *datum)
{
    ListNode *node;

    assert(datum != NULL);

    for (node = list->first; node != NULL; node = node->next) {
	if (node->datum == datum) {
	    return node;
	}
    }

    return NULL;
}

void
Lst_ForEach(List *list, LstActionProc proc, void *procData)
{
    ListNode *node;
    for (node = list->first; node != NULL; node = node->next)
        proc(node->datum, procData);
}

/* Apply the given function to each element of the given list. The function
 * should return 0 if traversal should continue and non-zero if it should
 * abort. */
int
Lst_ForEachUntil(List *list, LstActionUntilProc proc, void *procData)
{
    ListNode *tln = list->first;
    int result = 0;

    while (tln != NULL) {
	/*
	 * Take care of having the current element deleted out from under
	 * us.
	 */
	ListNode *next = tln->next;

	/*
	 * We're done with the traversal if
	 *  - the next node to examine doesn't exist and
	 *  - nothing's been added after the current node (check this
	 *    after proc() has been called).
	 */
	Boolean done = next == NULL;

	tln->priv_useCount++;
	result = proc(tln->datum, procData);
	tln->priv_useCount--;

	/*
	 * Now check whether a node has been added.
	 * Note: this doesn't work if this node was deleted before
	 *       the new node was added.
	 */
	if (next != tln->next) {
	    next = tln->next;
	    done = 0;
	}

	if (tln->priv_deleted) {
	    free((char *)tln);
	}
	tln = next;
	if (result || LstIsEmpty(list) || done)
	    break;
    }

    return result;
}

/* Move all nodes from list2 to the end of list1.
 * List2 is destroyed and freed. */
void
Lst_MoveAll(List *list1, List *list2)
{
    if (list2->first != NULL) {
	list2->first->prev = list1->last;
	if (list1->last != NULL) {
	    list1->last->next = list2->first;
	} else {
	    list1->first = list2->first;
	}
	list1->last = list2->last;
    }
    free(list2);
}

/* Copy the element data from src to the start of dst. */
void
Lst_PrependAll(List *dst, List *src)
{
    ListNode *node;
    for (node = src->last; node != NULL; node = node->prev)
	Lst_Prepend(dst, node->datum);
}

/* Copy the element data from src to the end of dst. */
void
Lst_AppendAll(List *dst, List *src)
{
    ListNode *node;
    for (node = src->first; node != NULL; node = node->next)
	Lst_Append(dst, node->datum);
}

/*
 * these functions are for dealing with a list as a table, of sorts.
 * An idea of the "current element" is kept and used by all the functions
 * between Lst_Open() and Lst_Close().
 *
 * The sequential functions access the list in a slightly different way.
 * CurPtr points to their idea of the current node in the list and they
 * access the list based on it.
 */

/* Open a list for sequential access. A list can still be searched, etc.,
 * without confusing these functions. */
void
Lst_Open(List *list)
{
    assert(!list->priv_isOpen);

    list->priv_isOpen = TRUE;
    list->priv_lastAccess = LstIsEmpty(list) ? Head : Unknown;
    list->priv_curr = NULL;
}

/* Return the next node for the given list, or NULL if the end has been
 * reached. */
ListNode *
Lst_Next(List *list)
{
    ListNode *node;

    assert(list->priv_isOpen);

    list->priv_prev = list->priv_curr;

    if (list->priv_curr == NULL) {
	if (list->priv_lastAccess == Unknown) {
	    /*
	     * If we're just starting out, lastAccess will be Unknown.
	     * Then we want to start this thing off in the right
	     * direction -- at the start with lastAccess being Middle.
	     */
	    list->priv_curr = node = list->first;
	    list->priv_lastAccess = Middle;
	} else {
	    node = NULL;
	    list->priv_lastAccess = Tail;
	}
    } else {
	node = list->priv_curr->next;
	list->priv_curr = node;

	if (node == list->first || node == NULL) {
	    /*
	     * If back at the front, then we've hit the end...
	     */
	    list->priv_lastAccess = Tail;
	} else {
	    /*
	     * Reset to Middle if gone past first.
	     */
	    list->priv_lastAccess = Middle;
	}
    }

    return node;
}

/* Close a list which was opened for sequential access. */
void
Lst_Close(List *list)
{
    assert(list->priv_isOpen);

    list->priv_isOpen = FALSE;
    list->priv_lastAccess = Unknown;
}


/*
 * for using the list as a queue
 */

/* Add the datum to the tail of the given list. */
void
Lst_Enqueue(List *list, void *datum)
{
    Lst_Append(list, datum);
}

/* Remove and return the datum at the head of the given list. */
void *
Lst_Dequeue(List *list)
{
    void *datum = list->first->datum;
    Lst_Remove(list, list->first);
    assert(datum != NULL);	/* since NULL would mean end of the list */
    return datum;
}

void
Stack_Init(Stack *stack)
{
    stack->len = 0;
    stack->cap = 10;
    stack->items = bmake_malloc(stack->cap * sizeof stack->items[0]);
}

Boolean Stack_IsEmpty(Stack *stack)
{
    return stack->len == 0;
}

void Stack_Push(Stack *stack, void *datum)
{
    if (stack->len >= stack->cap) {
	stack->cap *= 2;
	stack->items = bmake_realloc(stack->items,
				     stack->cap * sizeof stack->items[0]);
    }
    stack->items[stack->len] = datum;
    stack->len++;
}

void *Stack_Pop(Stack *stack)
{
    void *datum;

    assert(stack->len > 0);
    stack->len--;
    datum = stack->items[stack->len];
#ifdef CLEANUP
    stack->items[stack->len] = NULL;
#endif
    return datum;
}

void Stack_Done(Stack *stack)
{
    free(stack->items);
}
