/*	$NetBSD: targ.c,v 1.112 2020/10/05 22:15:45 rillig Exp $	*/

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

/*
 * Copyright (c) 1989 by Berkeley Softworks
 * All rights reserved.
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
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the University of
 *	California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
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

/*-
 * targ.c --
 *	Functions for maintaining the Lst allTargets. Target nodes are
 *	kept in two structures: a Lst and a hash table.
 *
 * Interface:
 *	Targ_Init	Initialization procedure.
 *
 *	Targ_End	Cleanup the module
 *
 *	Targ_List	Return the list of all targets so far.
 *
 *	Targ_NewGN	Create a new GNode for the passed target
 *			(string). The node is *not* placed in the
 *			hash table, though all its fields are
 *			initialized.
 *
 *	Targ_FindNode	Find the node, or return NULL.
 *
 *	Targ_GetNode	Find the node, or create it.
 *
 *	Targ_NewInternalNode
 *			Create an internal node.
 *
 *	Targ_FindList	Given a list of names, find nodes for all
 *			of them, creating them as necessary.
 *
 *	Targ_Ignore	Return TRUE if errors should be ignored when
 *			creating the given target.
 *
 *	Targ_Silent	Return TRUE if we should be silent when
 *			creating the given target.
 *
 *	Targ_Precious	Return TRUE if the target is precious and
 *			should not be removed if we are interrupted.
 *
 *	Targ_Propagate	Propagate information between related nodes.
 *			Should be called after the makefiles are parsed
 *			but before any action is taken.
 *
 * Debugging:
 *	Targ_PrintGraph
 *			Print out the entire graphm all variables and
 *			statistics for the directory cache. Should print
 *			something for suffixes, too, but...
 */

#include <time.h>

#include "make.h"
#include "dir.h"

/*	"@(#)targ.c	8.2 (Berkeley) 3/19/94"	*/
MAKE_RCSID("$NetBSD: targ.c,v 1.112 2020/10/05 22:15:45 rillig Exp $");

static GNodeList *allTargets;	/* the list of all targets found so far */
#ifdef CLEANUP
static GNodeList *allGNs;	/* List of all the GNodes */
#endif
static Hash_Table targets;	/* a hash table of same */

#ifdef CLEANUP
static void TargFreeGN(void *);
#endif

void
Targ_Init(void)
{
    allTargets = Lst_Init();
    Hash_InitTable(&targets);
}

void
Targ_End(void)
{
    Targ_Stats();
#ifdef CLEANUP
    Lst_Free(allTargets);
    if (allGNs != NULL)
	Lst_Destroy(allGNs, TargFreeGN);
    Hash_DeleteTable(&targets);
#endif
}

void
Targ_Stats(void)
{
    Hash_DebugStats(&targets, "targets");
}

/* Return the list of all targets. */
GNodeList *
Targ_List(void)
{
    return allTargets;
}

/* Create and initialize a new graph node. The gnode is added to the list of
 * all gnodes.
 *
 * Input:
 *	name		the name of the node, such as "clean", "src.c", ".END"
 */
GNode *
Targ_NewGN(const char *name)
{
    GNode *gn;

    gn = bmake_malloc(sizeof(GNode));
    gn->name = bmake_strdup(name);
    gn->uname = NULL;
    gn->path = NULL;
    gn->type = name[0] == '-' && name[1] == 'l' ? OP_LIB : 0;
    gn->unmade = 0;
    gn->unmade_cohorts = 0;
    gn->cohort_num[0] = '\0';
    gn->centurion = NULL;
    gn->made = UNMADE;
    gn->flags = 0;
    gn->checked = 0;
    gn->mtime = 0;
    gn->cmgn = NULL;
    gn->implicitParents = Lst_Init();
    gn->cohorts = Lst_Init();
    gn->parents = Lst_Init();
    gn->children = Lst_Init();
    gn->order_pred = Lst_Init();
    gn->order_succ = Lst_Init();
    Hash_InitTable(&gn->context);
    gn->commands = Lst_Init();
    gn->suffix = NULL;
    gn->fname = NULL;
    gn->lineno = 0;

#ifdef CLEANUP
    if (allGNs == NULL)
	allGNs = Lst_Init();
    Lst_Append(allGNs, gn);
#endif

    return gn;
}

#ifdef CLEANUP
static void
TargFreeGN(void *gnp)
{
    GNode *gn = (GNode *)gnp;

    free(gn->name);
    free(gn->uname);
    free(gn->path);

    Lst_Free(gn->implicitParents);
    Lst_Free(gn->cohorts);
    Lst_Free(gn->parents);
    Lst_Free(gn->children);
    Lst_Free(gn->order_succ);
    Lst_Free(gn->order_pred);
    Hash_DeleteTable(&gn->context);
    Lst_Free(gn->commands);

    /* XXX: does gn->suffix need to be freed? It is reference-counted. */

    free(gn);
}
#endif

/* Get the existing global node, or return NULL. */
GNode *
Targ_FindNode(const char *name)
{
    return Hash_FindValue(&targets, name);
}

/* Get the existing global node, or create it. */
GNode *
Targ_GetNode(const char *name)
{
    Boolean isNew;
    Hash_Entry *he = Hash_CreateEntry(&targets, name, &isNew);
    if (!isNew)
	return Hash_GetValue(he);

    {
	GNode *gn = Targ_NewInternalNode(name);
	Hash_SetValue(he, gn);
	return gn;
    }
}

/* Create a node, register it in .ALLTARGETS but don't store it in the
 * table of global nodes.  This means it cannot be found by name.
 *
 * This is used for internal nodes, such as cohorts or .WAIT nodes. */
GNode *
Targ_NewInternalNode(const char *name)
{
    GNode *gn = Targ_NewGN(name);
    Var_Append(".ALLTARGETS", name, VAR_GLOBAL);
    Lst_Append(allTargets, gn);
    if (doing_depend)
	gn->flags |= FROM_DEPEND;
    return gn;
}

/* Return the .END node, which contains the commands to be executed when
 * everything else is done. */
GNode *Targ_GetEndNode(void)
{
    /* Save the node locally to avoid having to search for it all the time. */
    static GNode *endNode = NULL;
    if (endNode == NULL) {
	endNode = Targ_GetNode(".END");
	endNode->type = OP_SPECIAL;
    }
    return endNode;
}

/* Return the named nodes, creating them as necessary. */
GNodeList *
Targ_FindList(StringList *names)
{
    StringListNode *ln;
    GNodeList *nodes = Lst_Init();
    for (ln = names->first; ln != NULL; ln = ln->next) {
	const char *name = ln->datum;
	GNode *gn = Targ_GetNode(name);
	Lst_Append(nodes, gn);
    }
    return nodes;
}

/* Return true if should ignore errors when creating gn. */
Boolean
Targ_Ignore(GNode *gn)
{
    return ignoreErrors || gn->type & OP_IGNORE;
}

/* Return true if be silent when creating gn. */
Boolean
Targ_Silent(GNode *gn)
{
    return beSilent || gn->type & OP_SILENT;
}

/* See if the given target is precious. */
Boolean
Targ_Precious(GNode *gn)
{
    return allPrecious || gn->type & (OP_PRECIOUS | OP_DOUBLEDEP);
}

/******************* DEBUG INFO PRINTING ****************/

static GNode	  *mainTarg;	/* the main target, as set by Targ_SetMain */

/* Set our idea of the main target we'll be creating. Used for debugging
 * output. */
void
Targ_SetMain(GNode *gn)
{
    mainTarg = gn;
}

static void
PrintNodeNames(GNodeList *gnodes)
{
    GNodeListNode *node;

    for (node = gnodes->first; node != NULL; node = node->next) {
	GNode *gn = node->datum;
	debug_printf(" %s%s", gn->name, gn->cohort_num);
    }
}

static void
PrintNodeNamesLine(const char *label, GNodeList *gnodes)
{
    if (Lst_IsEmpty(gnodes))
	return;
    debug_printf("# %s:", label);
    PrintNodeNames(gnodes);
    debug_printf("\n");
}

void
Targ_PrintCmds(GNode *gn)
{
    StringListNode *node = gn->commands->first;
    for (; node != NULL; node = node->next) {
	const char *cmd = LstNode_Datum(node);
	debug_printf("\t%s\n", cmd);
    }
}

/* Format a modification time in some reasonable way and return it.
 * The time is placed in a static area, so it is overwritten with each call. */
char *
Targ_FmtTime(time_t tm)
{
    struct tm *parts;
    static char buf[128];

    parts = localtime(&tm);
    (void)strftime(buf, sizeof buf, "%k:%M:%S %b %d, %Y", parts);
    return buf;
}

/* Print out a type field giving only those attributes the user can set. */
void
Targ_PrintType(int type)
{
    int    tbit;

#define PRINTBIT(attr)	case CONCAT(OP_,attr): debug_printf(" ." #attr); break
#define PRINTDBIT(attr) case CONCAT(OP_,attr): if (DEBUG(TARG))debug_printf(" ." #attr); break

    type &= ~OP_OPMASK;

    while (type) {
	tbit = 1 << (ffs(type) - 1);
	type &= ~tbit;

	switch(tbit) {
	PRINTBIT(OPTIONAL);
	PRINTBIT(USE);
	PRINTBIT(EXEC);
	PRINTBIT(IGNORE);
	PRINTBIT(PRECIOUS);
	PRINTBIT(SILENT);
	PRINTBIT(MAKE);
	PRINTBIT(JOIN);
	PRINTBIT(INVISIBLE);
	PRINTBIT(NOTMAIN);
	PRINTDBIT(LIB);
	    /*XXX: MEMBER is defined, so CONCAT(OP_,MEMBER) gives OP_"%" */
	case OP_MEMBER: if (DEBUG(TARG))debug_printf(" .MEMBER"); break;
	PRINTDBIT(ARCHV);
	PRINTDBIT(MADE);
	PRINTDBIT(PHONY);
	}
    }
}

static const char *
made_name(GNodeMade made)
{
    switch (made) {
    case UNMADE:     return "unmade";
    case DEFERRED:   return "deferred";
    case REQUESTED:  return "requested";
    case BEINGMADE:  return "being made";
    case MADE:       return "made";
    case UPTODATE:   return "up-to-date";
    case ERROR:      return "error when made";
    case ABORTED:    return "aborted";
    default:         return "unknown enum_made value";
    }
}

/* Print the contents of a node. */
void
Targ_PrintNode(GNode *gn, int pass)
{
    debug_printf("# %s%s", gn->name, gn->cohort_num);
    GNode_FprintDetails(debug_file, ", ", gn, "\n");
    if (gn->flags == 0)
	return;

    if (!OP_NOP(gn->type)) {
	debug_printf("#\n");
	if (gn == mainTarg) {
	    debug_printf("# *** MAIN TARGET ***\n");
	}
	if (pass >= 2) {
	    if (gn->unmade) {
		debug_printf("# %d unmade children\n", gn->unmade);
	    } else {
		debug_printf("# No unmade children\n");
	    }
	    if (! (gn->type & (OP_JOIN|OP_USE|OP_USEBEFORE|OP_EXEC))) {
		if (gn->mtime != 0) {
		    debug_printf("# last modified %s: %s\n",
			    Targ_FmtTime(gn->mtime),
			    made_name(gn->made));
		} else if (gn->made != UNMADE) {
		    debug_printf("# non-existent (maybe): %s\n",
			    made_name(gn->made));
		} else {
		    debug_printf("# unmade\n");
		}
	    }
	    PrintNodeNamesLine("implicit parents", gn->implicitParents);
	} else {
	    if (gn->unmade)
		debug_printf("# %d unmade children\n", gn->unmade);
	}
	PrintNodeNamesLine("parents", gn->parents);
	PrintNodeNamesLine("order_pred", gn->order_pred);
	PrintNodeNamesLine("order_succ", gn->order_succ);

	debug_printf("%-16s", gn->name);
	switch (gn->type & OP_OPMASK) {
	case OP_DEPENDS:
	    debug_printf(":"); break;
	case OP_FORCE:
	    debug_printf("!"); break;
	case OP_DOUBLEDEP:
	    debug_printf("::"); break;
	}
	Targ_PrintType(gn->type);
	PrintNodeNames(gn->children);
	debug_printf("\n");
	Targ_PrintCmds(gn);
	debug_printf("\n\n");
	if (gn->type & OP_DOUBLEDEP) {
	    Targ_PrintNodes(gn->cohorts, pass);
	}
    }
}

void
Targ_PrintNodes(GNodeList *gnodes, int pass)
{
    GNodeListNode *ln;
    for (ln = gnodes->first; ln != NULL; ln = ln->next)
	Targ_PrintNode(ln->datum, pass);
}

/* Print only those targets that are just a source. */
static void
PrintOnlySources(void)
{
    GNodeListNode *ln;

    for (ln = allTargets->first; ln != NULL; ln = ln->next) {
	GNode *gn = ln->datum;
	if (!OP_NOP(gn->type))
	    continue;

	debug_printf("#\t%s [%s]",
		gn->name, gn->path ? gn->path : gn->name);
	Targ_PrintType(gn->type);
	debug_printf("\n");
    }
}

/* Input:
 *	pass		1 => before processing
 *			2 => after processing
 *			3 => after processing, an error occurred
 */
void
Targ_PrintGraph(int pass)
{
    debug_printf("#*** Input graph:\n");
    Targ_PrintNodes(allTargets, pass);
    debug_printf("\n\n");
    debug_printf("#\n#   Files that are only sources:\n");
    PrintOnlySources();
    debug_printf("#*** Global Variables:\n");
    Var_Dump(VAR_GLOBAL);
    debug_printf("#*** Command-line Variables:\n");
    Var_Dump(VAR_CMD);
    debug_printf("\n");
    Dir_PrintDirectories();
    debug_printf("\n");
    Suff_PrintAll();
}

/* Propagate some type information to cohort nodes (those from the ::
 * dependency operator).
 *
 * Should be called after the makefiles are parsed but before any action is
 * taken. */
void
Targ_Propagate(void)
{
    GNodeListNode *pn, *cn;

    for (pn = allTargets->first; pn != NULL; pn = pn->next) {
	GNode *pgn = pn->datum;

	if (!(pgn->type & OP_DOUBLEDEP))
	    continue;

	for (cn = pgn->cohorts->first; cn != NULL; cn = cn->next) {
	    GNode *cgn = cn->datum;

	    cgn->type |= pgn->type & ~OP_OPMASK;
	}
    }
}
