/*	$NetBSD: make.h,v 1.156 2020/10/05 22:15:45 rillig Exp $	*/

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
 *
 *	from: @(#)make.h	8.3 (Berkeley) 6/13/95
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
 *
 *	from: @(#)make.h	8.3 (Berkeley) 6/13/95
 */

/*-
 * make.h --
 *	The global definitions for pmake
 */

#ifndef MAKE_MAKE_H
#define MAKE_MAKE_H

#include <sys/types.h>
#include <sys/param.h>
#include <sys/stat.h>

#include <assert.h>
#include <ctype.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef BSD4_4
# include <sys/cdefs.h>
#endif

#ifndef FD_CLOEXEC
#define FD_CLOEXEC 1
#endif

#if defined(__GNUC__)
#define	MAKE_GNUC_PREREQ(x, y)						\
	((__GNUC__ == (x) && __GNUC_MINOR__ >= (y)) ||			\
	 (__GNUC__ > (x)))
#else /* defined(__GNUC__) */
#define	MAKE_GNUC_PREREQ(x, y)	0
#endif /* defined(__GNUC__) */

#if MAKE_GNUC_PREREQ(2, 7)
#define	MAKE_ATTR_UNUSED	__attribute__((__unused__))
#else
#define	MAKE_ATTR_UNUSED	/* delete */
#endif

#if MAKE_GNUC_PREREQ(2, 5)
#define	MAKE_ATTR_DEAD		__attribute__((__noreturn__))
#elif defined(__GNUC__)
#define	MAKE_ATTR_DEAD		__volatile
#else
#define	MAKE_ATTR_DEAD		/* delete */
#endif

#if MAKE_GNUC_PREREQ(2, 7)
#define MAKE_ATTR_PRINTFLIKE(fmtarg, firstvararg)	\
	    __attribute__((__format__ (__printf__, fmtarg, firstvararg)))
#else
#define MAKE_ATTR_PRINTFLIKE(fmtarg, firstvararg)	/* delete */
#endif

/*
 * A boolean type is defined as an integer, not an enum, for historic reasons.
 * The only allowed values are the constants TRUE and FALSE (1 and 0).
 */

#ifdef USE_DOUBLE_BOOLEAN
/* During development, to find type mismatches in function declarations. */
typedef double Boolean;
#elif defined(USE_UCHAR_BOOLEAN)
/* During development, to find code that depends on the exact value of TRUE or
 * that stores other values in Boolean variables. */
typedef unsigned char Boolean;
#define TRUE ((unsigned char)0xFF)
#define FALSE ((unsigned char)0x00)
#elif defined(USE_ENUM_BOOLEAN)
typedef enum { FALSE, TRUE } Boolean;
#else
typedef int Boolean;
#endif
#ifndef TRUE
#define TRUE	1
#endif
#ifndef FALSE
#define FALSE	0
#endif

#include "lst.h"
#include "enum.h"
#include "hash.h"
#include "config.h"
#include "buf.h"
#include "make_malloc.h"

typedef enum  {
    UNMADE,			/* Not examined yet */
    DEFERRED,			/* Examined once (building child) */
    REQUESTED,			/* on toBeMade list */
    BEINGMADE,			/* Target is already being made.
				 * Indicates a cycle in the graph. */
    MADE,			/* Was out-of-date and has been made */
    UPTODATE,			/* Was already up-to-date */
    ERROR,			/* An error occurred while it was being
				 * made (used only in compat mode) */
    ABORTED			/* The target was aborted due to an error
				 * making an inferior (compat). */
} GNodeMade;

/* The OP_ constants are used when parsing a dependency line as a way of
 * communicating to other parts of the program the way in which a target
 * should be made.
 *
 * These constants are bitwise-OR'ed together and placed in the 'type' field
 * of each node. Any node that has a 'type' field which satisfies the OP_NOP
 * function was never never on the left-hand side of an operator, though it
 * may have been on the right-hand side... */
typedef enum {
    /* Execution of commands depends on children (:) */
    OP_DEPENDS		= 1 << 0,
    /* Always execute commands (!) */
    OP_FORCE		= 1 << 1,
    /* Execution of commands depends on children per line (::) */
    OP_DOUBLEDEP	= 1 << 2,

    OP_OPMASK		= OP_DEPENDS|OP_FORCE|OP_DOUBLEDEP,

    /* Don't care if the target doesn't exist and can't be created */
    OP_OPTIONAL		= 1 << 3,
    /* Use associated commands for parents */
    OP_USE		= 1 << 4,
    /* Target is never out of date, but always execute commands anyway.
     * Its time doesn't matter, so it has none...sort of */
    OP_EXEC		= 1 << 5,
    /* Ignore errors when creating the node */
    OP_IGNORE		= 1 << 6,
    /* Don't remove the target when interrupted */
    OP_PRECIOUS		= 1 << 7,
    /* Don't echo commands when executed */
    OP_SILENT		= 1 << 8,
    /* Target is a recursive make so its commands should always be executed
     * when it is out of date, regardless of the state of the -n or -t flags */
    OP_MAKE		= 1 << 9,
    /* Target is out-of-date only if any of its children was out-of-date */
    OP_JOIN		= 1 << 10,
    /* Assume the children of the node have been already made */
    OP_MADE		= 1 << 11,
    /* Special .BEGIN, .END, .INTERRUPT */
    OP_SPECIAL		= 1 << 12,
    /* Like .USE, only prepend commands */
    OP_USEBEFORE	= 1 << 13,
    /* The node is invisible to its parents. I.e. it doesn't show up in the
     * parents' local variables. */
    OP_INVISIBLE	= 1 << 14,
    /* The node is exempt from normal 'main target' processing in parse.c */
    OP_NOTMAIN		= 1 << 15,
    /* Not a file target; run always */
    OP_PHONY		= 1 << 16,
    /* Don't search for file in the path */
    OP_NOPATH		= 1 << 17,
    /* .WAIT phony node */
    OP_WAIT		= 1 << 18,
    /* .NOMETA do not create a .meta file */
    OP_NOMETA		= 1 << 19,
    /* .META we _do_ want a .meta file */
    OP_META		= 1 << 20,
    /* Do not compare commands in .meta file */
    OP_NOMETA_CMP	= 1 << 21,
    /* Possibly a submake node */
    OP_SUBMAKE		= 1 << 22,

    /* Attributes applied by PMake */

    /* The node is a transformation rule */
    OP_TRANSFORM	= 1 << 31,
    /* Target is a member of an archive */
    OP_MEMBER		= 1 << 30,
    /* Target is a library */
    OP_LIB		= 1 << 29,
    /* Target is an archive construct */
    OP_ARCHV		= 1 << 28,
    /* Target has all the commands it should. Used when parsing to catch
     * multiple commands for a target. */
    OP_HAS_COMMANDS	= 1 << 27,
    /* The special command "..." has been seen. All further commands from
     * this node will be saved on the .END node instead, to be executed at
     * the very end. */
    OP_SAVE_CMDS	= 1 << 26,
    /* Already processed by Suff_FindDeps */
    OP_DEPS_FOUND	= 1 << 25,
    /* Node found while expanding .ALLSRC */
    OP_MARK		= 1 << 24,

    OP_NOTARGET		= OP_NOTMAIN | OP_USE | OP_EXEC | OP_TRANSFORM
} GNodeType;

typedef enum {
    REMAKE	= 0x0001,	/* this target needs to be (re)made */
    CHILDMADE	= 0x0002,	/* children of this target were made */
    FORCE	= 0x0004,	/* children don't exist, and we pretend made */
    DONE_WAIT	= 0x0008,	/* Set by Make_ProcessWait() */
    DONE_ORDER	= 0x0010,	/* Build requested by .ORDER processing */
    FROM_DEPEND	= 0x0020,	/* Node created from .depend */
    DONE_ALLSRC	= 0x0040,	/* We do it once only */
    CYCLE	= 0x1000,	/* Used by MakePrintStatus */
    DONECYCLE	= 0x2000,	/* Used by MakePrintStatus */
    INTERNAL	= 0x4000	/* Internal use only */
} GNodeFlags;

typedef struct List StringList;
typedef struct ListNode StringListNode;

typedef struct List GNodeList;
typedef struct ListNode GNodeListNode;

typedef struct List /* of CachedDir */ SearchPath;

/* A graph node represents a target that can possibly be made, including its
 * relation to other targets and a lot of other details. */
typedef struct GNode {
    /* The target's name, such as "clean" or "make.c" */
    char *name;
    /* The unexpanded name of a .USE node */
    char *uname;
    /* The full pathname of the file belonging to the target.
     * XXX: What about .PHONY targets? These don't have an associated path. */
    char *path;

    /* The type of operator used to define the sources (see the OP flags below).
     * XXX: This looks like a wild mixture of type and flags. */
    GNodeType type;
    /* whether it is involved in this invocation of make */
    GNodeFlags flags;

    /* The state of processing on this node */
    GNodeMade made;
    int unmade;			/* The number of unmade children */

    time_t mtime;		/* Its modification time */
    struct GNode *cmgn;		/* The youngest child */

    /* The GNodes for which this node is an implied source. May be empty.
     * For example, when there is an inference rule for .c.o, the node for
     * file.c has the node for file.o in this list. */
    GNodeList *implicitParents;

    /* Other nodes of the same name for the :: operator. */
    GNodeList *cohorts;

    /* The nodes that depend on this one, or in other words, the nodes for
     * which this is a source. */
    GNodeList *parents;
    /* The nodes on which this one depends. */
    GNodeList *children;

    /* .ORDER nodes we need made. The nodes that must be made (if they're
     * made) before this node can be made, but that do not enter into the
     * datedness of this node. */
    GNodeList *order_pred;
    /* .ORDER nodes who need us. The nodes that must be made (if they're made
     * at all) after this node is made, but that do not depend on this node,
     * in the normal sense. */
    GNodeList *order_succ;

    /* The "#n" suffix for this cohort, or "" for other nodes */
    char cohort_num[8];
    /* The number of unmade instances on the cohorts list */
    int unmade_cohorts;
    /* Pointer to the first instance of a '::' node; only set when on a
     * cohorts list */
    struct GNode *centurion;

    /* Last time (sequence number) we tried to make this node */
    unsigned int checked;

    /* The "local" variables that are specific to this target and this target
     * only, such as $@, $<, $?. */
    Hash_Table context;

    /* The commands to be given to a shell to create this target. */
    StringList *commands;

    /* Suffix for the node (determined by Suff_FindDeps and opaque to everyone
     * but the Suff module) */
    struct Suff *suffix;

    /* filename where the GNode got defined */
    const char *fname;
    /* line number where the GNode got defined */
    int lineno;
} GNode;

/*
 * Error levels for parsing. PARSE_FATAL means the process cannot continue
 * once the makefile has been parsed. PARSE_WARNING means it can. Passed
 * as the first argument to Parse_Error.
 */
#define PARSE_INFO	3
#define PARSE_WARNING	2
#define PARSE_FATAL	1

/*
 * Values returned by Cond_EvalLine and Cond_EvalCondition.
 */
typedef enum {
    COND_PARSE,			/* Parse the next lines */
    COND_SKIP,			/* Skip the next lines */
    COND_INVALID		/* Not a conditional statement */
} CondEvalResult;

/*
 * Definitions for the "local" variables. Used only for clarity.
 */
#define TARGET		"@"	/* Target of dependency */
#define OODATE		"?"	/* All out-of-date sources */
#define ALLSRC		">"	/* All sources */
#define IMPSRC		"<"	/* Source implied by transformation */
#define PREFIX		"*"	/* Common prefix */
#define ARCHIVE		"!"	/* Archive in "archive(member)" syntax */
#define MEMBER		"%"	/* Member in "archive(member)" syntax */

#define FTARGET		"@F"	/* file part of TARGET */
#define DTARGET		"@D"	/* directory part of TARGET */
#define FIMPSRC		"<F"	/* file part of IMPSRC */
#define DIMPSRC		"<D"	/* directory part of IMPSRC */
#define FPREFIX		"*F"	/* file part of PREFIX */
#define DPREFIX		"*D"	/* directory part of PREFIX */

/*
 * Global Variables
 */
extern StringList *create;	/* The list of target names specified on the
				 * command line. used to resolve #if
				 * make(...) statements */
extern SearchPath *dirSearchPath;
				/* The list of directories to search when
				 * looking for targets */

extern Boolean	compatMake;	/* True if we are make compatible */
extern Boolean	ignoreErrors;	/* True if should ignore all errors */
extern Boolean  beSilent;	/* True if should print no commands */
extern Boolean  noExecute;	/* True if should execute nothing */
extern Boolean  noRecursiveExecute;
				/* True if should execute nothing */
extern Boolean  allPrecious;	/* True if every target is precious */
extern Boolean  deleteOnError;	/* True if failed targets should be deleted */
extern Boolean  keepgoing;	/* True if should continue on unaffected
				 * portions of the graph when have an error
				 * in one portion */
extern Boolean	touchFlag;	/* TRUE if targets should just be 'touched'
				 * if out of date. Set by the -t flag */
extern Boolean	queryFlag;	/* TRUE if we aren't supposed to really make
				 * anything, just see if the targets are out-
				 * of-date */
extern Boolean	doing_depend;	/* TRUE if processing .depend */

extern Boolean	checkEnvFirst;	/* TRUE if environment should be searched for
				 * variables before the global context */

extern Boolean	parseWarnFatal;	/* TRUE if makefile parsing warnings are
				 * treated as errors */

extern Boolean	varNoExportEnv;	/* TRUE if we should not export variables
				 * set on the command line to the env. */

extern GNode    *DEFAULT;	/* .DEFAULT rule */

extern GNode	*VAR_INTERNAL;	/* Variables defined internally by make
				 * which should not override those set by
				 * makefiles.
				 */
extern GNode    *VAR_GLOBAL;	/* Variables defined in a global context, e.g
				 * in the Makefile itself */
extern GNode    *VAR_CMD;	/* Variables defined on the command line */
extern char	var_Error[];	/* Value returned by Var_Parse when an error
				 * is encountered. It actually points to
				 * an empty string, so naive callers needn't
				 * worry about it. */

extern time_t	now;		/* The time at the start of this whole
				 * process */

extern Boolean	oldVars;	/* Do old-style variable substitution */

extern SearchPath *sysIncPath;	/* The system include path. */
extern SearchPath *defIncPath;	/* The default include path. */

extern char	curdir[];	/* Startup directory */
extern char	*progname;	/* The program name */
extern char	*makeDependfile; /* .depend */
extern char	**savedEnv;	 /* if we replaced environ this will be non-NULL */

extern int	makelevel;

/*
 * We cannot vfork() in a child of vfork().
 * Most systems do not enforce this but some do.
 */
#define vFork() ((getpid() == myPid) ? vfork() : fork())
extern pid_t	myPid;

#define	MAKEFLAGS	".MAKEFLAGS"
#define	MAKEOVERRIDES	".MAKEOVERRIDES"
#define	MAKE_JOB_PREFIX	".MAKE.JOB.PREFIX" /* prefix for job target output */
#define	MAKE_EXPORTED	".MAKE.EXPORTED"   /* variables we export */
#define	MAKE_MAKEFILES	".MAKE.MAKEFILES"  /* all the makefiles we read */
#define	MAKE_LEVEL	".MAKE.LEVEL"	   /* recursion level */
#define MAKEFILE_PREFERENCE ".MAKE.MAKEFILE_PREFERENCE"
#define MAKE_DEPENDFILE	".MAKE.DEPENDFILE" /* .depend */
#define MAKE_MODE	".MAKE.MODE"
#ifndef MAKE_LEVEL_ENV
# define MAKE_LEVEL_ENV	"MAKELEVEL"
#endif

/*
 * debug control:
 *	There is one bit per module.  It is up to the module what debug
 *	information to print.
 */
extern FILE *debug_file;	/* Output is written here - default stderr */
extern int debug;
#define	DEBUG_ARCH	0x00001
#define	DEBUG_COND	0x00002
#define	DEBUG_DIR	0x00004
#define	DEBUG_GRAPH1	0x00008
#define	DEBUG_GRAPH2	0x00010
#define	DEBUG_JOB	0x00020
#define	DEBUG_MAKE	0x00040
#define	DEBUG_SUFF	0x00080
#define	DEBUG_TARG	0x00100
#define	DEBUG_VAR	0x00200
#define DEBUG_FOR	0x00400
#define DEBUG_SHELL	0x00800
#define DEBUG_ERROR	0x01000
#define DEBUG_LOUD	0x02000
#define DEBUG_META	0x04000
#define DEBUG_HASH	0x08000

#define DEBUG_GRAPH3	0x10000
#define DEBUG_SCRIPT	0x20000
#define DEBUG_PARSE	0x40000
#define DEBUG_CWD	0x80000

#define DEBUG_LINT	0x100000

#define CONCAT(a,b)	a##b

#define	DEBUG(module)	(debug & CONCAT(DEBUG_,module))

void debug_printf(const char *, ...) MAKE_ATTR_PRINTFLIKE(1, 2);

#define DEBUG0(module, text) \
    if (!DEBUG(module)) (void)0; \
    else debug_printf("%s", text)

#define DEBUG1(module, fmt, arg1) \
    if (!DEBUG(module)) (void)0; \
    else debug_printf(fmt, arg1)

#define DEBUG2(module, fmt, arg1, arg2) \
    if (!DEBUG(module)) (void)0; \
    else debug_printf(fmt, arg1, arg2)

#define DEBUG3(module, fmt, arg1, arg2, arg3) \
    if (!DEBUG(module)) (void)0; \
    else debug_printf(fmt, arg1, arg2, arg3)

#define DEBUG4(module, fmt, arg1, arg2, arg3, arg4) \
    if (!DEBUG(module)) (void)0; \
    else debug_printf(fmt, arg1, arg2, arg3, arg4)

#define DEBUG5(module, fmt, arg1, arg2, arg3, arg4, arg5) \
    if (!DEBUG(module)) (void)0; \
    else debug_printf(fmt, arg1, arg2, arg3, arg4, arg5)

#include "nonints.h"

void Make_TimeStamp(GNode *, GNode *);
Boolean Make_OODate(GNode *);
void Make_ExpandUse(GNodeList *);
time_t Make_Recheck(GNode *);
void Make_HandleUse(GNode *, GNode *);
void Make_Update(GNode *);
void Make_DoAllVar(GNode *);
Boolean Make_Run(GNodeList *);
int dieQuietly(GNode *, int);
void PrintOnError(GNode *, const char *);
void Main_ExportMAKEFLAGS(Boolean);
Boolean Main_SetObjdir(const char *, ...) MAKE_ATTR_PRINTFLIKE(1, 2);
int mkTempFile(const char *, char **);
int str2Lst_Append(StringList *, char *, const char *);
void GNode_FprintDetails(FILE *, const char *, const GNode *, const char *);

static Boolean MAKE_ATTR_UNUSED
NoExecute(GNode *gn)
{
    return (gn->type & OP_MAKE) ? noRecursiveExecute : noExecute;
}

/*
 * See if the node with the given type was not the object of a dependency
 * operator.
 */
static Boolean MAKE_ATTR_UNUSED
OP_NOP(GNodeType t)
{
    return !(t & OP_OPMASK);
}

#ifdef __GNUC__
#define UNCONST(ptr)	({		\
    union __unconst {			\
	const void *__cp;		\
	void *__p;			\
    } __d;				\
    __d.__cp = ptr, __d.__p; })
#else
#define UNCONST(ptr)	(void *)(ptr)
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/* At least GNU/Hurd systems lack hardcoded MAXPATHLEN/PATH_MAX */
#include <limits.h>
#ifndef MAXPATHLEN
#define MAXPATHLEN	4096
#endif
#ifndef PATH_MAX
#define PATH_MAX	MAXPATHLEN
#endif

#if defined(SYSV)
#define KILLPG(pid, sig)	kill(-(pid), (sig))
#else
#define KILLPG(pid, sig)	killpg((pid), (sig))
#endif

static inline MAKE_ATTR_UNUSED Boolean ch_isalnum(char ch)
{ return isalnum((unsigned char)ch) != 0; }
static inline MAKE_ATTR_UNUSED Boolean ch_isalpha(char ch)
{ return isalpha((unsigned char)ch) != 0; }
static inline MAKE_ATTR_UNUSED Boolean ch_isdigit(char ch)
{ return isdigit((unsigned char)ch) != 0; }
static inline MAKE_ATTR_UNUSED Boolean ch_isspace(char ch)
{ return isspace((unsigned char)ch) != 0; }
static inline MAKE_ATTR_UNUSED Boolean ch_isupper(char ch)
{ return isupper((unsigned char)ch) != 0; }
static inline MAKE_ATTR_UNUSED char ch_tolower(char ch)
{ return (char)tolower((unsigned char)ch); }
static inline MAKE_ATTR_UNUSED char ch_toupper(char ch)
{ return (char)toupper((unsigned char)ch); }

static inline MAKE_ATTR_UNUSED void
cpp_skip_whitespace(const char **pp)
{
    while (ch_isspace(**pp))
	(*pp)++;
}

static inline MAKE_ATTR_UNUSED void
pp_skip_whitespace(char **pp)
{
    while (ch_isspace(**pp))
	(*pp)++;
}

#ifndef MAKE_NATIVE
#define MAKE_RCSID(id) static volatile char rcsid[] = id
#else
#include <sys/cdefs.h>
#ifndef lint
#define MAKE_RCSID(id) __RCSID(id)
#endif
#endif

#endif /* MAKE_MAKE_H */
