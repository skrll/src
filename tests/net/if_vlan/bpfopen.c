/*	$NetBSD: bpfopen.c,v 1.1 2021/07/09 05:54:11 yamaguchi Exp $	*/

/*
 * Copyright (c) 2021 Internet Initiative Japan Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__RCSID("$NetBSD: bpfopen.c,v 1.1 2021/07/09 05:54:11 yamaguchi Exp $");

#include <sys/param.h>
#include <sys/ioctl.h>

#include <net/if.h>
#include <net/bpf.h>

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

enum {
	ARG_PROG = 0,
	ARG_HOST,
	ARG_IFNAME,
	ARG_NUM
};

enum {
	PFD_BPF = 0,
	PFD_NUM
};

static void	sighandler(int);

static sig_atomic_t quit;

static void
usage(void)
{

	fprintf(stderr, "%s {-r|-h} <ifname>\n"
	    "\t-r: rump_server\n"
	    "\t-h: host\n",
	    getprogname());
	exit(1);
}

int
main(int argc, char *argv[])
{
	struct ifreq ifr;
	struct pollfd pfd[PFD_NUM];
	const char *bpf_path;
	int n, bpfd, nfds;
	size_t bufsiz;
	char *buf;

	if (argc != ARG_NUM)
		usage();

	if (strcmp(argv[ARG_HOST], "-h") == 0) {
		bpf_path = "/dev/bpf";
	} else if (strcmp(argv[ARG_HOST], "-r") == 0){
		bpf_path = "/rump/dev/bpf";
	} else {
		errx(1, "-r or -h");
	}

	bpfd = open(bpf_path, O_RDONLY);
	if (bpfd < 0)
		err(1, "open %s", bpf_path);

	memset(&ifr, 0, sizeof(ifr));
	strlcpy(ifr.ifr_name, argv[ARG_IFNAME],
	    sizeof(ifr.ifr_name));
	if (ioctl(bpfd, BIOCSETIF, &ifr) != 0)
		err(1, "BIOCSETIF");
	if (ioctl(bpfd, BIOCPROMISC, NULL) != 0)
		err(1, "BIOCPROMISC");
	if (ioctl(bpfd, BIOCGBLEN, &bufsiz) != 0)
		err(1, "BIOCGBLEN");
	bufsiz = MIN(bufsiz, BPF_DFLTBUFSIZE * 4);

	buf = malloc(bufsiz);
	if (buf == NULL)
		err(1, "malloc");

	quit = 0;
	signal(SIGTERM, sighandler);
	signal(SIGQUIT, sighandler);
	signal(SIGINT, sighandler);
	signal(SIGHUP, sighandler);

	fprintf(stderr, "bpf open %s\n", ifr.ifr_name);
	while (quit == 0) {
		pfd[PFD_BPF].fd = bpfd;
		pfd[PFD_BPF].events = POLLIN;

		nfds = poll(pfd, PFD_NUM, 1 * 1000);
		if (nfds == -1 && errno != EINTR) {
			warn("poll");
			quit = 1;
		}

		if (nfds > 0 && (pfd[PFD_BPF].revents & POLLIN)) {
			/* read & drop */
			memset(buf, 0, sizeof(bufsiz));
			n = read(pfd[PFD_BPF].fd, buf, bufsiz);
			if (n < 0)
				quit = 1;
		}
	}

	close(bpfd);
	free(buf);
	fprintf(stderr, "closed\n");

	return 0;
}

static void
sighandler(int signo)
{

	quit = 1;
}
