/*
 * Brontes frame grabber example
 *
 * Copyright (c) 2009, 3M
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the 3M nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 *
 * Authors:
 * 	Justin Bronder <jsbronder@brontes3d.com>
 * 	Daniel Drake <ddrake@brontes3d.com>
 *
 */


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <poll.h>

#include <libb3dfg/b3dfg.h>

#define NRUNS 5

int main(void)
{
    static b3dfg_dev *dev;
    int r = 0;
    int i, fd;
    struct pollfd pfd;

	dev = b3dfg_init(0);
	if (!dev) {
		fprintf(stderr, "open failed\n");
        r = 1;
		goto out;
	}

	printf("wand is %s\n",
		b3dfg_get_wand_status(dev) ? "connected" : "disconnected");

    printf("Beginning test, change cable state %d times\n", NRUNS);
    
    for (i = 0; i < NRUNS; i++) {
        fd = -1;
        fd = b3dfg_get_wand_status_fd(dev);
        printf("fd is %d\n", fd);
        if (fd < 0) {
            fprintf(stderr, "Error b3dfg_get_wand_status_fd returned %d\n", fd);
            break;
        }
        pfd.fd = fd;
        pfd.events = POLLERR|POLLPRI;
        r = poll(&pfd, 1, -1);
        if ( r <= 0 ){
            fprintf(stderr, "Error poll returned %d\n", r);
            break;
        }
    	printf("wand is %s\n",
		    b3dfg_get_wand_status(dev) ? "connected" : "disconnected");
    }

    if (fd > 0)
        close(fd);

	b3dfg_exit(dev);
out:
	return r;
}
