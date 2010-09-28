/*
 * Brontes frame grabber example
 *
 * Copyright (C) 2008 3M Company
 * Author: Daniel Drake <ddrake@brontes3d.com>
 *
 * Private software. Source code may not be distributed.
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
