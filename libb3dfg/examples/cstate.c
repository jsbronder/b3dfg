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

#include <libb3dfg/b3dfg.h>


int main(void)
{
    static b3dfg_dev *dev;
    int r = 0;

	dev = b3dfg_init(0);
	if (!dev) {
		fprintf(stderr, "open failed\n");
        r = 1;
		goto out;
	}

	printf("wand is %s\n",
		b3dfg_get_wand_status(dev) ? "connected" : "disconnected");

	b3dfg_exit(dev);
out:
	return r;
}
