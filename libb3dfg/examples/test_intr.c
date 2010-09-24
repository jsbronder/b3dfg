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
#include <examples/serial_cmd.h>

#define NUM_BUFFERS 3
#define RUNTIME 60

static b3dfg_dev *dev;
static unsigned int frame_size;

int main(void)
{
	int fd;
	int r = 1;
    int triggering = 0;

	dev = b3dfg_init(0);
	if (!dev) {
		fprintf(stderr, "open failed\n");
		goto out;
	}

	printf("wand is %s\n",
		b3dfg_get_wand_status(dev) ? "connected" : "disconnected");

	frame_size = b3dfg_get_frame_size(dev);

    if ( (fd = open_serial("/dev/ttyUSB0")) <= 0 ) {
        r = -1;
        goto out;
    }

    if ( (triggering = send_cmd(fd, START_TRIGGERS)) == -1){
        r = -1;
        goto out;
    }

	r = b3dfg_open(dev, 0);
	if (r) {
		fprintf(stderr, "open failed\n");
		goto out;
	}

    printf("Allowing triggering to run for %d seconds\n", RUNTIME);
    sleep(RUNTIME);

	r = 0;
out:
    if (triggering)
        send_cmd(fd, STOP_TRIGGERS);
	if (dev) {
	    b3dfg_close(dev);
		b3dfg_exit(dev);
    }
	return r;
}
