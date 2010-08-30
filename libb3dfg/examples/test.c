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

#define GET_N_BUFFERS 1000
#define NUM_BUFFERS 3

static b3dfg_dev *dev;
static unsigned char *mapping;
static unsigned int frame_size;

int main(void)
{
	int i, fd;
    int buf = -1;
	int r = 1;
    int triggering = 0;
    int buffer_counts[NUM_BUFFERS] = {0, 0, 0};

	dev = b3dfg_open(0);
	if (!dev) {
		fprintf(stderr, "open failed\n");
		goto out;
	}

	printf("wand is %s\n",
		b3dfg_get_wand_status(dev) ? "connected" : "disconnected");

	frame_size = b3dfg_get_frame_size(dev);

	mapping = b3dfg_map_buffers(dev, 0);
	if (!mapping) {
		fprintf(stderr, "mapping failed\n");
		goto out;
	}

    if ( (fd = open_serial("/dev/ttyUSB0")) <= 0 ) {
        r = -1;
        goto out;
    }

    if ( (triggering = send_cmd(fd, START_TRIGGERS)) == -1){
        r = -1;
        goto out;
    }

	r = b3dfg_set_transmission(dev, 1);
	if (r) {
		fprintf(stderr, "set_transmission failed\n");
		goto out;
	}

    printf("Running until we acquire %d buffers\n", GET_N_BUFFERS);

	for (i = 0; i < GET_N_BUFFERS; i++) {
        r = b3dfg_get_buffer(dev, &buf, 1000, NULL, NULL);
        if (r < 0) {
            fprintf(stderr, "get_buffer failed\n");
            goto out;
        }
        buffer_counts[buf]++;
        usleep( rand() % 100000 );
        if ( !(i % 10) ){
            printf ("*");
            fflush(stdout);
        }
        b3dfg_release_buffer(dev, buf);
        buf = -1;
	}
    printf("\n");

	r = 0;
out:
    if (triggering)
        send_cmd(fd, STOP_TRIGGERS);
	if (dev) {
		b3dfg_unmap_buffers(dev);
        if (buf != -1){
            b3dfg_release_buffer(dev, buf);
        }
    }
	b3dfg_close(dev);

    if (i != GET_N_BUFFERS) {
        fprintf(stderr, "Only acquired %d of %d requested buffers\n", i, GET_N_BUFFERS);
        r = -1;
    } else {
        printf("\n");
        for (i = 0; i < NUM_BUFFERS; i++){
            printf("Buffer %d used %d times.\n", i, buffer_counts[i]);
        }
    }
	return r;
}
