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
static unsigned int frame_size;

static void write_buffer(char *buffer_start)
{
	char filename[50];
	FILE *fd;
    int i;
   
    for (i=0; i<3; i++) {
	    sprintf(filename, "cap%02d.pgm", i);
	    fd = fopen(filename, "w");
	    fprintf(fd, "P5 1024 768 255 ");
	    if ( fwrite(buffer_start + (frame_size * i), 1, frame_size, fd) <= 0 ){
            fprintf(stderr, "fwrite: %s failed\n", filename);
        }
	    fclose(fd);
    }

}

int main(void)
{
	int i = 0, fd;
    int buf = -1;
	int r = 1;
    int triggering = 0;
    int buffer_counts[NUM_BUFFERS] = {0, 0, 0};
    unsigned char *mapping;
    b3dfg_buffer_state state;

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
        r = b3dfg_get_buffer(dev, 1000, &state);
        if (r < 0) {
            fprintf(stderr, "get_buffer failed\n");
            goto out;
        }
        buffer_counts[state.buffer]++;
        write_buffer(state.addr);
        usleep( rand() % 100000 );
        if ( !(i % 10) ){
            printf ("*");
            fflush(stdout);
        }
        b3dfg_release_buffer(dev);
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
            b3dfg_release_buffer(dev);
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
