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

static b3dfg_dev *dev;
static unsigned int frame_size;
static int frame_number = 0;

static void write_frame(char *buffer_start, int frame)
{
	char filename[50];
	FILE *fd;
    int r;
    
    frame_number++;
	sprintf(filename, "cap%02d.pgm", frame_number);
	fd = fopen(filename, "w");
	fprintf(fd, "P5 1024 768 255 ");
	r = fwrite(buffer_start + (frame_size * frame), 1,
		frame_size, fd);
    if (r < frame_size)
        fprintf(stderr, "Failed to write frame %d.\n", frame);
	fclose(fd);
}

static void write_to_file(char *addr)
{
	write_frame(addr, 0);
	write_frame(addr, 1);
	write_frame(addr, 2);
}

int main(void)
{
	int i, fd;
	int r = 1;
    int triggering = 0;
    struct b3dfg_buffer_state state;

	dev = b3dfg_init(0);
	if (!dev) {
		fprintf(stderr, "init failed\n");
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

	for (i = 0; i < NUM_BUFFERS; i++) {
        while (true) {
		    r = b3dfg_get_buffer(dev, 1000, &state);
		    if (r < 0) {
			    fprintf(stderr, "get_buffer failed\n");
			    goto out;
		    }
		    write_to_file(state.addr);
            if (state.buffer == i) {
                printf("Got buffer %d\n", state.buffer);
                b3dfg_release_buffer(dev);
                break;
            }
            b3dfg_release_buffer(dev);
            usleep( rand() % 4000 );
        }
	}

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
