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
static unsigned char *mapping;
static unsigned int frame_size;

static void write_frame(int buffer, int frame)
{
	char filename[50];
	FILE *fd;
	int num = (buffer * 3) + frame;

	sprintf(filename, "cap%02d.pgm", num);
	fd = fopen(filename, "w");
	fprintf(fd, "P5 1024 768 255 ");
	fwrite(mapping + (buffer * frame_size * 3) + (frame_size * frame), 1,
		frame_size, fd);
	fclose(fd);
}

static void write_to_file(int buffer)
{
	write_frame(buffer, 0);
	write_frame(buffer, 1);
	write_frame(buffer, 2);
}

int main(void)
{
	int i, buf, fd;
	int r = 1;
    int triggering = 0;
    struct b3dfg_buffer_state state;

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

	for (i = 0; i < NUM_BUFFERS; i++) {
        while (true) {
		    r = b3dfg_get_buffer(dev, 1000, &state);
		    if (r < 0) {
			    fprintf(stderr, "get_buffer failed\n");
			    goto out;
		    } else if (state.buffer == i) {
                printf("Got buffer %d\n", state.buffer);
		        write_to_file(i);
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
	if (dev)
		b3dfg_unmap_buffers(dev);
	b3dfg_close(dev);
	return r;
}
