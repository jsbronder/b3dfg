/*
 * Brontes frame grabber example
 *
 * Copyright (C) 2008 3M Company
 * Author: Daniel Drake <ddrake@brontes3d.com>
 *
 * Private software. Source code may not be distributed.
 */

#include <stdio.h>
#include <unistd.h>

#include <libb3dfg/b3dfg.h>

#define NUM_BUFFERS 2

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
	int i;
	int r = 1;

	dev = b3dfg_open(0);
	if (!dev) {
		fprintf(stderr, "open failed\n");
		goto out;
	}

	frame_size = b3dfg_get_frame_size(dev);
	r = b3dfg_set_num_buffers(dev, NUM_BUFFERS);
	if (r) {
		fprintf(stderr, "set_num_buffers failed\n");
		goto out;
	}

	mapping = b3dfg_map_buffers(dev, 0);
	if (!mapping) {
		fprintf(stderr, "mapping failed\n");
		goto out;
	}

	for (i = 0; i < NUM_BUFFERS; i++) {
		r = b3dfg_queue_buffer(dev, i);
		if (r) {
			fprintf(stderr, "queue_buffer failed\n");
			goto out;
		}
	}

	r = b3dfg_set_transmission(dev, 1);
	if (r) {
		fprintf(stderr, "set_transmission failed\n");
		goto out;
	}

	for (i = 0; i < NUM_BUFFERS; i++) {
		r = b3dfg_wait_buffer(dev, i);
		if (r) {
			fprintf(stderr, "wait_buffer failed\n");
			goto out;
		}
		write_to_file(i);
	}

	r = 0;
out:
	if (dev)
		b3dfg_unmap_buffers(dev);
	b3dfg_close(dev);
	return r;
}
