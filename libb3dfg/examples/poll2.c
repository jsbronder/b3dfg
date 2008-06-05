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
#include <poll.h>

#include <libb3dfg/b3dfg.h>

#define NUM_BUFFERS 4

static b3dfg_dev *dev;
static unsigned char *mapping;
static unsigned int frame_size;

int main(void)
{
	int i;
	int r = 1;
	int next_buf = 0;
	unsigned int dropped;

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

	while (1) {
		r = b3dfg_wait_buffer(dev, next_buf, 1000, &dropped);
		if (r < 0) {
			fprintf(stderr, "poll failed error %d\n", r);
			break;
		}
		if (dropped > 0)
			printf("%d frame(s) dropped\n", dropped);

		r = b3dfg_queue_buffer(dev, next_buf);
		if (r < 0) {
			fprintf(stderr, "buffer requeue failed %d\n", r);
			break;
		}
		next_buf++;
		if (next_buf >= NUM_BUFFERS)
			next_buf = 0;
	}

	r = 0;
out:
	b3dfg_set_transmission(dev, 0);
	b3dfg_unmap_buffers(dev);
	b3dfg_close(dev);
	return r;
}
