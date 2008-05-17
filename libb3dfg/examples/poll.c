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

static int aborted = 0;

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
	struct pollfd pfd = { .events = POLLIN };
	int next_buf = 0;
	int nodata = 0;
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

	pfd.fd = b3dfg_get_fd(dev);
	while (!aborted) {
		r = poll(&pfd, 1, 1000);
		if (r < 0) {
			fprintf(stderr, "poll returned %d\n", r);
			break;
		}
		if (!(pfd.revents & POLLIN)) {
			printf("poll returned with no data?\n");
			if (++nodata == 3) {
				printf("aborting\n");
				break;
			}
			continue;
		}
		r = b3dfg_poll_buffer(dev, next_buf, &dropped);
		if (r < 0) {
			fprintf(stderr, "poll failed error %d\n", r);
			break;
		}
		if (r == 0) {
			fprintf(stderr, "buffer %d not ready after poll()?\n", next_buf);
			break;
		}
		if (dropped > 0) {
			printf("%d frame(s) dropped\n", dropped);
		}
		write_to_file(next_buf);
		r = b3dfg_queue_buffer(dev, next_buf);
		if (r < 0) {
			fprintf(stderr, "buffer requeue failed %d\n", r);
			break;
		}
		nodata = 0;
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
