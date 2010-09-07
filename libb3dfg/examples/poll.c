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
#include <examples/serial_cmd.h>

#define NUM_BUFFERS 4

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
	int i, buffer;
	int r = 1;
    int fd = -1;
	struct pollfd pfd = { .events = POLLIN };
	unsigned int dropped;
    int triggering = 0;

	dev = b3dfg_open(0);
	if (!dev) {
		fprintf(stderr, "open failed\n");
		goto out;
	}

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

	pfd.fd = b3dfg_get_fd(dev);
    for (i = 0; i < 3; i++ ) {
		r = poll(&pfd, 1, 1000);
		if (r < 0) {
			fprintf(stderr, "poll returned %d\n", r);
			break;
		}
		if (!(pfd.revents & POLLIN)) {
			printf("poll returned with no data?\n");
            r = -1;
            goto out;
		}

		r = b3dfg_get_buffer(dev, &buffer, 1000, &dropped, NULL);
		if (r < 0) {
			fprintf(stderr, "poll failed error %d\n", r);
			break;
		}
		if (r == 0) {
			fprintf(stderr, "buffer not ready after poll()?\n");
			break;
		}
		if (dropped > 0) {
			printf("%d frame(s) dropped\n", dropped);
		}
		write_to_file(buffer);
        b3dfg_release_buffer(dev);
	}

	r = 0;
out:
    if (triggering)
        send_cmd(fd, STOP_TRIGGERS);
    if (dev)	
        b3dfg_unmap_buffers(dev);
    b3dfg_set_transmission(dev, 0);
	b3dfg_close(dev);
	return r;
}
