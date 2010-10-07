/*
 * Brontes frame grabber example
 *
 * Copyright (c) 2009, 3M
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the 3M nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 *
 * Authors:
 * 	Justin Bronder <jsbronder@brontes3d.com>
 * 	Daniel Drake <ddrake@brontes3d.com>
 *
 */


#include <stdio.h>
#include <unistd.h>
#include <poll.h>

#include <libb3dfg/b3dfg.h>
#include <examples/serial_cmd.h>

#define NUM_BUFFERS 4

static b3dfg_dev *dev;
static unsigned int frame_size;
static int frame_number = 0;

static void write_frame(char *addr, int frame)
{
	char filename[50];
	FILE *fd;
    int r;

    frame_number++;
	sprintf(filename, "cap%02d.pgm", frame_number);
	fd = fopen(filename, "w");
	fprintf(fd, "P5 1024 768 255 ");
	r = fwrite(addr + (frame_size * frame), 1, frame_size, fd);
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
	int i;
	int r = 1;
    int fd = -1;
	struct pollfd pfd = { .events = POLLIN };
    int triggering = 0;
    b3dfg_buffer_state state;

	dev = b3dfg_init(0);
	if (!dev) {
		fprintf(stderr, "init failed\n");
		goto out;
	}

	frame_size = b3dfg_get_frame_size(dev);

    if ( (fd = open_serial("/dev/ttyUSB0")) <= 0 ) {
        r = -1;
        goto out;
    }

    if ( (triggering = send_cmd(fd, START_TRIGGERS)) == -1){
        r = -1;
        goto out;
    }

	r = b3dfg_open(dev, 1);
	if (r) {
		fprintf(stderr, "open failed\n");
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

		r = b3dfg_get_buffer(dev, 1000, &state);
		if (r < 0) {
			fprintf(stderr, "poll failed error %d\n", r);
			break;
		}
		if (r == 0) {
			fprintf(stderr, "buffer not ready after poll()?\n");
			break;
		}
		if (state.dropped > 0) {
			printf("%d frame(s) dropped\n", state.dropped);
		}
		write_to_file(state.addr);
        b3dfg_release_buffer(dev);
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
