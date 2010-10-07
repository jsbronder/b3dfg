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
