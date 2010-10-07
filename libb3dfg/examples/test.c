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
    b3dfg_buffer_state state;

	dev = b3dfg_init(0);
	if (!dev) {
		fprintf(stderr, "init failed\n");
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

	printf("wand is %s\n",
		b3dfg_get_wand_status(dev) ? "connected" : "disconnected");

	frame_size = b3dfg_get_frame_size(dev);

	r = b3dfg_open(dev, 0);
	if (r) {
		fprintf(stderr, "open failed\n");
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
        if (buf != -1)
            b3dfg_release_buffer(dev);
		b3dfg_close(dev);
        b3dfg_exit(dev);
    }

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
