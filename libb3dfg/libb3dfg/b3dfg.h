/*
 * Brontes frame grabber access library public header
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

#ifndef __B3DFG_H___
#define __B3DFG_H___
#include <sys/time.h>

struct b3dfg_dev;
typedef struct b3dfg_dev b3dfg_dev;

struct b3dfg_buffer_state {
	unsigned int dropped;
	struct timeval stamp;
	int buffer;
	char *addr;
};
typedef struct b3dfg_buffer_state b3dfg_buffer_state;

b3dfg_dev *b3dfg_init(unsigned int idx);
void b3dfg_exit(b3dfg_dev *dev);

int b3dfg_open(b3dfg_dev *dev, int prefault);
void b3dfg_close(b3dfg_dev *dev);

int b3dfg_get_wand_status(b3dfg_dev *dev);
int b3dfg_get_wand_status_fd(b3dfg_dev *dev);

int b3dfg_get_fd(b3dfg_dev *dev);
unsigned int b3dfg_get_frame_size(b3dfg_dev *dev);

int b3dfg_release_buffer(b3dfg_dev *dev);
int b3dfg_get_buffer(b3dfg_dev *dev, unsigned int timeout, 
	b3dfg_buffer_state *state);
#endif

// vim: noet
