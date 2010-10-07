/*
 * Brontes frame grabber access library private header
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

#ifndef __B3DFGI_H___
#define __B3DFGI_H___
#include <sys/ioctl.h>
#include <sys/time.h>

struct b3dfg_wait {
	int buffer_idx;
	unsigned int timeout;
	unsigned int triplets_dropped;
	struct timeval tv;
};

#define B3DFG_IOC_MAGIC		0xb3 /* dfg :-) */
#define B3DFG_IOCTRELBUF	_IO(B3DFG_IOC_MAGIC, 1)
#define B3DFG_IOCTGETBUF	_IOWR(B3DFG_IOC_MAGIC, 2, struct b3dfg_wait)

#define FRAMES_PER_BUFFER 3
#define B3DFG_NUM_BUFFERS 3

struct b3dfg_dev {
	int fd;
	int cstate_fd;
	unsigned int idx;
	unsigned int frame_size;
	unsigned int num_buffers;
	unsigned char *mapping;
};

enum b3dfg_log_level {
	LOG_LEVEL_DEBUG,
	LOG_LEVEL_INFO,
	LOG_LEVEL_WARNING,
	LOG_LEVEL_ERROR,
};

void b3dfg_log(enum b3dfg_log_level, const char *function,
	const char *format, ...);

#ifdef ENABLE_LOGGING
#define _b3dfg_log(level, fmt...) b3dfg_log(level, __FUNCTION__, fmt)
#else
#define _b3dfg_log(level, fmt...)
#endif

#define b3dfg_info(fmt...) _b3dfg_log(LOG_LEVEL_INFO, fmt)
#define b3dfg_warn(fmt...) _b3dfg_log(LOG_LEVEL_WARNING, fmt)
#define b3dfg_err(fmt...) _b3dfg_log(LOG_LEVEL_ERROR, fmt)

#ifdef ENABLE_DEBUG_LOGGING
#define b3dfg_dbg(fmt...) _b3dfg_log(LOG_LEVEL_DEBUG, fmt)
#else
#define b3dfg_dbg(fmt...)
#endif

#endif

// vim: noet
