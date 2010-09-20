/*
 * Brontes frame grabber access library private include
 *
 * Copyright (C) 2008 3M Company
 * Author: Daniel Drake <ddrake@brontes3d.com>
 *
 * Private software. Source code may not be distributed.
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
#define B3DFG_IOCTTRANS		_IO(B3DFG_IOC_MAGIC, 1)
#define B3DFG_IOCTRELBUF	_IO(B3DFG_IOC_MAGIC, 2)
#define B3DFG_IOCTGETBUF	_IOWR(B3DFG_IOC_MAGIC, 3, struct b3dfg_wait)

#define FRAMES_PER_BUFFER 3
#define B3DFG_NUM_BUFFERS 3

struct b3dfg_dev {
	int fd;
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
