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

#define B3DFG_IOC_MAGIC         0xb3 /* dfg :-) */
#define B3DFG_IOCGFRMSZ         _IOR(B3DFG_IOC_MAGIC, 1, int)
#define B3DFG_IOCTNUMBUFS       _IO(B3DFG_IOC_MAGIC, 2)
#define B3DFG_IOCTTRANS         _IO(B3DFG_IOC_MAGIC, 3)
#define B3DFG_IOCTQUEUEBUF      _IO(B3DFG_IOC_MAGIC, 4)
#define B3DFG_IOCQPOLLBUF       _IO(B3DFG_IOC_MAGIC, 5)
#define B3DFG_IOCQWAITBUF       _IO(B3DFG_IOC_MAGIC, 6)

struct b3dfg_dev {
	int fd;
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

