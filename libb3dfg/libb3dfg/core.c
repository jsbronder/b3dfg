/*
 * Core functionality
 *
 * Copyright (C) 2008 3M Company
 * Author: Daniel Drake <ddrake@brontes3d.com>
 *
 * Private software. Source code may not be distributed.
 */

#include <config.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "b3dfg.h"
#include "b3dfgi.h"

void b3dfg_log(enum b3dfg_log_level level, const char *function,
	const char *format, ...)
{
	const char *prefix;
	va_list args;

	switch (level) {
		case LOG_LEVEL_INFO: prefix = "info"; break;
		case LOG_LEVEL_WARNING: prefix = "warning"; break;
		case LOG_LEVEL_ERROR: prefix = "error"; break;
		case LOG_LEVEL_DEBUG: prefix = "debug"; break;
		default: prefix = "unknown"; break;
	}

	fprintf(stderr, "b3dfg:%s:%s: ", prefix, function);
	va_start(args, format);
	vfprintf(stderr, format, args);
	va_end(args);
}

API_EXPORTED struct b3dfg_dev *b3dfg_open(unsigned int idx)
{
	struct b3dfg_dev *dev;
	char filename[12];
	int fd;
	int r;
	int frame_size;

	if (idx > 9) {
		b3dfg_err("invalid index %d", idx);
		return NULL;
	}

	r = snprintf(filename, sizeof(filename), "/dev/b3dfg%1d", idx);
	if (r != sizeof(filename) - 1)
		return NULL;

	b3dfg_dbg("%s", filename);
	fd = open(filename, O_RDONLY);
	if (fd < 0) {
		b3dfg_err("open(%s) failed errno=%d", filename, errno);
		return NULL;
	}

	r = ioctl(fd, B3DFG_IOCGFRMSZ, &frame_size);
	if (r < 0) {
		b3dfg_err("IOCGFRMSIZE failed %d errno=%d", r, errno);
		close(fd);
		return NULL;
	}

	dev = malloc(sizeof(*dev));
	if (!dev) {
		close(fd);
		return NULL;
	}

	b3dfg_dbg("opened fd=%d frame_size=%d", fd, frame_size);
	dev->fd = fd;
	dev->frame_size = frame_size;
	dev->num_buffers = 0;
	dev->mapping = NULL;
	return dev;
}

API_EXPORTED void b3dfg_close(struct b3dfg_dev *dev)
{
	if (!dev)
		return;
	b3dfg_dbg("fd=%d", dev->fd);

	b3dfg_unmap_buffers(dev);
	if (close(dev->fd) < 0)
		b3dfg_err("close failed errno=%d", errno);
}

API_EXPORTED unsigned int b3dfg_get_frame_size(struct b3dfg_dev *dev)
{
	return dev->frame_size;
}

API_EXPORTED int b3dfg_set_num_buffers(struct b3dfg_dev *dev, int buffers)
{
	int r;
	b3dfg_dbg("%d buffers", buffers);
	r = ioctl(dev->fd, B3DFG_IOCTNUMBUFS, buffers);
	if (r < 0)
		b3dfg_err("IOCTNUMBUFS failed r=%d errno=%d", r, errno);
	else
		dev->num_buffers = buffers;
	return r;
}

API_EXPORTED int b3dfg_queue_buffer(struct b3dfg_dev *dev, int buffer)
{
	int r;
	
	b3dfg_dbg("buffer %d", buffer);
	r = ioctl(dev->fd, B3DFG_IOCTQUEUEBUF, buffer);
	if (r < 0)
		b3dfg_err("IOCTQUEUEBUF(%d) failed r=%d errno=%d", buffer, r, errno);
	return r;
}

API_EXPORTED int b3dfg_poll_buffer(struct b3dfg_dev *dev, int buffer)
{
	int r;

	b3dfg_dbg("buffer %d", buffer);
	r = ioctl(dev->fd, B3DFG_IOCQPOLLBUF, buffer);
	if (r < 0)
		b3dfg_err("IOCQPOLLBUF(%d) failed r=%d errno=%d", buffer, r, errno);
	return r;
}

API_EXPORTED int b3dfg_wait_buffer(struct b3dfg_dev *dev, int buffer)
{
	int r;

	b3dfg_dbg("buffer %d", buffer);
	r = ioctl(dev->fd, B3DFG_IOCQWAITBUF, buffer);
	if (r < 0)
		b3dfg_err("IOCQWAITBUF(%d) failed r=%d errno=%d", buffer, r, errno);
	return r;
}



API_EXPORTED unsigned char *b3dfg_map_buffers(struct b3dfg_dev *dev,
	int prefault)
{
	unsigned char *mapping;
	int flags = MAP_SHARED;

	if (dev->mapping) {
		b3dfg_err("buffers already mapped");
		return NULL;
	}

	if (prefault)
		flags |= MAP_POPULATE;
	
	b3dfg_dbg("");
	mapping = mmap(NULL,
		dev->frame_size * FRAMES_PER_BUFFER * dev->num_buffers, PROT_READ,
		flags, dev->fd, 0);
	if (mapping == MAP_FAILED) {
		b3dfg_err("mmap failed errno=%d", errno);
		return NULL;
	}

	dev->mapping = mapping;
	return mapping;
}

API_EXPORTED unsigned char *b3dfg_get_mapping(struct b3dfg_dev *dev)
{
	return dev->mapping;
}

API_EXPORTED void b3dfg_unmap_buffers(struct b3dfg_dev *dev)
{
	int r;
	if (!dev->mapping)
		return;

	b3dfg_dbg("");
	r = munmap(dev->mapping,
		dev->frame_size * FRAMES_PER_BUFFER * dev->num_buffers);
	if (r != 0)
		b3dfg_err("munmap failed r=%d errno=%d", r, errno);
	dev->mapping = NULL;
}

API_EXPORTED int b3dfg_init(void)
{
	return 0;
}

API_EXPORTED void b3dfg_exit(void)
{

}

