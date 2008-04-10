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

/**
 * \mainpage libb3dfg API reference
 * libb3dfg is a dynamic shared library providing a C API to the low-level
 * kernel interface to the Brontes Frame Grabber driver (b3dfg).
 *
 * The interface is deliberately modelled to be similar to the
 * libdc1394/video1394 interface used on previous versions of the product.
 *
 * A general introduction to this library is shown below. Be sure to check
 * the full API documentation for further details and some other
 * functionality.
 *
 * \section libinit Library initialization
 *
 * Before using this library, call b3dfg_init(). After using the library it is
 * good practice to call b3dfg_exit() to clean up resources, but this is not
 * mandatory; those resources will be cleaned up at application exit anyway.
 *
 * \section devhandling Device handling
 *
 * After initializing the library, the first thing you will want to do is
 * obtain a device handle (a b3dfg_dev pointer) by calling b3dfg_open().
 * All subsequent operations are performed based on this handle.
 *
 * \section buftheory Buffer theory
 *
 * The optics capture 3 images for any one frame, which are referred to
 * as the red, green and blue channels (even though they do not actually
 * represent colours: they are 3 distinct frames). A group of 3 frames is
 * referred to as a buffer.
 *
 * Before enabling image transmission, you must call b3dfg_set_num_buffers()
 * to allocate some buffers. The buffers are allocated by the kernel driver,
 * so you simply specify how many you would like.
 *
 * After allocating buffers, you can gain access to the frame data within by
 * <em>mapping</em> the buffers into your process using b3dfg_map_buffers().
 * This function returns a pointer to an area of memory with the following
 * structure:
 *
 * <table>
 * <tr>
 * <td>Buffer 0, red frame</td>
 * <td>Buffer 0, green frame</td>
 * <td>Buffer 0, blue frame</td>
 * </tr>
 * <tr>
 * <td>Buffer 1, red frame</td>
 * <td>Buffer 1, green frame</td>
 * <td>Buffer 1, blue frame</td>
 * </tr>
 * <tr>
 * <td>Buffer 2, red frame</td>
 * <td>Buffer 2, green frame</td>
 * <td>Buffer 2, blue frame</td>
 * </tr>
 * </table>
 *
 * Read the above representation left-to-right and then top-to-bottom. The
 * image data is presented contiguously in memory in this fashion.
 *
 * \section bufaccess Buffer addressing
 *
 * Once you have created the mapping and have a pointer to the start of it,
 * you can compute the address of any frame with simple pointer arithmetic.
 * Assigning frame numbers:
 * <ul><li>0 = red</li><li>1 = green</li><li>2 = blue</li></ul>
 *
 * The equation is:
 *
 * <code>
 *     frame_start_addr = mapping + (frame_size * ((buffer_idx * 3) + frame_num))
 * </code>
 *
 * It may be more intuitive to think of it as follows:
 *
 * <code>
 *     frame_start_addr = mapping + (buffer_idx * frame_size * 3) + (frame_num * frame_size)
 * </code>
 *
 * For example, buffer 2 green frame:
 *
 * <code>
 * frame_start_addr = mapping + (frame_size * ((2 * 3) + 1)) = mapping + (frame_size * 7)
 * </code>
 *
 * The frame_size can be determined by calling b3dfg_get_frame_size(). You
 * may wish to hardcode it to 1024x768 for optimization purposes, but do
 * remember that the frame size may change in future: there are plans to
 * include the first level pyramid data after the full-res frame.
 *
 * \section transfer Data transfer
 *
 * When you are ready to receive data into a buffer, you must <em>queue</em>
 * it with b3dfg_queue_buffer(). A queued buffer will be filled with 3 frames
 * of image data at some point in the future, whereas an unqueued buffer will
 * not be touched by the underlying driver or hardware.
 *
 * It is legal to queue multiple buffers, and it is in fact necessary for
 * performance: unless there are free buffers available, frames will be
 * dropped. Buffers will be filled in the order that they were queued.
 *
 * \section bufmgmt Buffer ownership
 *
 * Each buffer has two modes of ownership; a buffer is either owned by the
 * software or it is owned by the hardware.
 *
 * Reading from any buffer is only permitted at times when that buffer is
 * owned by the software.
 *
 * Writing to any buffer from software will result in undefined behaviour,
 * regardless of ownership.
 *
 * After allocation, all buffers are automatically owned by the software, and
 * they will not be used for data transfer by default. If you want to use a
 * buffer to receive some data, you must <em>queue</em> that buffer using
 * b3dfg_queue_buffer(). Queuing a buffer also transfers ownership of that
 * buffer to the hardware - you cannot access that buffer until further notice.
 *
 * 
 *
 */

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
	fprintf(stderr, "\n");
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

API_EXPORTED int b3dfg_get_fd(struct b3dfg_dev *dev)
{
	return dev->fd;
}

API_EXPORTED unsigned int b3dfg_get_frame_size(struct b3dfg_dev *dev)
{
	return dev->frame_size;
}

API_EXPORTED int b3dfg_set_transmission(struct b3dfg_dev *dev, int enabled)
{
	int r;
	b3dfg_dbg("%s", enabled ? "enabled" : "disabled");
	r = ioctl(dev->fd, B3DFG_IOCTTRANS, enabled);
	if (r < 0)
		b3dfg_err("IOCTTRANS(%d) failed r=%d errno=%d", enabled, r, errno);
	return r;
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

