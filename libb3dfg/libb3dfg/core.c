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
 * good practice to call b3dfg_exit() to clean up resources, but this is
 * optional; those resources will be cleaned up at application exit anyway.
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
 * Buffers are dequeued when they are filled (populated). Your application
 * must then poll the buffer before accessing the data.
 *
 * \section bufstate Buffer states
 *
 * A buffer is in 1 of 3 states at any moment in time:
 *  - <b>Polled</b> - the buffer is not queued, and if it was previously
 *    queued, it has been dequeued and polled for its data.
 *  - <b>Pending</b> - the buffer has been queued with b3dfg_queue_buffer()
 *    and will be filled with image data at some point in the future.
 *  - <b>Populated</b> - the buffer was previously queued and has been
 *    populated with image data. It is no longer queued.
 *
 * At the point when transmission is enabled, all buffers are reset to the
 * polled state.
 *
 * \section pollbuf Buffer polling
 *
 * Some time after a buffer is queued, it will move into the populated state.
 * When this state change occurs, your application will want to move the
 * buffer into the polled state and then access the data inside.
 *
 * In order to move a populated buffer into the polled state, call the
 * b3dfg_poll_buffer() function. It is legal to call this on a pending buffer
 * too - the return code indicates if any state change actually occured. In
 * other words, you can use this function to check if a previously-queued
 * buffer has been filled with data or if it is still pending.
 *
 * You should always ensure that a buffer moves into the polled state (by
 * checking the b3dfg_poll_buffer() return code) before accessing the data
 * contained within.
 *
 * The b3dfg_wait_buffer() can be used to sleep on a pending buffer - it will
 * put your application to sleep, wait until the buffer moves to the populated
 * state, poll it, and then wake up your application again. It is legal to call
 * this function on a buffer that is already populated (it will poll it and
 * return immediately). Also note that since wait includes poll, you do not
 * need to call b3dfg_poll_buffer() on a buffer that you have just waited on.
 *
 * The underlying kernel driver also implements the <code>poll</code>
 * operation meaning that system calls such as poll() and select() are
 * supported. The b3dfg_get_fd() function returns a file descriptor which
 * can be monitored for read events (<code>POLLIN</code>). If one of these 
 * system calls indicates activity on the file descriptor it means that at
 * least one buffer is in the populated state.
 *
 * To use the poll() implementation, your application will want to track which
 * buffer is going to be filled up next. Then call poll() or select() or
 * similar, wait for activity, and then poll the buffer that you were expecting
 * to be filled. Process the data, do whatever else, then loop. Even though
 * you may know exactly which buffer was filled up when select() returned, it
 * is important that you poll the buffer to move it into the polled state so
 * that the slate is clean for the next iteration of the loop.
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
 * Buffers are owned by the software when they are in the polled state. They
 * are owned by the hardware at all other times. In other words, queueing a
 * buffer transfers control to the hardware until you poll it and observe
 * that data has been captured.
 *
 */

/** @defgroup core Core operations */
/** @defgroup io Buffers and I/O */

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

/** \ingroup core
 * Obtain a device handle for a frame grabber.
 *
 * \param idx numerical index of the frame grabber you wish to open, usually 0
 * \returns a device handle, or NULL on error
 */
API_EXPORTED b3dfg_dev *b3dfg_open(unsigned int idx)
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

/** \ingroup core
 * Close a device handle. Call this when you have finished using a device.
 * If they were mapped, buffers are unmapped during this operation.
 *
 * If a NULL device handle is provided, this function simply returns without
 * doing anything.
 *
 * \param dev the device to close
 */
API_EXPORTED void b3dfg_close(b3dfg_dev *dev)
{
	if (!dev)
		return;
	b3dfg_dbg("fd=%d", dev->fd);

	b3dfg_unmap_buffers(dev);
	if (close(dev->fd) < 0)
		b3dfg_err("close failed errno=%d", errno);
}

/** \ingroup io
 * Obtain a file descriptor corresponding to a device handle. You can pass this
 * file descriptor to the poll() or select() system calls (or a variant) -
 * if those system calls indicate availability of data to be read then it
 * indicates that one or more buffers are in the populated state.
 *
 * \param dev a device handle
 * \returns a file descriptor corresponding to the device handle
 */
API_EXPORTED int b3dfg_get_fd(b3dfg_dev *dev)
{
	return dev->fd;
}

/** \ingroup io
 * Obtain the size of the frames presented by the frame grabber. Multiply
 * this by 3 to calculate the size of a buffer.
 *
 * \param dev a device handle
 * \returns the frame size in bytes
 */
API_EXPORTED unsigned int b3dfg_get_frame_size(b3dfg_dev *dev)
{
	return dev->frame_size;
}

/** \ingroup io
 * Enable or disable transmission. It is legal to attempt to enable
 * transmission if it was already enabled, or disable transmission if it was
 * already disabled; this function does not indicate error in either case.
 * FIXME: kernel driver does not agree yet!
 *
 * \param dev a device handle
 * \param enabled 1 to enable transmission, 0 to disable
 * \returns 0 on success, non-zero on error
 */
API_EXPORTED int b3dfg_set_transmission(b3dfg_dev *dev, int enabled)
{
	int r;
	b3dfg_dbg("%s", enabled ? "enabled" : "disabled");
	r = ioctl(dev->fd, B3DFG_IOCTTRANS, enabled);
	if (r < 0)
		b3dfg_err("IOCTTRANS(%d) failed r=%d errno=%d", enabled, r, errno);
	return r;
}

/** \ingroup io
 * Set the number of buffers to potentially be used for later I/O. Calling
 * this function causes the kernel to allocate the buffers internally.
 *
 * It is legal to call this function multiple times with different values,
 * the kernel will grow or shrink its buffer pool accordingly.
 *
 * This function cannot be called when a mapping is active or transmission
 * is enabled.
 *
 * This function will dequeue all buffers that were previously queued.
 *
 * \param dev a device handle
 * \param buffers the number of buffers to keep in the pool
 * \returns 0 on success, non-zero on error
 */
API_EXPORTED int b3dfg_set_num_buffers(b3dfg_dev *dev, int buffers)
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

/** \ingroup io
 * Queue a buffer. This moves a buffer from the polled state into the pending
 * state.
 *
 * \param dev a device handle
 * \param buffer the buffer to queue
 * \returns 0 on success, non-zero on error
 */
API_EXPORTED int b3dfg_queue_buffer(b3dfg_dev *dev, int buffer)
{
	int r;
	
	b3dfg_dbg("buffer %d", buffer);
	r = ioctl(dev->fd, B3DFG_IOCTQUEUEBUF, buffer);
	if (r < 0)
		b3dfg_err("IOCTQUEUEBUF(%d) failed r=%d errno=%d", buffer, r, errno);
	return r;
}

/** \ingroup io
 * Poll a buffer. This function can be used to determine whether a buffer is
 * pending or if it has been populated.
 *
 * If the buffer has been populated, this function returns 1 and the buffer
 * is automatically moved into the polled state. If the buffer has not been
 * populated (it is in the polled or pending states) this function returns 0
 * without taking further actions.
 * 
 * \param dev a device handle
 * \param buffer the buffer to poll
 * \returns 1 if the buffer was populated (and is now polled)
 * \returns 0 otherwise
 * \returns negative on error
 */
API_EXPORTED int b3dfg_poll_buffer(b3dfg_dev *dev, int buffer)
{
	int r;

	b3dfg_dbg("buffer %d", buffer);
	r = ioctl(dev->fd, B3DFG_IOCQPOLLBUF, buffer);
	if (r < 0)
		b3dfg_err("IOCQPOLLBUF(%d) failed r=%d errno=%d", buffer, r, errno);
	return r;
}

/** \ingroup io
 * Wait on a pending buffer. This function can be used to sleep until a
 * specific buffer is populated.
 *
 * You can view this function as equivalent to calling b3dfg_poll_buffer() in
 * a loop until it returns non-zero, except it is actually much more efficient
 * than that. This function causes the kernel to put your application to
 * sleep, then some clever scheduler magic ensures the process is only woken
 * up again when frame data is ready.
 * 
 * If the buffer was already populated, this function immediately returns
 * success without sleeping.
 *
 * Upon success (i.e. frame data is present in buffer), this function call
 * automatically moves the buffer from the populated state to the polled state.
 *
 * \param dev a device handle
 * \param buffer the buffer to wait upon
 * \returns 0 on success (buffer now contains data and has been moved to
 * polled state)
 * \returns non-zero on error
 */
API_EXPORTED int b3dfg_wait_buffer(b3dfg_dev *dev, int buffer)
{
	int r;

	b3dfg_dbg("buffer %d", buffer);
	r = ioctl(dev->fd, B3DFG_IOCQWAITBUF, buffer);
	if (r < 0)
		b3dfg_err("IOCQWAITBUF(%d) failed r=%d errno=%d", buffer, r, errno);
	return r;
}

/** \ingroup io
 * Map frame buffers into process address space. This function returns the
 * base address of a mapping where you can find the image data. Buffers are
 * located contiguously throughout this mapping, starting from buffer 0.
 * Inside each buffer, each frame is located contiguously, in red-green-blue
 * order. You can use simple arithmetic based on the size of a frame in order
 * to locate any frame within the mapping. The mapping is of size precisely
 * big enough to accomodate all of the allocated buffers and their frames.
 *
 * This function can only be called after a specified number of buffers
 * have been set with b3dfg_set_num_buffers(). You cannot map the buffers
 * twice, but you can use b3dfg_get_mapping() to retrieve the address of
 * an already-created mapping.
 * 
 * You must unmap the buffer with b3dfg_unmap_buffers() before altering
 * the number of buffers.
 *
 * \param dev a device handle
 * \param prefault whether to prefault the buffers or not. If prefault=0, you
 * will incur a small performance penalty the first time you access each 4kb
 * page within the mapping. If non-zero, this parameter causes the system to
 * access each page immediately so that no performance penalty occurs later.
 * \returns the address of the new mapping
 * \returns NULL on error
 */
API_EXPORTED unsigned char *b3dfg_map_buffers(b3dfg_dev *dev, int prefault)
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

/** \ingroup io
 * Obtain the base address a mapping previously created with
 * b3dfg_map_buffers().
 *
 * \param dev a device handle
 * \returns the address of the active mapping
 * \returns NULL if there is no active mapping
 */
API_EXPORTED unsigned char *b3dfg_get_mapping(b3dfg_dev *dev)
{
	return dev->mapping;
}

/** \ingroup io
 * Unmap the mapping previously created with b3dfg_map_buffers(). It is legal
 * to call this function even when there is no active mapping, in which case
 * this function simply returns.
 *
 * Do not attempt to access any previous mapping after calling this function.
 *
 * \param dev a device handle
 */
API_EXPORTED void b3dfg_unmap_buffers(b3dfg_dev *dev)
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

/** \ingroup core
 * Initialize the library. This function should be called before any other
 * libb3dfg function is called.
 *
 * \returns 0 on success
 * \returns non-zero on error
 */
API_EXPORTED int b3dfg_init(void)
{
	return 0;
}

/** \ingroup core
 * Deinitialize the library and clean up resources. Calling this functional
 * is optional (if you do not, all cleanup happens at exit anyway). Do not call
 * any libb3dfg functions after calling this function unless you call
 * b3dfg_init() again.
 */
API_EXPORTED void b3dfg_exit(void)
{

}

