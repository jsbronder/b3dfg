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
#include <sys/param.h>
#include <unistd.h>
#include <string.h>

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
 * This page provides a general introduction to the library. For specific
 * details on the API, be sure to consult the rest of this documentation.
 * Click the 'Modules' tab at the top of this page.
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
 * referred to as a buffer or triplet.
 *
 * You can gain access to the frame data within buffers by <em>mapping</em>
 * the buffers into your process using b3dfg_map_buffers(). This function
 * returns a pointer to an area of memory with the following structure:
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
 * When you are ready to receive data you must request a lock on associated
 * buffer with b3dfg_get_buffer().  This lets the driver know that the buffer is
 * being accessed by userspace and it will not touch the buffer until it is
 * released with b3dfg_release_buffer().  Due to triple buffering in the driver
 * b3dfg_get_buffer() will almost always return immediately.  Before requesting
 * another buffer, b3dfg_release_buffer() should be called in order to insure
 * the driver always has two free buffers to use.
 *
 * \section bufstate Buffer states
 *
 * A buffer is in 1 of 3 states at any moment in time:
 *  - <b>Idle</b> - the buffer is not in use by userland or the driver.
 *  - <b>Pending</b> - the buffer is currently being used to serve
 *    DMA requests from the board.
 *  - <b>Populated</b> - the buffer has been filled and is ready to be
 *    sent to userspace.
 *  - <b>User</b> - the buffer is currently being used by this library.
 *
 * At the point when transmission is enabled, all buffers are reset to the
 * idle state.
 *
 * \section pollbuf Buffer polling
 *
 * The underlying kernel driver also implements the <code>poll</code>
 * operation meaning that system calls such as poll() and select() are
 * supported. The b3dfg_get_fd() function returns a file descriptor which
 * can be monitored for read events (<code>POLLIN</code>). If one of these 
 * system calls indicates activity on the file descriptor it means that at
 * least one buffer is in the populated state.
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
 * Buffers are owned by the software when they are in the user state. They
 * are owned by the hardware at all other times. In other words, releasing
 * a buffer transfers control to the hardware until it is later delivered
 * back to the user as a result of b3dfg_get_buffer().
 *
 * \section dropped Tracking dropped frames
 *
 * If the software does not request buffers quickly enough, there is a chance
 * that the kernel will drop a previously filled buffer in favor of the newest
 * coming from the board.  In this case, the <tt>dropped</tt> parameter of
 * b3dfg_get_buffer() will reflect the number of buffers dropped since 
 * transmission was enabled.  If the driver itself is unable to keep up with
 * the board (this should not occur), the board will also report the number of
 * buffers it has dropped.  This count is also appended to <tt>dropped</tt>.
 *
 * The dropped parameter is optional, you can pass NULL if you don't care.
 * However, if you do want to track dropped frames, you must pass an output
 * location <em>every time</em>. You are only informed about each dropped
 * frame triplet once, and if the library does receive information that
 * frames have been dropped but cannot pass it on, then that information will
 * be lost (it does not accumulate).
 *
 * \section tv Timestamps
 *
 * When the driver completes the DMA transfer of a buffer, it also keeps a
 * record of when this occured.  You can access this timestamp by passing a
 * non-NULL struct timeval as the <tt>tv</tt> parameter of b3dfg_get_buffer().
 * Timestamps are tracked using the jiffies counter within the kernel and are
 * precise to milliseconds.
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

static int read_sysfs_int(const char *path, int *val)
{
	FILE *fp = NULL;
	char str[32];

	fp = fopen(path, "r");
	if (!fp) {
		b3dfg_err("open(%s) failed errno %d", path, errno);
		return -1;
	}

	if (!fgets(str, 32, fp)) {
		b3dfg_err("fgets() failed errno %d", errno);
		fclose(fp);
		return -1;
	}
	fclose(fp);

	errno = 0;
	*val = (int)strtol(str, NULL, 10);
	if (errno == ERANGE) {
		b3dfg_err("strtol(%s) failed errno %d", str, errno);
		return -1;
	}
	return 0;
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
	char filename[MAXPATHLEN];
	int fd;
	int r;
	int frame_size;

	if (idx > 9) {
		b3dfg_err("invalid index %d", idx);
		return NULL;
	}

	r = snprintf(filename, MAXPATHLEN, "/dev/b3dfg%1d", idx);
	if (r < 0 || r >= MAXPATHLEN) {
		b3dfg_err("snprintf(devpath) failed errno=%d\n", idx, errno);
		return NULL;
	}

	b3dfg_dbg("%s", filename);
	fd = open(filename, O_RDONLY);
	if (fd < 0) {
		b3dfg_err("open(%s) failed errno=%d", filename, errno);
		return NULL;
	}

	dev = malloc(sizeof(*dev));
	if (!dev) {
		close(fd);
		return NULL;
	}

	r = snprintf(filename, MAXPATHLEN,
		"/sys/class/b3dfg/b3dfg%1d/device/frame_size", idx);
	if (r < 0 || r >= MAXPATHLEN) {
		b3dfg_err("snprintf(syspath) failed errno=%d\n", errno);
		return NULL;
	}

	r = read_sysfs_int(filename, &frame_size);
	/* b3dfg_err() called in read_sysfs_int() */
	if (r < 0)
		return NULL;

	b3dfg_dbg("opened fd=%d frame_size=%d", fd, frame_size);
	dev->fd = fd;
	dev->idx = idx;
	dev->frame_size = frame_size;
	dev->num_buffers = B3DFG_NUM_BUFFERS;
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

/** \ingroup core
 * Retrieve wand cable status.
 *
 * \param dev a device handle
 * \returns 1 if wand is present
 * \returns 0 if wand is disconnected
 * \returns negative code on error
 */
API_EXPORTED int b3dfg_get_wand_status(b3dfg_dev *dev)
{
	int r;
	int status;
	char filename[MAXPATHLEN];	
	
	r = snprintf(filename, MAXPATHLEN,
		"/sys/class/b3dfg/b3dfg%1d/device/cable_status", dev->idx);
	if (r < 0 || r >= MAXPATHLEN) {
		b3dfg_err("snprintf(syspath) failed errno=%d\n", errno);
		return -1;
	}

	r = read_sysfs_int(filename, &status);
	/* b3dfg_err() called in read_sysfs_int() */
	if (r < 0)
		return -1;

	return status;
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
 * Release a buffer. This moves a buffer from the user state into the idle state
 * state.  This function should be called after processing on any buffer recieved
 * via b3dfg_get_buffer() and before said function is called again.
 *
 * \param dev a device handle
 * \param buffer the buffer to release. 
 * \returns 0 on buffer release, 1 if the buffer was already released and negative
 * on error
 */
API_EXPORTED int b3dfg_release_buffer(b3dfg_dev *dev)
{
	int r;
	
	r = ioctl(dev->fd, B3DFG_IOCTRELBUF);
	if (r < 0) {
		b3dfg_err("IOCTRELBUF failed r=%d errno=%d", r, errno);
	}
	return r;
}


/** \ingroup io
 * Get the newest buffer.  If no buffers are available (very rare aside from 
 * immediately after starting transmission) this function will sleep until a
 * buffer is ready.
 *
 * Upon success (i.e. frame data is present in buffer), this function call
 * automatically moves the buffer from the populated state to the user state.
 *
 * \param dev a device handle
 * \param buffer the buffer returned from the driver, used with release_buffer()
 * \param timeout timeout in milliseconds, or 0 for unlimited timeout
 * \param state the state of the driver after the ioctl call, fills in the 
 * buffer timestamp, buffer index, number of dropped triplets and address
 * of the beginning of the buffer.
 * \returns milliseconds remaining in timeout on success (buffer now contains
 * data and has been moved to user state), 0 if there was no timeout
 * \returns -ETIMEDOUT on timeout
 * \returns other negative code on other error
 */
API_EXPORTED int b3dfg_get_buffer(b3dfg_dev *dev, unsigned int timeout,
	b3dfg_buffer_state *state)
{
	struct b3dfg_wait w = { .timeout = timeout };
	int r;

	if (!state)
		return -EINVAL;

	r = ioctl(dev->fd, B3DFG_IOCTGETBUF, &w);
	if (r < 0) {
		if (timeout && errno == ETIMEDOUT) {
			b3dfg_dbg("timed out");
			return -ETIMEDOUT;
		}
		b3dfg_err("IOCTGETBUF failed r=%d errno=%d", r, errno);
	} else {
		state->dropped = w.triplets_dropped;
		memcpy(&state->stamp, &w.tv, sizeof(state->stamp));
		state->buffer = w.buffer_idx;
		state->addr = dev->mapping + (state->buffer * dev->frame_size * 3);
	}
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
 * You cannot map the buffers twice, but you can use b3dfg_get_mapping() to
 * retrieve the address of an already-created mapping.
 *
 * \param dev a device handle \param prefault whether to prefault the buffers
 * or not. If prefault=0, you will incur a small performance penalty the first
 * time you access each 4kb page within the mapping. If non-zero, this
 * parameter causes the system to access each page immediately so that no
 * performance penalty occurs later.  \returns the address of the new mapping
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

// vim: noet
