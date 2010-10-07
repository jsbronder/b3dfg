/*
 * Core functionality
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
 * As part of initializing the library, b3dfg_init will return a device
 * handle (a b3dfg_dev pointer).  All subsequent operations are performed based
 * on this handle.
 *
 * \section buftheory Buffer theory
 *
 * The optics capture 3 images for any one frame, which are referred to
 * as the red, green and blue channels (even though they do not actually
 * represent colours: they are 3 distinct frames). A group of 3 frames is
 * referred to as a buffer or triplet.
 *
 * You can gain access to the frame data within buffers by <em>mapping</em>
 * the buffers into your process using b3dfg_open() which calls
 * b3dfg_map_buffers(). The latter sets up an area of memory with the
 * following structure:
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
 * Once you have created the mapping you may call b3dfg_get_buffer() which will
 * return a pointer to the beginning of the buffer.  From here you can compute
 * the address of any frame with simple pointer arithmetic.
 * Assigning frame numbers:
 * <ul><li>0 = red</li><li>1 = green</li><li>2 = blue</li></ul>
 *
 * The equation is:
 *
 * <code>
 *     frame_start_addr = mapping + (frame_size * frame_num)
 * </code>
 *
 * For example, buffer 2 green frame:
 *
 * <code>
 * frame_start_addr = mapping + (frame_size * 1)
 * </code>
 *
 * The frame_size can be determined by calling b3dfg_get_frame_size().
 *
 * \section transfer Data transfer
 *
 * When you are ready to receive data you must request a lock on associated
 * buffer with b3dfg_get_buffer().  This lets the driver know that the buffer is
 * being accessed by userspace and it will not touch the buffer until it is
 * released with b3dfg_release_buffer().  Due to triple buffering in the driver
 * b3dfg_get_buffer() will almost always return immediately.  Before requesting
 * another buffer, b3dfg_release_buffer() should be called in order to insure
 * the driver always has two free buffers to use.  This is enforced by the driver
 * and attempting to hold two buffers at the same time will yield EPERM.
 *
 * \section bufstate Buffer states
 *
 * A buffer is in 1 of 4 states at any moment in time:
 *  - <b>Idle</b> - the buffer is not in use by userland or the driver.
 *  - <b>Pending</b> - the buffer is currently being used to serve
 *    DMA requests from the board.
 *  - <b>Populated</b> - the buffer has been filled and is ready to be
 *    sent to userspace.
 *  - <b>User</b> - the buffer is currently being used by this library.
 *
 * At the point when the device is opened with b3dfg_open_buffer, all buffers are
 * reset to the idle state.
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
 * the device was last opened.
 *
 * \section tv Timestamps
 *
 * When the driver completes the DMA transfer of a buffer, it also keeps a
 * record of when this occured.  Timestamps are tracked using the jiffies 
 * counter within the kernel and are precise to milliseconds.
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

/*
 * Unmap the mapping previously created with b3dfg_open(). It is legal
 * to call this function even when there is no active mapping, in which case
 * this function simply returns.
 *
 * Do not attempt to access any previous mapping after calling this function.
 *
 * \param dev a device handle
 */
static void b3dfg_unmap_buffers(b3dfg_dev *dev)
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
 * Request access to the frame buffers.  If successful, transmission is enabled
 * on the board and the driver will start handling buffers as they are passed from
 * the hardware.  Only one process at a time is allowed to own frame buffers in
 * this manner.  If access is granted, frame buffers are mapped into the process
 * address space and can be accessed after calling b3dfg_get_buffer().
 *
 * \param dev a device handle
 * \param prefault whether to prefault the buffers or not. If prefault=0, you will
 * incur a small performance penalty the first time you access each 4kb page within
 * the mapping. If non-zero, this parameter causes the system to access each page
 *
 * \returns zero on success, non-zero on error.
 */
API_EXPORTED int b3dfg_open(b3dfg_dev *dev, int prefault)
{
	char filename[MAXPATHLEN];
	int flags = MAP_SHARED;
	unsigned char *mapping;
	int fd;
	int r;

	if (dev->fd > 0)
		return 0;

	r = snprintf(filename, MAXPATHLEN, "/dev/b3dfg%1d", dev->idx);
	if (r < 0 || r >= MAXPATHLEN) {
		r = errno == 0 ? ENOMEM : errno;
		b3dfg_err("snprintf() failed errno=%d\n", dev->idx, r);
		return r;
	}

	b3dfg_dbg("%s", filename);
	fd = open(filename, O_RDONLY);
	if (fd < 0) {
		r = errno;
		b3dfg_err("open(%s) failed errno=%d", filename, r);
		return r;
	}

	if (dev->mapping)
		b3dfg_unmap_buffers(dev);

	if (prefault)
		flags |= MAP_POPULATE;
	
	mapping = mmap(NULL,
		dev->frame_size * FRAMES_PER_BUFFER * dev->num_buffers, PROT_READ,
		flags, fd, 0);
	if (mapping == MAP_FAILED) {
		r = errno;
		b3dfg_err("mmap failed errno=%d", r);
		if(close(fd) < 0)
			b3dfg_err("cleanup close failed errno=%d", errno);
		return r;
	}

	dev->fd = fd;
	dev->mapping = mapping;
	
	return 0;
}

/** \ingroup core
 * Close a device handle. Call this when no longer want to receive buffer from the
 * hardware.  Once closed, b3dfg_open() can be called again if more buffers are
 * needed.
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
	if (dev->fd > 0) {
		if (close(dev->fd) < 0)
			b3dfg_err("close failed errno=%d", errno);
		dev->fd = -1;
	}
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

	if (dev->cstate_fd != -1) {
		dev->cstate_fd = -1;
		close(dev->cstate_fd);
	}
	
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
 * Obtain a file descriptor corresponding to the cable state.  You can
 * pass this descriptor to the poll() or select() system calls to be alerted
 * when the driver notices a cable state change.
 *
 * Note that this is exported via sysfs, so reading from this file descriptor
 * is undefined.  The correct way to get state after poll() or select()
 * to call b3dfg_get_wand_status() (which will close this file descriptor).
 */
API_EXPORTED int b3dfg_get_wand_status_fd(b3dfg_dev *dev)
{
	char filename[MAXPATHLEN];
	char str[32];
	int fd, r;

	if (dev->cstate_fd != -1) {
		close(dev->cstate_fd);
		dev->cstate_fd = -1;
	}
	
	r = snprintf(filename, MAXPATHLEN,
		"/sys/class/b3dfg/b3dfg%1d/device/cable_status", dev->idx);
	if (r < 0 || r >= MAXPATHLEN) {
		r = errno == 0 ? ENOMEM : errno;
		b3dfg_err("snprintf(syspath) failed errno=%d\n", errno);
		return -1;
	}

	fd = open(filename, O_RDONLY);
	if (fd < 0) {
		r = errno;
		b3dfg_err("open(%s) failed errno=%d", filename, r);
		return r;
	}

	if (read(fd, str, 32) < 0){
		r = errno;
		close(fd);
		b3dfg_err("read(%s) failed errno=%d", filename, r);
		return r;
	}
	dev->cstate_fd = fd;
	return fd;
}

/** \ingroup io
 * Release a buffer. This moves a buffer from the user state into the idle state
 * state.  This function should be called after processing on any buffer recieved
 * via b3dfg_get_buffer() and before said function is called again.
 *
 * \param dev a device handle
 * \param buffer the buffer to release. 
 * \returns 0 on buffer release, negative on error.
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


/** \ingroup core
 * Initialize the library and obtain a device handle for the frame grabber.
 * This function should be called before any other libb3dfg function is called.
 *
 * \param idx numerical index of the frame grabber to be opened.
 * \returns a device handle or NULL on error.
 */
API_EXPORTED b3dfg_dev * b3dfg_init(unsigned int idx)
{
	struct b3dfg_dev *dev;
	char filename[MAXPATHLEN];
	int r;
	int frame_size;

	if (idx > 9) {
		b3dfg_err("invalid index %d", idx);
		return NULL;
	}

	dev = malloc(sizeof(*dev));
	if (!dev)
		return NULL;

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

	b3dfg_dbg("init frame_size=%d", frame_size);
	dev->fd = -1;
	dev->idx = idx;
	dev->frame_size = frame_size;
	dev->num_buffers = B3DFG_NUM_BUFFERS;
	dev->mapping = NULL;
	return dev;
}

/** \ingroup core
 * Deinitialize the library and clean up resources. Calling this functional
 * is optional (if you do not, all cleanup happens at exit anyway). Do not call
 * any libb3dfg functions after calling this function unless you call
 * b3dfg_init() again.
 */
API_EXPORTED void b3dfg_exit(b3dfg_dev *dev)
{

}

// vim: noet
