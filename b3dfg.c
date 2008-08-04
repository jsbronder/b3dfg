 /*
 * Brontes PCI frame grabber driver
 *
 * Copyright (C) 2008 3M Company
 * Contact: Daniel Drake <ddrake@brontes3d.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <linux/version.h>
#include <linux/mutex.h>

#include <asm/atomic.h>
#include <asm/uaccess.h>

/* TODO:
 * locking
 * queue/wait buffer presents filltime results for each frame?
 * counting of dropped frames
 * review endianness
 */

#define DRIVER_NAME "b3dfg"
#define B3DFG_MAX_DEVS 4
#define B3DFG_NR_TRIPLET_BUFFERS 4
#define B3DFG_NR_FRAME_BUFFERS (B3DFG_NR_TRIPLET_BUFFERS * 3)
#define B3DFG_FRAMES_PER_BUFFER 3

#define B3DFG_BAR_REGS	0
#define B3DFG_REGS_LENGTH 0x10000

#define B3DFG_IOC_MAGIC		0xb3 /* dfg :-) */
#define B3DFG_IOCGFRMSZ		_IOR(B3DFG_IOC_MAGIC, 1, int)
#define B3DFG_IOCTNUMBUFS	_IO(B3DFG_IOC_MAGIC, 2)
#define B3DFG_IOCTTRANS		_IO(B3DFG_IOC_MAGIC, 3)
#define B3DFG_IOCTQUEUEBUF	_IO(B3DFG_IOC_MAGIC, 4)
#define B3DFG_IOCTPOLLBUF	_IOWR(B3DFG_IOC_MAGIC, 5, struct b3dfg_poll)
#define B3DFG_IOCTWAITBUF	_IOWR(B3DFG_IOC_MAGIC, 6, struct b3dfg_wait)
#define B3DFG_IOCGWANDSTAT	_IOR(B3DFG_IOC_MAGIC, 7, int)

enum {
	/* number of 4kb pages per frame */
	B3D_REG_FRM_SIZE = 0x0,

	/* bit 0: set to enable interrupts
	 * bit 1: set to enable cable status change interrupts */
	B3D_REG_HW_CTRL = 0x4,

	/* bit 0-1 - 1-based ID of next pending frame transfer (0 = none)
	 * bit 2 indicates the previous DMA transfer has completed
	 * bit 3 indicates wand cable status change
	 * bit 8:15 - counter of number of discarded triplets */
	B3D_REG_DMA_STS = 0x8,

	/* bit 0: wand status (1 = present, 0 = disconnected) */
	B3D_REG_WAND_STS = 0xc,

	/* bus address for DMA transfers. lower 2 bits must be zero because DMA
	 * works with 32 bit word size. */
	B3D_REG_EC220_DMA_ADDR = 0x8000,

	/* bit 20:0 - number of 32 bit words to be transferred
	 * bit 21:31 - reserved */
	B3D_REG_EC220_TRF_SIZE = 0x8004,

	/* bit 0 - error bit
	 * bit 1 - interrupt bit (set to generate interrupt at end of transfer)
	 * bit 2 - start bit (set to start transfer)
	 * bit 3 - direction (0 = DMA_TO_DEVICE, 1 = DMA_FROM_DEVICE
	 * bit 4:31 - reserved */
	B3D_REG_EC220_DMA_STS = 0x8008,
};

enum b3dfg_buffer_state {
	B3DFG_BUFFER_POLLED = 0,
	B3DFG_BUFFER_PENDING,
	B3DFG_BUFFER_POPULATED,
};

struct b3dfg_buffer {
	unsigned char *frame[B3DFG_FRAMES_PER_BUFFER];
	u8 state;
	struct list_head list;
};

struct b3dfg_dev {
	/* no protection needed: all finalized at initialization time */
	struct pci_dev *pdev;
	struct cdev chardev;
	struct class_device *classdev;
	void __iomem *regs;
	unsigned int frame_size;

	/* we want to serialize some ioctl operations */
	struct mutex ioctl_mutex;

	/* preallocated frame buffers */
	unsigned char *frame_buffer[B3DFG_NR_FRAME_BUFFERS];

	/* buffers_lock protects num_buffers, buffers, buffer_queue */
	spinlock_t buffer_lock;
	int num_buffers;
	struct b3dfg_buffer *buffers;
	struct list_head buffer_queue;

	/* cable_state_lock protected cable_state */
	spinlock_t cstate_lock;
	unsigned long cstate_tstamp;

	wait_queue_head_t buffer_waitqueue;

	atomic_t mapping_count;

	spinlock_t triplets_dropped_lock;
	unsigned int triplets_dropped;

	/* FIXME: we need some locking here. this could be accessed in parallel
	 * from the queue_buffer ioctl and the interrupt handler. */
	int cur_dma_frame_idx;
	dma_addr_t cur_dma_frame_addr;

	unsigned int transmission_enabled:1;
	unsigned int triplet_ready:1;
};

static u8 b3dfg_devices[B3DFG_MAX_DEVS];

static struct class *b3dfg_class;
static dev_t b3dfg_devt;

static const struct pci_device_id b3dfg_ids[] __devinitdata = {
	{ PCI_DEVICE(0x0b3d, 0x0001) },

	/* FIXME: remove this ID once all boards have been moved to 0xb3d.
	 * Eureka's vendor ID that we borrowed before we bought our own. */
	{ PCI_DEVICE(0x1901, 0x0001) },
	{ },
};

/***** user-visible types *****/

struct b3dfg_poll {
	int buffer_idx;
	unsigned int triplets_dropped;
};

struct b3dfg_wait {
	int buffer_idx;
	unsigned int timeout;
	unsigned int triplets_dropped;
};

/**** register I/O ****/

static u32 b3dfg_read32(struct b3dfg_dev *fgdev, u16 reg)
{
	return ioread32(fgdev->regs + reg);
}

static void b3dfg_write32(struct b3dfg_dev *fgdev, u16 reg, u32 value)
{
	iowrite32(value, fgdev->regs + reg);
}

/**** buffer management ****/

/* program EC220 for transfer of a specific frame */
static int setup_frame_transfer(struct b3dfg_dev *fgdev,
	struct b3dfg_buffer *buf, int frame, int acknowledge)
{
	unsigned char *frm_addr;
	dma_addr_t frm_addr_dma;
	unsigned int frm_size = fgdev->frame_size;
	unsigned char dma_sts = 0xd;

	frm_addr = buf->frame[frame];
	frm_addr_dma = pci_map_single(fgdev->pdev, frm_addr,
				      frm_size, PCI_DMA_FROMDEVICE);
	if (pci_dma_mapping_error(frm_addr_dma))
		return -ENOMEM;

	fgdev->cur_dma_frame_addr = frm_addr_dma;
	fgdev->cur_dma_frame_idx = frame;

	b3dfg_write32(fgdev, B3D_REG_EC220_DMA_ADDR, cpu_to_le32(frm_addr_dma));
	b3dfg_write32(fgdev, B3D_REG_EC220_TRF_SIZE, cpu_to_le32(frm_size >> 2));

	if (likely(acknowledge))
		dma_sts |= 0x2;
	b3dfg_write32(fgdev, B3D_REG_EC220_DMA_STS, 0xf);

	return 0;
}

/* retrieve a buffer pointer from a buffer index. also checks that the
 * requested buffer actually exists. buffer_lock should be held by caller */
static inline struct b3dfg_buffer *buffer_from_idx(struct b3dfg_dev *fgdev,
	int idx)
{
	if (unlikely(idx < 0 || idx >= fgdev->num_buffers))
		return NULL;
	return &fgdev->buffers[idx];
}

/* caller should hold buffer lock */
static void free_all_buffers(struct b3dfg_dev *fgdev)
{
	kfree(fgdev->buffers);
	fgdev->buffers = NULL;
	fgdev->num_buffers = 0;
}

static void dequeue_all_buffers(struct b3dfg_dev *fgdev)
{
	int i;
	for (i = 0; i < fgdev->num_buffers; i++) {
		struct b3dfg_buffer *buf = &fgdev->buffers[i];
		buf->state = B3DFG_BUFFER_POLLED;
		list_del_init(&buf->list);
	}
}

/* initialize a buffer: allocate its frames, set default values */
static void init_buffer(struct b3dfg_dev *fgdev, struct b3dfg_buffer *buf,
	int idx)
{
	unsigned int addr_offset = idx * B3DFG_FRAMES_PER_BUFFER;
	int i;

	memset(buf, 0, sizeof(struct b3dfg_buffer));
	for (i = 0; i < B3DFG_FRAMES_PER_BUFFER; i++)
		buf->frame[i] = fgdev->frame_buffer[addr_offset + i];

	INIT_LIST_HEAD(&buf->list);
}

/* adjust the number of buffers, growing or shrinking the pool appropriately. */
static int set_num_buffers(struct b3dfg_dev *fgdev, int num_buffers)
{
	int i;
	struct device *dev = &fgdev->pdev->dev;
	struct b3dfg_buffer *buffers;
	unsigned long flags;

	dev_dbg(dev, "set %d buffers\n", num_buffers);
	if (fgdev->transmission_enabled) {
		dev_dbg(dev, "cannot set buffer count, transmission enabled\n");
		return -EBUSY;
	}

	if (atomic_read(&fgdev->mapping_count) > 0) {
		dev_dbg(dev, "cannot set buffer count, memory mapped\n");
		return -EBUSY;
	}

	if (num_buffers > B3DFG_NR_TRIPLET_BUFFERS) {
		dev_dbg(dev, "limited to %d triplet buffers\n",
			B3DFG_NR_TRIPLET_BUFFERS);
		return -E2BIG;
	}

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	if (num_buffers == fgdev->num_buffers) {
		spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
		return 0;
	}

	/* free all buffers then allocate new ones */
	dequeue_all_buffers(fgdev);
	free_all_buffers(fgdev);

	/* must unlock to allocate GFP_KERNEL memory */
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);

	if (num_buffers == 0)
		return 0;

	buffers = kmalloc(num_buffers * sizeof(struct b3dfg_buffer),
		GFP_KERNEL);
	if (!buffers)
		return -ENOMEM;

	for (i = 0; i < num_buffers; i++)
		init_buffer(fgdev, &buffers[i], i);

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	fgdev->buffers = buffers;
	fgdev->num_buffers = num_buffers;
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);

	return 0;
}

/* queue a buffer to receive data */
static int queue_buffer(struct b3dfg_dev *fgdev, int bufidx)
{
	struct device *dev = &fgdev->pdev->dev;
	struct b3dfg_buffer *buf;
	unsigned long flags;
	int r = 0;

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	buf = buffer_from_idx(fgdev, bufidx);
	if (unlikely(!buf)) {
		r = -ENOENT;
		goto out;
	}

	if (unlikely(buf->state == B3DFG_BUFFER_PENDING)) {
		dev_dbg(dev, "buffer %d is already queued\n", bufidx);
		r = -EINVAL;
		goto out;
	}

	buf->state = B3DFG_BUFFER_PENDING;
	list_add_tail(&buf->list, &fgdev->buffer_queue);

	if (fgdev->transmission_enabled && fgdev->triplet_ready) {
		dev_dbg(dev, "triplet is ready, pushing immediately\n");
		fgdev->triplet_ready = 0;
		r = setup_frame_transfer(fgdev, buf, 0, 0);
		if (r)
			dev_err(dev, "unable to map DMA buffer\n");
	}

out:
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
	return r;
}

/* non-blocking buffer poll. returns 1 if data is present in the buffer,
 * 0 otherwise */
static int poll_buffer(struct b3dfg_dev *fgdev, void __user *arg)
{
	struct device *dev = &fgdev->pdev->dev;
	struct b3dfg_poll p;
	struct b3dfg_buffer *buf;
	unsigned long flags;
	int r = 1;

	if (copy_from_user(&p, arg, sizeof(p)))
		return -EFAULT;

	if (unlikely(!fgdev->transmission_enabled)) {
		dev_dbg(dev, "cannot poll, transmission disabled\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	buf = buffer_from_idx(fgdev, p.buffer_idx);
	if (unlikely(!buf)) {
		r = -ENOENT;
		goto out;
	}

	if (likely(buf->state == B3DFG_BUFFER_POPULATED)) {
		buf->state = B3DFG_BUFFER_POLLED;
		spin_lock(&fgdev->triplets_dropped_lock);
		p.triplets_dropped = fgdev->triplets_dropped;
		fgdev->triplets_dropped = 0;
		spin_unlock(&fgdev->triplets_dropped_lock);
		if (copy_to_user(arg, &p, sizeof(p)))
			r = -EFAULT;
	} else {
		r = 0;
	}

out:
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
	return r;
}

static u8 buffer_state(struct b3dfg_dev *fgdev, struct b3dfg_buffer *buf)
{
	unsigned long flags;
	u8 state;

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	state = buf->state;
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
	return state;
}

static unsigned long get_cstate_change(struct b3dfg_dev *fgdev)
{
	unsigned long flags, when;

	spin_lock_irqsave(&fgdev->cstate_lock, flags);
	when = fgdev->cstate_tstamp;
	spin_unlock_irqrestore(&fgdev->cstate_lock, flags);
	return when;
}

static int is_event_ready(struct b3dfg_dev *fgdev, struct b3dfg_buffer *buf,
	unsigned long when)
{
	int result;
	unsigned long flags;

	spin_lock_irqsave(&fgdev->cstate_lock, flags);
	result = (buffer_state(fgdev, buf) == B3DFG_BUFFER_POPULATED ||
		  when != fgdev->cstate_tstamp);
	spin_unlock_irqrestore(&fgdev->cstate_lock, flags);
	return result;
}

/* sleep until a specific buffer becomes populated */
static int wait_buffer(struct b3dfg_dev *fgdev, void __user *arg)
{
	struct device *dev = &fgdev->pdev->dev;
	struct b3dfg_wait w;
	struct b3dfg_buffer *buf;
	unsigned long flags, when;
	int r;

	if (copy_from_user(&w, arg, sizeof(w)))
		return -EFAULT;

	if (unlikely(!fgdev->transmission_enabled)) {
		dev_dbg(dev, "cannot wait, transmission disabled\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	buf = buffer_from_idx(fgdev, w.buffer_idx);
	if (unlikely(!buf)) {
		r = -ENOENT;
		goto out;
	}

	if (buf->state == B3DFG_BUFFER_POPULATED) {
		r = w.timeout;
		goto out_triplets_dropped;
	}

	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
	/* FIXME: what prevents the buffer going away at this time? */

	when = get_cstate_change(fgdev);
	if (w.timeout > 0) {
		r = wait_event_interruptible_timeout(fgdev->buffer_waitqueue,
			is_event_ready(fgdev, buf, when),
			(w.timeout * HZ) / 1000);

		if (unlikely(r < 0))
			return r;

		w.timeout = r * 1000 / HZ;

		/* TODO: Inform the user via field(s) in w? */
		if (when != get_cstate_change(fgdev))
			return -EINVAL;

		if (buffer_state(fgdev, buf) != B3DFG_BUFFER_POPULATED)
			return -ETIMEDOUT;
	} else {
		r = wait_event_interruptible(fgdev->buffer_waitqueue,
			is_event_ready(fgdev, buf, when));
		if (unlikely(r))
			return -ERESTARTSYS;
	}

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	/* FIXME: rediscover buffer? it might have changed during the unlocked
	 * time */
	buf->state = B3DFG_BUFFER_POLLED;

out_triplets_dropped:
	spin_lock(&fgdev->triplets_dropped_lock);
	w.triplets_dropped = fgdev->triplets_dropped;
	fgdev->triplets_dropped = 0;
	spin_unlock(&fgdev->triplets_dropped_lock);
	if (copy_to_user(arg, &w, sizeof(w)))
		r = -EFAULT;
out:
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
	return r;
}

/**** virtual memory mapping ****/

static void b3dfg_vma_open(struct vm_area_struct *vma)
{
	struct b3dfg_dev *fgdev = vma->vm_file->private_data;
	atomic_inc(&fgdev->mapping_count);
}

static void b3dfg_vma_close(struct vm_area_struct *vma)
{
	struct b3dfg_dev *fgdev = vma->vm_file->private_data;
	atomic_dec(&fgdev->mapping_count);
}

/* page fault handler */
static unsigned long b3dfg_vma_nopfn(struct vm_area_struct *vma,
	unsigned long address)
{
	struct b3dfg_dev *fgdev = vma->vm_file->private_data;
	unsigned long off = address - vma->vm_start;
	unsigned int frame_size = fgdev->frame_size;
	unsigned int buf_size = frame_size * B3DFG_FRAMES_PER_BUFFER;
	unsigned long flags;
	unsigned char *addr;

	/* determine which buffer the offset lies within */
	unsigned int buf_idx = off / buf_size;
	/* and the offset into the buffer */
	unsigned int buf_off = off % buf_size;

	/* determine which frame inside the buffer the offset lies in */
	unsigned int frm_idx = buf_off / frame_size;
	/* and the offset into the frame */
	unsigned int frm_off = buf_off % frame_size;

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	if (unlikely(buf_idx > fgdev->num_buffers)) {
		spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
		return NOPFN_SIGBUS;
	}

	addr = fgdev->buffers[buf_idx].frame[frm_idx] + frm_off;
	vm_insert_pfn(vma, vma->vm_start + off,
		virt_to_phys(addr) >> PAGE_SHIFT);
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
	return NOPFN_REFAULT;
}

static struct vm_operations_struct b3dfg_vm_ops = {
	.open = b3dfg_vma_open,
	.close = b3dfg_vma_close,
	.nopfn = b3dfg_vma_nopfn,
};

static int get_wand_status(struct b3dfg_dev *fgdev, int __user *arg)
{
	u32 wndstat = b3dfg_read32(fgdev, B3D_REG_WAND_STS);
	dev_dbg(&fgdev->pdev->dev, "wand status %x\n", wndstat);
	return __put_user(wndstat & 0x1, arg);
}

static int enable_transmission(struct b3dfg_dev *fgdev)
{
	u16 command;
	unsigned long flags;
	struct device *dev = &fgdev->pdev->dev;

	dev_dbg(dev, "enable transmission\n");

	/* check the cable is plugged in. */
	if (!b3dfg_read32(fgdev, B3D_REG_WAND_STS)) {
		dev_dbg(dev, "cannot start transmission without wand\n");
		return -EINVAL;
	}

	/*
	 * Check we're a bus master.
	 * TODO: I think we can remove this having added the pci_set_master call
	 */
	pci_read_config_word(fgdev->pdev, PCI_COMMAND, &command);
	if (!(command & PCI_COMMAND_MASTER)) {
		dev_err(dev, "not a bus master, force-enabling\n");
		pci_write_config_word(fgdev->pdev, PCI_COMMAND,
			command | PCI_COMMAND_MASTER);
	}

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	if (fgdev->num_buffers == 0) {
		spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
		dev_dbg(dev, "cannot start transmission to 0 buffers\n");
		return -EINVAL;
	}
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);

	spin_lock_irqsave(&fgdev->triplets_dropped_lock, flags);
	fgdev->triplets_dropped = 0;
	spin_unlock_irqrestore(&fgdev->triplets_dropped_lock, flags);

	fgdev->triplet_ready = 0;
	fgdev->transmission_enabled = 1;
	fgdev->cur_dma_frame_idx = -1;

	/* Enable DMA and cable status interrupts. */
	b3dfg_write32(fgdev, B3D_REG_HW_CTRL, 0x03);

	return 0;
}

static void disable_transmission(struct b3dfg_dev *fgdev)
{
	struct device *dev = &fgdev->pdev->dev;
	unsigned long flags;
	u32 tmp;

	dev_dbg(dev, "disable transmission\n");

	/* guarantee that no more interrupts will be serviced */
	fgdev->transmission_enabled = 0;
	synchronize_irq(fgdev->pdev->irq);

	b3dfg_write32(fgdev, B3D_REG_HW_CTRL, 0);

	/* FIXME: temporary debugging only. if the board stops transmitting,
	 * hitting ctrl+c and seeing this message is useful for determining
	 * the state of the board. */
	tmp = b3dfg_read32(fgdev, B3D_REG_DMA_STS);
	dev_dbg(dev, "DMA_STS reads %x after TX stopped\n", tmp);

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	dequeue_all_buffers(fgdev);
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
}

static int set_transmission(struct b3dfg_dev *fgdev, int enabled)
{
	if (enabled && !fgdev->transmission_enabled)
		return enable_transmission(fgdev);
	else if (!enabled && fgdev->transmission_enabled)
		disable_transmission(fgdev);
	return 0;
}

static void handle_cstate_unplug(struct b3dfg_dev *fgdev)
{
	/* Disable all interrupts. */
	b3dfg_write32(fgdev, B3D_REG_HW_CTRL, 0);

	/* Stop transmission. */
	fgdev->triplet_ready = 0;
	fgdev->transmission_enabled = 0;
	fgdev->cur_dma_frame_idx = -1;
	if (fgdev->cur_dma_frame_addr) {
		pci_unmap_single(fgdev->pdev, fgdev->cur_dma_frame_addr,
				 fgdev->frame_size, PCI_DMA_FROMDEVICE);
		fgdev->cur_dma_frame_addr = 0;
	}

	/* Dequeue all buffers. */
	spin_lock(&fgdev->buffer_lock);
	dequeue_all_buffers(fgdev);
	spin_unlock(&fgdev->buffer_lock);
}


static void handle_cstate_change(struct b3dfg_dev *fgdev)
{
	u32 cstate = b3dfg_read32(fgdev, B3D_REG_WAND_STS);
	unsigned long when;
	struct device *dev = &fgdev->pdev->dev;

	dev_dbg(dev, "cable state change: %u\n", cstate);

	/*
	 * When the wand is unplugged we reset our state. The hardware will
	 * have done the same internally.
	 *
	 * Note we should never see a cable *plugged* event, as interrupts
	 * should only be enabled when transmitting, which requires the cable
	 * to be plugged. If we do see one it probably means the cable has been
	 * unplugged and re-plugged very rapidly. Possibly because it has a
	 * broken wire and is momentarily losing contact.
	 *
	 * TODO: At the moment if you plug in the cable then enable transmission
	 *       the hardware will raise a couple of spurious interrupts, so
	 *       just ignore them for now.
	 *
	 *      Once the hardware is fixed we should complain and treat it as an
	 *      unplug. Or at least track how frequently it is happening and do
	 *      so if too many come in.
	 */
	if (cstate) {
		dev_warn(dev, "ignoring unexpected plug event\n");
		return;
	}
	handle_cstate_unplug(fgdev);

	/*
	 * Record cable state change timestamp & wake anyone waiting
	 * on a cable state change. Be paranoid about ensuring events
	 * are not missed if we somehow get two interrupts in a jiffy.
	 */
	spin_lock(&fgdev->cstate_lock);
	when = jiffies_64;
	if (when <= fgdev->cstate_tstamp)
		when = fgdev->cstate_tstamp + 1;
	fgdev->cstate_tstamp = when;
	wake_up_interruptible(&fgdev->buffer_waitqueue);
	spin_unlock(&fgdev->cstate_lock);
}

static irqreturn_t b3dfg_intr(int irq, void *dev_id)
{
	struct b3dfg_dev *fgdev = dev_id;
	struct device *dev = &fgdev->pdev->dev;
	struct b3dfg_buffer *buf = NULL;
	unsigned int frame_size;
	u32 sts;
	u8 dropped;
	int next_trf;
	int need_ack = 1;

	sts = b3dfg_read32(fgdev, B3D_REG_DMA_STS);
	if (unlikely(sts == 0)) {
		dev_warn(dev, "ignore interrupt, DMA status is 0\n");
		/* FIXME should return IRQ_NONE when we are stable */
		goto out;
	}

	if (unlikely(!fgdev->transmission_enabled)) {
		dev_warn(dev, "ignore interrupt, TX disabled\n");
		/* FIXME should return IRQ_NONE when we are stable */
		goto out;
	}

	dropped = (sts >> 8) & 0xff;
	dev_dbg(dev, "intr: DMA_STS=%08x (drop=%d comp=%d next=%d)\n",
		sts, dropped, !!(sts & 0x4), sts & 0x3);

	if (unlikely(dropped > 0)) {
		spin_lock(&fgdev->triplets_dropped_lock);
		fgdev->triplets_dropped += dropped;
		spin_unlock(&fgdev->triplets_dropped_lock);
	}

	/* Handle a cable state change (i.e. the wand being unplugged). */
	if (sts & 0x08) {
		handle_cstate_change(fgdev);
		goto out;
	}

	frame_size = fgdev->frame_size;

	spin_lock(&fgdev->buffer_lock);
	if (unlikely(list_empty(&fgdev->buffer_queue))) {
		/* FIXME need more sanity checking here */
		dev_info(dev, "buffer not ready for next transfer\n");
		fgdev->triplet_ready = 1;
		goto out_unlock;
	}

	next_trf = sts & 0x3;

	if (sts & 0x4) {
		u32 tmp;

		tmp = b3dfg_read32(fgdev, B3D_REG_EC220_DMA_STS);
		/* last DMA completed */
		if (unlikely(tmp & 0x1)) {
			dev_err(dev, "EC220 error: %08x\n", tmp);
			/* FIXME flesh out error handling */
			goto out_unlock;
		}
		if (unlikely(fgdev->cur_dma_frame_idx == -1)) {
			dev_err(dev, "completed but no last idx?\n");
			/* FIXME flesh out error handling */
			goto out_unlock;
		}
		pci_unmap_single(fgdev->pdev, fgdev->cur_dma_frame_addr,
				 frame_size, PCI_DMA_FROMDEVICE);
		fgdev->cur_dma_frame_addr = 0;

		buf = list_entry(fgdev->buffer_queue.next,
				 struct b3dfg_buffer, list);
		if (likely(buf)) {
			dev_dbg(dev, "handle frame completion\n");
			if (fgdev->cur_dma_frame_idx == B3DFG_FRAMES_PER_BUFFER - 1) {
				/* last frame of that triplet completed */
				dev_dbg(dev, "triplet completed\n");
				buf->state = B3DFG_BUFFER_POPULATED;
				list_del_init(&buf->list);
				wake_up_interruptible(&fgdev->buffer_waitqueue);
			}
		} else {
			dev_err(dev, "got frame but no buffer!\n");
		}
	}

	if (next_trf) {
		next_trf--;

		buf = list_entry(fgdev->buffer_queue.next,
				 struct b3dfg_buffer, list);
		dev_dbg(dev, "program DMA transfer for frame %d\n",
			next_trf + 1);
		if (likely(buf)) {
			if (next_trf != fgdev->cur_dma_frame_idx + 1) {
				dev_err(dev,
					"frame mismatch, got %d, expected %d\n",
					next_trf, fgdev->cur_dma_frame_idx);
				/* FIXME handle dropped triplets here */
				goto out_unlock;
			}
			if (setup_frame_transfer(fgdev, buf, next_trf, 1))
				dev_err(dev, "unable to map DMA buffer\n");
			need_ack = 0;
		} else {
			dev_err(dev, "cannot setup DMA, no buffer\n");
		}
	} else {
		fgdev->cur_dma_frame_idx = -1;
	}

out_unlock:
	spin_unlock(&fgdev->buffer_lock);
out:
	if (need_ack) {
		dev_dbg(dev, "acknowledging interrupt\n");
		b3dfg_write32(fgdev, B3D_REG_EC220_DMA_STS, 0x0b);
	}
	return IRQ_HANDLED;
}

static int b3dfg_open(struct inode *inode, struct file *filp)
{
	struct b3dfg_dev *fgdev =
		container_of(inode->i_cdev, struct b3dfg_dev, chardev);

	dev_dbg(&fgdev->pdev->dev, "open\n");
	filp->private_data = fgdev;
	return 0;
}

static int b3dfg_release(struct inode *inode, struct file *filp)
{
	struct b3dfg_dev *fgdev = filp->private_data;
	dev_dbg(&fgdev->pdev->dev, "release\n");
	set_transmission(fgdev, 0);

	/* no buffer locking needed, this is serialized */
	dequeue_all_buffers(fgdev);
	return set_num_buffers(fgdev, 0);
}

static long b3dfg_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct b3dfg_dev *fgdev = filp->private_data;
	int r;

	switch (cmd) {
	case B3DFG_IOCGFRMSZ:
		return __put_user(fgdev->frame_size, (int __user *) arg);
	case B3DFG_IOCGWANDSTAT:
		return get_wand_status(fgdev, (int __user *) arg);
	case B3DFG_IOCTNUMBUFS:
		mutex_lock(&fgdev->ioctl_mutex);
		r = set_num_buffers(fgdev, (int) arg);
		mutex_unlock(&fgdev->ioctl_mutex);
		return r;
	case B3DFG_IOCTTRANS:
		mutex_lock(&fgdev->ioctl_mutex);
		r = set_transmission(fgdev, (int) arg);
		mutex_unlock(&fgdev->ioctl_mutex);
		return r;
	case B3DFG_IOCTQUEUEBUF:
		return queue_buffer(fgdev, (int) arg);
	case B3DFG_IOCTPOLLBUF:
		return poll_buffer(fgdev, (void __user *) arg);
	case B3DFG_IOCTWAITBUF:
		return wait_buffer(fgdev, (void __user *) arg);
	default:
		dev_dbg(&fgdev->pdev->dev, "unrecognised ioctl %x\n", cmd);
		return -EINVAL;
	}
}

static unsigned int b3dfg_poll(struct file *filp, poll_table *poll_table)
{
	struct b3dfg_dev *fgdev = filp->private_data;
	struct device *dev = &fgdev->pdev->dev;
	unsigned long flags, when;
	int i;
	int r = 0;

	/* don't let the user mess with buffer allocations etc. while polling */
	mutex_lock(&fgdev->ioctl_mutex);

	if (unlikely(!fgdev->transmission_enabled)) {
		dev_dbg(dev, "cannot poll, transmission disabled\n");
		r = POLLERR;
		goto out;
	}

	when = get_cstate_change(fgdev);
	poll_wait(filp, &fgdev->buffer_waitqueue, poll_table);
	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	for (i = 0; i < fgdev->num_buffers; i++) {
		if (fgdev->buffers[i].state == B3DFG_BUFFER_POPULATED) {
			r = POLLIN | POLLRDNORM;
			goto out_buffer_unlock;
		}
	}

out_buffer_unlock:
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);

	/* TODO: Confirm this is how we want to communicate the change. */
	if (when != get_cstate_change(fgdev))
		r = POLLERR;

out:
	mutex_unlock(&fgdev->ioctl_mutex);
	return r;
}

static int b3dfg_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct b3dfg_dev *fgdev = filp->private_data;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long bufdatalen;
	unsigned long psize;
	unsigned long flags;
	int r = 0;

	/* don't let user mess with buffer allocations during mmap */
	mutex_lock(&fgdev->ioctl_mutex);

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	bufdatalen = fgdev->num_buffers * fgdev->frame_size * 3;
	psize = bufdatalen - offset;

	if (fgdev->num_buffers == 0) {
		spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
		r = -ENOENT;
		goto out;
	}
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
	if (vsize > psize) {
		r = -EINVAL;
		goto out;
	}

	vma->vm_flags |= VM_IO | VM_RESERVED | VM_CAN_NONLINEAR | VM_PFNMAP;
	vma->vm_ops = &b3dfg_vm_ops;
	b3dfg_vma_open(vma);

out:
	mutex_unlock(&fgdev->ioctl_mutex);
	return r;
}

static struct file_operations b3dfg_fops = {
	.owner = THIS_MODULE,
	.open = b3dfg_open,
	.release = b3dfg_release,
	.unlocked_ioctl = b3dfg_ioctl,
	.poll = b3dfg_poll,
	.mmap = b3dfg_mmap,
};

static void free_all_frame_buffers(struct b3dfg_dev *fgdev)
{
	int i;
	for (i = 0; i < B3DFG_NR_FRAME_BUFFERS; i++)
		kfree(fgdev->frame_buffer[i]);
}

/* initialize device and any data structures. called before any interrupts
 * are enabled. */
static int b3dfg_init_dev(struct b3dfg_dev *fgdev)
{
	int i;
	u32 frm_size = b3dfg_read32(fgdev, B3D_REG_FRM_SIZE);

	/* Disable interrupts. In abnormal circumstances (e.g. after a crash)
	 * the board may still be transmitting from the previous session. If we
	 * ensure that interrupts are disabled before we later enable them, we
	 * are sure to capture a triplet from the start, rather than starting
	 * from frame 2 or 3. Disabling interrupts causes the FG to throw away
	 * all buffered data and stop buffering more until interrupts are
	 * enabled again.
	 */
	b3dfg_write32(fgdev, B3D_REG_HW_CTRL, 0);

	fgdev->frame_size = frm_size * 4096;
	for (i = 0; i < B3DFG_NR_FRAME_BUFFERS; i++) {
		fgdev->frame_buffer[i] = kmalloc(fgdev->frame_size, GFP_KERNEL);
		if (!fgdev->frame_buffer[i])
			goto err_no_mem;
	}

	INIT_LIST_HEAD(&fgdev->buffer_queue);
	init_waitqueue_head(&fgdev->buffer_waitqueue);
	spin_lock_init(&fgdev->buffer_lock);
	spin_lock_init(&fgdev->cstate_lock);
	spin_lock_init(&fgdev->triplets_dropped_lock);
	atomic_set(&fgdev->mapping_count, 0);
	mutex_init(&fgdev->ioctl_mutex);
	return 0;

err_no_mem:
	free_all_frame_buffers(fgdev);
	return -ENOMEM;
}

/* find next free minor number, returns -1 if none are availabile */
static int get_free_minor(void)
{
	int i;
	for (i = 0; i < B3DFG_MAX_DEVS; i++) {
		if (b3dfg_devices[i] == 0)
			return i;
	}
	return -1;
}

static int __devinit b3dfg_probe(struct pci_dev *pdev,
	const struct pci_device_id *id)
{
	struct b3dfg_dev *fgdev = kzalloc(sizeof(*fgdev), GFP_KERNEL);
	int r = 0;
	int minor = get_free_minor();
	dev_t devno = MKDEV(MAJOR(b3dfg_devt), minor);
	unsigned long res_len;
	resource_size_t res_base;

	if (fgdev == NULL)
		return -ENOMEM;

	if (minor < 0) {
		dev_err(&pdev->dev, "too many devices found!\n");
		r = -EIO;
		goto err_free;
	}

	b3dfg_devices[minor] = 1;
	dev_info(&pdev->dev, "probe device with IRQ %d\n", pdev->irq);

	cdev_init(&fgdev->chardev, &b3dfg_fops);
	fgdev->chardev.owner = THIS_MODULE;

	r = cdev_add(&fgdev->chardev, devno, 1);
	if (r) {
		dev_err(&pdev->dev, "cannot add char device\n");
		goto err_release_minor;
	}

	fgdev->classdev = class_device_create(b3dfg_class, NULL, devno,
					      &pdev->dev, DRIVER_NAME "%d",
					      minor);
	if (IS_ERR(fgdev->classdev)) {
		dev_err(&pdev->dev, "cannot create class device\n");
		r = PTR_ERR(fgdev->classdev);
		goto err_del_cdev;
	}

	r = pci_enable_device(pdev);
	if (r) {
		dev_err(&pdev->dev, "cannot enable PCI device\n");
		goto err_dev_unreg;
	}

	res_len = pci_resource_len(pdev, B3DFG_BAR_REGS);
	if (res_len != B3DFG_REGS_LENGTH) {
		dev_err(&pdev->dev, "invalid register resource size\n");
		r = -EIO;
		goto err_disable;
	}

	if (pci_resource_flags(pdev, B3DFG_BAR_REGS) != IORESOURCE_MEM) {
		dev_err(&pdev->dev, "invalid resource flags\n");
		r = -EIO;
		goto err_disable;
	}

	r = pci_request_regions(pdev, DRIVER_NAME);
	if (r) {
		dev_err(&pdev->dev, "cannot obtain PCI resources\n");
		goto err_disable;
	}

	pci_set_master(pdev);

	res_base = pci_resource_start(pdev, B3DFG_BAR_REGS);
	fgdev->regs = ioremap_nocache(res_base, res_len);
	if (!fgdev->regs) {
		dev_err(&pdev->dev, "regs ioremap failed\n");
		r = -EIO;
		goto err_free_res;
	}

	fgdev->pdev = pdev;
	pci_set_drvdata(pdev, fgdev);
	r = b3dfg_init_dev(fgdev);
	if (r < 0) {
		dev_err(&pdev->dev, "failed to initalize device\n");
		goto err_unmap;
	}

	r = request_irq(pdev->irq, b3dfg_intr, IRQF_SHARED, DRIVER_NAME, fgdev);
	if (r) {
		dev_err(&pdev->dev, "couldn't request irq %d\n", pdev->irq);
		goto err_free_bufs;
	}

	return 0;

err_free_bufs:
	free_all_frame_buffers(fgdev);
err_unmap:
	iounmap(fgdev->regs);
err_free_res:
	pci_release_regions(pdev);
err_disable:
	pci_disable_device(pdev);
err_dev_unreg:
	class_device_unregister(fgdev->classdev);
err_del_cdev:
	cdev_del(&fgdev->chardev);
err_release_minor:
	b3dfg_devices[minor] = 0;
err_free:
	kfree(fgdev);
	return r;
}

static void __devexit b3dfg_remove(struct pci_dev *pdev)
{
	struct b3dfg_dev *fgdev = pci_get_drvdata(pdev);
	unsigned int minor = MINOR(fgdev->chardev.dev);

	dev_dbg(&pdev->dev, "remove\n");

	free_irq(pdev->irq, fgdev);
	iounmap(fgdev->regs);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	class_device_unregister(fgdev->classdev);
	cdev_del(&fgdev->chardev);
	free_all_frame_buffers(fgdev);
	kfree(fgdev);
	b3dfg_devices[minor] = 0;
}

static struct pci_driver b3dfg_driver = {
	.name = DRIVER_NAME,
	.id_table = b3dfg_ids,
	.probe = b3dfg_probe,
	.remove = __devexit_p(b3dfg_remove),
};

static int __init b3dfg_module_init(void)
{
	int r;

	printk(KERN_INFO DRIVER_NAME ": loaded\n");

	b3dfg_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(b3dfg_class))
		return PTR_ERR(b3dfg_class);

	r = alloc_chrdev_region(&b3dfg_devt, 0, B3DFG_MAX_DEVS, DRIVER_NAME);
	if (r)
		goto err1;

	r = pci_register_driver(&b3dfg_driver);
	if (r)
		goto err2;

	return r;

err2:
	unregister_chrdev_region(b3dfg_devt, B3DFG_MAX_DEVS);
err1:
	class_destroy(b3dfg_class);
	return r;
}

static void __exit b3dfg_module_exit(void)
{
	printk(KERN_INFO DRIVER_NAME ": unloaded\n");
	pci_unregister_driver(&b3dfg_driver);
	unregister_chrdev_region(b3dfg_devt, B3DFG_MAX_DEVS);
	class_destroy(b3dfg_class);
}

module_init(b3dfg_module_init);
module_exit(b3dfg_module_exit);
MODULE_AUTHOR("Daniel Drake <ddrake@brontes3d.com>");
MODULE_DESCRIPTION("Brontes frame grabber driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(pci, b3dfg_ids);

