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

#include <asm/uaccess.h>

/* TODO:
 * locking
 * queue/wait buffer presents filltime results for each frame?
 * counting of dropped frames
 * review endianness
 */

/* pre-2.6.23-compat */
#ifndef VM_CAN_NONLINEAR
#define VM_CAN_NONLINEAR 0
#endif

#define DRIVER_NAME "b3dfg"
#define PFX DRIVER_NAME ": "
#define B3DFG_MAX_DEVS 4
#define B3DFG_FRAMES_PER_BUFFER 3

#define B3DFG_BAR_REGS	0
#define B3DFG_REGS_LENGTH 0x10000

#define B3DFG_IOC_MAGIC			0xb3 /* dfg :-) */
#define B3DFG_IOCGFRMSZ			_IOR(B3DFG_IOC_MAGIC, 1, int)
#define B3DFG_IOCTNUMBUFS		_IO(B3DFG_IOC_MAGIC, 2)
#define B3DFG_IOCTTRANS			_IO(B3DFG_IOC_MAGIC, 3)
#define B3DFG_IOCTQUEUEBUF		_IO(B3DFG_IOC_MAGIC, 4)
#define B3DFG_IOCQPOLLBUF		_IO(B3DFG_IOC_MAGIC, 5)
#define B3DFG_IOCQWAITBUF		_IO(B3DFG_IOC_MAGIC, 6)

enum {
	/* number of 4kb pages per frame */
	B3D_REG_FRM_SIZE = 0x0,

	/* bit 0: set to enable interrupts */
	B3D_REG_HW_CTRL = 0x4,

	/* bit 0-1 - 1-based ID of next pending frame transfer (0 = nothing pending)
	 * bit 2 indicates the previous DMA transfer has completed
	 * bit 8:15 - counter of number of discarded triplets */
	B3D_REG_DMA_STS = 0x8,

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

#define B3DFG_BUFFER_STATUS_QUEUED			(1<<0)
#define B3DFG_BUFFER_STATUS_POPULATED		(1<<1)

enum b3dfg_buffer_state {
	B3DFG_BUFFER_POLLED = 0,
	B3DFG_BUFFER_PENDING,
	B3DFG_BUFFER_POPULATED,
};

struct b3dfg_buffer {
	unsigned char *frame[B3DFG_FRAMES_PER_BUFFER];
	u8 state;
	u8 nr_populated_frames;
	struct list_head list;
};

struct b3dfg_dev {
	struct pci_dev *pdev;
    struct cdev chardev;
    struct class_device *classdev;
	void __iomem *regs;

	unsigned int frame_size;
	
	int num_buffers;
	struct b3dfg_buffer *buffers;
	struct list_head buffer_queue;
	wait_queue_head_t buffer_waitqueue;

	int mapping_count;

	spinlock_t irq_lock;
	int transmission_enabled;
	int cur_dma_frame_idx;
	dma_addr_t cur_dma_frame_addr;
};

static u8 b3dfg_devices[B3DFG_MAX_DEVS];

static struct class *b3dfg_class;
static dev_t b3dfg_devt;

static const struct pci_device_id b3dfg_ids[] __devinitdata = {
	{ PCI_DEVICE(0x1901, 0x0001) },
	{ },
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

/* retrieve a buffer pointer from a buffer index. also checks that the
 * requested buffer actually exists. */
static inline struct b3dfg_buffer *buffer_from_idx(struct b3dfg_dev *fgdev,
	int idx)
{
	if (unlikely(idx >= fgdev->num_buffers))
		return NULL;
	return &fgdev->buffers[idx];
}

/* free the frames in a buffer */
static void free_buffer(struct b3dfg_buffer *buf)
{
	int i;
	for (i = 0; i < B3DFG_FRAMES_PER_BUFFER; i++)
		kfree(buf->frame[i]);
}

static void free_all_buffers(struct b3dfg_dev *fgdev)
{
	int i;
	for (i = 0; i < fgdev->num_buffers; i++)
		free_buffer(&fgdev->buffers[i]);

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
static int init_buffer(struct b3dfg_dev *fgdev, struct b3dfg_buffer *buf)
{
	unsigned int frame_size = fgdev->frame_size;
	int i;

	memset(buf, 0, sizeof(struct b3dfg_buffer));
	for (i = 0; i < B3DFG_FRAMES_PER_BUFFER; i++) {
		buf->frame[i] = kmalloc(frame_size, GFP_KERNEL);
		if (!buf->frame[i]) {
			printk(KERN_ERR PFX "frame allocation failed\n");
			goto err;
		}
	}

	INIT_LIST_HEAD(&buf->list);
	return 0;

err:
	free_buffer(buf);
	return -ENOMEM;
}

/* adjust the number of buffers, growing or shrinking the pool appropriately. */
static int set_num_buffers(struct b3dfg_dev *fgdev, int num_buffers)
{
	int i;
	int r;
	struct b3dfg_buffer *newbufs;

	printk(KERN_INFO PFX "set %d buffers\n", num_buffers);
	if (fgdev->transmission_enabled) {
		printk(KERN_ERR PFX
			"cannot set buffer count while transmission is enabled\n");
		return -EBUSY;
	}

	if (fgdev->mapping_count > 0) {
		printk(KERN_ERR PFX
			"cannot set buffer count while memory mappings are active\n");
		return -EBUSY;
	}

	if (!fgdev->buffers) {
		/* no buffers allocated yet */
		fgdev->buffers = kmalloc(num_buffers * sizeof(struct b3dfg_buffer),
			GFP_KERNEL);
		if (!fgdev->buffers)
			return -ENOMEM;

		for (i = 0; i < num_buffers; i++) {
			r = init_buffer(fgdev, &fgdev->buffers[i]);
			if (r)
				goto err;
		}
	} else if (num_buffers == 0) {
		free_all_buffers(fgdev);
	} else if (fgdev->num_buffers < num_buffers) {
		/* app requested more buffers than we currently have allocated */
		newbufs = krealloc(fgdev->buffers,
			num_buffers * sizeof(struct b3dfg_buffer), GFP_KERNEL);
		if (!newbufs) {
			r = -ENOMEM;
			goto err;
		}

		for (i = fgdev->num_buffers; i < num_buffers; i++) {
			r = init_buffer(fgdev, &newbufs[i]);
			if (r)
				goto err;
		}
		fgdev->buffers = newbufs;
	} else if (fgdev->num_buffers > num_buffers) {
		/* app requests a decrease in buffers */
		for (i = num_buffers; i < fgdev->num_buffers; i++)
			free_buffer(&fgdev->buffers[i]);

		newbufs = krealloc(fgdev->buffers,
			num_buffers * sizeof(struct b3dfg_buffer), GFP_KERNEL);
		if (!newbufs) {
			r = -ENOMEM;
			goto err;
		}

		fgdev->buffers = newbufs;
	}
	fgdev->num_buffers = num_buffers;
	return 0;

err:
	free_all_buffers(fgdev);
	return r;
}

/* queue a buffer to receive data */
static int queue_buffer(struct b3dfg_dev *fgdev, int bufidx)
{
	struct b3dfg_buffer *buf = buffer_from_idx(fgdev, bufidx);
	if (unlikely(!buf))
		return -ENOENT;

	if (unlikely(buf->state == B3DFG_BUFFER_PENDING)) {
		printk(KERN_ERR PFX "buffer %d is already queued", bufidx);
		return -EINVAL;
	}

	buf->nr_populated_frames = 0;
	buf->state = B3DFG_BUFFER_PENDING;
	list_add_tail(&buf->list, &fgdev->buffer_queue);
	return 0;
}

/* non-blocking buffer poll. returns 1 if data is present in the buffer,
 * 0 otherwise */
static int poll_buffer(struct b3dfg_dev *fgdev, int bufidx)
{
	struct b3dfg_buffer *buf = buffer_from_idx(fgdev, bufidx);
	if (unlikely(!buf))
		return -ENOENT;

	if (unlikely(!fgdev->transmission_enabled)) {
		printk(KERN_ERR PFX
			"cannot poll buffers when transmission is disabled\n");
		return -EINVAL;
	}

	if (buf->state != B3DFG_BUFFER_POPULATED)
		return 0;

	if (likely(buf->state == B3DFG_BUFFER_POPULATED))
		buf->state = B3DFG_BUFFER_POLLED;

	return 1;
}

/* sleep until a specific buffer becomes populated */
static int wait_buffer(struct b3dfg_dev *fgdev, int bufidx)
{
	struct b3dfg_buffer *buf = buffer_from_idx(fgdev, bufidx);
	int r;

	if (unlikely(!buf))
		return -ENOENT;

	if (unlikely(!fgdev->transmission_enabled)) {
		printk(KERN_ERR PFX
			"cannot wait on buffers when transmission is disabled\n");
		return -EINVAL;
	}

	if (buf->state == B3DFG_BUFFER_STATUS_POPULATED)
		return 0;

	r = wait_event_interruptible(fgdev->buffer_waitqueue,
		buf->state == B3DFG_BUFFER_POPULATED);
	if (unlikely(r))
		return -ERESTARTSYS;

	buf->state = B3DFG_BUFFER_POLLED;
	return 0;
}

/**** virtual memory mapping ****/

static void b3dfg_vma_open(struct vm_area_struct *vma)
{
	struct b3dfg_dev *fgdev = vma->vm_file->private_data;
	fgdev->mapping_count++;
}

static void b3dfg_vma_close(struct vm_area_struct *vma)
{
	struct b3dfg_dev *fgdev = vma->vm_file->private_data;
	fgdev->mapping_count--;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22)
/* page fault handler */
static int b3dfg_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct b3dfg_dev *fgdev = vma->vm_file->private_data;
	unsigned long off = vmf->pgoff << PAGE_SHIFT;
	unsigned int frame_size = fgdev->frame_size;
	unsigned int buf_size = frame_size * B3DFG_FRAMES_PER_BUFFER;
	struct page *page;

	/* determine which buffer the offset lies within */
	unsigned int buf_idx = off / buf_size;
	/* and the offset into the buffer */
	unsigned int buf_off = off % buf_size;

	/* determine which frame inside the buffer the offset lies in */
	unsigned int frm_idx = buf_off / frame_size;
	/* and the offset into the frame */
	unsigned int frm_off = buf_off % frame_size;

	if (unlikely(buf_idx > fgdev->num_buffers))
		return VM_FAULT_SIGBUS;

	page = virt_to_page(fgdev->buffers[buf_idx].frame[frm_idx] + frm_off);
	get_page(page);
	vmf->page = page;
	return 0;
}
#else
/* page fault handler */
static struct page *b3dfg_vma_nopage(struct vm_area_struct *vma,
	unsigned long address, int *type)
{
	struct b3dfg_dev *fgdev = vma->vm_file->private_data;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned int frame_size = fgdev->frame_size;
	unsigned int buf_size = frame_size * B3DFG_FRAMES_PER_BUFFER;
	struct page *page;

	/* determine which buffer the offset lies within */
	unsigned int buf_idx = off / buf_size;
	/* and the offset into the buffer */
	unsigned int buf_off = off % buf_size;

	/* determine which frame inside the buffer the offset lies in */
	unsigned int frm_idx = buf_off / frame_size;
	/* and the offset into the frame */
	unsigned int frm_off = buf_off % frame_size;

	if (unlikely(buf_idx > fgdev->num_buffers))
		return NOPAGE_SIGBUS;

	page = virt_to_page(fgdev->buffers[buf_idx].frame[frm_idx] + frm_off);
	get_page(page);
	if (*type)
		*type = VM_FAULT_MINOR;
	return page;
}
#endif

static struct vm_operations_struct b3dfg_vm_ops = {
	.open = b3dfg_vma_open,
	.close = b3dfg_vma_close,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22)
	.fault = b3dfg_vma_fault,
#else
	.nopage = b3dfg_vma_nopage,
#endif
};

static int enable_transmission(struct b3dfg_dev *fgdev)
{
	u16 command;
	printk(KERN_INFO PFX "enable transmission\n");

	/* check we're a bus master */
	pci_read_config_word(fgdev->pdev, PCI_COMMAND, &command);
	if (!(command & PCI_COMMAND_MASTER)) {
		printk(KERN_ERR PFX "cannot transmit; am not a bus master\n");
		return -EIO;
	}

	if (fgdev->num_buffers == 0) {
		printk(KERN_ERR PFX "cannot start transmission to 0 buffers\n");
		return -EINVAL;
	}

	fgdev->transmission_enabled = 1;
	b3dfg_write32(fgdev, B3D_REG_HW_CTRL, 1);
	return 0;
}

static void disable_transmission(struct b3dfg_dev *fgdev)
{
	unsigned long flags;
	u32 tmp;

	printk(KERN_INFO PFX "disable transmission\n");

	/* guarantee that no more interrupts will be serviced */
	spin_lock_irqsave(&fgdev->irq_lock, flags);
	fgdev->transmission_enabled = 0;
	spin_unlock_irqrestore(&fgdev->irq_lock, flags);

	b3dfg_write32(fgdev, B3D_REG_HW_CTRL, 0);

	/* reset dropped triplet counter */
	/* FIXME will this throw away useful dma data too? */
	tmp = b3dfg_read32(fgdev, B3D_REG_DMA_STS);
	printk("brontes DMA_STS reads %x after TX stopped\n", tmp);

	dequeue_all_buffers(fgdev);
}

static int set_transmission(struct b3dfg_dev *fgdev, int enabled)
{
	if (enabled && !fgdev->transmission_enabled)
		return enable_transmission(fgdev);
	else if (!enabled && fgdev->transmission_enabled)
		disable_transmission(fgdev);
	return 0;
}

static void setup_frame_transfer(struct b3dfg_dev *fgdev,
	struct b3dfg_buffer *buf, int frame, int acknowledge)
{
	unsigned char *frm_addr;
	dma_addr_t frm_addr_dma;
	struct device *dev = &fgdev->pdev->dev;
	unsigned int frame_size = fgdev->frame_size;
	unsigned char dma_sts = 0xd;

	frm_addr = buf->frame[frame];
	frm_addr_dma = dma_map_single(dev, frm_addr, frame_size, DMA_FROM_DEVICE);
	fgdev->cur_dma_frame_addr = frm_addr_dma;
	fgdev->cur_dma_frame_idx = frame;

	b3dfg_write32(fgdev, B3D_REG_EC220_DMA_ADDR, cpu_to_le32(frm_addr_dma));
	b3dfg_write32(fgdev, B3D_REG_EC220_TRF_SIZE, cpu_to_le32(frame_size >> 2));

	if (likely(acknowledge))
		dma_sts |= 0x2;
	b3dfg_write32(fgdev, B3D_REG_EC220_DMA_STS, 0xf);
}

static int nr_unhandled = 0;

static void unhandled_irq(void)
{
	if (++nr_unhandled == 10) {
		printk("b3dfg too many unhandled interrupts\n");
		panic("b3dfg: too many unhandled interrupts");
	}
}

static irqreturn_t handle_interrupt(struct b3dfg_dev *fgdev)
{
	struct device *dev;
	struct b3dfg_buffer *buf = NULL;
	unsigned int frame_size;
	u32 sts;
	int next_trf;
	int need_ack = 1;
	int unhandled = 0;

	if (unlikely(!fgdev->transmission_enabled)) {
		printk("ignore interrupt, TX disabled\n");
		unhandled = 1;
		/* FIXME should return IRQ_NONE when we are stable */
		goto out;
	}

	sts = b3dfg_read32(fgdev, B3D_REG_DMA_STS);
	if (unlikely(sts == 0)) {
		printk("ignore interrupt, brontes DMA status is 0\n");
		unhandled = 1;
		/* FIXME should return IRQ_NONE when we are stable */
		goto out;
	}

	/* acknowledge interrupt */
	printk(KERN_INFO PFX "got intr, brontes DMASTS=%08x (dropped=%d comp=%d next_trf=%d)\n", sts, (sts >> 8) & 0xff, !!(sts & 0x4), sts & 0x3);

	dev = &fgdev->pdev->dev;
	frame_size = fgdev->frame_size;

	/* FIXME: what happens with list_entry on an empty list? */
	if (unlikely(list_empty(&fgdev->buffer_queue))) {
		/* FIXME need more sanity checking here */
		printk("driver has no buffer ready --> cannot program any more transfers\n");
		goto out;
	}
	
	next_trf = sts & 0x3;

	if (sts & 0x4) {
		int tmpidx;
		u32 tmp;

		tmp = b3dfg_read32(fgdev, B3D_REG_EC220_DMA_STS);
		/* last DMA completed */
		printk("DMA_COMP detected, ec220 dmasts = %08x\n", tmp);
		if (unlikely(fgdev->cur_dma_frame_idx == -1)) {
			printk("ERROR completed but no last idx?\n");
			/* FIXME flesh out error handling */
			goto out;
		}
		dma_unmap_single(dev, fgdev->cur_dma_frame_addr, frame_size,
			DMA_FROM_DEVICE);
		tmpidx = fgdev->cur_dma_frame_idx;
		fgdev->cur_dma_frame_idx = -1;

		buf = list_entry(fgdev->buffer_queue.next, struct b3dfg_buffer, list);
		if (likely(buf)) {
			printk("handle frame completion\n");
			if (++buf->nr_populated_frames == B3DFG_FRAMES_PER_BUFFER) {
				/* last frame of that triplet completed */
				printk("triplet completed\n");
				buf->state = B3DFG_BUFFER_POPULATED;
				list_del_init(&buf->list);
				wake_up_interruptible(&fgdev->buffer_waitqueue);
			}
		} else {
			printk("got frame but no buffer!\n");
		}
	}

	if (next_trf) {
		next_trf--;

		buf = list_entry(fgdev->buffer_queue.next, struct b3dfg_buffer, list);
		printk("program DMA transfer for frame %d\n", next_trf + 1);
		if (likely(buf)) {
			if (next_trf != buf->nr_populated_frames) {
				printk("ERROR mismatch, nr_populated_frames=%d\n",
					buf->nr_populated_frames);
				/* FIXME this is where we should handle dropped triplets */
				goto out;
			}
			setup_frame_transfer(fgdev, buf, next_trf, 1);
			need_ack = 0;
		} else {
			printk("cannot setup next DMA due to no buffer\n");
		}
	}

out:
	if (need_ack) {
		printk("acknowledging interrupt\n");
		b3dfg_write32(fgdev, B3D_REG_EC220_DMA_STS, 0x0b);
	}
	if (unhandled)
		unhandled_irq();
	return IRQ_HANDLED;
}

static irqreturn_t b3dfg_intr(int irq, void *dev_id)
{
	struct b3dfg_dev *fgdev = dev_id;
	irqreturn_t ret;

	spin_lock(&fgdev->irq_lock);
	ret = handle_interrupt(fgdev);
	spin_unlock(&fgdev->irq_lock);
	return ret;
}

static int b3dfg_open(struct inode *inode, struct file *filp)
{
	struct b3dfg_dev *fgdev =
		container_of(inode->i_cdev, struct b3dfg_dev, chardev);

	printk(KERN_INFO PFX "open\n");
	filp->private_data = fgdev;
	return 0;
}

static int b3dfg_release(struct inode *inode, struct file *filp)
{
	struct b3dfg_dev *fgdev = filp->private_data;
	printk(KERN_INFO PFX "release\n");
	set_transmission(fgdev, 0);
	return set_num_buffers(fgdev, 0);
}

static long b3dfg_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct b3dfg_dev *fgdev = filp->private_data;
	switch (cmd) {
	case B3DFG_IOCGFRMSZ:
		return __put_user(fgdev->frame_size, (int __user *) arg);
	case B3DFG_IOCTNUMBUFS:
		return set_num_buffers(fgdev, (int) arg);
	case B3DFG_IOCTTRANS:
		return set_transmission(fgdev, (int) arg);
	case B3DFG_IOCTQUEUEBUF:
		return queue_buffer(fgdev, (int) arg);
	case B3DFG_IOCQPOLLBUF:
		return poll_buffer(fgdev, (int) arg);
	case B3DFG_IOCQWAITBUF:
		return wait_buffer(fgdev, (int) arg);
	default:
		printk(KERN_ERR PFX "unrecognised ioctl %x\n", cmd);
		return -EINVAL;
	}
}

static unsigned int b3dfg_poll(struct file *filp, poll_table *poll_table)
{
	struct b3dfg_dev *fgdev = filp->private_data;
	int i;

	if (unlikely(!fgdev->transmission_enabled)) {
		printk(KERN_ERR PFX "cannot poll() when transmission is disabled\n");
		return POLLERR;
	}

	poll_wait(filp, &fgdev->buffer_waitqueue, poll_table);
	for (i = 0; i < fgdev->num_buffers; i++) {
		if (fgdev->buffers[i].state == B3DFG_BUFFER_POPULATED)
			return POLLIN | POLLRDNORM;
	}

	return 0;
}

static int b3dfg_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct b3dfg_dev *fgdev = filp->private_data;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long bufdatalen = fgdev->num_buffers * fgdev->frame_size * 3;
	unsigned long psize = bufdatalen - offset;

	if (fgdev->num_buffers == 0)
		return -ENOENT;
	if (vsize > psize)
		return -EINVAL;

	vma->vm_flags |= VM_IO | VM_RESERVED | VM_CAN_NONLINEAR;
	vma->vm_ops = &b3dfg_vm_ops;
	b3dfg_vma_open(vma);
	return 0;
}

static struct file_operations b3dfg_fops = {
	.owner = THIS_MODULE,
	.open = b3dfg_open,
	.release = b3dfg_release,
	.unlocked_ioctl = b3dfg_ioctl,
	.poll = b3dfg_poll,
	.mmap = b3dfg_mmap,
};

/* initialize device and any data structures. called before any interrupts
 * are enabled. */
static void b3dfg_init_dev(struct b3dfg_dev *fgdev)
{
	u32 frm_size = b3dfg_read32(fgdev, B3D_REG_FRM_SIZE);
	fgdev->frame_size = frm_size * 4096;
	INIT_LIST_HEAD(&fgdev->buffer_queue);
	init_waitqueue_head(&fgdev->buffer_waitqueue);
	spin_lock_init(&fgdev->irq_lock);
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

	if (fgdev == NULL)
		return -ENOMEM;

	if (minor < 0) {
		printk(KERN_ERR PFX "too many devices found!\n");
		return -EIO;
	}

	b3dfg_devices[minor] = 1;
	printk(KERN_INFO PFX "probe device at %s with IRQ %d\n",
		pci_name(pdev), pdev->irq);

	cdev_init(&fgdev->chardev, &b3dfg_fops);
	fgdev->chardev.owner = THIS_MODULE;

	r = cdev_add(&fgdev->chardev, devno, 1);
	if (r)
		goto err1;

	fgdev->classdev = class_device_create(b3dfg_class, NULL, devno, &pdev->dev,
		DRIVER_NAME "%d", minor);
	if (IS_ERR(fgdev->classdev)) {
		r = PTR_ERR(fgdev->classdev);
		goto err2;
	}

	r = pci_enable_device(pdev);
	if (r)
		goto err3;
	
	if (pci_resource_len(pdev, B3DFG_BAR_REGS) != B3DFG_REGS_LENGTH) {
		printk(KERN_ERR PFX "invalid register resource size\n");
		goto err4;
	}

	if (pci_resource_flags(pdev, B3DFG_BAR_REGS) != IORESOURCE_MEM) {
		printk(KERN_ERR PFX "invalid resource flags");
		goto err4;
	}

	fgdev->regs = ioremap_nocache(pci_resource_start(pdev, B3DFG_BAR_REGS),
		B3DFG_REGS_LENGTH);
	if (!fgdev->regs) {
		printk(KERN_ERR PFX "regs ioremap failed\n");
		goto err4;
	}

	fgdev->pdev = pdev;
	pci_set_drvdata(pdev, fgdev);
	b3dfg_init_dev(fgdev);

	r = request_irq(pdev->irq, b3dfg_intr, IRQF_SHARED, DRIVER_NAME, fgdev);
	if (r) {
		printk(KERN_ERR PFX "couldn't request irq %d\n", pdev->irq);
		goto err5;
	}

	return 0;

err5:
	iounmap(fgdev->regs);
err4:
	pci_disable_device(pdev);
err3:
	class_device_unregister(fgdev->classdev);
err2:
	cdev_del(&fgdev->chardev);
err1:
	kfree(fgdev);
	if (minor >= 0)
		b3dfg_devices[minor] = 0;
	return r;
}

static void __devexit b3dfg_remove(struct pci_dev *pdev)
{
	struct b3dfg_dev *fgdev = pci_get_drvdata(pdev);
	unsigned int minor = MINOR(fgdev->chardev.dev);

	printk(KERN_INFO PFX "remove\n");

	free_irq(pdev->irq, fgdev);
	iounmap(fgdev->regs);
	pci_disable_device(pdev);
	class_device_unregister(fgdev->classdev);
	cdev_del(&fgdev->chardev);
	kfree(fgdev);
	b3dfg_devices[minor] = 0;
}

static struct pci_driver b3dfg_driver = {
	.name = DRIVER_NAME,
	.id_table = b3dfg_ids,
	.probe = b3dfg_probe,
	.remove = b3dfg_remove,
};

static int __init b3dfg_module_init(void)
{
	int r;

	printk(KERN_INFO PFX "loaded\n");

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
	printk(KERN_INFO PFX "unloaded\n");
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

