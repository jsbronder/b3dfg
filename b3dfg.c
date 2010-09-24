 /*
 * Brontes PCI frame grabber driver
 *
 * Copyright (C) 2008 3M Company
 * Contact: Justin Bronder <jsbronder@brontes3d.com>
 * Original Authors: Daniel Drake <ddrake@brontes3d.com>
 *                   Duane Griffin <duaneg@dghda.com>
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
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/sysfs.h>

static unsigned int b3dfg_nbuf = 3;

MODULE_AUTHOR("Daniel Drake <ddrake@brontes3d.com>");
MODULE_DESCRIPTION("Brontes frame grabber driver");
MODULE_LICENSE("GPL");

#define DRIVER_NAME "b3dfg"
#define B3DFG_MAX_DEVS 4
#define B3DFG_FRAMES_PER_BUFFER 3

#define B3DFG_BAR_REGS	0
#define B3DFG_REGS_LENGTH 0x10000

#define B3DFG_IOC_MAGIC		0xb3 /* dfg :-) */
#define B3DFG_IOCTRELBUF	_IO(B3DFG_IOC_MAGIC, 1)
#define B3DFG_IOCTGETBUF	_IOWR(B3DFG_IOC_MAGIC, 2, struct b3dfg_wait)

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
	B3DFG_BUFFER_IDLE = 0,
	B3DFG_BUFFER_PENDING,
	B3DFG_BUFFER_POPULATED,
	B3DFG_BUFFER_USER,
};

struct b3dfg_buffer {
	unsigned char *frame[B3DFG_FRAMES_PER_BUFFER];
	struct list_head list;
	u8 state;
	unsigned long jiffies;
};

struct b3dfg_dev {

	/* no protection needed: all finalized at initialization time */
	struct pci_dev *pdev;
	struct cdev chardev;
	struct device *dev;
	void __iomem *regs;
	unsigned int frame_size;

	/*
	 * Protects buffer state, including triplet_ready, cur_dma_buf_idx,
	 * cur_dma_frame_idx, cur_dma_frame_addr, cur_user_buf_idx, consumer.
	 */
	spinlock_t buffer_lock;
	struct b3dfg_buffer *buffers;

	/* Current dma buffer */
	int cur_dma_buf_idx;
	
	/* Last frame in triplet transferred (-1 if none). */
	int cur_dma_frame_idx;

	/* Current frame's address for DMA. */
	dma_addr_t cur_dma_frame_addr;

	/* Buffer currently locked by userspace (-1 if none). */
	int cur_user_buf_idx;

	/* Process that owns incoming buffers */
	pid_t consumer;

	/*
	 * Protects cstate_tstamp.
	 * Nests inside buffer_lock.
	 */
	spinlock_t cstate_lock;
	unsigned long cstate_tstamp;

	/*
	 * Protects triplets_dropped.
	 * Nests inside buffers_lock.
	 */
	spinlock_t triplets_dropped_lock;
	unsigned int triplets_dropped;

	wait_queue_head_t buffer_waitqueue;

	unsigned int transmission_enabled:1;
	unsigned int triplet_ready:1;

};

static u8 b3dfg_devices[B3DFG_MAX_DEVS];

static struct class *b3dfg_class;
static dev_t b3dfg_devt;

static const struct pci_device_id b3dfg_ids[] __devinitdata = {
	{ PCI_DEVICE(0x0b3d, 0x0001) },
	{ },
};

MODULE_DEVICE_TABLE(pci, b3dfg_ids);

/***** user-visible types *****/

struct b3dfg_wait {
	int buffer_idx;
	unsigned int timeout;
	unsigned int triplets_dropped;
	struct timeval tv;
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

/*
 * Program EC220 for transfer of a specific frame.
 * Called with buffer_lock held.
 */
static int setup_frame_transfer(struct b3dfg_dev *fgdev, int frame)
{
	unsigned char *frm_addr;
	dma_addr_t frm_addr_dma;
	unsigned int frm_size = fgdev->frame_size;

	frm_addr = fgdev->buffers[fgdev->cur_dma_buf_idx].frame[frame];
	frm_addr_dma = pci_map_single(fgdev->pdev, frm_addr,
					  frm_size, PCI_DMA_FROMDEVICE);
	if (pci_dma_mapping_error(fgdev->pdev, frm_addr_dma))
		return -ENOMEM;

	fgdev->cur_dma_frame_addr = frm_addr_dma;
	fgdev->cur_dma_frame_idx = frame;

	b3dfg_write32(fgdev, B3D_REG_EC220_DMA_ADDR,
					cpu_to_le32(frm_addr_dma));
	b3dfg_write32(fgdev, B3D_REG_EC220_TRF_SIZE,
					cpu_to_le32(frm_size >> 2));
	b3dfg_write32(fgdev, B3D_REG_EC220_DMA_STS, 0xf);

	return 0;
}

/* Caller should hold buffer lock */
static int find_free_buffer(struct b3dfg_dev *fgdev, int *idx)
{
	int i;

	*idx = -1;
	for (i = 0; i < b3dfg_nbuf; i++) {
		if (fgdev->buffers[i].state == B3DFG_BUFFER_IDLE) {
			*idx = i;
			break;
		} else if (fgdev->buffers[i].state == B3DFG_BUFFER_POPULATED &&
					(*idx == -1 || 
					time_before(fgdev->buffers[i].jiffies, fgdev->buffers[*idx].jiffies)))
			*idx = i;
	}
	return *idx != -1 ? 0 : -EBUSY;
}

/* Caller should hold buffer lock */
static int find_newest_buffer(struct b3dfg_dev *fgdev, int *idx)
{
	int i;
	int dropped = 0;

	*idx = -1;
	for (i = 0; i < b3dfg_nbuf; i++) {
		if (fgdev->buffers[i].state == B3DFG_BUFFER_POPULATED) {
			if (*idx == -1)
				*idx = i;
			else if (time_after(fgdev->buffers[i].jiffies, fgdev->buffers[*idx].jiffies)) {
				fgdev->buffers[*idx].state = B3DFG_BUFFER_IDLE;
				*idx = i;
				dropped++;
			} else {
				fgdev->buffers[i].state = B3DFG_BUFFER_IDLE;
				dropped++;
			}
		}
	}

	if (dropped) {
		spin_lock(&fgdev->triplets_dropped_lock);
		fgdev->triplets_dropped++;
		spin_unlock(&fgdev->triplets_dropped_lock);
	}

	return *idx != -1 ? 0 : -EBUSY;
}
			
/* Caller should hold buffer lock */
static void dequeue_all_buffers(struct b3dfg_dev *fgdev)
{
	int i;
	for (i = 0; i < b3dfg_nbuf; i++)
		fgdev->buffers[i].state = B3DFG_BUFFER_IDLE;
}

static unsigned long get_cstate_change(struct b3dfg_dev *fgdev)
{
	unsigned long flags, when;

	spin_lock_irqsave(&fgdev->cstate_lock, flags);
	when = fgdev->cstate_tstamp;
	spin_unlock_irqrestore(&fgdev->cstate_lock, flags);
	return when;
}

static int is_event_ready(struct b3dfg_dev *fgdev, int *idx,
			  unsigned long when)
{
	int result;
	unsigned long flags;

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	spin_lock(&fgdev->cstate_lock);
	result = (!fgdev->transmission_enabled ||
		  find_newest_buffer(fgdev, idx) == 0 ||
		  when != fgdev->cstate_tstamp);
	spin_unlock(&fgdev->cstate_lock);
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);

	return result;
}

static int release_buffer(struct b3dfg_dev *fgdev)
{
	struct device *dev = &fgdev->pdev->dev;
	unsigned long flags;
	int r = 0;

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	if (fgdev->consumer != current->pid) {
		dev_dbg(dev, "release_buffer:  pid %d does not have consumer lock\n", current->pid);
		r = -EBUSY;
		goto out;
	}

	if (unlikely(fgdev->cur_user_buf_idx < 0))
		goto out;

	fgdev->buffers[fgdev->cur_user_buf_idx].state = B3DFG_BUFFER_IDLE;
	fgdev->cur_user_buf_idx = -1;

out:
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
	return r;
}


static int get_buffer(struct b3dfg_dev *fgdev, void __user *arg)
{
	struct device *dev = &fgdev->pdev->dev;
	struct b3dfg_wait w;
	unsigned long flags, when;
	int idx, r = 0;

	if (copy_from_user(&w, arg, sizeof(w)))
		return -EFAULT;

	if (!fgdev->transmission_enabled) {
		dev_dbg(dev, "cannot wait, transmission disabled\n");
		return -EINVAL;
	}

	if (unlikely(!b3dfg_read32(fgdev, B3D_REG_WAND_STS))) {
		dev_dbg(dev, "cannot get buffer without wand\n");
		return -EINVAL;
	}
	
	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	if (fgdev->consumer != current->pid) {
		dev_dbg(dev, "get_buffer:  pid %d does not have consumer lock\n", current->pid);
		r = -EBUSY;
		goto out_unlock;
	}
	
	if (fgdev->cur_user_buf_idx >= 0){
		dev_dbg(dev, "get_buffer:  pid %d already locked buffer %d\n",
			current->pid, fgdev->cur_user_buf_idx);
		r = -EPERM;
		goto out_unlock;
	}
		
	if ( likely(find_newest_buffer(fgdev, &idx) == 0 )) {
		r = w.timeout;
		goto out_triplets_dropped;
	}
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);

	when = get_cstate_change(fgdev);
	if (w.timeout > 0) {
		r = wait_event_interruptible_timeout(fgdev->buffer_waitqueue,
			is_event_ready(fgdev, &idx, when),
			(w.timeout * HZ) / 1000);

		if (unlikely(r < 0))
			goto out;

		w.timeout = r * 1000 / HZ;
	} else {
		r = wait_event_interruptible(fgdev->buffer_waitqueue,
			is_event_ready(fgdev, &idx, when));

		if (unlikely(r)) {
			r = -ERESTARTSYS;
			goto out;
		}
	}

	/* TODO: Inform the user via field(s) in w? */
	if (!fgdev->transmission_enabled || when != get_cstate_change(fgdev)) {
		r = -EINVAL;
		goto out;
	}

	spin_lock_irqsave(&fgdev->buffer_lock, flags);

	if (fgdev->buffers[idx].state != B3DFG_BUFFER_POPULATED) {
		r = -ETIMEDOUT;
		goto out_unlock;
	}

out_triplets_dropped:
	dev_dbg(dev, "get_buffer() got %d\n", idx);
	fgdev->buffers[idx].state = B3DFG_BUFFER_USER;
	jiffies_to_timeval(fgdev->buffers[idx].jiffies, &w.tv);
	w.buffer_idx = idx;

	/* IRQs already disabled by spin_lock_irqsave above. */
	spin_lock(&fgdev->triplets_dropped_lock);
	w.triplets_dropped = fgdev->triplets_dropped;
	fgdev->triplets_dropped = 0;
	fgdev->cur_user_buf_idx = idx;
	spin_unlock(&fgdev->triplets_dropped_lock);

out_unlock:
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
	if (copy_to_user(arg, &w, sizeof(w)))
		r = -EFAULT;
out:
	return r;
}

/* mmap page fault handler */
static int b3dfg_vma_fault(struct vm_area_struct *vma,
	struct vm_fault *vmf)
{
	struct b3dfg_dev *fgdev = vma->vm_file->private_data;
	unsigned long off = vmf->pgoff << PAGE_SHIFT;
	unsigned int frame_size = fgdev->frame_size;
	unsigned int buf_size = frame_size * B3DFG_FRAMES_PER_BUFFER;
	unsigned char *addr;

	/* determine which buffer the offset lies within */
	unsigned int buf_idx = off / buf_size;
	/* and the offset into the buffer */
	unsigned int buf_off = off % buf_size;

	/* determine which frame inside the buffer the offset lies in */
	unsigned int frm_idx = buf_off / frame_size;
	/* and the offset into the frame */
	unsigned int frm_off = buf_off % frame_size;

	if (unlikely(buf_idx >= b3dfg_nbuf))
		return VM_FAULT_SIGBUS;

	addr = fgdev->buffers[buf_idx].frame[frm_idx] + frm_off;
	vm_insert_pfn(vma, (unsigned long)vmf->virtual_address,
			  virt_to_phys(addr) >> PAGE_SHIFT);

	return VM_FAULT_NOPAGE;
}

static struct vm_operations_struct b3dfg_vm_ops = {
	.fault = b3dfg_vma_fault,
};


static int disable_transmission(struct b3dfg_dev *fgdev)
{
	struct device *dev = &fgdev->pdev->dev;
	unsigned long flags;
	u32 tmp;
	int r = 0;

	dev_dbg(dev, "disable transmission\n");

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	if (fgdev->consumer && fgdev->consumer != current->pid) {
		dev_dbg(dev, "locked by consumer, pid %d", fgdev->consumer);
		r = -EBUSY;
		goto out_unlock;
	}
	fgdev->consumer = 0;

	/* guarantee that no more interrupts will be serviced */
	if (!fgdev->transmission_enabled)
		goto out_unlock;	
	fgdev->transmission_enabled = 0;

	b3dfg_write32(fgdev, B3D_REG_HW_CTRL, 0);

	/* FIXME: temporary debugging only. if the board stops transmitting,
	 * hitting ctrl+c and seeing this message is useful for determining
	 * the state of the board. */
	tmp = b3dfg_read32(fgdev, B3D_REG_DMA_STS);
	dev_dbg(dev, "DMA_STS reads %x after TX stopped\n", tmp);

	dequeue_all_buffers(fgdev);
	wake_up_interruptible(&fgdev->buffer_waitqueue);

out_unlock:
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
	return r;
}

/* Called in interrupt context. */
static void handle_cstate_unplug(struct b3dfg_dev *fgdev)
{
	/* Disable all interrupts. */
	b3dfg_write32(fgdev, B3D_REG_HW_CTRL, 0);

	/* Stop transmission. */
	spin_lock(&fgdev->buffer_lock);
	fgdev->transmission_enabled = 0;

	fgdev->cur_dma_frame_idx = -1;
	fgdev->triplet_ready = 0;
	if (fgdev->cur_dma_frame_addr) {
		pci_unmap_single(fgdev->pdev, fgdev->cur_dma_frame_addr,
				 fgdev->frame_size, PCI_DMA_FROMDEVICE);
		fgdev->cur_dma_frame_addr = 0;
	}
	dequeue_all_buffers(fgdev);
	spin_unlock(&fgdev->buffer_lock);
}

/* Called in interrupt context. */
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
	 * the hardware will raise a couple of spurious interrupts, so
	 * just ignore them for now.
	 *
	 * Once the hardware is fixed we should complain and treat it as an
	 * unplug. Or at least track how frequently it is happening and do
	 * so if too many come in.
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

/* Called with buffer_lock held. */
static void transfer_complete(struct b3dfg_dev *fgdev)
{
	struct device *dev = &fgdev->pdev->dev;

	pci_unmap_single(fgdev->pdev, fgdev->cur_dma_frame_addr,
			 fgdev->frame_size, PCI_DMA_FROMDEVICE);
	fgdev->cur_dma_frame_addr = 0;

	dev_dbg(dev, "handle frame completion\n");
	if (fgdev->cur_dma_frame_idx == B3DFG_FRAMES_PER_BUFFER - 1) {
		/* last frame of that triplet completed */
		dev_dbg(dev, "triplet completed\n");
		fgdev->buffers[fgdev->cur_dma_buf_idx].state = B3DFG_BUFFER_POPULATED;
		fgdev->buffers[fgdev->cur_dma_buf_idx].jiffies = jiffies;
		wake_up_interruptible(&fgdev->buffer_waitqueue);
		fgdev->cur_dma_buf_idx = -1;
	}
}

/*
 * Called with buffer_lock held.
 *
 * Note that idx is the (1-based) *next* frame to be transferred, while
 * cur_dma_frame_idx is the (0-based) *last* frame to have been transferred (or
 * -1 if none). Thus there should be a difference of 2 between them.
 */
static bool setup_next_frame_transfer(struct b3dfg_dev *fgdev, int idx)
{
	struct device *dev = &fgdev->pdev->dev;
	bool need_ack = 1;

	dev_dbg(dev, "program DMA transfer for next frame: %d\n", idx);

	if (fgdev->cur_dma_buf_idx == -1) {
		/* Find a new buffer to pack the triplet into */
		while (find_free_buffer(fgdev, &fgdev->cur_dma_buf_idx) != 0) {
			/* TOFIX:  Can loop forever */
			dev_warn(dev, "Unable to find free buffer.\n");
		}
		fgdev->buffers[fgdev->cur_dma_buf_idx].state = B3DFG_BUFFER_PENDING;
		dev_dbg(dev, "Preparing to fill buffer %d\n", fgdev->cur_dma_buf_idx);
	}

	if (idx == fgdev->cur_dma_frame_idx + 2) {
		if (setup_frame_transfer(fgdev, idx - 1))
			dev_err(dev, "unable to map DMA buffer\n");
		need_ack = 0;
	} else {
		dev_err(dev, "frame mismatch, got %d, expected %d\n",
			idx, fgdev->cur_dma_frame_idx + 2);

		/* FIXME: handle dropped triplets here */
	}

	return need_ack;
}

static irqreturn_t b3dfg_intr(int irq, void *dev_id)
{
	struct b3dfg_dev *fgdev = dev_id;
	struct device *dev = &fgdev->pdev->dev;
	u32 sts;
	u8 dropped;
	bool need_ack = 1;
	irqreturn_t res = IRQ_HANDLED;

	sts = b3dfg_read32(fgdev, B3D_REG_DMA_STS);
	if (unlikely(sts == 0)) {
		dev_warn(dev, "ignore interrupt, DMA status is 0\n");
		res = IRQ_NONE;
		goto out;
	}

	if (unlikely(!fgdev->transmission_enabled)) {
		dev_warn(dev, "ignore interrupt, TX disabled\n");
		res = IRQ_HANDLED;
		goto out;
	}

	/* Handle dropped frames, as reported by the hardware. */
	dropped = (sts >> 8) & 0xff;
	dev_dbg(dev, "intr: DMA_STS=%08x (drop=%d comp=%d next=%d)\n",
		sts, dropped, !!(sts & 0x4), sts & 0x3);
	if (unlikely(dropped > 0))
		dev_err(dev,
			"b3dfg reported %d buffers dropped since last interrupt was handled",
			dropped);

	/* Handle a cable state change (i.e. the wand being unplugged). */
	if (sts & 0x08) {
		handle_cstate_change(fgdev);
		goto out;
	}

	spin_lock(&fgdev->buffer_lock);

	/* Has a frame transfer been completed? */
	if (sts & 0x4) {
		u32 dma_status = b3dfg_read32(fgdev, B3D_REG_EC220_DMA_STS);

		/* Check for DMA errors reported by the hardware. */
		if (unlikely(dma_status & 0x1)) {
			dev_err(dev, "EC220 error: %08x\n", dma_status);

			/* FIXME flesh out error handling */
			goto out_unlock;
		}

		/* Sanity check, we should have a frame index at this point. */
		if (unlikely(fgdev->cur_dma_frame_idx == -1)) {
			dev_err(dev, "completed but no last idx?\n");

			/* FIXME flesh out error handling */
			goto out_unlock;
		}

		transfer_complete(fgdev);
	}

	/* Is there another frame transfer pending? */
	if (sts & 0x3)
		need_ack = setup_next_frame_transfer(fgdev, sts & 0x3);
	else
		fgdev->cur_dma_frame_idx = -1;

out_unlock:
	spin_unlock(&fgdev->buffer_lock);
out:
	if (need_ack) {
		dev_dbg(dev, "acknowledging interrupt\n");
		b3dfg_write32(fgdev, B3D_REG_EC220_DMA_STS, 0x0b);
	}
	return res;
}

static int b3dfg_open(struct inode *inode, struct file *filp)
{
	struct b3dfg_dev *fgdev =
		container_of(inode->i_cdev, struct b3dfg_dev, chardev);
	struct device *dev = &fgdev->pdev->dev;
	int r = 0;
	unsigned long flags;

	dev_dbg(&fgdev->pdev->dev, "open\n");
	filp->private_data = fgdev;

	/* check the cable is plugged in. */
	if (!b3dfg_read32(fgdev, B3D_REG_WAND_STS)) {
		dev_dbg(dev, "cannot start transmission without wand\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	if (fgdev->consumer && fgdev->consumer != current->pid) {
		dev_dbg(dev, "locked by consumer, pid %d", fgdev->consumer);
		r = -EBUSY;
		goto out_unlock;
	} 

	/* Handle racing open calls. */
	if (fgdev->consumer)
		goto out_unlock;

	spin_lock(&fgdev->triplets_dropped_lock);
	fgdev->triplets_dropped = 0;
	spin_unlock(&fgdev->triplets_dropped_lock);

	fgdev->consumer = current->pid;
	fgdev->triplet_ready = 0;
	fgdev->cur_dma_frame_idx = -1;
	fgdev->cur_dma_buf_idx = -1;
	fgdev->cur_user_buf_idx = -1;
	fgdev->transmission_enabled = 1;

	/* Enable DMA and cable status interrupts. */
	b3dfg_write32(fgdev, B3D_REG_HW_CTRL, 0x03);

out_unlock:
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);
	return r;
}

static int b3dfg_release(struct inode *inode, struct file *filp)
{
	struct b3dfg_dev *fgdev = filp->private_data;

	disable_transmission(fgdev);
	return 0;
}

static long b3dfg_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct b3dfg_dev *fgdev = filp->private_data;

	switch (cmd) {
	case B3DFG_IOCTRELBUF:
		return release_buffer(fgdev);
	case B3DFG_IOCTGETBUF:
		return get_buffer(fgdev, (void __user *) arg);
	default:
		dev_dbg(&fgdev->pdev->dev, "unrecognised ioctl %x\n", cmd);
		return -EINVAL;
	}
}

static unsigned int b3dfg_poll(struct file *filp, poll_table *poll_table)
{
	struct b3dfg_dev *fgdev = filp->private_data;
	unsigned long flags, when;
	int i;
	int r = 0;

	when = get_cstate_change(fgdev);
	poll_wait(filp, &fgdev->buffer_waitqueue, poll_table);

	spin_lock_irqsave(&fgdev->buffer_lock, flags);
	for (i = 0; i < b3dfg_nbuf; i++) {
		if (fgdev->buffers[i].state == B3DFG_BUFFER_POPULATED) {
			r = POLLIN | POLLRDNORM;
			break;
		}
	}
	spin_unlock_irqrestore(&fgdev->buffer_lock, flags);

	/* TODO: Confirm this is how we want to communicate the change. */
	if (!fgdev->transmission_enabled || when != get_cstate_change(fgdev))
		r = POLLERR;

	return r;
}

static int b3dfg_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct b3dfg_dev *fgdev = filp->private_data;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long bufdatalen = b3dfg_nbuf * fgdev->frame_size * 3;
	unsigned long psize = bufdatalen - offset;
	int r = 0;

	if (vsize <= psize) {
		vma->vm_flags |= VM_IO | VM_RESERVED | VM_CAN_NONLINEAR |
				 VM_PFNMAP;
		vma->vm_ops = &b3dfg_vm_ops;
	} else {
		r = -EINVAL;
	}

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


static ssize_t show_frame_size(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct b3dfg_dev *fgdev = pci_get_drvdata(pdev);
	return sprintf(buf, "%u\n", fgdev->frame_size);
}
DEVICE_ATTR(frame_size, S_IRUGO, show_frame_size, NULL);

static ssize_t show_cable_status(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct b3dfg_dev *fgdev = pci_get_drvdata(pdev);
	return sprintf(buf, "%u\n", b3dfg_read32(fgdev, B3D_REG_WAND_STS));
}
DEVICE_ATTR(cable_status, S_IRUGO, show_cable_status, NULL);

static ssize_t show_transmission(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct b3dfg_dev *fgdev = pci_get_drvdata(pdev);
	return sprintf(buf, "%u\n",
		b3dfg_read32(fgdev, B3D_REG_HW_CTRL) & 0x3 ? 1 : 0);
}
DEVICE_ATTR(transmission, S_IRUGO, show_transmission, NULL);

static struct attribute *b3dfg_attributes[] = {
	&dev_attr_frame_size.attr,
	&dev_attr_cable_status.attr,
	&dev_attr_transmission.attr,
	NULL,
};

static struct attribute_group b3dfg_attribute_group = {
	.attrs = b3dfg_attributes,
};


static void free_all_frame_buffers(struct b3dfg_dev *fgdev)
{
	int i, j;
	for (i = 0; i < b3dfg_nbuf; i++)
		for (j = 0; j < B3DFG_FRAMES_PER_BUFFER; j++)
			kfree(fgdev->buffers[i].frame[j]);
	kfree(fgdev->buffers);
}

/* initialize device and any data structures. called before any interrupts
 * are enabled. */
static int b3dfg_init_dev(struct b3dfg_dev *fgdev)
{
	int i, j;
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
	fgdev->buffers = kzalloc(sizeof(struct b3dfg_buffer) * b3dfg_nbuf,
				 GFP_KERNEL);
	if (!fgdev->buffers)
		goto err_no_buf;
	for (i = 0; i < b3dfg_nbuf; i++) {
		struct b3dfg_buffer *buf = &fgdev->buffers[i];
		for (j = 0; j < B3DFG_FRAMES_PER_BUFFER; j++) {
			buf->frame[j] = kmalloc(fgdev->frame_size, GFP_KERNEL);
			if (!buf->frame[j])
				goto err_no_mem;
		}
		INIT_LIST_HEAD(&buf->list);
	}

	fgdev->consumer = 0;
	init_waitqueue_head(&fgdev->buffer_waitqueue);
	spin_lock_init(&fgdev->buffer_lock);
	spin_lock_init(&fgdev->cstate_lock);
	spin_lock_init(&fgdev->triplets_dropped_lock);
	return 0;

err_no_mem:
	free_all_frame_buffers(fgdev);
err_no_buf:
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

	fgdev->dev = device_create(
		b3dfg_class,
		&pdev->dev,
		devno,
		dev_get_drvdata(&pdev->dev),
		DRIVER_NAME "%d", minor);

	if (IS_ERR(fgdev->dev)) {
		dev_err(&pdev->dev, "cannot create device\n");
		r = PTR_ERR(fgdev->dev);
		goto err_del_cdev;
	}

	r = sysfs_create_group(&pdev->dev.kobj, &b3dfg_attribute_group);
	if (r) {
		dev_err(&pdev->dev, "cannot create sysfs group\n");
		goto err_dev_unreg;
	}

	r = pci_enable_device(pdev);
	if (r) {
		dev_err(&pdev->dev, "cannot enable PCI device\n");
		goto err_sysfs_remove;
	}

	res_len = pci_resource_len(pdev, B3DFG_BAR_REGS);
	if (res_len != B3DFG_REGS_LENGTH) {
		dev_err(&pdev->dev, "invalid register resource size\n");
		r = -EIO;
		goto err_disable;
	}

	if (pci_resource_flags(pdev, B3DFG_BAR_REGS)
				!= (IORESOURCE_MEM | IORESOURCE_SIZEALIGN)) {
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

	r = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (r) {
		dev_err(&pdev->dev, "no usable DMA configuration\n");
		goto err_free_res;
	}

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
	device_destroy(b3dfg_class, devno);
err_sysfs_remove:
	sysfs_remove_group(&pdev->dev.kobj, &b3dfg_attribute_group);
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
	sysfs_remove_group(&pdev->dev.kobj, &b3dfg_attribute_group);
	device_destroy(b3dfg_class, MKDEV(MAJOR(b3dfg_devt), minor));
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

// vim: noet
