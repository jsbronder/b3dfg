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
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/cdev.h>

#include <asm/uaccess.h>

#define DRIVER_NAME "b3dfg"
#define PFX DRIVER_NAME ": "
#define B3DFG_MAX_DEVS 4

#define B3DFG_BAR_REGS	0
#define B3DFG_REGS_LENGTH 0x10000

#define B3DFG_IOC_MAGIC		0xb3 /* dfg :-) */
#define B3DFG_IOCGFRMSZ		_IOR(B3DFG_IOC_MAGIC, 1, int)
#define B3DFG_IOCSNUMBUFS	_IOW(B3DFG_IOC_MAGIC, 2, int)

enum {
	/* number of 4kb pages per frame */
	B3D_REG_FRM_SIZE = 0x0,

	/* bit 0: set to enable interrupts */
	B3D_REG_HW_CTRL = 0x4,

	/* bit 0-1 - 1-based ID of next pending frame transfer (0 = nothing pending)
	 * bit 2 indicates the previous DMA transfer has completed
	 * bit 8:15 - counter of number of discarded triplets */
	B3D_REG_DMA_STS = 0x8,
};

struct b3dfg_dev {
	struct pci_dev *pdev;
    struct cdev chardev;
    struct class_device *classdev;
	void __iomem *regs;

	int frame_size;
	
	int num_buffers;
	unsigned char **buffers;
};

static struct class *b3dfg_class;
static dev_t b3dfg_devt;

static const struct pci_device_id b3dfg_ids[] __devinitdata = {
	{ PCI_DEVICE(0x1901, 0x0001) },
	{ },
};

/**** register I/O ****/

static u32 b3dfg_read32(struct b3dfg_dev *fgdev, u8 reg)
{
	return (uint32_t) ioread32(fgdev->regs + reg);
}

/**** buffer management ****/

static long set_num_buffers(struct b3dfg_dev *fgdev, int num_buffers)
{
	int i;
	int frame_size = fgdev->frame_size;
	unsigned char **newbufs;

	printk(KERN_INFO PFX "set %d buffers\n", num_buffers);

	/* FIXME check transmission is not currently enabled */

	if (!fgdev->buffers) {
		/* no buffers allocated yet */
		fgdev->buffers = kmalloc(num_buffers * sizeof(unsigned char *),
			GFP_KERNEL);
		if (!fgdev->buffers)
			return -ENOMEM;

		for (i = 0; i < num_buffers; i++) {
			fgdev->buffers[i] = kmalloc(frame_size, GFP_KERNEL);
			if (!fgdev->buffers[i])
				printk("failed kmalloc\n");
			/* FIXME return value checking */
		}
	} else if (num_buffers == 0) {
		for (i = 0; i < fgdev->num_buffers; i++)
			kfree(fgdev->buffers[i]);
		kfree(fgdev->buffers);
		fgdev->buffers = NULL;
	} else if (fgdev->num_buffers < num_buffers) {
		/* app requested more buffers than we currently have allocated */
		newbufs = krealloc(fgdev->buffers,
			num_buffers * sizeof(unsigned char *), GFP_KERNEL);
		if (!newbufs)
			return -ENOMEM;
		for (i = fgdev->num_buffers; i < num_buffers; i++) {
			newbufs[i] = kmalloc(frame_size, GFP_KERNEL);
			if (!newbufs[i])
				printk("failed kmalloc\n");
			/* FIXME return value checking */
		}
		fgdev->buffers = newbufs;
	} else if (fgdev->num_buffers > num_buffers) {
		/* app requests a decrease in buffers */
		for (i = num_buffers; i < fgdev->num_buffers; i++)
			kfree(fgdev->buffers[i]);
		newbufs = krealloc(fgdev->buffers,
			num_buffers * sizeof(unsigned char *), GFP_KERNEL);
		/* FIXME return value checking */
		fgdev->buffers = newbufs;
	}
	fgdev->num_buffers = num_buffers;
	return 0;
}

static irqreturn_t b3dfg_intr(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static int b3dfg_open(struct inode *inode, struct file *filp)
{
	struct b3dfg_dev *fgdev =
		container_of(inode->i_cdev, struct b3dfg_dev, chardev);

	filp->private_data = fgdev;
	return 0;
}

static int b3dfg_release(struct inode *inode, struct file *filp)
{
	struct b3dfg_dev *fgdev = filp->private_data;
	return set_num_buffers(fgdev, 0);
}

static long b3dfg_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct b3dfg_dev *fgdev = filp->private_data;
	printk(KERN_INFO PFX "ioctl %x %ld\n", cmd, arg);
	switch (cmd) {
	case B3DFG_IOCGFRMSZ:
		printk("get frame size\n");
		return __put_user(fgdev->frame_size, (int __user *) arg);
	case B3DFG_IOCSNUMBUFS:
		printk("set num buffers\n");
		return set_num_buffers(fgdev, (int) arg);
	default:
		printk(KERN_ERR PFX "unrecognised ioctl %x\n", cmd);
		return -EINVAL;
	}
}

static struct file_operations b3dfg_fops = {
	.owner = THIS_MODULE,
	.open = b3dfg_open,
	.release = b3dfg_release,
	.unlocked_ioctl = b3dfg_ioctl,
};

static void b3dfg_init_dev(struct b3dfg_dev *fgdev)
{
	u32 frm_size = b3dfg_read32(fgdev, B3D_REG_FRM_SIZE);
	printk("frm_size %d\n", frm_size);
	fgdev->frame_size = frm_size * 4096;
}

static int __devinit b3dfg_probe(struct pci_dev *pdev,
	const struct pci_device_id *id)
{
	struct b3dfg_dev *fgdev = kzalloc(sizeof(*fgdev), GFP_KERNEL);
	int r = 0;

	/* FIXME: always using minor 0, hope we don't probe twice! */
	int minor = 0;
	dev_t devno = MKDEV(MAJOR(b3dfg_devt), minor);

	if (fgdev == NULL)
		return -ENOMEM;

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

	r = request_irq(pdev->irq, b3dfg_intr, IRQF_SHARED, DRIVER_NAME, fgdev);
	if (r) {
		printk(KERN_ERR PFX "couldn't request irq %d\n", pdev->irq);
		goto err5;
	}

	b3dfg_init_dev(fgdev);
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
	return r;
}

static void __devexit b3dfg_remove(struct pci_dev *pdev)
{
	struct b3dfg_dev *fgdev = pci_get_drvdata(pdev);

	printk(KERN_INFO PFX "remove\n");

	free_irq(pdev->irq, fgdev);
	iounmap(fgdev->regs);
	pci_disable_device(pdev);
	class_device_unregister(fgdev->classdev);
	cdev_del(&fgdev->chardev);
	kfree(fgdev);
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

