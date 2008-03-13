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

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/pci.h>

#define DRVNAME			"b3dfg"
#define PREFIX			DRVNAME ":"
#define B3DFG_MAX_MINORS	4

static struct class *b3dfg_class;

struct b3dfg_dev {
	struct cdev chardev;
	struct class_device *classdev;
};

struct b3dfg_dev *b3dfg_devs[B3DFG_MAX_MINORS];
static int b3dfg_major;

static int b3dfg_open(struct inode *inode, struct file *file)
{
	struct b3dfg_dev *b3d_dev = container_of(inode->i_cdev,
		struct b3dfg_dev, chardev);
	file->private_data = b3d_dev;
	return 0;
}

static int b3dfg_release(struct inode *inode, struct file *file)
{
	return 0;
}

struct file_operations b3dfg_fops = {
	.owner = THIS_MODULE,
	.open = b3dfg_open,
	.release = b3dfg_release,
};

static unsigned int __devinit b3dfg_get_next_minor(void)
{
	unsigned int i;
	for (i = 0; i < B3DFG_MAX_MINORS; i++)
		if (!b3dfg_devs[i])
			break;

	return i;
}

static int __devinit b3dfg_probe(struct pci_dev *pdev,
	const struct pci_device_id *id)
{
	int r;
	unsigned int minor;
	struct b3dfg_dev *b3d_dev;

	r = pci_enable_device(pdev);
	if (r < 0)
		return r;

	minor = b3dfg_get_next_minor();
	if (minor == B3DFG_MAX_MINORS) {
		printk(KERN_ERR PREFIX "too many devices\n");
		r = -EIO;
		goto err_disable;
	}

	b3d_dev = kzalloc(sizeof(*b3d_dev), GFP_KERNEL);
	if (!b3d_dev)
		goto err_disable;

	cdev_init(&b3d_dev->chardev, &b3dfg_fops);
	b3d_dev->chardev.owner = THIS_MODULE;

	r = cdev_add(&b3d_dev->chardev, MKDEV(b3dfg_major, minor), 1);
	if (r < 0)
		goto err_free_dev;

	b3d_dev->classdev = class_device_create(b3dfg_class, NULL,
		MKDEV(b3dfg_major, minor), &pdev->dev, DRVNAME "%u", minor);
	if (IS_ERR(b3d_dev->classdev)) {
		r = PTR_ERR(b3d_dev->classdev);
		goto err_cdev_del;
	}

	b3dfg_devs[minor] = b3d_dev;
	pci_set_drvdata(pdev, b3d_dev);
	printk(KERN_INFO PREFIX "adding device %u\n", minor);
	return 0;

err_cdev_del:
	cdev_del(&b3d_dev->chardev);
err_free_dev:
	kfree(b3d_dev);
err_disable:
	pci_disable_device(pdev);
	return r;
}

static void __devexit b3dfg_remove(struct pci_dev *pdev)
{
	struct b3dfg_dev *b3d_dev = pci_get_drvdata(pdev);
	unsigned int minor = MINOR(b3d_dev->chardev.dev);

	class_device_unregister(b3d_dev->classdev);
	cdev_del(&b3d_dev->chardev);
	b3dfg_devs[minor] = NULL;
	kfree(b3d_dev);
	pci_disable_device(pdev);
	printk(KERN_INFO PREFIX "removing device %u\n", minor);
}

static struct pci_device_id b3dfg_ids[] = {
	{ PCI_DEVICE(0x1901, 0x0001) },
	{ 0, },
};

static struct pci_driver b3dfg_driver = {
	.name = DRVNAME,
	.id_table = b3dfg_ids,
	.probe = b3dfg_probe,
	.remove = b3dfg_remove,
};

static int __init b3dfg_module_init(void)
{
	dev_t first_dev;
	int r;

	r = alloc_chrdev_region(&first_dev, 0, B3DFG_MAX_MINORS, DRVNAME);
	if (r < 0)
		return r;

	b3dfg_class = class_create(THIS_MODULE, DRVNAME);
	if (IS_ERR(b3dfg_class)) {
		r = PTR_ERR(b3dfg_class);
		goto out_chrdev;
	}

	r = pci_register_driver(&b3dfg_driver);
	if (r < 0)
		goto out_class;

	b3dfg_major = MAJOR(first_dev);
	printk(KERN_INFO PREFIX "loaded\n");
	return 0;
out_class:
	class_destroy(b3dfg_class);
out_chrdev:
	unregister_chrdev_region(first_dev, B3DFG_MAX_MINORS);
	return r;
}

static void __exit b3dfg_module_exit(void)
{
	pci_unregister_driver(&b3dfg_driver);
	unregister_chrdev_region(MKDEV(b3dfg_major, 0), B3DFG_MAX_MINORS);
	class_destroy(b3dfg_class);
	printk(KERN_INFO PREFIX "unloaded\n");
}

MODULE_AUTHOR("Daniel Drake <ddrake@brontes3d.com>");
MODULE_DESCRIPTION("Brontes PCI frame grabber driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(pci, b3dfg_ids);
module_init(b3dfg_module_init);
module_exit(b3dfg_module_exit);

