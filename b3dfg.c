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

#include <linux/kernel.h>
#include <linux/pci.h>

static int __devinit b3dfg_probe(struct pci_dev *dev,
	const struct pci_device_id *id)
{
	int r;
	r = pci_enable_device(dev);
	if (r < 0)
		return r;
	
	return 0;
}

static void __devexit b3dfg_remove(struct pci_dev *dev)
{
	pci_disable_device(dev);
}

static struct pci_device_id b3dfg_ids[] = {
	{ PCI_DEVICE(0x1901, 0x0001) },
	{ 0, },
};

static struct pci_driver b3dfg_driver = {
	.name = "b3dfg",
	.id_table = b3dfg_ids,
	.probe = b3dfg_probe,
	.remove = b3dfg_remove,
};

static int __init b3dfg_module_init(void)
{
	return pci_register_driver(&b3dfg_driver);
}

static void __exit b3dfg_module_exit(void)
{
	pci_unregister_driver(&b3dfg_driver);
}

MODULE_AUTHOR("Daniel Drake <ddrake@brontes3d.com>");
MODULE_DESCRIPTION("Brontes PCI frame grabber driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(pci, b3dfg_ids);

module_init(b3dfg_module_init);
module_exit(b3dfg_module_exit);

