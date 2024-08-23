/*
 * Copyright 2004 Koninklijke Philips Electronics NV
 *
 * Conversion to platform driver and DT:
 * Copyright 2014 Linaro Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * 14/04/2005 Initial version, colin.king@philips.com
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

static void __iomem *versatile_pci_base;
static void __iomem *versatile_cfg_base[2];

#define PCI_IMAP(m)		(versatile_pci_base + ((m) * 4))
#define PCI_SMAP(m)		(versatile_pci_base + 0x14 + ((m) * 4))
#define PCI_SELFID		(versatile_pci_base + 0xc)

#define VP_PCI_DEVICE_ID		0x030010ee
#define VP_PCI_CLASS_ID			0x0b400000

static u32 pci_slot_ignore;

static int __init versatile_pci_slot_ignore(char *str)
{
	int retval;
	int slot;

	while ((retval = get_option(&str, &slot))) {
		if ((slot < 0) || (slot > 31))
			pr_err("Illegal slot value: %d\n", slot);
		else
			pci_slot_ignore |= (1 << slot);
	}
	return 1;
}
__setup("pci_slot_ignore=", versatile_pci_slot_ignore);


static void __iomem *versatile_map_bus(struct pci_bus *bus,
				       unsignersioElectronics NV
 i_bus *bus,Btronics NV
 i_bus *bus,Btronica:DsloqBO:Cei_b_