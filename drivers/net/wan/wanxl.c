/*
 * wanXL serial card driver for Linux
 * host part
 *
 * Copyright (C) 2003 Krzysztof Halasa <khc@pm.waw.pl>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 *
 * Status:
 *   - Only DTE (external clock) support with NRZ and NRZI encodings
 *   - wanXL100 will require minor driver modifications, no access to hw
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <linux/hdlc.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <asm/io.h>

#include "wanxl.h"

static const char* version = "wanXL serial card driver version: 0.48";

#define PLX_CTL_RESET   0x40000000 /* adapter reset */

#undef DEBUG_PKT
#undef DEBUG_PCI

/* MAILBOX #1 - PUTS COMMANDS */
#define MBX1_CMD_ABORTJ 0x85000000 /* Abort and Jump */
#ifdef __LITTLE_ENDIAN
#define MBX1_CMD_BSWAP  0x8C000001 /* little-endian Byte Swap Mode */
#else
#define MBX1_CMD_BSWAP  0x8C000000 /* big-endian Byte Swap Mode */
#endif

/* MAILBOX #2 - DRAM SIZE */
#define MBX2_MEMSZ_MASK 0xFFFF0000 /* PUTS Memory Size Register mask */


struct port {
	struct net_device *dev;
	struct card *card;
	spinlock_t lock;	/* for wanxl_xmit */
        int node;		/* physical port #0 - 3 */
	unsigned int clock_type;
	int tx_in, tx_out;
	struct sk_buff *tx_skbs[TX_BUFFERS];
};


struct card_status {
	desc_t rx_descs[RX_QUEUE_LENGTH];
	port_status_t port_status[4];
};


struct card {
	int n_ports;		/* 1, 2 or 4 ports */
	u8 irq;

	u8 __iomem *plx;	/* PLX PCI9060 virtual base address */
	struct pci_dev *pdev;	/* for pci_name(pdev) */
	int rx_in;
	struct sk_buff *rx_skbs[RX_QUEUE_LENGTH];
	struct card_status *status;	/* shared between host and card */
	dma_addr_t status_address;
	struct port ports[0];	/* 1 - 4 port structures follow */
};



static inline struct port *dev_to_port(struct net_device *dev)
{
	return (struct port *)dev_to_hdlc(dev)->priv;
}


static inline port_status_t *get_status(struct port *port)
{
	return &port->card->status->port_status[port->node];
}


#ifdef DEBUG_PCI
static inline dma_addr_t pci_map_single_debug(struct pci_dev *pdev, void *ptr,
					      size_t size, int direction)
{
	dma_addr_t addr = pci_map_single(pdev, ptr, size, direction);
	if (addr + size > 0x100000000LL)
		pr_crit("%s: pci_map_single() returned memory at 0x%llx!\n",
			pci_name(pdev), (unsigned long long)addr);
	return addr;
}

#undef pci_map_single
#define pci_map_single pci_map_single_debug
#endif


/* Cable and/or personality module change interrupt service */
static inline void wanxl_cable_intr(struct port *port)
{
	u32 value = get_status(port)->cable;
	int valid = 1;
	const char *cable, *pm, *dte = "", *dsr = "", *dcd = "";

	switch(value & 0x7) {
	case STATUS_CABLE_V35: cable = "V.35"; break;
	case STATUS_CABLE_X21: cable = "X.21"; break;
	case STATUS_CABLE_V24: cable = "V.24"; break;
	case STATUS_CABLE_EIA530: cable = "EIA530"; break;
	case STATUS_CABLE_NONE: cable = "no"; break;
	default: cable = "invalid";
	}

	switch((value >> STATUS_CABLE_PM_SHIFT) & 0x7) {
	case STATUS_CABLE_V35: pm = "V.35"; break;
	case STATUS_CABLE_X21: pm = "X.21"; break;
	case STATUS_CABLE_V24: pm = "V.24"; break;
	case STATUS_CABLE_EIA530: pm = "EIA530"; break;
	case STATUS_CABLE_NONE: pm = "no personality"; valid = 0; break;
	default: pm = "invalid personality"; valid = 0;
	}

	if (valid) {
		if ((value & 7) == ((value >> STATUS_CABLE_PM_SHIFT) & 7)) {
			dsr = (value & STATUS_CABLE_DSR) ? ", DSR ON" :
				", DSR off";
			dcd = (value & STATUS_CABLE_DCD) ? ", carrier ON" :
				", carrier off";
		}
		dte = (value & STATUS_CABLE_DCE) ? " DCE" : " DTE";
	}
	netdev_info(port->dev, "%s%s module, %s cable%s%s\n",
		    pm, dte, cable, dsr, dcd);

	if (value & STATUS_CABLE_DCD)
		netif_carrier_on(port->dev);
	else
		netif_carrier_off(port->dev);
}



/* Transmit complete interrupt service */
static inline void wanxl_tx_intr(struct port *port)
{
	struct net_device *dev = port->dev;
	while (1) {
                desc_t *desc = &get_status(port)->tx_descs[port->tx_in];
		struct sk_buff *skb = port->tx_skbs[port->tx_in];

		s
	intl