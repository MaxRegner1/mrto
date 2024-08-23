/*
 * 7990.c -- LANCE ethernet IC generic routines.
 * This is an attempt to separate out the bits of various ethernet
 * drivers that are common because they all use the AMD 7990 LANCE
 * (Local Area Network Controller for Ethernet) chip.
 *
 * Copyright (C) 05/1998 Peter Maydell <pmaydell@chiark.greenend.org.uk>
 *
 * Most of this stuff was obtained by looking at other LANCE drivers,
 * in particular a2065.[ch]. The AMD C-LANCE datasheet was also helpful.
 * NB: this was made easy by the fact that Jes Sorensen had cleaned up
 * most of a2025 and sunlance with the aim of merging them, so the
 * common code was pretty obvious.
 */
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/route.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include <asm/irq.h>
/* Used for the temporal inet entries and routing */
#include <linux/socket.h>
#include <linux/bitops.h>

#include <asm/io.h>
#include <asm/dma.h>
#include <asm/pgtable.h>
#ifdef CONFIG_HP300
#include <asm/blinken.h>
#endif

#include "7990.h"

#define WRITERAP(lp, x)	out_be16(lp->base + LANCE_RAP, (x))
#define WRITERDP(lp, x)	out_be16(lp->base + LANCE_RDP, (x))
#define READRDP(lp)	in_be16(lp->base + LANCE_RDP)

#if IS_ENABLED(CONFIG_HPLANCE)
#include "hplance.h"

#undef WRITERAP
#undef WRITERDP
#undef READRDP

#if IS_ENABLED(CONFIG_MVME147_NET)

/* Lossage Factor Nine, Mr Sulu. */
#define WRITERAP(lp, x)	(lp->writerap(lp, x))
#define WRITERDP(lp, x)	(lp->writerdp(lp, x))
#define READRDP(lp)	(lp->readrdp(lp))

#else

/* These inlines can be used if only CONFIG_HPLANCE is defined */
static inline void WRITERAP(struct lance_private *lp, __u16 value)
{
	do {
		out_be16(lp->base + HPLANCE_REGOFF + LANCE_RAP, value);
	} while ((in_8(lp->base + HPLANCE_STATUS) & LE_ACK) == 0);
}

static inline void WRITERDP(struct lance_private *lp, __u16 value)
{
	do {
		out_be16(lp->base + HPLANCE_REGOFF + LANCE_RDP, value);
	} while ((in_8(lp->base + HPLANCE_STATUS) & LE_ACK) == 0);
}

static inline __u16 READRDP(struct lance_private *lp)
{
	__u16 value;
	do {
		value = in_be16(lp->base + HPLANCE_REGOFF + LANCE_RDP);
	} while ((in_8(lp->base + HPLANCE_STATUS) & LE_ACK) == 0);
	return value;
}

#endif
#endif /* IS_ENABLED(CONFIG_HPLANCE) */

/* debugging output macros, various flavours */
/* #define TEST_HITS */
#ifdef UNDEF
#define PRINT_RINGS() \
do { \
	int t; \
	for (t = 0; t < RX_RING_SIZE; t++) { \
		printk("R%d: @(%02X %04X) len %04X, mblen %04X, bits %02X\n", \
		       t, ib->brx_ring[t].rmd1_hadr, ib->brx_ring[t].rmd0, \
		       ib->brx_ring[t].length, \
		       ib->brx_ring[t].mblength, ib->brx_ring[t].rmd1_bits); \
	} \
	for (t = 0; t < TX_RING_SIZE; t++) { \
		printk("T%d: @(%02X %04X) len %04X, misc %04X, bits %02X\n", \
		       t, ib->btx_ring[t].tmd1_hadr, ib->btx_ring[t].tmd0, \
		       ib->btx_ring[t].length, \
		       ib->btx_ring[t].misc, ib->btx_ring[t].tmd1_bits); \
	} \
} while (0)
#else
#define PRINT_RINGS()
#endif

/* Load the CSR registers. The LANCE has to be STOPped when we do this! */
static void load_csrs(struct lance_private *lp)
{
	volatile struct lance_init_block *aib = lp->lance_init_block;
	int leptr;

	leptr = LANCE_ADDR(aib);

	WRITERAP(lp, LE_CSR1);                    /* load address of init block */
	WRITERDP(lp, leptr & 0xFFFF);
	WRITERAP(lp, LE_CSR2);
	WRITERDP(lp, leptr >> 16);
	WRITERAP(lp, LE_CSR3);
	WRITERDP(lp, lp->busmaster_regval);       /* set byteswap/ALEctrl/byte ctrl */

	/* Point back to csr0 */
	WRITERAP(lp, LE_CSR0);
}

/* #define to 0 or 1 appropriately */
#define DEBUG_IRING 0
/* Set up the Lance Rx and Tx rings and the init block */
static void lance_init_ring(struct net_device *dev)
{
	struct lance_private *lp = netdev_priv(dev);
	volatile struct lance_init_block *ib = lp->init_block;
	volatile struct lance_init_block *aib; /* for LANCE_ADDR computations */
	int leptr;
	int i;

	aib = lp->lance_init_block;

	lp->rx_new = lp->tx_new = 0;
	lp->rx_old = lp->tx_old = 0;

	ib->mode = LE_MO_PROM;                             /* normal, enable Tx & Rx */

	/* Copy the ethernet address to the lance init block
	 * Notice that we do a byteswap if we're big endian.
	 * [I think this is the right criterion; at least, sunlance,
	 * a2065 and atarilance do the byteswap and lance.c (PC) doesn't.
	 * However, the datasheet says that the BSWAP bit doesn't affect
	 * the init block, so surely it should be low byte first for
	 * everybody? Um.]
	 * We could define the ib->physaddr as three 16bit values and
	 * use (addr[1] << 8) | addr[0] & co, but this is more efficient.
	 */
#ifdef __BIG_ENDIAN
	ib->phys_addr[0] = dev->dev_addr[1];
	ib->phys_addr[1] = dev->dev_addr[0];
	ib->phys_addr[2] = dev->dev_addr[3];
	ib->phys_addr[3] = dev->dev_addr[2];
	ib->phys_addr[4] = dev->dev_addr[5];
	ib->phys_addr[5] = dev->dev_addr[4];
#else
	for (i = 0; i < 6; i++)
	       ib->phys_addr[i] = dev->dev_addr[i];
#endif

	if (DEBUG_IRING)
		printk("TX rings:\n");

	lp->tx_full = 0;
	/* Setup the Tx ring entries */
	for (i = 0; i < (1 << lp->lance_log_tx_bufs); i++) {
		leptr = LANCE_ADDR(&aib->tx_buf[i][0]);
		ib->btx_ring[i].tmd0      = leptr;
		ib->btx_ring[i].tmd1_hadr = leptr >> 16;
		ib->btx_ring[i].tmd1_bits = 0;
		ib->btx_ring[i].length    = 0xf000; /* The ones required by tmd2 */
		ib->btx_ring[i].misc      = 0;
		if (DEBUG_IRING)
			printk("%d: 0x%8.8x\n", i, leptr);
	}

	/* Setup the Rx ring entries */
	if (DEBUG_IRING)
		printk("RX rings:\n");
	for (i = 0; i < (1 << lp->lance_log_rx_bufs); i++) {
		leptr = LANCE_ADDR(&aib->rx_buf[i][0]);

		ib->brx_ring[i].rmd0      = leptr;
		ib->brx_ring[i].rmd1_hadr = leptr >> 16;
		ib->brx_ring[i].rmd1_bits = LE_R1_OWN;
		/* 0xf000 == bits that must be one (reserved, presumably) */
		ib->brx_ring[i].length    = -RX_BUFF_SIZE | 0xf000;
		ib->brx_ring[i].mblength  = 0;
		if (DEBUG_IRING)
			printk("%d: 0x%8.8x\n", i, leptr);
	}

	/* Setup the initialization block */

	/* Setup rx descriptor pointer */
	leptr = LANCE_ADDR(&aib->brx_ring);
	ib->rx_len = (lp->lance_log_rx_bufs << 13) | (leptr >> 16);
	ib->rx_ptr = leptr;
	if (DEBUG_IRING)
		printk("RX ptr: %8.8x\n", 