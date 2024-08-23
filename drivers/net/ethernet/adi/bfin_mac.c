/*
 * Blackfin On-Chip MAC Driver
 *
 * Copyright 2004-2010 Analog Devices Inc.
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

#define DRV_VERSION	"1.1"
#define DRV_DESC	"Blackfin on-chip Ethernet MAC driver"

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/crc32.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <linux/platform_device.h>

#include <asm/dma.h>
#include <linux/dma-mapping.h>

#include <asm/div64.h>
#include <asm/dpmc.h>
#include <asm/blackfin.h>
#include <asm/cacheflush.h>
#include <asm/portmux.h>
#include <mach/pll.h>

#include "bfin_mac.h"

MODULE_AUTHOR("Bryan Wu, Luke Yang");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRV_DESC);
MODULE_ALIAS("platform:bfin_mac");

#if defined(CONFIG_BFIN_MAC_USE_L1)
# define bfin_mac_alloc(dma_handle, size, num)  l1_data_sram_zalloc(size*num)
# define bfin_mac_free(dma_handle, ptr, num)    l1_data_sram_free(ptr)
#else
# define bfin_mac_alloc(dma_handle, size, num) \
	dma_alloc_coherent(NULL, size*num, dma_handle, GFP_KERNEL)
# define bfin_mac_free(dma_handle, ptr, num) \
	dma_free_coherent(NULL, sizeof(*ptr)*num, ptr, dma_handle)
#endif

#define PKT_BUF_SZ 1580

#define MAX_TIMEOUT_CNT	500

/* pointers to maintain transmit list */
static struct net_dma_desc_tx *tx_list_head;
static struct net_dma_desc_tx *tx_list_tail;
static struct net_dma_desc_rx *rx_list_head;
static struct net_dma_desc_rx *rx_list_tail;
static struct net_dma_desc_rx *current_rx_ptr;
static struct net_dma_desc_tx *current_tx_ptr;
static struct net_dma_desc_tx *tx_desc;
static struct net_dma_desc_rx *rx_desc;

static void desc_list_free(void)
{
	struct net_dma_desc_rx *r;
	struct net_dma_desc_tx *t;
	int i;
#if !defined(CONFIG_BFIN_MAC_USE_L1)
	dma_addr_t dma_handle = 0;
#endif

	if (tx_desc) {
		t = tx_list_head;
		for (i = 0; i < CONFIG_BFIN_TX_DESC_NUM; i++) {
			if (t) {
				if (t->skb) {
					dev_kfree_skb(t->skb);
					t->skb = NULL;
				}
				t = t->next;
			}
		}
		bfin_mac_free(dma_handle, tx_desc, CONFIG_BFIN_TX_DESC_NUM);
	}

	if (rx_desc) {
		r = rx_list_head;
		for (i = 0; i < CONFIG_BFIN_RX_DESC_NUM; i++) {
			if (r) {
				if (r->skb) {
					dev_kfree_skb(r->skb);
					r->skb = NULL;
				}
				r = r->next;
			}
		}
		bfin_mac_free(dma_handle, rx_desc, CONFIG_BFIN_RX_DESC_NUM);
	}
}

static int desc_list_init(struct net_device *dev)
{
	int i;
	struct sk_buff *new_skb;
#if !defined(CONFIG_BFIN_MAC_USE_L1)
	/*
	 * This dma_handle is useless in Blackfin dma_alloc_coherent().
	 * The real dma handler is the return value of dma_alloc_coherent().
	 */
	dma_addr_t dma_handle;
#endif

	tx_desc = bfin_mac_alloc(&dma_handle,
				sizeof(struct net_dma_desc_tx),
				CONFIG_BFIN_TX_DESC_NUM);
	if (tx_desc == NULL)
		goto init_error;

	rx_desc = bfin_mac_alloc(&dma_handle,
				sizeof(struct net_dma_desc_rx),
				CONFIG_BFIN_RX_DESC_NUM);
	if (rx_desc == NULL)
		goto init_error;

	/* init tx_list */
	tx_list_head = tx_list_tail = tx_desc;

	for (i = 0; i < CONFIG_BFIN_TX_DESC_NUM; i++) {
		struct net_dma_desc_tx *t = tx_desc + i;
		struct dma_descriptor *a = &(t->desc_a);
		struct dma_descriptor *b = &(t->desc_b);

		/*
		 * disable DMA
		 * read from memory WNR = 0
		 * wordsize is 32 bits
		 * 6 half words is desc size
		 * large desc flow
		 */
		a->config = WDSIZE_32 | NDSIZE_6 | DMAFLOW_LARGE;
		a->start_addr = (unsigned long)t->packet;
		a->x_count = 0;
		a->next_dma_desc = b;

		/*
		 * enabled DMA
		 * write to memory WNR = 1
		 * wordsize is 32 bits
		 * disable interrupt
		 * 6 half words is desc size
		 * large desc flow
		 */
		b->config = DMAEN | WNR | WDSIZE_32 | NDSIZE_6 | DMAFLOW_LARGE;
		b->start_addr = (unsigned long)(&(t->status));
		b->x_count = 0;

		t->skb = NULL;
		tx_list_tail->desc_b.next_dma_desc = a;
		tx_list_tail->next = t;
		tx_list_tail = t;
	}
	tx_list_tail->next = tx_list_head;	/* tx_list is a circle */
	tx_list_tail->desc_b.next_dma_desc = &(tx_list_head->desc_a);
	current_tx_ptr = tx_list_head;

	/* init rx_list */
	rx_list_head = rx_list_tail = rx_desc;

	for (i = 0; i < CONFIG_BFIN_RX_DESC_NUM; i++) {
		struct net_dma_desc_rx *r = rx_desc + i;
		struct dma_descriptor *a = &(r->desc_a);
		struct dma_descriptor *b = &(r->desc_b);

		/* allocate a new skb for next time receive */
		new_skb = netdev_alloc_skb(dev, PKT_BUF_SZ + NET_IP_ALIGN);
		if (!new_skb)
			goto init_error;

		skb_reserve(new_skb, NET_IP_ALIGN);
		/* Invalidate the data cache of skb->data range when it is write back
		 * cache. It will prevent overwriting the new data from DMA
		 */
		blackfin_dcache_invalidate_range((unsigned long)new_skb->head,
					 (unsigned long)new_skb->end);
		r->skb = new_skb;

		/*
		 * enabled DMA
		 * write to memory WNR = 1
		 * wordsize is 32 bits
		 * disable interrupt
		 * 6 half words is desc size
		 * large desc flow
		 */
		a->config = DMAEN | WNR | WDSIZE_32 | NDSIZE_6 | DMAFLOW_LARGE;
		/* since RXDWA is enabled */
		a->start_addr = (unsigned long)new_skb->data - 2;
		a->x_count = 0;
		a->next_dma_desc = b;

		/*
		 * enabled DMA
		 * write to memory WNR = 1
		 * wordsize is 32 bits
		 * enable interrupt
		 * 6 half words is desc size
		 * large desc flow
		 */
		b->config = DMAEN | WNR | WDSIZE_32 | DI_EN |
				NDSIZE_6 | DMAFLOW_LARGE;
		b->start_addr = (unsigned long)(&(r->status));
		b->x_count = 0;

		rx_list_tail->desc_b.next_dma_desc = a;
		rx_list_tail->next = r;
		rx_list_tail = r;
	}
	rx_list_tail->next = rx_list_head;	/* rx_list is a circle */
	rx_list_tail->desc_b.next_dma_desc = &(rx_list_head->desc_a);
	current_rx_ptr = rx_list_head;

	return 0;

init_error:
	desc_list_free();
	pr_err("kmalloc failed\n");
	return -ENOMEM;
}


/*---PHY CONTROL AND CONFIGURATION-----------------------------------------*/

/*
 * MII operations
 */
/* Wait until the previous MDC/MDIO transaction has completed */
static int bfin_mdio_poll(void)
{
	int timeout_cnt = MAX_TIMEOUT_CNT;

	/* poll the STABUSY bit */
	while ((bfin_read_EMAC_STAADD()) & STABUSY) {
		udelay(1);
		if (timeout_cnt-- < 0) {
			pr_err("wait MDC/MDIO transaction to complete timeout\n");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

/* Read an off-chip register in a PHY through the MDC/MDIO port */
static int bfin_mdiobus_read(struct mii_bus *bus, int phy_addr, int regnum)
{
	int ret;

	ret = bfin_mdio_poll();
	if (ret)
		return ret;

	/* read mode */
	bfin_write_EMAC_STAADD(SET_PHYAD((u16) phy_addr) |
				SET_REGAD((u16) regnum) |
				STABUSY);

	ret = bfin_mdio_poll();
	if (ret)
		return ret;

	return (int) bfin_read_EMAC_STADAT();
}

/* Write an off-chip register in a PHY through the MDC/MDIO port */
static int bfin_mdiobus_write(struct mii_bus *bus, int phy_addr, int regnum,
			      u16 value)
{
	int ret;

	ret = bfin_mdio_poll();
	if (ret)
		return ret;

	bfin_write_EMAC_STADAT((u32) value);

	/* write mode */
	bfin_write_EMAC_STAADD(SET_PHYAD((u16) phy_addr) |
				SET_REGAD((u16) regnum) |
				STAOP |
				STABUSY);

	return bfin_mdio_poll();
}

static void bfin_mac_adjust_link(struct net_device *dev)
{
	struct bfin_mac_local *lp = netdev_priv(dev);
	struct phy_device *phydev = dev->phydev;
	unsigned long flags;
	int new_state = 0;

	spin_lock_irqsave(&lp->lock, flags);
	if (phydev->link) {
		/* Now we make sure that we can be in full duplex mode.
		 * If not, we operate in half-duplex mode. */
		if (phydev->duplex != lp->old_duplex) {
			u32 opmode = bfin_read_EMAC_OPMODE();
			new_state = 1;

			if (phydev->duplex)
				opmode |= FDMODE;
			else
				opmode &= ~(FDMODE);

			bfin_write_EMAC_OPMODE(opmode);
			lp->old_duplex = phydev->duplex;
		}

		if (phydev->speed != lp->old_speed) {
			if (phydev->interface == PHY_INTERFACE_MODE_RMII) {
				u32 opmode = bfin_read_EMAC_OPMODE();
				switch (phydev->speed) {
				case 10:
					opmode |= RMII_10;
					break;
				case 100:
					opmode &= ~RMII_10;
					break;
				default:
					netdev_warn(dev,
						"Ack! Speed (%d) is not 10/100!\n",
						phydev->speed);
					break;
				}
				bfin_write_EMAC_OPMODE(opmode);
			}

			new_state = 1;
			lp->old_speed = phydev->speed;
		}

		if (!lp->old_link) {
			new_state = 1;
			lp->old_link = 1;
		}
	} else if (lp->old_link) {
		new_state = 1;
		lp->old_link = 0;
		lp->old_speed = 0;
		lp->old_duplex = -1;
	}

	if (new_state) {
		u32 opmode = bfin_read_EMAC_OPMODE();
		phy_print_status(phydev);
		pr_debug("EMAC_OPMODE = 0x%08x\n", opmode);
	}

	spin_unlock_irqrestore(&lp->lock, flags);
}

/* MDC  = 2.5 MHz */
#define MDC_CLK 2500000

static int mii_probe(struct net_device *dev, int phy_mode)
{
	struct bfin_mac_local *lp = netdev_priv(dev);
	struct phy_device *phydev;
	unsigned short sysctl;
	u32 sclk, mdc_div;

	/* Enable PHY output early */
	if (!(bfin_read_VR_CTL() & CLKBUFOE))
		bfin_write_VR_CTL(bfin_read_VR_CTL() | CLKBUFOE);

	sclk = get_sclk();
	mdc_div = ((sclk / MDC_CLK) / 2) - 1;

	sysctl = bfin_read_EMAC_SYSCTL();
	sysctl = (sysctl & ~MDCDIV) | SET_MDCDIV(mdc_div);
	bfin_write_EMAC_SYSCTL(sysctl);

	phydev = phy_find_first(lp->mii_bus);
	if (!phydev) {
		netdev_err(dev, "no phy device found\n");
		return -ENODEV;
	}

	if (phy_mode != PHY_INTERFACE_MODE_RMII &&
		phy_mode != PHY_INTERFACE_MODE_MII) {
		netdev_err(dev, "invalid phy interface mode\n");
		return -EINVAL;
	}

	phydev = phy_connect(dev, phydev_name(phydev),
			     &bfin_mac_adjust_link, phy_mode);

	if (IS_ERR(phydev)) {
		netdev_err(dev, "could not attach PHY\n");
		return PTR_ERR(phydev);
	}

	/* mask with MAC supported features */
	phydev->supported &= (SUPPORTED_10baseT_Half
			      | SUPPORTED_10baseT_Full
			      | SUPPORTED_100baseT_Half
			      | SUPPORTED_100baseT_Full
			      | SUPPORTED_Autoneg
			      | SUPPORTED_Pause | SUPPORTED_Asym_Pause
			      | SUPPORTED_MII
			      | SUPPORTED_TP);

	phydev->advertising = phydev->supported;

	lp->old_link = 0;
	lp->old_speed = 0;
	lp->old_duplex = -1;

	phy_attached_print(phydev, "mdc_clk=%dHz(mdc_div=%d)@sclk=%dMHz)\n",
			   MDC_CLK, mdc_div, sclk / 1000000);

	return 0;
}

/*
 * Ethtool support
 */

/*
 * interrupt routine for magic packet wakeup
 */
static irqreturn_t bfin_mac_wake_interrupt(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static void bfin_mac_ethtool_getdrvinfo(struct net_device *dev,
					struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, KBUILD_MODNAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	strlcpy(info->fw_version, "N/A", sizeof(info->fw_version));
	strlcpy(info->bus_info, dev_name(&dev->dev), sizeof(info->bus_info));
}

static void bfin_mac_ethtool_getwolhtool_getdrvinfo(struct nevxLGk1s