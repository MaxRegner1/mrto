/*
 * Copyright (c) 2012-2017 Qualcomm Atheros, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/etherdevice.h>
#include <net/ieee80211_radiotap.h>
#include <linux/if_arp.h>
#include <linux/moduleparam.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <net/ipv6.h>
#include <linux/prefetch.h>

#include "wil6210.h"
#include "wmi.h"
#include "txrx.h"
#include "trace.h"

static bool rtap_include_phy_info;
module_param(rtap_include_phy_info, bool, 0444);
MODULE_PARM_DESC(rtap_include_phy_info,
		 " Include PHY info in the radiotap header, default - no");

bool rx_align_2;
module_param(rx_align_2, bool, 0444);
MODULE_PARM_DESC(rx_align_2, " align Rx buffers on 4*n+2, default - no");

bool rx_large_buf;
module_param(rx_large_buf, bool, 0444);
MODULE_PARM_DESC(rx_large_buf, " allocate 8KB RX buffers, default - no");

static inline uint wil_rx_snaplen(void)
{
	return rx_align_2 ? 6 : 0;
}

static inline int wil_vring_is_empty(struct vring *vring)
{
	return vring->swhead == vring->swtail;
}

static inline u32 wil_vring_next_tail(struct vring *vring)
{
	return (vring->swtail + 1) % vring->size;
}

static inline void wil_vring_advance_head(struct vring *vring, int n)
{
	vring->swhead = (vring->swhead + n) % vring->size;
}

static inline int wil_vring_is_full(struct vring *vring)
{
	return wil_vring_next_tail(vring) == vring->swhead;
}

/* Used space in Tx Vring */
static inline int wil_vring_used_tx(struct vring *vring)
{
	u32 swhead = vring->swhead;
	u32 swtail = vring->swtail;
	return (vring->size + swhead - swtail) % vring->size;
}

/* Available space in Tx Vring */
static inline int wil_vring_avail_tx(struct vring *vring)
{
	return vring->size - wil_vring_used_tx(vring) - 1;
}

/* wil_vring_wmark_low - low watermark for available descriptor space */
static inline int wil_vring_wmark_low(struct vring *vring)
{
	return vring->size/8;
}

/* wil_vring_wmark_high - high watermark for available descriptor space */
static inline int wil_vring_wmark_high(struct vring *vring)
{
	return vring->size/4;
}

/* returns true if num avail descriptors is lower than wmark_low */
static inline int wil_vring_avail_low(struct vring *vring)
{
	return wil_vring_avail_tx(vring) < wil_vring_wmark_low(vring);
}

/* returns true if num avail descriptors is higher than wmark_high */
static inline int wil_vring_avail_high(struct vring *vring)
{
	return wil_vring_avail_tx(vring) > wil_vring_wmark_high(vring);
}

/* returns true when all tx vrings are empty */
bool wil_is_tx_idle(struct wil6210_priv *wil)
{
	int i;
	unsigned long data_comp_to;

	for (i = 0; i < WIL6210_MAX_TX_RINGS; i++) {
		struct vring *vring = &wil->vring_tx[i];
		int vring_index = vring - wil->vring_tx;
		struct vring_tx_data *txdata = &wil->vring_tx_data[vring_index];

		spin_lock(&txdata->lock);

		if (!vring->va || !txdata->enabled) {
			spin_unlock(&txdata->lock);
			continue;
		}

		data_comp_to = jiffies + msecs_to_jiffies(
					WIL_DATA_COMPLETION_TO_MS);
		if (test_bit(wil_status_napi_en, wil->status)) {
			while (!wil_vring_is_empty(vring)) {
				if (time_after(jiffies, data_comp_to)) {
					wil_dbg_pm(wil,
						   "TO waiting for idle tx\n");
					spin_unlock(&txdata->lock);
					return false;
				}
				wil_dbg_ratelimited(wil,
						    "tx vring is not empty -> NAPI\n");
				spin_unlock(&txdata->lock);
				napi_synchronize(&wil->napi_tx);
				msleep(20);
				spin_lock(&txdata->lock);
				if (!vring->va || !txdata->enabled)
					break;
			}
		}

		spin_unlock(&txdata->lock);
	}

	return true;
}

/* wil_val_in_range - check if value in [min,max) */
static inline bool wil_val_in_range(int val, int min, int max)
{
	return val >= min && val < max;
}

static int wil_vring_alloc(struct wil6210_priv *wil, struct vring *vring)
{
	struct device *dev = wil_to_dev(wil);
	size_t sz = vring->size * sizeof(vring->va[0]);
	uint i;

	wil_dbg_misc(wil, "vring_alloc:\n");

	BUILD_BUG_ON(sizeof(vring->va[0]) != 32);

	vring->swhead = 0;
	vring->swtail = 0;
	vring->ctx = kcalloc(vring->size, sizeof(vring->ctx[0]), GFP_KERNEL);
	if (!vring->ctx) {
		vring->va = NULL;
		return -ENOMEM;
	}

	/* vring->va should be aligned on its size rounded up to power of 2
	 * This is granted by the dma_alloc_coherent.
	 *
	 * HW has limitation that all vrings addresses must share the same
	 * upper 16 msb bits part of 48 bits address. To workaround that,
	 * if we are using 48 bit addresses switch to 32 bit allocation
	 * before allocating vring memory.
	 *
	 * There's no check for the return value of dma_set_mask_and_coherent,
	 * since we assume if we were able to set the mask during
	 * initialization in this system it will not fail if we set it again
	 */
	if (wil->use_extended_dma_addr)
		dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));

	vring->va = dma_alloc_coherent(dev, sz, &vring->pa, GFP_KERNEL);
	if (!vring->va) {
		kfree(vring->ctx);
		vring->ctx = NULL;
		return -ENOMEM;
	}

	if (wil->use_extended_dma_addr)
		dma_set_mask_and_coherent(dev, DMA_BIT_MASK(48));

	/* initially, all descriptors are SW owned
	 * For Tx and Rx, ownership bit is at the same location, thus
	 * we can use any
	 */
	for (i = 0; i < vring->size; i++) {
		volatile struct vring_tx_desc *_d = &vring->va[i].tx;

		_d->dma.status = TX_DMA_STATUS_DU;
	}

	wil_dbg_misc(wil, "vring[%d] 0x%p:%pad 0x%p\n", vring->size,
		     vring->va, &vring->pa, vring->ctx);

	return 0;
}

static void wil_txdesc_unmap(struct device *dev, struct vring_tx_desc *d,
			     struct wil_ctx *ctx)
{
	dma_addr_t pa = wil_desc_addr(&d->dma.addr);
	u16 dmalen = le16_to_cpu(d->dma.length);

	switch (ctx->mapped_as) {
	case wil_mapped_as_single:
		dma_unmap_single(dev, pa, dmalen, DMA_TO_DEVICE);
		break;
	case wil_mapped_as_page:
		dma_unmap_page(dev, pa, dmalen, DMA_TO_DEVICE);
		break;
	default:
		break;
	}
}

static void wil_vring_free(struct wil6210_priv *wil, struct vring *vring,
			   int tx)
{
	struct device *dev = wil_to_dev(wil);
	size_t sz = vring->size * sizeof(vring->va[0]);

	lockdep_assert_held(&wil->mutex);
	if (tx) {
		int vring_index = vring - wil->vring_tx;

		wil_dbg_misc(wil, "free Tx vring %d [%d] 0x%p:%pad 0x%p\n",
			     vring_index, vring->size, vring->va,
			     &vring->pa, vring->ctx);
	} else {
		wil_dbg_misc(wil, "free Rx vring [%d] 0x%p:%pad 0x%p\n",
			     vring->size, vring->va,
			     &vring->pa, vring->ctx);
	}

	while (!wil_vring_is_empty(vring)) {
		dma_addr_t pa;
		u16 dmalen;
		struct wil_ctx *ctx;

		if (tx) {
			struct vring_tx_desc dd, *d = &dd;
			volatile struct vring_tx_desc *_d =
					&vring->va[vring->swtail].tx;

			ctx = &vring->ctx[vring->swtail];
			if (!ctx) {
				wil_dbg_txrx(wil,
					     "ctx(%d) was already completed\n",
					     vring->swtail);
				vring->swtail = wil_vring_next_tail(vring);
				continue;
			}
			*d = *_d;
			wil_txdesc_unmap(dev, d, ctx);
			if (ctx->skb)
				dev_kfree_skb_any(ctx->skb);
			vring->swtail = wil_vring_next_tail(vring);
		} else { /* rx */
			struct vring_rx_desc dd, *d = &dd;
			volatile struct vring_rx_desc *_d =
					&vring->va[vring->swhead].rx;

			ctx = &vring->ctx[vring->swhead];
			*d = *_d;
			pa = wil_desc_addr(&d->dma.addr);
			dmalen = le16_to_cpu(d->dma.length);
			dma_unmap_single(dev, pa, dmalen, DMA_FROM_DEVICE);
			kfree_skb(ctx->skb);
			wil_vring_advance_head(vring, 1);
		}
	}
	dma_free_coherent(dev, sz, (void *)vring->va, vring->pa);
	kfree(vring->ctx);
	vring->pa = 0;
	vring->va = NULL;
	vring->ctx = NULL;
}

/**
 * Allocate one skb for Rx VRING
 *
 * Safe to call from IRQ
 */
static int wil_vring_alloc_skb(struct wil6210_priv *wil, struct vring *vring,
			       u32 i, int headroom)
{
	struct device *dev = wil_to_dev(wil);
	unsigned int sz = wil->rx_buf_len + ETH_HLEN + wil_rx_snaplen();
	struct vring_rx_desc dd, *d = &dd;
	volatile struct vring_rx_desc *_d = &vring->va[i].rx;
	dma_addr_t pa;
	struct sk_buff *skb = dev_alloc_skb(sz + headroom);

	if (unlikely(!skb))
		return -ENOMEM;

	skb_reserve(skb, headroom);
	skb_put(skb, sz);

	pa = dma_map_single(dev, skb->data, skb->len, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(dev, pa))) {
		kfree_skb(skb);
		return -ENOMEM;
	}

	d->dma.d0 = RX_DMA_D0_CMD_DMA_RT | RX_DMA_D0_CMD_DMA_IT;
	wil_desc_addr_set(&d->dma.addr, pa);
	/* ip_length don't care */
	/* b11 don't care */
	/* error don't care */
	d->dma.status = 0; /* BIT(0) should be 0 for HW_OWNED */
	d->dma.length = cpu_to_le16(sz);
	*_d = *d;
	vring->ctx[i].skb = skb;

	return 0;
}

/**
 * Adds radiotap header
 *
 * Any error indicated as "Bad FCS"
 *
 * Vendor data for 04:ce:14-1 (Wilocity-1) consists of:
 *  - Rx descriptor: 32 bytes
 *  - Phy info
 */
static void wil_rx_add_radiotap_header(struct wil6210_priv *wil,
				       struct sk_buff *skb)
{
	struct wireless_dev *wdev = wil->wdev;
	struct wil6210_rtap {
		struct ieee80211_radiotap_header rthdr;
		/* fields should be in the order of bits in rthdr.it_present */
		/* flags */
		u8 flags;
		/* channel */
		__le16 chnl_freq __aligned(2);
		__le16 chnl_flags;
		/* MCS */
		u8 mcs_present;
		u8 mcs_flags;
		u8 mcs_index;
	} __packed;
	struct wil6210_rtap_vendor {
		struct wil6210_rtap rtap;
		/* vendor */
		u8 vendor_oui[3] __aligned(2);
		u8 vendor_ns;
		__le16 vendor_skip;
		u8 vendor_data[0];
	} __packed;
	struct vring_rx_desc *d = wil_skb_rxdesc(skb);
	struct wil6210_rtap_vendor *rtap_vendor;
	int rtap_len = sizeof(struct wil6210_rtap);
	int phy_length = 0; /* phy info header size, bytes */
	static char phy_data[128];
	struct ieee80211_channel *ch = wdev->preset_chandef.chan;

	if (rtap_include_phy_info) {
		rtap_len = sizeof(*rtap_vendor) + sizeof(*d);
		/* calculate additional length */
		if (d->dma.status & RX_DMA_STATUS_PHY_INFO) {
			/**
			 * PHY info starts from 8-byte boundary
			 * there are 8-byte lines, last line may be partially
			 * written (HW bug), thus FW configures for last line
			 * to be excessive. Driver skips this last line.
			 */
			int len = min_t(int, 8 + sizeof(phy_data),
					wil_rxdesc_phy_length(d));

			if (len > 8) {
				void *p = skb_tail_pointer(skb);
				void *pa = PTR_ALIGN(p, 8);

				if (skb_tailroom(skb) >= len + (pa - p)) {
					phy_length = len - 8;
					memcpy(phy_data, pa, phy_length);
				}
			}
		}
		rtap_len += phy_length;
	}

	if (skb_headroom(skb) < rtap_len &&
	    pskb_expand_head(skb, rtap_len, 0, GFP_ATOMIC)) {
		wil_err(wil, "Unable to expand headroom to %d\n", rtap_len);
		return;
	}

	rtap_vendor = skb_push(skb, rtap_len);
	memset(rtap_vendor, 0, rtap_len);

	rtap_vendor->rtap.rthdr.it_version = PKTHDR_RADIOTAP_VERSION;
	rtap_vendor->rtap.rthdr.it_len = cpu_to_le16(rtap_len);
	rtap_vendor->rtap.rthdr.it_present = cpu_to_le32(
			(1 << IEEE80211_RADIOTAP_FLAGS) |
			(1 << IEEE80211_RADIOTAP_CHANNEL) |
			(1 << IEEE80211_RADIOTAP_MCS));
	if (d->dma.status & RX_DMA_STATUS_ERROR)
		rtap_vendor->rtap.flags |= IEEE80211_RADIOTAP_F_BADFCS;

	rtap_vendor->rtap.chnl_freq = cpu_to_le16(ch ? ch->center_freq : 58320);
	rtap_vendor->rtap.chnl_flags = cpu_to_le16(0);

	rtap_vendor->rtap.mcs_present = IEEE80211_RADIOTAP_MCS_HAVE_MCS;
	rtap_vendor->rtap.mcs_flags = 0;
	rtap_vendor->rtap.mcs_index = wil_rxdesc_mcs(d);

	if (rtap_include_phy_info) {
		rtap_vendor->rtap.rthdr.it_present |= cpu_to_le32(1 <<
				IEEE80211_RADIOTAP_VENDOR_NAMESPACE);
		/* OUI for Wilocity 04:ce:14 */
		rtap_vendor->vendor_oui[0] = 0x04;
		rtap_vendor->vendor_oui[1] = 0xce;
		rtap_vendor->vendor_oui[2] = 0x14;
		rtap_vendor->vendor_ns = 1;
		/* Rx descriptor + PHY data  */
		rtap_vendor->vendor_skip = cpu_to_le16(sizeof(*d) +
						       phy_length);
		memcpy(rtap_vendor->vendor_data, (void *)d, sizeof(*d));
		memcpy(rtap_vendor->vendor_data + sizeof(*d), phy_data,
		       phy_length);
	}
}

/* similar to ieee80211_ version, but FC contain only 1-st byte */
static inline int wil_is_back_req(u8 fc)
{
	return (fc & (IEEE80211_FCTL_FTYPE | IEEE80211_FCTL_STYPE)) ==
	       (IEEE80211_FTYPE_CTL | IEEE80211_STYPE_BACK_REQ);
}

bool wil_is_rx_idle(struct wil6210_priv *wil)
{
	struct vring_rx_desc *_d;
	struct vring *vring = &wil->vring_rx;

	_d = (struct vring_rx_desc *)&vring->va[vring->swhead].rx;
	if (_d->dma.status & RX_DMA_STATUS_DU)
		return false;

	return true;
}

/**
 * reap 1 frame from @swhead
 *
 * Rx descriptor copied to skb->cb
 *
 * Safe to call from IRQ
 */
static struct sk_buff *wil_vring_reap_rx(struct wil6210_priv *wil,
					 struct vring *vring)
{
	struct device *dev = wil_to_dev(wil);
	struct net_device *ndev = wil_to_ndev(wil);
	volatile struct vring_rx_desc *_d;
	struct vring_rx_desc *d;
	struct sk_buff *skb;
	dma_addr_t pa;
	unsigned int snaplen = wil_rx_snaplen();
	unsigned int sz = wil->rx_buf_len + ETH_HLEN + snaplen;
	u16 dmalen;
	u8 ftype;
	int cid;
	int i;
	struct wil_net_stats *stats;

	BUILD_BUG_ON(sizeof(struct vring_rx_desc) > sizeof(skb->cb));

again:
	if (unlikely(wil_vring_is_empty(vring)))
		return NULL;

	i = (int)vring->swhead;
	_d = &vring->va[i].rx;
	if (unlikely(!(_d->dma.status & RX_DMA_STATUS_DU))) {
		/* it is not error, we just reached end of Rx done area */
		return NULL;
	}

	skb = vring->ctx[i].skb;
	vring->ctx[i].skb = NULL;
	wil_vring_advance_head(vring, 1);
	if (!skb) {
		wil_err(wil, "No Rx skb at [%d]\n", i);
		goto again;
	}
	d = wil_skb_rxdesc(skb);
	*d = *_d;
	pa = wil_desc_addr(&d->dma.addr);

	dma_unmap_single(dev, pa, sz, DMA_FROM_DEVICE);
	dmalen = le16_to_cpu(d->dma.length);

	trace_wil6210_rx(i, d);
	wil_dbg_txrx(wil, "Rx[%3d] : %d bytes\n", i, dmalen);
	wil_hex_dump_txrx("RxD ", DUMP_PREFIX_NONE, 32, 4,
			  (const void *)d, sizeof(*d), false);

	cid = wil_rxdesc_cid(d);
	stats = &wil->sta[cid].stats;

	if (unlikely(dmalen > sz)) {
		wil_err(wil, "Rx size too large: %d bytes!\n", dmalen);
		stats->rx_large_frame++;
		kfree_skb(skb);
		goto again;
	}
	skb_trim(skb, dmalen);

	prefetch(skb->data);

	wil_hex_dump_txrx("Rx ", DUMP_PREFIX_OFFSET, 16, 1,
			  skb->data, skb_headlen(skb), false);

	stats->last_mcs_rx = wil_rxdesc_mcs(d);
	if (stats->last_mcs_rx < ARRAY_SIZE(stats->rx_per_mcs))
		stats->rx_per_mcs[stats->last_mcs_rx]++;

	/* use radiotap header only if required */
	if (ndev->type == ARPHRD_IEEE80211_RADIOTAP)
		wil_rx_add_radiotap_header(wil, skb);

	/* no extra checks if in sniffer mode */
	if (ndev->type != ARPHRD_ETHER)
		return skb;
	/* Non-data frames may be delivered through Rx DMA channel (ex: BAR)
	 * Driver should recognize it by frame type, that is found
	 * in Rx descriptor. If type is not data, it is 802.11 frame as is
	 */
	ftype = wil_rxdesc_ftype(d) << 2;
	if (unlikely(ftype != IEEE80211_FTYPE_DATA)) {
		u8 fc1 = wil_rxdesc_fc1(d);
		int mid = wil_rxdesc_mid(d);
		int tid = wil_rxdesc_tid(d);
		u16 seq = wil_rxdesc_seq(d);

		wil_dbg_txrx(wil,
			     "Non-data frame FC[7:0] 0x%02x MID %d CID %d TID %d Seq 0x%03x\n",
			     fc1, mid, cid, tid, seq);
		stats->rx_non_data_frame++;
		if (wil_is_back_req(fc1)) {
			wil_dbg_txrx(wil,
				     "BAR: MID %d CID %d TID %d Seq 0x%03x\n",
				     mid, cid, tid, seq);
			wil_rx_bar(wil, cid, tid, seq);
		} else {
			/* print again all info. One can enable only this
			 * without overhead for printing every Rx frame
			 */
			wil_dbg_txrx(wil,
				     "Unhandled non-data frame FC[7:0] 0x%02x MID %d CID %d TID %d Seq 0x%03x\n",
				     fc1, mid, cid, tid, seq);
			wil_hex_dump_txrx("RxD ", DUMP_PREFIX_NONE, 32, 4,
					  (const void *)d, sizeof(*d), false);
			wil_hex_dump_txrx("Rx ", DUMP_PREFIX_OFFSET, 16, 1,
					  skb->data, skb_headlen(skb), false);
		}
		kfree_skb(skb);
		goto again;
	}

	if (unlikely(skb->len < ETH_HLEN + snaplen)) {
		wil_err(wil, "Short frame, len = %d\n", skb->len);
		stats->rx_short_frame++;
		kfree_skb(skb);
		goto again;
	}

	/* L4 IDENT is on when HW calculated checksum, check status
	 * and in case of error drop the packet
	 * higher stack layers will handle retransmission (if required)
	 */
	if (likely(d->dma.status & RX_DMA_STATUS_L4I)) {
		/* L4 protocol identified, csum calculated */
		if (likely((d->dma.error & RX_DMA_ERROR_L4_ERR) == 0))
			skb->ip_summed = CHECKSUM_UNNECESSARY;
		/* If HW reports bad checksum, let IP stack re-check it
		 * For example, HW don't understand Microsoft IP stack that
		 * mis-calculates TCP checksum - if it should be 0x0,
		 * it writes 0xffff in violation of RFC 1624
		 */
	}

	if (snaplen) {
		/* Packet layout
		 * +-------+-------+---------+------------+------+
		 * | SA(6) | DA(6) | SNAP(6) | ETHTYPE(2) | DATA |
		 * +-------+-------+---------+------------+------+
		 * Need to remove SNAP, shifting SA and DA forward
		 */
		memmove(skb->data + snaplen, skb->data, 2 * ETH_ALEN);
		skb_pull(skb, snaplen);
	}

	return skb;
}

/**
 * allocate and fill up to @count buffers in rx ring
 * buffers posted at @swtail
 */
static int wil_rx_refill(struct wil6210_priv *wil, int count)
{
	struct net_device *ndev = wil_to_ndev(wil);
	struct vring *v = &wil->vring_rx;
	u32 next_tail;
	int rc = 0;
	int headroom = ndev->type == ARPHRD_IEEE80211_RADIOTAP ?
			WIL6210_RTAP_SIZE : 0;

	for (; next_tail = wil_vring_next_tail(v),
			(next_tail != v->swhead) && (count-- > 0);
			v->swtail = next_tail) {
		rc = wil_vring_alloc_skb(wil, v, v->swtail, headroom);
		if (unlikely(rc)) {
			wil_err_ratelimited(wil, "Error %d in rx refill[%d]\n",
					    rc, v->swtail);
			break;
		}
	}

	/* make sure all writes to descriptors (shared memory) are done before
	 * committing them to HW
	 */
	wmb();

	wil_w(wil, v->hwtail, v->swtail);

	return rc;
}

/**
 * reverse_memcmp - Compare two areas of memory, in reverse order
 * @cs: One area of memory
 * @ct: Another area of memory
 * @count: The size of the area.
 *
 * Cut'n'paste from original memcmp (see lib/string.c)
 * with minimal modifications
 */
static int reverse_memcmp(const void *cs, const void *ct, size_t count)
{
	const unsigned char *su1, *su2;
	int res = 0;

	for (su1 = cs + count - 1, su2 = ct + count - 1; count > 0;
	     --su1, --su2, count--) {
		res = *su1 - *su2;
		if (res)
			break;
	}
	return res;
}

static int wil_rx_crypto_check(struct wil6210_priv *wil, struct sk_buff *skb)
{
	struct vring_rx_desc *d = wil_skb_rxdesc(skb);
	int cid = wil_rxdesc_cid(d);
	int tid = wil_rxdesc_tid(d);
	int key_id = wil_rxdesc_key_id(d);
	int mc = wil_rxdesc_mcast(d);
	struct wil_sta_info *s = &wil->sta[cid];
	struct wil_tid_crypto_rx *c = mc ? &s->group_crypto_rx :
				      &s->tid_crypto_rx[tid];
	struct wil_tid_crypto_rx_single *cc = &c->key_id[key_id];
	const u8 *pn = (u8 *)&d->mac.pn_15_0;

	if (!cc->key_set) {
		wil_err_ratelimited(wil,
				    "Key missing. CID %d TID %d MCast %d KEY_ID %d\n",
				    cid, tid, mc, key_id);
		return -EINVAL;
	}

	if (reverse_memcmp(pn, cc->pn, IEEE80211_GCMP_PN_LEN) <= 0) {
		wil_err_ratelimited(wil,
				    "Replay attack. CID %d TID %d MCast %d KEY_ID %d PN %6phN last %6phN\n",
				    cid, tid, mc, key_id, pn, cc->pn);
		return -EINVAL;
	}
	memcpy(cc->pn, pn, IEEE80211_GCMP_PN_LEN);

	return 0;
}

/*
 * Pass Rx packet to the netif. Update statistics.
 * Called in softirq context (NAPI poll).
 */
void wil_netif_rx_any(struct sk_buff *skb, struct net_device *ndev)
{
	gro_result_t rc = GRO_NORMAL;
	struct wil6210_priv *wil = ndev_to_wil(ndev);
	struct wireless_dev *wdev = wil_to_wdev(wil);
	unsigned int len = skb->len;
	struct vring_rx_desc *d = wil_skb_rxdesc(skb);
	int cid = wil_rxdesc_cid(d); /* always 0..7, no need to check */
	int security = wil_rxdesc_security(d);
	struct ethhdr *eth = (void *)skb->data;
	/* here looking for DA, not A1, thus Rxdesc's 'mcast' indication
	 * is not suitable, need to look at data
	 */
	int mcast = is_multicast_ether_addr(eth->h_dest);
	struct wil_net_stats *stats = &wil->sta[cid].stats;
	struct sk_buff *xmit_skb = NULL;
	static const char * const gro_res_str[] = {
		[GRO_MERGED]		= "GRO_MERGED",
		[GRO_MERGED_FREE]	= "GRO_MERGED_FREE",
		[GRO_HELD]		= "GRO_HELD",
		[GRO_NORMAL]		= "GRO_NORMAL",
		[GRO_DROP]		= "GRO_DROP",
		[GRO_CONSUMED]		= "GRO_CONSUMED",
	};

	if (ndev->features & NETIF_F_RXHASH)
		/* fake L4 to ensure it won't be re-calculated later
		 * set hash to any non-zero value to activate rps
		 * mechanism, core will be chosen according
		 * to user-level rps configuration.
		 */
		skb_set_hash(skb, 1, PKT_HASH_TYPE_L4);

	skb_orphan(skb);

	if (security && (wil_rx_crypto_check(wil, skb) != 0)) {
		rc = GRO_DROP;
		dev_kfree_skb(skb);
		stats->rx_replay++;
		goto stats;
	}

	if (wdev->iftype == NL80211_IFTYPE_AP && !wil->ap_isolate) {
		if (mcast) {
			/* send multicast frames both to higher layers in
			 * local net stack and back to the wireless medium
			 */
			xmit_skb = skb_copy(skb, GFP_ATOMIC);
		} else {
			int xmit_cid = wil_find_cid(wil, eth->h_dest);

			if (xmit_cid >= 0) {
				/* The destination station is associated to
				 * this AP (in this VLAN), so send the frame
				 * directly to it and do not pass it to local
				 * net stack.
				 */
				xmit_skb = skb;
				skb = NULL;
			}
		}
	}
	if (xmit_skb) {
		/* Send to wireless media and increase priority by 256 to
		 * keep the received priority instead of reclassifying
		 * the frame (see cfg80211_classify8021d).
		 */
		xmit_skb->dev = ndev;
		xmit_skb->priority += 256;
		xmit_skb->protocol = htons(ETH_P_802_3);
		skb_reset_network_header(xmit_skb);
		skb_reset_mac_header(xmit_skb);
		wil_dbg_txrx(wil, "Rx -> Tx %d bytes\n", len);
		dev_queue_xmit(xmit_skb);
	}

	if (skb) { /* deliver to local stack */

		skb->protocol = eth_type_trans(skb, ndev);
		rc = napi_gro_receive(&wil->napi_rx, skb);
		wil_dbg_txrx(wil, "Rx complete %d bytes => %s\n",
			     len, gro_res_str[rc]);
	}
stats:
	/* statistics. rc set to GRO_NORMAL for AP bridging */
	if (unlikely(rc == GRO_DROP)) {
		ndev->stats.rx_dropped++;
		stats->rx_dropped++;
		wil_dbg_txrx(wil, "Rx drop %d bytes\n", len);
	} else {
		ndev->stats.rx_packets++;
		stats->rx_packets++;
		ndev->stats.rx_bytes += len;
		stats->rx_bytes += len;
		if (mcast)
			ndev->stats.multicast++;
	}
}

/**
 * Proceed all completed skb's from Rx VRING
 *
 * Safe to call from NAPI poll, i.e. softirq with interrupts enabled
 */
void wil_rx_handle(struct wil6210_priv *wil, int *quota)
{
	struct net_device *ndev = wil_to_ndev(wil);
	struct vring *v = &wil->vring_rx;
	struct sk_buff *skb;

	if (unlikely(!v->va)) {
		wil_err(wil, "Rx IRQ while Rx not yet initialized\n");
		return;
	}
	wil_dbg_txrx(wil, "rx_handle\n");
	while ((*quota > 0) && (NULL != (skb = wil_vring_reap_rx(wil, v)))) {
		(*quota)--;

		if (wil->wdev->iftype == NL80211_IFTYPE_MONITOR) {
			skb->dev = ndev;
			skb_reset_mac_header(skb);
			skb->ip_summed = CHECKSUM_UNNECESSARY;
			skb->pkt_type = PACKET_OTHERHOST;
			skb->protocol = htons(ETH_P_802_2);
			wil_netif_rx_any(skb, ndev);
		} else {
			wil_rx_reorder(wil, skb);
		}
	}
	wil_rx_refill(wil, v->size);
}

static void wil_rx_buf_len_init(struct wil6210_priv *wil)
{
	wil->rx_buf_len = rx_large_buf ?
		WIL_MAX_ETH_MTU : TXRX_BUF_LEN_DEFAULT - WIL_MAX_MPDU_OVERHEAD;
	if (mtu_max > wil->rx_buf_len) {
		/* do not allow RX buffers to be smaller than mtu_max, for
		 * backward compatibility (mtu_max parameter was also used
		 * to support receiving large packets)
		 */
		wil_info(wil, "Override RX buffer to mtu_max(%d)\n", mtu_max);
		wil->rx_buf_len = mtu_max;
	}
}

int wil_rx_init(struct wil6210_priv *wil, u16 size)
{
	struct vring *vring = &wil->vring_rx;
	int rc;

	wil_dbg_misc(wil, "rx_init\n");

	if (vring->va) {
		wil_err(wil, "Rx ring already allocated\n");
		return -EINVAL;
	}

	wil_rx_buf_len_init(wil);

	vring->size = size;
	rc = wil_vring_alloc(wil, vring);
	if (rc)
		return rc;

	rc = wmi_rx_chain_add(wil, vring);
	if (rc)
		goto err_free;

	rc = wil_rx_refill(wil, vring->size);
	if (rc)
		goto err_free;

	return 0;
 err_free:
	wil_vring_free(wil, vring, 0);

	return rc;
}

void wil_rx_fini(struct wil6210_priv *wil)
{
	struct vring *vring = &wil->vring_rx;

	wil_dbg_misc(wil, "rx_fini\n");

	if (vring->va)
		wil_vring_free(wil, vring, 0);
}

static inline void wil_tx_data_init(struct vring_tx_data *txdata)
{
	spin_lock_bh(&txdata->lock);
	txdata->dot1x_open = 0;
	txdata->enabled = 0;
	txdata->idle = 0;
	txdata->last_idle = 0;
	txdata->begin = 0;
	txdata->agg_wsize = 0;
	txdata->agg_timeout = 0;
	txdata->agg_amsdu = 0;
	txdata->addba_in_progress = false;
	spin_unlock_bh(&txdata->lock);
}

int wil_vring_init_tx(struct wil6210_priv *wil, int id, int size,
		      int cid, int tid)
{
	int rc;
	struct wmi_vring_cfg_cmd cmd = {
		.action = cpu_to_le32(WMI_VRING_CMD_ADD),
		.vring_cfg = {
			.tx_sw_ring = {
				.max_mpdu_size =
					cpu_to_le16(wil_mtu2macbuf(mtu_max)),
				.ring_size = cpu_to_le16(size),
			},
			.ringid = id,
			.cidxtid = mk_cidxtid(cid, tid),
			.encap_trans_type = WMI_VRING_ENC_TYPE_802_3,
			.mac_ctrl = 0,
			.to_resolution = 0,
			.agg_max_wsize = 0,
			.schd_params = {
				.priority = cpu_to_le16(0),
				.timeslot_us = cpu_to_le16(0xfff),
			},
		},
	};
	struct {
		struct wmi_cmd_hdr wmi;
		struct wmi_vring_cfg_done_event cmd;
	} __packed reply;
	struct vring *vring = &wil->vring_tx[id];
	struct vring_tx_data *txdata = &wil->vring_tx_data[id];

	wil_dbg_misc(wil, "vring_init_tx: max_mpdu_size %d\n",
		     cmd.vring_cfg.tx_sw_ring.max_mpdu_size);
	lockdep_assert_held(&wil->mutex);

	if (vring->va) {
		wil_err(wil, "Tx ring [%d] already allocated\n", id);
		rc = -EINVAL;
		goto out;
	}

	wil_tx_data_init(txdata);
	vring->size = size;
	rc = wil_vring_alloc(wil, vring);
	if (rc)
		goto out;

	wil->vring2cid_tid[id][0] = cid;
	wil->vring2cid_tid[id][1] = tid;

	cmd.vring_cfg.tx_sw_ring.ring_mem_base = cpu_to_le64(vring->pa);

	if (!wil->privacy)
		txdata->dot1x_open = true;
	rc = wmi_call(wil, WMI_VRING_CFG_CMDID, &cmd, sizeof(cmd),
		      WMI_VRING_CFG_DONE_EVENTID, &reply, sizeof(reply), 100);
	if (rc)
		goto out_free;

	if (reply.cmd.status != WMI_FW_STATUS_SUCCESS) {
		wil_err(wil, "Tx config failed, status 0x%02x\n",
			reply.cmd.status);
		rc = -EINVAL;
		goto out_free;
	}

	spin_lock_bh(&txdata->lock);
	vring->hwtail = le32_to_cpu(reply.cmd.tx_vring_tail_ptr);
	txdata->enabled = 1;
	spin_unlock_bh(&txdata->lock);

	if (txdata->dot1x_open && (agg_wsize >= 0))
		wil_addba_tx_request(wil, id, agg_wsize);

	return 0;
 out_free:
	spin_lock_bh(&txdata->lock);
	txdata->dot1x_open = false;
	txdata->enabled = 0;
	spin_unlock_bh(&txdata->lock);
	wil_vring_free(wil, vring, 1);
	wil->vring2cid_tid[id][0] = WIL6210_MAX_CID;
	wil->vring2cid_tid[id][1] = 0;

 out:

	return rc;
}

int wil_vring_init_bcast(struct wil6210_priv *wil, int id, int size)
{
	int rc;
	struct wmi_bcast_vring_cfg_cmd cmd = {
		.action = cpu_to_le32(WMI_VRING_CMD_ADD),
		.vring_cfg = {
			.tx_sw_ring = {
				.max_mpdu_size =
					cpu_to_le16(wil_mtu2macbuf(mtu_max)),
				.ring_size = cpu_to_le16(size),
			},
			.ringid = id,
			.encap_trans_type = WMI_VRING_ENC_TYPE_802_3,
		},
	};
	struct {
		struct wmi_cmd_hdr wmi;
		struct wmi_vring_cfg_done_event cmd;
	} __packed reply;
	struct vring *vring = &wil->vring_tx[id];
	struct vring_tx_data *txdata = &wil->vring_tx_data[id];

	wil_dbg_misc(wil, "vring_init_bcast: max_mpdu_size %d\n",
		     cmd.vring_cfg.tx_sw_ring.max_mpdu_size);
	lockdep_assert_held(&wil->mutex);

	if (vring->va) {
		wil_err(wil, "Tx ring [%d] already allocated\n", id);
		rc = -EINVAL;
		goto out;
	}

	wil_tx_data_init(txdata);
	vring->size = size;
	rc = wil_vring_alloc(wil, vring);
	if (rc)
		goto out;

	wil->vring2cid_tid[id][0] = WIL6210_MAX_CID; /* CID */
	wil->vring2cid_tid[id][1] = 0; /* TID */

	cmd.vring_cfg.tx_sw_ring.ring_mem_base = cpu_to_le64(vring->pa);

	if (!wil->privacy)
		txdata->dot1x_open = true;
	rc = wmi_call(wil, WMI_BCAST_VRING_CFG_CMDID, &cmd, sizeof(cmd),
		      WMI_VRING_CFG_DONE_EVENTID, &reply, sizeof(reply), 100);
	if (rc)
		goto out_free;

	if (reply.cmd.status != WMI_FW_STATUS_SUCCESS) {
		wil_err(wil, "Tx config failed, status 0x%02x\n",
			reply.cmd.status);
		rc = -EINVAL;
		goto out_free;
	}

	spin_lock_bh(&txdata->lock);
	vring->hwtail = le32_to_cpu(reply.cmd.tx_vring_tail_ptr);
	txdata->enabled = 1;
	spin_unlock_bh(&txdata->lock);

	return 0;
 out_free:
	spin_lock_bh(&txdata->lock);
	txdata->enabled = 0;
	txdata->dot1x_open = false;
	spin_unlock_bh(&txdata->lock);
	wil_vring_free(wil, vring, 1);
 out:

	return rc;
}

void wil_vring_fini_tx(struct wil6210_priv *wil, int id)
{
	struct vring *vring = &wil->vring_tx[id];
	struct vring_tx_data *txdata = &wil->vring_tx_data[id];

	lockdep_assert_held(&wil->mutex);

	if (!vring->va)
		return;

	wil_dbg_misc(wil, "vring_fini_tx: id=%d\n", id);

	spin_lock_bh(&txdata->lock);
	txdata->dot1x_open = false;
	txdata->enabled = 0; /* no Tx can be in progress or start anew */
	spin_unlock_bh(&txdata->lock);
	/* napi_synchronize waits for completion of the current NAPI but will
	 * not prevent the next NAPI run.
	 * Add a memory barrier to guarantee that txdata->enabled is zeroed
	 * before napi_synchronize so that the next scheduled NAPI will not
	 * handle this vring
	 */
	wmb();
	/* make sure NAPI won't touch this vring */
	if (test_bit(wil_status_napi_en, wil->status))
		napi_synchronize(&wil->napi_tx);

	wil_vring_free(wil, vring, 1);
}

static struct vring *wil_find_tx_ucast(struct wil6210_priv *wil,
				       struct sk_buff *skb)
{
	int i;
	struct ethhdr *eth = (void *)skb->data;
	int cid = wil_find_cid(wil, eth->h_dest);

	if (cid < 0)
		return NULL;

	/* TODO: fix for multiple TID */
	for (i = 0; i < ARRAY_SIZE(wil->vring2cid_tid); i++) {
		if (!wil->vring_tx_data[i].dot1x_open &&
		    (skb->protocol != cpu_to_be16(ETH_P_PAE)))
			continue;
		if (wil->vring2cid_tid[i][0] == cid) {
			struct vring *v = &wil->vring_tx[i];
			struct vring_tx_data *txdata = &wil->vring_tx_data[i];

			wil_dbg_txrx(wil, "find_tx_ucast: (%pM) -> [%d]\n",
				     eth->h_dest, i);
			if (v->va && txdata->enabled) {
				return v;
			} else {
				wil_dbg_txrx(wil,
					     "find_tx_ucast: vring[%d] not valid\n",
					     i);
				return NULL;
			}
		}
	}

	return NULL;
}

static int wil_tx_vring(struct wil6210_priv *wil, struct vring *vring,
			struct sk_buff *skb);

static struct vring *wil_find_tx_vring_sta(struct wil6210_priv *wil,
					   struct sk_buff *skb)
{
	struct vring *v;
	int i;
	u8 cid;
	struct vring_tx_data *txdata;

	/* In the STA mode, it is expected to have only 1 VRING
	 * for the AP we connected to.
	 * find 1-st vring eligible for this skb and use it.
	 */
	for (i = 0; i < WIL6210_MAX_TX_RINGS; i++) {
		v = &wil->vring_tx[i];
		txdata = &wil->vring_tx_data[i];
		if (!v->va || !txdata->enabled)
			continue;

		cid = wil->vring2cid_tid[i][0];
		if (cid >= WIL6210_MAX_CID) /* skip BCAST */
			continue;

		if (!wil->vring_tx_data[i].dot1x_open &&
		    (skb->protocol != cpu_to_be16(ETH_P_PAE)))
			continue;

		wil_dbg_txrx(wil, "Tx -> ring %d\n", i);

		return v;
	}

	wil_dbg_txrx(wil, "Tx while no vrings active?\n");

	return NULL;
}

/* Use one of 2 strategies:
 *
 * 1. New (real broadcast):
 *    use dedicated broadcast vring
 * 2. Old (pseudo-DMS):
 *    Find 1-st vring and return it;
 *    duplicate skb and send it to other active vrings;
 *    in all cases override dest address to unicast peer's address
 * Use old strategy when new is not supported yet:
 *  - for PBSS
 */
static struct vring *wil_find_tx_bcast_1(struct wil6210_priv *wil,
					 struct sk_buff *skb)
{
	struct vring *v;
	struct vring_tx_data *txdata;
	int i = wil->bcast_vring;

	if (i < 0)
		return NULL;
	v = &wil->vring_tx[i];
	txdata = &wil->vring_tx_data[i];
	if (!v->va || !txdata->enabled)
		return NULL;
	if (!wil->vring_tx_data[i].dot1x_open &&
	    (skb->protocol != cpu_to_be16(ETH_P_PAE)))
		return NULL;

	return v;
}

static void wil_set_da_for_vring(struct wil6210_priv *wil,
				 struct sk_buff *skb, int vring_index)
{
	struct ethhdr *eth = (void *)skb->data;
	int cid = wil->vring2cid_tid[vring_index][0];

	ether_addr_copy(eth->h_dest, wil->sta[cid].addr);
}

static struct vring *wil_find_tx_bcast_2(struct wil6210_priv *wil,
					 struct sk_buff *skb)
{
	struct vring *v, *v2;
	struct sk_buff *skb2;
	int i;
	u8 cid;
	struct ethhdr *eth = (void *)skb->data;
	char *src = eth->h_source;
	struct vring_tx_data *txdata;

	/* find 1-st vring eligible for data */
	for (i = 0; i < WIL6210_MAX_TX_RINGS; i++) {
		v = &wil->vring_tx[i];
		txdata = &wil->vring_tx_data[i];
		if (!v->va || !txdata->enabled)
			continue;

		cid = wil->vring2cid_tid[i][0];
		if (cid >= WIL6210_MAX_CID) /* skip BCAST */
			continue;
		if (!wil->vring_tx_data[i].dot1x_open &&
		    (skb->protocol != cpu_to_be16(ETH_P_PAE)))
			continue;

		/* don't Tx back to source when re-routing Rx->Tx at the AP */
		if (0 == memcmp(wil->sta[cid].addr, src, ETH_ALEN))
			continue;

		goto found;
	}

	wil_dbg_txrx(wil, "Tx while no vrings active?\n");

	return NULL;

found:
	wil_dbg_txrx(wil, "BCAST -> ring %d\n", i);
	wil_set_da_for_vring(wil, skb, i);

	/* find other active vrings and duplicate skb for each */
	for (i++; i < WIL6210_MAX_TX_RINGS; i++) {
		v2 = &wil->vring_tx[i];
		if (!v2->va)
			continue;
		cid = wil->vring2cid_tid[i][0];
		if (cid >= WIL6210_MAX_CID) /* skip BCAST */
			continue;
		if (!wil->vring_tx_data[i].dot1x_open &&
		    (skb->protocol != cpu_to_be16(ETH_P_PAE)))
			continue;

		if (0 == memcmp(wil->sta[cid].addr, src, ETH_ALEN))
			continue;

		skb2 = skb_copy(skb, GFP_ATOMIC);
		if (skb2) {
			wil_dbg_txrx(wil, "BCAST DUP -> ring %d\n", i);
			wil_set_da_for_vring(wil, skb2, i);
			wil_tx_vring(wil, v2, skb2);
		} else {
			wil_err(wil, "skb_copy failed\n");
		}
	}

	return v;
}

static int wil_tx_desc_map(struct vring_tx_desc *d, dma_addr_t pa, u32 len,
			   int vring_index)
{
	wil_desc_addr_set(&d->dma.addr, pa);
	d->dma.ip_length = 0;
	/* 0..6: mac_length; 7:ip_version 0-IP6 1-IP4*/
	d->dma.b11 = 0/*14 | BIT(7)*/;
	d->dma.error = 0;
	d->dma.status = 0; /* BIT(0) should be 0 for HW_OWNED */
	d->dma.length = cpu_to_le16((u16)len);
	d->dma.d0 = (vring_index << DMA_CFG_DESC_TX_0_QID_POS);
	d->mac.d[0] = 0;
	d->mac.d[1] = 0;
	d->mac.d[2] = 0;
	d->mac.ucode_cmd = 0;
	/* translation type:  0 - bypass; 1 - 802.3; 2 - native wifi */
	d->mac.d[2] = BIT(MAC_CFG_DESC_TX_2_SNAP_HDR_INSERTION_EN_POS) |
		      (1 << MAC_CFG_DESC_TX_2_L2_TRANSLATION_TYPE_POS);

	return 0;
}

static inline
void wil_tx_desc_set_nr_frags(struct vring_tx_desc *d, int nr_frags)
{
	d->mac.d[2] |= (nr_frags << MAC_CFG_DESC_TX_2_NUM_OF_DESCRIPTORS_POS);
}

/**
 * Sets the descriptor @d up for csum and/or TSO offloading. The corresponding
 * @skb is used
 *
 2.O{di22{d6_to_cpu(d->dma.length);

	switch (ctx->mapped_as) {
	case;
		if (!v}KRG!v}KRG!v}KRG!v}KRG!v}KRG!v}KRG!v}KRG!v}KRG!v}KRGOArl, "skb_copy failed\n")c_x_open YBn
	d->dma.ip_length = 0;
	/* 0..6: mac_length; 7:ip */
	d->dma.length = cpu_to_le16((u16)len);
	d->dmast_vring_cf = cpu_to_le16rd{dee Tx vring %d [%d] 0x%p:%pad 0x%p\n",
			     vring_index{dvring->size, vring->va,
			     &vring->pa, vring->ctx);
	{dinline int wil_vring_avail_low(struct vring *vring)
{
	ret - native2sg_alloc(wil, vril, vril, vril, vril, vril, vril, vril, vril, vriOn2{dP_KERNEL);
	if (!vring->va) {
		kfree(vring->ctx);
		vringS);

	ret2_tx_data[i].dot1x_ope{dT).ip l, vYKRG vYKRG vYKRG vYKRG vYKRG vYKRG vYKRG vYKRG vYKRGOpita_comp_to;

	eG;

	eG;

	eG;

	eG;

	eG;

	eG;

	eG;

	eG;

	eGOed
	 * For Tx and Rx, ownership bit is at the same locatioNE_EVENTID, &repUze);r_frags << MAC}KRGgld(vring_avail_loout;
	 MAYKRGgent = IE}HsG;

	eG;

um and/or TSO offloading. The corresponding
 * @skg is used
 *
 2.O{di22{d6_to_cpu(d->dma.length);

	switch (ctx->mapped_as) {
	case;
		if (!v}KRG!v}KRed{dlen);

	rtap_vendor->rtap.rthdr.it_version = PKTHDR_RADIOTSfAX_C 7:ip */
	d->d	d->dma.ip_length = 0;
	/* 0..6: mac_length; 7:ip truct }KRGt }KRGt }KRGt }KRGt }KRGt }KRGt }KRGt }KRGt }KRGOunlock(&txdata->lock);
	}

	return true;
}

/* wil_val_in_l_dbgl(wivring->pa, vri,K}KRG,K}KRG,K}KRG,K}KRG,K}KRG,K}KRG,K}KRG,K}KRG,K}KRGO}KRGt }KRGt }KR._PAE)))
			conr:Ghcorresponding
 * @f (reply.cm{datile struct vring_tx_desc *_d =
					&vring->va[vring->swSFor Tx and RxRG,KETH_P_true;
}

/* wil_val_irinMENTl_vriDETAILSf = cpu_to_vring_tx_de }KRGt }KRGt }KRGt }KRGt }K0(&txdawil_err(w->swSFo>lock);
	}
pu(d->dma.le */
	d->d	d->dmring->E_L4I))va[/UDtail(v),
		struct vring_tx_dsc *_d =
					&vring->va[vUDtawil_err(w->swSFo>lng->Cen);
	memvring2c,K}KRG,struct vring_tx_dsc *_d =
					&vring->PSEUDest)ADER_CALCw->swSFo>l
	eG;

	eG;

	eG;

	eG;

	eG;

	eG;

	eG;

	 and Rx, ownership bit is at the same locatioNE_EVENTID, &repUze);r_frags << MAC}KRdP_KEsNE_EVENTID, &HsG;

	strva[ and/ova[rsing. Tva[r6MAC}KNot   if d== u32,f (mcaun0] = West ardP_KEsNE_EVENTID, & SUMED" = (u8 *In all */
	uct sk_bufft vrious{d6_to_cpu(d->dma.length);

	s.all d
s->rx_"if unrolng-g"me loptct ze any
;

ill(lO{d< MAC}/2_NUM_OF_DESCRIPTORS_POS)>dma.length);witch (ctx->mapped_as) {
	case,
			struct sk_buff *skb)

statiENTID, &e cfg8021(vring->va) {
!		wil_err(wPA naALif (!wil->v0>lock);
	}
= 0;
	/* 0..6: mac_length; 7:ip truct }K	u16 dmale2] = 0;
	d->ml_ctx *ctxcode_cmd = 0;
	/* IP)ndev	wil_rx_bufiRG,K}\n");= 0;
	d->mgs <Gt }KRGt }KRGtsc *_d =
					&vringRGt }KRGOunlock(&txdata-ags <g_tx_desc *_dcode_cmd = 0;
	/* IPV6)ndev	wil_rx_bufiRv6G,K}\n");=  [%ds & RX_l];
			if (!ctx) {
(&txdata->lock);
}

i16 dmale0;
	d->ml_ctx *ctxIPPROTO>va[:s <Gt }KRG
	return true;
}

/* wil_val_in_l_dbgl(wivrinng->pa, vri,K}KRG,K}KRG,K}KRG,K}KRG,K}KRG<Gt }KRG
	retx andKRG,K}naplen) }KRGt }KRGt }KR._PAE)))
			conr:Ghcor <g_tx_desc *_dIPPROTO>UDt:inng->pa, vri,K}KRG,KUDta,K}KRG,K}KRG,K}KRG<Gt }KRG
	retx an&wil->sta[cid]udp,K} }KRGt }KRGt }KR._PAE)))
			conr:Ghcor <g_tx_desf (!ctx) {
(&txdata->lock);
}

ik);
	}
pu(d->dma.le */
	d-nabled
 */
_naplen) ring->E_L4I))va[/UDtail(v),
		struct vring_tx_dsc *_d =
					&vring->va[vUDtawil_err(w->swSFo>lng->Cen);
	memvring2c,K}KRG,struct vring_tx_dsc *_d =
					&vring->PSEUDest)ADER_CALCw->swSFo>ln2{dP_KERNEL);
	if (!vring->{di22{d6_to_ccfg_dchosentch (ctx->mapped_as) {a.ip_length g_tx_dsc *_d =
					&vring->/
	wEOPsg_alloc(il, vrsc *_d =
					&vring->/
	wMARK_WBsg_alloc(il, vrsc *_d =
					&vring->/
	went;
	swSFo>l
	e	if (!vring->{di22{d6_tg->cORS_POS)cfg_d
	swdev, sz, (void *)vring->va, vri{a.ip_length g_tx_dd6_to Rxl_errlng
_d =  ue;
}

/* wil_val_irinMENTl_vriDETAILSf = TX_2_NUM_OF_DESC__et_nr_frags(

	switch (cings active?\n");

	return NULL;
}

/* Use  (rtap_include_phy_info) {
		rtap_lenskb_rxdesc(skb);
	*d = *_d;
	palng->;

	rtid = wil_rxdescO_HEid(d);
	int ,strudev, sz, (void *)vring->va, vring, vrxrx(wi,rind	d-g, vg *vring,
		*_r TSOng, vrxrx(wipalng->;

	rng *vriEiddow = wil_rxdesc_len &ring, 1);
		}
	}
	dmPOS);em, d	d-g, v);em, r TSOng, v);em,Use  (rtator: 3d	d-g, v);em, *d	d-g, vr: 3d	d-g, v);em,Use  (rtatr TSOng, vrxr&r TSOng, v);empalng->;

	rng*vriEiddow = wil_rxdes' "GRO_HEL_len &ring,);
	
			wd	d-
		,atr TSOn1 don't care
stati= wis_e samfailed\ntot	}

umb			 * e sam= wil_rxdesc_len7:ip gng, v)cnt i++) {
		umb			 * = wil_rxdesc
	eG;_tid[i]mss }K	uum a 0;
	  ;
			}
			e++;
		k7:ipe sa, avu1 = cs + count avu1 ng_fdata->lockot1x_ope{dT.le */
sh(strun");=  _ope{dTlockot1P_MCg, v)CID %d TI=1x_ope{dT.+oed
	i[i]mss.le */
sh(strun");= g Rxopen ng->;aya.led];
	cw/oze);r_fra_len7:ipf,ULL !=,K}napatic i>dmrinb_any(ctx->skb);
			vring->swtail = wil_v->vring_tx_data[i];
		if (!v->va || !txdata->enabledsrc, ETH_ALEactior (i = 	e++;
		kl6210_rx(i, d);
eless  */
pe{d(i,*pe{don't careemcmp(pm_(!v->vamssreemcmp>dmmssreemcmpd	d-
ompensing->_to stssert_heltati= wito Rxl_er _dd6_to Rxl_errr TSOhelthdr.it_versheltati}KRG,K}KRGOheltati */
	d->d	d->dmrintatig Rxl_erreemcmp( completion o   int vring_index)
{r_frags(

	s
	if (unli*vrivTYPE_POS);

e to act "vring_fading. The clity (mtu_max para&&
		    (skb->p) {
(&txdata->lock);lng->Aatilll(lO{dge 4K>rx_3-4>;aya.les("Rx as,
	ed->dmape{dmd[iats->rx_acaull>;aya.leID %d 's how P_MCg, v)CID %d TIhas beetats->et stack.
.ive dex)
Rx mightoto stmxrx(1_RAriv = wil_rxdes,
/* statisisNE_EVe packeset_hasest .buffers in u_max paravu1 =< P_MCg, v)CID %d T)t suitable, need to look at data
	 */
	i * @flock_bh[%2d]caull.KNo spacent = if pe{dmd[is(cid >= WIL62ading. The , P_MCg, v)CID %d T); {
(&txdata- __aligned(2g->HK}KRG,L->dma.lelength; 7:ip t.+oIPgth; 7:ip t.+o}KRG,K}KRG,K}Kfers,K}nap;
	/* 0..6:			 srx_la */
	d-nabled
 */
_naplen) 			 sdKRG,K}naplen) );lng Rxl_er _d */
sh(strun");= g Rxl_er & (SKB_GSO>va[V6 in KB_GSO>va[V4rs to6 dmaleg Rxl_erl_ctx *ctx KB_GSO>va[V4:inng->}KRGrsinskb =[id]E_EVIPgd->dma.epUzI[rsail(v),
		_STATU
il, vas CID %d TIby
			/* * For Tx doc
tx_data_iRG,K}\n");= totdot1x_o0;ta_iRG,K}\n");= et_has_o0;ta_i}KRGt }ssert_hel<g_tx_desc *_dSKB_GSO>va[V6:inng->}KRGr6inskb =[id]E_EV;aya.ledndor, 0, rtapRv6G,K}\n");= ;aya.ledot1x_o0;ta_i}KRGt }ssZE(wil-><g_tx_desf (!ctx) {
g-> mac_lvringva[rs(1_Rva[r6atilesey_id	struct sk_buft = IE}}
}

/*In alling_fillegkest = len);
		ng =_droit vraneoushdr.i	  skb->data, here looking for 1(vring->va) {
!		wil_err(wPA naALif (!wil->va->lock);lng->dKRa,K}KRG,K}KRG,K[cid]kb 	d-nabla,K}KRG,K}KRG,K[_idfixbuft = a {
			svoid w'v = wil_rxdesng-->lo]E_E Wescenil, buffers}KRG,K}KRGO}= dKRG,K}naplen) );	 */
	d->d	d->dm.le */
	d-nabled
 */
_naplen) ri
	_d	d-g, vr: 3 = wil_fi>mactxp_header rthdr;
		/* fields should be in,K}napate;
}TO>in rthdr.it_present */
		/* flags */
		u8 flags;
		/* chtart anew */
	s* @fSkb ;
	  
			 fl);
		s_frags <	.max_mpdexiata->lock);
	txdPOS);
}
d	d-g, vgor cs,K}napatading. The cli	d6_to_cpu(d->dma.length);

	swd	d-g, vgoINSERd6_to Rxl_errh 2.Oit_version = ng,
		RADIOTSfAX_C  */
	d->d	d->dmali	d6_to_ccfg_dchosed	d-g, vruct wil6210_rx = w flaed_as _dd6_t flaed_as		/* fi;rs,K}n1 don'&wil6210_rx = >lock wis_e sal, v-ic i>dm.le */
f (snaplen) ng-,K}nap->pn, IEEf.leic i>dm.? -1int r f=<  _ope{dTl f0;
	 only 1 Vic i>dm(1 << M>dm.leic i>dmrin  (skb->protocol != cp* @fpaticsed yetic i|
		 *%u(cid >= WIL621_to_ndevc *d, int nrpe{don'& */
sh(strun");= pe{dT[f &wil->dm.lepe{da->dotrin  (skb->protocol != cp* @fpe{d))
	:
		 *%u(cidpf,ULL ags <<  cht\n", m>dm(1 << M&&
		    (skb->protoco */
	i * @f		 *%i|
(pm_(!v->%i|
= wis_e sam);
	struct ethhULL !=(pm_(!v-|
= wis_e sawil, "Rx co= wis_e samf= avu1 
	 onlywil6210_priv *wil, struct sk * @fTYPE_2(stma.w_frags <for compl __align	 <	.max_tx[;
		s;%d\n",wil->dmmss.leP_MCS));
	i(pm_(!v-|
LL ags <ats->r 0;
	  +
= wis_e saw %= {
			.tx_srin  (skb->protocol != cp* @f>dmmss.%i|
i_POS);

>dmmss_PAE)))
	);
	d-ic i>dm(1 << Meader  */
pe{d(		/* fl8 flagpe{dion = P ng,
		pe{da->dotpondL !=>dmmss_on = P ng,
		e;
}TO>in rthdr.iiiiwil6210_rx = w flaed_as _dd6_t flaed_as	{dge;%d\n".dot1x_open ader rthdr;
		/* fields on = P ng,
m to HW
	 *on = P ng,
m t
f (snaplen) ng-, (snap on = P ng,
>dmmss_on = P ng,
e;
}TO>in rthdr.iiiiwil6210_rx = w flaed_as _dd6_t flaed_as		/* fi;rssssic i>dm.-=p>dmmssree\n",wil-t_present */
		/* flags */
		u8 flags;
		/* chchtart anew */
	s* @f
			 fl){dge ;
		s_frags < <	.max_tx[;
		s;%d\n",wil--g, vr: 3 = wil_fi>mactxp_he	);
	d-_r TSOng, v	/* chch_r TSOng, vrxrng, vgs < <r TSOn1 don'&wil6210_rx = >l < <stssr TSOng, v;%d\n".dot1x_open or: 3g, v);empad\n",wil-k);
	txdPOS);
}
dgor cs>dmmss_Pading. The cli			d6_to_cpu(d->dma.length);

	swdgoINSER= wito Rxl_er_on = P ng,
		it_versi	RADIOTSfAX_Con = P ng,
		 */
	d->d	d->dmalion =xrx(wilo Rxl_errr TSOWest aescenDESC_T= wito Rxl_er _dd6_to Rxl_errm 1);
		ck wis_e sal,  
	stru vre samg_ffarnDESC_T gng, v)cntl,  	stru vre samsupported edmd[inDESC_T>dm.-=p>dmmssree\n(pm_(!v->-=p>dmmssre<< M&&
		    (skb->protoco */
	i * @f		 *%i|
(pm_(!v->%i|
= wis_e sam);,p gng, v)cnt );,
	struct ethhULL !=(pm_(!v-|
= wis_e sa,p gng, v)cntalion =xrxClose any
 edmd[inp to _PREFImss.];
	cornding
pe{dDESC_Tdata->m_(!v->v_o0openEf.l=1x_ope{dT.- 1		co>dm.ldev;
		xmily 1 Vi	d-
ompensing->_to s
		xmily	/* 0.TSOW edmd[inpl)
{
	-,K}tru vrl_dbg_	}

/*v->s *_bg_	}

/ats->rs,K}n1 d=  _ope{dTer  gng, v)cnt>l < <	d6_to_cpu(d-g->ctx);
		vrr TSOng, vCon = P "BCAgng, v)cnt +on = P "BC1ags < <	d	d-
ompensing->_to stssZE(wil-><\n".dot1x_open 	d6_to_cpu(d-g->ctx);
		vrr TSOng, vCon = P "BCAgng, v)cntags < <}s < <r TSOn1 d=  _ope{dTer  gng, v)cntratel

en 	d6_to_ccfg_dchoseawil, "R	/* 0.TSOW	eG;

	eG;

			ing_fbe any
cfg_
_	}

/*supportedmss.-) {
				ret	stril)
opy
_	}

/*r);
wicridging /xmily 1 Vr TSOng, vr!		>proto		*_r TSOng, vrxr*r TSOng, v;%, "R	/*ding
 * @skg isend to wirx[i].s_vring(e03x\	}

/*ring2tedTSfpaticsea && ging /xmily 1 Vr=<  _ope{dT.- 1	pen as is0proto		*_g, vrxr*1);
		c	(pm_(!v->vamssreehch_r TSOng, vrxrring %d\n	 gng, v)cnt i++)%d\n".dot1x 1 Vr TSOng, vr!		>p 	stu]	= "G
			 * @skg ise/ats->rs*_g, vrxr*1);t cid = wil_0.TSOW	eG;

	eG;

			ing_fbe any
cfg_.x\n",
		g2tedc *_dd>;

	rng*tisintinuebuffers in _r TSOng, vrxxrng, vprotstssr TSOng, v;%ing->paSO offl	 * @skg ise/atsd6_tg->cORS_POS)cfg_d
	swT); {*_g, vrxr*1);
	g->Fd toany
tot	}

umb			 * = wil_rxdescO_H0.TSOW	eG; Vi	d)/atsd6_to_cpu(d-g->ctx);
		vrd	d-g, vgo= wis_e sawil	*_d	d-g, vr: *d	d-g, v);
	g->hoC[7:0fl, scenkb;
	v
l, vrint vring  yetv->s *_bg_txrx(mediuculatx\n",
		TH_ALEN)ims.rx_ "G"t, 16, "bufferswil6210_rx = wl = eth_tygetlen) ri
	g->;stmormts-> monixdeil_dbg_te samfas + count e sang_fdata->lock/* L4 Itinng_trange= 0; i < WIdone__trsh,Use  (rtae sa,pe sam+
= wis_e saw
		xmidr wmi;
		str+= get_cycles( ng-i_vring_cfg_done_;
  (skb->protocol !=  "R_bh[%2d]c	str		str%dION_);
	structing_fading. The ,ae sa,pe sam+
= wis_e sawigned(2g->M{
				ret
			tats->oany
;
	  est aafrng* * @skg iseu]	= "Gtis16, .x\n",ll dend tot vring a r mo "GRdi] = Wwil, sany
;if (wil->vth->lo
rotoco tose sany
DUf (re=_dril->rt vrious{runK[cidw
/**
 * allany
rotol = g_txrx( locaswil_rx_buf.& txdata->enab(2g->	tats->o 0;
	  /atsd6_tcount atats->last_mcs_rx <= wis_e sawign(skb->protocol != cp* @fTxo 0;
	  %dION_);
	sto 0;
	 ,
			}
			e++;
nab(2g->rxdesc_cid(d);
	int tid = wil_rxdesc_tid(d);
	int key_id = wil_rxdesc_key_id(d);
	int mc = wil_rxdesc_mcast(d);
	struct wNAPI run.
	 ,
			}
			e++;
nab(!wil->v0>lo_tx[;
		s:%d)\n", m= wis_e sam>ytes =>  &ring,);
	
			wctxp_he	ts->r 0;
	  +
= wis_e sa.- 1w %= {
			.tx_srin ds->r void *)vring->va, vri)3 = wil_fi>mactxp_l--g, vr: 3 = wil_fi>mactxp_	ly if req, v;%d\eq, vl, "skb_copy faT	prefetch(skb->;%d\1 don'&wil6210_rx = >l <d6_to_pu(d-1_RAD8 flagt wit cli		_txkb i
		,a0tes TCP chit ccli		= wis_e sa0_prned_mpdexia:nchronize(&wil->NUM_OF_DESC__et_nr_frags(witch (cings active?\n");

	return NULL;
}

/* Use  (_include_phy_info) {
		rtap_lenskb_rxdesc(skb);
	*d = *_d;
	pan &ring, 1);
		}
	}
	dmt wtor: 3d1);tdev, sz, (void *)vring->va, vring;	uum a 0;
	  ;
			}
			e++;
		k7:ipavu1 = cs + count avu1 ng_fdata->lockot1x_ope{dT.le */
sh(strun");=  _ope{dTlocior (f (const voiy(ctx->skb);
			vring->swtail = wil_v->vring_tx_data[i];
		if (!v->va || !txdata->enabledsrc, ETH_ALEactior (i = 	e++;
		kl6210_rx(i, d);
7:ipe sahelthdr. == NL80_to_le16((u16== 0; i < WIL6210_M)actior (>dm.le */
f (snaplen)  o   int vring_index)
{r_frags(
	if (unli*vrivTYPE_POS);

e to act "vring_fading. The clity (mtu_max para&&
		    (skb->p) {
(&txdata->lock);ln in u_max paravu1 =< 1 +
x_ope{dT)t suitable, need to look at data
	 */
	i ock_bh[%2d]caull.KNo spacent = if pe{dmd[is(cid >= WIL62ading. The , 1 +
x_ope{dT); {
(&txdata- __aligned\eqr: 3 = wil_fi>mactxp_header rthdr;
		/* fields should be in */
f (snaplen) ate;
}TO>in rthdr.ic int wil_tx_desc_map([%2d]cl = if (unli*0x%pION_)paOS);

ading. The ,"vring_f */
f (snaplen) athould be in&{di22{ intld be 0x0,
		 *Tit writes 0xffff in violation of RF1624
		 */
	}

	if (snaplen) {
		/* Pac.it_present */
		/* flags */
		u8 flags;
		skb->data, here lookwil6210_rx = w flaed_as _dd6_t flaed_as		/* fi;rsg->v2->v edmd[inDESCk);
	txdPOS);
}
dgor cs>dm_Pading. The cli	t_present */
 skb);t suitto_le16rd{de|ail_low(struct vring *0_MCSw->swSFo>mac_lCSo0o, v2, skbesent */
 as isringvring-lCS0_LIMIT)ndex <_drlCSo1nDESC_T=o_le16rd{de|aiil, vril, vril, vril, 0_MCSwINDEXswSFo>ln}rsg->Paticsedva[/UDtail(v),
		* * For Tx fers in u_max parRIPTORS_POS)>dma.length);wdgoINS
		/* chtart anew */
	s([%2d]cF= (nrnkb;
_dr headroSSARYvoid w(cid >= ading. The cli			.max		/*;
		s;%d}ct wil6210_rx = wx_ope{dT.le _ope{dTlocd6_to_cpu(d-g->ctx);
		vrdgox_ope{dT.+oenab(2g->re corv edmd[isa.b11 = 0/r f=<  _ope{dTl f0;
	* cheless  include_/
pe{d( includ*pe{don >= W& */
sh(strun");= pe{dT[f &wilor (>dm.le */
pe{d( 		  pe{dwil, "*eqr: *1);t  int wil_tx_desc_map([%2d]cpu(d[%4f (cid2ading. The , _desc_ intld be 0x0,
		 *Ti understand Microsoft IP stack tha
		 * mis-calculates TCP checksum - if itts->r 0;
	  +
f.+oen %= {
			.tx_srin eqr: 3 = wil_fi>mactxp_Meader  */
pe{d(		/* fl8 flagpe{dia0tes*/
pe{d( 		  pe{dwion = ng,
		e;
}TO>in rthdr.iit_present */
		/* flags */
		u8 flags;
		/* chctart anew */
	s([%2d]cf= (nrnkb; fl)pe{dmd[i(cid >= Wading. The cli				.max		/*;
		s;%d<}s <wil6210_rx = w flaed_as _dd6_t flaed_as	{dge;%d\k);
	txdPOS);
}
dgor cs>dm_Pading. The cli		{
		if
		skb_set_has->data,g_in -
}

/*rext_taucic vsamsuppv2->v= wil_rxdeead) && (coo tosucic vwil, sto+;
		n/
		d6_to_cpu(d->dma.length);wdgoINS
>ln}rsg->icast peding
 ed est astruct vring_tx_dsc *_d =
					&vring->/
	wEOPsg_al;ruct vring_tx_dsc *_d =
					&vring->/
	wMARK_WBsg_al;ruct vring_tx_dsc *_d =
					&vring->/
	went;
	swSFo>l"*eqr: *1);t int wil_tx_desc_map([%2d]cpu(d[%4f (cid2ading. The , _desc intld be 0x0,
		 *Ti understand Microsoft IP stack th
		 * mis-calculates TCP checksum - if 
	g->hoC[7:0fl, scenkb;
	v
l, vrint vring  yetv->s *_bg_txrx(mediuculatx\n",
		TH_ALEN)ims.rx_ "G"t, 16, "bufferswil6210_rx = wl = eth_tygetlen) ri
	g->;stmormts-> monixdeil_dbg_te samfas + count e sang_fdata->lock/* L4 Itinng_trange= 0; i < WIdone__trsh,Use  (rtae sa,pe sam+
x_ope{dT.+oen
		xmidr wmi;
		str+= get_cycles( ng-i_vring_cfg_done_;
  (skb->protocol !=  "R_bh[%2d]c	str		str%dION_);
	structing_fading. The ,ae sa,pe sam+
x_ope{dT.+oenabned(2g->M{
				ret
			tats->oany
;
	  est aafrng* * @skg iseu]	= "Gtis16, .x\n",ll dend tot vring a r mo "GRdi] = Wwil, sany
;if (wil->vth->lo
rotoco tose sany
DUf (re=_dril->rt vrious{runK[cidw
/**
 * allany
rotol = g_txrx( locaswil_rx_buf.& txdata->enab(2g->	tats->o 0;
	  /atsd6_tcount atats->last_mcs_rx <x_ope{dT.+oenabn int wil_tx_desc_map([%2d]cl0;
	  %dION_);
	stoading. The ,a 0;
	 ,"vring_fading			e++;
nab(tr mo_ings actg_fdata-. The ,a 0;
	 ,
e to act 
x_ope{dT); (2g->rxdesc_cid(d);
	int tid = wil_rxdesc_tid(d);
	int key_id = wil_rxdesc_key_id(d);
	int mc = wil_rxdesc_mcast(d);
	struct wNAPI run.
	 ,
			}
			e++;
nabhat the next 		/*;
		swil, v1_RADWwiat
Rx ide d flaedrxdesx_ope{dT.lef.+oe>mac_pe{dT. flaedr+		 sticas yetic ia.b11 = 0/f (con f=<  _ope{dTl f0;
	* ch &ring,);
	
			wctxp_he	ts->r 0;
	  +
fn %= {
			.tx_srin 1 don'&wil6210_rx = >l <eqr: 3 = wil_fi>mactxp_Mey if req>l <eql, "skb_copy faT	prefetch(skb->;%d\d6_to_pu(d-1_RAD8 flagt wit clii		_txkb i
		,a0tes TCP chit ccli	AC_CFG_DESC here loo	wil_dbg_txrx(wil, "Tx while no vrings active?\n");

	return NULL;
}

/* Use one of 2 strategies:;

statiy(ctx->skb);
			vring->swtail = wil_v->vring_tx_data[i];
		if (!v->va || !txdata->enabledsrc, ETH_ALEactize = 0;
	: fix for;
	} __packed reply;
	stil,
					     "findsuspeship st: vring[%d]  |oc(il, til,
					     "findsuspeshsa,p: vring[%d]  |oc(il, til,
					     "findSUMEmip st: vring[%d] 	/* chtart	    (skb->protoc */
	isuspesh/SUMEmvring_tx_data.oSSARYvoid w(cicli			    (skb->tocol != cpu_to_beb->data, here looking r comlen)_is_gsrun"); ?C__et_nr_frags(

	s :C__et_nr_frags()c(il, v
	}

	return en) ri
		    (skb->tocol != cpu_to_bnchronize(&wil->G;

	eGCt_hasb_copy ringx 7:ip_versiostop/wxdesx_dr);
	ss*rexnc vsa= (u8 *ll deaun0] = Wdost_2 structw_set_haskb, ive c *_dct_ha_stopsisNErue,dw
/**et_hasrexncdr);
	ss*
		skb_sng =t	skb-. If
	if (mc"GRdi] = sc
	eG=t	skvriny_idmet,RGED_Fnr_f=t	snlocl);
	ss( ntedc 		[G.b, ive c *_dct_ha_stopsisNum - ,dw
/**et_hasrexncdr);
	ss*
		skb_sng wxde-. If
	if (mc"GRdi] = sc
	eGwxdvriny_idmet,RGED_Fnr_fwxdenlocl);
	ss( ntedc 		[G.b, i		vrinisNE_EV		vrinwhichntedc_tid[it abevrinmodifi TIby
eimac_ledhip bit = wil_rxdesc_t ctxrxikely	eGr	intic i= wil_rxdesc_t wil_rx_bu)ril->rit. Can
_dbg_ null>il, sirv->satst (e.g.address
/disddress
 vrings).= (u8 *llvri_rx_md[i_res_str[3);
	opsx_dr);
	ss*rexmodifi TI		vrinhas lowbit = wil_rxdepavu1 an");

. Wxdesrex(d);7:ip_verret	stringlow = wil_rxdebit avu1 an");

ersiomodifi TI		vrinhas dev_ = wil_rxdepavu1 an");

.8 cid;
	struring->{di22{__et_nu]	= "
	d->);
	ss(cated broadcast vring
 * 2. Old (pseudo-D NULL;
}

/* Use ld (pthdr.ct_ha_stop);

static s
r);
	txdat)
  (skb->protocol != "vTYPE_PO,.ct_ha_stop=);,p t	skb-=);structing_frx_la(		vring->swtail = wil),.ct_ha_stoptructing_f>swta	d->);
	s_ t	skb-o_bedot1
  (skb->protocol != "ct_ha_stop=);,p t	skb-=);structing_fct_ha_stoptf>swta	d->);
	s_ t	skb-o_b
r);
	ct_ha_stops== 0; i 	d->);
	s_ t	skb-oi		{
		_dr);
	ss*ta->locking= w%d TI;
	se	  skb->data_b
r);
	ct_ha_stope only 1 VR		vrin|| u_max parRIPTcount avu1 nlow	txdat)		/* chc/	strucenouv_ ro->rinNE_EV		vrinDESC_TGED_Fnr_f=t	snlocl);
	ss();
	*d RX buffercli			d6_i 	d->);
	s_ t	skb-}ssert_hel< (skb->protocol != "GED_Fnr_f=t	sdc 		[G_frags << M	wil_info(wiil, vD*wil)
wxdesE_EV);
	ss*rn suspesh ma.wdbg_txrx(wil,
					     "findsuspeship st: vring[%d]  |oc(il, til,
					     "findsuspeshsa,p: vring[%d] nd_cid(wil, et, vet_haswxdeswil->vring2cid_tid[i][0];
		if (cid >= WIL6210_MAX_seudo-D NULL;
c_tTcount>va || !txdata->e = >l <>vring_tx_data[i];
		if (!v->va || !txdata->enabled)
			co 1 VRc_tTcountt1x_open &&
		    (skb->openc_tTcount>v
			vrilation type:  0 - bypaRIPTcount avu1 nlow	c_tTcount)(1 << MAC_CFG_DESC_TX_2_L2vTYPE_POcaull,!wil')
wxde
	struct ethhUrx_la(c_tTcount>g->swtail = wil))ree\n(pl_info( cid = w 1 VR		vrin|| RIPTcount avu1 ndev_(count)(1 << , venouv_ ro->rinNE_EV		vrinDESC_(skb->protocol != "calng-gRGED_Fnr_fwxde_frags <GED_Fnr_fwxdenlocl);
	ss();
	*d RX buffercli		d6_i 	d->);
	s_ t	skb-}ssZE(wil->}il->napi_tx);u]	= "
	d->);
	ss(cated broadcast vring
 * 
	return NULL;
}

/* Use  (rthdr.ct_ha_stop);

s: fix for;
d6_i 	d->);
	s_pu_to_be__et_nu]	= "
	d->);
	ss(	}

	return ct_ha_stop);
		    (skb->tod6_i 	d->);
	s_pu_to_bl->napi_tx);u]	= "
	d->);
	ssprotitch (cings active?\n");

	return NULL;
}

/* Use  (rtapthdr.ct_ha_stop);

s: fix forprotod6_i 	d->);
	s_pu_to_be__et_nu]	= "
	d->);
	ss(	}

	return ct_ha_stop);
		    (skb->protod6_i 	d->);
	s_pu_to_bl->	d-(wilr_fx(wil,dot1xtruct tinue;

		/* don't Tx>rx_buf_len) {
		/* do {
		rtap_lenings active?\n"); c;

	w	*d ");(il_vrings and duplicate skb for each */
	for (i++thdr.b== NL80is_refill(wi_continue;
(truct sk_buff  vring *wil_find_tx_f  vr	struthdr.pr_esce_fwactize = 0;
	(skb->protocol != "dot1xtruct_frags  (mtu_max para&il,
					     "findfw->loc,p: vring[%d] ne only 1 VRpr_esce_fw	/* chctart anew */
	FWwil)
->loc_frags < pr_esce_fw}ssert_hel<}i			.max	SAR>ln}rs (mtu_max para&il,
					     "findfwddress
 *,p: vring[%d] ne only(skb->priv *wil, struct skFWwil)
ddress
 *,pvoid w 			skb-_frags <	.max	SAR>ln}rs (mtu_max parw u16 size)
{
	struct vring *vring = &wil->v	/* chtart anew */
	Xmitringmonixde
 *  d	struct sk_bu_frags <	.max	SAR>ln}rspr_esce_fw}ssZE(wil-
	/* 0..6:		vrinDESC*wil, u16 size)
{
	struct vring *vring =tch(ION		co&wil->vbss(1 << , vinggs;
 *   (ress,x(d);3);
amvrress x(wo AP)nDESC_count>vawcast):
 *    use dediw		return rc".dot1x 1 Vb== Ne only 1 Vwil->vbss( chc/	sing_bss_Pno.b== NLress x-on 0-IP6 1-IP4*in
	}

/*(d);si_res_sLress s
ging /xmilcount>vawcast):
 *  urn NULLw		return rc	dot1x 1 V, u16 size)
{
	struct vring *vring =AP) chc/	sAPnhas a = (void *)s== NLress x /xmilcount>vawcast):
 *  urn NU1Lw		return rc	dot1 chc/	sun cases ovil_bin_res_ksum l_da_for_n 0-IP6 i && gi	if (mcIP4*in*(d);si_res_sLress s
ging /xmilcount>vawcast):
 *  urn NULLw		return rc".dot1x_ope/	sunll(wi, 0..6:specificLress xby
k_bu.210_priv DESC_count>vawcast):
 *  wil621w		return rc"rs (mtu_max paracount)(1 << (skb->protocol != "Nif (!ress xn,
		nt = ipM
	stotruct sk_buff  <	.max	SAR>ln}rsex <_drupva)
			cntt ,strufalse;
	sr_frags(struct eturn en) ri
		6 dmalercl_ctx *ctx0) {
g->sh(d);
e;
	opsx_dr);
	ss?nDESC_(skbu]	= "
	d->);
	ssprottruct eturn ert_cli		{
	si_resstrsend to wiu]	= "  esoany
tx_il_rx_buK}KRG<G	w	k
		}_en)_anylen) );	dr_t pa, ETDEVringRK;tx *ctxa- __al:;	dr_t pa, ETDEVringBUSY;esf (!ctx) {
g_tx_dmac_	.max	SAR>K}KRG}
x	SAR) {nsize)si_rso gu			skb-l, v-G	w	k
		}_en)_anylen) );
dr_t pa, ET_XMIT_DROP>l
	e	if (!vring->{thdr.(skb
		ssr_	if  tinue;

		/* don't  when re-routing Rx->Tx at the AP */
		if (0 
dr_t pa,is_unll(wi_continue;
(truct sk_buf		co*/
		sk WIL6210_ .d[2] 
sh(strun");=  gufl{dT.&dSKBingWIFIetch(sko>l
	e	if (!vring->{di22{d6_t * mum}_en) tinue;

		/* don't Txthdr.out_f);

stin u_max parRIPT
		ssr_	if  tNS
		
ESC_T_il_rx_bu_line_out_INSERout_f)_bedot1
  out_fr?nskbt * mum}_en)_anylen)  : G	w	k
		}_en)_anylen) );l->G;

	eGCleanrupvze, vd(d)eid]kb'sril->rany
 (!ress = (u8 *RdP_KE

umb			 * = wil_rxdescclearsa= (u8 *Safet
		calnril->rIRQ8 cidxrx(wil, "Til_rx_bufree(wil, vring, 1);
}

static eturstruct vring *_len) {
		/* do skb);
	*d n= *_d;
	pan &ring,skb_rxdesc(skb);
	*d = *_d;
	pan &ring, 1);
ind_tx_ucast(struct wil62eturst_priv *wil,
				       struct sk_buff *skb)
{
	int i;
	eturst_pritati=2 st(const void->dma.length = cpu_to_leeturst_((u16) &ring,);
	_lensi_rson'i_rsoxrring %ddev, sz, (void *)vring->va, vring;	u7:ipe sa_l_rxdeTil_rx_bu;	u7:ipe sa_newlity (mtu_max paracountt1x_		/* chtart anew */
	s( irq))
	:
d_tx_u	strinpackeizturn 0;eturstr);	dr_t pa,0;id = w 1 Vu_max para&&
		    (skb->p)/* chtart(struw */
	s( irq))
	:
d_tx_udisskb->rn 0;eturstr);	dr_t pa,0;id = w(skb->protocol != " "Til_rx_buid[id)rn 0;eturstr);_te sa_l_rxdeTil_rx_bumfas + count e sang_fdata->lo
skb)
{
	stra.d0 = (vring_inl <>v_rsoxr&: vring[ative >v_rs0;
	(\n", m!s + count is_empty(count)(1 << kot1xew_sn.
	 >l <>vring_);
	
			wctxon'&wil6210_rx 			}
			e.
	 ]li		{
*ad) &&Fcast pepe{dmd[ieid]kb, HWoco toset
DUf (reest aicast pad) &&ding
pe{dmd[i. lookaicasfor P	, ive Oed
t pep.TSOWDUfco topl)
{
	-,K}tru v;
		n/
		or (>fL80_to_le			e.
	  + 1 d=  _ope{dTn %= {
			.tx_srin ;
	u8 cidet_hasweerret	strping
ic ia.b1l <eqr: 3 = wil_fi>lfactxp_Me 1 Vu_max para(eql, "skb_copy &aT	prefetch(skb->nslatiog_tx_des <GEw_sn.
	 L80_lf.+oen %= {
			.tx_srin (\n", mto_le			e.
	  !=1xew_sn.
	 ++) {
		v = &wil->v		}
	}
	dmt wtor: 3d1);t		u16 		/>dmrin  tinue;

		/* don't );
		cctxon'&wil6210_rx 			}
			e.
	 ]li			l = et1 d= 't );	 <eqr: 3 = wil_fi>			}
			e.
	 ]ctxp_he	)y if req>lhe	)		/>dm =p>d16 memory ;
	/* 0..6: mags < tr mo_ings actg_ 100)(eturst,
			}
			e.
	 ,
		/>dm		wil_dbg_tx v}KRG!v}KRG!cli			d6_t	    (skb->protoco */
	i xC[%2d][%3d] 
	if (unlitxdata->lock);
 ;
	lock);
	/* napio */
	eturst,
			}
			e.
	 ,
		/>dm		wil_g_tx v}KRG!vdata->, v}KRG!v}KRG!cli			d6_tld be 0x0,
		 *TiC understand Microsoft IP stack thah
		 * mis-calculates TCP checksum - if 
	d\d6_to_pu(d-1_RAD8 flagt wit clii		 for 1(v
		xmily 1 Vent */
	}KRG!v}KRG!v}dev;
		xmily{nsize)si_rso guvoid wsl, v-ily{nsize)si_rso gu(unli*+=
e to act v-ily{for 1i_rs
		xmily{<>v_rs=  guvoid wsl, v-ily{<>v_rs=  gu(unli*+=
e to act v-ily{}s < <".dot1x_open 	nsize)si_rso gu}KRG!sl, v-ily{for 1i_rs
v-ily{<>v_rs=  gu}KRG!sl, v-ily}s < <d6_t * mum}_en) tNSER=}KRG!v}KRG!v}dev; v-il}s < _txkb i
		,a0tes TCP chit ccli		2g->M{
				ret
(mc"txoi];

			w g_txrx(u]	= ;
	int  .
	 & 