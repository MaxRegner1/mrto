/*
 * Copyright 2002-2005, Instant802 Networks, Inc.
 * Copyright 2005-2006, Devicescape Software, Inc.
 * Copyright 2006-2007	Jiri Benc <jbenc@suse.cz>
 * Copyright 2007	Johannes Berg <johannes@sipsolutions.net>
 * Copyright 2013-2014  Intel Mobile Communications GmbH
 * Copyright (C) 2015-2017	Intel Deutschland GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * utilities for mac80211
 */

#include <net/mac80211.h>
#include <linux/netdevice.h>
#include <linux/export.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/bitmap.h>
#include <linux/crc32.h>
#include <net/net_namespace.h>
#include <net/cfg80211.h>
#include <net/rtnetlink.h>

#include "ieee80211_i.h"
#include "driver-ops.h"
#include "rate.h"
#include "mesh.h"
#include "wme.h"
#include "led.h"
#include "wep.h"

/* privid for wiphys to determine whether they belong to us or not */
const void *const mac80211_wiphy_privid = &mac80211_wiphy_privid;

struct ieee80211_hw *wiphy_to_ieee80211_hw(struct wiphy *wiphy)
{
	struct ieee80211_local *local;
	BUG_ON(!wiphy);

	local = wiphy_priv(wiphy);
	return &local->hw;
}
EXPORT_SYMBOL(wiphy_to_ieee80211_hw);

void ieee80211_tx_set_protected(struct ieee80211_tx_data *tx)
{
	struct sk_buff *skb;
	struct ieee80211_hdr *hdr;

	skb_queue_walk(&tx->skbs, skb) {
		hdr = (struct ieee80211_hdr *) skb->data;
		hdr->frame_control |= cpu_to_le16(IEEE80211_FCTL_PROTECTED);
	}
}

int ieee80211_frame_duration(enum nl80211_band band, size_t len,
			     int rate, int erp, int short_preamble,
			     int shift)
{
	int dur;

	/* calculate duration (in microseconds, rounded up to next higher
	 * integer if it includes a fractional microsecond) to send frame of
	 * len bytes (does not include FCS) at the given rate. Duration will
	 * also include SIFS.
	 *
	 * rate is in 100 kbps, so divident is multiplied by 10 in the
	 * DIV_ROUND_UP() operations.
	 *
	 * shift may be 2 for 5 MHz channels or 1 for 10 MHz channels, and
	 * is assumed to be 0 otherwise.
	 */

	if (band == NL80211_BAND_5GHZ || erp) {
		/*
		 * OFDM:
		 *
		 * N_DBPS = DATARATE x 4
		 * N_SYM = Ceiling((16+8xLENGTH+6) / N_DBPS)
		 *	(16 = SIGNAL time, 6 = tail bits)
		 * TXTIME = T_PREAMBLE + T_SIGNAL + T_SYM x N_SYM + Signal Ext
		 *
		 * T_SYM = 4 usec
		 * 802.11a - 18.5.2: aSIFSTime = 16 usec
		 * 802.11g - 19.8.4: aSIFSTime = 10 usec +
		 *	signal ext = 6 usec
		 */
		dur = 16; /* SIFS + signal ext */
		dur += 16; /* IEEE 802.11-2012 18.3.2.4: T_PREAMBLE = 16 usec */
		dur += 4; /* IEEE 802.11-2012 18.3.2.4: T_SIGNAL = 4 usec */

		/* IEEE 802.11-2012 18.3.2.4: all values above are:
		 *  * times 4 for 5 MHz
		 *  * times 2 for 10 MHz
		 */
		dur *= 1 << shift;

		/* rates should already consider the channel bandwidth,
		 * don't apply divisor again.
		 */
		dur += 4 * DIV_ROUND_UP((16 + 8 * (len + 4) + 6) * 10,
					4 * rate); /* T_SYM x N_SYM */
	} else {
		/*
		 * 802.11b or 802.11g with 802.11b compatibility:
		 * 18.3.4: TXTIME = PreambleLength + PLCPHeaderTime +
		 * Ceiling(((LENGTH+PBCC)x8)/DATARATE). PBCC=0.
		 *
		 * 802.11 (DS): 15.3.3, 802.11b: 18.3.4
		 * aSIFSTime = 10 usec
		 * aPreambleLength = 144 usec or 72 usec with short preamble
		 * aPLCPHeaderLength = 48 usec or 24 usec with short preamble
		 */
		dur = 10; /* aSIFSTime = 10 usec */
		dur += short_preamble ? (72 + 24) : (144 + 48);

		dur += DIV_ROUND_UP(8 * (len + 4) * 10, rate);
	}

	return dur;
}

/* Exported duration function for driver use */
__le16 ieee80211_generic_frame_duration(struct ieee80211_hw *hw,
					struct ieee80211_vif *vif,
					enum nl80211_band band,
					size_t frame_len,
					struct ieee80211_rate *rate)
{
	struct ieee80211_sub_if_data *sdata;
	u16 dur;
	int erp, shift = 0;
	bool short_preamble = false;

	erp = 0;
	if (vif) {
		sdata = vif_to_sdata(vif);
		short_preamble = sdata->vif.bss_conf.use_short_preamble;
		if (sdata->flags & IEEE80211_SDATA_OPERATING_GMODE)
			erp = rate->flags & IEEE80211_RATE_ERP_G;
		shift = ieee80211_vif_get_shift(vif);
	}

	dur = ieee80211_frame_duration(band, frame_len, rate->bitrate, erp,
				       short_preamble, shift);

	return cpu_to_le16(dur);
}
EXPORT_SYMBOL(ieee80211_generic_frame_duration);

__le16 ieee80211_rts_duration(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif, size_t frame_len,
			      const struct ieee80211_tx_info *frame_txctl)
{
	struct ieee80211_local *local = hw_to_local(hw);
	struct ieee80211_rate *rate;
	struct ieee80211_sub_if_data *sdata;
	bool short_preamble;
	int erp, shift = 0, bitrate;
	u16 dur;
	struct ieee80211_supported_band *sband;

	sband = local->hw.wiphy->bands[frame_txctl->band];

	short_preamble = false;

	rate = &sband->bitrates[frame_txctl->control.rts_cts_rate_idx];

	erp = 0;
	if (vif) {
		sdata = vif_to_sdata(vif);
		short_preamble = sdata->vif.bss_conf.use_short_preamble;
		if (sdata->flags & IEEE80211_SDATA_OPERATING_GMODE)
			erp = rate->flags & IEEE80211_RATE_ERP_G;
		shift = ieee80211_vif_get_shift(vif);
	}

	bitrate = DIV_ROUND_UP(rate->bitrate, 1 << shift);

	/* CTS duration */
	dur = ieee80211_frame_duration(sband->band, 10, bitrate,
				       erp, short_preamble, shift);
	/* Data frame duration */
	dur += ieee80211_frame_duration(sband->band, frame_len, bitrate,
					erp, short_preamble, shift);
	/* ACK duration */
	dur += ieee80211_frame_duration(sband->band, 10, bitrate,
					erp, short_preamble, shift);

	return cpu_to_le16(dur);
}
EXPORT_SYMBOL(ieee80211_rts_duration);

__le16 ieee80211_ctstoself_duration(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif,
				    size_t frame_len,
				    const struct ieee80211_tx_info *frame_txctl)
{
	struct ieee80211_local *local = hw_to_local(hw);
	struct ieee80211_rate *rate;
	struct ieee80211_sub_if_data *sdata;
	bool short_preamble;
	int erp, shift = 0, bitrate;
	u16 dur;
	struct ieee80211_supported_band *sband;

	sband = local->hw.wiphy->bands[frame_txctl->band];

	short_preamble = false;

	rate = &sband->bitrates[frame_txctl->control.rts_cts_rate_idx];
	erp = 0;
	if (vif) {
		sdata = vif_to_sdata(vif);
		short_preamble = sdata->vif.bss_conf.use_short_preamble;
		if (sdata->flags & IEEE80211_SDATA_OPERATING_GMODE)
			erp = rate->flags & IEEE80211_RATE_ERP_G;
		shift = ieee80211_vif_get_shift(vif);
	}

	bitrate = DIV_ROUND_UP(rate->bitrate, 1 << shift);

	/* Data frame duration */
	dur = ieee80211_frame_duration(sband->band, frame_len, bitrate,
				       erp, short_preamble, shift);
	if (!(frame_txctl->flags & IEEE80211_TX_CTL_NO_ACK)) {
		/* ACK duration */
		dur += ieee80211_frame_duration(sband->band, 10, bitrate,
						erp, short_preamble, shift);
	}

	return cpu_to_le16(dur);
}
EXPORT_SYMBOL(ieee80211_ctstoself_duration);

void ieee80211_propagate_queue_wake(struct ieee80211_local *local, int queue)
{
	struct ieee80211_sub_if_data *sdata;
	int n_acs = IEEE80211_NUM_ACS;

	if (local->ops->wake_tx_queue)
		return;

	if (local->hw.queues < IEEE80211_NUM_ACS)
		n_acs = 1;

	list_for_each_entry_rcu(sdata, &local->interfaces, list) {
		int ac;

		if (!sdata->dev)
			continue;

		if (sdata->vif.cab_queue != IEEE80211_INVAL_HW_QUEUE &&
		    local->queue_stop_reasons[sdata->vif.cab_queue] != 0)
			continue;

		for (ac = 0; ac < n_acs; ac++) {
			int ac_queue = sdata->vif.hw_queue[ac];

			if (ac_queue == queue ||
			    (sdata->vif.cab_queue == queue &&
			     local->queue_stop_reasons[ac_queue] == 0 &&
			     skb_queue_empty(&local->pending[ac_queue])))
				netif_wake_subqueue(sdata->dev, ac);
		}
	}
}

static void __ieee80211_wake_queue(struct ieee80211_hw *hw, int queue,
				   enum queue_stop_reason reason,
				   bool refcounted)
{
	struct ieee80211_local *local = hw_to_local(hw);

	trace_wake_queue(local, queue, reason);

	if (WARN_ON(queue >= hw->queues))
		return;

	if (!test_bit(reason, &local->queue_stop_reasons[queue]))
		return;

	if (!refcounted) {
		local->q_stop_reasons[queue][reason] = 0;
	} else {
		local->q_stop_reasons[queue][reason]--;
		if (WARN_ON(local->q_stop_reasons[queue][reason] < 0))
			local->q_stop_reasons[queue][reason] = 0;
	}

	if (local->q_stop_reasons[queue][reason] == 0)
		__clear_bit(reason, &local->queue_stop_reasons[queue]);

	if (local->queue_stop_reasons[queue] != 0)
		/* someone still has this queue stopped */
		return;

	if (skb_queue_empty(&local->pending[queue])) {
		rcu_read_lock();
		ieee80211_propagate_queue_wake(local, queue);
		rcu_read_unlock();
	} else
		tasklet_schedule(&local->tx_pending_tasklet);
}

void ieee80211_wake_queue_by_reason(struct ieee80211_hw *hw, int queue,
				    enum queue_stop_reason reason,
				    bool refcounted)
{
	struct ieee80211_local *local = hw_to_local(hw);
	unsigned long flags;

	spin_lock_irqsave(&local->queue_stop_reason_lock, flags);
	__ieee80211_wake_queue(hw, queue, reason, refcounted);
	spin_unlock_irqrestore(&local->queue_stop_reason_lock, flags);
lock();
	} else
		tasklet_sc0PGason_lock, flags);
lock();
	} else
		tasklet_sc0PGason_lock, flagsP%_cATING0PGason_lock, flagsP%_cATING0PGason_lock, flagsP%_cATING0PGason_lo%_cC>v