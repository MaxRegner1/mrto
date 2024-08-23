/*
 * Copyright (c) 2004-2011 Atheros Communications Inc.
 * Copyright (c) 2011-2012 Qualcomm Atheros, Inc.
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

#include <linux/ip.h>
#include <linux/in.h>
#include "core.h"
#include "debug.h"
#include "testmode.h"
#include "trace.h"
#include "../regd.h"
#include "../regd_common.h"

static int ath6kl_wmi_sync_point(struct wmi *wmi, u8 if_idx);

static const s32 wmi_rate_tbl[][2] = {
	/* {W/O SGI, with SGI} */
	{1000, 1000},
	{2000, 2000},
	{5500, 5500},
	{11000, 11000},
	{6000, 6000},
	{9000, 9000},
	{12000, 12000},
	{18000, 18000},
	{24000, 24000},
	{36000, 36000},
	{48000, 48000},
	{54000, 54000},
	{6500, 7200},
	{13000, 14400},
	{19500, 21700},
	{26000, 28900},
	{39000, 43300},
	{52000, 57800},
	{58500, 65000},
	{65000, 72200},
	{13500, 15000},
	{27000, 30000},
	{40500, 45000},
	{54000, 60000},
	{81000, 90000},
	{108000, 120000},
	{121500, 135000},
	{135000, 150000},
	{0, 0}
};

static const s32 wmi_rate_tbl_mcs15[][2] = {
	/* {W/O SGI, with SGI} */
	{1000, 1000},
	{2000, 2000},
	{5500, 5500},
	{11000, 11000},
	{6000, 6000},
	{9000, 9000},
	{12000, 12000},
	{18000, 18000},
	{24000, 24000},
	{36000, 36000},
	{48000, 48000},
	{54000, 54000},
	{6500, 7200},     /* HT 20, MCS 0 */
	{13000, 14400},
	{19500, 21700},
	{26000, 28900},
	{39000, 43300},
	{52000, 57800},
	{58500, 65000},
	{65000, 72200},
	{13000, 14400},   /* HT 20, MCS 8 */
	{26000, 28900},
	{39000, 43300},
	{52000, 57800},
	{78000, 86700},
	{104000, 115600},
	{117000, 130000},
	{130000, 144400}, /* HT 20, MCS 15 */
	{13500, 15000},   /*HT 40, MCS 0 */
	{27000, 30000},
	{40500, 45000},
	{54000, 60000},
	{81000, 90000},
	{108000, 120000},
	{121500, 135000},
	{135000, 150000},
	{27000, 30000},   /*HT 40, MCS 8 */
	{54000, 60000},
	{81000, 90000},
	{108000, 120000},
	{162000, 180000},
	{216000, 240000},
	{243000, 270000},
	{270000, 300000}, /*HT 40, MCS 15 */
	{0, 0}
};

/* 802.1d to AC mapping. Refer pg 57 of WMM-test-plan-v1.2 */
static const u8 up_to_ac[] = {
	WMM_AC_BE,
	WMM_AC_BK,
	WMM_AC_BK,
	WMM_AC_BE,
	WMM_AC_VI,
	WMM_AC_VI,
	WMM_AC_VO,
	WMM_AC_VO,
};

void ath6kl_wmi_set_control_ep(struct wmi *wmi, enum htc_endpoint_id ep_id)
{
	if (WARN_ON(ep_id == ENDPOINT_UNUSED || ep_id >= ENDPOINT_MAX))
		return;

	wmi->ep_id = ep_id;
}

enum htc_endpoint_id ath6kl_wmi_get_control_ep(struct wmi *wmi)
{
	return wmi->ep_id;
}

struct ath6kl_vif *ath6kl_get_vif_by_index(struct ath6kl *ar, u8 if_idx)
{
	struct ath6kl_vif *vif, *found = NULL;

	if (WARN_ON(if_idx > (ar->vif_max - 1)))
		return NULL;

	/* FIXME: Locking */
	spin_lock_bh(&ar->list_lock);
	list_for_each_entry(vif, &ar->vif_list, list) {
		if (vif->fw_vif_idx == if_idx) {
			found = vif;
			break;
		}
	}
	spin_unlock_bh(&ar->list_lock);

	return found;
}

/*  Performs DIX to 802.3 encapsulation for transmit packets.
 *  Assumes the entire DIX header is contiguous and that there is
 *  enough room in the buffer for a 802.3 mac header and LLC+SNAP headers.
 */
int ath6kl_wmi_dix_2_dot3(struct wmi *wmi, struct sk_buff *skb)
{
	struct ath6kl_llc_snap_hdr *llc_hdr;
	struct ethhdr *eth_hdr;
	size_t new_len;
	__be16 type;
	u8 *datap;
	u16 size;

	if (WARN_ON(skb == NULL))
		return -EINVAL;

	size = sizeof(struct ath6kl_llc_snap_hdr) + sizeof(struct wmi_data_hdr);
	if (skb_headroom(skb) < size)
		return -ENOMEM;

	eth_hdr = (struct ethhdr *) skb->data;
	type = eth_hdr->h_proto;

	if (!is_ethertype(be16_to_cpu(type))) {
		ath6kl_dbg(ATH6KL_DBG_WMI,
			   "%s: pkt is already in 802.3 format\n", __func__);
		return 0;
	}

	new_len = skb->len - sizeof(*eth_hdr) + sizeof(*llc_hdr);

	skb_push(skb, sizeof(struct ath6kl_llc_snap_hdr));
	datap = skb->data;

	eth_hdr->h_proto = cpu_to_be16(new_len);

	memcpy(datap, eth_hdr, sizeof(*eth_hdr));

	llc_hdr = (struct ath6kl_llc_snap_hdr *)(datap + sizeof(*eth_hdr));
	llc_hdr->dsap = 0xAA;
	llc_hdr->ssap = 0xAA;
	llc_hdr->cntl = 0x03;
	llc_hdr->org_code[0] = 0x0;
	llc_hdr->org_code[1] = 0x0;
	llc_hdr->org_code[2] = 0x0;
	llc_hdr->eth_type = type;

	return 0;
}

static int ath6kl_wmi_meta_add(struct wmi *wmi, struct sk_buff *skb,
			       u8 *version, void *tx_meta_info)
{
	struct wmi_tx_meta_v1 *v1;
	struct wmi_tx_meta_v2 *v2;

	if (WARN_ON(skb == NULL || version == NULL))
		return -EINVAL;

	switch (*version) {
	case WMI_META_VERSION_1:
		skb_push(skb, WMI_MAX_TX_META_SZ);
		v1 = (struct wmi_tx_meta_v1 *) skb->data;
		v1->pkt_id = 0;
		v1->rate_plcy_id = 0;
		*version = WMI_META_VERSION_1;
		break;
	case WMI_META_VERSION_2:
		skb_push(skb, WMI_MAX_TX_META_SZ);
		v2 = (struct wmi_tx_meta_v2 *) skb->data;
		memcpy(v2, (struct wmi_tx_meta_v2 *) tx_meta_info,
		       sizeof(struct wmi_tx_meta_v2));
		break;
	}

	return 0;
}

int ath6kl_wmi_data_hdr_add(struct wmi *wmi, struct sk_buff *skb,
			    u8 msg_type, u32 flags,
			    enum wmi_data_hdr_data_type data_type,
			    u8 meta_ver, void *tx_meta_info, u8 if_idx)
{
	struct wmi_data_hdr *data_hdr;
	int ret;

	if (WARN_ON(skb == NULL || (if_idx > wmi->parent_dev->vif_max - 1)))
		return -EINVAL;

	if (tx_meta_info) {
		ret = ath6kl_wmi_meta_add(wmi, skb, &meta_ver, tx_meta_info);
		if (ret)
			return ret;
	}

	skb_push(skb, sizeof(struct wmi_data_hdr));

	data_hdr = (struct wmi_data_hdr *)skb->data;
	memset(data_hdr, 0, sizeof(struct wmi_data_hdr));

	data_hdr->info = msg_type << WMI_DATA_HDR_MSG_TYPE_SHIFT;
	data_hdr->info |= data_type << WMI_DATA_HDR_DATA_TYPE_SHIFT;

	if (flags & WMI_DATA_HDR_FLAGS_MORE)
		data_hdr->info |= WMI_DATA_HDR_MORE;

	if (flags & WMI_DATA_HDR_FLAGS_EOSP)
		data_hdr->info3 |= cpu_to_le16(WMI_DATA_HDR_EOSP);

	data_hdr->info2 |= cpu_to_le16(meta_ver << WMI_DATA_HDR_META_SHIFT);
	data_hdr->info3 |= cpu_to_le16(if_idx & WMI_DATA_HDR_IF_IDX_MASK);

	return 0;
}

u8 ath6kl_wmi_determine_user_priority(u8 *pkt, u32 layer2_pri)
{
	struct iphdr *ip_hdr = (struct iphdr *) pkt;
	u8 ip_pri;

	/*
	 * Determine IPTOS priority
	 *
	 * IP-TOS - 8bits
	 *          : DSCP(6-bits) ECN(2-bits)
	 *          : DSCP - P2 P1 P0 X X X
	 * where (P2 P1 P0) form 802.1D
	 */
	ip_pri = ip_hdr->tos >> 5;
	ip_pri &= 0x7;

	if ((layer2_pri & 0x7) > ip_pri)
		return (u8) layer2_pri & 0x7;
	else
		return ip_pri;
}

u8 ath6kl_wmi_get_traffic_class(u8 user_priority)
{
	return  up_to_ac[user_priority & 0x7];
}

int ath6kl_wmi_implicit_create_pstream(struct wmi *wmi, u8 if_idx,
				       struct sk_buff *skb,
				       u32 layer2_priority, bool wmm_enabled,
				       u8 *ac)
{
	struct wmi_data_hdr *data_hdr;
	struct ath6kl_llc_snap_hdr *llc_hdr;
	struct wmi_create_pstream_cmd cmd;
	u32 meta_size, hdr_size;
	u16 ip_type = IP_ETHERTYPE;
	u8 stream_exist, usr_pri;
	u8 traffic_class = WMM_AC_BE;
	u8 *datap;

	if (WARN_ON(skb == NULL))
		return -EINVAL;

	datap = skb->data;
	data_hdr = (struct wmi_data_hdr *) datap;

	meta_size = ((le16_to_cpu(data_hdr->info2) >> WMI_DATA_HDR_META_SHIFT) &
		     WMI_DATA_HDR_META_MASK) ? WMI_MAX_TX_META_SZ : 0;

	if (!wmm_enabled) {
		/* If WMM is disabled all traffic goes as BE traffic */
		usr_pri = 0;
	} else {
		hdr_size = sizeof(struct ethhdr);

		llc_hdr = (struct ath6kl_llc_snap_hdr *)(datap +
							 sizeof(struct
								wmi_data_hdr) +
							 meta_size + hdr_size);

		if (llc_hdr->eth_type == htons(ip_type)) {
			/*
			 * Extract the endpoint info from the TOS field
			 * in the IP header.
			 */
			usr_pri =
			   ath6kl_wmi_determine_user_priority(((u8 *) llc_hdr) +
					sizeof(struct ath6kl_llc_snap_hdr),
					layer2_priority);
		} else {
			usr_pri = layer2_priority & 0x7;
		}

		/*
		 * Queue the EAPOL frames in the same WMM_AC_VO queue
		 * as that of management frames.
		 */
		if (skb->protocol == cpu_to_be16(ETH_P_PAE))
			usr_pri = WMI_VOICE_USER_PRIORITY;
	}

	/*
	 * workaround for WMM S5
	 *
	 * FIXME: wmi->traffic_class is always 100 so this test doesn't
	 * make sense
	 */
	if ((wmi->traffic_class == WMM_AC_VI) &&
	    ((usr_pri == 5) || (usr_pri == 4)))
		usr_pri = 1;

	/* Convert user priority to traffic class */
	traffic_class = up_to_ac[usr_pri & 0x7];

	wmi_data_hdr_set_up(data_hdr, usr_pri);

	spin_lock_bh(&wmi->lock);
	stream_exist = wmi->fat_pipe_exist;
	spin_unlock_bh(&wmi->lock);

	if (!(stream_exist & (1 << traffic_class))) {
		memset(&cmd, 0, sizeof(cmd));
		cmd.traffic_class = traffic_class;
		cmd.user_pri = usr_pri;
		cmd.inactivity_int =
			cpu_to_le32(WMI_IMPLICIT_PSTREAM_INACTIVITY_INT);
		/* Implicit streams are created with TSID 0xFF */
		cmd.tsid = WMI_IMPLICIT_PSTREAM;
		ath6kl_wmi_create_pstream_cmd(wmi, if_idx, &cmd);
	}

	*ac = traffic_class;

	return 0;
}

int ath6kl_wmi_dot11_hdr_remove(struct wmi *wmi, struct sk_buff *skb)
{
	struct ieee80211_hdr_3addr *pwh, wh;
	struct ath6kl_llc_snap_hdr *llc_hdr;
	struct ethhdr eth_hdr;
	u32 hdr_size;
	u8 *datap;
	__le16 sub_type;

	if (WARN_ON(skb == NULL))
		return -EINVAL;

	datap = skb->data;
	pwh = (struct ieee80211_hdr_3addr *) datap;

	sub_type = pwh->frame_control & cpu_to_le16(IEEE80211_FCTL_STYPE);

	memcpy((u8 *) &wh, datap, sizeof(struct ieee80211_hdr_3addr));

	/* Strip off the 802.11 header */
	if (sub_type == cpu_to_le16(IEEE80211_STYPE_QOS_DATA)) {
		hdr_size = roundup(sizeof(struct ieee80211_qos_hdr),
				   sizeof(u32));
		skb_pull(skb, hdr_size);
	} else if (sub_type == cpu_to_le16(IEEE80211_STYPE_DATA)) {
		skb_pull(skb, sizeof(struct ieee80211_hdr_3addr));
	}

	datap = skb->data;
	llc_hdr = (struct ath6kl_llc_snap_hdr *)(datap);

	memset(&eth_hdr, 0, sizeof(eth_hdr));
	eth_hdr.h_proto = llc_hdr->eth_type;

	switch ((le16_to_cpu(wh.frame_control)) &
		(IEEE80211_FCTL_FROMDS | IEEE80211_FCTL_TODS)) {
	case IEEE80211_FCTL_TODS:
		memcpy(eth_hdr.h_dest, wh.addr3, ETH_ALEN);
		memcpy(eth_hdr.h_source, wh.addr2, ETH_ALEN);
		break;
	case IEEE80211_FCTL_FROMDS:
		memcpy(eth_hdr.h_dest, wh.addr1, ETH_ALEN);
		memcpy(eth_hdr.h_source, wh.addr3, ETH_ALEN);
		break;
	case IEEE80211_FCTL_FROMDS | IEEE80211_FCTL_TODS:
		break;
	default:
		memcpy(eth_hdr.h_dest, wh.addr1, ETH_ALEN);
		memcpy(eth_hdr.h_source, wh.addr2, ETH_ALEN);
		break;
	}

	skb_pull(skb, sizeof(struct ath6kl_llc_snap_hdr));
	skb_push(skb, sizeof(eth_hdr));

	datap = skb->data;

	memcpy(datap, &eth_hdr, sizeof(eth_hdr));

	return 0;
}

/*
 * Performs 802.3 to DIX encapsulation for received packets.
 * Assumes the entire 802.3 header is contiguous.
 */
int ath6kl_wmi_dot3_2_dix(struct sk_buff *skb)
{
	struct ath6kl_llc_snap_hdr *llc_hdr;
	struct ethhdr eth_hdr;
	u8 *datap;

	if (WARN_ON(skb == NULL))
		return -EINVAL;

	datap = skb->data;

	memcpy(&eth_hdr, datap, sizeof(eth_hdr));

	llc_hdr = (struct ath6kl_llc_snap_hdr *) (datap + sizeof(eth_hdr));
	eth_hdr.h_proto = llc_hdr->eth_type;

	skb_pull(skb, sizeof(struct ath6kl_llc_snap_hdr));
	datap = skb->data;

	memcpy(datap, &eth_hdr, sizeof(eth_hdr));

	return 0;
}

static int ath6kl_wmi_tx_complete_event_rx(u8 *datap, int len)
{
	struct tx_complete_msg_v1 *msg_v1;
	struct wmi_tx_complete_event *evt;
	int index;
	u16 size;

	evt = (struct wmi_tx_complete_event *) datap;

	ath6kl_dbg(ATH6KL_DBG_WMI, "comp: %d %d %d\n",
		   evt->num_msg, evt->msg_len, evt->msg_type);

	for (index = 0; index < evt->num_msg; index++) {
		size = sizeof(struct wmi_tx_complete_event) +
		    (index * sizeof(struct tx_complete_msg_v1));
		msg_v1 = (struct tx_complete_msg_v1 *)(datap + size);

		ath6kl_dbg(ATH6KL_DBG_WMI, "msg: %d %d %d %d\n",
			   msg_v1->status, msg_v1->pkt_id,
			   msg_v1->rate_idx, msg_v1->ack_failures);
	}

	return 0;
}

static int ath6kl_wmi_remain_on_chnl_event_rx(struct wmi *wmi, u8 *datap,
					      int len, struct ath6kl_vif *vif)
{
	struct wmi_remain_on_chnl_event *ev;
	u32 freq;
	u32 dur;
	struct ieee80211_channel *chan;
	struct ath6kl *ar = wmi->parent_dev;
	u32 id;

	if (len < sizeof(*ev))
		return -EINVAL;

	ev = (struct wmi_remain_on_chnl_event *) datap;
	freq = le32_to_cpu(ev->freq);
	dur = le32_to_cpu(ev->duration);
	ath6kl_dbg(ATH6KL_DBG_WMI, "remain_on_chnl: freq=%u dur=%u\n",
		   freq, dur);
	chan = ieee80211_get_channel(ar->wiphy, freq);
	if (!chan) {
		ath6kl_dbg(ATH6KL_DBG_WMI,
			   "remain_on_chnl: Unknown channel (freq=%u)\n",
			   freq);
		return -EINVAL;
	}
	id = vif->last_roc_id;
	cfg80211_ready_on_channel(&vif->wdev, id, chan,
				  dur, GFP_ATOMIC);

	return 0;
}

static int ath6kl_wmi_cancel_remain_on_chnl_event_rx(struct wmi *wmi,
						     u8 *datap, int len,
						     struct ath6kl_vif *vif)
{
	struct wmi_cancel_remain_on_chnl_event *ev;
	u32 freq;
	u32 dur;
	struct ieee80211_channel *chan;
	struct ath6kl *ar = wmi->parent_dev;
	u32 id;

	if (len < sizeof(*ev))
		return -EINVAL;

	ev = (struct wmi_cancel_remain_on_chnl_event *) datap;
	freq = le32_to_cpu(ev->freq);
	dur = le32_to_cpu(ev->duration);
	ath6kl_dbg(ATH6KL_DBG_WMI,
		   "cancel_remain_on_chnl: freq=%u dur=%u status=%u\n",
		   freq, dur, ev->status);
	chan = ieee80211_get_channel(ar->wiphy, freq);
	if (!chan) {
		ath6kl_dbg(ATH6KL_DBG_WMI,
			   "cancel_remain_on_chnl: Unknown channel (freq=%u)\n",
			   freq);
		return -EINVAL;
	}
	if (vif->last_cancel_roc_id &&
	    vif->last_cancel_roc_id + 1 == vif->last_roc_id)
		id = vif->last_cancel_roc_id; /* event for cancel command */
	else
		id = vif->last_roc_id; /* timeout on uncanceled r-o-c */
	vif->last_cancel_roc_id = 0;
	cfg80211_remain_on_channel_expired(&vif->wdev, id, chan, GFP_ATOMIC);

	return 0;
}

static int ath6kl_wmi_tx_status_event_rx(struct wmi *wmi, u8 *datap, int len,
					 struct ath6kl_vif *vif)
{
	struct wmi_tx_status_event *ev;
	u32 id;

	if (len < sizeof(*ev))
		return -EINVAL;

	ev = (struct wmi_tx_status_event *) datap;
	id = le32_to_cpu(ev->id);
	ath6kl_dbg(ATH6KL_DBG_WMI, "tx_status: id=%x ack_status=%u\n",
		   id, ev->ack_status);
	if (wmi->last_mgmt_tx_frame) {
		cfg80211_mgmt_tx_status(&vif->wdev, id,
					wmi->last_mgmt_tx_frame,
					wmi->last_mgmt_tx_frame_len,
					!!ev->ack_status, GFP_ATOMIC);
		kfree(wmi->last_mgmt_tx_frame);
		wmi->last_mgmt_tx_frame = NULL;
		wmi->last_mgmt_tx_frame_len = 0;
	}

	return 0;
}

static int ath6kl_wmi_rx_probe_req_event_rx(struct wmi *wmi, u8 *datap, int len,
					    struct ath6kl_vif *vif)
{
	struct wmi_p2p_rx_probe_req_event *ev;
	u32 freq;
	u16 dlen;

	if (len < sizeof(*ev))
		return -EINVAL;

	ev = (struct wmi_p2p_rx_probe_req_event *) datap;
	freq = le32_to_cpu(ev->freq);
	dlen = le16_to_cpu(ev->len);
	if (datap + len < ev->data + dlen) {
		ath6kl_err("invalid wmi_p2p_rx_probe_req_event: len=%d dlen=%u\n",
			   len, dlen);
		return -EINVAL;
	}
	ath6kl_dbg(ATH6KL_DBG_WMI,
		   "rx_probe_req: len=%u freq=%u probe_req_report=%d\n",
		   dlen, freq, vif->probe_req_report);

	if (vif->probe_req_report || vif->nw_type == AP_NETWORK)
		cfg80211_rx_mgmt(&vif->wdev, freq, 0, ev->data, dlen, 0);

	return 0;
}

static int ath6kl_wmi_p2p_capabilities_event_rx(u8 *datap, int len)
{
	struct wmi_p2p_capabilities_event *ev;
	u16 dlen;

	if (len < sizeof(*ev))
		return -EINVAL;

	ev = (struct wmi_p2p_capabilities_event *) datap;
	dlen = le16_to_cpu(ev->len);
	ath6kl_dbg(ATH6KL_DBG_WMI, "p2p_capab: len=%u\n", dlen);

	return 0;
}

static int ath6kl_wmi_rx_action_event_rx(struct wmi *wmi, u8 *datap, int len,
					 struct ath6kl_vif *vif)
{
	struct wmi_rx_action_event *ev;
	u32 freq;
	u16 dlen;

	if (len < sizeof(*ev))
		return -EINVAL;

	ev = (struct wmi_rx_action_event *) datap;
	freq = le32_to_cpu(ev->freq);
	dlen = le16_to_cpu(ev->len);
	if (datap + len < ev->data + dlen) {
		ath6kl_err("invalid wmi_rx_action_event: len=%d dlen=%u\n",
			   len, dlen);
		return -EINVAL;
	}
	ath6kl_dbg(ATH6KL_DBG_WMI, "rx_action: len=%u freq=%u\n", dlen, freq);
	cfg80211_rx_mgmt(&vif->wdev, freq, 0, ev->data, dlen, 0);

	return 0;
}

static int ath6kl_wmi_p2p_info_event_rx(u8 *datap, int len)
{
	struct wmi_p2p_info_event *ev;
	u32 flags;
	u16 dlen;

	if (len < sizeof(*ev))
		return -EINVAL;

	ev = (struct wmi_p2p_info_event *) datap;
	flags = le32_to_cpu(ev->info_req_flags);
	dlen = le16_to_cpu(ev->len);
	ath6kl_dbg(ATH6KL_DBG_WMI, "p2p_info: flags=%x len=%d\n", flags, dlen);

	if (flags & P2P_FLAG_CAPABILITIES_REQ) {
		struct wmi_p2p_capabilities *cap;
		if (dlen < sizeof(*cap))
			return -EINVAL;
		cap = (struct wmi_p2p_capabilities *) ev->data;
		ath6kl_dbg(ATH6KL_DBG_WMI, "p2p_info: GO Power Save = %d\n",
			   cap->go_power_save);
	}

	if (flags & P2P_FLAG_MACADDR_REQ) {
		struct wmi_p2p_macaddr *mac;
		if (dlen < sizeof(*mac))
			return -EINVAL;
		mac = (struct wmi_p2p_macaddr *) ev->data;
		ath6kl_dbg(ATH6KL_DBG_WMI, "p2p_info: MAC Address = %pM\n",
			   mac->mac_addr);
	}

	if (flags & P2P_FLAG_HMODEL_REQ) {
		struct wmi_p2p_hmodel *mod;
		if (dlen < sizeof(*mod))
			return -EINVAL;
		mod = (struct wmi_p2p_hmodel *) ev->data;
		ath6kl_dbg(ATH6KL_DBG_WMI, "p2p_info: P2P Model = %d (%s)\n",
			   mod->p2p_model,
			   mod->p2p_model ? "host" : "firmware");
	}
	return 0;
}

static inline struct sk_buff *ath6kl_wmi_get_new_buf(u32 size)
{
	struct sk_buff *skb;

	skb = ath6kl_buf_alloc(size);
	if (!skb)
		return NULL;

	skb_put(skb, size);
	if (size)
		memset(skb->data, 0, size);

	return skb;
}

/* Send a "simple" wmi command -- one with no arguments */
static int ath6kl_wmi_simple_cmd(struct wmi *wmi, u8 if_idx,
				 enum wmi_cmd_id cmd_id)
{
	struct sk_buff *skb;
	int ret;

	skb = ath6kl_wmi_get_new_buf(0);
	if (!skb)
		return -ENOMEM;

	ret = ath6kl_wmi_cmd_send(wmi, if_idx, skb, cmd_id, NO_SYNC_WMIFLAG);

	return ret;
}

static int ath6kl_wmi_ready_event_rx(struct wmi *wmi, u8 *datap, int len)
{
	struct wmi_ready_event_2 *ev = (struct wmi_ready_event_2 *) datap;

	if (len < sizeof(struct wmi_ready_event_2))
		return -EINVAL;

	ath6kl_ready_event(wmi->parent_dev, ev->mac_addr,
			   le32_to_cpu(ev->sw_version),
			   le32_to_cpu(ev->abi_version), ev->phy_cap);

	return 0;
}

/*
 * Mechanism to modify the roaming behavior in the firmware. The lower rssi
 * at which the station has to roam can be passed with
 * WMI_SET_LRSSI_SCAN_PARAMS. Subtract 96 from RSSI to get the signal level
 * in dBm.
 */
int ath6kl_wmi_set_roam_lrssi_cmd(struct wmi *wmi, u8 lrssi)
{
	struct sk_buff *skb;
	struct roam_ctrl_cmd *cmd;

	skb = ath6kl_wmi_get_new_buf(sizeof(*cmd));
	if (!skb)
		return -ENOMEM;

	cmd = (struct roam_ctrl_cmd *) skb->data;

	cmd->info.params.lrssi_scan_period = cpu_to_le16(DEF_LRSSI_SCAN_PERIOD);
	cmd->info.params.lrssi_scan_threshold = a_cpu_to_sle16(lrssi +
						       DEF_SCAN_FOR_ROAM_INTVL);
	cmd->info.params.lrssi_roam_threshold = a_cpu_to_sle16(lrssi);
	cmd->info.params.roam_rssi_floor = DEF_LRSSI_ROAM_FLOOR;
	cmd->roam_ctrl = WMI_SET_LRSSI_SCAN_PARAMS;

	ath6kl_wmi_cmd_send(wmi, 0, skb, WMI_SET_ROAM_CTRL_CMDID,
			    NO_SYNC_WMIFLAG);

	return 0;
}

int ath6kl_wmi_force_roam_cmd(struct wmi *wmi, const u8 *bssid)
{
	struct sk_buff *skb;
	struct roam_ctrl_cmd *cmd;

	skb = ath6kl_wmi_get_new_buf(sizeof(*cmd));
	if (!skb)
		return -ENOMEM;

	cmd = (struct roam_ctrl_cmd *) skb->data;

	memcpy(cmd->info.bssid, bssid, ETH_ALEN);
	cmd->roam_ctrl = WMI_FORCE_ROAM;

	ath6kl_dbg(ATH6KL_DBG_WMI, "force roam to %pM\n", bssid);
	return ath6kl_wmi_cmd_send(wmi, 0, skb, WMI_SET_ROAM_CTRL_CMDID,
				   NO_SYNC_WMIFLAG);
}

int ath6kl_wmi_ap_set_beacon_intvl_cmd(struct wmi *wmi, u8 if_idx,
				       u32 beacon_intvl)
{
	struct sk_buff *skb;
	struct set_beacon_int_cmd *cmd;

	skb = ath6kl_wmi_get_new_buf(sizeof(*cmd));
	if (!skb)
		return -ENOMEM;

	cmd = (struct set_beacon_int_cmd *) skb->data;

	cmd->beacon_intvl = cpu_to_le32(beacon_intvl);
	return ath6kl_wmi_cmd_send(wmi, if_idx, skb,
				   WMI_SET_BEACON_INT_CMDID, NO_SYNC_WMIFLAG);
}

int ath6kl_wmi_ap_set_dtim_cmd(struct wmi *wmi, u8 if_idx, u32 dtim_period)
{
	struct sk_buff *skb;
	struct set_dtim_cmd *cmd;

	skb = ath6kl_wmi_get_new_buf(sizeof(*cmd));
	if (!skb)
		return -ENOMEM;

	cmd = (struct set_dtim_cmd *) skb->data;

	cmd->dtim_period = cpu_to_le32(dtim_period);
	return ath6kl_wmi_cmd_send(wmi, if_idx, skb,
				   WMI_AP_SET_DTIM_CMDID, NO_SYNC_WMIFLAG);
}

int ath6kl_wmi_set_roam_mode_cmd(struct wmi *wmi, enum wmi_roam_mode mode)
{
	struct sk_buff *skb;
	struct roam_ctrl_cmd *cmd;

	skb = ath6kl_wmi_get_new_buf(sizeof(*cmd));
	if (!skb)
		return -ENOMEM;

	cmd = (struct roam_ctrl_cmd *) skb->data;

	cmd->info.roam_mode = mode;
	cmd->roam_ctrl = WMI_SET_ROAM_MODE;

	ath6kl_dbg(ATH6KL_DBG_WMI, "set roam mode %d\n", mode);
	return ath6kl_wmi_cmd_send(wmi, 0, skb, WMI_SET_ROAM_CTRL_CMDID,
				   NO_SYNC_WMIFLAG);
}

static int ath6kl_wmi_connect_event_rx(struct wmi *wmi, u8 *datap, int len,
				       struct ath6kl_vif *vif)
{
	struct wmi_connect_event *ev;
	u8 *pie, *peie;

	if (len < sizeof(struct wmi_connect_event))
		return -EINVAL;

	ev = (struct wmi_connect_event *) datap;

	if (vif->nw_type == AP_NETWORK) {
		/* AP mode start/STA connected event */
		struct net_device *dev = vif->ndev;
		if (memcmp(dev->dev_addr, ev->u.ap_bss.bssid, ETH_ALEN) == 0) {
			ath6kl_dbg(ATH6KL_DBG_WMI,
				   "%s: freq %d bssid %pM (AP started)\n",
				   __func__, le16_to_cpu(ev->u.ap_bss.ch),
				   ev->u.ap_bss.bssid);
			ath6kl_connect_ap_mode_bss(
				vif, le16_to_cpu(ev->u.ap_bss.ch));
		} else {
			ath6kl_dbg(ATH6KL_DBG_WMI,
				   "%s: aid %u mac_addr %pM auth=%u keymgmt=%u cipher=%u apsd_info=%u (STA connected)\n",
				   __func__, ev->u.ap_sta.aid,
				   ev->u.ap_sta.mac_addr,
				   ev->u.ap_sta.auth,
				   ev->u.ap_sta.keymgmt,
				   le16_to_cpu(ev->u.ap_sta.cipher),
				   ev->u.ap_sta.apsd_info);

			ath6kl_connect_ap_mode_sta(
				vif, ev->u.ap_sta.aid, ev->u.ap_sta.mac_addr,
				ev->u.ap_sta.keymgmt,
				le16_to_cpu(ev->u.ap_sta.cipher),
				ev->u.ap_sta.auth, ev->assoc_req_len,
				ev->assoc_info + ev->beacon_ie_len,
				ev->u.ap_sta.apsd_info);
		}
		return 0;
	}

	/* STA/IBSS mode connection event */

	ath6kl_dbg(ATH6KL_DBG_WMI,
		   "wmi event connect freq %d bssid %pM listen_intvl %d beacon_intvl %d type %d\n",
		   le16_to_cpu(ev->u.sta.ch), ev->u.sta.bssid,
		   le16_to_cpu(ev->u.sta.listen_intvl),
		   le16_to_cpu(ev->u.sta.beacon_intvl),
		   le32_to_cpu(ev->u.sta.nw_type));

	/* Start of assoc rsp IEs */
	pie = ev->assoc_info + ev->beacon_ie_len +
	      ev->assoc_req_len + (sizeof(u16) * 3); /* capinfo, status, aid */

	/* End of assoc rsp IEs */
	peie = ev->assoc_info + ev->beacon_ie_len + ev->assoc_req_len +
	    ev->assoc_resp_len;

	while (pie < peie) {
		switch (*pie) {
		case WLAN_EID_VENDOR_SPECIFIC:
			if (pie[1] > 3 && pie[2] == 0x00 && pie[3] == 0x50 &&
			    pie[4] == 0xf2 && pie[5] == WMM_OUI_TYPE) {
				/* WMM OUT (00:50:F2) */
				if (pie[1] > 5 &&
				    pie[6] == WMM_PARAM_OUI_SUBTYPE)
					wmi->is_wmm_enabled = true;
			}
			break;
		}

		if (wmi->is_wmm_enabled)
			break;

		pie += pie[1] + 2;
	}

	ath6kl_connect_event(vif, le16_to_cpu(ev->u.sta.ch),
			     ev->u.sta.bssid,
			     le16_to_cpu(ev->u.sta.listen_intvl),
			     le16_to_cpu(ev->u.sta.beacon_intvl),
			     le32_to_cpu(ev->u.sta.nw_type),
			     ev->beacon_ie_len, ev->assoc_req_len,
			     ev->assoc_resp_len, ev->assoc_info);

	return 0;
}

static struct country_code_to_enum_rd *
ath6kl_regd_find_country(u16 countryCode)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(allCountries); i++) {
		if (allCountries[i].countryCode == countryCode)
			return &allCountries[i];
	}

	return NULL;
}

static struct reg_dmn_pair_mapping *
ath6kl_get_regpair(u16 regdmn)
{
	int i;

	if (regdmn == NO_ENUMRD)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(regDomainPairs); i++) {
		if (regDomainPairs[i].reg_domain == regdmn)
			return &regDomainPairs[i];
	}

	return NULL;
}

static struct country_code_to_enum_rd *
ath6kl_regd_find_country_by_rd(u16 regdmn)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(allCountries); i++) {
		if (allCountries[i].regDmnEnum == regdmn)
			return &allCountries[i];
	}

	return NULL;
}

static void ath6kl_wmi_regdomain_event(struct wmi *wmi, u8 *datap, int len)
{
	struct ath6kl_wmi_regdomain *ev;
	struct country_code_to_enum_rd *country = NULL;
	struct reg_dmn_pair_mapping *regpair = NULL;
	char alpha2[2];
	u32 reg_code;

	ev = (struct ath6kl_wmi_regdomain *) datap;
	reg_code = le32_to_cpu(ev->reg_code);

	if ((reg_code >> ATH6KL_COUNTRY_RD_SHIFT) & COUNTRY_ERD_FLAG) {
		country = ath6kl_regd_find_country((u16) reg_code);
	} else if (!(((u16) reg_code & WORLD_SKU_MASK) == WORLD_SKU_PREFIX)) {
		regpair = ath6kl_get_regpair((u16) reg_code);
		country = ath6kl_regd_find_country_by_rd((u16) reg_code);
		if (regpair)
			ath6kl_dbg(ATH6KL_DBG_WMI, "Regpair used: 0x%0x\n",
				   regpair->reg_domain);
		else
			ath6kl_warn("Regpair not found reg_code 0x%0x\n",
				    reg_code);
	}

	if (country && wmi->parent_dev->wiphy_registered) {
		alpha2[0] = country->isoName[0];
		alpha2[1] = country->isoName[1];

		regulatory_hint(wmi->parent_dev->wiphy, alpha2);

		ath6kl_dbg(ATH6KL_DBG_WMI, "Country alpha2 being used: %c%c\n",
			   alpha2[0], alpha2[1]);
	}
}

static int ath6kl_wmi_disconnect_event_rx(struct wmi *wmi, u8 *datap, int len,
					  struct ath6kl_vif *vif)
{
	struct wmi_disconnect_event *ev;
	wmi->traffic_class = 100;

	if (len < sizeof(struct wmi_disconnect_event))
		return -EINVAL;

	ev = (struct wmi_disconnect_event *) datap;

	ath6kl_dbg(ATH6KL_DBG_WMI,
		   "wmi event disconnect proto_reason %d bssid %pM wmi_reason %d assoc_resp_len %d\n",
		   le16_to_cpu(ev->proto_reason_status), ev->bssid,
		   ev->disconn_reason, ev->assoc_resp_len);

	wmi->is_wmm_enabled = false;

	ath6kl_disconnect_event(vif, ev->disconn_reason,
				ev->bssid, ev->assoc_resp_len, ev->assoc_info,
				le16_to_cpu(ev->proto_reason_status));

	return 0;
}

static int ath6kl_wmi_peer_node_event_rx(struct wmi *wmi, u8 *datap, int len)
{
	struct wmi_peer_node_event *ev;

	if (len < sizeof(struct wmi_peer_node_event))
		return -EINVAL;

	ev = (struct wmi_peer_node_event *) datap;

	if (ev->event_code == PEER_NODE_JOIN_EVENT)
		ath6kl_dbg(ATH6KL_DBG_WMI, "joined node with mac addr: %pM\n",
			   ev->peer_mac_addr);
	else if (ev->event_code == PEER_NODE_LEAVE_EVENT)
		ath6kl_dbg(ATH6KL_DBG_WMI, "left node with mac addr: %pM\n",
			   ev->peer_mac_addr);

	return 0;
}

static int ath6kl_wmi_tkip_micerr_event_rx(struct wmi *wmi, u8 *datap, int len,
					   struct ath6kl_vif *vif)
{
	struct wmi_tkip_micerr_event *ev;

	if (len < sizeof(struct wmi_tkip_micerr_event))
		return -EINVAL;

	ev = (struct wmi_tkip_micerr_event *) datap;

	ath6kl_tkip_micerr_event(vif, ev->key_id, ev->is_mcast);

	return 0;
}

void ath6kl_wmi_sscan_timer(unsigned long ptr)
{
	struct ath6kl_vif *vif = (struct ath6kl_vif *) ptr;

	cfg80211_sched_scan_results(vif->ar->wiphy, 0);
}

static int ath6kl_wmi_bssinfo_event_rx(struct wmi *wmi, u8 *datap, int len,
				       struct ath6kl_vif *vif)
{
	struct wmi_bss_info_hdr2 *bih;
	u8 *buf;
	struct ieee80211_channel *channel;
	struct ath6kl *ar = wmi->parent_dev;
	struct cfg80211_bss *bss;

	if (len <= sizeof(struct wmi_bss_info_hdr2))
		return -EINVAL;

	bih = (struct wmi_bss_info_hdr2 *) datap;
	buf = datap + sizeof(struct wmi_bss_info_hdr2);
	len -= sizeof(struct wmi_bss_info_hdr2);

	ath6kl_dbg(ATH6KL_DBG_WMI,
		   "bss info evt - ch %u, snr %d, rssi %d, bssid \"%pM\" "
		   "frame_type=%d\n",
		   bih->ch, bih->snr, bih->snr - 95, bih->bssid,
		   bih->frame_type);

	if (bih->frame_type != BEACON_FTYPE &&
	    bih->frame_type != PROBERESP_FTYPE)
		return 0; /* Only update BSS table for now */

	if (bih->frame_type == BEACON_FTYPE &&
	    test_bit(CLEAR_BSSFILTER_ON_BEACON, &vif->flags)) {
		clear_bit(CLEAR_BSSFILTER_ON_BEACON, &vif->flags);
		ath6kl_wmi_bssfilter_cmd(ar->wmi, vif->fw_vif_idx,
					 NONE_BSS_FILTER, 0);
	}

	channel = ieee80211_get_channel(ar->wiphy, le16_to_cpu(bih->ch));
	if (channel == NULL)
		return -EINVAL;

	if (len < 8 + 2 + 2)
		return -EINVAL;

	if (bih->frame_type == BEACON_FTYPE &&
	    test_bit(CONNECTED, &vif->flags) &&
	    memcmp(bih->bssid, vif->bssid, ETH_ALEN) == 0) {
		const u8 *tim;
		tim = cfg80211_find_ie(WLAN_EID_TIM, buf + 8 + 2 + 2,
				       len - 8 - 2 - 2);
		if (tim && tim[1] >= 2) {
			vif->assoc_bss_dtim_period = tim[3];
			set_bit(DTIM_PERIOD_AVAIL, &vif->flags);
		}
	}

	bss = cfg80211_inform_bss(ar->wiphy, channel,
				  bih->frame_type == BEACON_FTYPE ?
					CFG80211_BSS_FTYPE_BEACON :
					CFG80211_BSS_FTYPE_PRESP,
				  bih->bssid, get_unaligned_le64((__le64 *)buf),
				  get_unaligned_le16(((__le16 *)buf) + 5),
				  get_unaligned_le16(((__le16 *)buf) + 4),
				  buf + 8 + 2 + 2, len - 8 - 2 - 2,
				  (bih->snr - 95) * 100, GFP_ATOMIC);
	if (bss == NULL)
		return -ENOMEM;
	cfg80211_put_bss(ar->wiphy, bss);

	/*
	 * Firmware doesn't return any event when scheduled scan has
	 * finished, so we need to use a timer to find out when there are
	 * no more results.
	 *
	 * The timer is started from the first bss info received, otherwise
	 * the timer would not ever fire if the scan interval is short
	 * enough.
	 */
	if (test_bit(SCHED_SCANNING, &vif->flags) &&
	    !timer_pending(&vif->sched_scan_timer)) {
		mod_timer(&vif->sched_scan_timer, jiffies +
			  msecs_to_jiffies(ATH6KL_SCHED_SCAN_RESULT_DELAY));
	}

	return 0;
}

/* Inactivity timeout of a fatpipe(pstream) at the target */
static int ath6kl_wmi_pstream_timeout_event_rx(struct wmi *wmi, u8 *datap,
					       int len)
{
	struct wmi_pstream_timeout_event *ev;

	if (len < sizeof(struct wmi_pstream_timeout_event))
		return -EINVAL;

	ev = (struct wmi_pstream_timeout_event *) datap;
	if (ev->traffic_class >= WMM_NUM_AC) {
		ath6kl_err("invalid traffic class: %d\n", ev->traffic_class);
		return -EINVAL;
	}

	/*
	 * When the pstream (fat pipe == AC) timesout, it means there were
	 * no thinStreams within this pstream & it got implicitly created
	 * due to data flow on this AC. We start the inactivity timer only
	 * for implicitly created pstream. Just reset the host state.
	 */
	spin_lock_bh(&wmi->lock);
	wmi->stream_exist_for_ac[ev->traffic_class] = 0;
	wmi->fat_pipe_exist &= ~(1 << ev->traffic_class);
	spin_unlock_bh(&wmi->lock);

	/* Indicate inactivity to driver layer for this fatpipe (pstream) */
	ath6kl_indicate_tx_activity(wmi->parent_dev, ev->traffic_class, false);

	return 0;
}

static int ath6kl_wmi_bitrate_reply_rx(struct wmi *wmi, u8 *datap, int len)
{
	struct wmi_bit_rate_reply *reply;
	s32 rate;
	u32 sgi, index;

	if (len < sizeof(struct wmi_bit_rate_reply))
		return -EINVAL;

	reply = (struct wmi_bit_rate_reply *) datap;

	ath6kl_dbg(ATH6KL_DBG_WMI, "rateindex %d\n", reply->rate_index);

	if (reply->rate_index == (s8) RATE_AUTO) {
		rate = RATE_AUTO;
	} else {
		index = reply->rate_index & 0x7f;
		if (WARN_ON_ONCE(index > (RATE_MCS_7_40 + 1)))
			return -EINVAL;

		sgi = (reply->rate_index & 0x80) ? 1 : 0;
		rate = wmi_rate_tbl[index][sgi];
	}

	ath6kl_wakeup_event(wmi->parent_dev);

	return 0;
}

static int ath6kl_wmi_test_rx(struct wmi *wmi, u8 *datap, int len)
{
	ath6kl_tm_rx_event(wmi->parent_dev, datap, len);

	return 0;
}

static int ath6kl_wmi_ratemask_reply_rx(struct wmi *wmi, u8 *datap, int len)
{
	if (len < sizeof(struct wmi_fix_rates_reply))
		return -EINVAL;

	ath6kl_wakeup_event(wmi->parent_dev);

	return 0;
}

static int ath6kl_wmi_ch_list_reply_rx(struct wmi *wmi, u8 *datap, int len)
{
	if (len < sizeof(struct wmi_channel_list_reply))
		return -EINVAL;

	ath6kl_wakeup_event(wmi->parent_dev);

	return 0;
}

static int ath6kl_wmi_tx_pwr_reply_rx(struct wmi *wmi, u8 *datap, int len)
{
	struct wmi_tx_pwr_reply *reply;

	if (len < sizeof(struct wmi_tx_pwr_reply))
		return -EINVAL;

	reply = (struct wmi_tx_pwr_reply *) datap;
	ath6kl_txpwr_rx_evt(wmi->parent_dev, reply->dbM);

	return 0;
}

static int ath6kl_wmi_keepalive_reply_rx(struct wmi *wmi, u8 *datap, int len)
{
	if (len < sizeof(struct wmi_get_keepalive_cmd))
		return -EINVAL;

	ath6kl_wakeup_event(wmi->parent_dev);

	return 0;
}

static int ath6kl_wmi_scan_complete_rx(struct wmi *wmi, u8 *datap, int len,
				       struct ath6kl_vif *vif)
{
	struct wmi_scan_complete_event *ev;

	ev = (struct wmi_scan_complete_event *) datap;

	ath6kl_scan_complete_evt(vif, a_sle32_to_cpu(ev->status));
	wmi->is_probe_ssid = false;

	return 0;
}

static int ath6kl_wmi_neighbor_report_event_rx(struct wmi *wmi, u8 *datap,
					       int len, struct ath6kl_vif *vif)
{
	struct wmi_neighbor_report_event *ev;
	u8 i;

	if (len < sizeof(*ev))
		return -EINVAL;
	ev = (struct wmi_neighbor_report_event *) datap;
	if (sizeof(*ev) + ev->num_neighbors * sizeof(struct wmi_neighbor_info)
	    > len) {
		ath6kl_dbg(ATH6KL_DBG_WMI,
			   "truncated neighbor event (num=%d len=%d)\n",
			   ev->num_neighbors, len);
		return -EINVAL;
	}
	for (i = 0; i < ev->num_neighbors; i++) {
		ath6kl_dbg(ATH6KL_DBG_WMI, "neighbor %d/%d - %pM 0x%x\n",
			   i + 1, ev->num_neighbors, ev->neighbor[i].bssid,
			   ev->neighbor[i].bss_flags);
		cfg80211_pmksa_candidate_notify(vif->ndev, i,
						ev->neighbor[i].bssid,
						!!(ev->neighbor[i].bss_flags &
						   WMI_PREAUTH_CAPABLE_BSS),
						GFP_ATOMIC);
	}

	return 0;
}

/*
 * Target is reporting a programming error.  This is for
 * developer aid only.  Target only checks a few common violations
 * and it is responsibility of host to do all error checking.
 * Behavior of target after wmi error event is undefined.
 * A reset is recommended.
 */
static int ath6kl_wmi_error_event_rx(struct wmi *wmi, u8 *datap, int len)
{
	const char *type = "unknown error";
	struct wmi_cmd_error_event *ev;
	ev = (struct wmi_cmd_error_event *) datap;

	switch (ev->err_code) {
	case INVALID_PARAM:
		type = "invalid parameter";
		break;
	case ILLEGAL_STATE:
		type = "invalid state";
		break;
	case INTERNAL_ERROR:
		type = "internal error";
		break;
	}

	ath6kl_dbg(ATH6KL_DBG_WMI, "programming error, cmd=%d %s\n",
		   ev->cmd_id, type);

	return 0;
}

static int ath6kl_wmi_stats_event_rx(struct wmi *wmi, u8 *datap, int len,
				     struct ath6kl_vif *vif)
{
	ath6kl_tgt_stats_event(vif, datap, len);

	return 0;
}

static u8 ath6kl_wmi_get_upper_threshold(s16 rssi,
					 struct sq_threshold_params *sq_thresh,
					 u32 size)
{
	u32 index;
	u8 threshold = (u8) sq_thresh->upper_threshold[size - 1];

	/* The list is already in sorted order. Get the next lower value */
	for (index = 0; index < size; index++) {
		if (rssi < sq_thresh->upper_threshold[index]) {
			threshold = (u8) sq_thresh->upper_threshold[index];
			break;
		}
	}

	return threshold;
}

static u8 ath6kl_wmi_get_lower_threshold(s16 rssi,
					 struct sq_threshold_params *sq_thresh,
					 u32 size)
{
	u32 index;
	u8 threshold = (u8) sq_thresh->lower_threshold[size - 1];

	/* The list is already in sorted order. Get the next lower value */
	for (index = 0; index < size; index++) {
		if (rssi > sq_thresh->lower_threshold[index]) {
			threshold = (u8) sq_thresh->lower_threshold[index];
			break;
		}
	}

	return threshold;
}

static int ath6kl_wmi_send_rssi_threshold_params(struct wmi *wmi,
			struct wmi_rssi_threshold_params_cmd *rssi_cmd)
{
	struct sk_buff *skb;
	struct wmi_rssi_threshold_params_cmd *cmd;

	skb = ath6kl_wmi_get_new_buf(sizeof(*cmd));
	if (!skb)
		return -ENOMEM;

	cmd = (struct wmi_rssi_threshold_params_cmd *) skb->data;
	memcpy(cmd, rssi_cmd, sizeof(struct wmi_rssi_threshold_params_cmd));

	return ath6kl_wmi_cmd_send(wmi, 0, skb, WMI_RSSI_THRESHOLD_PARAMS_CMDID,
				   NO_SYNC_WMIFLAG);
}

static int ath6kl_wmi_rssi_threshold_event_rx(struct wmi *wmi, u8 *datap,
					      int len)
{
	struct wmi_rssi_threshold_event *reply;
	struct wmi_rssi_threshold_params_cmd cmd;
	struct sq_threshold_params *sq_thresh;
	enum wmi_rssi_threshold_val new_threshold;
	u8 upper_rssi_threshold, lower_rssi_threshold;
	s16 rssi;
	int ret;

	if (len < sizeof(struct wmi_rssi_threshold_event))
		return -EINVAL;

	reply = (struct wmi_rssi_threshold_event *) datap;
	new_threshold = (enum wmi_rssi_threshold_val) reply->range;
	rssi = a_sle16_to_cpu(reply->rssi);

	sq_thresh = &wmi->sq_threshld[SIGNAL_QUALITY_METRICS_RSSI];

	/*
	 * Identify the threshold breached and communicate that to the app.
	 * After that install a new set of thresholds based on the signal
	 * quality reported by the target
	 */
	if (new_threshold) {
		/* Upper threshold breached */
		if (rssi < sq_thresh->upper_threshold[0]) {
			ath6kl_dbg(ATH6KL_DBG_WMI,
				   "spurious upper rssi threshold event: %d\n",
				   rssi);
		} else if ((rssi < sq_thresh->upper_threshold[1]) &&
			   (rssi >= sq_thresh->upper_threshold[0])) {
			new_threshold = WMI_RSSI_THRESHOLD1_ABOVE;
		} else if ((rssi < sq_thresh->upper_threshold[2]) &&
			   (rssi >= sq_thresh->upper_threshold[1])) {
			new_threshold = WMI_RSSI_THRESHOLD2_ABOVE;
		} else if ((rssi < sq_thresh->upper_threshold[3]) &&
			   (rssi >= sq_thresh->upper_threshold[2])) {
			new_threshold = WMI_RSSI_THRESHOLD3_ABOVE;
		} else if ((rssi < sq_thresh->upper_threshold[4]) &&
			   (rssi >= sq_thresh->upper_threshold[3])) {
			new_threshold = WMI_RSSI_THRESHOLD4_ABOVE;
		} else if ((rssi < sq_thresh->upper_threshold[5]) &&
			   (rssi >= sq_thresh->upper_threshold[4])) {
			new_threshold = WMI_RSSI_THRESHOLD5_ABOVE;
		} else if (rssi >= sq_thresh->upper_threshold[5]) {
			new_threshold = WMI_RSSI_THRESHOLD6_ABOVE;
		}
	} else {
		/* Lower threshold breached */
		if (rssi > sq_thresh->lower_threshold[0]) {
			ath6kl_dbg(ATH6KL_DBG_WMI,
				   "spurious lower rssi threshold event: %d %d\n",
				rssi, sq_thresh->lower_threshold[0]);
		} else if ((rssi > sq_thresh->lower_threshold[1]) &&
			   (rssi <= sq_thresh->lower_threshold[0])) {
			new_threshold = WMI_RSSI_THRESHOLD6_BELOW;
		} else if ((rssi > sq_thresh->lower_threshold[2]) &&
			   (rssi <= sq_thresh->lower_threshold[1])) {
			new_threshold = WMI_RSSI_THRESHOLD5_BELOW;
		} else if ((rssi > sq_thresh->lower_threshold[3]) &&
			   (rssi <= sq_thresh->lower_threshold[2])) {
			new_threshold = WMI_RSSI_THRESHOLD4_BELOW;
		} else if ((rssi > sq_thresh->lower_threshold[4]) &&
			   (rssi <= sq_thresh->lower_threshold[3])) {
			new_threshold = WMI_RSSI_THRESHOLD3_BELOW;
		} else if ((rssi > sq_thresh->lower_threshold[5]) &&
			   (rssi <= sq_thresh->lower_threshold[4])) {
			new_threshold = WMI_RSSI_THRESHOLD2_BELOW;
		} else if (rssi <= sq_thresh->lower_threshold[5]) {
			new_threshold = WMI_RSSI_THRESHOLD1_BELOW;
		}
	}

	/* Calculate and install the next set of thresholds */
	lower_rssi_threshold = ath6kl_wmi_get_lower_threshold(rssi, sq_thresh,
				       sq_thresh->lower_threshold_valid_count);
	upper_rssi_threshold = ath6kl_wmi_get_upper_threshold(rssi, sq_thresh,
				       sq_thresh->upper_threshold_valid_count);

	/* Issue a wmi command to install the thresholds */
	cmd.thresh_above1_val = a_cpu_to_sle16(upper_rssi_threshold);
	cmd.thresh_below1_val = a_cpu_to_sle16(lower_rssi_threshold);
	cmd.weight = sq_thresh->weight;
	cmd.poll_time = cpu_to_le32(sq_thresh->polling_interval);

	ret = ath6kl_wmi_send_rssi_threshold_params(wmi, &cmd);
	if (ret) {
		ath6kl_err("unable to configure rssi thresholds\n");
		return -EIO;
	}

	return 0;
}

static int ath6kl_wmi_cac_event_rx(struct wmi *wmi, u8 *datap, int len,
				   struct ath6kl_vif *vif)
{
	struct wmi_cac_event *reply;
	struct ieee80211_tspec_ie *ts;
	u16 active_tsids, tsinfo;
	u8 tsid, index;
	u8 ts_id;

	if (len < sizeof(struct wmi_cac_event))
		return -EINVAL;

	reply = (struct wmi_cac_event *) datap;
	if (reply->ac >= WMM_NUM_AC) {
		ath6kl_err("invalid AC: %d\n", reply->ac);
		return -EINVAL;
	}

	if ((reply->cac_indication == CAC_INDICATION_ADMISSION_RESP) &&
	    (reply->status_code != IEEE80211_TSPEC_STATUS_ADMISS_ACCEPTED)) {
		ts = (struct ieee80211_tspec_ie *) &(reply->tspec_suggestion);
		tsinfo = le16_to_cpu(ts->tsinfo);
		tsid = (tsinfo >> IEEE80211_WMM_IE_TSPEC_TID_SHIFT) &
			IEEE80211_WMM_IE_TSPEC_TID_MASK;

		ath6kl_wmi_delete_pstream_cmd(wmi, vif->fw_vif_idx,
					      reply->ac, tsid);
	} else if (reply->cac_indication == CAC_INDICATION_NO_RESP) {
		/*
		 * Following assumes that there is only one outstanding
		 * ADDTS request when this event is received
		 */
		spin_lock_bh(&wmi->lock);
		active_tsids = wmi->stream_exist_for_ac[reply->ac];
		spin_unlock_bh(&wmi->lock);

		for (index = 0; index < sizeof(active_tsids) * 8; index++) {
			if ((active_tsids >> index) & 1)
				break;
		}
		if (index < (sizeof(active_tsids) * 8))
			ath6kl_wmi_delete_pstream_cmd(wmi, vif->fw_vif_idx,
						      reply->ac, index);
	}

	/*
	 * Clear active tsids and Add missing handling
	 * for delete qos stream from AP
	 */
	else if (reply->cac_indication == CAC_INDICATION_DELETE) {
		ts = (struct ieee80211_tspec_ie *) &(reply->tspec_suggestion);
		tsinfo = le16_to_cpu(ts->tsinfo);
		ts_id = ((tsinfo >> IEEE80211_WMM_IE_TSPEC_TID_SHIFT) &
			 IEEE80211_WMM_IE_TSPEC_TID_MASK);

		spin_lock_bh(&wmi->lock);
		wmi->stream_exist_for_ac[reply->ac] &= ~(1 << ts_id);
		active_tsids = wmi->stream_exist_for_ac[reply->ac];
		spin_unlock_bh(&wmi->lock);

		/* Indicate stream inactivity to driver layer only if all tsids
		 * within this AC are deleted.
		 */
		if (!active_tsids) {
			ath6kl_indicate_tx_activity(wmi->parent_dev, reply->ac,
						    false);
			wmi->fat_pipe_exist &= ~(1 << reply->ac);
		}
	}

	return 0;
}

static int ath6kl_wmi_txe_notify_event_rx(struct wmi *wmi, u8 *datap, int len,
					  struct ath6kl_vif *vif)
{
	struct wmi_txe_notify_event *ev;
	u32 rate, pkts;

	if (len < sizeof(*ev))
		return -EINVAL;

	if (vif->nw_type != INFRA_NETWORK ||
	    !test_bit(ATH6KL_FW_CAPABILITY_TX_ERR_NOTIFY,
		      vif->ar->fw_capabilities))
		return -EOPNOTSUPP;

	if (vif->sme_state != SME_CONNECTED)
		return -ENOTCONN;

	ev = (struct wmi_txe_notify_event *) datap;
	rate = le32_to_cpu(ev->rate);
	pkts = le32_to_cpu(ev->pkts);

	ath6kl_dbg(ATH6KL_DBG_WMI, "TXE notify event: peer %pM rate %d%% pkts %d intvl %ds\n",
		   vif->bssid, rate, pkts, vif->txe_intvl);

	cfg80211_cqm_txe_notify(vif->ndev, vif->bssid, pkts,
				rate, vif->txe_intvl, GFP_KERNEL);

	return 0;
}

int ath6kl_wmi_set_txe_notify(struct wmi *wmi, u8 idx,
			      u32 rate, u32 pkts, u32 intvl)
{
	struct sk_buff *skb;
	struct wmi_txe_notify_cmd *cmd;

	skb = ath6kl_wmi_get_new_buf(sizeof(*cmd));
	if (!skb)
		return -ENOMEM;

	cmd = (struct wmi_txe_notify_cmd *) skb->data;
	cmd->rate = cpu_to_le32(rate);
	cmd->pkts = cpu_to_le32(pkts);
	cmd->intvl = cpu_to_le32(intvl);

	return ath6kl_wmi_cmd_send(wmi, idx, skb, WMI_SET_TXE_NOTIFY_CMDID,
				   NO_SYNC_WMIFLAG);
}

int ath6kl_wmi_set_rssi_filter_cmd(struct wmi *wmi, u8 if_idx, s8 rssi)
{
	struct sk_buff *skb;
	struct wmi_set_rssi_filter_cmd *cmd;
	int ret;

	skb = ath6kl_wmi_get_new_buf(sizeof(*cmd));
	if (!skb)
		return -ENOMEM;

	cmd = (struct wmi_set_rssi_filter_cmd *) skb->data;
	cmd->rssi = rssi;

	ret = ath6kl_wmi_cmd_send(wmi, if_idx, skb, WMI_SET_RSSI_FILTER_CMDID,
				  NO_SYNC_WMIFLAG);
	return ret;
}

static int ath6kl_wmi_send_snr_threshold_params(struct wmi *wmi,
			struct wmi_snr_threshold_params_cmd *snr_cmd)
{
	struct sk_buff *skb;
	struct wmi_snr_threshold_params_cmd *cmd;

	skb = ath6kl_wmi_get_new_buf(sizeof(*cmd));
	if (!skb)
		return -ENOMEM;

	cmd = (struct wmi_snr_threshold_params_cmd *) skb->data;
	memcpy(cmd, snr_cmd, sizeof(struct wmi_snr_threshold_params_cmd));

	return ath6kl_wmi_cmd_send(wmi, 0, skb, WMI_SNR_THRESHOLD_PARAMS_CMDID,
				   NO_SYNC_WMIFLAG);
}

static int ath6kl_wmi_snr_threshold_event_rx(struct wmi *wmi, u8 *datap,
					     int len)
{
	struct wmi_snr_threshold_event *reply;
	struct sq_threshold_params *sq_thresh;
	struct wmi_snr_threshold_params_cmd cmd;
	enum wmi_snr_threshold_val new_threshold;
	u8 upper_snr_threshold, lower_snr_threshold;
	s16 snr;
	int ret;

	if (len < sizeof(struct wmi_snr_threshold_event))
		return -EINVAL;

	reply = (struct wmi_snr_threshold_event *) datap;

	new_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->snr;

	sq_thresh = &wmi->sq_threshld[SIGNAL_QUALITY_METRICS_SNR];

	/*
	 * Identify the threshold breached and communicate that to the app.
	 * After that install a new set of thresholds based on the signal
	 * quality reported by the target.
	 */
	if (new_threshold) {
		/* Upper threshold breached */
		if (snr < sq_thresh->upper_threshold[0]) {
			ath6kl_dbg(ATH6KL_DBG_WMI,
				   "spurious upper snr threshold event: %d\n",
				   snr);
		} else if ((snr < sq_thresh->upper_threshold[1]) &&
			   (snr >= sq_thresh->upper_threshold[0])) {
			new_threshold = WMI_SNR_THRESHOLD1_ABOVE;
		} else if ((snr < sq_thresh->upper_threshold[2]) &&
			   (snr >= sq_thresh->upper_threshold[1])) {
			new_threshold = WMI_SNR_THRESHOLD2_ABOVE;
		} else if ((snr < sq_thresh->upper_threshold[3]) &&
			   (snr >= sq_thresh->upper_threshold[2])) {
			new_threshold = WMI_SNR_THRESHOLD3_ABOVE;
		} else if (snr >= sq_thresh->upper_threshold[3]) {
			new_threshold = WMI_SNR_THRESHOLD4_ABOVE;
		}
	} else {
		/* Lower threshold breached */
		if (snr > sq_thresh->lower_threshold[0]) {
			ath6kl_dbg(ATH6KL_DBG_WMI,
				   "spurious lower snr threshold event: %d\n",
				   sq_thresh->lower_threshold[0]);
		} else if ((snr > sq_thresh->lower_threshold[1]) &&
			   (snr <= sq_thresh->lower_threshold[0])) {
			new_threshold = WMI_SNR_THRESHOLD4_BELOW;
		} else if ((snr > sq_thresh->lower_threshold[2]) &&
			   (snr <= sq_thresh->lower_threshold[1])) {
			new_threshold = WMI_SNR_THRESHOLD3_BELOW;
		} else if ((snr > sq_thresh->lower_threshold[3]) &&
			   (snr <= sq_thresh->lower_threshold[2])) {
			new_threshold = WMI_SNR_THRESHOLD2_BELOW;
		} else if (snr <= sq_thresh->lower_threshold[3]) {
			new_threshold = WMI_SNR_THRESHOLD1_BELOW;
		}
	}

	/* Calculate and install the next set of thresholds */
	lower_snr_threshold = ath6kl_wmi_get_lower_threshold(snr, sq_thresh,
				       sq_thresh->lower_threshold_valid_count);
	upper_snr_threshold = ath6kl_wmi_get_upper_threshold(snr, sq_thresh,
				       sq_thresh->upper_threshold_valid_count);

	/* Issue a wmi command to install the thresholds */
	cmd.thresh_above1_val = upper_snr_threshold;
	cmd.thresh_below1_val = lower_snr_threshold;
	cmd.weight = sq_thresh->weight;
	cmd.poll_time = cpu_to_le32(sq_thresh->polling_interval);

	ath6kl_dbg(ATH6KL_DBG_WMI,
		   "snr: %d, threshold: %d, lower: %d, upper: %d\n",
		   snr, new_threshold,
		   lower_snr_threshold, upper_snr_threshold);

	ret = ath6kl_wmi_send_snr_threshold_params(wmi, &cmd);
	if (ret) {
		ath6kl_err("unable to configure snr threshold\n");
		return -EIO;
	}

	return 0;
}

static int ath6kl_wmi_aplist_event_rx(struct wmi *wmi, u8 *datap, int len)
{
	u16 ap_info_entry_size;
	struct wmi_aplist_event *ev = (struct wmi_aplist_event *) datap;
	struct wmi_ap_info_v1 *ap_info_v1;
	u8 index;

	if (len < sizeof(struct wmi_aplist_event) ||
	    ev->ap_list_ver != APLIST_VER1)
		return -EINVAL;

	ap_info_entry_size = sizeof(struct wmi_ap_info_v1);
	ap_info_v1 = (struct wmi_ap_info_v1 *) ev->ap_list;

	ath6kl_dbg(ATH6KL_DBG_WMI,
		   "number of APs in aplist event: %d\n", ev->num_ap);

	if (len < (int) (sizeof(struct wmi_aplist_event) +
			 (ev->num_ap - 1) * ap_info_entry_size))
		return -EINVAL;

	/* AP list version 1 contents */
	for (index = 0; index <  0; index E4 <  0; ien this event is received
		 */
		spinolds\n");
		return -EIO;
	}

	return 0;
}

static int ath6kl_wmi_cac_event_rx(struct wmi *wmi, u8 *datap, int len,
				   struct ath6kl_vif *vif)
{
	struct wmi_cac_event *reply;
	struct ieee802N0_Gv,oiEie *ts;
	u16 active_tsids, tsinfo;
	u8 tsid, index;
	u8 ts_id;

	if (len < sizeof(struct wmi_cac_event))
		return -EINVAL;

	reply = (struct wmi_cac_event *) datap;
	if (reply->ac >= WMM_NUM_ACN    struct ath6kl_vif *vif)
{
	S;
	u8 ts_id;

	if (lenrn -EINVAL;
	}

	if ((reply->cac_indication == CAC_INDICATION_ADMISSION_RESP) &&
	    (reply->status_code != IEEE80211_TSPEC_STATUS_ADMISS_ACCEPTED)) {
		ts = (sN0Rx	    (re!G!_GbyCode)
			rG!_Gbh_NUM_]d_params *sq_thresh;
	enum wmi_rssi_threshold_val x!_G!_Gw	}
		if (index < (sizeof(active_tsids) * 8!_Gvu) &
			IEEE80211_WMM_IE_TSPEC_TID_MASK;

		ath6kl_wmi_delete_pstream_c * Identify thx!_G!_G!_Gf8ps;
	enu?USG!_GwNint lGf_GwNint oEity(wmi->parent_dev, reply->ac,
						    false);
			wmi->fat_pipe_exist &= ~(1 << reply->ac);
		}
	}

	return 0;
}

static int ath6kl_wmi_txe_notify_event_rx(struct wmi *wmi, u8 *datap, int ledex E4 <  0; )otify_evid = ((tsinfo >> IfMSGv U1 = (struct wmi_ap_info_v1 *) ev->ap_list;

	ath6kl_dbg(ATH6KL_DBG_WMI,
		   "number of APs in aplSG
"nu	 (ev->num_ap - 1) * ap_info_entry_size))
		return -EINVAL;

	/* AP list version 1 contents */
	foMASK;

		ac!Gfck_bh(&wmi->lock);

		/* Indicate stream if	/* AP list version 1 c[USGgnten_Gw*if  ath6kl_dbg(ATH6KL_DBG_WMI,
		   "number of A * for delete qos stream from AP
	 */
	else if (reply->cac_indication == CAC_INDICATION_DELETE) {
Nf	/* AP list version 1 c[US?USG!_G!_G!_Gxh0GvrW;
		}f *vif)
{
	struct wmi_cac_event *reply;
	struct ieee802N0_Gv,oiEie *ts;
	u16 active_tsids, tsinfo;
	u8 tsid, index;
	u8 ts_id;

	if (len < sizeof(struct wmi_cac_SG!_G!_GxcsUshold_val x!_G!_Gw	}
		if (index < (sizeof(active_tsids) * 8!_Gvu) &
			IEEE80211_WMM_IE_TSPEC_TIDit(ATH6KL_FW_CAPABILITY_TX_ERRvazeof(>uppxy

	h->lower_thresy	ath6kl_df LITY_TX_sh->li	TH>lower_th   false);
			MakCN _ACN 0; len);currsiolysnreuighis? 1 nsmitsh->letsie);
			indeABOVexecu	   .  Es(ar-ish(snr > s_df ete qotify_cmd ent))
		retu_df ete qbreached andreturn -d = (sshruct roam_ for delete qosnumberWMI_SNMI,
		  t lower value */
	h_eventAL_QUALITY_METR	h_esi_cmd)
 n_intvl);
	retcmd->in
	elsac_woulNf	/* &n -EICMD_HDR_IF_layer onlMETR	h_esilsac_wouintvl);
	retlsac_version cfg80tsinOPholdICMD,xist_BE delete q.1])) {
		_cmd)
 nrn -EIOPholdIFRAskb,[2]) ev->assc_event))
		retALIT	h_e_FTYbreachuct rOPhoMSG) {

		if (!activ= cpu,v= cpu,v0_GxcsUshed andretu_event *) dataEEE80211_WMM_IE_TSPEC_T set of thresL_FW_Ctream frent))
	ac2 delete qos >pkts = cpu_to_le3txe_Apxy
);untry_code_to_entroor[i>pkts = cpu_to_le3uct rtreamo_reason_sl_df LITY_TX_sh->lAFe aplower_thresy	ath6kl_df LITY_TX_sh->li	TH>lower_th   false);
			MakCN _ACN 0; r > len);nreuighwaits wmi_che*/
	for (in);
			execu	e. Es(ar-ish(snr > s_df ete qotify_cmd ent))
		retu_df ete qbreached andreturn -	/* AP list version 1 content_rx(str WMI_SET_ROAM_CTRL_CMDID,
				   NO
		iA * network(sizeouf(size   NO
		iA * dot11_pu(e.apsd_dot11_pu(e.apsd   NO
		iA * pu(e.apsd_pu(e.apsd   NO
		iA * crypl);sizeo	ret ina_crypl)   NO
		DID	ret ina_crypl)Y_ERD_FLAG) iA * crypl);sizeogroup_crypl)   NO
		DIDgroup_crypl)if *viersip - if *viDICApper_thresho_SET_DTIMviDFIX)urn -EIl_wmi->u.ad[siz_thresho_SEuf(sub	strued */
		if (snr < sq_thresh->upper_tht_rx(str WM  "c	}
	} else {
	h6kl_tkip_micerr_event(vif, ev->key_i_mcast);void ath6k	return d[sizeif, ip - if *urn ned_le16d *
ath_dot11_pu(en_timu(en_ti	ret inan_tigroups, tsinfo;
	u_DTIMvi)urn -EIl->u.ad[siz_ip - if *viuf(size   N
	udot11_pu(e.apsd _pu(e.apsd o	ret ina_crypl) Dgroup_crypl)	if (len < sizeof(struct wmi_cac_S_GxcsUshooid a"_ip - _ip - if *phy, 0);
}icerr_event *ev;

	if (len (	ret ina_crypl)x%0x\nNkb,RYPts,
& (group_crypl)zeof\nNkb,RYPtst;

	skb = ath6kl_wmi_get_(	ret ina_crypl)x!0x\nNkb,RYPts,
& (group_crypl)z=of\nNkb,RYPtst;

	skb = ath6kl_wmi_L_DBG_WMI,
				   "spurious upper snh->upper_tht_rx(str WMreshold event: %d\n",
				   snr);
	c t lower value *_rx(str WM  .cipher),
				 of(ac - if *p %d  (snr >c->p - _ip - _ip - if *phy, >c->p - if *u=ip - if *;, >c->ta.bssid,iuf(size;, >c->dot11_pu(e.apsd_=udot11_pu(e.apsd;, >c->pu(e.apsd_=upu(e.apsd;, >c->pt ina_crypl)Ybssid,i	ret ina_crypl);, >c->pt ina_crypl)Yf *u=i	ret ina_crypl)Y_ER;, >c->grp_crypl)Ybssid,igroup_crypl);, >c->grp_crypl)Yf *u=igroup_crypl)if *;, >c->ch n_intvl);
	retctry_size))>c->c>u.ad[sizmi, u8 *datap, c>u.ad[siz);, >c->ta.sub	strd,iuf(sub	str>sched_scoid a!n this AC.  (snr >c->f(*cmd));
	if (!skb)
		re	 * Identify the threshold breached and communicat, WMI_Spp.
	 * After that install a neww set of threshol&
			   (rssi >=et_rx(str WMI_SET_ROAM_CTRL_CMDID,
				 o_SET_DTIMv_params_cDFIX)urn -Eold) {
		/* Upper threshold breached */et_rx(str WM  "c	}
	} else {
	h6kl_tkip_micerr_event(vif>key_i/et_rx(st;void ath6k	returntsinfo;
	u_DTIMvi)urn -Ephy, 0);
}icerr_event *ev;

	if (L_DBG_WMI,
				   "spurious upper snh->upper_th/et_rx(str WMreshold event: %d\n",
				   snr);
	c t lower value /et_rx(str WM  ntAL_QUALITY_MEc_entry_si n_intvl);
	retctry_size)ched_scoid a!n this AC.  (snr >c->f(*cmd));
	if (!skb)
		re	 * Identify the threshold breached and communicatRE, WMI_Spp.
	 * After that install a neww set of threshol&
			   (rssi >vent_rx(str WMI_SET_ROAM_CTRL_CMDID,
				ed: %c%c\lse {
	h6kl_tkip_micerr_event(vif>key_ivent_rx(sth6kl_v, 0);
}icerr_event *ev;

	if (on Dis_mcast);/
	for (m. Jyer ftream_ex

	r_sh->>letsie.1])) cac_event))
		retuct we -EOPNOTSUed and cc_SG!IS, WMI_Spp.
	 neww set of thresholon _rssi_thresholrtundefinmghis? o>le dep/etmd=%. Useupperly_rx(struceginundefinmghinstea%. T);
	aw ie[4

	atsuphress P2Puppecpu( >uppresh->xisy_evenrn oream) *faces *sq_thresh,
					 u32 sizeholrtndefinmgI_SET_ROAM_CTRL_CMDID,
				   NO_SYNC 1 c[USGgn *datstrdn *datstr   NO_SYNC_wmitsice_fgn *dkl_wmi_s;
	gacy   NO_SYNC_wmihome_dwe wmi *w   NO_SYNC_wmitsice_n *dauct ath6   NO_SYNCsSEuum(struviDFIX*ct wmi_hed */
		if (snr < sq_thresh->upper_throlrtundefinmgh*sc	}
sSEstatus_c%c\n, datap;

SPEC_TID_MASK;

		ath6klrolrtundefinmgo_reason_sl *datstrd!rn -EILONG	s32 s,
& (l *datstrd!rn -EISHORT	s32 st;

	skb = ath6kl_wmi_get_uum(stru >n -EIMAdICHAWMILSt;

	skb = ath6kl_wmi_get_uum(strut;


SPEC+_TID_MASKdmn == _uum(stru G!_Gif (L_DBG_WMI,
				   "spurious uppereshold event: %d\n",
				   snr);
sc t lower value rolrtundefinmgh*ntAL_QUALITY_Msc->p *datstrd=dn *datstrY_Msc->tsice_fgundefmi, u8 *datap, tsice_fgn *d)Y_Msc->_s;
	gacyt wmi_snr_threshs;
	gacy)Y_Msc->home_dwe wmi *wt wmi_snr_threshome_dwe wmi *w)Y_Msc->tsice_n *dauctuct wmi_snr_threstsice_n *dauct ath6)Y_Msc->uum(std,iuum(strur);
ct wmi *wmi, u8 uum(strur			  ;


c_ent wmi_[i] n_intvl);
	retct wmi_[i]	re	 * Identify the threshold breached and communicatSTART	s32 pp.
	 * After that install a neww set of threshol*/
	foceginndefmsuphress (ned.aream_exstruholrtndef) P2Pecpu( >uppresh->xisy_e
	foenrn oream) *face cold ;

	in or
	at_timern orelikCN _phresh-> *vif(in)pperdvertinanor (xmit> *vif(ct wi_err
static s *sq_r_reply_rx(struceginndefr WMI_SET_ROAM_CTRL_CMDID,
				   NO
		NC 1 c[USGgn *datstrdn *datstr   NO SYNC_wmitsice_fgn *dkl_wmi_s;
	gacy   NO SYNC_wmihome_dwe wmi *w C_wmitsice_n *dauct ath6   NO SYNCsSEuum(struviDFIX*ct wmi_ C_wmino_cck C_wmi* *vifhed */
		if  peer %pM  _phresh-_bor (*sbor ; */
		if (snr < sq_thresh->upper_thceginundefinmgh*sc	}
sSEstat,h*s_phf *vifus_c%c\n, bor , datap 8 - 2 - 2);
		if (tim && tim[1] >= 2) c%c\= (st*vifus_d));

	rbor_wmi_get_turn -ENOMEM;

	cmd = (struct wSTA_P2PDEV_DUPLEXilter_cmd b->data;
	cmd->rssi =ev->asscHRESHOLD1_ABOVEholrtndefinmgIreached and 		if (	n *datstr itsice_fgn *dk		if (	_s;
	gacy ihome_dwe wmi *w   NO_, pktce_n *dauct ath6   NO_		uum(struvict wmi_heturn -dSPEC_TID_MASK;

		ath6klceginundefinmgo_reason_sl *datstrd!rn -EILONG	s32 s,
& (l *datstrd!rn -EISHORT	s32 st;

	skb = ath6kl_wmi_get_uum(stru >n -EIMAdICHAWMILSt;

	skb = ath6kl_wmi_get_uum(strut;


SPEC+_TID_MASKdmn == _uum(stru G!_Gif (L_DBG_WMI,
				   "spurious uppereshold event: %d\n",
				   snr);
sc t lower value ceginundefinmgh*ntAL_QUALITY_Msc->p *datstrd=dn *datstrY_Msc->tsice_fgundefmi, u8 *datap, tsice_fgn *d)Y_Msc->_s;
	gacyt wmi_snr_threshs;
	gacy)Y_Msc->home_dwe wmi *wt wmi_snr_threshome_dwe wmi *w)Y_Msc->tsice_n *dauctuct wmi_snr_threstsice_n *dauct ath6)Y_Msc->uo_cckt wmi_snr_thresuo_cck)Y_Msc->uum(std,iuum(strur);
ct wmbor (*wmi,bor (< ct iNL WMM_NUANDSi,bor _v1 *) esbor (*wy
	 * for->for s[for   ev->ld evefor ar->f_entinue ev->ld eon == CAbor (>=>bssid,
ct iUANDS)6kl_wmi_regdoma

	rbor_esho*vif[for   e es_phf *vifd=dn ->p_phf *vif[for  .t*vifus_	= (st*vif(*wmidomact wmi *wmi, u8 efor ->uuct wmi 
				     stason_sBIT(i   !

	rbor_	return  NO__entinue M;
	skip_buff gdmn)
p_phf *vif[= (st*vif++] n  NO SYN	/*
	(efor ->ct wmi 
threst wmi  / 5(wmi, id	n ->p_phf *vif[for  .nt*vif(*w= (st*vifus_});
ct wmi *wmi, u8 uum(strur			  ;


c_ent wmi_[i] n_intvl);
	retct wmi_[i]	re	 * Identify the threshold breached and communicatBEGIN	s32 pp.
	 * After that install a neww set of threshol
					 u32 size)->ac -EINVAL;

	i WMI_SET_ROAM_CTRL_CMDID,
				 obool )->ac hed */
		if (snr < sq_thresh->upper_th)->ac -EINVAL;

	i WMh*sc	}
 *) datap;

	new_threshold = (enum wmi_snr_thresscreshold event: %d\n",
				   snr);
h6kl_tkip_micerr_event(vif>k%s
	 */
	spin_lock) sq_furntsinfo;
	u)->ac  ? ")->acy_e" : "ven>acy_e"ched andretusc t lower value )->ac -EINVAL;

	i WMh*ntAL_QUALITY_Msc->)->ac  =u)->ac  ? len,
			 * Identify the threshold breached and commu After icatENreadyply;
	s32 rp.
	 * After that install a new set of thresholing error.  This isSHOLD3_ABOI_SET_ROAM_CTRL_CMDID,
				   NO
		NCiDFIXfgunolrtunec   NO
		NCiDFIXfgurn -Eec iDFIXbgunec   NO
		NCiDFIXmew_bu(stdw_mEec iDFIXmax_bu(stdw_mEec   NO
		NCiDFIXpas(stdw_mEec iD86kl_inL;

	ifrn o   NO
		NCiD8dn *dac>u.ad[si C_wmimax_dfEINe);
mi *w   NO
		NCiDFIXmax_bu(n *dan < s		ated */
		if (snr < sq_thresh->upper_thr *daner thresholdc	}
 *) datap;

	new_threshold = (enum wmi_snr_thresscreshold event: %d\n",
				   snr);
sc t lower value r *daner thresholntAL_QUALITY_Msc->fgunolrtunt len)
{intvl);
	retfgunolrtunec)Y_Msc->fgurn -nt len)
{intvl);
	retfgurn -Eec)Y_Msc->bgunt len)
{intvl);
	retbgunec)Y_Msc->mew_bu(stdwe wmi *wt wmi_snr_thretmew_bu(stdw_mEec)Y_Msc->max_bu(stdwe wmi *wt wmi_snr_thretmax_bu(stdw_mEec)Y_Msc->pas(stdwe wmi *wt wmi_snr_thretpas(stdw_mEec)Y_Msc->kl_inL;

	ifrn od=dnl_inL;

	ifrn oY_Msc->p *dac>u.ad[sizmi,p *dac>u.ad[siY_Msc->max_dfEINe);
mi *wt wmi_snr_thresmax_dfEINe);
mi *w)Y_Msc->max_bu(n *dan < s		at wmi_snr_thretmax_bu(n *dan < s		at*
	 * Identify the threshold breached and communicate ths32 rhreshold[2]) &&
			 that install a new set of thresholing error.  ThiThe timer is s_SET_ROAM_CTRL_CMDID,
				 o_SE timerkl_wmi_e_bor_	ed */
		if (snr < sq_thresh->upper_thsholdr_threshold_event *) datap;
ld edr_thr(>=>LASTved, otherwt;

	skb = ath6kl_wmi_L_DBG_WMI,
				   "spurious upper snr threshold event: %d\n",
				   snr);
		} else if ((snr sholdr_thresholdGNAL_QUALITY_METRICsholdr_thr eldr_thrY_METRIC_e_bor_t wmi_snr_threshe_bor_	*
	 * Identify the threshold breached and communicate thed, otherwld[2]) &&
			 that install a new set of thresholing error.  Thii_errdp - iis s_SET_ROAM_CTRL_CMDID,
				 o_SEplSG
"n_SE [si   NO
		NCiD8dn - if *viDICApper	ed */
		if (snr < sq_thresh->upper_thi_errd_p - iis ld_event *) datap;
ld e

	retu= MAdIbss =
	sSIDSt;

	skb = ath6kl_wmi_get_p - if *u>TID_MASKETRICpper	t;

	skb = ath6kl_wmi_get_(LITY_& (DISreadypSID_l a  | ANYypSID_l a )s,
& (l - if *u>T0	t;

	skb = ath6kl_wmi_get_(LITY_& l, GIFICypSID_l a ),
& !c - if *p %d	skb = ath6kl_wmi_get_LITY_& l, GIFICypSID_l a )
	ath6kl_wmi_error_event->uewmi_L_DBG_WMI,
				   "spurious upper snr threshold event: %d\n",
				   snr);
		} else if ((snr i_errd_p - iis ldGNAL_QUALITY_METRIC_rx(sto >> IfMtsids anETRICLITY_T d[siY_METRICpperif *u=ip - if *;,   (snr >= ->p - _ip - _ip - if *phy, * Identify the threshold breached and communicate thbss =
	sSIDld[2]) &&
			 that install a new set of thresholing error.  Thiwmi_enuct ath6inmgI_SET_ROAM_CTRL_CMDID,
				   NO_SYDFIXwmi_enauct ath6   NO_SYDFIXwmi_enabeahres	ed */
		if (snr < sq_thresh->upper_thwmi_enaucteshold_event *) datap;

	new_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =wmi_enauctesholdGNAL_QUALITY_METRICwmi_enauctvi n_intvl);
	retwmi_enauct ath6thresh;
	uum(beahres n_intvl);
	retwmi_enabeahres	hy, * Identify the threshold breached and communicate thctivEN_INSpp.
	 * After that install a new set of thresholing error.  ThiTotifi *w_ABOI_SET_ROAM_CTRL_CMDID,
				   NO
		NCDFIXbotifmi *w C_FIXuum(beahres	ed */
		if (snr < sq_thresh->upper_thsotifmi *weshold_event *) datap;

	new_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =sotifmi *wesholdGNAL_QUALITY_METRICsotifmi *w)
{intvl);
	retbotifmi *wthresh;
	uum(beahres n_intvl);
	retuum(beahres	*
	 * Identify the threshold breached and communicate theck_bhTIskb,[2]) &&
			 that install a new set of thresholing error.  Thiix;
	apsdiis s_SET_ROAM_CTRL_CMDID,
				 o_SE].bsapsd	ed */
		if (snr < sq_thresh->upper_thix;
	uapsdiis ld_event *) datap;

	new_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =ix;
	uapsdiis ldGNAL_QUALITY_METRIC].bsapsdu=i	.bsapsd;, 0);
}].bsapsdu=i	.bsapsd;,, * Identify the threshold breached and communicate thbOWrwlMODkb,[2]) &&
			 that install a new set of thresholing error.  ThiimSHOLD3_ABOI_SET_ROAM_CTRL_CMDID,
				 C_FIXidleunt len   NO
		NDFIXpwmit wmuum C_FIX    inolicy   NO SYN_FIXtx	}

	retnolicy C_FIXuum(tx	l);}

	re   NO
		NDFIXpwmfaily->cac_nolicy	ed */
		if (snr < sq_thresh->upper_thix;
	uner thresholpment *) datap;

	new_threshold = (enum wmi_snr_threspmd_val) reply->range;
	snr = reply->pm;

	sq_thresh =ix;
	uner thresholnAL_QUALITY_Mpmkl_dleunt lenwouintvl);
	retldleunt len)Y_Mpmklpsit wmuumbhr elmi_snr_thretpwmit wmuum)Y_Mpmkl    inolicy elmi_snr_thret    inolicy)Y_Mpmkltx	}

	retnolicy elmi_snr_threttx	}

	retnolicy)Y_Mpmkluum(tx	l);}

	re n_intvl);
	retuum(tx	l);}

	re)Y_Mpmklpsmfaily->cac_nolicy elmi_snr_thretpwmfaily->cac_nolicy	;,, * Identify the threshold breached and communicate thbOWrwlhreshold[2]) &&
			 that install a new set of thresholing error.  Thiventindex > is s_SET_ROAM_CTRL_CMDID,
				 o_SEindex >	ed */
		if (snr < sq_thresh->upper_thvent_index > is ld_event *) datap;

	new_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =vent_index > is ldGNAL_QUALITY_METRICvent_r_index > =Eindex >;,, * Identify the threshold breached and communicate th!IS,hTIskOUSpp.
	 * After that install a neww event *return  Nh6kl_tkiebug= &wmvent_rx(strindex >>pkts = cpu_to_le3index >	eww set of threshol
					 u32 sizeaddkey is s_SET_ROAM_CTRL_CMDID,
				 o_SEkey plSG
"  NO
	iA * crypl);sizeokey tstr   NO S_SEkey usage o_SEkey _ERD_FLAG)DICAkey rsc iDn&&
	ect wm key rsc _ERD_FLAG)DICAkey mat aih6   NO S_SEkey opac>u.viDICAmaceaddr"  NO
	iA * wSGgnten_Gw*if  ath6kl_dbg(ATH6KL_D(snr < sq_thresh->upper_thadd_ciph
	ukey is ld_event *) datap;
h6kl_tkip_micerr_event(vif, ev->kaddkeyeABO:Ekey plSG
=%uokey tstrold_key usageold_key , typeEkey opac>u.=rntsinfo;
	ukey plSG
"okey tstr Ekey usage okey _ERDEkey opac>u.o_reason_skey plSG
 >n -EIMAdIKEYbg(AEXhres (key _ER >n -EIMAdIKEYb)
		resy	ath6kkey mat aih6NDICATIOhres key rsc _ER >n8t;

	skb = ath6kl_wmi_get_(WEPb,RYPtd!rnkey tstrs,
& (ATION== key rscst;

	skb = ath6kl_wmi_L_DBG_WMI,
				   "spurious upper snshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =add_ciph
	ukey is ldGNAL_QUALITY_METRICkey plSG
 =ukey plSG
Y_METRICkey tstrd=dkey tstrY_METRICkey usaged=dkey usageY_METRICkey f *u=ikey f *;,   (snr >= ->keyDEkey mat aih6 ikey f *o_reason_key rsca!n this AC.  (snr >TRICkey rsc ikey rsc ikey rscif *phy, >TRICkey opac>u.u=ikey opac>u._reason_maceaddr AC.  (snr >TRICkey maceaddr" maceaddr" (!skb)
		re	 * Identify the threshold breached and communicatADDldIPHrwlKEYbp.
	 * After  ts_id;

	ifw set of threshol
					 u32 sizeadd_krk is s_SET_ROAM_CTRL_CMDID,
				 ohreshoDICAkr_	ed */
		if (snr < sq_thresh->upper_thadd_krk is ld_event *) datap;

	new_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =add_krk is ldld[1]) &&
			   (snr >= ICkrk ikrk iicatKRK_)
		re	 * Identify the threshold breached and communicatADDlKRK_p.
	 * After that install a neww set of threshol&
			   (rssi >v_txe_key is s_SET_ROAM_CTRL_CMDID,
				 o_SEkey plSG
	ed */
		if (snr < sq_thresh->upper_thves))
	ciph
	ukey is ld_event *) datap;
son_key plSG
 >n -EIMAdIKEYbg(AEXh;

	skb = ath6kl_wmi_L_DBG_WMI,
				   "spurious upper snshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =ves))
	ciph
	ukey is ldGNAL_QUALITY_METRICkey plSG
 =ukey plSG
Y_	 * Identify the threshold breached and communicatG_WMI,ldIPHrwlKEYbp.
	 * After that install a neww set of threshol&
			   (rssi > &wpmk- iis s_SET_ROAM_CTRL_CMDID,
				 ohreshoDICA_DTIMv_params_hreshoDICApmk-  obool  &went))
		return -EINVAL;

	reply = (structpmk- iis ld_event *) datap;
son_coid a=n this AC.	skb = ath6kl_wmi_get_p Id
& pmk- a=n this AC.	skb = ath6kl_wmi_
	new_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->snr;

	sq_thresh = &wpmk- iis ldld[1]) &&
			   (snr >= ICf(*cmd));
	if (!skb)
		re_get_p I    st  (snr >= ICpmk-  opmk-  oID_MASKETRICpmk- d_valMETRIC_r>ac  =uPMKIDtENread;threshold;
	c  (p I >= ICpmk-  o0 oID_MASKETRICpmk- d_valMETRIC_r>ac  =uPMKIDtDISread;turn -	/*dentify the threshold breached and communicate thbMKIDtp.
	 * After that install a neww set of thresholthresh,
					 u32 sizeALIT	 ts_i/
	foMASK;

		ac!Gfck_ck);

		/* Indicate stre 
		NC 1 c[for delete qos streamCMDID,
				ed: %sq_thresh =vLIT	h_eCATIONmber of *) datap;
son_on == CAC_INDICATION_Dstream ffrom AP
	 */_Gxh0GvrW;
		}f *vif)
{
	struct wmi_cac_event *repd = (sshruct roam_ for delete qoTIONmberWMI_SNvLIT	h_eC

	sq_thresh =vLIT	h_eCAld[1]) &&
			 vLIT	h_esilsac X_sh->lMSG) {
_wmiicatGATA_HDR_MSG_) {
d, pkt		 vLIT	h_esilsac3wouintvl);
	retlNf	/* &n -EIGATA_HDR_IF_laXERNEL);

		/*dentify th_entroor[i>pkts = cpu_to_le3uct rtreamo_reaset of thresholthresh,
					 u32 sizeu_df ete qb_SET_ROAM_CTRL_CMDID,
				ed: %
		return -EINVAL;

	reply = (stru_df is ld_eventsq_thresh =vLIT	u_df -EIs vLIT	u_df -EIs[struct iee  e  1 c[for delete qos stream;]d_params ,Xuum(prVEhoget_f(*wmidf *) dat(*wmidom  (p I vLIT	u_df -EIs o0 oID_MASKvLIT	u_df -EIsWMI_SNeturn 0;
}

int ath6kl_wmi((tsinfo >> IfMSGv U1 = (sstruct iee(reply->rssi);

	sqw2(intvl);

	return a kl_wmiplSG
	= APLISTum(prVEhoget_fEINVA		vLIT	u_df -EIs[uum(prVEhoget_f(shol.icerr_event *evstre 
		plSG
Y_Mize = si wmi_txe_notify_cmd *cmd;

	sk
	new_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->snr;

	sq_thresh = _df is ld.cipher),
				 = sq_thrd[2]) sh->>is ls		tsld[2]) _entroo Ep cold ;
 bitmapsq_thv1 = e len);eprepl which = e Den);S_df will>le s		td[1])) ETRICvLIT	u_df map(tim && tvl);

	returnmi((tsinfo >> IfMSGv U1 = (suum(prVEhoget_f(reply->rssi);
vLIT	u_df -EIs[reache.
	new_thresho-EI_alcmd(0retu_evenvLIT	u_df -EIs[reache.
	newICATIOhrAPLISdat(*wr = reply	nd communicate tha= sq_thrf -EINpperlcmdrn oretsinanyhv1 = e len)S_df failz_th			indnx

	er fold ;2]) s_dfhronSPECis lld[2]) _entroo epd[1])) {
		t *)nicgoto 	}f *resho;

	ha= sq_thSld ;s_df is lfmi->fh->lows_df len);messagerepl rlcsq_thdelete qs>ley_evuseven1])) cac_event))
		retreshold breached and communicateh->HRONIZkb,[2]) &&
			 that install a new) {
		t *)nicgoto 	}f *vLIT	u;

	hatsinfo >> IfMSGv U1 = (suum(prVEhoget_f(reply->rssi);
son_on == CA!vLIT	u_df -EIs[reache.
	n)6kl_wgoto 	}f *vLIT	u;

	haCtream frent))
	ac2 delete qos >pkts = cpu_to_le
	if (!activ vLIT	u_df -EIs[reache.
	if (!activ icerr_event *	struct evstrcmd b	 u32 sizeALIT	 ts_i/
	foreachvLIT	u_df -EIs[reache.
	ne
	if (!activtreamCMed andret);
vLIT	u_df -EIs[reache.
	new_ATIO ev->ld et *)nicwgoto 	}f *vLIT	u;

	urn -	/* AP lis
	}f *resho;
: (on 	}f  upnanyhresourcereleftsl(siz(po;
	bly du >= Wanrssi_t)1])) rW;
		}f *vif)
{
	st
	}f *vLIT	u;
:hatsinfo >> IfMSGv U1 = (suum(prVEhoget_f(reply->rs0GvrW;
		}f *vif)(
		return -EINVA)vLIT	u_df -EIs[reache.
	n)eww set of threshol&
			   (rssi >cget)
		return -EOP_SET_ROAM_CTRL_CMDID,
				   NO_SYower value *get)
		return -EOCAper thued */
		if (snr < sq_thresh->upper_thtget)
		return -EOCA_event_SE at;

	return sk_buf(*wmidfswmimmi_phy(*wmidfswminomew_l_phy(*wmidf *) datap;
son_!_(	rr th->user(prV_eve0x7_SHIFT) &
YN	/pvl);ac[	rr th->user(prV_&e0x7]ewIC	rr th->icerr_event *	SHIFT) &
YN		rr th->icerr_evdirecewICUPLINK_TRAFFICresy	ath6666	rr th->icerr_evdirecewICDNLINK_TRAFFICresy	ath6666	rr th->icerr_evdirecewICBIDIR_TRAFFIC	SHIFT) &
YN		rr th->icerr_evtstrd== TRAFFIC_) {
dAPERIODICresy	ath6666	rr th->icerr_evtstrd== TRAFFIC_) {
dPERIODIC	SHIFT) &
YN		rr th->voic
		rca;
	ewICDISreadyFOAL;
Ibh(&resy	ath6666	rr th->voic
		rca;
	ewICENreadyFOAL;
Ibh(&resy	ath6666	rr th->voic
		rca;
	ewICENreadyFOALALLieee8HIFT) &
YN		rr th->is)
 nrn -EIIMPLICIthbSTREAMresy	ath6666	rr th->is)
 <=n -EIMAdI;
INSTREAM) =ev->asscHREScac_event *repENOTCONNheckinomew_l PHY_buff istu= mewimalPHY_th			so		} elDUT lockrlcmw TSRS IEen1])) (on Ge= ~(1 physic_l buff (units v1 bps)1])) mmi_phy(*w(((struct wmi 	rr th->mmi_physt*vi) /;

	0) /;

	0)if (on Nheckimewimal phy (suomew_l phy buff gdmnson_	rr th->nomew_l_phy(u= mew_phyd = WMI_Sunit v1 500 kbps gdmn)nomew_l_phy(*w_	rr th->nomew_l_phy(*;

	0) /;5
	ifg_interval);

	ath6kl_dbg(ATH6KLv->kTSRS IEu)->ac d::MinPhy(%x->Nomew_lPhy(===>(%xof APs in amew_phy,inomew_l_phyret);
	rr th->nomew_l_phy(=inomew_l_phy;threshold;
	c	rr th->nomew_l_phy(=i0nt *repd =ew_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->f *vif)
{
	struct wmi_cac_event *re}

	r tget)
		return -EO: acypeEurn -:rntsinfo;
	u	rr th->icerr_event *,6	rr th->is)
)ly->snr;

	sq_thresh =tget)
		return -EOCAld[1]) &&
			   (snr >= sq	rr th oID_MASKshold_va (on T (sti wmi ct wiciolystget)
d F el;

	 gdmnson_(u32) 	rr th->is)
 nrn(u32)  -EIIMPLICIthbSTREAM1 *) esturn 0;
}

int ath6kl_wmi_s at;

	return sk_buf(*wqw2(intvl);

	return a
	if (kl_wmi	rr th->icerr_event *	wmi_sw2(intvl);

	return |= kl_wmi	rr th->icerr_event *	;
ct wmi_txe_notify_cmd *cmd;

	hreshold;
	cmd.ex wiciolystget)
d 	cmd return;

	cmd aE atl;

	 gdmnesturn 0;
}

int ath6kl_wmi_s at;

	return sk_buf(*wqw2(intvl);

	return a
	if (kl_wmi	rr th->icerr_event *	wmi_sw2(ins)
{
	struct sk_buff	rr th->icerr_event *] |=strcmd kl_wmi	rr th->i		ath6klse);
			rf a 	cmdreturn;becoexisb->dat, ~(1  atl;

	 autoern c_lly);
			becoexisb->dat
ify_cmd w2(intvl);

	return |= kl_wmi	rr th->icerr_event *	;
ct wmi_txe_notify_cmd *cmd;

	hr		 = sq_thrdh6kl_wmb->datyi)urngeeof(*cmd));
	if (!skb)
		t (sti w~(1atap;
irrn TWMI,of( (estget)
d md = (strucex wiciolyssinan ct wicioatap;
atl;

	 (st (et
	r tget)
dld[1])) {
		! at;

	return sk_bufn  Nh6kl_tkto_le32(rate);
	cmd->pkts = cpu_to_le
	if (!act	rr th->icerr_event *,6->ue	re	 * Identify the threshold breached and communicat,REATEhbSTREAMld[2]) &&
			 that install a new set of thresholing error.  Thives))
		return -EOP_SET_ROAM_CTRL_CMDID,
				 o_SEicerr_event *,  NO_SYD activ	ed */
		if (snr < sq_thresh->upper_thves))
		return -EOCA_event_, index);
	}

	(*wmidf *) datap;
son_icerr_event *e= (struct ieee80211_tspec_ie *) &(replyicerr_e ent *
			wmi->icerr_event *	struct wmi_cac_event *repd =BG_WMI,
				   "spurious upper snshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =ves))
		return -EOCAld[1]) &&
			 ETRICicerr_event *ev;icerr_event *		 ETRICi_event-smissinsturn 0;
}

int ath6kl_wmi_, u32 pkts, u32 intvl)
{
	struct sk_bufficerr_event *];si wmi_txe_notify_cmd *cmd;

	skson_!_, u32 pkts, ua kl_wmictiv	_Gxh0GvrW;
		}f *vif)
{
	struinterval);

	ath6kl_dbg(ATH6KLv->kTSMI,
 (m. Jn't eturn wmi_ccerr_e ent *
			wmi-H6KLv->ctive icerr_event *	struct wmi_caNOGATA;t *reply;
	struct ieee802N0_Gv,oevent *re}

	r ves))
		return -EO:_ccerr_e ent *
			>ctiv=rntsinfo;
	uicerr_event *,6-		at*
	 * Identify the threshold breached and communicatG_WMI,lbSTREAMld[2]) &&
			 >uppxy

	h->lower_thssinsturn 0;
}

int ath6kl_wmi_ intvl)
{
	struct sk_bufficerr_event *]dx,
			      	ath6k, u32 pkts, u32 intvl)
{
	struct sk_bufficerr_event *];si wmi_txe_notify_cmd *cmd;

	sk= sq_thrdh6kl_wms_get_new_buf(sizeof(*cmd));
	if (!skb)
		return -ENOEM;

	cmd = (struct wmi_txe_noti1])) {
		!b->data;
	cmd->rath6kl_tkto_le32(rate);
	cmd->pkts = cpu_to_le
	if (!acticerr_event *,6= cpu_to_lw2(intvl);

	return a,
			     cerr_event *	strrn -	/* AP threshol&
			   (rssi > &w_ip_ABOI_SET_ROAM_CTRL_CMDID,
				   NO
	__bewmi_ps0,	__bewmi_ps1ent))
		return -EINVAL;

	reply = (struct ip_ABOld_event *) datap;
md.Muln c_rn addre *et wmer f(reply])) {
		ipv4_is_muln c_rn(_ps0	resy	ath6ipv4_is_muln c_rn(_ps1st;

	skb = ath6kl_wmi_L_DBG_WMI,
				   "spurious upper snh->upper_thuct ip_ABOd_val) reply->range;
	snr = reply->snr;

	sq_thresh = &wmip_ABOldld[1]) &&
			 ETRIC_ps[0] n__ps0		 ETRIC_ps[1] n__ps1;,, * Identify the threshold breached and communicate thIPpp.
	 * After that install a new set of thresholds basevoply	   (rssi >=elinquish_ct wicio		return -reditsI_SET_ROAM_CTRL__RESP) &&
->data;
	cment_SEl)
{
	structent *) i
	sk= sq_thRelinquish -reditsev->raretuct wiciolystget)
d 	returnENOEM;sinceint atwe goeof(sleep.	rf userstget)
d ex wicioth			inmdreturns eturns;

	c md aE at;

	 lea(strhe_newt
->atap;
	ra~(1 usersof(*_txe_oti1])) sturn 0;
}

int ath6kl_wmi_l)
{
	struct(tim && tvl);

	returnmii wmi_txe_notify_cmd *cmd;

	skct wmi *wmi, u8 struct iee(re>rssi);
son_l)
{
	struct(a kl_wmip	= APLISse);

			wIXME: Is = (stcmd;(a txe_nohinside);

			ct wloop correct? mayftreamrework.);

		dmn)
pturn 0;
}

int ath6kl_wmi_sk, u32 pkts, u32 intvl)
{
	struct sk_buffi  e et wmi_txe_notify_cmd *cmd;

	skbSse);

			If ~(1 <<t wmer userstget)
d 	cmd returns);

			te = le~(1  at;

	);

		dmn)
*) skb->data;
	cmd->rate_l)
{
	struct(a,
			    iwmi_skSse);

q_thrdh6kl_wmew_buf(sizeof(*cmd));
	if (sk_);

q_th= (st at;

	 (	return));

q_tdmn)
th6kl_tkto_le32(rate);
	cmd->pkts = cpu_to_le
	if (re 
		p,6= cpu_to_l, id	ze = sizeowIXME: Caatwe doh= (stt *ignm		ts

	cx > cmd;
	r ?1])) sturn 0;
}

int ath6kl_wmi_w2(intvl);

	return =El)
{
	structent wmi_txe_notify_cmd *cmd;

	holthresh,
					 u32  &wmst wmi _bor_64I_SET_ROAM_CTRL_CMDID,
				   NO_SYNC_hresho_SET_RO_buf(sizest wmi _bor_CAmar_	ed */
		if (snr < sq_thres *) dat, apsd obor ; *u64 mcs

	ret

	rbor_[bssid,
ct iUANDS]
	reply = (struct atese =ctst*vif646kl_dbg(ATH6K  (p I &

	rbor_ o0 oID_MASK

	rbor_	_va (on !skb)checki2.4nor (5 GHzobor h oIkip_~(1 re t = ((tsinfbor (*wmi,bor (<= NL WMM_NUAND_5GHZi,bor _v1 *) eon copy 
	gacytbuff bor_CAdmn)

	rbor_[for  (*wbor_->_entroo[for  .
	gacyetu_evenbor (*= NL WMM_NUAND_5GHZ));



	rbor_[for  (*  NO_bor_->_entroo[for  .
	gacy    4
	skbon copy mcstbuff bor_CAdmn)mcs

	r(*wbor_->_entroo[for  .ht_mcs[1];mn)mcs

	r(<<= 8;mn)mcs

	r(|*wbor_->_entroo[for  .ht_mcs[0];mn)

	rbor_[for  (|*wbcs

	r(<< 12;mn)

	rbor_[for  (|*wbcs

	r(<< 28;t *reply;
	struct ieee802N0_Gv,oevent *R
	rbor_e64 st :i2.4:%llx 5:%llxtsinfo;
	u

	rbor_[0]et

	rbor_[1]

	sk
	new_threshold = (enum wmi_snr_threshold_thicatRATESlMODkbMAX_val) reply->range;
	snr = reply->snr;

	sq_thresh = &wmatese =ctst*vif646kl_dbld[1]) &&
			 tsinfapsdu=imi,apsdu<hicatRATESlMODkbMAXi,apsd_v1 *) eon A,apsdu>upprewmew 5GHZ,bor (t;
	cmd.papsdu==hicatRATESlMODkb11AN_DELETE) apsdu==hicatRATESlMODkb11A_HT20N_DELETE) apsdu==hicatRATESlMODkb11A_HT4rn  NObor (*wNL WMM_NUAND_5GHZihaCtcpu  NObor (*wNL WMM_NUAND_2GHZihaCcnr_thresbor_[apsd] n_intvl);
	64I

	rbor_[for  	strrn -	/*dentify the threshold breached and commu  NO_SY_thresholdresLI_SpRATESlp.
	 * After that install a new set of thresholds based on the si &wmst wmi _bor_p, iSET_ROAM_CTRL_CMDID,
				   NO_SYNC_hresho_SET_RO_buf(sizest wmi _bor_CAmar_	ed */
		if (snr < sq_thres *) dat, apsd obor ; *uwmimcs

	ret

	rbor_[bssid,
ct iUANDS]
	reply = (struct atese =ctst*vif326kl_dbg(ATH6K  (p I &

	rbor_ o0 oID_MASK

	rbor_	_va (on !skb)checki2.4nor (5 GHzobor h oIkip_~(1 re t = ((tsinfbor (*wmi,bor (<= NL WMM_NUAND_5GHZi,bor _v1 *) eon copy 
	gacytbuff bor_CAdmn)

	rbor_[for  (*wbor_->_entroo[for  .
	gacyetu_evenbor (*= NL WMM_NUAND_5GHZ));



	rbor_[for  (*  NO_bor_->_entroo[for  .
	gacy    4
	skbon copy mcstbuff bor_CAdmn)mcs

	r(*wbor_->_entroo[for  .ht_mcs[0];mn)

	rbor_[for  (|*wbcs

	r(<< 12;mn)

	rbor_[for  (|*wbcs

	r(<< 20;t *reply;
	struct ieee802N0_Gv,oevent *R
	rbor_ewmist :i2.4:%x 5:%xtsinfo;
	u

	rbor_[0]et

	rbor_[1]

	sk
	new_threshold = (enum wmi_snr_threshold_thicatRATESlMODkbMAX_val) reply->range;
	snr = reply->snr;

	sq_thresh = &wmatese =ctst*vif326kl_dbld[1]) &&
			 tsinfapsdu=imi,apsdu<hicatRATESlMODkbMAXi,apsd_v1 *) eon A,apsdu>upprewmew 5GHZ,bor (t;
	cmd.papsdu==hicatRATESlMODkb11AN_DELETE) apsdu==hicatRATESlMODkb11A_HT20N_DELETE) apsdu==hicatRATESlMODkb11A_HT4rn  NObor (*wNL WMM_NUAND_5GHZihaCtcpu  NObor (*wNL WMM_NUAND_2GHZihaCcnr_thresbor_[apsd] n_intvl);
	p, 

	rbor_[for  	strrn -	/*dentify the threshold breached and commu  NO_SY_thresholdresLI_SpRATESlp.
	 * After that install a new set of threshol&
			   (rssi > &w_st wmi _bor_ iSET_ROAM_CTRL_CMDID,
				   NO_hresho_SET_RO_buf(sizest wmi _bor_CAmar_	ed */
		if  2);
		if (tim && tim[1] >= 2);
son_irn -ENOMEM;

	cmd = (struct w64BISpRATESilter_cmdb->data;
	cmd->rssi =range;
	sn		 u32  &wmst wmi _bor_64Ireached and cbor_	*
Ctcpu  Nge;
	sn		 u32  &wmst wmi _bor_p, reached and cbor_	*
hol&
			   (rssi > &w_hon -sleep_apsdiis s_SET_ROAM_CTRL_CMDID,
				   NO_SYNC_		iA * p   (rshon -apsduhon -apsdent))
		return -EINVAL;

	reply = (struct hon -sleep_apsdiis ld_event *) datap;
son_(hon -apsdu!=>bssid,
HOSTlMODkbASLEEPe8HIFT) &
(hon -apsdu!=>bssid,
HOSTlMODkbAWAKE)e80211_tspec_ie *) &(replyhon (sleep apsd
			wmi->hon -apsdestruct wmi_cac_event *repd =BG_WMI,
				   "spurious upper snshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =uct hon -sleep_apsdiis ld.cipher),
				 of(ahon -apsdu==>bssid,
HOSTlMODkbASLEEPe80211_tspec_si >=elinquish_ct wicio		return -reditsIRL__ihaCcnr_tasleep n_intvl);
	p, 1

	hreshold;
	ccnr_ta}

	 n_intvl);
	p, 1

	hrn -	/*dentify the threshold breached and commu  NO_SY_threshoHOSTlSLEEPlMODkb,[2]) &&
			 that install a new set of thresholon T (st/
	for (has zero actgth payload*sq_thresh,
					 u32 sizehon -sleep_apsdiis _prcd_evc_indication == CAC_I
if (re 
		SYower va		 u32 q_fu*q_f	ed */
		if  2);
		if (tim && tim[1] >= 2);
 &wmst (HOSTlSLEEPlMODkb,[2hbssCESSED, &q_fICLITY*	str}

	_up(&b->d->cac_wq)eww set of list version 1 conten &wmwow_apsdiis s_SET_ROAM_CTRL_CMDID,
				   NO_iA * p   (rswow_apsd wow_apsd   NO__wmittimerkl_16>hon -req=vesay	ed */
		if (snr < sq_thresh->upper_th &wmwow_apsdiis ld_event *) datap;
son_(wow_apsd !=>bssid,
WOWlMODkbENreade8HIFT) &
wow_apsd !=>bssid,
WOWlMODkbDISreade80211_tspec_ie *) &(replywow apsd
			wmi->wow_apsdestruct wmi_cac_event *repd =BG_WMI,
				   "spurious upper snshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =uct wow_apsdiis ldGNAL_QUALITY_METRIC_r>ac -wow n_intvl);
	p, wow_apsdestrETRICdr_thr elmi_snr_threstr_threstrETRIChon -req=vesaywouintvl);
	rethon -req=vesay	;,, * Identify the threshold breached and communicate thWOWlMODkbp.
	 * After that install a new set of threshol&
			   (rssi >add_wow_patthrfinmgI_SET_ROAM_CTRL_CMDID,
				   NO_SYNDID 0;
}amCMDIDdr_threstat,  NO_SYNDIDdr_threoffsat, hreshoDICAttimerk  NO_SYNhreshoDICAbor_	ed */
		if (snr < sq_thresh->upper_thadd_wow_patthrfinmgCA_event_, istatus_DICAttimer_bor_wmt *) datap;
mdatap;Alcmdrne;

	in or
	a  (ory md = e -EINppeto t atatap;
i_thr or (bor_C(reuretwhich i w~wice v1 dr_threstatoti1])) sSPEC_TID_MASKshold_+ (2ap;
i_thr_pperesh (L_DBG_WMI,
				   "spurious uppereshold event: %d\n",
				   snr);
snr;

	sq_thresh =add_wow_patthrfinmgCAGNAL_QUALITY_METRIC
i_thr_ 0;
}am;

 0;
}amY_METRIC
i_thr_sSPEC_T
i_thr_sSPEY_METRIC
i_thr_offsatC_T
i_thr_offsatTH6K  (snr >= ICttimerklttimerklttimer_pperesh (ttimer_bor_;

	DICA)  >= ICttimer_+ ttimer_ppereshK  (snr ttimer_bor_ cbor_klttimer_pperesh (* Identify the threshold breached and communicatADDlWOWlPATTER pp.
	 * After that install a neww set of threshol
					 u32 sizeves_wow_patthrfinmgI_SET_ROAM_CTRL_CMDID,
				   NO_SYNDFIXwmi__IMviDFIXttimer_iv	ed */
		if (snr < sq_thresh->upper_thves_wow_patthrfinmgCA_event *) datap;

	new_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =ves_wow_patthrfinmgCAGNAL_QUALITY_METRIC
i_thr_ 0;
}am;

intvl);
	retwmi_->in
	eETRIC
i_thr_am;

intvl);
	retttimer_iv	*
	 * Identify the threshold breached and communicatG_WlWOWlPATTER pp.
	 * After that install a newaset of thresholthresh,
					 u32 sizereshold _xt	foMASK;

		ac!Gfck_ck);

		/* Indicate stre_SYNC 1 c[USGx_/
	for list versiostre_SYNC 1 c[USGgnten_Gw*if  ath6kl_dbg(ATH6KL_DBG_xWMI,
		   "number of *) datap;

	n (sshruct roam_ for delete qxWMI,
		 WMI_SNMI,
		  t lower valuexWMI,
		   ntAL_QUALITY_METR	h_esi_cmd)
 n_intvl);
	p, cmd->in
		 * Identify the threshold breach0 communicatEXTENSIO pp.
	 *  ts_id;

	ifw set of threshol
					 u32 size (enchalactge-resp_ABOI_SET_ROAM_CTRL_CMDwmi-ookiw C_wmisource	ed */
		if (snr < sq_thresh->upper_tx_hbnchalactge-resp_ABOCA_event *) datap;

	new_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->snr;

	sq_thresh x_hbnchalactge-resp_ABOCAGNAL_QUALITY_METRIC-ookiw n_intvl);
	p, cookiwn
	eETRICsource n_intvl);
	p, source	
		 * Identify the threshold _xt	foweachuct ricaX_HBICHALLENGE_RESPpp.
	 * After      that install a new set of threshol&
			   (rssi >hrefigkiebug=apsuldiis s_SET_ROAM_CTRL_CMDwmi(replCMDwmi-oefig	ed */
		if  2);
	_sh x_dbglog_cfg=apsuldiis ld_eventsq_thre(snr < sq_thres *) datap;

	new_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->snr;

	sq_thre 2);
	_sh x_dbglog_cfg=apsuldiis ldGNAL_QUALITY_METRIC(replyn_intvl);
	p, (repl)Y_METRIC-oefig n_intvl);
	p, coefig	
		 * Identify the threshold _xt	foweachuct ricaX_DBGLOG_CFGlMODULEpp.
	 * After      that install a new set of threshol&
			   (rssi > (enthre3_ABOI_SET_ROAM_CTRL_CMDID,
					ed *ge;
	sn		 u32 USGgnct we -EOPNOTSUed and cc_SGG thsTATISTICSpp.
	 newt version 1 conten &wmtx_	.bs-EOP_SET_ROAM_CTRL_CMDID,
				 o_SEdbM	ed */
		if (snr < sq_thresh->upper_th &wmtx_	.bs-EOCA_event *) datap;

	new_threshold = (enum wmi_snr_threh->upper_th &wmtx_	.bs-EOd_val) reply->range;
	snr = reply->snr;

	sq_thresh =uct tx_	.bs-EOCAGNAL_QUALITY_METRICvbM;

vbM;,, * Identify the threshold breached and communicate thTX_PWwld[2]) &&
			 that install a neww set of threshol&
			   (rssi > (entx_	.bs-EOP_SET_ROAM_CTRL_CMDID,
					ed *ge;
	sn		 u32 USGgnct we -EOPNOTSUed and cc_SGG thTX_PWwld[2])newt version 1 conten (enroam_tb6inmgI_SET_ROAM_CTRL_	ed *ge;
	sn		 u32 USGgnct we -EOPNOTSU0 cc_SGG thROAM_TBLpp.
	 newt version 1 conten &wmlpturnbldiis s_SET_ROAM_CTRL_CMDID,
				 o_SEthreu*,  NO_S_SE]turnbldinolicy	ed */
		if (snr < sq_thresh->upper_th &wmlpturnbldiis CA_event *) datap;

	new_threshold = (enum wmi_snr_threh->upper_th &wmlpturnbldiis d_val) reply->range;
	snr = reply->snr;

	sq_thresh =uct lpturnbldiis CAGNAL_QUALITY_METRICthreu* =El)reu*Y_METRIC]turnbldinolicy =E]turnbldinolicyhy, * Identify the threshold breached and communicate thcPREAMBLEpp.
	 * After that install a new set of threshol&
			   (rssi > &w_re3_ABOI_SET_ROAM_CTRL_CMDFIXthrest at	ed */
		if (snr < sq_thresh->upper_th &wmre3_ABOCA_event *) datap;

	new_threshold = (enum wmi_snr_threh->upper_th &wmre3_ABOd_val) reply->range;
	snr = reply->snr;

	sq_thresh =uct re3_ABOCAld[1]) &&
			 ETRICihrest at;

intvl);
	retthrest at	
		 * Identify the threshold breach0 communicate thRTSlp.
	 * After that install a new set of threshol&
			   (rssi > &w_wmm(txops_SET_ROAM_CTRL_CMDID,
				 o 1 c[USGgtxop_cfgO_bu	ed */
		if (snr < sq_thresh->upper_th &wmwmm(txopiis ld_event *) datap;
son_!((cfgO==hicatTXOPbDISreadDhres (cfgO==hicatTXOPbENreadD)st;

	skb = ath6kl_wmi_L_DBG_WMI,
				   "spurious upper snh->upper_thuct wmm(txopiis d_val) reply->range;
	snr = reply->snr;

	sq_thresh =uct wmm(txopiis ldld[1]) &&
			 ETRICixopi_r>ac  =ucfg;,, * Identify the threshold breached and communicate thWMMtTXOPbp.
	 * After that install a new set of threshol&
			   (rssi > &w_keeprepvdiis s_SET_ROAM_CTRL_CMDID,
				   NO_S_SEkeep_repvdiuctvi	ed */
		if (snr < sq_thresh->upper_th &wmkeeprepvdiis CA_event *) datap;

	new_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->snr;

	sq_thresh = &wmkeeprepvdiis CAGNAL_QUALITY_METRICkeep_repvdiuctvi =Ekeep_repvdiuctvi;,, * Identify the threshold breached and communicate thKEEPALIVkb,[2]) &&
			 that install a new) {
		t *return  Nh6kl_tkiebug= &wmkeeprepvd>pkts = cpu_to_le3keep_repvdiuctvi	eww set of threshol&
			   (rssi > &w_htcap_ABOI_SET_ROAM_CTRL_CMDID,
				   NO
	YNC 1 c[nlf(sizesor (bor ,  NO
	YNCsq_thre 2);
	_htcap *htcapent))
		return -EINVAL;

	reply = (struct htcap_ABOdbg(ATH6Kd =BG_WMI,
				   "spurious upper snshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =uct htcap_ABOdb.cipher),
				 = sq_thNOTE: Bor (in;
irmwt wmmatchesC 1 c[nlf(sizesor chet i wunlikCly);_th= (stwill>le )urnge (in;
irmwt w.	rf at	retur(1 <<i wmiyi)urngeein);_thsor ((reuret= e hon (treas? o>le fix_noti1])) ETRICbor (*wbor ; *ETRIChti_r>ac  =u!!htcapIChti_r>ac ; *ETRICht20_sgi =u!!(htcapICcap_lsac & IEEEf(sizeHT = (_SGI_20estrETRICht40  _phresh-(*  N!!(htcapICcap_lsac & IEEEf(sizeHT = (_SUP_WIDTH_20_40estrETRICht40  gi =u!!(htcapICcap_lsac & IEEEf(sizeHT = (_SGI_40estrETRIC&
	olerau_t40mhz(*  N!!(htcapICcap_lsac & IEEEf(sizeHT = (_40MHZ_INSOLERANTestrETRICmax_ampduif *_exp n_htcapICampduifactorly->f *vif)
{
	struct wmi_cac_event *S *rhtcap:wbor :%d hti_r>ac :%d 40mhz:%d  gi_20mhz:%d  gi_40mhz:%d 40mhz_&
	olerau_:%d ampduif *_exp:rntsinfo;
	uETRICbor , ETRIChti_r>ac , ETRICht40  _phresh-nfo;
	uETRICht20_sgi, ETRICht40  gi, ETRIC&
	olerau_t40mhznfo;
	uETRICmax_ampduif *_expnew set of tify the threshold breached and communicate thHT = (_p.
	 * After  that install a newhol&
			   (rssi >irn -ABOI_SET_ROAM_CTRL_CMvoply*us  roam__t f *p d */
		if (snr < sq_thres *) datTH6Kd =BG_WMI,
				   "spurious uf *phyl) reply->range;
	snr = reply->  (snr ipher),
	, us  rf *phy, * Identify the threshold breach0 communicatTESSpp.
	 * that install a neww set of threshol&
			   (rssi >mc_rnldr_threshoI_SET_ROAM_CTRL_CMDID,
				 obool mc_alc_o*p d */
		if (snr < sq_thressq_thresh =mc_rnldr_threshoCA_event *) datap;

	new_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =mc_rnldr_threshoCAGNAL_QUALITY_METRICmc_rnlalc__r>ac  =umc_alc_o*;,, * Identify the threshold breached and communicatMCASTvotherwld[2]) &&
			 that install a new set of thresholing error.  Thiadd_ves_mc_rnldr_threshoI_SET_ROAM_CTRL_CMDID,
				 
if (rDICAttimerkobool add_tr_thre d */
		if (snr < sq_thressq_thresh =mc_rnldr_threadd_ves_is ld_event *) datap;
son_tttimer[0] !ve0x33res ttimer[1] !ve0x33e8HIFT) &
(ttimer[0] !ve0x01res ttimer[1] !ve0x00resy	ath6ttimer[2] !ve0x5eres ttimer[3]u>T0x7f)e80211_tspec_wt n*) &(replymuln c_rn;
i_thr oddre *tsiestruct wmi_cac_event *repd =BG_WMI,
				   "spurious upper snshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =mc_rnldr_threadd_ves_is ldld[1]) &&
			   (snr >= ICmc_rnlmacklttimerklstruct MCASTvotherwlMACtADDR_SIZEnew setdentify the threshold breached and commu  NO_SYadd_tr_thr ?nicate thMCASTvotherwld[2]) :  NO_SY_thrG_WlMCASTvotherwld[2]) &&
			 that install a neww set of threshol&
			   (rssi > ta=sotifmenurncdiis s_SET_ROAM_CTRL_CMDID,
				 obool )-urncdent))
		return -EINVAL;

	reply = (struta=sotifmenurncdiis CA_event *) datap;

	new_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->snr;

	sq_thresh = ta=sotifmenurncdiis CAGNAL_QUALITY_METRIC_r>ac  = )-urncd ? len,
			 * Identify the threshold breached and commu After icatSTA_eck_bhENHANCEpp.
	 * After that install a new set of threshol&
			   (rssi > &w_regdomaifinmgI_SET_ROAM_CTRL_CMhresho)urr *alpha2	ed */
		if (snr < sq_thresh->upper_th &wmregdomaifinmgdbg(ATH6Kd =BG_WMI,
				   "spurious upper snshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =uct regdomaifinmgdbld[1]) &&
			   (snr >= ICiso_na*w Calpha2, 2neww set of tify the threshold breach0 commu After  icate thREGDOMAIN_p.
	 * After  that install a newholswmion 1 conten (enrateoMASK;

		ac!Gfck_c8t

	r plSG
	ed */
		if  2);
		if (tim && tim[1] >= 2)t_SElgi *wmidfswmidatap;
son_

	r plSG
retuRATE_AUTO>range;
	sn0va (on SGI<i wstorh-(a w~(1 MSBhv1 = e 

	r plSG
r])) {
		t
	r plSG
r&uRATE_g(AEX_MSB=ev->as
	r plSG
r&=uRATE_g(AEX_WITHOUSpSGI_RNELstrulgi *w1nt *repson_irn -ENOMEM;

	cmd = (struct wRATETreadyMCS15ilter_cmdb->data;
	cmd->rssi =si);
son_on == CAs
	r plSG
r>=>bRRAY_SIZEbreanrate_tb6imcs15)st;

nge;
	sn0va ( * Idenreanrate_tb6imcs15[(u32) s
	r plSG
][lgi]
	hreshold;
	cson_on == CAs
	r plSG
r>=>bRRAY_SIZEbreanrate_tb6)st;

nge;
	sn0va ( * Idenreanrate_tb6[(u32) s
	r plSG
][lgi]
	hrww set of thresholthresh,
					 u32 size (enpmk- iwmi_-->cac_indication == CAC_IoDICA&&
	pe
	if (!activDwmif *p d */
		if sizepmk- iwmi_-reply *reply; *uwmiexpecsh- f *;,epson_f *u<roam_ for delete qopmk- iwmi_-replyst;

	skb = ath6kl_wmi_reply 

	sq_thresh =imk- iwmi_-reply *)vLITp; *expecsh- f *C_TID_MASKreplykluum(pmk- d +;

(struct wmi replykluum(pmk- d thicatbMKIDtLEN;,epson_f *u<rexpecsh- f *t;

	skb = ath6kl_wmi_ret of list vthresh,
					 u32 sizeaddba-req=->cac_indication == CAC_IoDICA&&
	pe,
			_ERD_FLA (!ower va		 u32 q_fu*q_f	ed */
		if sizeaddba-req=->cacdbg(A;

	sq_thresh =addba-req=->cacdb) vLITp; ->fggr-recv=addba-req=->t(q_f, ETRICtsiostre_
	reuct wmi ETRICthholq_no), ETRICwifisz)eww set of list vthresh,
					 u32 sizeAelba-req=->cac_indication == CAC_IoDICA&&
	pe,
			_ERD_FLA (!ower va		 u32 q_fu*q_f	ed */
		if sizeAelba-->cacdbg(A;

	sq_thresh =Aelba-->cacdb) vLITp; ->fggr-recv=Aelba-req=->t(q_f, ETRICtsi)eww set of list von  AP apsdufuncn ors gdmling error.  Thiap i_etr_e_/
	fiqb_SET_ROAM_CTRL_pCMDID,
				   NO_Ssq_thresh =t_rx(strnmgdbpent))
		return -EINVAL;

	reply = (strt_rx(strnmgdbcment *) dasTH6Kd =BG_WMI,
				   "spurious upper snshod_val) reply->range;
	snr = reply->sn;

	sq_thresh =t_rx(strnmgdbld[1]) &&
			   (snr >=, p oID_MASKsho))eww sesdentify the threshold breapched and communicatAP_CONFIG_COMMISpp.
	 * After that install a new f *vif)
{
	struct wmi_cac_event *%s: nw tstrolu auth_apsdolu cholu c>u.ad[siz=0x%x-> das=rntsinfo;
	u__func__, pkluw tstr EpICauth_apsd rf reuct wmi pICch)nfo;
	u(struct wmi 	ICc>u.ad[siz), dasnew set of thsesholing error.  Thiap=uct mlmeoMASK;

		ac!GfcpCMDID,
				 o_SEc= sqhreshoDICAboc,  NO
	YDFIXreaso*p d */
		if (snr < sq_thressq_thresh =ap=uct mlmernmgdbcmen6Kd =BG_WMI,
				   "spurious upper snshod_val) reply->range;
	snr = reply->sn;

	sq_thresh =ap=uct mlmernmgdbld[1]) &&
			   (snr >=ICmac" macf (!skb)
		re_>=ICreaso*;

intvl);
	retreaso*pre_>=ICg(A;

g(ATH6Kf *vif)
{
	struct wmi_cac_ "ap=uct mlme:
g(AypeEreaso*=rntsin >=ICg(Anfo;
	uETICreaso*neww set of tify the threshold breapched and communicatAP_e thMLskb,[2]) &&
			  that install a newhol&
			   (rssi >ap=hidden_p - s_SET_ROAM_CTRL_CMDID,
				 obool )->ac p d */
		if (snr < sq_thressq_thresh =ap=hidden_p - inmgdbg(ATH6Kd =BG_WMI,
				   "spurious upper snshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =ap=hidden_p - inmgdbGNAL_QUALITY_METRIChidden_p -  = )->ac  ? len,
			 * I of tify the threshold breached and communicatAP_HIDDEN	sSIDld[2]) &&
			  that install a newholon T (st/
	for (will>le usev? o>)->ac /dis>ac  AP uAPSD feaI o	 gdming error.  Thiap=uct aps s_SET_ROAM_CTRL_CMDID,
				 o_SE)->ac p d */
		if  Thiap=uct aps iis ld_eventsq_thre(snr < sq_thre6Kd =BG_WMI,
				   "spurious upper snshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =ap=uct aps iis ld)AL_QUALITY_METRIC_r>ac  = )->ac ; 	 * I of tify the threshold breached and communicatAP_e thAPSDb,[2]) &&
			  that install a newhol&
			   (rssi >uct aps ibfrd_ cerI_SET_ROAM_CTRL_CMDID,
				 
if (ractivD &&
IMviDFIXbitmapCMDwmid[siz) d */
		if  Thiap=aps ibEINpped_ cerr_eves ld_eventsq_thre(snr < sq_thre6Kd =BG_WMI,
				   "spurious upper snshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =ap=aps ibEINpped_ cerr_eves ld)AL_QUALITY_METRICaam;

intvl);
	retapl)Y_METRICbitmap;

intvl);
	retbitmapn
	eETRIC
[siz elmi_snr_threstITY*	st	 * I of tify the threshold breached and commu After  icatAP_APSDbBUFFERED_TRAFFIC_p.
	 * After  that install a newholshresh,
					 u32 sizepsit wm->cac_indication == CAC_IoDICA&&
	pe,
			_ERD_FLA  
		SYower va		 u32 q_fu*q_f	ed */
		if sizepsit wm->cac *= 2);
son_f *u<roam_ for delete qopsit wm->cacst;

	skb = ath6kl_wmi_ev 

	sq_thresh =isit wm->cac *) vLITp; ->f	 u32 isit wm->cac(q_f, f reuct wmi evICaam))eww set of list vthresh,
					 u32 sizeAindexpiry=->cac_indication == CAC_IoDICA&&
	pe,
			_ERD_FLA (!Yower va		 u32 q_fu*q_f	ed *f *vif)
indexpiry=->cac(q_f)eww set of list version 1 conten &wmpvbiis s_SET_ROAM_CTRL_CMDID,
				 o_ &&
IMv  NO
	Ybool 6kl_dbg(ATH6KL_D(snr < sq_thresh->upper_thapn &wmpvbiis CA_event *) datap;

	new_threshold = (enum wmi_snr_threh->upper_thapn &wmpvbiis d_val) reply->range;
	snr = reply->snr;

	sq_thresh =ap=uct pvbiis CAGNAL_QUALITY_METRICaam;

intvl);
	retapl)Y_METRICrsvm;

intvl);
	ret0n
	eETRIC
[si elmi_snr_threstITYesh (* Identify the threshold breached and communicatAP_e thPVBbp.
	 * After that install a neww set of threshol&
			   (rssi > &w_in_framerforma> is s_SET_ROAM_CTRL_CMDID,
				  After      DIDin_meta_verk  NO_SYN 
	Ybool in_dot11	h_e obool defrag_onehon dbg(ATH6KL_D(snr < sq_thresh->upper_thin_framerforma> is CA_event *) datap;

	new_threshold = (enum wmi_snr_threshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =in_framerforma> is CAGNAL_QUALITY_METRICvot11	h_e;

in_dot11	h_e ? len,
		METRICvefrag_onehon ;

vefrag_onehon ;? len,
		METRICmeta_ver;

in_meta_verva (on De = le~(1 cmdrl fggrEl)red ro*;hon ;])) cac_event))
		retreshold breached and communicatRX_FRAMdyFOAMASpp.
	 * After that install a neww set of threshol&
			   (rssi > &w_appidiis s_SET_ROAM_CTRL_CMDID,
				 o_SEmgmt_frm tstr   NO SSYNhreshoDICAie o_SEidif *p d */
		if (snr < sq_thressq_thresh =uct appidiis  *pap;

	new_threshold = (enum wmi_snr_threspd_+ idif *pval) reply->range;
	snr = reply->f *vif)
{
	struct wmi_cac_event *uct appidiis :Emgmt_frm tstrolu idif *olutsinfo;
	umgmt_frm tstr  idif *pvalp;

	sq_thresh =uct appidiis  *GNAL_QUALITY_M	ICmgmt_frm tstr =umgmt_frm tstrY_M	ICidif * =uidif *2);
son_iea!n thisd
& idif * >urn  N  (snr 	ICidilsac  id  idif *pvaw set of tify the threshold breached and communicate thAPPIkb,[2]) &&
			  that install a newhol&
			   (rssi > &wmidiis s_SET_ROAM_CTRL_CMDID,
				 o_SEidild o_SEidifielMv  NO
	hreshoDICAieilsac  _SEidif *p d */
		if (snr < sq_thressq_thresh =uct idiis  *pap;

	new_threshold = (enum wmi_snr_threspd_+ idif *pval) reply->range;
	snr = reply->f *vif)
{
	struct wmi_cac_ *uct idiis :Eidildolu idiidifielMolu idif *olutsinfo;
	uidild oidifielMv idif *pvalp;

	sq_thresh =uct idiis  *GNAL_QUALITY_M	ICidild =uidiamY_M	ICidifielM =uidifielMY_M	ICidif * =uidif *2)
son_ie_lsac && idif * >urn  N  (snr 	ICidilsac  idilsac  idif *pvaw set of tify the threshold breached and communicate thIkb,[2]) &&
			  that install a newhol&
			   (rssi >dis>ac _11bst*vifiis s_SET_ROAM_CTRL_CMbool dis>ac 	ed */
		if (snr < sq_thresh->upper_thvis>ac _11bst*vifiis dbg(ATH6Kd =BG_WMI,
				   "spurious upper snshold_val) reply->range;
	snr = reply->f *vif)
{
	struct wmi_cac_ *vis>ac _11bst*vifiis : dis>ac olutsinfo;
	udis>ac 	;->snr;

	sq_thresh =vis>ac _11bst*vifiis dbGNAL_QUALITY_METRICvis>ac  =udis>ac  ? len,
			 * I of tify the threshold breach0 communicatDISready11BpRATESlp.
	 * After  that install a newhol&
			   (rssi >remaifionechn6inmgI_SET_ROAM_CTRL_CMDID,
				 o_wmidreq o_wmidure d */
		if (snr < sq_thressq_thresh =remaifionechn6inmg *pap;

	new_threshold = (enum wmi_snr_threspd_val) reply->range;
	snr = reply->f *vif)
{
	struct wmi_cac_ *remaifionechn6inmg:idreqolu durolutsinfo;
	udreq odurevalp;

	sq_thresh =remaifionechn6inmg *GNAL_QUALITY_M	ICdreq elmi_snr_threstreq)Y_M	ICdurrn oreelmi_snr_thresdurevalset of tify the threshold breached and communicatREMAIN_O ppHNLld[2]) &&
			  that install a newholon 	   (rssi > &nde);
	ofinmgdis? o>le deprect)
dl Use
 n 	   (rssi > &ndemgmt_nmgdiesheadl T(1 um ufuncn or  _phress P2P
 n mgmtu>uppre ors us
	r thresor &
	erface.
*sq_thresh,
					 u32 size &nde);
	ofinmgI_SET_ROAM_CTRL_CMDID,
				 o_wmisiostre_SYNC o_wmidreq o_wmiwait, hreshoDICA),
	,stre_SYNC o_ &&vLIT	f *p d */
		if (snr < sq_thressq_thresh =ucnde);
	ofinmgd*paprDICAr <2);
son_waitt;

	skb = ath6kl_w on Offload*tsinwait	er fo_phresh-(])) (r < =Ekmalcmd(vLIT	f *, GFP_KERNEL_val) repr <>range;
	snr = reply->
	new_threshold = (enum wmi_snr_threspd_+ vLIT	f *pval) reply->d;
	c		}f (r <>;range;
	snr = replyhrww 		}f (m && l_rnlmgmt_tn_frameeshK  (snr us  r),
	, vLIT	f *pvalm && l_rnlmgmt_tn_frame(*wb <2)lm && l_rnlmgmt_tn_frameif * =uvLIT	f *ly->f *vif)
{
	struct wmi_cac_event *ucnde);
	ofinmg:EiMolu dreqolu waitolu f *olutsinfo;
	uiMv dreq owait, vLIT	f *pvalp;

	sq_thresh =ucnde);
	ofinmgd*GNAL_QUALITY_M	ICilyn_intvl);
	p, pl)Y_M	ICdreq elmi_snr_threstreq)Y_M	ICwait	n_intvl);
	p, waittY_M	ICf * =uintvl);
	retvLIT	f *pval  (snr 	IC),
	, vLIT, vLIT	f *pvalset of tify the threshold breached and communicate ND_ACTIO pp.
	 * After  that install a newholshresh,
			__	   (rssi > &ndemgmt_nmgI_SET_ROAM_CTRL_CMDID,
				 o_wmisiostre_SYNC o_wmidreq o_wmiwait, hreshoDICA),
	,stre_SYNC o_ &&vLIT	f * o_wmino_cc_	ed */
		if (snr < sq_thresh->upper_th &ndemgmt_nmgd*paprDICAr <2);
son_waitt;

	skb = ath6kl_w on Offload*tsinwait	er fo_phresh-(])) (r < =Ekmalcmd(vLIT	f *, GFP_KERNEL_val) repr <>range;
	snr = reply->
	new_threshold = (enum wmi_snr_threspd_+ vLIT	f *pval) reply->d;
	c		}f (r <>;range;
	snr = replyhrww 		}f (m && l_rnlmgmt_tn_frameeshK  (snr us  r),
	, vLIT	f *pvalm && l_rnlmgmt_tn_frame(*wb <2)lm && l_rnlmgmt_tn_frameif * =uvLIT	f *ly->f *vif)
{
	struct wmi_cac_event *ucnde);
	ofinmg:EiMolu dreqolu waitolu f *olutsinfo;
	uiMv dreq owait, vLIT	f *pvalp;

	sq_tpper_th &ndemgmt_nmgd*GNAL_QUALITY_M	ICilyn_intvl);
	p, pl)Y_M	ICdreq elmi_snr_threstreq)Y_M	ICwait	n_intvl);
	p, waittY_M	ICno_cc_	n_intvl);
	p, no_cc_	Y_M	ICf * =uintvl);
	retvLIT	f *pval  (snr 	IC),
	, vLIT, vLIT	f *pvalset of tify the threshold breached and communicate ND_MGMTb,[2]) &&
			  that install a newhol&
			   (rssi > &ndemgmt_nmgI_SET_ROAM_CTRL_CMDID,
				 o_wmisioo_wmidreq 
 NO__wmiwait, hreshoDICA),
	,o_ &&vLIT	f * 
 NO__wmino_cc_	ed *&
		l)reu*Y_M/
		if  2);
		if (tim && tim[1] >= 2)epson_irn -ENOMEM;

	cmd = (struct wSTA_P2PDEV_DUPLEXilter_cmdb->data;
	cmd->rssi =si);
se);
			rf ;
	cmle v1 do
	r P2P mgmtu>uppre ors us
	r);
			thresor &
	erface cold ;

	in or
	alsacrma>sor likC);
			t_phresh-(t*vif>= Wadvertise or (xmit	t*vif>sk_);
			prob1 requrn s

q_tdmn)threu* =E__	   (rssi > &ndemgmt_nmgIb->dreached and ciMv dreq _FLA (T) &
wait, vLIT,&vLIT	f * 
 NO_(T) &
no_cc_	Y_Mreshold;
	cthreu* =E		 u32 size &nde);
	ofinmgIb->dreached and ciMv dreq _FLA (T) &
wait, vLIT,&vLIT	f *)
	hrww set of l)reu*Y_hol&
			   (rssi > &ndeprob1-resporsdiis s_SET_ROAM_CTRL_CMDID,
				 o_wmidreq 
 NO_SYN 
	YhreshoDICA)st, hreshoDICA),
	,stre_SYNC oo_ &&vLIT	f *p d */
		if (snr < sq_thressq_thresh =p2peprob1-resporsdiis d*paproam__t reshf *C_TID_MASKspd_+ vLIT	f *2)epson_vLIT	f *return  Nreshf *++w on workdb-oud ;2ar (esmewimum actgth requiremcac */y->
	new_threshold = (enum wmi_sreshf *pval) reply->range;
	snr = reply->f *vif)
{
	struct wmi_cac_event *ucndeprob1-resporsdiis :idreqolu dstolpM f *olutsinfo;
	udreq odst, vLIT	f *pvalp;

	sq_tpper_thp2peprob1-resporsdiis d*GNAL_QUALITY_M	ICdreq elmi_snr_threstreq)Y_M  (snr 	IC)rn ina>sor=addr odst, (!skb)
		re_	ICf * =uintvl);
	retvLIT	f *pval  (snr 	IC),
	, vLIT, vLIT	f *pvalset of tify the threshold breached and commu After  icate N2hbssBE_RESPONSkb,[2]) &&
			  that install a newhol&
			   (rssi >prob1-rehres-req=is s_SET_ROAM_CTRL_CMDID,
				 obool )->ac 	ed */
		if (snr < sq_thresh->upper_thprob1-req-rehres-nmg *pap;

	new_threshold = (enum wmi_snr_threspd_val) reply->range;
	snr = reply->f *vif)
{
	struct wmi_cac_ *prob1-rehres-req=is : )->ac olutsinfo;
	u)->ac 	valp;

	sq_tpper_thprob1-req-rehres-nmg *GNAL_QUALITY_M	IC_r>ac  = )->ac ;? len,
		Mset of tify the threshold breached and communicatbssBE_REQ_REPORTb,[2]) &&
			  that install a newhol&
			   (rssi >lsac-req=is s_SET_ROAM_CTRL_CMDID,
				 o_wmissac-req=d[siz) d */
		if (snr < sq_thresh->upper_th (enp2p_lsac *pap;

	new_threshold = (enum wmi_snr_threspd_val) reply->range;
	snr = reply->f *vif)
{
	struct wmi_cac_ *lsac-req=is :id[siz=%xtsinfo;
	ussac-req=d[siz)valp;

	sq_tpper_th (enp2p_lsac *GNAL_QUALITY_M	ICisac-req=d[sizyn_intvl);
	p, psac-req=d[siz)valset of tify the threshold breached and communicatG thP2P_INFOb,[2]) &&
			  that install a newhol&
			   (rssi >crncdl>remaifionechn6inmgI_SET_ROAM_CTRL_CMDID,
					ed *f *vif)
{
	struct wmi_cac_ *crncdl>remaifionechn6inmgtsiestrge;
	sn		 u32 USGgnct we -EOPNOTSUed and stre_SYNC icat,ANCELtREMAIN_O ppHNLld[2])newhol&
			   (rssi > &wmiw_bu_uppio s_SET_ROAM_CTRL_CMDID,
				 o *) iw_bu_indeou dbg(ATH6KL_D(snr < sq_thresh->upper_th &wmiw_bu_uppio inmgdbg(ATH6Kd =BG_WMI,
				   "spurious upper snshold_val) reply->range;
	snr = reply->snr;

	sq_thresh =uct iw_bu_uppio inmgdbld[1]) &&
			 ETRIC_w_bu_uppio yn_intvl);
	p, ps_bu_indeou d		 ETRICuum(null_func *wmid	 * I of tify the threshold breached and communicatAP_CONN_INACTb,[2]) &&
			  that install a newholds basevoply	   (rssi >hbnchalactge-resp_->cac(ication == CAC_IoDICA&&
	pe
	if (!activ,
			_ERdbg(ATH6KL_DBG_xWhbnchalactge-resp_ABOCA_even;
son_f *u<roam_ for delete qxWhbnchalactge-resp_ABOst;

	skb =ly->snr;

	sq_thresh x_hbnchalactge-resp_ABOCAGNvLITp; *	   (rsrecovery_hbn->cac(wkts = cpu_to_le
	if u(struct wmi cTRIC-ookiw)newholshresh,
					 u32 size_entroo=in_xt	foMASK;

		ac!Gfck_ck);

		/* Indicatedbg(ATH6KL_DBG_xWMI,
		   "nu; *uwmif *2)
_ &&amY_MDICA&&
	pent *) dat *wmid	 son_l1]) f *u<roam_ for delete qxWcI,
		 WM80211_tspec_ie *)bad packat 1tsiestruct wmi_cac_event *repsnr;

	sq_thresh x_MI,
		   ld[1]) &&
			 am;

 struct wmi cTRIC-md->in
		 s	n (sllruct roam_ for delete qxWMI,
		 WMI_SN&&
	pC_TI1]) &&
			 f *C_TI1]) f *
		 switch  pl)8021caoldicaX_HBICHALLENGE_RESPpEVENTID:
ruinterval);

	ath6kl_dbg(AT "		ac->cac hb chalactge thsptsiestru	   (rssi >hbnchalactge-resp_->cac(Gfck_&&
	pe,f *pval	brea_wmtcaoldicaX_DBGLOG_EVENTID:
ruinterval);

	ath6kl_dbg(AT "		ac->cac dbglog,f *			wmi->f *pval	h6kl_tkiebug=fwlog_->cac(wkts = cpu_to_le_&&
	pe,f *pval	brea_wmtdefault:
ruintervawt n*)unknown nmgdid 0x%xwmi->	ath6kldat *wcac_event 	brea_wmtrww set of thresholthresh,
					 u32 sizeroam_tb6i->cac_indication == CAC_IoDICA&&
	pe,
			_ER	ed *ge;
	sn		 u32 iebug=roam_tb6i->cac(wkts = cpu_to_le_&&
	pe,f *pvaholon Proce *e&
	erface specir_e 		ac->cacs, halacr wouldudrele~(1 &&
	pCsq_thresh,
					 u32 sizeproci->cacs q_fI_SET_ROAM_CTRL_CMDFIX,
				 o_ &&cversiostre_	DICA&&
	pe,Dwmif *p d */
		if 		 u32 q_fu*q_f
		 q_fuG_WMI,
		 "spq_f_by plSG
(wkts = cpu_to_le_,
					val) repq_f)80211_tspec_l);

	ath6kl_dbg(ATH6KLv->kW	ac->cac tsinunavail>ac ;q_f, q_f_plSG
:		wmi-H6KLv->_,
					valuct wmi_cac_event *repdwitch  -md->in8021caoldica_CONNI_SpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_CONNI_SpEVENTIDtsiestruct wmi_		 u32 size_enx(str->cac_indGfck_&&
	pe,f *, q_f)wmtcaoldicatDISCONNI_SpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_DISCONNI_SpEVENTIDtsiestruct wmi_		 u32 sizedis_enx(str->cac_indGfck_&&
	pe,f *, q_f)wmtcaoldicatTKIP_MICERRpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_TKIP_MICERRpEVENTIDtsiestruct wmi_		 u32 sizetkip_micie r->cac_indGfck_&&
	pe,f *, q_f)wmtcaoldicatBSSINFObEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_BSSINFObEVENTIDtsiestruct wmi_		 u32 sizebsspsac-->cac_indGfck_&&
	pe,f *, q_f)wmtcaoldicatNEIGHBOR_REPORTbEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_NEIGHBOR_REPORTbEVENTIDtsiestruct wmi_		 u32 sizeneighbor-rehres-->cac_indGfck_&&
	pe,f *,_FLA (TLv->q_f)wmtcaoldicatS,AN_COMPWMI,lEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_S,AN_COMPWMI,lEVENTIDtsiestruct wmi_		 u32 sizescrn_/
	ps))
	indGfck_&&
	pe,f *, q_f)wmtcaoldicatREPORTbsTATISTICSpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_REPORTbsTATISTICSpEVENTIDtsiestruct wmi_		 u32 sizeshre3_->cac_indGfck_&&
	pe,f *, q_f)wmtcaoldicatCACpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_CACpEVENTIDtsiestruct wmi_		 u32 size_ac_->cac_indGfck_&&
	pe,f *, q_f)wmtcaoldicatPSPOLLpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_PSPOLLpEVENTIDtsiestruct wmi_		 u32 sizepsit wm->cac_indGfck_&&
	pe,f *, q_f)wmtcaoldicatDTIMEXPIRYpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_DTIMEXPIRYpEVENTIDtsiestruct wmi_		 u32 sizedindexpiry=->cac_indGfck_&&
	pe,f *, q_f)wmtcaoldicatADDBA_REQ_EVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_ADDBA_REQ_EVENTIDtsiestruct wmi_		 u32 sizeaddba-req=->cac_indGfck_&&
	pe,f *, q_f)wmtcaoldicatDELBA_REQ_EVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_DELBA_REQ_EVENTIDtsiestruct wmi_		 u32 sizedelba-req=->cac_indGfck_&&
	pe,f *, q_f)wmtcaoldicateshoHOSTlSLEEPlMODkb,[2hbssCESSED_EVENTID:
ruinterval);

	ath6kl_dbg(ATH6KLv->kWcateshoHOSTlSLEEPlMODkb,[2hbssCESSED_EVENTIDiestruct wmi_		 u32 sizehon -sleep_apsdiis _prcd_evc_indGfck_q_f)wmtcaoldicatREMAIN_O ppHNLlEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_REMAIN_O ppHNLlEVENTIDtsiestruct wmi_		 u32 sizeremaifionechn6i->cac_indGfck_&&
	pe,f *, q_f)wmtcaoldicatCANCELtREMAIN_O ppHNLlEVENTID:
ruinterval);

	ath6kl_dbg(ATH6KLv->kWcatCANCELtREMAIN_O ppHNLlEVENTIDtsiestruct wmi_		 u32 size_ancdl>remaifionechn6i->cac_indGfck_&&
	pe_FLA (TL	,f *, q_f)wmtcaoldicatTXbsTATUSpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_TXbsTATUSpEVENTIDtsiestruct wmi_		 u32 sizetx_l)reu*i->cac_indGfck_&&
	pe,f *, q_f)wmtcaoldicatRXtbssBE_REQ_EVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_RXtbssBE_REQ_EVENTIDtsiestruct wmi_		 u32 sizerxhprob1-req-->cac_indGfck_&&
	pe,f *, q_f)wmtcaoldicatRXtACTIO pEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_RXtACTIO pEVENTIDtsiestruct wmi_		 u32 sizerxh);
	ofi->cac_indGfck_&&
	pe,f *, q_f)wmtcaoldicatTXE_NOTIFYpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_TXE_NOTIFYpEVENTIDtsiestruct wmi_		 u32 sizetxe_notify=->cac_indGfck_&&
	pe,f *, q_f)wmtdefault:
ruinterval);

	ath6kl_dbg(AT "unknown nmgdid 0x%xwmi->-md->in
	luct wmi_cac_event *repset of list vthresh,
					 u32 sizeproci->cacsoMASK;

		ac!Gfck_ck);

		/* Indicatedbg(ATH6KL_DBG_WMI,
		   "nu; * *) dat *wmid*uwmif *2)
_ &&amY_MDIC,
				Y_MDICA&&
	pen->snr;

	sq_thresh =MI,
		   ld[1]) &&
			 am;

 sreuct wmi ETRIC-md->in
	l,
				;

 sreuct wmi ETRICpsac1) &dicatC[2hHDR_IF_ID_RNELst	 s	n (sllruct roam_ for delete qWMI,
		 WMI_N&&
	pC_TI1]) &&
			 f *C_TI1]) f *
		 interval);

	ath6kl_dbg(AT "		acrxdid %d,f *			wmi->iMv f *pvalinterval);_dump

	ath6kl_dbg(A_DUMP, thisT "		acrxdi-H6KL&&
	pe,f *pva	 switch  pl)8021caoldicatG thBITRATE_,[2]):
ruinterval);

	ath6kl_dbg(AT "ica_G thBITRATE_,[2])tsiestruct BG_WMI,
				  st wmi _reply_indGfck_&&
	pe,f *pval	brea_wmtcaoldica_G thCHANNIL_LISSpp.
	 :
ruinterval);

	ath6kl_dbg(AT "ica_G thCHANNIL_LISSpp.
	 tsiestruct BG_WMI,
				  chiwmi_-reply_indGfck_&&
	pe,f *pval	brea_wmtcaoldica_G thTX_PWwld[2]):
ruinterval);

	ath6kl_dbg(AT "ica_G thTX_PWwld[2])tsiestruct BG_WMI,
				  tx_	.bsreply_indGfck_&&
	pe,f *pval	brea_wmtcaoldica_READYpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_READYpEVENTIDtsiestruct BG_WMI,
				  ready=->cac_indGfck_&&
	pe,f *pval	brea_wmtcaoldica_PEER_NOD,lEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_PEER_NOD,lEVENTIDtsiestruct BG_WMI,
				  peer_nosdi->cac_indGfck_&&
	pe,f *pval	brea_wmtcaoldica_REGDOMAIN_EVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_REGDOMAIN_EVENTIDtsiestru	   (rssi >regdomaifi->cac(Gfck_&&
	pe,f *pval	brea_wmtcaoldica_PSTREAM_TIMEOUSpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_PSTREAM_TIMEOUSpEVENTIDtsiestruct BG_WMI,
				  pseturn indeou i->cac_indGfck_&&
	pe,f *pval	brea_wmtcaoldica_d[2ERRORpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_d[2ERRORpEVENTIDtsiestruct BG_WMI,
				  erro r->cac_indGfck_&&
	pe,f *pval	brea_wmtcaoldica_RSSa_THRESHOLD_EVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_RSSa_THRESHOLD_EVENTIDtsiestruct BG_WMI,
				  rss  threst atr->cac_indGfck_&&
	pe,f *pval	brea_wmtcaoldica_ERRORpREPORTbEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_ERRORpREPORTbEVENTIDtsiestrubrea_wmtcaoldica_OPTtRX_FRAMdyEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_OPTtRX_FRAMdyEVENTIDtsiestru/th= (st->cac has be *	deprect)
d_tdmn)brea_wmtcaoldica_REPORTbROAM_TBLpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_REPORTbROAM_TBLpEVENTIDtsiestruct BG_WMI,
				  roam_tb6i->cac_indGfck_&&
	pe,f *pval	brea_wmtcaoldica_EXTENSIO pEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_EXTENSIO pEVENTIDtsiestruct BG_WMI,
				  centroo=in_xt	foweachuctpval	brea_wmtcaoldica_dHANNIL_dHANGdyEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_dHANNIL_dHANGdyEVENTIDtsiestrubrea_wmtcaoldica_REPORTbROAM_DATApEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_REPORTbROAM_DATApEVENTIDtsiestrubrea_wmtcaoldica_TESSpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_TESSpEVENTIDtsiestruct BG_WMI,
				  trn -indGfck_&&
	pe,f *pval	brea_wmtcaoldica_G thFIXRATESlp.
	 :
ruinterval);

	ath6kl_dbg(AT "ica_G thFIXRATESlp.
	 tsiestruct BG_WMI,
				  r
	rbor_sreply_indGfck_&&
	pe,f *pval	brea_wmtcaoldica_TX_RETRYpERRpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_TX_RETRYpERRpEVENTIDtsiestrubrea_wmtcaoldica_SNR_THRESHOLD_EVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_SNR_THRESHOLD_EVENTIDtsiestruct BG_WMI,
				  snr threst atr->cac_indGfck_&&
	pe,f *pval	brea_wmtcaoldica_LQ_THRESHOLD_EVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_LQ_THRESHOLD_EVENTIDtsiestrubrea_wmtcaoldica_APLISSpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_APLISSpEVENTIDtsiestruct BG_WMI,
				  apwmi_-->cac_indGfck_&&
	pe,f *pval	brea_wmtcaoldica_G thKEEPALIVkb,[2]):
ruinterval);

	ath6kl_dbg(AT "ica_G thKEEPALIVkb,[2])tsiestruct BG_WMI,
				  keeprepvdireply_indGfck_&&
	pe,f *pval	brea_wmtcaoldica_G thWOWlLISSpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_G thWOWlLISSpEVENTIDtsiestrubrea_wmtcaoldica_G thPMKIDtLISSpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_G thPMKIDtLISSpEVENTIDtsiestruct BG_WMI,
				   (enpmk- iwmi_-->cac_indGfck_&&
	pe,f *pval	brea_wmtcaoldica_e thPARAMS_REPLYpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_e thPARAMS_REPLYpEVENTIDtsiestrubrea_wmtcaoldica_ADDBA_RESPpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_ADDBA_RESPpEVENTIDtsiestrubrea_wmtcaoldica_REPORTbBTCOEX_CONFIG_EVENTID:
ruinterval);

	ath6kl_dbg(ATH6KLv->kWcatREPORTbBTCOEX_CONFIG_EVENTIDtsiestrubrea_wmtcaoldica_REPORTbBTCOEX_sTATSpEVENTID:
ruinterval);

	ath6kl_dbg(ATH6KLv->kWcatREPORTbBTCOEX_sTATSpEVENTIDtsiestrubrea_wmtcaoldica_TX_COMPWMI,lEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_TX_COMPWMI,lEVENTIDtsiestruct BG_WMI,
				  tx_/
	ps))
	->cac_ind&&
	pe,f *pval	brea_wmtcaoldica_P2P = (structIESpEVENTID:
ruinterval);

	ath6kl_dbg(AT "ica_P2P = (structIESpEVENTIDtsiestruct BG_WMI,
				  p2pa;
	cmd->rssi	->cac_ind&&
	pe,f *pval	brea_wmtcaoldica_P2P INFObEVENTID:
ruinterval);

	ath