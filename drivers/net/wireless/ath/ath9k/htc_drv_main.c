/*
 * Copyright (c) 2010-2011 Atheros Communications Inc.
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

#include "htc.h"

/*************/
/* Utilities */
/*************/

/* HACK Alert: Use 11NG for 2.4, use 11NA for 5 */
static enum htc_phymode ath9k_htc_get_curmode(struct ath9k_htc_priv *priv,
					      struct ath9k_channel *ichan)
{
	if (IS_CHAN_5GHZ(ichan))
		return HTC_MODE_11NA;

	return HTC_MODE_11NG;
}

bool ath9k_htc_setpower(struct ath9k_htc_priv *priv,
			enum ath9k_power_mode mode)
{
	bool ret;

	mutex_lock(&priv->htc_pm_lock);
	ret = ath9k_hw_setpower(priv->ah, mode);
	mutex_unlock(&priv->htc_pm_lock);

	return ret;
}

void ath9k_htc_ps_wakeup(struct ath9k_htc_priv *priv)
{
	mutex_lock(&priv->htc_pm_lock);
	if (++priv->ps_usecount != 1)
		goto unlock;
	ath9k_hw_setpower(priv->ah, ATH9K_PM_AWAKE);

unlock:
	mutex_unlock(&priv->htc_pm_lock);
}

void ath9k_htc_ps_restore(struct ath9k_htc_priv *priv)
{
	bool reset;

	mutex_lock(&priv->htc_pm_lock);
	if (--priv->ps_usecount != 0)
		goto unlock;

	if (priv->ps_idle) {
		ath9k_hw_setrxabort(priv->ah, true);
		ath9k_hw_stopdmarecv(priv->ah, &reset);
		ath9k_hw_setpower(priv->ah, ATH9K_PM_FULL_SLEEP);
	} else if (priv->ps_enabled) {
		ath9k_hw_setpower(priv->ah, ATH9K_PM_NETWORK_SLEEP);
	}

unlock:
	mutex_unlock(&priv->htc_pm_lock);
}

void ath9k_ps_work(struct work_struct *work)
{
	struct ath9k_htc_priv *priv =
		container_of(work, struct ath9k_htc_priv,
			     ps_work);
	ath9k_htc_setpower(priv, ATH9K_PM_AWAKE);

	/* The chip wakes up after receiving the first beacon
	   while network sleep is enabled. For the driver to
	   be in sync with the hw, set the chip to awake and
	   only then set it to sleep.
	 */
	ath9k_htc_setpower(priv, ATH9K_PM_NETWORK_SLEEP);
}

static void ath9k_htc_vif_iter(void *data, u8 *mac, struct ieee80211_vif *vif)
{
	struct ath9k_htc_priv *priv = data;
	struct ieee80211_bss_conf *bss_conf = &vif->bss_conf;

	if ((vif->type == NL80211_IFTYPE_AP ||
	     vif->type == NL80211_IFTYPE_MESH_POINT) &&
	    bss_conf->enable_beacon) {
		priv->reconfig_beacon = true;
		priv->rearm_ani = true;
	}

	if (bss_conf->assoc) {
		priv->rearm_ani = true;
		priv->reconfig_beacon = true;
	}
}

static void ath9k_htc_vif_reconfig(struct ath9k_htc_priv *priv)
{
	priv->rearm_ani = false;
	priv->reconfig_beacon = false;

	ieee80211_iterate_active_interfaces_atomic(
		priv->hw, IEEE80211_IFACE_ITER_RESUME_ALL,
		ath9k_htc_vif_iter, priv);
	if (priv->rearm_ani)
		ath9k_htc_start_ani(priv);

	if (priv->reconfig_beacon) {
		ath9k_htc_ps_wakeup(priv);
		ath9k_htc_beacon_reconfig(priv);
		ath9k_htc_ps_restore(priv);
	}
}

static void ath9k_htc_bssid_iter(void *data, u8 *mac, struct ieee80211_vif *vif)
{
	struct ath9k_vif_iter_data *iter_data = data;
	int i;

	if (iter_data->hw_macaddr != NULL) {
		for (i = 0; i < ETH_ALEN; i++)
			iter_data->mask[i] &= ~(iter_data->hw_macaddr[i] ^ mac[i]);
	} else {
		iter_data->hw_macaddr = mac;
	}
}

static void ath9k_htc_set_mac_bssid_mask(struct ath9k_htc_priv *priv,
				     struct ieee80211_vif *vif)
{
	struct ath_common *common = ath9k_hw_common(priv->ah);
	struct ath9k_vif_iter_data iter_data;

	/*
	 * Pick the MAC address of the first interface as the new hardware
	 * MAC address. The hardware will use it together with the BSSID mask
	 * when matching addresses.
	 */
	iter_data.hw_macaddr = NULL;
	eth_broadcast_addr(iter_data.mask);

	if (vif)
		ath9k_htc_bssid_iter(&iter_data, vif->addr, vif);

	/* Get list of all active MAC addresses */
	ieee80211_iterate_active_interfaces_atomic(
		priv->hw, IEEE80211_IFACE_ITER_RESUME_ALL,
		ath9k_htc_bssid_iter, &iter_data);

	memcpy(common->bssidmask, iter_data.mask, ETH_ALEN);

	if (iter_data.hw_macaddr)
		memcpy(common->macaddr, iter_data.hw_macaddr, ETH_ALEN);

	ath_hw_setbssidmask(common);
}

static void ath9k_htc_set_opmode(struct ath9k_htc_priv *priv)
{
	if (priv->num_ibss_vif)
		priv->ah->opmode = NL80211_IFTYPE_ADHOC;
	else if (priv->num_ap_vif)
		priv->ah->opmode = NL80211_IFTYPE_AP;
	else if (priv->num_mbss_vif)
		priv->ah->opmode = NL80211_IFTYPE_MESH_POINT;
	else
		priv->ah->opmode = NL80211_IFTYPE_STATION;

	ath9k_hw_setopmode(priv->ah);
}

void ath9k_htc_reset(struct ath9k_htc_priv *priv)
{
	struct ath_hw *ah = priv->ah;
	struct ath_common *common = ath9k_hw_common(ah);
	struct ieee80211_channel *channel = priv->hw->conf.chandef.chan;
	struct ath9k_hw_cal_data *caldata = NULL;
	enum htc_phymode mode;
	__be16 htc_mode;
	u8 cmd_rsp;
	int ret;

	mutex_lock(&priv->mutex);
	ath9k_htc_ps_wakeup(priv);

	ath9k_htc_stop_ani(priv);
	ieee80211_stop_queues(priv->hw);

	del_timer_sync(&priv->tx.cleanup_timer);
	ath9k_htc_tx_drain(priv);

	WMI_CMD(WMI_DISABLE_INTR_CMDID);
	WMI_CMD(WMI_DRAIN_TXQ_ALL_CMDID);
	WMI_CMD(WMI_STOP_RECV_CMDID);

	ath9k_wmi_event_drain(priv);

	caldata = &priv->caldata;
	ret = ath9k_hw_reset(ah, ah->curchan, caldata, false);
	if (ret) {
		ath_err(common,
			"Unable to reset device (%u Mhz) reset status %d\n",
			channel->center_freq, ret);
	}

	ath9k_cmn_update_txpow(ah, priv->curtxpow, priv->txpowlimit,
			       &priv->curtxpow);

	WMI_CMD(WMI_START_RECV_CMDID);
	ath9k_host_rx_init(priv);

	mode = ath9k_htc_get_curmode(priv, ah->curchan);
	htc_mode = cpu_to_be16(mode);
	WMI_CMD_BUF(WMI_SET_MODE_CMDID, &htc_mode);

	WMI_CMD(WMI_ENABLE_INTR_CMDID);
	htc_start(priv->htc);
	ath9k_htc_vif_reconfig(priv);
	ieee80211_wake_queues(priv->hw);

	mod_timer(&priv->tx.cleanup_timer,
		  jiffies + msecs_to_jiffies(ATH9K_HTC_TX_CLEANUP_INTERVAL));

	ath9k_htc_ps_restore(priv);
	mutex_unlock(&priv->mutex);
}

static int ath9k_htc_set_channel(struct ath9k_htc_priv *priv,
				 struct ieee80211_hw *hw,
				 struct ath9k_channel *hchan)
{
	struct ath_hw *ah = priv->ah;
	struct ath_common *common = ath9k_hw_common(ah);
	struct ieee80211_conf *conf = &common->hw->conf;
	bool fastcc;
	struct ieee80211_channel *channel = hw->conf.chandef.chan;
	struct ath9k_hw_cal_data *caldata;
	enum htc_phymode mode;
	__be16 htc_mode;
	u8 cmd_rsp;
	int ret;

	if (test_bit(ATH_OP_INVALID, &common->op_flags))
		return -EIO;

	fastcc = !!(hw->conf.flags & IEEE80211_CONF_OFFCHANNEL);

	ath9k_htc_ps_wakeup(priv);

	ath9k_htc_stop_ani(priv);
	del_timer_sync(&priv->tx.cleanup_timer);
	ath9k_htc_tx_drain(priv);

	WMI_CMD(WMI_DISABLE_INTR_CMDID);
	WMI_CMD(WMI_DRAIN_TXQ_ALL_CMDID);
	WMI_CMD(WMI_STOP_RECV_CMDID);

	ath9k_wmi_event_drain(priv);

	ath_dbg(common, CONFIG,
		"(%u MHz) -> (%u MHz), HT: %d, HT40: %d fastcc: %d\n",
		priv->ah->curchan->channel,
		channel->center_freq, conf_is_ht(conf), conf_is_ht40(conf),
		fastcc);
	caldata = fastcc ? NULL : &priv->caldata;
	ret = ath9k_hw_reset(ah, hchan, caldata, fastcc);
	if (ret) {
		ath_err(common,
			"Unable to reset channel (%u Mhz) reset status %d\n",
			channel->center_freq, ret);
		goto err;
	}

	ath9k_cmn_update_txpow(ah, priv->curtxpow, priv->txpowlimit,
			       &priv->curtxpow);

	WMI_CMD(WMI_START_RECV_CMDID);
	if (ret)
		goto err;

	ath9k_host_rx_init(priv);

	mode = ath9k_htc_get_curmode(priv, hchan);
	htc_mode = cpu_to_be16(mode);
	WMI_CMD_BUF(WMI_SET_MODE_CMDID, &htc_mode);
	if (ret)
		goto err;

	WMI_CMD(WMI_ENABLE_INTR_CMDID);
	if (ret)
		goto err;

	htc_start(priv->htc);

	if (!test_bit(ATH_OP_SCANNING, &common->op_flags) &&
	    !(hw->conf.flags & IEEE80211_CONF_OFFCHANNEL))
		ath9k_htc_vif_reconfig(priv);

	mod_timer(&priv->tx.cleanup_timer,
		  jiffies + msecs_to_jiffies(ATH9K_HTC_TX_CLEANUP_INTERVAL));

	/* perform spectral scan if requested. */
	if (test_bit(ATH_OP_SCANNING, &common->op_flags) &&
		     priv->spec_priv.spectral_mode == SPECTRAL_CHANSCAN)
		ath9k_cmn_spectral_scan_trigger(common, &priv->spec_priv);
err:
	ath9k_htc_ps_restore(priv);
	return ret;
}

/*
 * Monitor mode handling is a tad complicated because the firmware requires
 * an interface to be created exclusively, while mac80211 doesn't associate
 * an interface with the mode.
 *
 * So, for now, only one monitor interface can be configured.
 */
static void __ath9k_htc_remove_monitor_interface(struct ath9k_htc_priv *priv)
{
	struct ath_common *common = ath9k_hw_common(priv->ah);
	struct ath9k_htc_target_vif hvif;
	int ret = 0;
	u8 cmd_rsp;

	memset(&hvif, 0, sizeof(struct ath9k_htc_target_vif));
	memcpy(&hvif.myaddr, common->macaddr, ETH_ALEN);
	hvif.index = priv->mon_vif_idx;
	WMI_CMD_BUF(WMI_VAP_REMOVE_CMDID, &hvif);
	if (ret) {
		ath_err(common, "Unable to remove monitor interface at idx: %d\n",
			priv->mon_vif_idx);
	}

	priv->nvifs--;
	priv->vif_slot &= ~(1 << priv->mon_vif_idx);
}

static int ath9k_htc_add_monitor_interface(struct ath9k_htc_priv *priv)
{
	struct ath_common *common = ath9k_hw_common(priv->ah);
	struct ath9k_htc_target_vif hvif;
	struct ath9k_htc_target_sta tsta;
	int ret = 0, sta_idx;
	u8 cmd_rsp;

	if ((priv->nvifs >= ATH9K_HTC_MAX_VIF) ||
	    (priv->nstations >= ATH9K_HTC_MAX_STA)) {
		ret = -ENOBUFS;
		goto err_vif;
	}

	sta_idx = ffz(priv->sta_slot);
	if ((sta_idx < 0) || (sta_idx > ATH9K_HTC_MAX_STA)) {
		ret = -ENOBUFS;
		goto err_vif;
	}

	/*
	 * Add an interface.
	 */
	memset(&hvif, 0, sizeof(struct ath9k_htc_target_vif));
	memcpy(&hvif.myaddr, common->macaddr, ETH_ALEN);

	hvif.opmode = HTC_M_MONITOR;
	hvif.index = ffz(priv->vif_slot);

	WMI_CMD_BUF(WMI_VAP_CREATE_CMDID, &hvif);
	if (ret)
		goto err_vif;

	/*
	 * Assign the monitor interface index as a special case here.
	 * This is needed when the interface is brought down.
	 */
	priv->mon_vif_idx = hvif.index;
	priv->vif_slot |= (1 << hvif.index);

	/*
	 * Set the hardware mode to monitor only if there are no
	 * other interfaces.
	 */
	if (!priv->nvifs)
		priv->ah->opmode = NL80211_IFTYPE_MONITOR;

	priv->nvifs++;

	/*
	 * Associate a station with the interface for packet injection.
	 */
	memset(&tsta, 0, sizeof(struct ath9k_htc_target_sta));

	memcpy(&tsta.macaddr, common->macaddr, ETH_ALEN);

	tsta.is_vif_sta = 1;
	tsta.sta_index = sta_idx;
	tsta.vif_index = hvif.index;
	tsta.maxampdu = cpu_to_be16(0xffff);

	WMI_CMD_BUF(WMI_NODE_CREATE_CMDID, &tsta);
	if (ret) {
		ath_err(common, "Unable to add station entry for monitor mode\n");
		goto err_sta;
	}

	priv->sta_slot |= (1 << sta_idx);
	priv->nstations++;
	priv->vif_sta_pos[priv->mon_vif_idx] = sta_idx;
	priv->ah->is_monitoring = true;

	ath_dbg(common, CONFIG,
		"Attached a monitor interface at idx: %d, sta idx: %d\n",
		priv->mon_vif_idx, sta_idx);

	return 0;

err_sta:
	/*
	 * Remove the interface from the target.
	 */
	__ath9k_htc_remove_monitor_interface(priv);
err_vif:
	ath_dbg(common, FATAL, "Unable to attach a monitor interface\n");

	return ret;
}

static int ath9k_htc_remove_monitor_interface(struct ath9k_htc_priv *priv)
{
	struct ath_common *common = ath9k_hw_common(priv->ah);
	int ret = 0;
	u8 cmd_rsp, sta_idx;

	__ath9k_htc_remove_monitor_interface(priv);

	sta_idx = priv->vif_sta_pos[priv->mon_vif_idx];

	WMI_CMD_BUF(WMI_NODE_REMOVE_CMDID, &sta_idx);
	if (ret) {
		ath_err(common, "Unable to remove station entry for monitor mode\n");
		return ret;
	}

	priv->sta_slot &= ~(1 << sta_idx);
	priv->nstations--;
	priv->ah->is_monitoring = false;

	ath_dbg(common, CONFIG,
		"Removed a monitor interface at idx: %d, sta idx: %d\n",
		priv->mon_vif_idx, sta_idx);

	return 0;
}

static int ath9k_htc_add_station(struct ath9k_htc_priv *priv,
				 struct ieee80211_vif *vif,
				 struct ieee80211_sta *sta)
{
	struct ath_common *common = ath9k_hw_common(priv->ah);
	struct ath9k_htc_target_sta tsta;
	struct ath9k_htc_vif *avp = (struct ath9k_htc_vif *) vif->drv_priv;
	struct ath9k_htc_sta *ista;
	int ret, sta_idx;
	u8 cmd_rsp;
	u16 maxampdu;

	if (priv->nstations >= ATH9K_HTC_MAX_STA)
		return -ENOBUFS;

	sta_idx = ffz(priv->sta_slot);
	if ((sta_idx < 0) || (sta_idx > ATH9K_HTC_MAX_STA))
		return -ENOBUFS;

	memset(&tsta, 0, sizeof(struct ath9k_htc_target_sta));

	if (sta) {
		ista = (struct ath9k_htc_sta *) sta->drv_priv;
		memcpy(&tsta.macaddr, sta->addr, ETH_ALEN);
		memcpy(&tsta.bssid, common->curbssid, ETH_ALEN);
		ista->index = sta_idx;
		tsta.is_vif_sta = 0;
		maxampdu = 1 << (IEEE80211_HT_MAX_AMPDU_FACTOR +
				 sta->ht_cap.ampdu_factor);
		tsta.maxampdu = cpu_to_be16(maxampdu);
	} else {
		memcpy(&tsta.macaddr, vif->addr, ETH_ALEN);
		tsta.is_vif_sta = 1;
		tsta.maxampdu = cpu_to_be16(0xffff);
	}

	tsta.sta_index = sta_idx;
	tsta.vif_index = avp->index;

	WMI_CMD_BUF(WMI_NODE_CREATE_CMDID, &tsta);
	if (ret) {
		if (sta)
			ath_err(common,
				"Unable to add station entry for: %pM\n",
				sta->addr);
		return ret;
	}

	if (sta) {
		ath_dbg(common, CONFIG,
			"Added a station entry for: %pM (idx: %d)\n",
			sta->addr, tsta.sta_index);
	} else {
		ath_dbg(common, CONFIG,
			"Added a station entry for VIF %d (idx: %d)\n",
			avp->index, tsta.sta_index);
	}

	priv->sta_slot |= (1 << sta_idx);
	priv->nstations++;
	if (!sta)
		priv->vif_sta_pos[avp->index] = sta_idx;

	return 0;
}

static int ath9k_htc_remove_station(struct ath9k_htc_priv *priv,
				    struct ieee80211_vif *vif,
				    struct ieee80211_sta *sta)
{
	struct ath_common *common = ath9k_hw_common(priv->ah);
	struct ath9k_htc_vif *avp = (struct ath9k_htc_vif *) vif->drv_priv;
	struct ath9k_htc_sta *ista;
	int ret;
	u8 cmd_rsp, sta_idx;

	if (sta) {
		ista = (struct ath9k_htc_sta *) sta->drv_priv;
		sta_idx = ista->index;
	} else {
		sta_idx = priv->vif_sta_pos[avp->index];
	}

	WMI_CMD_BUF(WMI_NODE_REMOVE_CMDID, &sta_idx);
	if (ret) {
		if (sta)
			ath_err(common,
				"Unable to remove station entry for: %pM\n",
				sta->addr);
		return ret;
	}

	if (sta) {
		ath_dbg(common, CONFIG,
			"Removed a station entry for: %pM (idx: %d)\n",
			sta->addr, sta_idx);
	} else {
		ath_dbg(common, CONFIG,
			"Removed a station entry for VIF %d (idx: %d)\n",
			avp->index, sta_idx);
	}

	priv->sta_slot &= ~(1 << sta_idx);
	priv->nstations--;

	return 0;
}

int ath9k_htc_update_cap_target(struct ath9k_htc_priv *priv,
				u8 enable_coex)
{
	struct ath9k_htc_cap_target tcap;
	int ret;
	u8 cmd_rsp;

	memset(&tcap, 0, sizeof(struct ath9k_htc_cap_target));

	tcap.ampdu_limit = cpu_to_be32(0xffff);
	tcap.ampdu_subframes = 0xff;
	tcap.enable_coex = enable_coex;
	tcap.tx_chainmask = priv->ah->caps.tx_chainmask;

	WMI_CMD_BUF(WMI_TARGET_IC_UPDATE_CMDID, &tcap);

	return ret;
}

static void ath9k_htc_setup_rate(struct ath9k_htc_priv *priv,
				 struct ieee80211_sta *sta,
				 struct ath9k_htc_target_rate *trate)
{
	struct ath9k_htc_sta *ista = (struct ath9k_htc_sta *) sta->drv_priv;
	struct ieee80211_supported_band *sband;
	u32 caps = 0;
	int i, j;

	sband = priv->hw->wiphy->bands[priv->hw->conf.chandef.chan->band];

	for (i = 0, j = 0; i < sband->n_bitrates; i++) {
		if (sta->supp_rates[sband->band] & BIT(i)) {
			trate->rates.legacy_rates.rs_rates[j]
				= (sband->bitrates[i].bitrate * 2) / 10;
			j++;
		}
	}
	trate->rates.legacy_rates.rs_nrates = j;

	if (sta->ht_cap.ht_supported) {
		for (i = 0, j = 0; i < 77; i++) {
			if (sta->ht_cap.mcs.rx_mask[i/8] & (1<<(i%8)))
				trate->rates.ht_rates.rs_rates[j++] = i;
			if (j == ATH_HTC_RATE_MAX)
				break;
		}
		trate->rates.ht_rates.rs_nrates = j;

		caps = WLAN_RC_HT_FLAG;
		if (sta->ht_cap.cap & IEEE80211_HT_CAP_RX_STBC)
			caps |= ATH_RC_TX_STBC_FLAG;
		if (sta->ht_cap.mcs.rx_mask[1])
			caps |= WLAN_RC_DS_FLAG;
		if ((sta->ht_cap.cap & IEEE80211_HT_CAP_SUP_WIDTH_20_40) &&
		     (conf_is_ht40(&priv->hw->conf)))
			caps |= WLAN_RC_40_FLAG;
		if (conf_is_ht40(&priv->hw->conf) &&
		    (sta->ht_cap.cap & IEEE80211_HT_CAP_SGI_40))
			caps |= WLAN_RC_SGI_FLAG;
		else if (conf_is_ht20(&priv->hw->conf) &&
			 (sta->ht_cap.cap & IEEE80211_HT_CAP_SGI_20))
			caps |= WLAN_RC_SGI_FLAG;
	}

	trate->sta_index = ista->index;
	trate->isnew = 1;
	trate->capflags = cpu_to_be32(caps);
}

static int ath9k_htc_send_rate_cmd(struct ath9k_htc_priv *priv,
				    struct ath9k_htc_target_rate *trate)
{
	struct ath_common *common = ath9k_hw_common(priv->ah);
	int ret;
	u8 cmd_rsp;

	WMI_CMD_BUF(WMI_RC_RATE_UPDATE_CMDID, trate);
	if (ret) {
		ath_err(common,
			"Unable to initialize Rate information on target\n");
	}

	return ret;
}

static void ath9k_htc_init_rate(struct ath9k_htc_priv *priv,
				struct ieee80211_sta *sta)
{
	struct ath_common *common = ath9k_hw_common(priv->ah);
	struct ath9k_htc_target_rate trate;
	int ret;

	memset(&trate, 0, sizeof(struct ath9k_htc_target_rate));
	ath9k_htc_setup_rate(priv, sta, &trate);
	ret = ath9k_htc_send_rate_cmd(priv, &trate);
	if (!ret)
		ath_dbg(common, CONFIG,
			"Updated target sta: %pM, rate caps: 0x%X\n",
			sta->addr, be32_to_cpu(trate.capflags));
}

static void ath9k_htc_update_rate(struct ath9k_htc_priv *priv,
				  struct ieee80211_vif *vif,
				  struct ieee80211_bss_conf *bss_conf)
{
	struct ath_common *common = ath9k_hw_common(priv->ah);
	struct ath9k_htc_target_rate trate;
	struct ieee80211_sta *sta;
	int ret;

	memset(&trate, 0, sizeof(struct ath9k_htc_target_rate));

	rcu_read_lock();
	sta = ieee80211_find_sta(vif, bss_conf->bssid);
	if (!sta) {
		rcu_read_unlock();
		return;
	}
	ath9k_htc_setup_rate(priv, sta, &trate);
	rcu_read_unlock();

	ret = ath9k_htc_send_rate_cmd(priv, &trate);
	if (!ret)
		ath_dbg(common, CONFIG,
			"Updated target sta: %pM, rate caps: 0x%X\n",
			bss_conf->bssid, be32_to_cpu(trate.capflags));
}

static int ath9k_htc_tx_aggr_oper(struct ath9k_htc_priv *priv,
				  struct ieee80211_vif *vif,
				  struct ieee80211_sta *sta,
				  enum ieee80211_ampdu_mlme_action action,
				  u16 tid)
{
	struct ath_common *common = ath9k_hw_common(priv->ah);
	struct ath9k_htc_target_aggr aggr;
	struct ath9k_htc_sta *ista;
	int ret = 0;
	u8 cmd_rsp;

	if (tid >= ATH9K_HTC_MAX_TID)
		return -EINVAL;

	memset(&aggr, 0, sizeof(struct ath9k_htc_target_aggr));
	ista = (struct ath9k_htc_sta *) sta->drv_priv;

	aggr.sta_index = ista->index;
	aggr.tidno = tid & 0xf;
	aggr.aggr_enable = (action == IEEE80211_AMPDU_TX_START) ? true : false;

	WMI_CMD_BUF(WMI_TX_AGGR_ENABLE_CMDID, &aggr);
	if (ret)
		ath_dbg(common, CONFIG,
			"Unable to %s TX aggregation for (%pM, %d)\n",
			(aggr.aggr_enable) ? "start" : "stop", sta->addr, tid);
	else
		ath_dbg(common, CONFIG,
			"%s TX aggregation for (%pM, %d)\n",
			(aggr.aggr_enable) ? "Starting" : "Stopping",
			sta->addr, tid);

	spin_lock_bh(&priv->tx.tx_lock);
	ista->tid_state[tid] = (aggr.aggr_enable && !ret) ? AGGR_START : AGGR_STOP;
	spin_unlock_bh(&priv->tx.tx_lock);

	return ret;
}

/*******/
/* ANI */
/*******/

void ath9k_htc_start_ani(struct ath9k_htc_priv *priv)
{
	struct ath_common *common = ath9k_hw_common(priv->ah);
	unsigned long timestamp = jiffies_to_msecs(jiffies);

	common->ani.longcal_timer = timestamp;
	common->ani.shortcal_timer = timestamp;
	common->ani.checkani_timer = timestamp;

	set_bit(ATH_OP_ANI_RUN, &common->op_flags);

	ieee80211_queue_delayed_work(common->hw, &priv->ani_work,
				     msecs_to_jiffies(ATH_ANI_POLLINTERVAL));
}

void ath9k_htc_stop_ani(struct ath9k_htc_priv *priv)
{
	struct ath_common *common = ath9k_hw_common(priv->ah);
	cancel_delayed_work_sync(&priv->ani_work);
	clear_bit(ATH_OP_ANI_RUN, &common->op_flags);
}

void ath9k_htc_ani_work(struct work_struct *work)
{
	struct ath9k_htc_priv *priv =
		container_of(work, struct ath9k_htc_priv, ani_work.work);
	struct ath_hw *ah = priv->ah;
	struct ath_common *common = ath9k_hw_common(ah);
	bool longcal = false;
	bool shortcal = false;
	bool aniflag = false;
	unsigned int timestamp = jiffies_to_msecs(jiffies);
	u32 cal_interval, short_cal_interval;

	short_cal_interval = (ah->opmode == NL80211_IFTYPE_AP) ?
		ATH_AP_SHORT_CALINTERVAL : ATH_STA_SHORT_CALINTERVAL;

	/* Only calibrate if awake */
	if (ah->power_mode != ATH9K_PM_AWAKE)
		goto set_timer;

	/* Long calibration runs independently of short calibration. */
	if ((timestamp - common->ani.longcal_timer) >= ATH_LONG_CALINTERVAL) {
		longcal = true;
		ath_dbg(common, ANI, "longcal @%lu\n", jiffies);
		common->ani.longcal_timer = timestamp;
	}

	/*
	 * Short calibration applies only while caldone
	 * is false or -ETIMEDOUT
	 */
	if (common->ani.caldone <= 0) {
		if ((timestamp - common->ani.shortcal_timer) >=
		    short_cal_interval) {
			shortcal = true;
			ath_dbg(common, ANI, "shortcal @%lu\n", jiffies);
			common->ani.shortcal_timer = timestamp;
			common->ani.resetcal_timer = timestamp;
		}
	} else {
		if ((timestamp - common->ani.resetcal_timer) >=
		    ATH_RESTART_CALINTERVAL) {
			common->ani.caldone = ath9k_hw_reset_calvalid(ah);
			if (common->ani.caldone)
				common->ani.resetcal_timer = timestamp;
		}
	}

	/* Verify whether we must check ANI */
	if ((timestamp - common->ani.checkani_timer) >= ATH_ANI_POLLINTERVAL) {
		aniflag = true;
		common->ani.checkani_timer = timestamp;
	}

	/* Skip all processing if there's nothing to do. */
	if (longcal || shortcal || aniflag) {

		ath9k_htc_ps_wakeup(priv);

		/* Call ANI routine if necessary */
		if (aniflag)
			ath9k_hw_ani_monitor(ah, ah->curchan);

		/* Perform calibration if necessary */
		if (longcal || shortcal)
			common->ani.caldone =
				ath9k_hw_calibrate(ah, ah->curchan,
						ah->rxchainmask, longcal) > 0;

		ath9k_htc_ps_restore(priv);
	}

set_timer:
	/*
	* Set timer interval based on previous results.
	* The interval must be the shortest necessary to satisfy ANI,
	* short calibration and long calibration.
	*/
	cal_interval = ATH_LONG_CALINTERVAL;
	cal_interval = min(cal_interval, (u32)ATH_ANI_POLLINTERVAL);
	/*
	 * Short calibration applies only while caldone
	 * is false or -ETIMEDOUT
	 */
	if (common->ani.caldone <= 0)
		cal_interval = min(cal_interval, (u32)short_cal_interval);

	ieee80211_queue_delayed_work(common->hw, &priv->ani_work,
				     msecs_to_jiffies(cal_interval));
}

/**********************/
/* mac80211 Callbacks */
/**********************/

static void ath9k_htc_tx(struct ieee80211_hw *hw,
			 struct ieee80211_tx_control *control,
			 struct sk_buff *skb)
{
	struct ieee80211_hdr *hdr;
	struct ath9k_htc_priv *priv = hw->priv;
	struct ath_common *common = ath9k_hw_common(priv->ah);
	int padpos, padsize, ret, slot;

	hdr = (struct ieee80211_hdr *) skb->data;

	/* Add the padding after the header if this is not already done */
	padpos = ieee80211_hdrlen(hdr->frame_control);
	padsize = padpos & 3;
	if (padsize && skb->len > padpos) {
		if (skb_headroom(skb) < padsize) {
			ath_dbg(common, XMIT, "No room for padding\n");
			goto fail_tx;
		}
		skb_push(skb, padsize);
		memmove(skb->data, skb->data + padsize, padpos);
	}

	slot = ath9k_htc_tx_get_slot(priv);
	if (slot < 0) {
		ath_dbg(common, XMIT, "No free TX slot\n");
		goto fail_tx;
	}

	ret = ath9k_htc_tx_start(priv, control->sta, skb, slot, false);
	if (ret != 0) {
		ath_dbg(common, XMIT, "Tx failed\n");
		goto clear_slot;
	}

	ath9k_htc_check_stop_queues(priv);

	return;

clear_slot:
	ath9k_htc_tx_clear_slot(priv, slot);
fail_tx:
	dev_kfree_skb_any(skb);
}

static int ath9k_htc_start(struct ieee80211_hw *hw)
{
	struct ath9k_htc_priv *priv = hw->priv;
	struct ath_hw *ah = priv->ah;
	struct ath_common *common = ath9k_hw_common(ah);
	struct ieee80211_channel *curchan = hw->conf.chandef.chan;
	struct ath9k_channel *iFLAGI%ion applies only while caldone
	 * is false or -ETIMEDOUT
	 */
	if (common->ani.caldone <= 0) {
		if ((timestamp -dAB>yilen > padpos) {
		if (skb_headroom(skb) < padsize) {
			atdsize, padpos);
	}

	slot = ath9k_htc_tx_get_slot(priv);
	
	struct ath9k_s=	s);
n>s);
n>s);
n>s);
n>s);
n>s);
n>s);
n>s);
n>s);
n>s);
n>s)I%atic int ath9k_htc_tx_aggr_oper(struct ath9k_htc_priv *priv,
				  struct ieee80211_vif *vif,
				  struct ieee802ABn>02ABn>02ABn>02ABn>02ABn>02ABn>02ABn>02ABn>02ABn>02ABn>02I%_delayed_work(common->hw, &priv->ani_work,
				     msecs_to_jiffies(cal_interval));
}

/**********************/
/N>yathc

	s^l%1_tx_control *control,
			 struct sk_buff *skb)
{
	struct ieee80211_hdr *hdr;
	struct ath9k_htc_priv *priv = hw->pet)
		atl^l2V_target_aggr));
	ista = (struct ath9k_htc_sta *) sta->drv_priv;

	aggr.sta_index = ista->index;
	aggr.tidno = tid ABn>vxf;
cBn>yddr, commo>
N>mo>
N>mo>
N>mo>
N>mo>
N>mo>
N>mo>
N>mo>
N>mo>
N>mo>
N>moICr