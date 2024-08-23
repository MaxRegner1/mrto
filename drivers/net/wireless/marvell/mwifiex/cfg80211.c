/*
 * Marvell Wireless LAN device driver: CFG80211
 *
 * Copyright (C) 2011-2014, Marvell International Ltd.
 *
 * This software file (the "File") is distributed by Marvell International
 * Ltd. under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#include "cfg80211.h"
#include "main.h"
#include "11n.h"
#include "wmm.h"

static char *reg_alpha2;
module_param(reg_alpha2, charp, 0);

static const struct ieee80211_iface_limit mwifiex_ap_sta_limits[] = {
	{
		.max = 3, .types = BIT(NL80211_IFTYPE_STATION) |
				   BIT(NL80211_IFTYPE_P2P_GO) |
				   BIT(NL80211_IFTYPE_P2P_CLIENT) |
				   BIT(NL80211_IFTYPE_AP),
	},
};

static const struct ieee80211_iface_combination
mwifiex_iface_comb_ap_sta = {
	.limits = mwifiex_ap_sta_limits,
	.num_different_channels = 1,
	.n_limits = ARRAY_SIZE(mwifiex_ap_sta_limits),
	.max_interfaces = MWIFIEX_MAX_BSS_NUM,
	.beacon_int_infra_match = true,
	.radar_detect_widths =	BIT(NL80211_CHAN_WIDTH_20_NOHT) |
				BIT(NL80211_CHAN_WIDTH_20) |
				BIT(NL80211_CHAN_WIDTH_40),
};

static const struct ieee80211_iface_combination
mwifiex_iface_comb_ap_sta_vht = {
	.limits = mwifiex_ap_sta_limits,
	.num_different_channels = 1,
	.n_limits = ARRAY_SIZE(mwifiex_ap_sta_limits),
	.max_interfaces = MWIFIEX_MAX_BSS_NUM,
	.beacon_int_infra_match = true,
	.radar_detect_widths =	BIT(NL80211_CHAN_WIDTH_20_NOHT) |
				BIT(NL80211_CHAN_WIDTH_20) |
				BIT(NL80211_CHAN_WIDTH_40) |
				BIT(NL80211_CHAN_WIDTH_80),
};

static const struct
ieee80211_iface_combination mwifiex_iface_comb_ap_sta_drcs = {
	.limits = mwifiex_ap_sta_limits,
	.num_different_channels = 2,
	.n_limits = ARRAY_SIZE(mwifiex_ap_sta_limits),
	.max_interfaces = MWIFIEX_MAX_BSS_NUM,
	.beacon_int_infra_match = true,
};

/*
 * This function maps the nl802.11 channel type into driver channel type.
 *
 * The mapping is as follows -
 *      NL80211_CHAN_NO_HT     -> IEEE80211_HT_PARAM_CHA_SEC_NONE
 *      NL80211_CHAN_HT20      -> IEEE80211_HT_PARAM_CHA_SEC_NONE
 *      NL80211_CHAN_HT40PLUS  -> IEEE80211_HT_PARAM_CHA_SEC_ABOVE
 *      NL80211_CHAN_HT40MINUS -> IEEE80211_HT_PARAM_CHA_SEC_BELOW
 *      Others                 -> IEEE80211_HT_PARAM_CHA_SEC_NONE
 */
u8 mwifiex_chan_type_to_sec_chan_offset(enum nl80211_channel_type chan_type)
{
	switch (chan_type) {
	case NL80211_CHAN_NO_HT:
	case NL80211_CHAN_HT20:
		return IEEE80211_HT_PARAM_CHA_SEC_NONE;
	case NL80211_CHAN_HT40PLUS:
		return IEEE80211_HT_PARAM_CHA_SEC_ABOVE;
	case NL80211_CHAN_HT40MINUS:
		return IEEE80211_HT_PARAM_CHA_SEC_BELOW;
	default:
		return IEEE80211_HT_PARAM_CHA_SEC_NONE;
	}
}

/* This function maps IEEE HT secondary channel type to NL80211 channel type
 */
u8 mwifiex_sec_chan_offset_to_chan_type(u8 second_chan_offset)
{
	switch (second_chan_offset) {
	case IEEE80211_HT_PARAM_CHA_SEC_NONE:
		return NL80211_CHAN_HT20;
	case IEEE80211_HT_PARAM_CHA_SEC_ABOVE:
		return NL80211_CHAN_HT40PLUS;
	case IEEE80211_HT_PARAM_CHA_SEC_BELOW:
		return NL80211_CHAN_HT40MINUS;
	default:
		return NL80211_CHAN_HT20;
	}
}

/*
 * This function checks whether WEP is set.
 */
static int
mwifiex_is_alg_wep(u32 cipher)
{
	switch (cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
		return 1;
	default:
		break;
	}

	return 0;
}

/*
 * This function retrieves the private structure from kernel wiphy structure.
 */
static void *mwifiex_cfg80211_get_adapter(struct wiphy *wiphy)
{
	return (void *) (*(unsigned long *) wiphy_priv(wiphy));
}

/*
 * CFG802.11 operation handler to delete a network key.
 */
static int
mwifiex_cfg80211_del_key(struct wiphy *wiphy, struct net_device *netdev,
			 u8 key_index, bool pairwise, const u8 *mac_addr)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(netdev);
	const u8 bc_mac[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	const u8 *peer_mac = pairwise ? mac_addr : bc_mac;

	if (mwifiex_set_encode(priv, NULL, NULL, 0, key_index, peer_mac, 1)) {
		mwifiex_dbg(priv->adapter, ERROR, "deleting the crypto keys\n");
		return -EFAULT;
	}

	mwifiex_dbg(priv->adapter, INFO, "info: crypto keys deleted\n");
	return 0;
}

/*
 * This function forms an skb for management frame.
 */
static int
mwifiex_form_mgmt_frame(struct sk_buff *skb, const u8 *buf, size_t len)
{
	u8 addr[ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	u16 pkt_len;
	u32 tx_control = 0, pkt_type = PKT_TYPE_MGMT;

	pkt_len = len + ETH_ALEN;

	skb_reserve(skb, MWIFIEX_MIN_DATA_HEADER_LEN +
		    MWIFIEX_MGMT_FRAME_HEADER_SIZE + sizeof(pkt_len));
	memcpy(skb_push(skb, sizeof(pkt_len)), &pkt_len, sizeof(pkt_len));

	memcpy(skb_push(skb, sizeof(tx_control)),
	       &tx_control, sizeof(tx_control));

	memcpy(skb_push(skb, sizeof(pkt_type)), &pkt_type, sizeof(pkt_type));

	/* Add packet data and address4 */
	skb_put_data(skb, buf, sizeof(struct ieee80211_hdr_3addr));
	skb_put_data(skb, addr, ETH_ALEN);
	skb_put_data(skb, buf + sizeof(struct ieee80211_hdr_3addr),
		     len - sizeof(struct ieee80211_hdr_3addr));

	skb->priority = LOW_PRIO_TID;
	__net_timestamp(skb);

	return 0;
}

/*
 * CFG802.11 operation handler to transmit a management frame.
 */
static int
mwifiex_cfg80211_mgmt_tx(struct wiphy *wiphy, struct wireless_dev *wdev,
			 struct cfg80211_mgmt_tx_params *params, u64 *cookie)
{
	const u8 *buf = params->buf;
	size_t len = params->len;
	struct sk_buff *skb;
	u16 pkt_len;
	const struct ieee80211_mgmt *mgmt;
	struct mwifiex_txinfo *tx_info;
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(wdev->netdev);

	if (!buf || !len) {
		mwifiex_dbg(priv->adapter, ERROR, "invalid buffer and length\n");
		return -EFAULT;
	}

	mgmt = (const struct ieee80211_mgmt *)buf;
	if (GET_BSS_ROLE(priv) != MWIFIEX_BSS_ROLE_STA &&
	    ieee80211_is_probe_resp(mgmt->frame_control)) {
		/* Since we support offload probe resp, we need to skip probe
		 * resp in AP or GO mode */
		mwifiex_dbg(priv->adapter, INFO,
			    "info: skip to send probe resp in AP or GO mode\n");
		return 0;
	}

	pkt_len = len + ETH_ALEN;
	skb = dev_alloc_skb(MWIFIEX_MIN_DATA_HEADER_LEN +
			    MWIFIEX_MGMT_FRAME_HEADER_SIZE +
			    pkt_len + sizeof(pkt_len));

	if (!skb) {
		mwifiex_dbg(priv->adapter, ERROR,
			    "allocate skb failed for management frame\n");
		return -ENOMEM;
	}

	tx_info = MWIFIEX_SKB_TXCB(skb);
	memset(tx_info, 0, sizeof(*tx_info));
	tx_info->bss_num = priv->bss_num;
	tx_info->bss_type = priv->bss_type;
	tx_info->pkt_len = pkt_len;

	mwifiex_form_mgmt_frame(skb, buf, len);
	*cookie = prandom_u32() | 1;

	if (ieee80211_is_action(mgmt->frame_control))
		skb = mwifiex_clone_skb_for_tx_status(priv,
						      skb,
				MWIFIEX_BUF_FLAG_ACTION_TX_STATUS, cookie);
	else
		cfg80211_mgmt_tx_status(wdev, *cookie, buf, len, true,
					GFP_ATOMIC);

	mwifiex_queue_tx_pkt(priv, skb);

	mwifiex_dbg(priv->adapter, INFO, "info: management frame transmitted\n");
	return 0;
}

/*
 * CFG802.11 operation handler to register a mgmt frame.
 */
static void
mwifiex_cfg80211_mgmt_frame_register(struct wiphy *wiphy,
				     struct wireless_dev *wdev,
				     u16 frame_type, bool reg)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(wdev->netdev);
	u32 mask;

	if (reg)
		mask = priv->mgmt_frame_mask | BIT(frame_type >> 4);
	else
		mask = priv->mgmt_frame_mask & ~BIT(frame_type >> 4);

	if (mask != priv->mgmt_frame_mask) {
		priv->mgmt_frame_mask = mask;
		mwifiex_send_cmd(priv, HostCmd_CMD_MGMT_FRAME_REG,
				 HostCmd_ACT_GEN_SET, 0,
				 &priv->mgmt_frame_mask, false);
		mwifiex_dbg(priv->adapter, INFO, "info: mgmt frame registered\n");
	}
}

/*
 * CFG802.11 operation handler to remain on channel.
 */
static int
mwifiex_cfg80211_remain_on_channel(struct wiphy *wiphy,
				   struct wireless_dev *wdev,
				   struct ieee80211_channel *chan,
				   unsigned int duration, u64 *cookie)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(wdev->netdev);
	int ret;

	if (!chan || !cookie) {
		mwifiex_dbg(priv->adapter, ERROR, "Invalid parameter for ROC\n");
		return -EINVAL;
	}

	if (priv->roc_cfg.cookie) {
		mwifiex_dbg(priv->adapter, INFO,
			    "info: ongoing ROC, cookie = 0x%llx\n",
			    priv->roc_cfg.cookie);
		return -EBUSY;
	}

	ret = mwifiex_remain_on_chan_cfg(priv, HostCmd_ACT_GEN_SET, chan,
					 duration);

	if (!ret) {
		*cookie = prandom_u32() | 1;
		priv->roc_cfg.cookie = *cookie;
		priv->roc_cfg.chan = *chan;

		cfg80211_ready_on_channel(wdev, *cookie, chan,
					  duration, GFP_ATOMIC);

		mwifiex_dbg(priv->adapter, INFO,
			    "info: ROC, cookie = 0x%llx\n", *cookie);
	}

	return ret;
}

/*
 * CFG802.11 operation handler to cancel remain on channel.
 */
static int
mwifiex_cfg80211_cancel_remain_on_channel(struct wiphy *wiphy,
					  struct wireless_dev *wdev, u64 cookie)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(wdev->netdev);
	int ret;

	if (cookie != priv->roc_cfg.cookie)
		return -ENOENT;

	ret = mwifiex_remain_on_chan_cfg(priv, HostCmd_ACT_GEN_REMOVE,
					 &priv->roc_cfg.chan, 0);

	if (!ret) {
		cfg80211_remain_on_channel_expired(wdev, cookie,
						   &priv->roc_cfg.chan,
						   GFP_ATOMIC);

		memset(&priv->roc_cfg, 0, sizeof(struct mwifiex_roc_cfg));

		mwifiex_dbg(priv->adapter, INFO,
			    "info: cancel ROC, cookie = 0x%llx\n", cookie);
	}

	return ret;
}

/*
 * CFG802.11 operation handler to set Tx power.
 */
static int
mwifiex_cfg80211_set_tx_power(struct wiphy *wiphy,
			      struct wireless_dev *wdev,
			      enum nl80211_tx_power_setting type,
			      int mbm)
{
	struct mwifiex_adapter *adapter = mwifiex_cfg80211_get_adapter(wiphy);
	struct mwifiex_private *priv;
	struct mwifiex_power_cfg power_cfg;
	int dbm = MBM_TO_DBM(mbm);

	switch (type) {
	case NL80211_TX_POWER_FIXED:
		power_cfg.is_power_auto = 0;
		power_cfg.is_power_fixed = 1;
		power_cfg.power_level = dbm;
		break;
	case NL80211_TX_POWER_LIMITED:
		power_cfg.is_power_auto = 0;
		power_cfg.is_power_fixed = 0;
		power_cfg.power_level = dbm;
		break;
	case NL80211_TX_POWER_AUTOMATIC:
		power_cfg.is_power_auto = 1;
		break;
	}

	priv = mwifiex_get_priv(adapter, MWIFIEX_BSS_ROLE_ANY);

	return mwifiex_set_tx_power(priv, &power_cfg);
}

/*
 * CFG802.11 operation handler to get Tx power.
 */
static int
mwifiex_cfg80211_get_tx_power(struct wiphy *wiphy,
			      struct wireless_dev *wdev,
			      int *dbm)
{
	struct mwifiex_adapter *adapter = mwifiex_cfg80211_get_adapter(wiphy);
	struct mwifiex_private *priv = mwifiex_get_priv(adapter,
							MWIFIEX_BSS_ROLE_ANY);
	int ret = mwifiex_send_cmd(priv, HostCmd_CMD_RF_TX_PWR,
				   HostCmd_ACT_GEN_GET, 0, NULL, true);

	if (ret < 0)
		return ret;

	/* tx_power_level is set in HostCmd_CMD_RF_TX_PWR command handler */
	*dbm = priv->tx_power_level;

	return 0;
}

/*
 * CFG802.11 operation handler to set Power Save option.
 *
 * The timeout value, if provided, is currently ignored.
 */
static int
mwifiex_cfg80211_set_power_mgmt(struct wiphy *wiphy,
				struct net_device *dev,
				bool enabled, int timeout)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(dev);
	u32 ps_mode;

	if (timeout)
		mwifiex_dbg(priv->adapter, INFO,
			    "info: ignore timeout value for IEEE Power Save\n");

	ps_mode = enabled;

	return mwifiex_drv_set_power(priv, &ps_mode);
}

/*
 * CFG802.11 operation handler to set the default network key.
 */
static int
mwifiex_cfg80211_set_default_key(struct wiphy *wiphy, struct net_device *netdev,
				 u8 key_index, bool unicast,
				 bool multicast)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(netdev);

	/* Return if WEP key not configured */
	if (!priv->sec_info.wep_enabled)
		return 0;

	if (priv->bss_type == MWIFIEX_BSS_TYPE_UAP) {
		priv->wep_key_curr_index = key_index;
	} else if (mwifiex_set_encode(priv, NULL, NULL, 0, key_index,
				      NULL, 0)) {
		mwifiex_dbg(priv->adapter, ERROR, "set default Tx key index\n");
		return -EFAULT;
	}

	return 0;
}

/*
 * CFG802.11 operation handler to add a network key.
 */
static int
mwifiex_cfg80211_add_key(struct wiphy *wiphy, struct net_device *netdev,
			 u8 key_index, bool pairwise, const u8 *mac_addr,
			 struct key_params *params)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(netdev);
	struct mwifiex_wep_key *wep_key;
	const u8 bc_mac[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	const u8 *peer_mac = pairwise ? mac_addr : bc_mac;

	if (GET_BSS_ROLE(priv) == MWIFIEX_BSS_ROLE_UAP &&
	    (params->cipher == WLAN_CIPHER_SUITE_WEP40 ||
	     params->cipher == WLAN_CIPHER_SUITE_WEP104)) {
		if (params->key && params->key_len) {
			wep_key = &priv->wep_key[key_index];
			memset(wep_key, 0, sizeof(struct mwifiex_wep_key));
			memcpy(wep_key->key_material, params->key,
			       params->key_len);
			wep_key->key_index = key_index;
			wep_key->key_length = params->key_len;
			priv->sec_info.wep_enabled = 1;
		}
		return 0;
	}

	if (mwifiex_set_encode(priv, params, params->key, params->key_len,
			       key_index, peer_mac, 0)) {
		mwifiex_dbg(priv->adapter, ERROR, "crypto keys added\n");
		return -EFAULT;
	}

	return 0;
}

/*
 * CFG802.11 operation handler to set default mgmt key.
 */
static int
mwifiex_cfg80211_set_default_mgmt_key(struct wiphy *wiphy,
				      struct net_device *netdev,
				      u8 key_index)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(netdev);
	struct mwifiex_ds_encrypt_key encrypt_key;

	wiphy_dbg(wiphy, "set default mgmt key, key index=%d\n", key_index);

	memset(&encrypt_key, 0, sizeof(struct mwifiex_ds_encrypt_key));
	encrypt_key.key_len = WLAN_KEY_LEN_CCMP;
	encrypt_key.key_index = key_index;
	encrypt_key.is_igtk_def_key = true;
	eth_broadcast_addr(encrypt_key.mac_addr);

	return mwifiex_send_cmd(priv, HostCmd_CMD_802_11_KEY_MATERIAL,
				HostCmd_ACT_GEN_SET, true, &encrypt_key, true);
}

/*
 * This function sends domain information to the firmware.
 *
 * The following information are passed to the firmware -
 *      - Country codes
 *      - Sub bands (first channel, number of channels, maximum Tx power)
 */
int mwifiex_send_domain_info_cmd_fw(struct wiphy *wiphy)
{
	u8 no_of_tt_device *netdev,
				      u8 key_index)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(netdev);
	stru