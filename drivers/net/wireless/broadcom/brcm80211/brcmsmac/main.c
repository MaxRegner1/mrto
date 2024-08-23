/*
 * Copyright (c) 2010 Broadcom Corporation
 * Copyright (c) 2013 Hauke Mehrtens <hauke@hauke-m.de>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/pci_ids.h>
#include <linux/if_ether.h>
#include <net/cfg80211.h>
#include <net/mac80211.h>
#include <brcm_hw_ids.h>
#include <aiutils.h>
#include <chipcommon.h>
#include "rate.h"
#include "scb.h"
#include "phy/phy_hal.h"
#include "channel.h"
#include "antsel.h"
#include "stf.h"
#include "ampdu.h"
#include "mac80211_if.h"
#include "ucode_loader.h"
#include "main.h"
#include "soc.h"
#include "dma.h"
#include "debug.h"
#include "brcms_trace_events.h"

/* watchdog timer, in unit of ms */
#define TIMER_INTERVAL_WATCHDOG		1000
/* radio monitor timer, in unit of ms */
#define TIMER_INTERVAL_RADIOCHK		800

/* beacon interval, in unit of 1024TU */
#define BEACON_INTERVAL_DEFAULT		100

/* n-mode support capability */
/* 2x2 includes both 1x1 & 2x2 devices
 * reserved #define 2 for future when we want to separate 1x1 & 2x2 and
 * control it independently
 */
#define WL_11N_2x2			1
#define WL_11N_3x3			3
#define WL_11N_4x4			4

#define EDCF_ACI_MASK			0x60
#define EDCF_ACI_SHIFT			5
#define EDCF_ECWMIN_MASK		0x0f
#define EDCF_ECWMAX_SHIFT		4
#define EDCF_AIFSN_MASK			0x0f
#define EDCF_AIFSN_MAX			15
#define EDCF_ECWMAX_MASK		0xf0

#define EDCF_AC_BE_TXOP_STA		0x0000
#define EDCF_AC_BK_TXOP_STA		0x0000
#define EDCF_AC_VO_ACI_STA		0x62
#define EDCF_AC_VO_ECW_STA		0x32
#define EDCF_AC_VI_ACI_STA		0x42
#define EDCF_AC_VI_ECW_STA		0x43
#define EDCF_AC_BK_ECW_STA		0xA4
#define EDCF_AC_VI_TXOP_STA		0x005e
#define EDCF_AC_VO_TXOP_STA		0x002f
#define EDCF_AC_BE_ACI_STA		0x03
#define EDCF_AC_BE_ECW_STA		0xA4
#define EDCF_AC_BK_ACI_STA		0x27
#define EDCF_AC_VO_TXOP_AP		0x002f

#define EDCF_TXOP2USEC(txop)		((txop) << 5)
#define EDCF_ECW2CW(exp)		((1 << (exp)) - 1)

#define APHY_SYMBOL_TIME		4
#define APHY_PREAMBLE_TIME		16
#define APHY_SIGNAL_TIME		4
#define APHY_SIFS_TIME			16
#define APHY_SERVICE_NBITS		16
#define APHY_TAIL_NBITS			6
#define BPHY_SIFS_TIME			10
#define BPHY_PLCP_SHORT_TIME		96

#define PREN_PREAMBLE			24
#define PREN_MM_EXT			12
#define PREN_PREAMBLE_EXT		4

#define DOT11_MAC_HDR_LEN		24
#define DOT11_ACK_LEN			10
#define DOT11_BA_LEN			4
#define DOT11_OFDM_SIGNAL_EXTENSION	6
#define DOT11_MIN_FRAG_LEN		256
#define DOT11_RTS_LEN			16
#define DOT11_CTS_LEN			10
#define DOT11_BA_BITMAP_LEN		128
#define DOT11_MAXNUMFRAGS		16
#define DOT11_MAX_FRAG_LEN		2346

#define BPHY_PLCP_TIME			192
#define RIFS_11N_TIME			2

/* length of the BCN template area */
#define BCN_TMPL_LEN			512

/* brcms_bss_info flag bit values */
#define BRCMS_BSS_HT			0x0020	/* BSS is HT (MIMO) capable */

/* chip rx buffer offset */
#define BRCMS_HWRXOFF			38

/* rfdisable delay timer 500 ms, runs of ALP clock */
#define RFDISABLE_DEFAULT		10000000

#define BRCMS_TEMPSENSE_PERIOD		10	/* 10 second timeout */

/* synthpu_dly times in us */
#define SYNTHPU_DLY_APHY_US		3700
#define SYNTHPU_DLY_BPHY_US		1050
#define SYNTHPU_DLY_NPHY_US		2048
#define SYNTHPU_DLY_LPPHY_US		300

#define ANTCNT				10	/* vanilla M_MAX_ANTCNT val */

/* Per-AC retry limit register definitions; uses defs.h bitfield macros */
#define EDCF_SHORT_S			0
#define EDCF_SFB_S			4
#define EDCF_LONG_S			8
#define EDCF_LFB_S			12
#define EDCF_SHORT_M			BITFIELD_MASK(4)
#define EDCF_SFB_M			BITFIELD_MASK(4)
#define EDCF_LONG_M			BITFIELD_MASK(4)
#define EDCF_LFB_M			BITFIELD_MASK(4)

#define RETRY_SHORT_DEF			7	/* Default Short retry Limit */
#define RETRY_SHORT_MAX			255	/* Maximum Short retry Limit */
#define RETRY_LONG_DEF			4	/* Default Long retry count */
#define RETRY_SHORT_FB			3	/* Short count for fb rate */
#define RETRY_LONG_FB			2	/* Long count for fb rate */

#define APHY_CWMIN			15
#define PHY_CWMAX			1023

#define EDCF_AIFSN_MIN			1

#define FRAGNUM_MASK			0xF

#define APHY_SLOT_TIME			9
#define BPHY_SLOT_TIME			20

#define WL_SPURAVOID_OFF		0
#define WL_SPURAVOID_ON1		1
#define WL_SPURAVOID_ON2		2

/* invalid core flags, use the saved coreflags */
#define BRCMS_USE_COREFLAGS		0xffffffff

/* values for PLCPHdr_override */
#define BRCMS_PLCP_AUTO			-1
#define BRCMS_PLCP_SHORT		0
#define BRCMS_PLCP_LONG			1

/* values for g_protection_override and n_protection_override */
#define BRCMS_PROTECTION_AUTO		-1
#define BRCMS_PROTECTION_OFF		0
#define BRCMS_PROTECTION_ON		1
#define BRCMS_PROTECTION_MMHDR_ONLY	2
#define BRCMS_PROTECTION_CTS_ONLY	3

/* values for g_protection_control and n_protection_control */
#define BRCMS_PROTECTION_CTL_OFF	0
#define BRCMS_PROTECTION_CTL_LOCAL	1
#define BRCMS_PROTECTION_CTL_OVERLAP	2

/* values for n_protection */
#define BRCMS_N_PROTECTION_OFF		0
#define BRCMS_N_PROTECTION_OPTIONAL	1
#define BRCMS_N_PROTECTION_20IN40	2
#define BRCMS_N_PROTECTION_MIXEDMODE	3

/* values for band specific 40MHz capabilities */
#define BRCMS_N_BW_20ALL		0
#define BRCMS_N_BW_40ALL		1
#define BRCMS_N_BW_20IN2G_40IN5G	2

/* bitflags for SGI support (sgi_rx iovar) */
#define BRCMS_N_SGI_20			0x01
#define BRCMS_N_SGI_40			0x02

/* defines used by the nrate iovar */
/* MSC in use,indicates b0-6 holds an mcs */
#define NRATE_MCS_INUSE			0x00000080
/* rate/mcs value */
#define NRATE_RATE_MASK			0x0000007f
/* stf mode mask: siso, cdd, stbc, sdm */
#define NRATE_STF_MASK			0x0000ff00
/* stf mode shift */
#define NRATE_STF_SHIFT			8
/* bit indicate to override mcs only */
#define NRATE_OVERRIDE_MCS_ONLY		0x40000000
#define NRATE_SGI_MASK			0x00800000	/* sgi mode */
#define NRATE_SGI_SHIFT			23		/* sgi mode */
#define NRATE_LDPC_CODING		0x00400000	/* adv coding in use */
#define NRATE_LDPC_SHIFT		22		/* ldpc shift */

#define NRATE_STF_SISO			0		/* stf mode SISO */
#define NRATE_STF_CDD			1		/* stf mode CDD */
#define NRATE_STF_STBC			2		/* stf mode STBC */
#define NRATE_STF_SDM			3		/* stf mode SDM */

#define MAX_DMA_SEGS			4

/* # of entries in Tx FIFO */
#define NTXD				64
/* Max # of entries in Rx FIFO based on 4kb page size */
#define NRXD				256

/* Amount of headroom to leave in Tx FIFO */
#define TX_HEADROOM			4

/* try to keep this # rbufs posted to the chip */
#define NRXBUFPOST			32

/* max # frames to process in brcms_c_recv() */
#define RXBND				8
/* max # tx status to process in wlc_txstatus() */
#define TXSBND				8

/* brcmu_format_flags() bit description structure */
struct brcms_c_bit_desc {
	u32 bit;
	const char *name;
};

/*
 * The following table lists the buffer memory allocated to xmt fifos in HW.
 * the size is in units of 256bytes(one block), total size is HW dependent
 * ucode has default fifo partition, sw can overwrite if necessary
 *
 * This is documented in twiki under the topic UcodeTxFifo. Please ensure
 * the twiki is updated before making changes.
 */

/* Starting corerev for the fifo size table */
#define XMTFIFOTBL_STARTREV	17

struct d11init {
	__le16 addr;
	__le16 size;
	__le32 value;
};

struct edcf_acparam {
	u8 ACI;
	u8 ECW;
	u16 TXOP;
} __packed;

/* debug/trace */
uint brcm_msg_level;

/* TX FIFO number to WME/802.1E Access Category */
static const u8 wme_fifo2ac[] = {
	IEEE80211_AC_BK,
	IEEE80211_AC_BE,
	IEEE80211_AC_VI,
	IEEE80211_AC_VO,
	IEEE80211_AC_BE,
	IEEE80211_AC_BE
};

/* ieee80211 Access Category to TX FIFO number */
static const u8 wme_ac2fifo[] = {
	TX_AC_VO_FIFO,
	TX_AC_VI_FIFO,
	TX_AC_BE_FIFO,
	TX_AC_BK_FIFO
};

static const u16 xmtfifo_sz[][NFIFO] = {
	/* corerev 17: 5120, 49152, 49152, 5376, 4352, 1280 */
	{20, 192, 192, 21, 17, 5},
	/* corerev 18: */
	{0, 0, 0, 0, 0, 0},
	/* corerev 19: */
	{0, 0, 0, 0, 0, 0},
	/* corerev 20: 5120, 49152, 49152, 5376, 4352, 1280 */
	{20, 192, 192, 21, 17, 5},
	/* corerev 21: 2304, 14848, 5632, 3584, 3584, 1280 */
	{9, 58, 22, 14, 14, 5},
	/* corerev 22: 5120, 49152, 49152, 5376, 4352, 1280 */
	{20, 192, 192, 21, 17, 5},
	/* corerev 23: 5120, 49152, 49152, 5376, 4352, 1280 */
	{20, 192, 192, 21, 17, 5},
	/* corerev 24: 2304, 14848, 5632, 3584, 3584, 1280 */
	{9, 58, 22, 14, 14, 5},
	/* corerev 25: */
	{0, 0, 0, 0, 0, 0},
	/* corerev 26: */
	{0, 0, 0, 0, 0, 0},
	/* corerev 27: */
	{0, 0, 0, 0, 0, 0},
	/* corerev 28: 2304, 14848, 5632, 3584, 3584, 1280 */
	{9, 58, 22, 14, 14, 5},
};

#ifdef DEBUG
static const char * const fifo_names[] = {
	"AC_BK", "AC_BE", "AC_VI", "AC_VO", "BCMC", "ATIM" };
#else
static const char fifo_names[6][1];
#endif

#ifdef DEBUG
/* pointer to most recently allocated wl/wlc */
static struct brcms_c_info *wlc_info_dbg = (struct brcms_c_info *) (NULL);
#endif

/* Mapping of ieee80211 AC numbers to tx fifos */
static const u8 ac_to_fifo_mapping[IEEE80211_NUM_ACS] = {
	[IEEE80211_AC_VO]	= TX_AC_VO_FIFO,
	[IEEE80211_AC_VI]	= TX_AC_VI_FIFO,
	[IEEE80211_AC_BE]	= TX_AC_BE_FIFO,
	[IEEE80211_AC_BK]	= TX_AC_BK_FIFO,
};

/* Mapping of tx fifos to ieee80211 AC numbers */
static const u8 fifo_to_ac_mapping[IEEE80211_NUM_ACS] = {
	[TX_AC_BK_FIFO]	= IEEE80211_AC_BK,
	[TX_AC_BE_FIFO]	= IEEE80211_AC_BE,
	[TX_AC_VI_FIFO]	= IEEE80211_AC_VI,
	[TX_AC_VO_FIFO]	= IEEE80211_AC_VO,
};

static u8 brcms_ac_to_fifo(u8 ac)
{
	if (ac >= ARRAY_SIZE(ac_to_fifo_mapping))
		return TX_AC_BE_FIFO;
	return ac_to_fifo_mapping[ac];
}

static u8 brcms_fifo_to_ac(u8 fifo)
{
	if (fifo >= ARRAY_SIZE(fifo_to_ac_mapping))
		return IEEE80211_AC_BE;
	return fifo_to_ac_mapping[fifo];
}

/* Find basic rate for a given rate */
static u8 brcms_basic_rate(struct brcms_c_info *wlc, u32 rspec)
{
	if (is_mcs_rate(rspec))
		return wlc->band->basic_rate[mcs_table[rspec & RSPEC_RATE_MASK]
		       .leg_ofdm];
	return wlc->band->basic_rate[rspec & RSPEC_RATE_MASK];
}

static u16 frametype(u32 rspec, u8 mimoframe)
{
	if (is_mcs_rate(rspec))
		return mimoframe;
	return is_cck_rate(rspec) ? FT_CCK : FT_OFDM;
}

/* currently the best mechanism for determining SIFS is the band in use */
static u16 get_sifs(struct brcms_band *band)
{
	return band->bandtype == BRCM_BAND_5G ? APHY_SIFS_TIME :
				 BPHY_SIFS_TIME;
}

/*
 * Detect Card removed.
 * Even checking an sbconfig register read will not false trigger when the core
 * is in reset it breaks CF address mechanism. Accessing gphy phyversion will
 * cause SB error if aphy is in reset on 4306B0-DB. Need a simple accessible
 * reg with fixed 0/1 pattern (some platforms return all 0).
 * If clocks are present, call the sb routine which will figure out if the
 * device is removed.
 */
static bool brcms_deviceremoved(struct brcms_c_info *wlc)
{
	u32 macctrl;

	if (!wlc->hw->clk)
		return ai_deviceremoved(wlc->hw->sih);
	macctrl = bcma_read32(wlc->hw->d11core,
			      D11REGOFFS(maccontrol));
	return (macctrl & (MCTL_PSM_JMP_0 | MCTL_IHR_EN)) != MCTL_IHR_EN;
}

/* sum the individual fifo tx pending packet counts */
static int brcms_txpktpendtot(struct brcms_c_info *wlc)
{
	int i;
	int pending = 0;

	for (i = 0; i < ARRAY_SIZE(wlc->hw->di); i++)
		if (wlc->hw->di[i])
			pending += dma_txpending(wlc->hw->di[i]);
	return pending;
}

static bool brcms_is_mband_unlocked(struct brcms_c_info *wlc)
{
	return wlc->pub->_nbands > 1 && !wlc->bandlocked;
}

static int brcms_chspec_bw(u16 chanspec)
{
	if (CHSPEC_IS40(chanspec))
		return BRCMS_40_MHZ;
	if (CHSPEC_IS20(chanspec))
		return BRCMS_20_MHZ;

	return BRCMS_10_MHZ;
}

static void brcms_c_bsscfg_mfree(struct brcms_bss_cfg *cfg)
{
	if (cfg == NULL)
		return;

	kfree(cfg->current_bss);
	kfree(cfg);
}

static void brcms_c_detach_mfree(struct brcms_c_info *wlc)
{
	if (wlc == NULL)
		return;

	brcms_c_bsscfg_mfree(wlc->bsscfg);
	kfree(wlc->pub);
	kfree(wlc->modulecb);
	kfree(wlc->default_bss);
	kfree(wlc->protection);
	kfree(wlc->stf);
	kfree(wlc->bandstate[0]);
	if (wlc->corestate)
		kfree(wlc->corestate->macstat_snapshot);
	kfree(wlc->corestate);
	if (wlc->hw)
		kfree(wlc->hw->bandstate[0]);
	kfree(wlc->hw);
	if (wlc->beacon)
		dev_kfree_skb_any(wlc->beacon);
	if (wlc->probe_resp)
		dev_kfree_skb_any(wlc->probe_resp);

	kfree(wlc);
}

static struct brcms_bss_cfg *brcms_c_bsscfg_malloc(uint unit)
{
	struct brcms_bss_cfg *cfg;

	cfg = kzalloc(sizeof(struct brcms_bss_cfg), GFP_ATOMIC);
	if (cfg == NULL)
		goto fail;

	cfg->current_bss = kzalloc(sizeof(struct brcms_bss_info), GFP_ATOMIC);
	if (cfg->current_bss == NULL)
		goto fail;

	return cfg;

 fail:
	brcms_c_bsscfg_mfree(cfg);
	return NULL;
}

static struct brcms_c_info *
brcms_c_attach_malloc(uint unit, uint *err, uint devid)
{
	struct brcms_c_info *wlc;

	wlc = kzalloc(sizeof(struct brcms_c_info), GFP_ATOMIC);
	if (wlc == NULL) {
		*err = 1002;
		goto fail;
	}

	/* allocate struct brcms_c_pub state structure */
	wlc->pub = kzalloc(sizeof(struct brcms_pub), GFP_ATOMIC);
	if (wlc->pub == NULL) {
		*err = 1003;
		goto fail;
	}
	wlc->pub->wlc = wlc;

	/* allocate struct brcms_hardware state structure */

	wlc->hw = kzalloc(sizeof(struct brcms_hardware), GFP_ATOMIC);
	if (wlc->hw == NULL) {
		*err = 1005;
		goto fail;
	}
	wlc->hw->wlc = wlc;

	wlc->hw->bandstate[0] =
		kzalloc(sizeof(struct brcms_hw_band) * MAXBANDS, GFP_ATOMIC);
	if (wlc->hw->bandstate[0] == NULL) {
		*err = 1006;
		goto fail;
	} else {
		int i;

		for (i = 1; i < MAXBANDS; i++)
			wlc->hw->bandstate[i] = (struct brcms_hw_band *)
			    ((unsigned long)wlc->hw->bandstate[0] +
			     (sizeof(struct brcms_hw_band) * i));
	}

	wlc->modulecb =
		kzalloc(sizeof(struct modulecb) * BRCMS_MAXMODULES, GFP_ATOMIC);
	if (wlc->modulecb == NULL) {
		*err = 1009;
		goto fail;
	}

	wlc->default_bss = kzalloc(sizeof(struct brcms_bss_info), GFP_ATOMIC);
	if (wlc->default_bss == NULL) {
		*err = 1010;
		goto fail;
	}

	wlc->bsscfg = brcms_c_bsscfg_malloc(unit);
	if (wlc->bsscfg == NULL) {
		*err = 1011;
		goto fail;
	}

	wlc->protection = kzalloc(sizeof(struct brcms_protection),
				  GFP_ATOMIC);
	if (wlc->protection == NULL) {
		*err = 1016;
		goto fail;
	}

	wlc->stf = kzalloc(sizeof(struct brcms_stf), GFP_ATOMIC);
	if (wlc->stf == NULL) {
		*err = 1017;
		goto fail;
	}

	wlc->bandstate[0] =
		kzalloc(sizeof(struct brcms_band)*MAXBANDS, GFP_ATOMIC);
	if (wlc->bandstate[0] == NULL) {
		*err = 1025;
		goto fail;
	} else {
		int i;

		for (i = 1; i < MAXBANDS; i++)
			wlc->bandstate[i] = (struct brcms_band *)
				((unsigned long)wlc->bandstate[0]
				+ (sizeof(struct brcms_band)*i));
	}

	wlc->corestate = kzalloc(sizeof(struct brcms_core), GFP_ATOMIC);
	if (wlc->corestate == NULL) {
		*err = 1026;
		goto fail;
	}

	wlc->corestate->macstat_snapshot =
		kzalloc(sizeof(struct macstat), GFP_ATOMIC);
	if (wlc->corestate->macstat_snapshot == NULL) {
		*err = 1027;
		goto fail;
	}

	return wlc;

 fail:
	brcms_c_detach_mfree(wlc);
	return NULL;
}

/*
 * Update the slot timing for standard 11b/g (20us slots)
 * or shortslot 11g (9us slots)
 * The PSM needs to be suspended for this call.
 */
static void brcms_b_update_slot_timing(struct brcms_hardware *wlc_hw,
					bool shortslot)
{
	struct bcma_device *core = wlc_hw->d11core;

	if (shortslot) {
		/* 11g short slot: 11a timing */
		bcma_write16(core, D11REGOFFS(ifs_slot), 0x0207);
		brcms_b_write_shm(wlc_hw, M_DOT11_SLOT, APHY_SLOT_TIME);
	} else {
		/* 11g long slot: 11b timing */
		bcma_write16(core, D11REGOFFS(ifs_slot), 0x0212);
		brcms_b_write_shm(wlc_hw, M_DOT11_SLOT, BPHY_SLOT_TIME);
	}
}

/*
 * calculate frame duration of a given rate and length, return
 * time in usec unit
 */
static uint brcms_c_calc_frame_time(struct brcms_c_info *wlc, u32 ratespec,
				    u8 preamble_type, uint mac_len)
{
	uint nsyms, dur = 0, Ndps, kNdps;
	uint rate = rspec2rate(ratespec);

	if (rate == 0) {
		brcms_err(wlc->hw->d11core, "wl%d: WAR: using rate of 1 mbps\n",
			  wlc->pub->unit);
		rate = BRCM_RATE_1M;
	}

	if (is_mcs_rate(ratespec)) {
		uint mcs = ratespec & RSPEC_RATE_MASK;
		int tot_streams = mcs_2_txstreams(mcs) + rspec_stc(ratespec);

		dur = PREN_PREAMBLE + (tot_streams * PREN_PREAMBLE_EXT);
		if (preamble_type == BRCMS_MM_PREAMBLE)
			dur += PREN_MM_EXT;
		/* 1000Ndbps = kbps * 4 */
		kNdps = mcs_2_rate(mcs, rspec_is40mhz(ratespec),
				   rspec_issgi(ratespec)) * 4;

		if (rspec_stc(ratespec) == 0)
			nsyms =
			    CEIL((APHY_SERVICE_NBITS + 8 * mac_len +
				  APHY_TAIL_NBITS) * 1000, kNdps);
		else
			/* STBC needs to have even number of symbols */
			nsyms =
			    2 *
			    CEIL((APHY_SERVICE_NBITS + 8 * mac_len +
				  APHY_TAIL_NBITS) * 1000, 2 * kNdps);

		dur += APHY_SYMBOL_TIME * nsyms;
		if (wlc->band->bandtype == BRCM_BAND_2G)
			dur += DOT11_OFDM_SIGNAL_EXTENSION;
	} else if (is_ofdm_rate(rate)) {
		dur = APHY_PREAMBLE_TIME;
		dur += APHY_SIGNAL_TIME;
		/* Ndbps = Mbps * 4 = rate(500Kbps) * 2 */
		Ndps = rate * 2;
		/* NSyms = CEILING((SERVICE + 8*NBytes + TAIL) / Ndbps) */
		nsyms =
		    CEIL((APHY_SERVICE_NBITS + 8 * mac_len + APHY_TAIL_NBITS),
			 Ndps);
		dur += APHY_SYMBOL_TIME * nsyms;
		if (wlc->band->bandtype == BRCM_BAND_2G)
			dur += DOT11_OFDM_SIGNAL_EXTENSION;
	} else {
		/*
		 * calc # bits * 2 so factor of 2 in rate (1/2 mbps)
		 * will divide out
		 */
		mac_len = mac_len * 8 * 2;
		/* calc ceiling of bits/rate = microseconds of air time */
		dur = (mac_len + rate - 1) / rate;
		if (preamble_type & BRCMS_SHORT_PREAMBLE)
			dur += BPHY_PLCP_SHORT_TIME;
		else
			dur += BPHY_PLCP_TIME;
	}
	return dur;
}

static void brcms_c_write_inits(struct brcms_hardware *wlc_hw,
				const struct d11init *inits)
{
	struct bcma_device *core = wlc_hw->d11core;
	int i;
	uint offset;
	u16 size;
	u32 value;

	brcms_dbg_info(wlc_hw->d11core, "wl%d\n", wlc_hw->unit);

	for (i = 0; inits[i].addr != cpu_to_le16(0xffff); i++) {
		size = le16_to_cpu(inits[i].size);
		offset = le16_to_cpu(inits[i].addr);
		value = le32_to_cpu(inits[i].value);
		if (size == 2)
			bcma_write16(core, offset, value);
		else if (size == 4)
			bcma_write32(core, offset, value);
		else
			break;
	}
}

static void brcms_c_write_mhf(struct brcms_hardware *wlc_hw, u16 *mhfs)
{
	u8 idx;
	u16 addr[] = {
		M_HOST_FLAGS1, M_HOST_FLAGS2, M_HOST_FLAGS3, M_HOST_FLAGS4,
		M_HOST_FLAGS5
	};

	for (idx = 0; idx < MHFMAX; idx++)
		brcms_b_write_shm(wlc_hw, addr[idx], mhfs[idx]);
}

static void brcms_c_ucode_bsinit(struct brcms_hardware *wlc_hw)
{
	struct brcms_ucode *ucode = &wlc_hw->wlc->wl->ucode;

	/* init microcode host flags */
	brcms_c_write_mhf(wlc_hw, wlc_hw->band->mhfs);

	/* do band-specific ucode IHR, SHM, and SCR inits */
	if (D11REV_IS(wlc_hw->corerev, 17) || D11REV_IS(wlc_hw->corerev, 23)) {
		if (BRCMS_ISNPHY(wlc_hw->band))
			brcms_c_write_inits(wlc_hw, ucode->d11n0bsinitvals16);
		else
			brcms_err(wlc_hw->d11core,
				  "%s: wl%d: unsupported phy in corerev %d\n",
				  __func__, wlc_hw->unit,
				  wlc_hw->corerev);
	} else {
		if (D11REV_IS(wlc_hw->corerev, 24)) {
			if (BRCMS_ISLCNPHY(wlc_hw->band))
				brcms_c_write_inits(wlc_hw,
						    ucode->d11lcn0bsinitvals24);
			else
				brcms_err(wlc_hw->d11core,
					  "%s: wl%d: unsupported phy in core rev %d\n",
					  __func__, wlc_hw->unit,
					  wlc_hw->corerev);
		} else {
			brcms_err(wlc_hw->d11core,
				  "%s: wl%d: unsupported corerev %d\n",
				  __func__, wlc_hw->unit, wlc_hw->corerev);
		}
	}
}

static void brcms_b_core_ioctl(struct brcms_hardware *wlc_hw, u32 m, u32 v)
{
	struct bcma_device *core = wlc_hw->d11core;
	u32 ioctl = bcma_aread32(core, BCMA_IOCTL) & ~m;

	bcma_awrite32(core, BCMA_IOCTL, ioctl | v);
}

static void brcms_b_core_phy_clk(struct brcms_hardware *wlc_hw, bool clk)
{
	brcms_dbg_info(wlc_hw->d11core, "wl%d: clk %d\n", wlc_hw->unit, clk);

	wlc_hw->phyclk = clk;

	if (OFF == clk) {	/* clear gmode bit, put phy into reset */

		brcms_b_core_ioctl(wlc_hw, (SICF_PRST | SICF_FGC | SICF_GMODE),
				   (SICF_PRST | SICF_FGC));
		udelay(1);
		brcms_b_core_ioctl(wlc_hw, (SICF_PRST | SICF_FGC), SICF_PRST);
		udelay(1);

	} else {		/* take phy out of reset */

		brcms_b_core_ioctl(wlc_hw, (SICF_PRST | SICF_FGC), SICF_FGC);
		udelay(1);
		brcms_b_core_ioctl(wlc_hw, SICF_FGC, 0);
		udelay(1);

	}
}

/* low-level band switch utility routine */
static void brcms_c_setxband(struct brcms_hardware *wlc_hw, uint bandunit)
{
	brcms_dbg_mac80211(wlc_hw->d11core, "wl%d: bandunit %d\n", wlc_hw->unit,
			   bandunit);

	wlc_hw->band = wlc_hw->bandstate[bandunit];

	/*
	 * BMAC_NOTE:
	 *   until we eliminate need for wlc->band refs in low level code
	 */
	wlc_hw->wlc->band = wlc_hw->wlc->bandstate[bandunit];

	/* set gmode core flag */
	if (wlc_hw->sbclk && !wlc_hw->noreset) {
		u32 gmode = 0;

		if (bandunit == 0)
			gmode = SICF_GMODE;

		brcms_b_core_ioctl(wlc_hw, SICF_GMODE, gmode);
	}
}

/* switch to new band but leave it inactive */
static u32 brcms_c_setband_inact(struct brcms_c_info *wlc, uint bandunit)
{
	struct brcms_hardware *wlc_hw = wlc->hw;
	u32 macintmask;
	u32 macctrl;

	brcms_dbg_mac80211(wlc_hw->d11core, "wl%d\n", wlc_hw->unit);
	macctrl = bcma_read32(wlc_hw->d11core,
			      D11REGOFFS(maccontrol));
	WARN_ON((macctrl & MCTL_EN_MAC) != 0);

	/* disable interrupts */
	macintmask = brcms_intrsoff(wlc->wl);

	/* radio off */
	wlc_phy_switch_radio(wlc_hw->band->pi, OFF);

	brcms_b_core_phy_clk(wlc_hw, OFF);

	brcms_c_setxband(wlc_hw, bandunit);

	return macintmask;
}

/* process an individual struct tx_status */
static bool
brcms_c_dotxstatus(struct brcms_c_info *wlc, struct tx_status *txs)
{
	struct sk_buff *p = NULL;
	uint queue = NFIFO;
	struct dma_pub *dma = NULL;
	struct d11txh *txh = NULL;
	struct scb *scb = NULL;
	bool free_pdu;
	int tx_rts, tx_frame_count, tx_rts_count;
	uint totlen, supr_status;
	bool lastframe;
	struct ieee80211_hdr *h;
	u16 mcl;
	struct ieee80211_tx_info *tx_info;
	struct ieee80211_tx_rate *txrate;
	int i;
	bool fatal = true;

	trace_brcms_txstatus(&wlc->hw->d11core->dev, txs->framelen,
			     txs->frameid, txs->status, txs->lasttxtime,
			     txs->sequence, txs->phyerr, txs->ackphyrxsh);

	/* discard intermediate indications for ucode with one legitimate case:
	 *   e.g. if "useRTS" is set. ucode did a successful rts/cts exchange,
	 *   but the subsequent tx of DATA failed. so it will start rts/cts
	 *   from the beginning (resetting the rts transmission count)
	 */
	if (!(txs->status & TX_STATUS_AMPDU)
	    && (txs->status & TX_STATUS_INTERMEDIATE)) {
		brcms_dbg_tx(wlc->hw->d11core, "INTERMEDIATE but not AMPDU\n");
		fatal = false;
		goto out;
	}

	queue = txs->frameid & TXFID_QUEUE_MASK;
	if (queue >= NFIFO) {
		brcms_err(wlc->hw->d11core, "queue %u >= NFIFO\n", queue);
		goto out;
	}

	dma = wlc->hw->di[queue];

	p = dma_getnexttxp(wlc->hw->di[queue], DMA_RANGE_TRANSMITTED);
	if (p == NULL) {
		brcms_err(wlc->hw->d11core, "dma_getnexttxp returned null!\n");
		goto out;
	}

	txh = (struct d11txh *) (p->data);
	mcl = le16_to_cpu(txh->MacTxControlLow);

	if (txs->phyerr)
		brcms_dbg_tx(wlc->hw->d11core, "phyerr 0x%x, rate 0x%x\n",
			     txs->phyerr, txh->MainRates);

	if (txs->frameid != le16_to_cpu(txh->TxFrameID)) {
		brcms_err(wlc->hw->d11core, "frameid != txh->TxFrameID\n");
		goto out;
	}
	tx_info = IEEE80211_SKB_CB(p);
	h = (struct ieee80211_hdr *)((u8 *) (txh + 1) + D11_PHY_HDR_LEN);

	if (tx_info->rate_driver_data[0])
		scb = &wlc->pri_scb;

	if (tx_info->flags & IEEE80211_TX_CTL_AMPDU) {
		brcms_c_ampdu_dotxstatus(wlc->ampdu, scb, p, txs);
		fatal = false;
		goto out;
	}

	/*
	 * brcms_c_ampdu_dotxstatus() will trace tx descriptors for AMPDU
	 * frames; this traces them for the rest.
	 */
	trace_brcms_txdesc(&wlc->hw->d11core->dev, txh, sizeof(*txh));

	supr_status = txs->status & TX_STATUS_SUPR_MASK;
	if (supr_status == TX_STATUS_SUPR_BADCH) {
		unsigned xfts = le16_to_cpu(txh->XtraFrameTypes);
		brcms_dbg_tx(wlc->hw->d11core,
			     "Pkt tx suppressed, dest chan %u, current %d\n",
			     (xfts >> XFTS_CHANNEL_SHIFT) & 0xff,
			     CHSPEC_CHANNEL(wlc->default_bss->chanspec));
	}

	tx_rts = le16_to_cpu(txh->MacTxControlLow) & TXC_SENDRTS;
	tx_frame_count =
	    (txs->status & TX_STATUS_FRM_RTX_MASK) >> TX_STATUS_FRM_RTX_SHIFT;
	tx_rts_count =
	    (txs->status & TX_STATUS_RTS_RTX_MASK) >> TX_STATUS_RTS_RTX_SHIFT;

	lastframe = !ieee80211_has_morefrags(h->frame_control);

	if (!lastframe) {
		brcms_err(wlc->hw->d11core, "Not last frame!\n");
	} else {
		/*
		 * Set information to be consumed by Minstrel ht.
		 *
		 * The "fallback limit" is the number of tx attempts a given
		 * MPDU is sent at the "primary" rate. Tx attempts beyond that
		 * limit are sent at the "secondary" rate.
		 * A 'short frame' does not exceed RTS treshold.
		 */
		u16 sfbl,	/* Short Frame Rate Fallback Limit */
		    lfbl,	/* Long Frame Rate Fallback Limit */
		    fbl;

		if (queue < IEEE80211_NUM_ACS) {
			sfbl = GFIELD(wlc->wme_retries[wme_fifo2ac[queue]],
				      EDCF_SFB);
			lfbl = GFIELD(wlc->wme_retries[wme_fifo2ac[queue]],
				      EDCF_LFB);
		} else {
			sfbl = wlc->SFBL;
			lfbl = wlc->LFBL;
		}

		txrate = tx_info->status.rates;
		if (txrate[0].flags & IEEE80211_TX_RC_USE_RTS_CTS)
			fbl = lfbl;
		else
			fbl = sfbl;

		ieee80211_tx_info_clear_status(tx_info);

		if ((tx_frame_count > fbl) && (txrate[1].idx >= 0)) {
			/*
			 * rate selection requested a fallback rate
			 * and we used it
			 */
			txrate[0].count = fbl;
			txrate[1].count = tx_frame_count - fbl;
		} else {
			/*
			 * rate selection did not request fallback rate, or
			 * we didn't need it
			 */
			txrate[0].count = tx_frame_count;
			/*
			 * rc80211_minstrel.c:minstrel_tx_status() expects
			 * unused rates to be marked with idx = -1
			 */
			txrate[1].idx = -1;
			txrate[1].count = 0;
		}

		/* clear the rest of the rates */
		for (i = 2; i < IEEE80211_TX_MAX_RATES; i++) {
			txrate[i].idx = -1;
			txrate[i].count = 0;
		}

		if (txs->status & TX_STATUS_ACK_RCV)
			tx_info->flags |= IEEE80211_TX_STAT_ACK;
	}

	totlen = p->len;
	free_pdu = true;

	if (lastframe) {
		/* remove PLCP & Broadcom tx descriptor header */
		skb_pull(p, D11_PHY_HDR_LEN);
		skb_pull(p, D11_TXH_LEN);
		ieee80211_tx_status_irqsafe(wlc->pub->ieee_hw, p);
	} else {
		brcms_err(wlc->hw->d11core,
			  "%s: Not last frame => not calling tx_status\n",
			  __func__);
	}

	fatal = false;

 out:
	if (fatal) {
		if (txh)
			trace_brcms_txdesc(&wlc->hw->d11core->dev, txh,
					   sizeof(*txh));
		brcmu_pkt_buf_free_skb(p);
	}

	if (dma && queue < NFIFO) {
		u16 ac_queue = brcms_fifo_to_ac(queue);
		if (dma->txavail > TX_HEADROOM && queue < TX_BCMC_FIFO &&
		    ieee80211_queue_stopped(wlc->pub->ieee_hw, ac_queue))
			ieee80211_wake_queue(wlc->pub->ieee_hw, ac_queue);
		dma_kick_tx(dma);
	}

	return fatal;
}

/* process tx completion events in BMAC
 * Return true if more tx status need to be processed. false otherwise.
 */
static bool
brcms_b_txstatus(struct brcms_hardware *wlc_hw, bool bound, bool *fatal)
{
	struct bcma_device *core;
	struct tx_status txstatus, *txs;
	u32 s1, s2;
	uint n = 0;
	/*
	 * Param 'max_tx_num' indicates max. # tx status to process before
	 * break out.
	 */
	uint max_tx_num = bound ? TXSBND : -1;

	txs = &txstatus;
	core = wlc_hw->d11core;
	*fatal = false;

	while (n < max_tx_num) {
		s1 = bcma_read32(core, D11REGOFFS(frmtxstatus));
		if (s1 == 0xffffffff) {
			brcms_err(core, "wl%d: %s: dead chip\n", wlc_hw->unit,
				  __func__);
			*fatal = true;
			return false;
		}
		/* only process when valid */
		if (!(s1 & TXS_V))
			break;

		s2 = bcma_read32(core, D11REGOFFS(frmtxstatus2));
		txs->status = s1 & TXS_STATUS_MASK;
		txs->frameid = (s1 & TXS_FID_MASK) >> TXS_FID_SHIFT;
		txs->sequence = s2 & TXS_SEQ_MASK;
		txs->phyerr = (s2 & TXS_PTX_MASK) >> TXS_PTX_SHIFT;
		txs->lasttxtime = 0;

		*fatal = brcms_c_dotxstatus(wlc_hw->wlc, txs);
		if (*fatal == true)
			return false;
		n++;
	}

	return n >= max_tx_num;
}

static void brcms_c_tbtt(struct brcms_c_info *wlc)
{
	if (wlc->bsscfg->type == BRCMS_TYPE_ADHOC)
		/*
		 * DirFrmQ is now valid...defer setting until end
		 * of ATIM window
		 */
		wlc->qvalid |= MCMD_DIRFRMQVAL;
}

/* set initial host flags value */
static void
brcms_c_mhfdef(struct brcms_c_info *wlc, u16 *mhfs, u16 mhf2_init)
{
	struct brcms_hardware *wlc_hw = wlc->hw;

	memset(mhfs, 0, MHFMAX * sizeof(u16));

	mhfs[MHF2] |= mhf2_init;

	/* prohibit use of slowclock on multifunction boards */
	if (wlc_hw->boardflags & BFL_NOPLLDOWN)
		mhfs[MHF1] |= MHF1_FORCEFASTCLK;

	if (BRCMS_ISNPHY(wlc_hw->band) && NREV_LT(wlc_hw->band->phyrev, 2)) {
		mhfs[MHF2] |= MHF2_NPHY40MHZ_WAR;
		mhfs[MHF1] |= MHF1_IQSWAP_WAR;
	}
}

static uint
dmareg(uint direction, uint fifonum)
{
	if (direction == DMA_TX)
		return offsetof(struct d11regs, fifo64regs[fifonum].dmaxmt);
	return offsetof(struct d11regs, fifo64regs[fifonum].dmarcv);
}

static bool brcms_b_attach_dmapio(struct brcms_c_info *wlc, uint j, bool wme)
{
	uint i;
	char name[8];
	/*
	 * ucode host flag 2 needed for pio mode, independent of band and fifo
	 */
	u16 pio_mhf2 = 0;
	struct brcms_hardware *wlc_hw = wlc->hw;
	uint unit = wlc_hw->unit;

	/* name and offsets for dma_attach */
	snprintf(name, sizeof(name), "wl%d", unit);

	if (wlc_hw->di[0] == NULL) {	/* Init FIFOs */
		int dma_attach_err = 0;

		/*
		 * FIFO 0
		 * TX: TX_AC_BK_FIFO (TX AC Background data packets)
		 * RX: RX_FIFO (RX data packets)
		 */
		wlc_hw->di[0] = dma_attach(name, wlc,
					   (wme ? dmareg(DMA_TX, 0) : 0),
					   dmareg(DMA_RX, 0),
					   (wme ? NTXD : 0), NRXD,
					   RXBUFSZ, -1, NRXBUFPOST,
					   BRCMS_HWRXOFF);
		dma_attach_err |= (NULL == wlc_hw->di[0]);

		/*
		 * FIFO 1
		 * TX: TX_AC_BE_FIFO (TX AC Best-Effort data packets)
		 *   (legacy) TX_DATA_FIFO (TX data packets)
		 * RX: UNUSED
		 */
		wlc_hw->di[1] = dma_attach(name, wlc,
					   dmareg(DMA_TX, 1), 0,
					   NTXD, 0, 0, -1, 0, 0);
		dma_attach_err |= (NULL == wlc_hw->di[1]);

		/*
		 * FIFO 2
		 * TX: TX_AC_VI_FIFO (TX AC Video data packets)
		 * RX: UNUSED
		 */
		wlc_hw->di[2] = dma_attach(name, wlc,
					   dmareg(DMA_TX, 2), 0,
					   NTXD, 0, 0, -1, 0, 0);
		dma_attach_err |= (NULL == wlc_hw->di[2]);
		/*
		 * FIFO 3
		 * TX: TX_AC_VO_FIFO (TX AC Voice data packets)
		 *   (legacy) TX_CTL_FIFO (TX control & mgmt packets)
		 */
		wlc_hw->di[3] = dma_attach(name, wlc,
					   dmareg(DMA_TX, 3),
					   0, NTXD, 0, 0, -1,
					   0, 0);
		dma_attach_err |= (NULL == wlc_hw->di[3]);
/* Cleaner to leave this as if with AP defined */

		if (dma_attach_err) {
			brcms_err(wlc_hw->d11core,
				  "wl%d: wlc_attach: dma_attach failed\n",
				  unit);
			return false;
		}

		/* get pointer to dma engine tx flow control variable */
		for (i = 0; i < NFIFO; i++)
			if (wlc_hw->di[i])
				wlc_hw->txavail[i] =
				    (uint *) dma_getvar(wlc_hw->di[i],
							"&txavail");
	}

	/* initial ucode host flags */
	brcms_c_mhfdef(wlc, wlc_hw->band->mhfs, pio_mhf2);

	return true;
}

static void brcms_b_detach_dmapio(struct brcms_hardware *wlc_hw)
{
	uint j;

	for (j = 0; j < NFIFO; j++) {
		if (wlc_hw->di[j]) {
			dma_detach(wlc_hw->di[j]);
			wlc_hw->di[j] = NULL;
		}
	}
}

/*
 * Initialize brcms_c_info default values ...
 * may get overrides later in this function
 *  BMAC_NOTES, move low out and resolve the dangling ones
 */
static void brcms_b_info_init(struct brcms_hardware *wlc_hw)
{
	struct brcms_c_info *wlc = wlc_hw->wlc;

	/* set default sw macintmask value */
	wlc->defmacintmask = DEF_MACINTMASK;

	/* various 802.11g modes */
	wlc_hw->shortslot = false;

	wlc_hw->SFBL = RETRY_SHORT_FB;
	wlc_hw->LFBL = RETRY_LONG_FB;

	/* default mac retry limits */
	wlc_hw->SRL = RETRY_SHORT_DEF;
	wlc_hw->LRL = RETRY_LONG_DEF;
	wlc_hw->chanspec = ch20mhz_chspec(1);
}

static void brcms_b_wait_for_wake(struct brcms_hardware *wlc_hw)
{
	/* delay before first read of ucode state */
	udelay(40);

	/* wait until ucode is no longer asleep */
	SPINWAIT((brcms_b_read_shm(wlc_hw, M_UCODE_DBGST) ==
		  DBGST_ASLEEP), wlc_hw->wlc->fastpwrup_dly);
}

/* control chip clock to save power, enable dynamic clock or force fast clock */
static void brcms_b_clkctl_clk(struct brcms_hardware *wlc_hw, enum bcma_clkmode mode)
{
	if (ai_get_cccaps(wlc_hw->sih) & CC_CAP_PMU) {
		/* new chips with PMU, CCS_FORCEHT will distribute the HT clock
		 * on backplane, but mac core will still run on ALP(not HT) when
		 * it enters powersave mode, which means the FCA bit may not be
		 * set. Should wakeup mac if driver wants it to run on HT.
		 */

		if (wlc_hw->clk) {
			if (mode == BCMA_CLKMODE_FAST) {
				bcma_set32(wlc_hw->d11core,
					   D11REGOFFS(clk_ctl_st),
					   CCS_FORCEHT);

				udelay(64);

				SPINWAIT(
				    ((bcma_read32(wlc_hw->d11core,
				      D11REGOFFS(clk_ctl_st)) &
				      CCS_HTAVAIL) == 0),
				      PMU_MAX_TRANSITION_DLY);
				WARN_ON(!(bcma_read32(wlc_hw->d11core,
					D11REGOFFS(clk_ctl_st)) &
					CCS_HTAVAIL));
			} else {
				if ((ai_get_pmurev(wlc_hw->sih) == 0) &&
				    (bcma_read32(wlc_hw->d11core,
					D11REGOFFS(clk_ctl_st)) &
					(CCS_FORCEHT | CCS_HTAREQ)))
					SPINWAIT(
					    ((bcma_read32(wlc_hw->d11core,
					      offsetof(struct d11regs,
						       clk_ctl_st)) &
					      CCS_HTAVAIL) == 0),
					      PMU_MAX_TRANSITION_DLY);
				bcma_mask32(wlc_hw->d11core,
					D11REGOFFS(clk_ctl_st),
					~CCS_FORCEHT);
			}
		}
		wlc_hw->forcefastclk = (mode == BCMA_CLKMODE_FAST);
	} else {

		/* old chips w/o PMU, force HT through cc,
		 * then use FCA to verify mac is running fast clock
		 */

		wlc_hw->forcefastclk = ai_clkctl_cc(wlc_hw->sih, mode);

		/* check fast clock is available (if core is not in reset) */
		if (wlc_hw->forcefastclk && wlc_hw->clk)
			WARN_ON(!(bcma_aread32(wlc_hw->d11core, BCMA_IOST) &
				  SISF_FCLKA));

		/*
		 * keep the ucode wake bit on if forcefastclk is on since we
		 * do not want ucode to put us back to slow clock when it dozes
		 * for PM mode. Code below matches the wake override bit with
		 * current forcefastclk state. Only setting bit in wake_override
		 * instead of waking ucode immediately since old code had this
		 * behavior. Older code set wlc->forcefastclk but only had the
		 * wake happen if the wakup_ucode work (protected by an up
		 * check) was executed just below.
		 */
		if (wlc_hw->forcefastclk)
			mboolset(wlc_hw->wake_override,
				 BRCMS_WAKE_OVERRIDE_FORCEFAST);
		else
			mboolclr(wlc_hw->wake_override,
				 BRCMS_WAKE_OVERRIDE_FORCEFAST);
	}
}

/* set or clear ucode host flag bits
 * it has an optimization for no-change write
 * it only writes through shared memory when the core has clock;
 * pre-CLK changes should use wlc_write_mhf to get around the optimization
 *
 *
 * bands values are: BRCM_BAND_AUTO <--- Current band only
 *                   BRCM_BAND_5G   <--- 5G band only
 *                   BRCM_BAND_2G   <--- 2G band only
 *                   BRCM_BAND_ALL  <--- All bands
 */
void
brcms_b_mhf(struct brcms_hardware *wlc_hw, u8 idx, u16 mask, u16 val,
	     int bands)
{
	u16 save;
	u16 addr[MHFMAX] = {
		M_HOST_FLAGS1, M_HOST_FLAGS2, M_HOST_FLAGS3, M_HOST_FLAGS4,
		M_HOST_FLAGS5
	};
	struct brcms_hw_band *band;

	if ((val & ~mask) || idx >= MHFMAX)
		return; /* error condition */

	switch (bands) {
		/* Current band only or all bands,
		 * then set the band to current band
		 */
	case BRCM_BAND_AUTO:
	case BRCM_BAND_ALL:
		band = wlc_hw->band;
		break;
	case BRCM_BAND_5G:
		band = wlc_hw->bandstate[BAND_5G_INDEX];
		break;
	case BRCM_BAND_2G:
		band = wlc_hw->bandstate[BAND_2G_INDEX];
		break;
	default:
		band = NULL;	/* error condition */
	}

	if (band) {
		save = band->mhfs[idx];
		band->mhfs[idx] = (band->mhfs[idx] & ~mask) | val;

		/* optimization: only write through if changed, and
		 * changed band is the current band
		 */
		if (wlc_hw->clk && (band->mhfs[idx] != save)
		    && (band == wlc_hw->band))
			brcms_b_write_shm(wlc_hw, addr[idx],
					   (u16) band->mhfs[idx]);
	}

	if (bands == BRCM_BAND_ALL) {
		wlc_hw->bandstate[0]->mhfs[idx] =
		    (wlc_hw->bandstate[0]->mhfs[idx] & ~mask) | val;
		wlc_hw->bandstate[1]->mhfs[idx] =
		    (wlc_hw->bandstate[1]->mhfs[idx] & ~mask) | val;
	}
}

/* set the maccontrol register to desired reset state and
 * initialize the sw cache of the register
 */
static void brcms_c_mctrl_reset(struct brcms_hardware *wlc_hw)
{
	/* IHR accesses are always enabled, PSM disabled, HPS off and WAKE on */
	wlc_hw->maccontrol = 0;
	wlc_hw->suspended_fifos = 0;
	wlc_hw->wake_override = 0;
	wlc_hw->mute_override = 0;
	brcms_b_mctrl(wlc_hw, ~0, MCTL_IHR_EN | MCTL_WAKE);
}

/*
 * write the software state of maccontrol and
 * overrides to the maccontrol register
 */
static void brcms_c_mctrl_write(struct brcms_hardware *wlc_hw)
{
	u32 maccontrol = wlc_hw->maccontrol;

	/* OR in the wake bit if overridden */
	if (wlc_hw->wake_override)
		maccontrol |= MCTL_WAKE;

	/* set AP and INFRA bits for mute if needed */
	if (wlc_hw->mute_override) {
		maccontrol &= ~(MCTL_AP);
		maccontrol |= MCTL_INFRA;
	}

	bcma_write32(wlc_hw->d11core, D11REGOFFS(maccontrol),
		     maccontrol);
}

/* set or clear maccontrol bits */
void brcms_b_mctrl(struct brcms_hardware *wlc_hw, u32 mask, u32 val)
{
	u32 maccontrol;
	u32 new_maccontrol;

	if (val & ~mask)
		return; /* error condition */
	maccontrol = wlc_hw->maccontrol;
	new_maccontrol = (maccontrol & ~mask) | val;

	/* if the new maccontrol value is the same as the old, nothing to do */
	if (new_maccontrol == maccontrol)
		return;

	/* something changed, cache the new value */
	wlc_hw->maccontrol = new_maccontrol;

	/* write the new values with overrides applied */
	brcms_c_mctrl_write(wlc_hw);
}

void brcms_c_ucode_wake_override_set(struct brcms_hardware *wlc_hw,
				 u32 override_bit)
{
	if (wlc_hw->wake_override || (wlc_hw->maccontrol & MCTL_WAKE)) {
		mboolset(wlc_hw->wake_override, override_bit);
		return;
	}

	mboolset(wlc_hw->wake_override, override_bit);

	brcms_c_mctrl_write(wlc_hw);
	brcms_b_wait_for_wake(wlc_hw);
}

void brcms_c_ucode_wake_override_clear(struct brcms_hardware *wlc_hw,
				   u32 override_bit)
{
	mboolclr(wlc_hw->wake_override, override_bit);

	if (wlc_hw->wake_override || (wlc_hw->maccontrol & MCTL_WAKE))
		return;

	brcms_c_mctrl_write(wlc_hw);
}

/* When driver needs ucode to stop beaconing, it has to make sure that
 * MCTL_AP is clear and MCTL_INFRA is set
 * Mode           MCTL_AP        MCTL_INFRA
 * AP                1              1
 * STA               0              1 <--- This will ensure no beacons
 * IBSS              0              0
 */
static void brcms_c_ucode_mute_override_set(struct brcms_hardware *wlc_hw)
{
	wlc_hw->mute_override = 1;

	/* if maccontrol already has AP == 0 and INFRA == 1 without this
	 * override, then there is no change to write
	 */
	if ((wlc_hw->maccontrol & (MCTL_AP | MCTL_INFRA)) == MCTL_INFRA)
		return;

	brcms_c_mctrl_write(wlc_hw);
}

/* Clear the override on AP and INFRA bits */
static void brcms_c_ucode_mute_override_clear(struct brcms_hardware *wlc_hw)
{
	if (wlc_hw->mute_override == 0)
		return;

	wlc_hw->mute_override = 0;

	/* if maccontrol already has AP == 0 and INFRA == 1 without this
	 * override, then there is no change to write
	 */
	if ((wlc_hw->maccontrol & (MCTL_AP | MCTL_INFRA)) == MCTL_INFRA)
		return;

	brcms_c_mctrl_write(wlc_hw);
}

/*
 * Write a MAC address to the given match reg offset in the RXE match engine.
 */
static void
brcms_b_set_addrmatch(struct brcms_hardware *wlc_hw, int match_reg_offset,
		       const u8 *addr)
{
	struct bcma_device *core = wlc_hw->d11core;
	u16 mac_l;
	u16 mac_m;
	u16 mac_h;

	brcms_dbg_rx(core, "wl%d: brcms_b_set_addrmatch\n", wlc_hw->unit);

	mac_l = addr[0] | (addr[1] << 8);
	mac_m = addr[2] | (addr[3] << 8);
	mac_h = addr[4] | (addr[5] << 8);

	/* enter the MAC addr into the RXE match registers */
	bcma_write16(core, D11REGOFFS(rcm_ctl),
		     RCM_INC_DATA | match_reg_offset);
	bcma_write16(core, D11REGOFFS(rcm_mat_data), mac_l);
	bcma_write16(core, D11REGOFFS(rcm_mat_data), mac_m);
	bcma_write16(core, D11REGOFFS(rcm_mat_data), mac_h);
}

void
brcms_b_write_template_ram(struct brcms_hardware *wlc_hw, int offset, int len,
			    void *buf)
{
	struct bcma_device *core = wlc_hw->d11core;
	u32 word;
	__le32 word_le;
	__be32 word_be;
	bool be_bit;
	brcms_dbg_info(core, "wl%d\n", wlc_hw->unit);

	bcma_write32(core, D11REGOFFS(tplatewrptr), offset);

	/* if MCTL_BIGEND bit set in mac control register,
	 * the chip swaps data in fifo, as well as data in
	 * template ram
	 */
	be_bit = (bcma_read32(core, D11REGOFFS(maccontrol)) & MCTL_BIGEND) != 0;

	while (len > 0) {
		memcpy(&word, buf, sizeof(u32));

		if (be_bit) {
			word_be = cpu_to_be32(word);
			word = *(u32 *)&word_be;
		} else {
			word_le = cpu_to_le32(word);
			word = *(u32 *)&word_le;
		}

		bcma_write32(core, D11REGOFFS(tplatewrdata), word);

		buf = (u8 *) buf + sizeof(u32);
		len -= sizeof(u32);
	}
}

static void brcms_b_set_cwmin(struct brcms_hardware *wlc_hw, u16 newmin)
{
	wlc_hw->band->CWmin = newmin;

	bcma_write32(wlc_hw->d11core, D11REGOFFS(objaddr),
		     OBJADDR_SCR_SEL | S_DOT11_CWMIN);
	(void)bcma_read32(wlc_hw->d11core, D11REGOFFS(objaddr));
	bcma_write32(wlc_hw->d11core, D11REGOFFS(objdata), newmin);
}

static void brcms_b_set_cwmax(struct brcms_hardware *wlc_hw, u16 newmax)
{
	wlc_hw->band->CWmax = newmax;

	bcma_write32(wlc_hw->d11core, D11REGOFFS(objaddr),
		     OBJADDR_SCR_SEL | S_DOT11_CWMAX);
	(void)bcma_read32(wlc_hw->d11core, D11REGOFFS(objaddr));
	bcma_write32(wlc_hw->d11core, D11REGOFFS(objdata), newmax);
}

void brcms_b_bw_set(struct brcms_hardware *wlc_hw, u16 bw)
{
	bool fastclk;

	/* request FAST clock if not on */
	fastclk = wlc_hw->forcefastclk;
	if (!fastclk)
		brcms_b_clkctl_clk(wlc_hw, BCMA_CLKMODE_FAST);

	wlc_phy_bw_state_set(wlc_hw->band->pi, bw);

	brcms_b_phy_reset(wlc_hw);
	wlc_phy_init(wlc_hw->band->pi, wlc_phy_chanspec_get(wlc_hw->band->pi));

	/* restore the clk */
	if (!fastclk)
		brcms_b_clkctl_clk(wlc_hw, BCMA_CLKMODE_DYNAMIC);
}

static void brcms_b_upd_synthpu(struct brcms_hardware *wlc_hw)
{
	u16 v;
	struct brcms_c_info *wlc = wlc_hw->wlc;
	/* update SYNTHPU_DLY */

	if (BRCMS_ISLCNPHY(wlc->band))
		v = SYNTHPU_DLY_LPPHY_US;
	else if (BRCMS_ISNPHY(wlc->band) && (NREV_GE(wlc->band->phyrev, 3)))
		v = SYNTHPU_DLY_NPHY_US;
	else
		v = SYNTHPU_DLY_BPHY_US;

	brcms_b_write_shm(wlc_hw, M_SYNTHPU_DLY, v);
}

static void brcms_c_ucode_txant_set(struct brcms_hardware *wlc_hw)
{
	u16 phyctl;
	u16 phytxant = wlc_hw->bmac_phytxant;
	u16 mask = PHY_TXC_ANT_MASK;

	/* set the Probe Response frame phy control word */
	phyctl = brcms_b_read_shm(wlc_hw, M_CTXPRS_BLK + C_CTX_PCTLWD_POS);
	phyctl = (phyctl & ~mask) | phytxant;
	brcms_b_write_shm(wlc_hw, M_CTXPRS_BLK + C_CTX_PCTLWD_POS, phyctl);

	/* set the Response (ACK/CTS) frame phy control word */
	phyctl = brcms_b_read_shm(wlc_hw, M_RSP_PCTLWD);
	phyctl = (phyctl & ~mask) | phytxant;
	brcms_b_write_shm(wlc_hw, M_RSP_PCTLWD, phyctl);
}

static u16 brcms_b_ofdm_ratetable_offset(struct brcms_hardware *wlc_hw,
					 u8 rate)
{
	uint i;
	u8 plcp_rate = 0;
	struct plcp_signal_rate_lookup {
		u8 rate;
		u8 signal_rate;
	};
	/* OFDM RATE sub-field of PLCP SIGNAL field, per 802.11 sec 17.3.4.1 */
	const struct plcp_signal_rate_lookup rate_lookup[] = {
		{BRCM_RATE_6M, 0xB},
		{BRCM_RATE_9M, 0xF},
		{BRCM_RATE_12M, 0xA},
		{BRCM_RATE_18M, 0xE},
		{BRCM_RATE_24M, 0x9},
		{BRCM_RATE_36M, 0xD},
		{BRCM_RATE_48M, 0x8},
		{BRCM_RATE_54M, 0xC}
	};

	for (i = 0; i < ARRAY_SIZE(rate_lookup); i++) {
		if (rate == rate_lookup[i].rate) {
			plcp_rate = rate_lookup[i].signal_rate;
			break;
		}
	}

	/* Find the SHM pointer to the rate table entry by looking in the
	 * Direct-map Table
	 */
	return 2 * brcms_b_read_shm(wlc_hw, M_RT_DIRMAP_A + (plcp_rate * 2));
}

static void brcms_upd_ofdm_pctl1_table(struct brcms_hardware *wlc_hw)
{
	u8 rate;
	u8 rates[8] = {
		BRCM_RATE_6M, BRCM_RATE_9M, BRCM_RATE_12M, BRCM_RATE_18M,
		BRCM_RATE_24M, BRCM_RATE_36M, BRCM_RATE_48M, BRCM_RATE_54M
	};
	u16 entry_ptr;
	u16 pctl1;
	uint i;

	if (!BRCMS_PHY_11N_CAP(wlc_hw->band))
		return;

	/* walk the phy rate table and update the entries */
	for (i = 0; i < ARRAY_SIZE(rates); i++) {
		rate = rates[i];

		entry_ptr = brcms_b_ofdm_ratetable_offset(wlc_hw, rate);

		/* read the SHM Rate Table entry OFDM PCTL1 values */
		pctl1 =
		    brcms_b_read_shm(wlc_hw, entry_ptr + M_RT_OFDM_PCTL1_POS);

		/* modify the value */
		pctl1 &= ~PHY_TXC1_MODE_MASK;
		pctl1 |= (wlc_hw->hw_stf_ss_opmode << PHY_TXC1_MODE_SHIFT);

		/* Update the SHM Rate Table entry OFDM PCTL1 values */
		brcms_b_write_shm(wlc_hw, entry_ptr + M_RT_OFDM_PCTL1_POS,
				   pctl1);
	}
}

/* band-specific init */
static void brcms_b_bsinit(struct brcms_c_info *wlc, u16 chanspec)
{
	struct brcms_hardware *wlc_hw = wlc->hw;

	brcms_dbg_mac80211(wlc_hw->d11core, "wl%d: bandunit %d\n", wlc_hw->unit,
			   wlc_hw->band->bandunit);

	brcms_c_ucode_bsinit(wlc_hw);

	wlc_phy_init(wlc_hw->band->pi, chanspec);

	brcms_c_ucode_txant_set(wlc_hw);

	/*
	 * cwmin is band-specific, update hardware
	 * with value for current band
	 */
	brcms_b_set_cwmin(wlc_hw, wlc_hw->band->CWmin);
	brcms_b_set_cwmax(wlc_hw, wlc_hw->band->CWmax);

	brcms_b_update_slot_timing(wlc_hw,
				   wlc_hw->band->bandtype == BRCM_BAND_5G ?
				   true : wlc_hw->shortslot);

	/* write phytype and phyvers */
	brcms_b_write_shm(wlc_hw, M_PHYTYPE, (u16) wlc_hw->band->phytype);
	brcms_b_write_shm(wlc_hw, M_PHYVER, (u16) wlc_hw->band->phyrev);

	/*
	 * initialize the txphyctl1 rate table since
	 * shmem is shared between bands
	 */
	brcms_upd_ofdm_pctl1_table(wlc_hw);

	brcms_b_upd_synthpu(wlc_hw);
}

/* Perform a soft reset of the PHY PLL */
void brcms_b_core_phypll_reset(struct brcms_hardware *wlc_hw)
{
	ai_cc_reg(wlc_hw->sih, offsetof(struct chipcregs, chipcontrol_addr),
		  ~0, 0);
	udelay(1);
	ai_cc_reg(wlc_hw->sih, offsetof(struct chipcregs, chipcontrol_data),
		  0x4, 0);
	udelay(1);
	ai_cc_reg(wlc_hw->sih, offsetof(struct chipcregs, chipcontrol_data),
		  0x4, 4);
	udelay(1);
	ai_cc_reg(wlc_hw->sih, offsetof(struct chipcregs, chipcontrol_data),
		  0x4, 0);
	udelay(1);
}

/* light way to turn on phy clock without reset for NPHY only
 *  refer to brcms_b_core_phy_clk for full version
 */
void brcms_b_phyclk_fgc(struct brcms_hardware *wlc_hw, bool clk)
{
	/* support(necessary for NPHY and HYPHY) only */
	if (!BRCMS_ISNPHY(wlc_hw->band))
		return;

	if (ON == clk)
		brcms_b_core_ioctl(wlc_hw, SICF_FGC, SICF_FGC);
	else
		brcms_b_core_ioctl(wlc_hw, SICF_FGC, 0);

}

void brcms_b_macphyclk_set(struct brcms_hardware *wlc_hw, bool clk)
{
	if (ON == clk)
		brcms_b_core_ioctl(wlc_hw, SICF_MPCLKE, SICF_MPCLKE);
	else
		brcms_b_core_ioctl(wlc_hw, SICF_MPCLKE, 0);
}

void brcms_b_phy_reset(struct brcms_hardware *wlc_hw)
{
	struct brcms_phy_pub *pih = wlc_hw->band->pi;
	u32 phy_bw_clkbits;
	bool phy_in_reset = false;

	brcms_dbg_info(wlc_hw->d11core, "wl%d: reset phy\n", wlc_hw->unit);

	if (pih == NULL)
		return;

	phy_bw_clkbits = wlc_phy_clk_bwbits(wlc_hw->band->pi);

	/* Specific reset sequence required for NPHY rev 3 and 4 */
	if (BRCMS_ISNPHY(wlc_hw->band) && NREV_GE(wlc_hw->band->phyrev, 3) &&
	    NREV_LE(wlc_hw->band->phyrev, 4)) {
		/* Set the PHY bandwidth */
		brcms_b_core_ioctl(wlc_hw, SICF_BWMASK, phy_bw_clkbits);

		udelay(1);

		/* Perform a soft reset of the PHY PLL */
		brcms_b_core_phypll_reset(wlc_hw);

		/* reset the PHY */
		brcms_b_core_ioctl(wlc_hw, (SICF_PRST | SICF_PCLKE),
				   (SICF_PRST | SICF_PCLKE));
		phy_in_reset = true;
	} else {
		brcms_b_core_ioctl(wlc_hw,
				   (SICF_PRST | SICF_PCLKE | SICF_BWMASK),
				   (SICF_PRST | SICF_PCLKE | phy_bw_clkbits));
	}

	udelay(2);
	brcms_b_core_phy_clk(wlc_hw, ON);

	if (pih)
		wlc_phy_anacore(pih, ON);
}

/* switch to and initialize new band */
static void brcms_b_setband(struct brcms_hardware *wlc_hw, uint bandunit,
			    u16 chanspec) {
	struct brcms_c_info *wlc = wlc_hw->wlc;
	u32 macintmask;

	/* Enable the d11 core before accessing it */
	if (!bcma_core_is_enabled(wlc_hw->d11core)) {
		bcma_core_enable(wlc_hw->d11core, 0);
		brcms_c_mctrl_reset(wlc_hw);
	}

	macintmask = brcms_c_setband_inact(wlc, bandunit);

	if (!wlc_hw->up)
		return;

	brcms_b_core_phy_clk(wlc_hw, ON);

	/* band-specific initializations */
	brcms_b_bsinit(wlc, chanspec);

	/*
	 * If there are any pending software interrupt bits,
	 * then replace these with a harmless nonzero value
	 * so brcms_c_dpc() will re-enable interrupts when done.
	 */
	if (wlc->macintstatus)
		wlc->macintstatus = MI_DMAINT;

	/* restore macintmask */
	brcms_intrsrestore(wlc->wl, macintmask);

	/* ucode should still be suspended.. */
	WARN_ON((bcma_read32(wlc_hw->d11core, D11REGOFFS(maccontrol)) &
		 MCTL_EN_MAC) != 0);
}

static bool brcms_c_isgoodchip(struct brcms_hardware *wlc_hw)
{

	/* reject unsupported corerev */
	if (!CONF_HAS(D11CONF, wlc_hw->corerev)) {
		wiphy_err(wlc_hw->wlc->wiphy, "unsupported core rev %d\n",
			  wlc_hw->corerev);
		return false;
	}

	return true;
}

/* Validate some board info parameters */
static bool brcms_c_validboardtype(struct brcms_hardware *wlc_hw)
{
	uint boardrev = wlc_hw->boardrev;

	/* 4 bits each for board type, major, minor, and tiny version */
	uint brt = (boardrev & 0xf000) >> 12;
	uint b0 = (boardrev & 0xf00) >> 8;
	uint b1 = (boardrev & 0xf0) >> 4;
	uint b2 = boardrev & 0xf;

	/* voards from other vendors are always considered valid */
	if (ai_get_boardvendor(wlc_hw->sih) != PCI_VENDOR_ID_BROADCOM)
		return true;

	/* do some boardrev sanity checks when boardvendor is Broadcom */
	if (boardrev == 0)
		return false;

	if (boardrev <= 0xff)
		return true;

	if ((brt > 2) || (brt == 0) || (b0 > 9) || (b0 == 0) || (b1 > 9)
		|| (b2 > 9))
		return false;

	return true;
}

static void brcms_c_get_macaddr(struct brcms_hardware *wlc_hw, u8 etheraddr[ETH_ALEN])
{
	struct ssb_sprom *sprom = &wlc_hw->d11core->bus->sprom;

	/* If macaddr exists, use it (Sromrev4, CIS, ...). */
	if (!is_zero_ether_addr(sprom->il0mac)) {
		memcpy(etheraddr, sprom->il0mac, ETH_ALEN);
		return;
	}

	if (wlc_hw->_nbands > 1)
		memcpy(etheraddr, sprom->et1mac, ETH_ALEN);
	else
		memcpy(etheraddr, sprom->il0mac, ETH_ALEN);
}

/* power both the pll and external oscillator on/off */
static void brcms_b_xtal(struct brcms_hardware *wlc_hw, bool want)
{
	brcms_dbg_info(wlc_hw->d11core, "wl%d: want %d\n", wlc_hw->unit, want);

	/*
	 * dont power down if plldown is false or
	 * we must poll hw radio disable
	 */
	if (!want && wlc_hw->pllreq)
		return;

	wlc_hw->sbclk = want;
	if (!wlc_hw->sbclk) {
		wlc_hw->clk = false;
		if (wlc_hw->band && wlc_hw->band->pi)
			wlc_phy_hw_clk_state_upd(wlc_hw->band->pi, false);
	}
}

/*
 * Return true if radio is disabled, otherwise false.
 * hw radio disable signal is an external pin, users activate it asynchronously
 * this function could be called when driver is down and w/o clock
 * it operates on different registers depending on corerev and boardflag.
 */
static bool brcms_b_radio_read_hwdisabled(struct brcms_hardware *wlc_hw)
{
	bool v, clk, xtal;
	u32 flags = 0;

	xtal = wlc_hw->sbclk;
	if (!xtal)
		brcms_b_xtal(wlc_hw, ON);

	/* may need to take core out of reset first */
	clk = wlc_hw->clk;
	if (!clk) {
		/*
		 * mac no longer enables phyclk automatically when driver
		 * accesses phyreg throughput mac. This can be skipped since
		 * only mac reg is accessed below
		 */
		if (D11REV_GE(wlc_hw->corerev, 18))
			flags |= SICF_PCLKE;

		/*
		 * TODO: test suspend/resume
		 *
		 * AI chip doesn't restore bar0win2 on
		 * hibernation/resume, need sw fixup
		 */

		bcma_core_enable(wlc_hw->d11core, flags);
		brcms_c_mctrl_reset(wlc_hw);
	}

	v = ((bcma_read32(wlc_hw->d11core,
			  D11REGOFFS(phydebug)) & PDBG_RFD) != 0);

	/* put core back into reset */
	if (!clk)
		bcma_core_disable(wlc_hw->d11core, 0);

	if (!xtal)
		brcms_b_xtal(wlc_hw, OFF);

	return v;
}

static bool wlc_dma_rxreset(struct brcms_hardware *wlc_hw, uint fifo)
{
	struct dma_pub *di = wlc_hw->di[fifo];
	return dma_rxreset(di);
}

/* d11 core reset
 *   ensure fask clock during reset
 *   reset dma
 *   reset d11(out of reset)
 *   reset phy(out of reset)
 *   clear software macintstatus for fresh new start
 * one testing hack wlc_hw->noreset will bypass the d11/phy reset
 */
void brcms_b_corereset(struct brcms_hardware *wlc_hw, u32 flags)
{
	uint i;
	bool fastclk;

	if (flags == BRCMS_USE_COREFLAGS)
		flags = (wlc_hw->band->pi ? wlc_hw->band->core_flags : 0);

	brcms_dbg_info(wlc_hw->d11core, "wl%d: core reset\n", wlc_hw->unit);

	/* request FAST clock if not on  */
	fastclk = wlc_hw->forcefastclk;
	if (!fastclk)
		brcms_b_clkctl_clk(wlc_hw, BCMA_CLKMODE_FAST);

	/* reset the dma engines except first time thru */
	if (bcma_core_is_enabled(wlc_hw->d11core)) {
		for (i = 0; i < NFIFO; i++)
			if ((wlc_hw->di[i]) && (!dma_txreset(wlc_hw->di[i])))
				brcms_err(wlc_hw->d11core, "wl%d: %s: "
					  "dma_txreset[%d]: cannot stop dma\n",
					   wlc_hw->unit, __func__, i);

		if ((wlc_hw->di[RX_FIFO])
		    && (!wlc_dma_rxreset(wlc_hw, RX_FIFO)))
			brcms_err(wlc_hw->d11core, "wl%d: %s: dma_rxreset"
				  "[%d]: cannot stop dma\n",
				  wlc_hw->unit, __func__, RX_FIFO);
	}
	/* if noreset, just stop the psm and return */
	if (wlc_hw->noreset) {
		wlc_hw->wlc->macintstatus = 0;	/* skip wl_dpc after down */
		brcms_b_mctrl(wlc_hw, MCTL_PSM_RUN | MCTL_EN_MAC, 0);
		return;
	}

	/*
	 * mac no longer enables phyclk automatically when driver accesses
	 * phyreg throughput mac, AND phy_reset is skipped at early stage when
	 * band->pi is invalid. need to enable PHY CLK
	 */
	if (D11REV_GE(wlc_hw->corerev, 18))
		flags |= SICF_PCLKE;

	/*
	 * reset the core
	 * In chips with PMU, the fastclk request goes through d11 core
	 * reg 0x1e0, which is cleared by the core_reset. have to re-request it.
	 *
	 * This adds some delay and we can optimize it by also requesting
	 * fastclk through chipcommon during this period if necessary. But
	 * that has to work coordinate with other driver like mips/arm since
	 * they may touch chipcommon as well.
	 */
	wlc_hw->clk = false;
	bcma_core_enable(wlc_hw->d11core, flags);
	wlc_hw->clk = true;
	if (wlc_hw->band && wlc_hw->band->pi)
		wlc_phy_hw_clk_state_upd(wlc_hw->band->pi, true);

	brcms_c_mctrl_reset(wlc_hw);

	if (ai_get_cccaps(wlc_hw->sih) & CC_CAP_PMU)
		brcms_b_clkctl_clk(wlc_hw, BCMA_CLKMODE_FAST);

	brcms_b_phy_reset(wlc_hw);

	/* turn on PHY_PLL */
	brcms_b_core_phypll_ctl(wlc_hw, true);

	/* clear sw intstatus */
	wlc_hw->wlc->macintstatus = 0;

	/* restore the clk setting */
	if (!fastclk)
		brcms_b_clkctl_clk(wlc_hw, BCMA_CLKMODE_DYNAMIC);
}

/* txfifo sizes needs to be modified(increased) since the newer cores
 * have more memory.
 */
static void brcms_b_corerev_fifofixup(struct brcms_hardware *wlc_hw)
{
	struct bcma_device *core = wlc_hw->d11core;
	u16 fifo_nu;
	u16 txfifo_startblk = TXFIFO_START_BLK, txfifo_endblk;
	u16 txfifo_def, txfifo_def1;
	u16 txfifo_cmd;

	/* tx fifos start at TXFIFO_START_BLK from the Base address */
	txfifo_startblk = TXFIFO_START_BLK;

	/* sequence of operations:  reset fifo, set fifo size, reset fifo */
	for (fifo_nu = 0; fifo_nu < NFIFO; fifo_nu++) {

		txfifo_endblk = txfifo_startblk + wlc_hw->xmtfifo_sz[fifo_nu];
		txfifo_def = (txfifo_startblk & 0xff) |
		    (((txfifo_endblk - 1) & 0xff) << TXFIFO_FIFOTOP_SHIFT);
		txfifo_def1 = ((txfifo_startblk >> 8) & 0x1) |
		    ((((txfifo_endblk -
			1) >> 8) & 0x1) << TXFIFO_FIFOTOP_SHIFT);
		txfifo_cmd =
		    TXFIFOCMD_RESET_MASK | (fifo_nu << TXFIFOCMD_FIFOSEL_SHIFT);

		bcma_write16(core, D11REGOFFS(xmtfifocmd), txfifo_cmd);
		bcma_write16(core, D11REGOFFS(xmtfifodef), txfifo_def);
		bcma_write16(core, D11REGOFFS(xmtfifodef1), txfifo_def1);

		bcma_write16(core, D11REGOFFS(xmtfifocmd), txfifo_cmd);

		txfifo_startblk += wlc_hw->xmtfifo_sz[fifo_nu];
	}
	/*
	 * need to propagate to shm location to be in sync since ucode/hw won't
	 * do this
	 */
	brcms_b_write_shm(wlc_hw, M_FIFOSIZE0,
			   wlc_hw->xmtfifo_sz[TX_AC_BE_FIFO]);
	brcms_b_write_shm(wlc_hw, M_FIFOSIZE1,
			   wlc_hw->xmtfifo_sz[TX_AC_VI_FIFO]);
	brcms_b_write_shm(wlc_hw, M_FIFOSIZE2,
			   ((wlc_hw->xmtfifo_sz[TX_AC_VO_FIFO] << 8) | wlc_hw->
			    xmtfifo_sz[TX_AC_BK_FIFO]));
	brcms_b_write_shm(wlc_hw, M_FIFOSIZE3,
			   ((wlc_hw->xmtfifo_sz[TX_ATIM_FIFO] << 8) | wlc_hw->
			    xmtfifo_sz[TX_BCMC_FIFO]));
}

/* This function is used for changing the tsf frac register
 * If spur avoidance mode is off, the mac freq will be 80/120/160Mhz
 * If spur avoidance mode is on1, the mac freq will be 82/123/164Mhz
 * If spur avoidance mode is on2, the mac freq will be 84/126/168Mhz
 * HTPHY Formula is 2^26/freq(MHz) e.g.
 * For spuron2 - 126MHz -> 2^26/126 = 532610.0
 *  - 532610 = 0x82082 => tsf_clk_frac_h = 0x8, tsf_clk_frac_l = 0x2082
 * For spuron: 123MHz -> 2^26/123    = 545600.5
 *  - 545601 = 0x85341 => tsf_clk_frac_h = 0x8, tsf_clk_frac_l = 0x5341
 * For spur off: 120MHz -> 2^26/120    = 559240.5
 *  - 559241 = 0x88889 => tsf_clk_frac_h = 0x8, tsf_clk_frac_l = 0x8889
 */

void brcms_b_switch_macfreq(struct brcms_hardware *wlc_hw, u8 spurmode)
{
	struct bcma_device *core = wlc_hw->d11core;

	if ((ai_get_chip_id(wlc_hw->sih) == BCMA_CHIP_ID_BCM43224) ||
	    (ai_get_chip_id(wlc_hw->sih) == BCMA_CHIP_ID_BCM43225)) {
		if (spurmode == WL_SPURAVOID_ON2) {	/* 126Mhz */
			bcma_write16(core, D11REGOFFS(tsf_clk_frac_l), 0x2082);
			bcma_write16(core, D11REGOFFS(tsf_clk_frac_h), 0x8);
		} else if (spurmode == WL_SPURAVOID_ON1) {	/* 123Mhz */
			bcma_write16(core, D11REGOFFS(tsf_clk_frac_l), 0x5341);
			bcma_write16(core, D11REGOFFS(tsf_clk_frac_h), 0x8);
		} else {	/* 120Mhz */
			bcma_write16(core, D11REGOFFS(tsf_clk_frac_l), 0x8889);
			bcma_write16(core, D11REGOFFS(tsf_clk_frac_h), 0x8);
		}
	} else if (BRCMS_ISLCNPHY(wlc_hw->band)) {
		if (spurmode == WL_SPURAVOID_ON1) {	/* 82Mhz */
			bcma_write16(core, D11REGOFFS(tsf_clk_frac_l), 0x7CE0);
			bcma_write16(core, D11REGOFFS(tsf_clk_frac_h), 0xC);
		} else {	/* 80Mhz */
			bcma_write16(core, D11REGOFFS(tsf_clk_frac_l), 0xCCCD);
			bcma_write16(core, D11REGOFFS(tsf_clk_frac_h), 0xC);
		}
	}
}

void brcms_c_start_station(struct brcms_c_info *wlc, u8 *addr)
{
	memcpy(wlc->pub->cur_etheraddr, addr, sizeof(wlc->pub->cur_etheraddr));
	wlc->bsscfg->type = BRCMS_TYPE_STATION;
}

void brcms_c_start_ap(struct brcms_c_info *wlc, u8 *addr, const u8 *bssid,
		      u8 *ssid, size_t ssid_len)
{
	brcms_c_set_ssid(wlc, ssid, ssid_len);

	memcpy(wlc->pub->cur_etheraddr, addr, sizeof(wlc->pub->cur_etheraddr));
	memcpy(wlc->bsscfg->BSSID, bssid, sizeof(wlc->bsscfg->BSSID));
	wlc->bsscfg->type = BRCMS_TYPE_AP;

	brcms_b_mctrl(wlc->hw, MCTL_AP | MCTL_INFRA, MCTL_AP | MCTL_INFRA);
}

void brcms_c_start_adhoc(struct brcms_c_info *wlc, u8 *addr)
{
	memcpy(wlc->pub->cur_etheraddr, addr, sizeof(wlc->pub->cur_etheraddr));
	wlc->bsscfg->type = BRCMS_TYPE_ADHOC;

	brcms_b_mctrl(wlc->hw, MCTL_AP | MCTL_INFRA, 0);
}

/* Initialize GPIOs that are controlled by D11 core */
static void brcms_c_gpio_init(struct brcms_c_info *wlc)
{
	struct brcms_hardware *wlc_hw = wlc->hw;
	u32 gc, gm;

	/* use GPIO select 0 to get all gpio signals from the gpio out reg */
	brcms_b_mctrl(wlc_hw, MCTL_GPOUT_SEL_MASK, 0);

	/*
	 * Common GPIO setup:
	 *      G0 = LED 0 = WLAN Activity
	 *      G1 = LED 1 = WLAN 2.4 GHz Radio State
	 *      G2 = LED 2 = WLAN 5 GHz Radio State
	 *      G4 = radio disable input (HI enabled, LO disabled)
	 */

	gc = gm = 0;

	/* Allocate GPIOs for mimo antenna diversity feature */
	if (wlc_hw->antsel_type == ANTSEL_2x3) {
		/* Enable antenna diversity, use 2x3 mode */
		brcms_b_mhf(wlc_hw, MHF3, MHF3_ANTSEL_EN,
			     MHF3_ANTSEL_EN, BRCM_BAND_ALL);
		brcms_b_mhf(wlc_hw, MHF3, MHF3_ANTSEL_MODE,
			     MHF3_ANTSEL_MODE, BRCM_BAND_ALL);

		/* init superswitch control */
		wlc_phy_antsel_init(wlc_hw->band->pi, false);

	} else if (wlc_hw->antsel_type == ANTSEL_2x4) {
		gm |= gc |= (BOARD_GPIO_12 | BOARD_GPIO_13);
		/*
		 * The board itself is powered by these GPIOs
		 * (when not sending pattern) so set them high
		 */
		bcma_set16(wlc_hw->d11core, D11REGOFFS(psm_gpio_oe),
			   (BOARD_GPIO_12 | BOARD_GPIO_13));
		bcma_set16(wlc_hw->d11core, D11REGOFFS(psm_gpio_out),
			   (BOARD_GPIO_12 | BOARD_GPIO_13));

		/* Enable antenna diversity, use 2x4 mode */
		brcms_b_mhf(wlc_hw, MHF3, MHF3_ANTSEL_EN,
			     MHF3_ANTSEL_EN, BRCM_BAND_ALL);
		brcms_b_mhf(wlc_hw, MHF3, MHF3_ANTSEL_MODE, 0,
			     BRCM_BAND_ALL);

		/* Configure the desired clock to be 4Mhz */
		brcms_b_write_shm(wlc_hw, M_ANTSEL_CLKDIV,
				   ANTSEL_CLKDIV_4MHZ);
	}

	/*
	 * gpio 9 controls the PA. ucode is responsible
	 * for wiggling out and oe
	 */
	if (wlc_hw->boardflags & BFL_PACTRL)
		gm |= gc |= BOARD_GPIO_PACTRL;

	/* apply to gpiocontrol register */
	bcma_chipco_gpio_control(&wlc_hw->d11core->bus->drv_cc, gm, gc);
}

static void brcms_ucode_write(struct brcms_hardware *wlc_hw,
			      const __le32 ucode[], const size_t nbytes)
{
	struct bcma_device *core = wlc_hw->d11core;
	uint i;
	uint count;

	brcms_dbg_info(wlc_hw->d11core, "wl%d\n", wlc_hw->unit);

	count = (nbytes / sizeof(u32));

	bcma_write32(core, D11REGOFFS(objaddr),
		     OBJADDR_AUTO_INC | OBJADDR_UCM_SEL);
	(void)bcma_read32(core, D11REGOFFS(objaddr));
	for (i = 0; i < count; i++)
		bcma_write32(core, D11REGOFFS(objdata), le32_to_cpu(ucode[i]));

}

static void brcms_ucode_download(struct brcms_hardware *wlc_hw)
{
	struct brcms_c_info *wlc;
	struct brcms_ucode *ucode = &wlc_hw->wlc->wl->ucode;

	wlc = wlc_hw->wlc;

	if (wlc_hw->ucode_loaded)
		return;

	if (D11REV_IS(wlc_hw->corerev, 17) || D11REV_IS(wlc_hw->corerev, 23)) {
		if (BRCMS_ISNPHY(wlc_hw->band)) {
			brcms_ucode_write(wlc_hw, ucode->bcm43xx_16_mimo,
					  ucode->bcm43xx_16_mimosz);
			wlc_hw->ucode_loaded = true;
		} else
			brcms_err(wlc_hw->d11core,
				  "%s: wl%d: unsupported phy in corerev %d\n",
				  __func__, wlc_hw->unit, wlc_hw->corerev);
	} else if (D11REV_IS(wlc_hw->corerev, 24)) {
		if (BRCMS_ISLCNPHY(wlc_hw->band)) {
			brcms_ucode_write(wlc_hw, ucode->bcm43xx_24_lcn,
					  ucode->bcm43xx_24_lcnsz);
			wlc_hw->ucode_loaded = true;
		} else {
			brcms_err(wlc_hw->d11core,
				  "%s: wl%d: unsupported phy in corerev %d\n",
				  __func__, wlc_hw->unit, wlc_hw->corerev);
		}
	}
}

void brcms_b_txant_set(struct brcms_hardware *wlc_hw, u16 phytxant)
{
	/* update sw state */
	wlc_hw->bmac_phytxant = phytxant;

	/* push to ucode if up */
	if (!wlc_hw->up)
		return;
	brcms_c_ucode_txant_set(wlc_hw);

}

u16 brcms_b_get_txant(struct brcms_hardware *wlc_hw)
{
	return (u16) wlc_hw->wlc->stf->txant;
}

void brcms_b_antsel_type_set(struct brcms_hardware *wlc_hw, u8 antsel_type)
{
	wlc_hw->antsel_type = antsel_type;

	/* Update the antsel type for phy module to use */
	wlc_phy_antsel_type_set(wlc_hw->band->pi, antsel_type);
}

static void brcms_b_fifoerrors(struct brcms_hardware *wlc_hw)
{
	bool fatal = false;
	uint unit;
	uint intstatus, idx;
	struct bcma_device *core = wlc_hw->d11core;

	unit = wlc_hw->unit;

	for (idx = 0; idx < NFIFO; idx++) {
		/* read intstatus register and ignore any non-error bits */
		intstatus =
			bcma_read32(core,
				    D11REGOFFS(intctrlregs[idx].intstatus)) &
			I_ERRORS;
		if (!intstatus)
			continue;

		brcms_dbg_int(core, "wl%d: intstatus%d 0x%x\n",
			      unit, idx, intstatus);

		if (intstatus & I_RO) {
			brcms_err(core, "wl%d: fifo %d: receive fifo "
				  "overflow\n", unit, idx);
			fatal = true;
		}

		if (intstatus & I_PC) {
			brcms_err(core, "wl%d: fifo %d: descriptor error\n",
				  unit, idx);
			fatal = true;
		}

		if (intstatus & I_PD) {
			brcms_err(core, "wl%d: fifo %d: data error\n", unit,
				  idx);
			fatal = true;
		}

		if (intstatus & I_DE) {
			brcms_err(core, "wl%d: fifo %d: descriptor protocol "
				  "error\n", unit, idx);
			fatal = true;
		}

		if (intstatus & I_RU)
			brcms_err(core, "wl%d: fifo %d: receive descriptor "
				  "underflow\n", idx, unit);

		if (intstatus & I_XU) {
			brcms_err(core, "wl%d: fifo %d: transmit fifo "
				  "underflow\n", idx, unit);
			fatal = true;
		}

		if (fatal) {
			brcms_fatal_error(wlc_hw->wlc->wl); /* big hammer */
			break;
		} else
			bcma_write32(core,
				     D11REGOFFS(intctrlregs[idx].intstatus),
				     intstatus);
	}
}

void brcms_c_intrson(struct brcms_c_info *wlc)
{
	struct brcms_hardware *wlc_hw = wlc->hw;
	wlc->macintmask = wlc->defmacintmask;
	bcma_write32(wlc_hw->d11core, D11REGOFFS(macintmask), wlc->macintmask);
}

u32 brcms_c_intrsoff(struct brcms_c_info *wlc)
{
	struct brcms_hardware *wlc_hw = wlc->hw;
	u32 macintmask;

	if (!wlc_hw->clk)
		return 0;

	macintmask = wlc->macintmask;	/* isr can still happen */

	bcma_write32(wlc_hw->d11core, D11REGOFFS(macintmask), 0);
	(void)bcma_read32(wlc_hw->d11core, D11REGOFFS(macintmask));
	udelay(1);		/* ensure int line is no longer driven */
	wlc->macintmask = 0;

	/* return previous macintmask; resolve race between us and our isr */
	return wlc->macintstatus ? 0 : macintmask;
}

void brcms_c_intrsrestore(struct brcms_c_info *wlc, u32 macintmask)
{
	struct brcms_hardware *wlc_hw = wlc->hw;
	if (!wlc_hw->clk)
		return;

	wlc->macintmask = macintmask;
	bcma_write32(wlc_hw->d11core, D11REGOFFS(macintmask), wlc->macintmask);
}

/* assumes that the d11 MAC is enabled */
static void brcms_b_tx_fifo_suspend(struct brcms_hardware *wlc_hw,
				    uint tx_fifo)
{
	u8 fifo = 1 << tx_fifo;

	/* Two clients of this code, 11h Quiet period and scanning. */

	/* only suspend if not already suspended */
	if ((wlc_hw->suspended_fifos & fifo) == fifo)
		return;

	/* force the core awake only if not already */
	if (wlc_hw->suspended_fifos == 0)
		brcms_c_ucode_wake_override_set(wlc_hw,
						BRCMS_WAKE_OVERRIDE_TXFIFO);

	wlc_hw->suspended_fifos |= fifo;

	if (wlc_hw->di[tx_fifo]) {
		/*
		 * Suspending AMPDU transmissions in the middle can cause
		 * underflow which may result in mismatch between ucode and
		 * driver so suspend the mac before suspending the FIFO
		 */
		if (BRCMS_PHY_11N_CAP(wlc_hw->band))
			brcms_c_suspend_mac_and_wait(wlc_hw->wlc);

		dma_txsuspend(wlc_hw->di[tx_fifo]);

		if (BRCMS_PHY_11N_CAP(wlc_hw->band))
			brcms_c_enable_mac(wlc_hw->wlc);
	}
}

static void brcms_b_tx_fifo_resume(struct brcms_hardware *wlc_hw,
				   uint tx_fifo)
{
	/* BMAC_NOTE: BRCMS_TX_FIFO_ENAB is done in brcms_c_dpc() for DMA case
	 * but need to be done here for PIO otherwise the watchdog will catch
	 * the inconsistency and fire
	 */
	/* Two clients of this code, 11h Quiet period and scanning. */
	if (wlc_hw->di[tx_fifo])
		dma_txresume(wlc_hw->di[tx_fifo]);

	/* allow core to sleep again */
	if (wlc_hw->suspended_fifos == 0)
		return;
	else {
		wlc_hw->suspended_fifos &= ~(1 << tx_fifo);
		if (wlc_hw->suspended_fifos == 0)
			brcms_c_ucode_wake_override_clear(wlc_hw,
						BRCMS_WAKE_OVERRIDE_TXFIFO);
	}
}

/* precondition: requires the mac core to be enabled */
static void brcms_b_mute(struct brcms_hardware *wlc_hw, bool mute_tx)
{
	static const u8 null_ether_addr[ETH_ALEN] = {0, 0, 0, 0, 0, 0};
	u8 *ethaddr = wlc_hw->wlc->pub->cur_etheraddr;

	if (mute_tx) {
		/* suspend tx fifos */
		brcms_b_tx_fifo_suspend(wlc_hw, TX_DATA_FIFO);
		brcms_b_tx_fifo_suspend(wlc_hw, TX_CTL_FIFO);
		brcms_b_tx_fifo_suspend(wlc_hw, TX_AC_BK_FIFO);
		brcms_b_tx_fifo_suspend(wlc_hw, TX_AC_VI_FIFO);

		/* zero the address match register so we do not send ACKs */
		brcms_b_set_addrmatch(wlc_hw, RCM_MAC_OFFSET, null_ether_addr);
	} else {
		/* resume tx fifos */
		brcms_b_tx_fifo_resume(wlc_hw, TX_DATA_FIFO);
		brcms_b_tx_fifo_resume(wlc_hw, TX_CTL_FIFO);
		brcms_b_tx_fifo_resume(wlc_hw, TX_AC_BK_FIFO);
		brcms_b_tx_fifo_resume(wlc_hw, TX_AC_VI_FIFO);

		/* Restore address */
		brcms_b_set_addrmatch(wlc_hw, RCM_MAC_OFFSET, ethaddr);
	}

	wlc_phy_mute_upd(wlc_hw->band->pi, mute_tx, 0);

	if (mute_tx)
		brcms_c_ucode_mute_override_set(wlc_hw);
	else
		brcms_c_ucode_mute_override_clear(wlc_hw);
}

void
brcms_c_mute(struct brcms_c_info *wlc, bool mute_tx)
{
	brcms_b_mute(wlc->hw, mute_tx);
}

/*
 * Read and clear macintmask and macintstatus and intstatus registers.
 * This routine should be called with interrupts off
 * Return:
 *   -1 if brcms_deviceremoved(wlc) evaluates to true;
 *   0 if the interrupt is not for us, or we are in some special cases;
 *   device interrupt status bits otherwise.
 */
static inline u32 wlc_intstatus(struct brcms_c_info *wlc, bool in_isr)
{
	struct brcms_hardware *wlc_hw = wlc->hw;
	struct bcma_device *core = wlc_hw->d11core;
	u32 macintstatus, mask;

	/* macintstatus includes a DMA interrupt summary bit */
	macintstatus = bcma_read32(core, D11REGOFFS(macintstatus));
	mask = in_isr ? wlc->macintmask : wlc->defmacintmask;

	trace_brcms_macintstatus(&core->dev, in_isr, macintstatus, mask);

	/* detect cardbus removed, in power down(suspend) and in reset */
	if (brcms_deviceremoved(wlc))
		return -1;

	/* brcms_deviceremoved() succeeds even when the core is still resetting,
	 * handle that case here.
	 */
	if (macintstatus == 0xffffffff)
		return 0;

	/* defer unsolicited interrupts */
	macintstatus &= mask;

	/* if not for us */
	if (macintstatus == 0)
		return 0;

	/* turn off the interrupts */
	bcma_write32(core, D11REGOFFS(macintmask), 0);
	(void)bcma_read32(core, D11REGOFFS(macintmask));
	wlc->macintmask = 0;

	/* clear device interrupts */
	bcma_write32(core, D11REGOFFS(macintstatus), macintstatus);

	/* MI_DMAINT is indication of non-zero intstatus */
	if (macintstatus & MI_DMAINT)
		/*
		 * only fifo interrupt enabled is I_RI in
		 * RX_FIFO. If MI_DMAINT is set, assume it
		 * is set and clear the interrupt.
		 */
		bcma_write32(core, D11REGOFFS(intctrlregs[RX_FIFO].intstatus),
			     DEF_RXINTMASK);

	return macintstatus;
}

/* Update wlc->macintstatus and wlc->intstatus[]. */
/* Return true if they are updated successfully. false otherwise */
bool brcms_c_intrsupd(struct brcms_c_info *wlc)
{
	u32 macintstatus;

	/* read and clear macintstatus and intstatus registers */
	macintstatus = wlc_intstatus(wlc, false);

	/* device is removed */
	if (macintstatus == 0xffffffff)
		return false;

	/* update interrupt status in software */
	wlc->macintstatus |= macintstatus;

	return true;
}

/*
 * First-level interrupt processing.
 * Return true if this was our interrupt
 * and if further brcms_c_dpc() processing is required,
 * false otherwise.
 */
bool brcms_c_isr(struct brcms_c_info *wlc)
{
	struct brcms_hardware *wlc_hw = wlc->hw;
	u32 macintstatus;

	if (!wlc_hw->up || !wlc->macintmask)
		return false;

	/* read and clear macintstatus and intstatus registers */
	macintstatus = wlc_intstatus(wlc, true);

	if (macintstatus == 0xffffffff) {
		brcms_err(wlc_hw->d11core,
			  "DEVICEREMOVED detected in the ISR code path\n");
		return false;
	}

	/* it is not for us */
	if (macintstatus == 0)
		return false;

	/* save interrupt status bits */
	wlc->macintstatus = macintstatus;

	return true;

}

void brcms_c_suspend_mac_and_wait(struct brcms_c_info *wlc)
{
	struct brcms_hardware *wlc_hw = wlc->hw;
	struct bcma_device *core = wlc_hw->d11core;
	u32 mc, mi;

	brcms_dbg_mac80211(core, "wl%d: bandunit %d\n", wlc_hw->unit,
			   wlc_hw->band->bandunit);

	/*
	 * Track overlapping suspend requests
	 */
	wlc_hw->mac_suspend_depth++;
	if (wlc_hw->mac_suspend_depth > 1)
		return;

	/* force the core awake */
	brcms_c_ucode_wake_override_set(wlc_hw, BRCMS_WAKE_OVERRIDE_MACSUSPEND);

	mc = bcma_read32(core, D11REGOFFS(maccontrol));

	if (mc == 0xffffffff) {
		brcms_err(core, "wl%d: %s: dead chip\n", wlc_hw->unit,
			  __func__);
		brcms_down(wlc->wl);
		return;
	}
	WARN_ON(mc & MCTL_PSM_JMP_0);
	WARN_ON(!(mc & MCTL_PSM_RUN));
	WARN_ON(!(mc & MCTL_EN_MAC));

	mi = bcma_read32(core, D11REGOFFS(macintstatus));
	if (mi == 0xffffffff) {
		brcms_err(core, "wl%d: %s: dead chip\n", wlc_hw->unit,
			  __func__);
		brcms_down(wlc->wl);
		return;
	}
	WARN_ON(mi & MI_MACSSPNDD);

	brcms_b_mctrl(wlc_hw, MCTL_EN_MAC, 0);

	SPINWAIT(!(bcma_read32(core, D11REGOFFS(macintstatus)) & MI_MACSSPNDD),
		 BRCMS_MAX_MAC_SUSPEND);

	if (!(bcma_read32(core, D11REGOFFS(macintstatus)) & MI_MACSSPNDD)) {
		brcms_err(core, "wl%d: wlc_suspend_mac_and_wait: waited %d uS"
			  " and MI_MACSSPNDD is still not on.\n",
			  wlc_hw->unit, BRCMS_MAX_MAC_SUSPEND);
		brcms_err(core, "wl%d: psmdebug 0x%08x, phydebug 0x%08x, "
			  "psm_brc 0x%04x\n", wlc_hw->unit,
			  bcma_read32(core, D11REGOFFS(psmdebug)),
			  bcma_read32(core, D11REGOFFS(phydebug)),
			  bcma_read16(core, D11REGOFFS(psm_brc)));
	}

	mc = bcma_read32(core, D11REGOFFS(maccontrol));
	if (mc == 0xffffffff) {
		brcms_err(core, "wl%d: %s: dead chip\n", wlc_hw->unit,
			  __func__);
		brcms_down(wlc->wl);
		return;
	}
	WARN_ON(mc & MCTL_PSM_JMP_0);
	WARN_ON(!(mc & MCTL_PSM_RUN));
	WARN_ON(mc & MCTL_EN_MAC);
}

void brcms_c_enable_mac(struct brcms_c_info *wlc)
{
	struct brcms_hardware *wlc_hw = wlc->hw;
	struct bcma_device *core = wlc_hw->d11core;
	u32 mc, mi;

	brcms_dbg_mac80211(core, "wl%d: bandunit %d\n", wlc_hw->unit,
			   wlc->band->bandunit);

	/*
	 * Track overlapping suspend requests
	 */
	wlc_hw->mac_suspend_depth--;
	if (wlc_hw->mac_suspend_depth > 0)
		return;

	mc = bcma_read32(core, D11REGOFFS(maccontrol));
	WARN_ON(mc & MCTL_PSM_JMP_0);
	WARN_ON(mc & MCTL_EN_MAC);
	WARN_ON(!(mc & MCTL_PSM_RUN));

	brcms_b_mctrl(wlc_hw, MCTL_EN_MAC, MCTL_EN_MAC);
	bcma_write32(core, D11REGOFFS(macintstatus), MI_MACSSPNDD);

	mc = bcma_read32(core, D11REGOFFS(maccontrol));
	WARN_ON(mc & MCTL_PSM_JMP_0);
	WARN_ON(!(mc & MCTL_EN_MAC));
	WARN_ON(!(mc & MCTL_PSM_RUN));

	mi = bcma_read32(core, D11REGOFFS(macintstatus));
	WARN_ON(mi & MI_MACSSPNDD);

	brcms_c_ucode_wake_override_clear(wlc_hw,
					  BRCMS_WAKE_OVERRIDE_MACSUSPEND);
}

void brcms_b_band_stf_ss_set(struct brcms_hardware *wlc_hw, u8 stf_mode)
{
	wlc_hw->hw_stf_ss_opmode = stf_mode;

	if (wlc_hw->clk)
		brcms_upd_ofdm_pctl1_table(wlc_hw);
}

static bool brcms_b_validate_chip_access(struct brcms_hardware *wlc_hw)
{
	struct bcma_device *core = wlc_hw->d11core;
	u32 w, val;
	struct wiphy *wiphy = wlc_hw->wlc->wiphy;

	/* Validate dchip register access */

	bcma_write32(core, D11REGOFFS(objaddr), OBJADDR_SHM_SEL | 0);
	(void)bcma_read32(core, D11REGOFFS(objaddr));
	w = bcma_read32(core, D11REGOFFS(objdata));

	/* Can we write and read back a 32bit register? */
	bcma_write32(core, D11REGOFFS(objaddr), OBJADDR_SHM_SEL | 0);
	(void)bcma_read32(core, D11REGOFFS(objaddr));
	bcma_write32(core, D11REGOFFS(objdata), (u32) 0xaa5555aa);

	bcma_write32(core, D11REGOFFS(objaddr), OBJADDR_SHM_SEL | 0);
	(void)bcma_read32(core, D11REGOFFS(objaddr));
	val = bcma_read32(core, D11REGOFFS(objdata));
	if (val != (u32) 0xaa5555aa) {
		wiphy_err(wiphy, "wl%d: validate_chip_access: SHM = 0x%x, "
			  "expected 0xaa5555aa\n", wlc_hw->unit, val);
		return false;
	}

	bcma_write32(core, D11REGOFFS(objaddr), OBJADDR_SHM_SEL | 0);
	(void)bcma_read32(core, D11REGOFFS(objaddr));
	bcma_write32(core, D11REGOFFS(objdata), (u32) 0x55aaaa55);

	bcma_write32(core, D11REGOFFS(objaddr), OBJADDR_SHM_SEL | 0);
	(void)bcma_read32(core, D11REGOFFS(objaddr));
	val = bcma_read32(core, D11REGOFFS(objdata));
	if (val != (u32) 0x55aaaa55) {
		wiphy_err(wiphy, "wl%d: validate_chip_access: SHM = 0x%x, "
			  "expected 0x55aaaa55\n", wlc_hw->unit, val);
		return false;
	}

	bcma_write32(core, D11REGOFFS(objaddr), OBJADDR_SHM_SEL | 0);
	(void)bcma_read32(core, D11REGOFFS(objaddr));
	bcma_write32(core, D11REGOFFS(objdata), w);

	/* clear CFPStart */
	bcma_write32(core, D11REGOFFS(tsf_cfpstart), 0);

	w = bcma_read32(core, D11REGOFFS(maccontrol));
	if ((w != (MCTL_IHR_EN | MCTL_WAKE)) &&
	    (w != (MCTL_IHR_EN | MCTL_GMODE | MCTL_WAKE))) {
		wiphy_err(wiphy, "wl%d: validate_chip_access: maccontrol = "
			  "0x%x, expected 0x%x or 0x%x\n", wlc_hw->unit, w,
			  (MCTL_IHR_EN | MCTL_WAKE),
			  (MCTL_IHR_EN | MCTL_GMODE | MCTL_WAKE));
		return false;
	}

	return true;
}

#define PHYPLL_WAIT_US	100000

void brcms_b_core_phypll_ctl(struct brcms_hardware *wlc_hw, bool on)
{
	struct bcma_device *core = wlc_hw->d11core;
	u32 tmp;

	brcms_dbg_info(core, "wl%d\n", wlc_hw->unit);

	tmp = 0;

	if (on) {
		if ((ai_get_chip_id(wlc_hw->sih) == BCMA_CHIP_ID_BCM4313)) {
			bcma_set32(core, D11REGOFFS(clk_ctl_st),
				   CCS_ERSRC_REQ_HT |
				   CCS_ERSRC_REQ_D11PLL |
				   CCS_ERSRC_REQ_PHYPLL);
			SPINWAIT((bcma_read32(core, D11REGOFFS(clk_ctl_st)) &
				  CCS_ERSRC_AVAIL_HT) != CCS_ERSRC_AVAIL_HT,
				 PHYPLL_WAIT_US);

			tmp = bcma_read32(core, D11REGOFFS(clk_ctl_st));
			if ((tmp & CCS_ERSRC_AVAIL_HT) != CCS_ERSRC_AVAIL_HT)
				brcms_err(core, "%s: turn on PHY PLL failed\n",
					  __func__);
		} else {
			bcma_set32(core, D11REGOFFS(clk_ctl_st),
				   tmp | CCS_ERSRC_REQ_D11PLL |
				   CCS_ERSRC_REQ_PHYPLL);
			SPINWAIT((bcma_read32(core, D11REGOFFS(clk_ctl_st)) &
				  (CCS_ERSRC_AVAIL_D11PLL |
				   CCS_ERSRC_AVAIL_PHYPLL)) !=
				 (CCS_ERSRC_AVAIL_D11PLL |
				  CCS_ERSRC_AVAIL_PHYPLL), PHYPLL_WAIT_US);

			tmp = bcma_read32(core, D11REGOFFS(clk_ctl_st));
			if ((tmp &
			     (CCS_ERSRC_AVAIL_D11PLL | CCS_ERSRC_AVAIL_PHYPLL))
			    !=
			    (CCS_ERSRC_AVAIL_D11PLL | CCS_ERSRC_AVAIL_PHYPLL))
				brcms_err(core, "%s: turn on PHY PLL failed\n",
					  __func__);
		}
	} else {
		/*
		 * Since the PLL may be shared, other cores can still
		 * be requesting it; so we'll deassert the request but
		 * not wait for status to comply.
		 */
		bcma_mask32(core, D11REGOFFS(clk_ctl_st),
			    ~CCS_ERSRC_REQ_PHYPLL);
		(void)bcma_read32(core, D11REGOFFS(clk_ctl_st));
	}
}

static void brcms_c_coredisable(struct brcms_hardware *wlc_hw)
{
	bool dev_gone;

	brcms_dbg_info(wlc_hw->d11core, "wl%d: disable core\n", wlc_hw->unit);

	dev_gone = brcms_deviceremoved(wlc_hw->wlc);

	if (dev_gone)
		return;

	if (wlc_hw->noreset)
		return;

	/* radio off */
	wlc_phy_switch_radio(wlc_hw->band->pi, OFF);

	/* turn off analog core */
	wlc_phy_anacore(wlc_hw->band->pi, OFF);

	/* turn off PHYPLL to save power */
	brcms_b_core_phypll_ctl(wlc_hw, false);

	wlc_hw->clk = false;
	bcma_core_disable(wlc_hw->d11core, 0);
	wlc_phy_hw_clk_state_upd(wlc_hw->band->pi, false);
}

static void brcms_c_flushqueues(struct brcms_c_info *wlc)
{
	struct brcms_hardware *wlc_hw = wlc->hw;
	uint i;

	/* free any posted tx packets */
	for (i = 0; i < NFIFO; i++) {
		if (wlc_hw->di[i]) {
			dma_txreclaim(wlc_hw->di[i], DMA_RANGE_ALL);
			if (i < TX_BCMC_FIFO)
				ieee80211_wake_queue(wlc->pub->ieee_hw,
						     brcms_fifo_to_ac(i));
		}
	}

	/* free any posted rx packets */
	dma_rxreclaim(wlc_hw->di[RX_FIFO]);
}

static u16
brcms_b_read_objmem(struct brcms_hardware *wlc_hw, uint offset, u32 sel)
{
	struct bcma_device *core = wlc_hw->d11core;
	u16 objoff = D11REGOFFS(objdata);

	bcma_write32(core, D11REGOFFS(objaddr), sel | (offset >> 2));
	(void)bcma_read32(core, D11REGOFFS(objaddr));
	if (offset & 2)
		objoff += 2;

	return bcma_read16(core, objoff);
}

static void
brcms_b_write_objmem(struct brcms_hardware *wlc_hw, uint offset, u16 v,
		     u32 sel)
{
	struct bcma_device *core = wlc_hw->d11core;
	u16 objoff = D11REGOFFS(objdata);

	bcma_write32(core, D11REGOFFS(objaddr), sel | (offset >> 2));
	(void)bcma_read32(core, D11REGOFFS(objaddr));
	if (offset & 2)
		objoff += 2;

	bcma_wflush16(core, objoff, v);
}

/*
 * Read a single u16 from shared memory.
 * SHM 'offset' needs to be an even address
 */
u16 brcms_b_read_shm(struct brcms_hardware *wlc_hw, uint offset)
{
	return brcms_b_read_objmem(wlc_hw, offset, OBJADDR_SHM_SEL);
}

/*
 * Write a single u16 to shared memory.
 * SHM 'offset' needs to be an even address
 */
void brcms_b_write_shm(struct brcms_hardware *wlc_hw, uint offset, u16 v)
{
	brcms_b_write_objmem(wlc_hw, offset, v, OBJADDR_SHM_SEL);
}

/*
 * Copy a buffer to shared memory of specified type .
 * SHM 'offset' needs to be an even address and
 * Buffer length 'len' must be an even number of bytes
 * 'sel' selects the type of memory
 */
void
brcms_b_copyto_objmem(struct brcms_hardware *wlc_hw, uint offset,
		      const void *buf, int len, u32 sel)
{
	u16 v;
	const u8 *p = (const u8 *)buf;
	int i;

	if (len <= 0 || (offset & 1) || (len & 1))
		return;

	for (i = 0; i < len; i += 2) {
		v = p[i] | (p[i + 1] << 8);
		brcms_b_write_objmem(wlc_hw, offset + i, v, sel);
	}
}

/*
 * Copy a piece of shared memory of specified type to a buffer .
 * SHM 'offset' needs to be an even address and
 * Buffer length 'len' must be an even number of bytes
 * 'sel' selects the type of memory
 */
void
brcms_b_copyfrom_objmem(struct brcms_hardware *wlc_hw, uint offset, void *buf,
			 int len, u32 sel)
{
	u16 v;
	u8 *p = (u8 *) buf;
	int i;

	if (len <= 0 || (offset & 1) || (len & 1))
		return;

	for (i = 0; i < len; i += 2) {
		v = brcms_b_read_objmem(wlc_hw, offset + i, sel);
		p[i] = v & 0xFF;
		p[i + 1] = (v >> 8) & 0xFF;
	}
}

/* Copy a buffer to shared memory.
 * SHM 'offset' needs to be an even address and
 * Buffer length 'len' must be an even number of bytes
 */
static void brcms_c_copyto_shm(struct brcms_c_info *wlc, uint offset,
			const void *buf, int len)
{
	brcms_b_copyto_objmem(wlc->hw, offset, buf, len, OBJADDR_SHM_SEL);
}

static void brcms_b_retrylimit_upd(struct brcms_hardware *wlc_hw,
				   u16 SRL, u16 LRL)
{
	wlc_hw->SRL = SRL;
	wlc_hw->LRL = LRL;

	/* write retry limit to SCR, shouldn't need to suspend */
	if (wlc_hw->up) {
		bcma_write32(wlc_hw->d11core, D11REGOFFS(objaddr),
			     OBJADDR_SCR_SEL | S_DOT11_SRC_LMT);
		(void)bcma_read32(wlc_hw->d11core, D11REGOFFS(objaddr));
		bcma_write32(wlc_hw->d11core, D11REGOFFS(objdata), wlc_hw->SRL);
		bcma_write32(wlc_hw->d11core, D11REGOFFS(objaddr),
			     OBJADDR_SCR_SEL | S_DOT11_LRC_LMT);
		(void)bcma_read32(wlc_hw->d11core, D11REGOFFS(objaddr));
		bcma_write32(wlc_hw->d11core, D11REGOFFS(objdata), wlc_hw->LRL);
	}
}

static void brcms_b_pllreq(struct brcms_hardware *wlc_hw, bool set, u32 req_bit)
{
	if (set) {
		if (mboolisset(wlc_hw->pllreq, req_bit))
			return;

		mboolset(wlc_hw->pllreq, req_bit);

		if (mboolisset(wlc_hw->pllreq, BRCMS_PLLREQ_FLIP)) {
			if (!wlc_hw->sbclk)
				brcms_b_xtal(wlc_hw, ON);
		}
	} else {
		if (!mboolisset(wlc_hw->pllreq, req_bit))
			return;

		mboolclr(wlc_hw->pllreq, req_bit);

		if (mboolisset(wlc_hw->pllreq, BRCMS_PLLREQ_FLIP)) {
			if (wlc_hw->sbclk)
				brcms_b_xtal(wlc_hw, OFF);
		}
	}
}

static void brcms_b_antsel_set(struct brcms_hardware *wlc_hw, u32 antsel_avail)
{
	wlc_hw->antsel_avail = antsel_avail;
}

/*
 * conditions under which the PM bit should be set in outgoing frames
 * and STAY_AWAKE is meaningful
 */
static bool brcms_c_ps_allowed(struct brcms_c_info *wlc)
{
	/* not supporting PS so always return false for now */
	return false;
}

static void brcms_c_statsupd(struct brcms_c_info *wlc)
{
	int i;
	struct macstat *macstats;
#ifdef DEBUG
	u16 delta;
	u16 rxf0ovfl;
	u16 txfunfl[NFIFO];
#endif				/* DEBUG */

	/* if driver down, make no sense to update stats */
	if (!wlc->pub->up)
		return;

	macstats = wlc->core->macstat_snapshot;

#ifdef DEBUG
	/* save last rx fifo 0 overflow count */
	rxf0ovfl = macstats->rxf0ovfl;

	/* save last tx fifo  underflow count */
	for (i = 0; i < NFIFO; i++)
		txfunfl[i] = macstats->txfunfl[i];
#endif				/* DEBUG */

	/* Read mac stats from contiguous shared memory */
	brcms_b_copyfrom_objmem(wlc->hw, M_UCODE_MACSTAT, macstats,
				sizeof(*macstats), OBJADDR_SHM_SEL);

#ifdef DEBUG
	/* check for rx fifo 0 overflow */
	delta = (u16)(macstats->rxf0ovfl - rxf0ovfl);
	if (delta)
		brcms_err(wlc->hw->d11core, "wl%d: %u rx fifo 0 overflows!\n",
			  wlc->pub->unit, delta);

	/* check for tx fifo underflows */
	for (i = 0; i < NFIFO; i++) {
		delta = macstats->txfunfl[i] - txfunfl[i];
		if (delta)
			brcms_err(wlc->hw->d11core,
				  "wl%d: %u tx fifo %d underflows!\n",
				  wlc->pub->unit, delta, i);
	}
#endif				/* DEBUG */

	/* merge counters from dma module */
	for (i = 0; i < NFIFO; i++) {
		if (wlc->hw->di[i])
			dma_counterreset(wlc->hw->di[i]);
	}
}

static void brcms_b_reset(struct brcms_hardware *wlc_hw)
{
	/* reset the core */
	if (!brcms_deviceremoved(wlc_hw->wlc))
		brcms_b_corereset(wlc_hw, BRCMS_USE_COREFLAGS);

	/* purge the dma rings */
	brcms_c_flushqueues(wlc_hw->wlc);
}

void brcms_c_reset(struct brcms_c_info *wlc)
{
	brcms_dbg_info(wlc->hw->d11core, "wl%d\n", wlc->pub->unit);

	/* slurp up hw mac counters before core reset */
	brcms_c_statsupd(wlc);

	/* reset our snapshot of macstat counters */
	memset(wlc->core->macstat_snapshot, 0, sizeof(struct macstat));

	brcms_b_reset(wlc->hw);
}

void brcms_c_init_scb(struct scb *scb)
{
	int i;

	memset(scb, 0, sizeof(struct scb));
	scb->flags = SCB_WMECAP | SCB_HTCAP;
	for (i = 0; i < NUMPRIO; i++) {
		scb->seqnum[i] = 0;
		scb->seqctl[i] = 0xFFFF;
	}

	scb->seqctl_nonqos = 0xFFFF;
	scb->magic = SCB_MAGIC;
}

/* d11 core init
 *   reset PSM
 *   download ucode/PCM
 *   let ucode run to suspended
 *   download ucode inits
 *   config other core registers
 *   init dma
 */
static void brcms_b_coreinit(struct brcms_c_info *wlc)
{
	struct brcms_hardware *wlc_hw = wlc->hw;
	struct bcma_device *core = wlc_hw->d11core;
	u32 sflags;
	u32 bcnint_us;
	uint i = 0;
	bool fifosz_fixup = false;
	int err = 0;
	u16 buf[NFIFO];
	struct brcms_ucode *ucode = &wlc_hw->wlc->wl->ucode;

	brcms_dbg_info(core, "wl%d: core init\n", wlc_hw->unit);

	/* reset PSM */
	brcms_b_mctrl(wlc_hw, ~0, (MCTL_IHR_EN | MCTL_PSM_JMP_0 | MCTL_WAKE));

	brcms_ucode_download(wlc_hw);
	/*
	 * FIFOSZ fixup. driver wants to controls the fifo allocation.
	 */
	fifosz_fixup = true;

	/* let the PSM run to the suspended state, set mode to BSS STA */
	bcma_write32(core, D11REGOFFS(macintstatus), -1);
	brcms_b_mctrl(wlc_hw, ~0,
		       (MCTL_IHR_EN | MCTL_INFRA | MCTL_PSM_RUN | MCTL_WAKE));

	/* wait for ucode to self-suspend after auto-init */
	SPINWAIT(((bcma_read32(core, D11REGOFFS(macintstatus)) &
		   MI_MACSSPNDD) == 0), 1000 * 1000);
	if ((bcma_read32(core, D11REGOFFS(macintstatus)) & MI_MACSSPNDD) == 0)
		brcms_err(core, "wl%d: wlc_coreinit: ucode did not self-"
			  "suspend!\n", wlc_hw->unit);

	brcms_c_gpio_init(wlc);

	sflags = bcma_aread32(core, BCMA_IOST);

	if (D11REV_IS(wlc_hw->corerev, 17) || D11REV_IS(wlc_hw->corerev, 23)) {
		if (BRCMS_ISNPHY(wlc_hw->band))
			brcms_c_write_inits(wlc_hw, ucode->d11n0initvals16);
		else
			brcms_err(core, "%s: wl%d: unsupported phy in corerev"
				  " %d\n", __func__, wlc_hw->unit,
				  wlc_hw->corerev);
	} else if (D11REV_IS(wlc_hw->corerev, 24)) {
		if (BRCMS_ISLCNPHY(wlc_hw->band))
			brcms_c_write_inits(wlc_hw, ucode->d11lcn0initvals24);
		else
			brcms_err(core, "%s: wl%d: unsupported phy in corerev"
				  " %d\n", __func__, wlc_hw->unit,
				  wlc_hw->corerev);
	} else {
		brcms_err(core, "%s: wl%d: unsupported corerev %d\n",
			  __func__, wlc_hw->unit, wlc_hw->corerev);
	}

	/* For old ucode, txfifo sizes needs to be modified(increased) */
	if (fifosz_fixup)
		brcms_b_corerev_fifofixup(wlc_hw);

	/* check txfifo allocations match between ucode and driver */
	buf[TX_AC_BE_FIFO] = brcms_b_read_shm(wlc_hw, M_FIFOSIZE0);
	if (buf[TX_AC_BE_FIFO] != wlc_hw->xmtfifo_sz[TX_AC_BE_FIFO]) {
		i = TX_AC_BE_FIFO;
		err = -1;
	}
	buf[TX_AC_VI_FIFO] = brcms_b_read_shm(wlc_hw, M_FIFOSIZE1);
	if (buf[TX_AC_VI_FIFO] != wlc_hw->xmtfifo_sz[TX_AC_VI_FIFO]) {
		i = TX_AC_VI_FIFO;
		err = -1;
	}
	buf[TX_AC_BK_FIFO] = brcms_b_read_shm(wlc_hw, M_FIFOSIZE2);
	buf[TX_AC_VO_FIFO] = (buf[TX_AC_BK_FIFO] >> 8) & 0xff;
	buf[TX_AC_BK_FIFO] &= 0xff;
	if (buf[TX_AC_BK_FIFO] != wlc_hw->xmtfifo_sz[TX_AC_BK_FIFO]) {
		i = TX_AC_BK_FIFO;
		err = -1;
	}
	if (buf[TX_AC_VO_FIFO] != wlc_hw->xmtfifo_sz[TX_AC_VO_FIFO]) {
		i = TX_AC_VO_FIFO;
		err = -1;
	}
	buf[TX_BCMC_FIFO] = brcms_b_read_shm(wlc_hw, M_FIFOSIZE3);
	buf[TX_ATIM_FIFO] = (buf[TX_BCMC_FIFO] >> 8) & 0xff;
	buf[TX_BCMC_FIFO] &= 0xff;
	if (buf[TX_BCMC_FIFO] != wlc_hw->xmtfifo_sz[TX_BCMC_FIFO]) {
		i = TX_BCMC_FIFO;
		err = -1;
	}
	if (buf[TX_ATIM_FIFO] != wlc_hw->xmtfifo_sz[TX_ATIM_FIFO]) {
		i = TX_ATIM_FIFO;
		err = -1;
	}
	if (err != 0)
		brcms_err(core, "wlc_coreinit: txfifo mismatch: ucode size %d"
			  " driver size %d index %d\n", buf[i],
			  wlc_hw->xmtfifo_sz[i], i);

	/* make sure we can still talk to the mac */
	WARN_ON(bcma_read32(core, D11REGOFFS(maccontrol)) == 0xffffffff);

	/* band-specific inits done by wlc_bsinit() */

	/* Set up frame burst size and antenna swap threshold init values */
	brcms_b_write_shm(wlc_hw, M_MBURST_SIZE, MAXTXFRAMEBURST);
	brcms_b_write_shm(wlc_hw, M_MAX_ANTCNT, ANTCNT);

	/* enable one rx interrupt per received frame */
	bcma_write32(core, D11REGOFFS(intrcvlazy[0]), (1 << IRL_FC_SHIFT));

	/* set the station mode (BSS STA) */
	brcms_b_mctrl(wlc_hw,
		       (MCTL_INFRA | MCTL_DISCARD_PMQ | MCTL_AP),
		       (MCTL_INFRA | MCTL_DISCARD_PMQ));

	/* set up Beacon interval */
	bcnint_us = 0x8000 << 10;
	bcma_write32(core, D11REGOFFS(tsf_cfprep),
		     (bcnint_us << CFPREP_CBI_SHIFT));
	bcma_write32(core, D11REGOFFS(tsf_cfpstart), bcnint_us);
	bcma_write32(core, D11REGOFFS(macintstatus), MI_GP1);

	/* write interrupt mask */
	bcma_write32(core, D11REGOFFS(intctrlregs[RX_FIFO].intmask),
		     DEF_RXINTMASK);

	/* allow the MAC to control the PHY clock (dynamic on/off) */
	brcms_b_macphyclk_set(wlc_hw, ON);

	/* program dynamic clock control fast powerup delay register */
	wlc->fastpwrup_dly = ai_clkctl_fast_pwrup_delay(wlc_hw->sih);
	bcma_write16(core, D11REGOFFS(scc_fastpwrup_dly), wlc->fastpwrup_dly);

	/* tell the ucode the corerev */
	brcms_b_write_shm(wlc_hw, M_MACHW_VER, (u16) wlc_hw->corerev);

	/* tell the ucode MAC capabilities */
	brcms_b_write_shm(wlc_hw, M_MACHW_CAP_L,
			   (u16) (wlc_hw->machwcap & 0xffff));
	brcms_b_write_shm(wlc_hw, M_MACHW_CAP_H,
			   (u16) ((wlc_hw->
				      machwcap >> 16) & 0xffff));

	/* write retry limits to SCR, this done after PSM init */
	bcma_write32(core, D11REGOFFS(objaddr),
		     OBJADDR_SCR_SEL | S_DOT11_SRC_LMT);
	(void)bcma_read32(core, D11REGOFFS(objaddr));
	bcma_write32(core, D11REGOFFS(objdata), wlc_hw->SRL);
	bcma_write32(core, D11REGOFFS(objaddr),
		     OBJADDR_SCR_SEL | S_DOT11_LRC_LMT);
	(void)bcma_read32(core, D11REGOFFS(objaddr));
	bcma_write32(core, D11REGOFFS(objdata), wlc_hw->LRL);

	/* write rate fallback retry limits */
	brcms_b_write_shm(wlc_hw, M_SFRMTXCNTFBRTHSD, wlc_hw->SFBL);
	brcms_b_write_shm(wlc_hw, M_LFRMTXCNTFBRTHSD, wlc_hw->LFBL);

	bcma_mask16(core, D11REGOFFS(ifs_ctl), 0x0FFF);
	bcma_write16(core, D11REGOFFS(ifs_aifsn), EDCF_AIFSN_MIN);

	/* init the tx dma engines */
	for (i = 0; i < NFIFO; i++) {
		if (wlc_hw->di[i])
			dma_txinit(wlc_hw->di[i]);
	}

	/* init the rx dma engine(s) and post receive buffers */
	dma_rxinit(wlc_hw->di[RX_FIFO]);
	dma_rxfill(wlc_hw->di[RX_FIFO]);
}

static void brcms_b_init(struct brcms_hardware *wlc_hw, u16 chanspec)
{
	u32 macintmask;
	bool fastclk;
	struct brcms_c_info *wlc = wlc_hw->wlc;

	/* request FAST clock if not on */
	fastclk = wlc_hw->forcefastclk;
	if (!fastclk)
		brcms_b_clkctl_clk(wlc_hw, BCMA_CLKMODE_FAST);

	/* disable interrupts */
	macintmask = brcms_intrsoff(wlc->wl);

	/* set up the specified band and chanspec */
	brcms_c_setxband(wlc_hw, chspec_bandunit(chanspec));
	wlc_phy_chanspec_radio_set(wlc_hw->band->pi, chanspec);

	/* do one-time phy inits and calibration */
	wlc_phy_cal_init(wlc_hw->band->pi);

	/* core-specific initialization */
	brcms_b_coreinit(wlc);

	/* band-specific inits */
	brcms_b_bsinit(wlc, chanspec);

	/* restore macintmask */
	brcms_intrsrestore(wlc->wl, macintmask);

	/* seed wake_override with BRCMS_WAKE_OVERRIDE_MACSUSPEND since the mac
	 * is suspended and brcms_c_enable_mac() will clear this override bit.
	 */
	mboolset(wlc_hw->wake_override, BRCMS_WAKE_OVERRIDE_MACSUSPEND);

	/*
	 * initialize mac_suspend_depth to 1 to match ucode
	 * initial suspended state
	 */
	wlc_hw->mac_suspend_depth = 1;

	/* restore the clk */
	if (!fastclk)
		brcms_b_clkctl_clk(wlc_hw, BCMA_CLKMODE_DYNAMIC);
}

static void brcms_c_set_phy_chanspec(struct brcms_c_info *wlc,
				     u16 chanspec)
{
	/* Save our copy of the chanspec */
	wlc->chanspec = chanspec;

	/* Set the chanspec and power limits for this locale */
	brcms_c_channel_set_chanspec(wlc->cmi, chanspec, BRCMS_TXPWR_MAX);

	if (wlc->stf->ss_algosel_auto)
		brcms_c_stf_ss_algo_channel_get(wlc, &wlc->stf->ss_algo_channel,
					    chanspec);

	brcms_c_stf_ss_update(wlc, wlc->band);
}

static void
brcms_default_rateset(struct brcms_c_info *wlc, struct brcms_c_rateset *rs)
{
	brcms_c_rateset_default(rs, NULL, wlc->band->phytype,
		wlc->band->bandtype, false, BRCMS_RATE_MASK_FULL,
		(bool) (wlc->pub->_n_enab & SUPPORT_11N),
		brcms_chspec_bw(wlc->default_bss->chanspec),
		wlc->stf->txstreams);
}

/* derive wlc->band->basic_rate[] table from 'rateset' */
static void brcms_c_rate_lookup_init(struct brcms_c_info *wlc,
			      struct brcms_c_rateset *rateset)
{
	u8 rate;
	u8 mandatory;
	u8 cck_basic = 0;
	u8 ofdm_basic = 0;
	u8 *br = wlc->band->basic_rate;
	uint i;

	/* incoming rates are in 500kbps units as in 802.11 Supported Rates */
	memset(br, 0, BRCM_MAXRATE + 1);

	/* For each basic rate in the rates list, make an entry in the
	 * best basic lookup.
	 */
	for (i = 0; i < rateset->count; i++) {
		/* only make an entry for a basic rate */
		if (!(rateset->rates[i] & BRCMS_RATE_FLAG))
			continue;

		/* mask off basic bit */
		rate = (rateset->rates[i] & BRCMS_RATE_MASK);

		if (rate > BRCM_MAXRATE) {
			brcms_err(wlc->hw->d11core, "brcms_c_rate_lookup_init: "
				  "invalid rate 0x%X in rate set\n",
				  rateset->rates[i]);
			continue;
		}

		br[rate] = rate;
	}

	/* The rate lookup table now has non-zero entries for each
	 * basic rate, equal to the basic rate: br[basicN] = basicN
	 *
	 * To look up the best basic rate corresponding to any
	 * particular rate, code can use the basic_rate table
	 * like this
	 *
	 * basic_rate = wlc->band->basic_rate[tx_rate]
	 *
	 * Make sure there is a best basic rate entry for
	 * every rate by walking up the table from low rates
	 * to high, filling in holes in the lookup table
	 */

	for (i = 0; i < wlc->band->hw_rateset.count; i++) {
		rate = wlc->band->hw_rateset.rates[i];

		if (br[rate] != 0) {
			/* This rate is a basic rate.
			 * Keep track of the best basic rate so far by
			 * modulation type.
			 */
			if (is_ofdm_rate(rate))
				ofdm_basic = rate;
			else
				cck_basic = rate;

			continue;
		}

		/* This rate is not a basic rate so figure out the
		 * best basic rate less than this rate and fill in
		 * the hole in the table
		 */

		br[rate] = is_ofdm_rate(rate) ? ofdm_basic : cck_basic;

		if (br[rate] != 0)
			continue;

		if (is_ofdm_rate(rate)) {
			/*
			 * In 11g and 11a, the OFDM mandatory rates
			 * are 6, 12, and 24 Mbps
			 */
			if (rate >= BRCM_RATE_24M)
				mandatory = BRCM_RATE_24M;
			else if (rate >= BRCM_RATE_12M)
				mandatory = BRCM_RATE_12M;
			else
				mandatory = BRCM_RATE_6M;
		} else {
			/* In 11b, all CCK rates are mandatory 1 - 11 Mbps */
			mandatory = rate;
		}

		br[rate] = mandatory;
	}
}

static void brcms_c_bandinit_ordered(struct brcms_c_info *wlc,
				     u16 chanspec)
{
	struct brcms_c_rateset default_rateset;
	uint parkband;
	uint i, band_order[2];

	/*
	 * We might have been bandlocked during down and the chip
	 * power-cycled (hibernate). Figure out the right band to park on
	 */
	if (wlc->bandlocked || wlc->pub->_nbands == 1) {
		/* updated in brcms_c_bandlock() */
		parkband = wlc->band->bandunit;
		band_order[0] = band_order[1] = parkband;
	} else {
		/* park on the band of the specified chanspec */
		parkband = chspec_bandunit(chanspec);

		/* order so that parkband initialize last */
		band_order[0] = parkband ^ 1;
		band_order[1] = parkband;
	}

	/* make each band operational, software state init */
	for (i = 0; i < wlc->pub->_nbands; i++) {
		uint j = band_order[i];

		wlc->band = wlc->bandstate[j];

		brcms_default_rateset(wlc, &default_rateset);

		/* fill in hw_rate */
		brcms_c_rateset_filter(&default_rateset, &wlc->band->hw_rateset,
				   false, BRCMS_RATES_CCK_OFDM, BRCMS_RATE_MASK,
				   (bool) (wlc->pub->_n_enab & SUPPORT_11N));

		/* init basic rate lookup */
		brcms_c_rate_lookup_init(wlc, &default_rateset);
	}

	/* sync up phy/radio chanspec */
	brcms_c_set_phy_chanspec(wlc, chanspec);
}

/*
 * Set or clear filtering related maccontrol bits based on
 * specified filter flags
 */
void brcms_c_mac_promisc(struct brcms_c_info *wlc, uint filter_flags)
{
	u32 promisc_bits = 0;

	wlc->filter_flags = filter_flags;

	if (filter_flags & FIF_OTHER_BSS)
		promisc_bits |= MCTL_PROMISC;

	if (filter_flags & FIF_BCN_PRBRESP_PROMISC)
		promisc_bits |= MCTL_BCNS_PROMISC;

	if (filter_flags & FIF_FCSFAIL)
		promisc_bits |= MCTL_KEEPBADFCS;

	if (filter_flags & (FIF_CONTROL | FIF_PSPOLL))
		promisc_bits |= MCTL_KEEPCONTROL;

	brcms_b_mctrl(wlc->hw,
		MCTL_PROMISC | MCTL_BCNS_PROMISC |
		MCTL_KEEPCONTROL | MCTL_KEEPBADFCS,
		promisc_bits);
}

/*
 * ucode, hwmac update
 *    Channel dependent updates for ucode and hw
 */
static void brcms_c_ucode_mac_upd(struct brcms_c_info *wlc)
{
	/* enable or disable any active IBSSs depending on whether or not
	 * we are on the home channel
	 */
	if (wlc->home_chanspec == wlc_phy_chanspec_get(wlc->band->pi)) {
		if (wlc->pub->associated) {
			/*
			 * BMAC_NOTE: This is something that should be fixed
			 * in ucode inits. I think that the ucode inits set
			 * up the bcn templates and shm values with a bogus
			 * beacon. This should not be done in the inits. If
			 * ucode needs to set up a beacon for testing, the
			 * test routines should write it down, not expect the
			 * inits to populate a bogus beacon.
			 */
			if (BRCMS_PHY_11N_CAP(wlc->band))
				brcms_b_write_shm(wlc->hw,
						M_BCN_TXTSF_OFFSET, 0);
		}
	} else {
		/* disable an active IBSS if we are not on the home channel */
	}
}

static void brcms_c_write_rate_shm(struct brcms_c_info *wlc, u8 rate,
				   u8 basic_rate)
{
	u8 phy_rate, index;
	u8 basic_phy_rate, basic_index;
	u16 dir_table, basic_table;
	u16 basic_ptr;

	/* Shared memory address for the table we are reading */
	dir_table = is_ofdm_rate(basic_rate) ? M_RT_DIRMAP_A : M_RT_DIRMAP_B;

	/* Shared memory address for the table we are writing */
	basic_table = is_ofdm_rate(rate) ? M_RT_BBRSMAP_A : M_RT_BBRSMAP_B;

	/*
	 * for a given rate, the LS-nibble of the PLCP SIGNAL field is
	 * the index into the rate table.
	 */
	phy_rate = rate_info[rate] & BRCMS_RATE_MASK;
	basic_phy_rate = rate_info[basic_rate] & BRCMS_RATE_MASK;
	index = phy_rate & 0xf;
	basic_index = basic_phy_rate & 0xf;

	/* Find the SHM pointer to the ACK rate entry by looking in the
	 * Direct-map Table
	 */
	basic_ptr = brcms_b_read_shm(wlc->hw, (dir_table + basic_index * 2));

	/* Update the SHM BSS-basic-rate-set mapping table with the pointer
	 * to the correct basic rate for the given incoming rate
	 */
	brcms_b_write_shm(wlc->hw, (basic_table + index * 2), basic_ptr);
}

static const struct brcms_c_rateset *
brcms_c_rateset_get_hwrs(struct brcms_c_info *wlc)
{
	const struct brcms_c_rateset *rs_dflt;

	if (BRCMS_PHY_11N_CAP(wlc->band)) {
		if (wlc->band->bandtype == BRCM_BAND_5G)
			rs_dflt = &ofdm_mimo_rates;
		else
			rs_dflt = &cck_ofdm_mimo_rates;
	} else if (wlc->band->gmode)
		rs_dflt = &cck_ofdm_rates;
	else
		rs_dflt = &cck_rates;

	return rs_dflt;
}

static void brcms_c_set_ratetable(struct brcms_c_info *wlc)
{
	const struct brcms_c_rateset *rs_dflt;
	struct brcms_c_rateset rs;
	u8 rate, basic_rate;
	uint i;

	rs_dflt = brcms_c_rateset_get_hwrs(wlc);

	brcms_c_rateset_copy(rs_dflt, &rs);
	brcms_c_rateset_mcs_upd(&rs, wlc->stf->txstreams);

	/* walk the phy rate table and update SHM basic rate lookup table */
	for (i = 0; i < rs.count; i++) {
		rate = rs.rates[i] & BRCMS_RATE_MASK;

		/* for a given rate brcms_basic_rate returns the rate at
		 * which a response ACK/CTS should be sent.
		 */
		basic_rate = brcms_basic_rate(wlc, rate);
		if (basic_rate == 0)
			/* This should only happen if we are using a
			 * restricted rateset.
			 */
			basic_rate = rs.rates[0] & BRCMS_RATE_MASK;

		brcms_c_write_rate_shm(wlc, rate, basic_rate);
	}
}

/* band-specific init */
static void brcms_c_bsinit(struct brcms_c_info *wlc)
{
	brcms_dbg_info(wlc->hw->d11core, "wl%d: bandunit %d\n",
		       wlc->pub->unit, wlc->band->bandunit);

	/* write ucode ACK/CTS rate table */
	brcms_c_set_ratetable(wlc);

	/* update some band specific mac configuration */
	brcms_c_ucode_mac_upd(wlc);

	/* init antenna selection */
	brcms_c_antsel_init(wlc->asi);

}

/* formula:  IDLE_BUSY_RATIO_X_16 = (100-duty_cycle)/duty_cycle*16 */
static int
brcms_c_duty_cycle_set(struct brcms_c_info *wlc, int duty_cycle, bool isOFDM,
		   bool writeToShm)
{
	int idle_busy_ratio_x_16 = 0;
	uint offset =
	    isOFDM ? M_TX_IDLE_BUSY_RATIO_X_16_OFDM :
	    M_TX_IDLE_BUSY_RATIO_X_16_CCK;
	if (duty_cycle > 100 || duty_cycle < 0) {
		brcms_err(wlc->hw->d11core,
			  "wl%d:  duty cycle value off limit\n",
			  wlc->pub->unit);
		return -EINVAL;
	}
	if (duty_cycle)
		idle_busy_ratio_x_16 = (100 - duty_cycle) * 16 / duty_cycle;
	/* Only write to shared memory  when wl is up */
	if (writeToShm)
		brcms_b_write_shm(wlc->hw, offset, (u16) idle_busy_ratio_x_16);

	if (isOFDM)
		wlc->tx_duty_cycle_ofdm = (u16) duty_cycle;
	else
		wlc->tx_duty_cycle_cck = (u16) duty_cycle;

	return 0;
}

/* push sw hps and wake state through hardware */
static void brcms_c_set_ps_ctrl(struct brcms_c_info *wlc)
{
	u32 v1, v2;
	bool hps;
	bool awake_before;

	hps = brcms_c_ps_allowed(wlc);

	brcms_dbg_mac80211(wlc->hw->d11core, "wl%d: hps %d\n", wlc->pub->unit,
			   hps);

	v1 = bcma_read32(wlc->hw->d11core, D11REGOFFS(maccontrol));
	v2 = MCTL_WAKE;
	if (hps)
		v2 |= MCTL_HPS;

	brcms_b_mctrl(wlc->hw, MCTL_WAKE | MCTL_HPS, v2);

	awake_before = ((v1 & MCTL_WAKE) || ((v1 & MCTL_HPS) == 0));

	if (!awake_before)
		brcms_b_wait_for_wake(wlc->hw);
}

/*
 * Write this BSS config's MAC address to core.
 * Updates RXE match engine.
 */
static int brcms_c_set_mac(struct brcms_bss_cfg *bsscfg)
{
	int err = 0;
	struct brcms_c_info *wlc = bsscfg->wlc;

	/* enter the MAC addr into the RXE match registers */
	brcms_c_set_addrmatch(wlc, RCM_MAC_OFFSET, wlc->pub->cur_etheraddr);

	brcms_c_ampdu_macaddr_upd(wlc);

	return err;
}

/* Write the BSS config's BSSID address to core (set_bssid in d11procs.tcl).
 * Updates RXE match engine.
 */
static void brcms_c_set_bssid(struct brcms_bss_cfg *bsscfg)
{
	/* we need to update BSSID in RXE match registers */
	brcms_c_set_addrmatch(bsscfg->wlc, RCM_BSSID_OFFSET, bsscfg->BSSID);
}

void brcms_c_set_ssid(struct brcms_c_info *wlc, u8 *ssid, size_t ssid_len)
{
	u8 len = min_t(u8, sizeof(wlc->bsscfg->SSID), ssid_len);
	memset(wlc->bsscfg->SSID, 0, sizeof(wlc->bsscfg->SSID));

	memcpy(wlc->bsscfg->SSID, ssid, len);
	wlc->bsscfg->SSID_len = len;
}

static void brcms_b_set_shortslot(struct brcms_hardware *wlc_hw, bool shortslot)
{
	wlc_hw->shortslot = shortslot;

	if (wlc_hw->band->bandtype == BRCM_BAND_2G && wlc_hw->up) {
		brcms_c_suspend_mac_and_wait(wlc_hw->wlc);
		brcms_b_update_slot_timing(wlc_hw, shortslot);
		brcms_c_enable_mac(wlc_hw->wlc);
	}
}

/*
 * Suspend the the MAC and update the slot timing
 * for standard 11b/g (20us slots) or shortslot 11g (9us slots).
 */
static void brcms_c_switch_shortslot(struct brcms_c_info *wlc, bool shortslot)
{
	/* use the override if it is set */
	if (wlc->shortslot_override != BRCMS_SHORTSLOT_AUTO)
		shortslot = (wlc->shortslot_override == BRCMS_SHORTSLOT_ON);

	if (wlc->shortslot == shortslot)
		return;

	wlc->shortslot = shortslot;

	brcms_b_set_shortslot(wlc->hw, shortslot);
}

static void brcms_c_set_home_chanspec(struct brcms_c_info *wlc, u16 chanspec)
{
	if (wlc->home_chanspec != chanspec) {
		wlc->home_chanspec = chanspec;

		if (wlc->pub->associated)
			wlc->bsscfg->current_bss->chanspec = chanspec;
	}
}

void
brcms_b_set_chanspec(struct brcms_hardware *wlc_hw, u16 chanspec,
		      bool mute_tx, struct txpwr_limits *txpwr)
{
	uint bandunit;

	brcms_dbg_mac80211(wlc_hw->d11core, "wl%d: 0x%x\n", wlc_hw->unit,
			   chanspec);

	wlc_hw->chanspec = chanspec;

	/* Switch bands if necessary */
	if (wlc_hw->_nbands > 1) {
		bandunit = chspec_bandunit(chanspec);
		if (wlc_hw->band->bandunit != bandunit) {
			/* brcms_b_setband disables other bandunit,
			 *  use light band switch if not up yet
			 */
			if (wlc_hw->up) {
				wlc_phy_chanspec_radio_set(wlc_hw->
							   bandstate[bandunit]->
							   pi, chanspec);
				brcms_b_setband(wlc_hw, bandunit, chanspec);
			} else {
				brcms_c_setxband(wlc_hw, bandunit);
			}
		}
	}

	wlc_phy_initcal_enable(wlc_hw->band->pi, !mute_tx);

	if (!wlc_hw->up) {
		if (wlc_hw->clk)
			wlc_phy_txpower_limit_set(wlc_hw->band->pi, txpwr,
						  chanspec);
		wlc_phy_chanspec_radio_set(wlc_hw->band->pi, chanspec);
	} else {
		wlc_phy_chanspec_set(wlc_hw->band->pi, chanspec);
		wlc_phy_txpower_limit_set(wlc_hw->band->pi, txpwr, chanspec);

		/* Update muting of the channel */
		brcms_b_mute(wlc_hw, mute_tx);
	}
}

/* switch to and initialize new band */
static void brcms_c_setband(struct brcms_c_info *wlc,
					   uint bandunit)
{
	wlc->band = wlc->bandstate[bandunit];

	if (!wlc->pub->up)
		return;

	/* wait for at least one beacon before entering sleeping state */
	brcms_c_set_ps_ctrl(wlc);

	/* band-specific initializations */
	brcms_c_bsinit(wlc);
}

static void brcms_c_set_chanspec(struct brcms_c_info *wlc, u16 chanspec)
{
	uint bandunit;
	bool switchband = false;
	u16 old_chanspec = wlc->chanspec;

	if (!brcms_c_valid_chanspec_db(wlc->cmi, chanspec)) {
		brcms_err(wlc->hw->d11core, "wl%d: %s: Bad channel %d\n",
			  wlc->pub->unit, __func__, CHSPEC_CHANNEL(chanspec));
		return;
	}

	/* Switch bands if necessary */
	if (wlc->pub->_nbands > 1) {
		bandunit = chspec_bandunit(chanspec);
		if (wlc->band->bandunit != bandunit || wlc->bandinit_pending) {
			switchband = true;
			if (wlc->bandlocked) {
				brcms_err(wlc->hw->d11core,
					  "wl%d: %s: chspec %d band is locked!\n",
					  wlc->pub->unit, __func__,
					  CHSPEC_CHANNEL(chanspec));
				return;
			}
			/*
			 * should the setband call come after the
			 * brcms_b_chanspec() ? if the setband updates
			 * (brcms_c_bsinit) use low level calls to inspect and
			 * set state, the state inspected may be from the wrong
			 * band, or the following brcms_b_set_chanspec() may
			 * undo the work.
			 */
			brcms_c_setband(wlc, bandunit);
		}
	}

	/* sync up phy/radio chanspec */
	brcms_c_set_phy_chanspec(wlc, chanspec);

	/* init antenna selection */
	if (brcms_chspec_bw(old_chanspec) != brcms_chspec_bw(chanspec)) {
		brcms_c_antsel_init(wlc->asi);

		/* Fix the hardware rateset based on bw.
		 * Mainly add MCS32 for 40Mhz, remove MCS 32 for 20Mhz
		 */
		brcms_c_rateset_bw_mcs_filter(&wlc->band->hw_rateset,
			wlc->band->mimo_cap_40 ? brcms_chspec_bw(chanspec) : 0);
	}

	/* update some mac configuration since chanspec changed */
	brcms_c_ucode_mac_upd(wlc);
}

/*
 * This function changes the phytxctl for beacon based on current
 * beacon ratespec AND txant setting as per this table:
 *  ratespec     CCK		ant = wlc->stf->txant
 *		OFDM		ant = 3
 */
void brcms_c_beacon_phytxctl_txant_upd(struct brcms_c_info *wlc,
				       u32 bcn_rspec)
{
	u16 phyctl;
	u16 phytxant = wlc->stf->phytxant;
	u16 mask = PHY_TXC_ANT_MASK;

	/* for non-siso rates or default setting, use the available chains */
	if (BRCMS_PHY_11N_CAP(wlc->band))
		phytxant = brcms_c_stf_phytxchain_sel(wlc, bcn_rspec);

	phyctl = brcms_b_read_shm(wlc->hw, M_BCN_PCTLWD);
	phyctl = (phyctl & ~mask) | phytxant;
	brcms_b_write_shm(wlc->hw, M_BCN_PCTLWD, phyctl);
}

/*
 * centralized protection config change function to simplify debugging, no
 * consistency checking this should be called only on changes to avoid overhead
 * in periodic function
 */
void brcms_c_protection_upd(struct brcms_c_info *wlc, uint idx, int val)
{
	/*
	 * Cannot use brcms_dbg_* here because this function is called
	 * before wlc is sufficiently initialized.
	 */
	BCMMSG(wlc->wiphy, "idx %d, val %d\n", idx, val);

	switch (idx) {
	case BRCMS_PROT_G_SPEC:
		wlc->protection->_g = (bool) val;
		break;
	case BRCMS_PROT_G_OVR:
		wlc->protection->g_override = (s8) val;
		break;
	case BRCMS_PROT_G_USER:
		wlc->protection->gmode_user = (u8) val;
		break;
	case BRCMS_PROT_OVERLAP:
		wlc->protection->overlap = (s8) val;
		break;
	case BRCMS_PROT_N_USER:
		wlc->protection->nmode_user = (s8) val;
		break;
	case BRCMS_PROT_N_CFG:
		wlc->protection->n_cfg = (s8) val;
		break;
	case BRCMS_PROT_N_CFG_OVR:
		wlc->protection->n_cfg_override = (s8) val;
		break;
	case BRCMS_PROT_N_NONGF:
		wlc->protection->nongf = (bool) val;
		break;
	case BRCMS_PROT_N_NONGF_OVR:
		wlc->protection->nongf_override = (s8) val;
		break;
	case BRCMS_PROT_N_PAM_OVR:
		wlc->protection->n_pam_override = (s8) val;
		break;
	case BRCMS_PROT_N_OBSS:
		wlc->protection->n_obss = (bool) val;
		break;

	default:
		break;
	}

}

static void brcms_c_ht_update_sgi_rx(struct brcms_c_info *wlc, int val)
{
	if (wlc->pub->up) {
		brcms_c_update_beacon(wlc);
		brcms_c_update_probe_resp(wlc, true);
	}
}

static void brcms_c_ht_update_ldpc(struct brcms_c_info *wlc, s8 val)
{
	wlc->stf->ldpc = val;

	if (wlc->pub->up) {
		brcms_c_update_beacon(wlc);
		brcms_c_update_probe_resp(wlc, true);
		wlc_phy_ldpc_override_set(wlc->band->pi, (val ? true : false));
	}
}

void brcms_c_wme_setparams(struct brcms_c_info *wlc, u16 aci,
		       const struct ieee80211_tx_queue_params *params,
		       bool suspend)
{
	int i;
	struct shm_acparams acp_shm;
	u16 *shm_entry;

	/* Only apply params if the core is out of reset and has clocks */
	if (!wlc->clk) {
		brcms_err(wlc->hw->d11core, "wl%d: %s : no-clock\n",
			  wlc->pub->unit, __func__);
		return;
	}

	memset(&acp_shm, 0, sizeof(struct shm_acparams));
	/* fill in shm ac params struct */
	acp_shm.txop = params->txop;
	/* convert from units of 32us to us for ucode */
	wlc->edcf_txop[aci & 0x3] = acp_shm.txop =
	    EDCF_TXOP2USEC(acp_shm.txop);
	acp_shm.aifs = (params->aifs & EDCF_AIFSN_MASK);

	if (aci == IEEE80211_AC_VI && acp_shm.txop == 0
	    && acp_shm.aifs < EDCF_AIFSN_MAX)
		acp_shm.aifs++;

	if (acp_shm.aifs < EDCF_AIFSN_MIN
	    || acp_shm.aifs > EDCF_AIFSN_MAX) {
		brcms_err(wlc->hw->d11core, "wl%d: edcf_setparams: bad "
			  "aifs %d\n", wlc->pub->unit, acp_shm.aifs);
	} else {
		acp_shm.cwmin = params->cw_min;
		acp_shm.cwmax = params->cw_max;
		acp_shm.cwcur = acp_shm.cwmin;
		acp_shm.bslots =
			bcma_read16(wlc->hw->d11core, D11REGOFFS(tsf_random)) &
			acp_shm.cwcur;
		acp_shm.reggap = acp_shm.bslots + acp_shm.aifs;
		/* Indicate the new params to the ucode */
		acp_shm.status = brcms_b_read_shm(wlc->hw, (M_EDCF_QINFO +
						  wme_ac2fifo[aci] *
						  M_EDCF_QLEN +
						  M_EDCF_STATUS_OFF));
		acp_shm.status |= WME_STATUS_NEWAC;

		/* Fill in shm acparam table */
		shm_entry = (u16 *) &acp_shm;
	 M_EDCF_STATUS_OFF));
R
p_shmWAC;

		/* Fill in shm acparam table */
		shm_entry = (u16k;
	caxu16 *) &acp_shm;
	 M_EDCF_STATUS_OFF));
R
p_shmWAC;

		/*  acpppppppppppppp2XOFF));
R
p_shmWAC;
ySc_hw, .tMc:xc:OHR
p_shmWAC;
ySxc:xc:xc:grcms2%_EDCF_STATUS_OFF));
R
p_shmWAC;

		/*  acpppppppppppppp2XO?5rn;

	/"aifs 2%:xc:grcms2%_EDCF_STATUS_OFF));
R
p_shmWAC;

		/*  acppppppppppppppppppp2%  acpppppppppppppp2XO?5rn;

	/"aifs 2%:xc:grcms2%_EDCF_STAts ++++++++++++++2[x));
R
p_shmWAC;

		/*  acppppppppppppppppppp2%  acpppppppp?Hc:xc:g(u16) 2Rb{ acppppppppppHcms2%_EDCF_STAts ++++++++++++++2[x));
R
p_shmWAC;

		/*  a?: */
 Oxppppppppppp2%  acpppppppp?Hc:xc:g(u16) 2Rb{ acppppppppppHc?>DCF_/* p*ECWppppHc?>DCF_/* p*& acpppp},(u16) 2Rb{ acpKppppppppHc?>DCF_/* K*ECWppppHc?>DCF_/* K*& acpppp},(u16) 2Rb{ acVIppppppppHc?>DCF_/*VIpECWppppHc?>DCF_/*VIp& acpppp},(u16) 2Rb{ acVOppppppppHc?>DCF_/*VOpECWppppHc?>DCF_/*VOp& acpppp}ppp; /e
			 * inits th>puaci] *eruct ->bandlop);* restore macintmasC;

		/*  a?: */
 Oxpppppp* */
 Oxpstruppppp2%  acpppppppp?Hc0]
		p[i + 1ppp M_EDCFppp <shm.aifs < NUMF_/SDCFppp++,: */
 Oxp
		 * wh ucod];

	  te == 0cion_upde  wlc (!wlc-> "wint idx_QLEN +	 M_( */
 Oxp->pppMAX)
		acCIm.aifsGOFF)
		acCImtatuse, BRCMS_RATES_Cwlc->edcf_txop[aci & 0x3]	++2[x));shm.txop */
 Oxp->& ac;3]	++2[x));sm.aifs  */
 Oxp->pppe, BRCMSCW =
			2^(ECW =
)d(st 0x3]	++2[x));sd16(wl			)
		aECW2CW( */
 Oxp->ECWMAX)
		aECW		 m.aifs++BRCMSCW  D11R2^(ECW ax)d(st 0x3]	++2[x));sd16( D11R)
		aECW2CW(( */
 Oxp->ECWMAX)
		aECW	AXm.aifs for at lGOFF)
		aECW	AXmtatus));
	}
}

voi
		       boolp2%  m_ac &++2[x));,pppppp?Hcpppppppppppppp2XO?5c_enable_mac(wlc_hw->wlc);
	}
}

/*
 tandard 11b/g (20us slots) if (wlc->pub->up) {
		brcms_cc_phy_mo reor_bcma_e on the home channel
	 */
	if (wDoJADDbcma_ */
stimer: noHWRADIO feTUS\n",
	hy_chandndunit || wlc-c_phy_mo reorms_c_set_home_chansc_phy_mo reorcked!\n",
pp2XOFF)q, req_acp_shm.sd!\nb_antsel_set(stRADIO_M_bit);pp2XOFadd_timer| wlc-c_phy_timer, TIMER_INTERVALtRADIOCHKms(structc->pub->up*/
	return fac_phy_mo reor_bcopu16 txfunfl[NFIFO];
#endif				/return;
	c_phy_mo reorms_c_set_h(macintstchansc_phy_mo reorcke_err(wlcpp2XOFF)q, req_acp_shm.st basic rate _set(stRADIO_M_bit);_set_h((wlc->wel_timer| wlc-c_phy_timerct brcms_OFF) hwct brcmsthe wo	if (ropagasy_rat
	caROMIet)
{
	u8 rate;
	u8 mandatphy_hwct brcme are on the home channel
	 */
	ifoverride_set(whw_r */s_c_set_home_Fix the habdatphy_OFF))hwct brcmdint brcms/s_c 1 to match e_set(watphy_ct brcmd, WLtRADIO_HWtsf_ABLEit);rdware >sbclk)
				e_set(watphy_ct brcmd, WLtRADIO_HWtsf_ABLEit)brcms_on chanhwpec_bwTATUS_O	if _set_h(dunit */
	return fa_prot_atphy_ct brcmdunit, wlc->band->bandunit);

	/* wrindatphy_hwct brcme arewates RXE match
	}
}

static e_set(watphy_ct brcmd, WLtRADIO_HWtsf_ABLEi ? for ieee80211_to *wlc)
{annot ual  ++= (hunt;c_bwbutt->banicmshe mac ,
	"				"et)
{
	u8 rate;
	u8 mandatphy_timer|ate;
*argve been bandlocked dlc, RCM_MAC_unit, wlc->band->bandu) argome_Fix the ha
	brcms_c_flushquANNEL(chanspec));
		return;
	}

	/* Switch bandes if ip fill in shm ac params struct */
	acp_sthe ha
			pec));
	wlc_shm.txop = par/* wrindatphy_hwct brcme arewates *wlc)
commo  the-followw *ssdog _EDCF_QLhortslot;

	if (wlc_w *ssdog->d11core;
	u32 sflags;
	u32 bcnint_us;
	uint i = 0;
	bool fifosz_fixupnspec);
		wlc_phy_

	/* band-speci betwmes_c_baticad mac stat
		wlc_pnowcore, CMSC;
		if (d fash enorms_c_setxband(wlif (wlc_:grc enor	  " %d\cms_c_anf);

	/* bRXFO]);t, _t(struct brcms_har2 macint(!brcms_dool fastclktxpower_liw *ssdog-;
	}
}

/* switces *wlc)
commo  w *ssdog _EDCF_QLhortslot;

	if (wlc_w *ssdog->d11core;
	u32 sflags;
	u32 b/* write ucode ACK/CTS rate table */
	et our snapshot of macst_set_ps_ctrl(wlc);

	/* band-spFix the ha
	brcms_c_flushquANNEL(chanspec));
		return;
	}

	/* Switch bandes if ip fill in shm ac params struct */
	acp_sthe ha
			pec));
	wlc_shm.txop = pareci betwmes_c_baticad mac stat
		ac paranowcore, /* wrindatphy_hwct brcme arewates ->macstt;c_bw,
	hy_chan,she mac 
			brc				brqudun "idxdunit || wlc->bandatphy_ct brcmd;

	/* band-spif (wlc_w *ssdog-uct brcms_itch occas->banly sample				sizeo

void brctoitch deruct 16-->d1
void b wpep/*
			 * BM(
		ac paranow % SW_TIMER_ con
p_s_UPbrcms_c_gpio_initizeof(struct macstarr(core, "%s: wl%d: BCN_PCT && "aifsM(
		ac paranow -shm acneedDEBUG_oftwtimesGO0211_ACalgo_chaEMPSENSE_PERIOD) chanspec;
needDEBUG_oftwtimefifosz_f paranowandard 11b/gneedDEBUG_ arewates -y_rate, index;
	u8 basic_p *ssdog_by_timer|ate;
*argve been bandlocked dlc, RCM_MAC_unit, wlc->band->bandu) argome_if (wlc_w *ssdog-TAts +++++++++++*/
	return fatimerscck_basic = 0;
	u8 ofdm_basic =
}

sbeacon before wdtimer:c_bandunitit_timer| wlc-ed a8 basic_p *ssdog_by_timerc_rate_/* S *ssdog"es -_set_ps_ctwdtimer chanspool)ec));
		rebool) vabusy_rawlnitit_timerif (dwdtimer:cwmax = faicm	et ounna selecgotbasiil;phy_txpowc-c_phy_timer:c_bandunitit_timer| wlc-ed a8 basic_c_phy_timer,_rate_/* c_phy"es -_set_ps_ctc_phy_timercchanspool)ec));
		rebool) vabusy_rawlnitit_timerif (dc_phy_timer:cwmax = faicm	et ounna selecgotbasiil;phy_tx_set_h(macintsasiil:tx_set_h(211_to *wlc)
24M)
>band = w;
	u8 ofdm_baphytxchabeacon ..eed t
			ge  wlc, u16s CN_Tr
	/* Ud\n", idx, e if it is set */
	if (wldm_bcck_basic = 0;
	u8 ofdm_basic =
}

sbeacon beTE + 1);

	/is locale */
	brcms_c_channel_set_chanspec(wlc->cmi,20mhz_mac_up(= 0; i < vannoS_Oake angCTL_Is_set_chanspec(struct b_err(wlcchanspec(struc
	default:
	 == shortslot)
		retome_if (wlc_ this function ic =
ection->gmode_us=
ection->gmECTION		retu;e_if (wlc_ this function ic =
ection->gmode= (spppppppp2XO?if (wlc_ this function ic =
ection->gmotion->non= 0;
	u8 *b
ection->gmECTION		retu;e_if (wlc_ this function ic =
ection->gmotion-=
ectionNn->gmECTION	O= 0; i f (wlc_ this function ic =
ection->gmotiection->n= 0;
	u8 *b
ection->gmECTION		retu;e_if (wlc_ this function ic =
ection->gmotiectiopppppppp2X_if (wlc_ this function ic =
ection->gmotin->n_ob, 	retu;eX_if (wlc_ this function ic =
ection->gmoe_user = 0;
	u8 *b
ection->gmECTION	MAC e_user  0; i < ake angCdraft 4.0 NonERP echaadC(aciswmes_cset_chans beludG_oegac)ec)FFS(macintst, true);
	xantrx
	dr:c_wlc-oolDIV_DEF;st, true);
	phyctl =wlc-TX_DEF;sat
		ac rb acpp_time
	  =
ection->B_TL_KETIMEOUT;sat
		acusr_fraglc_hw, =
lc_hw-DEFAULT_FRAG_LEN;a engine(s) and post receive anspec;
fraglc_hw,SM
 * lc_hw-DEFAULT_FRAG_LEN;a pec;
sloTc_hw, =
lc_hw-DEFAULT_slo_LEN;atializhytxcha}

stawlc_hw, M_LFRMTXCNTFBRTHSpec;
tl), =
RETRYhortsl_FB;a pec;
Ll), =
RETRYhLcti_FB;atializhytxcha			sM_LFRMTXCNTFBRTHSpec;
tR, =
RETRYhortsl_DEF;st, truLR, =
RETRYhLcti_DEF;sat
 */ME QoSCTL_INd\nAulocayizhytxchastat
		ac para11procl =wMPDU_AGG_HOST +++++++++++u;

	/* enterattach_			coe, basic_rate;
	uint i;

	rs_dfu, "wl%d: core i}

sbeacre itchbanhm ac params s;sat
		acas-l = (phyctle MCS 3attachewates -t || wlc-as-l =b & Scchanspool)ec));
		rebool) vabusy_rattach: e MCS 3attach:cwmax = faicm	et ounna selecl%d: c44elecgotbasiil;phy_txpowc-1procl = (phyctleprocsattachewates -t || wlc-aprocl =b & Scchanspool)ec));
		rebool) vabusy_rattach: eprocsattach:cwmax = faicm	et ounna selecl%d: c50elecgotbasiil;phy_txnit: u(phyctl & ~attachewate * arecchanspool)ec));
		rebool) vabusy_rattach:  & ~attach:cwmax = faicm	et ounna selecl%d: c68elecgotbasiil;phy_asiil:tx_set_h( engine. basic_rate;
 pates  (wlc_ ub, basic_rate;
	uint i;

	rs_df_set_h(hm ac pas *wlc)
the followattachfo *wlcruc->sckpla	(vottach,ntsel_nvramfo *wlcruc-tes[attachfo *wlcc->band = w= wlc->bandstatate corrock\n"	if N_PCfo *wlcp_c_bandw			cof ipable nd->hhe mac ockedndsta)ion
__func bsscfg->wlc;

	/* entb~attachetype,
		wlc->band->bandtype, falses + 
	brcm *cle)
		idlei}

sbeac  acppppio_set_r2 bcnint_us;
	uint i = 0;
	bool ;dfu, "wl%d: core i}

sj;chanspewmefif_err(wlcR
p_shmWA_x_1er_licf_txop[haicf_txowlcR
p_shmbool);
	ool);anhm ac	ool)wlcR
p_shmpci 
	b *pci
	b = chans>bus;
	} telchas clocks *sb_soid  *soid  truchans>bus;
soid cstarr(cchans>bus;
	} ttiming(wlspecHOSTTYPE_PCI_gpio_initte ucode }

	/* Switchvendte 	if  
	brcm 	if (wlc_nds > 1) {
	*wlcpci
	b->vendte> 1) {
	*wlcpci
	b->
	brcmit);rdware o_initte ucode }

	/* Switchvendte 	if  
	brcm 	if (wlc_nds > 1) {
	*wlcchans>bus;
bot icode.vendte> 1) {
	*wlcchans>bus;
bot icode.timiclktxpmefifmacintst, tol fifosz_fixupt
		wlc_pwbrcms_b_upt
		wlc_pitchbanbeacre ;
	}
}

/* s rrupts */
eping stat0]
	 ;
	}
}

_pio_set11REio_set;sat
 *	M_BCN_TXcnint_us;
	uint i = 0;w, (bphytxchabeacon nd(wlif (wlc_dm_bcck_ba " %d\cms_c_adate to_c_rateset_bw_ry iacinbrcms_cattach. Alsocc->band = w= wlc->bdate hps;
	boatpec_gets  u8 baste[tx_rate]nt i = 0;wy addreunning->protec;
	}
}

si, =
ai~attachechans>buses -t || wl}
}

si, ==b & Scchanspool)ec));
ool) vabusy_r/* entb~attach:  i~attach faicm	et o		idleina selecl%d: c11elecgotbasiil;phy_tx < vererioagat down,
	brcm se BRtry in t);
		retueturn fa_pip8 *ssichanecchanspool)ec));
ool) vabusy_r/* entb~attach: UnBRtry in t
	brcmet o		idlina selecl%d: c12elecgotbasiil;phy_txnit:chans>bus;
	} ttiming(wlspecHOSTTYPE_PCI_r, chansp
}

vendteis rrpci
	b->vendte; chansp
}


	brcmis rrpci
	b->
	brcm;pi, txpwr, chansp
}

vendteis rrchans>bus;
bot icode.vendte; chansp
}


	brcmis rrchans>bus;
bot icode.timi;phy_txpowands if nece rrchan
	 ;
	}
}

lities *rrchans>id.es 0; i < vali chan_pip,n_pipes *levelities */
	b	retueturn faisgood_pipa " %d\c_r, chl%d: c13elecgotbasiil;phy_tx < c->band = w(wlc->nt filteid(struct brcmp_dly);

	nits */
	brcmerev *(wlc_hw, BCMAct brcE_FA		if maciAct brcE_FAatic conratenbrcattachf
			 *ectionefg)
ock\n"		  wlc->pub->prowlcratePMUn_pips undo fiRST)*/
	c,
				    in th-opeacon bahans   >prowlcse Bific _err(w B	  it*
	 * t brcms_c_agat dinsr thiifo_sz[rince chrowlc inspeg)
o,
			  wlc->pub.ateset *
brcms_
	/* set up the specified band and chansk */
	brcms_resi_clkctl_faectionUSE_COREte >Smacst_set_k */
	brvali chaa_pip_act !=a " %d\c_r, chpool)ec));
ool) vabusy_r/* entb~attach: vali chaa_pip_act !=:cwmax faicm	et ounna selecl%d: c14elecgotbasiil;phy_txlc_gMS_TXPWbot ic->vwlc, d ju(ratethe set /* fsoid ;
bot i_es 0;up_delamo_TXcid  bot ies *wlc0xFFk */
	/
	b	retjng(wlOARDREVnel dOTABLEilec/* flOARDREVnel dOTEDre ;
	}
}

/ot ies *set_ps_cj;	b	retueturn favali /ot itimia " %d\c_r, chpool)ec));
ool) vabusy_r/* entb~attach: UnBRtry in tBroadcd  cwmax = bot ictimin(	if )" "stf_es is->b follow(	if )et o		idleina ,
ai~
	/*/ot itimia " %d\cmerevll in shm }
}

/ot ies selecl%d: c15elecgotbasiil;phy_c;
	}
}

sid es *sesoid ;
es is->bre ;
	}
}

/ot iags & FIsoid ;
bot iags &_lo + (soid ;
bot iags &_hiREGOuty_c ;
	}
}

/ot iags &2 FIsoid ;
bot iags &2_lo + (soid ;
bot iags &2_hiREGOuty_c -t || wl}
}

/ot iags & & BFL_NOPLLDOWNmatch engineq, req_acp_hm.sd!\nb_antsel_set(stSHAREac_suspec_prot,
	brcm sd(sid ,_nvram etc.) to popuit_penndunit) {
			/* 
	brcmis r(wlsp43224_Dwlc-ID ||211_AC{
			/* 
	brcmis r(wlsp43224_Dwlc-ID_VEN1 ||211_AC{
			/* 
	brcmis r(wlsp43224_CHIP-ID)+BRCMSDual/* s /ot ibrcms_c;
	}
}

_band_ord 2;hardware */
}
}

_band_ord 1_c -t ||(ai~
	/*_pip_ida " %d\cmerevng(wlspecCHIP-ID_lsp43225))re */
}
}

_band_ord 1_c -/t
			 * up thcap_40 tsel_wlc patbeacon fisOF/* enterattach()
rcms_caticit->banly doatespectsel_wlcth>puabeacon>protec;
	

vendteis rransp
}

vendteis;st, tru
	brcmis rr{
			/* 
	brcmis;at
		ac parasi, =
 " %d\cmere;at
		ac paralities *rr;
	}
}

lities ;at
		ac parasid es *se;
	}
}

sid es ;at
		ac para/ot ies *sehm }
}

/ot ies ;at
		ac para/ot iags & FI wl}
}

/ot iags &;at
		ac para/ot iags &2 FI wl}
}

/ot iags &2;at
		ac para_band_ord */
}
}

_band_o = chspec_baol)shim ething thashim~attachewat_hm.s wlc-ed at macstarr(chspec_baol)shim e=b & Scchanspool)ec));
ool) vabusy_r/* entb~attach: hing thashim~attach:cwmax faicm	et ounna selecl%d: c25elecgotbasiil;phy_;up_dea!=: cal baste[ *eruct rat
	cg thash_x_1eattach:t ds */[aci & 0x3][haicf_txo.si, =
 " %d\cmere;at[haicf_txo.ol)shim ethingc_baol)shim;at[haicf_txo.itchbanbeacre [haicf_txo.lities *rr;
	}
}

lities ;at[haicf_txo.vis rransp
}

vendteis;st[haicf_txo.dis rr{
			/* 
	brcmis;at[haicf_txo.l ipa= ai~
	/*_pip_ida " %d\cmerev;at[haicf_txo.l ipes *rrai~
	/*_pipes a " %d\cmerev;at[haicf_txo.l ippkg*rrai~
	/*_pippkga " %d\cmerev;at[haicf_txo.sid es *se;
	}
}

sid es ;at[haicf_txo./ot itimi*rrai~
	/*/ot itimia " %d\cmerev;at[haicf_txo./ot ies *sehm }
}

/ot ies ;at[haicf_txo./ot iags & FI wl}
}

/ot iags &;at[haicf_txo./ot iags &2 FI wl}
}

/ot iags &2;a;up_d bcm

	if ss lolc->hw, (diio_x_16ol);the wo	rearotec;
	}
}

 thash ething thash_x_1eattach(&[haicf_txoes -_set_ps_}
}

 thash_r, chl%d: c16elecgotbasiil;phy_tx < c->band = w= wlc->bandstatate corrock\n"	if N_PCs_basic_raj(s) anj_rates}
}

_band_o  j
		 * wh u
, banduni0is_oflways 2.4G_bw(chanduni1,acstpresin ,
s_o5G_bw(chansgpio_initizew->up) {
		if (wj)e */
		bwitch if not up yet
=cj;	b
		bwitch if not uptimi*rrj ?band->gmode)
 :wlc_hw, short;anspec;
 if not up yet
=cj;	b
		bch if not uptimi*rrj ?band->gmode)
 :wlc_hw, short;anspec;
chans>cms_idx*rrchans>chaning */
		b
		bwitchmorrwc_b_rems_b_mctrl(w_WAKE | MCTL_HPS, v2rwc_b));	b
		bwitchmorrwc_b_>sckup FI wl}
}

morrwc_bradio chanspetx :grcert frcms_cWARN_ON(;
	}
}

lities _raXMTt reTBLn
p_RTREV ||21		(;
	}
}

lities _-aXMTt reTBLn
p_RTREV) and(wlARRAY_SIZE(xmt:grc_sz));	b
		bwitchxmt:grc_szm.bsl1_ACxmt:grc_sz[(;
	}
}

lities _-aXMTt reTBLn
p_RTREV)]
	 cWARN_ON(!		bwitchxmt:grc_sz[0]	 */
		brGub->6ol);_algosel_N_PCs_bas
		bwitch if nopim.bslohing thaattachewat_hm

 thash, NEL(chanspe(wlc);

	witch if not uptimichanspe(wlc);

	ac	ool)light band switch if nop-l =b & Scchansspool)ec));
ool) vabusy_r/* entb~attach: hing tha
	}

	/* attach faicm	et ounna selechl%d: c17elechgotbasiil;ph u16 ching thamorrwc_b__tx);
	}
}

/* switch  wl}
}

morrwc_b)e */
		bw thags_chspvucti		/*  }
}

/* switch &*  }
}

/* swit	brcms_chsspe(wlc);&*  }
}

/* swit	b->vwchsspe(wlc);&*  }
}

/* swic_phyidwchsspe(wlc);&*  }
}

/* swic_phyes selec*  }
}

/* swiabgol)ecnnece rbsl1_AC		bw thags_ccnnece-;
	}
}

/* switces b
		bch if noabgol)ecnnece rC		bw thags_ccnnece-;
	}
}

/* switces b
		b}
}

/* swichaniags & Fbsl1_AC		bw thags_cchanags &-;
	}
}

/* switces /
		brvereriogoodlookitimi*& BRtry in ttes[is is->b et->ratesore, "%s: wl%d: }
}

/* s >= BRCMatesNCONF_HAS-;
	}
}

/* swit	b->v if we gotbagood_ol)wlc			mandatorgotbabad_ol)wlc	
}

static ore, "%sLC: wl%d: }
}

/* s >= BRCMatesLC:CONF_HAS-;
	}
}

/* swit	b->v if we gotbagood_ol)wlc			mandatorgotbabad_ol)wlc	
}

sta{
abad_ol):ansspool)ec));
ool) vabusy_r/* entb~attach: unBRtry in t
	}

	/* tes[timi/es _(%d/%d)(wlc_nds > 1) 	);

	witch if not	brcms_;

	witch if not	bes selechl%d: c18elechgotbasiil;ph u16agood_ol): wh u
, ban			 * up th		bch if nop-lines should wrpopuiethe 	if snes sw(chandwrite it down,ate  followattach. How
	fo;wy ke souldf);

e] != 0eo

 avoiduntil: calthe followact !=:al);rrent
 toitc_rat
	}
}

/* switc. Instes i_set_phy	bch if nop-lanspecethe,itc_rak
staticd switch if nop-lan felK		ant betwmes_al_on chanofitc_rathe followfnsM_RATEcu  wlc,athe info anspefisOF calfns
 If
			nd;
	if we are y	bch if nop-l=cd switch if nop-s b
		bch if not	brcmsl=cd switch if nop	brcmss b
		bch if not	bes *sehm }
}

/if not	bes s b
		bch if noc_phyid*sehm }
}

/if noc_phyids b
		bch if noc_phyes *sehm }
}

/if noc_phyes ;re o_initte ucode }

	/* Switchtes[%u/%utt;c_bw%x/%u(wlc_nds > 1) {
	*wlc		bch if not	brcms, 		bch if not	bes > 1) {
	*wlc		bch if noc_phyidwc		bch if noc_phyes selecalizhytxchant fes_->b wng owsert frTXCNTFBRTHS
		b}
}

/* swiCW =
			Ashm(CW		 s b
		b}
}

/* swiCW  D11Rshm(CW	AXhanspec(s!/* entb~attach_dmapio ic =
j,ewme >= BRCMl%d: c19elechgotbasiil;ph u1hy_tx < ct brcmsnece tbaset_sshe mac "				"e void brcms_c_set_necect brcm  " %d\cms_c_anMet_sshe mac "				"e void brcmss_b_	} telcha
			pec)ands if neces>buses _c_anet_h( dutpcalRATEx_al_tbaset_sshe mac "				"e void brcms_c_seb_x_al{
		if (wO= 0; _c_an*******************************************************************
rcmsT_rateset_bw_bcn  down,DOWN;the wo	tgosel_lc->h. | Msneceitch orsnecec_bandble nd->;w, (b_func__ du,d || wlc-bot icPLL= rate>pub-dutcstpossiRATE_MAS rateBeyo|| wlel_lc->h0; i < rb    =if_err(_RATEc ipaid(struct
_c_bsinit) ould wrtouchlc->prooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
chansgp chanspe core (se;the wovannchanspbrcms_c_set_gthe MA (seewat_hm.s wlands  core (set_bssle;

	_broadca te core_ (seewat_hms  core (set ||211_AC
	_zeroe core_ (seewat_hms  core (setcchanspool)ec));
ool) vabusy_r/* entb~attach: shm. MA (seet o		idleina selecl%d: c22elecgotbasiil;phy_tx/* write ucode ACKands if nece va
	brcmis 	if  band_or%s /ot i 	if (wlc
spe(wlc);

	witch
	brcmis,ates}
}

_band_oc
spe(wlc);ai~
	/*/ot itimia " %d\cmereves RXE match engi_asiil:txpool)ec));
ool) vabusy_r/* entb~attach: faicm	;w, (bl%d:%d(wlc_nds > 1) bl%dit);_set_h( engine. b+++++*/
	return faattach_ & ~antcck_basic = 0;
	u8 ofdm_basic on beTE +aa;e i}

sbeacre db(wlc->rcmss bclocks *sb_soid  *soid  truw		return;
	}

	s>bus;
soid cstaitchbanhm ac params s;s	t uptimi*rr		bch if not uptimi;_txlc_gMS_init(wlsrspec);

	p	/* Fix taif (wlc->band->gmode)
		rsaa FIsoid ;
antcspec);

	_a;hardware aa FIsoid ;
antcspec);

	_bg_c -t ||(aa_ra1BSS coaa_> 15ecchanspool)ec));
		rebool) vabusy_r banInvali _init(wlsrspec);

	pincwmax = Xcid  (	if ),_rate_s3(wlc_nds >ds > 1) {
	aam;
	 Ma FI3;phy_tx <  nd->;own,
	ytxchsms_c_wrhs loaeacogMS_RAit(wlc	/* Fix Ma F=ing) {
	w true);
	xantrx
	dr:c_wlc-oolDIV_FORCE_0;{
	w true);
	phyctl =wlc-TX_FORCE_0;{

}

static Ma F=i2g) {
	w true);
	xantrx
	dr:c_wlc-oolDIV_FORCE_1;{
	w true);
	phyctl =wlc-TX_FORCE_1;pi, txpwr, cy_tx < Compu woAAit(wlcGat d	/* Fix taif (wlc->band->gmode)
		rs		bch if noantgat dFIsoid ;
antt(wl_gat .a1;hardware */
st if noantgat dFIsoid ;
antt(wl_gat .a0s RXE matchmacintrate, index;
	u8 basic_lc, ppppp2% ck_basic = 0;
	u8 ofdm_basic on be_c_valid_cha; bcnint_us;
	uiN_PCs_N_PC; bcnint_us;
	uiNssfdm_basbi*rr		bchppppp2% Nssms_c_antsel_zhytxcha || wargMS_oid w, (bges tsane c->bandtbeacon 	/* ams->txbiconvert fromsbi));	bbist  = wlc-annot* flEACON_INTERVALtDEFAULTd))
		phical baszhytxchan necessasundo fiRST)vali _) {
			/*
	Dbcma_te_sms_c_setb2G_) {
			n>protecpec(wlc->cmi,20mhz_mac_up(= 0;spec;
	}
}

void
brcmsbistndunit(chanspec);
		if (wlfb_read_sN_PCsofocalezhytxchan necessate-setering sleepins -t || wlc->bandinit_pendin && "aifs		brcms_err(wlc->hd = true;
			if (wlc->banmatchatering sleeping statOTHERgmodUNITewate]ms_c_antsel__infms_c_s					  hater *wlc, u1zhytxcha}

stCMS_SHOR8 basic_c_}

	/*zhytxch(&bistn since b & S, /* swit	brcms_chs if not uptimicst basic rate rcms_c_bs_F& S,chs*wlc, i| wlc->banding (20 & SUPPtsl_11Nvll i8 basicac_upd(wlc);
}

/*0; i < rs.count; i++) {
		t || wlc->banding (20 & SUPPtsl_11Nvl i8istags & |=
ectionoid_HTntrate, index;
	u8 basic_pppppppd */
t upd(wcapu16 txfunfl[NFIFO];
#endicfg->(wcaprs_dfu, "wi; bcnint_us;
	uiN_PCs_N_PC; a engine(s) and po wlc->bandinit_pe at
		 * whhatering sleeping stati]ight ban if not uptimi*r>band->gmode)
	= BRCMates(bwc_b_r=
ectionNnBW_40ALL) 1) {
	*S cobwc_b_r=
ectionNnBW_20IN2G_40IN)
	if we ahanged */
	brcms_ked!\n",
			mandatorahanged */
	brcms_ke_err(wlc!mute_tx);

	 ban wc_b_r=
ectionNnBW_40ALL) 1)  ahanged */
	brcms_ked!\n",
			mandatorahanged */
	brcms_ke_err(wlc!m -y_rate, index;
	u8 basic_timerscdeck_basic = 0;
	u8 ofdm_basic on be(wlfre
stimer: void brcmt || wlc-wdtimer chans;
	u8 fre
_timer| wlc-edtimerct 	efore wdtimer:c_ & S;red memorws_ctc_phy_timercchans;
	u8 fre
_timer| wlc-c_phy_timerct 	xpowc-c_phy_timer:c_ & S;red rate, index;
	u8 basic_detach_			coe, basic_rate;
	uint i;

	rs_dft || wlc-as-r 40Mhz, remove MCS 3detach| wlc-as-rt 	xpowc-as-l = & S;red  -t || wlc-aprocr 40Mhz, removeprocsdetach| wlc-aprocrt 	xpowc-aprocl = & S;red  -u(phyctl & ~detach| wlng this shouthe followdetache if it is set */
	if (b~detach| basic_rate;
	uint i;

	rs_dfu, "wi; bcnint_us;
	uihwiN_PCs_N_PC; bcnint_us;
	uint i = 0;
	bool fifosz_fixupns
	if (b~detach_dmapio ic %d\cms_c/* s rrupts */
epin;a engine(s) and po wl}
}

_band_o  t
		 * wh ban if nopif (wlc_hw-Detachgosel_N_PC'shtes[(wlc, 		bw thadetach|/* switces b
	 if nop-l=c & S;re		 * /* s rrupts */
eping statOTHERgmodUNITewate]mscy_tx < Fre
sio_x_16ol);the wo(wlckfre
ewat_hm

 thashclktxpower_lishim~detach| wlgc_baol)shimacstarr(chspec_baerevn(wlcai~detach| wlgc_baerev;atc;
	}
}

si, =
 & S;red rats shouR matchaad mac brcms_cnumber:brche mac cwlc_hw,e Bific %s: chseed shouGenre c %olicy_bcn 0eo
8 basic_detach ke sinfo de bcm
/fre
s= wlc->bandstaseed tIt ke sNOTrtouchateset_bw_md(struct acon bonefg)
nece 
			brcble nd->;_PCfo *rcE_FA
			ould wrspec);

	eed tOne excepdx, val)sb_md(strucwact !=, te == el_lcssiRATaticcrys_al_bcn atceCfo *, v inspe"				"e void,she mac sinit) , int = wlc->batimer:w, (bonefexcepdx, fo *,stt;c_b_mo reorde if u;

	/* enterdetach| basic_rate;
	uint i;

	rs_dfu, "wcwlc_hw,ecstarr(chspl =b & Sc
	cms_c_info ns
	if (b~detach-uct brcms_ delete = wlc->batimern 	/* cwlc_hw,e  core 	retueturn fac_phy_mo reor_bcopuuct c
	ccwlc_hw,ecore, /* wrindn neces_mgrsdetach| wlc-cmiu;eX_if (wlc_timerscdeck_bauct brcm8 basic_detach_			coe,uct brcm8 basic_detach_	fre
ewatit);_set_h(cwlc_hw,ecsbrcms_on chanhps;
	boatpec_gets  u8 basnt
 *		 (100 - d "ap"et)
{
	u8 rate;
	u8 mandbrc are on the home channel
	 */
	if
	/iTA-oid;>home_(cwp;

	p	/*  wlc-PLCPHdr
	default:
	 == shPLCPhortslcsbrcms_)
>band = wju(rac_rateset_bw_fisOFcomic vo	  wlcPts orsS3/S5 sys_emandstas if it is set */
	if (b~hwiupandunit;

	brcms_dbg_mac80211(rs_dft || wl_lc_pwbre_set(whw_c);

	/* band-sp/* write ucode ACKands if nece va*/
	et our s_lc_pitchcms_c_adate E(20ustpcalRATEx_al, c->band = w bastwlc->nt filteid(struct chrow		if maciAct brcE_FAatic conramang *r:brc8 basic_pp().ateset *
brcms_x_al{
		if (wObit);p_dly);

	nits */
	brcmerev * *
brcms_
	/* set up the specified band and chan_c_adate TODO: tatenwlc_hw-/ ndume_MAS rateAIEc ipadoatJADDrateece bar0win2  u ratehibern maci/ ndume,cfg->Bswlfbxup
chansgp c rateInct b6ol);boatpacPts  nd->;t, _ocnt
 *t =  rateel_zoateaad mpletec    c->b>protec;
	}r_licor_inct b-;
	}
}

/* switces /
r s_lc_pispec loadercms_err(wlc wl_lc_pwbre_set(whw_c)fifmacintst* BM(
		}
}

/ot iags & & BFL_FEurn hm.aifs(ai~
	/*_pip_ida " %d\cmerevng(wlspecCHIP-ID_lsp4313)	 * wh ban!
spe(wl(
		}
}

/ot ies *> cox1250
spe(wlcifs(
		}
}

/ot iags & & BFL_FEu_BT)	if weai~epa_4313bg_ */
	brcmerev * d rate, inde;

	/* entb~up_prepandunit;

	brcms_dbg_mac80211(rs_df/* write ucode ACKands if nece va*/
	et our s_lc_pitchcms_c_adate E(20ustpcalRATEx_al, c->band = w bastwlc->nt filteid(struct chrow		if maciAct brcE_FAatic conramang *r:brc8 basic_pp().ateset *
brcms_x_al{
		if (wObit);p_dly);

	nits */
	brcmerev * *
brcms_
	/* set up the specified band and chan_c_adate Cytxctl erpci/pcmcian "idxinstes ibrciOF/* enterattach()
rcmsidx,
	/* mfg hotswap: c				brhotswap (c ipatwlc->nruct),_rp.ateset *s_b_	} telchairq_* spec)ands if neces>busour s_lc_pHANNEL(chans(wlc);(struct_c_adate Ng->BSSIOFF) c_ratwpec_bwTATUS_O "idxt brcv_etherac voiw "idxthbdate hys_ema			 *adercw, (bonefhunt;c_bwct brcmd. Wwrit	ouldwyctltoitch  *ectionefge mac up
	/* Ud\nc vo.ateset Fix the habdatphy_OFF))hwct brcmdint _d\c_r, chp_de	  SB PCIsscfgckedndsta_agat danspecs_b_	} telcha
			pec)ands if neces>buses 	ms_c_seb_x_al{
		if (wO= 0; 	cms_c_in-ENOMEDIUM;red  -us_b_	} telchauppec)ands if neces>buses tx <  nd->;own,
)
ock\n"set *
brcms_
ms_resi_clkctl_faectionUSE_COREte >Smacstms_c_info rate, inde;

	/* entb~up_fnitshandunit;

	brcms_dbg_mac80211(rs_dfr s_lc_pi)fifmacintc;
	}r_lihw_ndstation ic }
}

/* switch (struct_c_a F& SY  (20us dynamndetwlc->nt filte		if
)
ock\n"s_c_setxbeset *
brcms_
	/* set up the specified band aDYNAMICv * *
brcms_crs		/*  }
}

wc));
	wlc_ms_c_info rats shou/
stat/ME tu(20ustpe[ *eruct f (dcetransmit/  D1n sit(strs_c_hsplw->unit,o
			 *e if it is set */
	if (wl
		 cetries));
R
asic = 0;
	u8 ofdm_basic on beTE +a		if (wlNg->BrcE_FA,o
_set_i__);
		return;
	}

	

	/* band-spenginpp M_EDCpp <shm.aifs < NUMF_/SDCpp++match engineify debugging, no
 * AC_TXLMT_ADDRnpp)> 1) 	);

	);
		 cetries[ac]es *wlc)
f);

s_c_sfaciAope for bassate;

	/* enterupandunit;

	brc2 sflags;
	u32 bcnint_u2%_EDCF_STn necessachd-sp/* write ucode ACK/CTS rate table */
	et our snapshot of macstc)
HW_bcn atceCb-dutso
_sJADDLFRMidx,ct !=:aS_SHORTSLOT_ON)set(whw_r **S c/* writ	brcms_c_flushquAN 	cms_c_in-ENOMEDIUM;r
		return;
	set(whw_c); 40Mhz, remb~hwiupaACK/CTSrt 	xpowc-set(whw_c)fifmacinthy_txnit: powc-set(w/ot iags & & BFL_FEurn hm.aifs(ai~
	/*_pip_ida " /CTS rerevng(wlspecCHIP-ID_lsp4313)	 * wh banpowc-set(w/ot ies *> cox1250
spe(wlifs(
		c-set(w/ot iags & & BFL_FEu_BT)	
ppppp2XOFF)mhfging, no
 *HF5
 *HF5_4313_GPIOCTRL> 1) 	*HF5_4313_GPIOCTRL>band->gmodeALL)eleclmandatopp2XOFF)mhfging, no
 *HF4
 *HF4_EXTPA_ENABLEwchsspe(wl*HF4_EXTPA_ENABLEwband->gmodeALL)ele}t_c_adate Ng->BSSIOFF) c_ratwpec_bwTATUS_O "idxt brcv_etherac voiw "idxthbdate hys_ema			 *adercw, (bonefhunt;c_bwct brcmd. Wwrit	ouldwyctlto  *ect
rcmsinefge mac up
	/* Ud\nc vo. Istt;c_bw,
	hy_chand, abme_(up,utheer
rcmstwlc-,Dbcma_ t;c_bwtimer:aif _set_h(0(engiNDIS)
_sJADDcwlc
rcmsatphy_on chanidx, int loop sync up perupE_MAS rate/* entb~up_prepa) _set_hs eic_rad0 ors-lspEtRADIOO= sinfoateset Fix urn;
	set(watphy_ct brcmd; * wh ntwTATUS_O=e/* entb~up_prepaACK/CTSrt 	xpppppATUS_O==n-ENOMEDIUM	= BRCMates!
	}
}

stahans(wlc(rn;
	set(watphy_ct brcmd, WLtRADIO_HWtsf_ABLEi, __funccnint_us;
	uiNssfk;
	*u16 chring slee16 chs b
	c 1 to match e_set(watphy_ct brcmd,
for atWLtRADIO_HWtsf_ABLEit);

	 ban 16 ch->timi*r>band-S_TYPE_
p_sION ||21		"aifs	16 ch->timi*r>band-S_TYPE_ADHOCs for as;
	ui			  CHSPEC_CHANNEL(chanspeec));
				up: rfct brcms->t
	}

	ax = b16 ch_ct brcm()ld the set	  shm ac params sit);

	wlc_phy_txt || wlc->bandatphy_ct brcmd; 40Mhz, removc_phy_mo reor_bcma_ewatit);_ms_c_info cy_tx < /* entb~up_prep;t, _ite is_c_set_neceresi_c).tso
    in o	brd->;ihastat
		ac    =fmacintsteturn fac_phy_mo reor_bcopuuct 1);

	/i->;)
		 	} tags & set *
brcms_mhfging, no
 *HF1
 *HF1_STAT
 *HF1_STAT
 and->gmodeALL)el* *
brcms_for 20Mh
	wlc_hm ac param)fifmacintst* BMg sleepinl%d: %s: chspec %dchbanhm ac para2%_E}
}

linf.n nedef.n ne;0Mhz, removwlc_hw->wlc);
	}
}

/*
 tandard 11b/g/radio chanspwpec)) 20mhz_mac_up(ch(whw_(100 )rt 	xpowc-epinl%d: %s: chs_ke_err(wlc!rd 11b/g (20us slots) if (wlt *
brcms_up_fnitshaACK/CTSrt ;

	/Prog tablnefTXewmefcf_txopw, (bonefnt
 *		 in_sel(spbrcms_c_set_
		 cetries));
R
auct 1);

	/bcma_ s */[baticaw *ssdog timer:brcms_c_seadd_timer| wlc-wdtimer, TIMER_INTERVALtWATCHDOGms(struct  wlc-WDarmedfifmacintst
	/en	/* bRAit(wlcanges tin m)f,o
_oid brcms_c_set_c& ~mashytxant;
	uwates ->maen	/* bLDPCcanges tin S_Cw (brbrcms_c_set__update_beacon(wpec) i < rs.coaconmacstms_c_info rate, indeu;

	/* enterd			>wel_timer| basic_rate;
	uint i;

	rs_dfu, "wcwlc_hw,e  core);_set_h(cwlc_hw,ecsbrce, inde;

	/* entb~bwlc)d			>prepandunit;

	brcms_dbg_mac80211(rs_df//
	rt	b_gs *;dfu, "wcwlc_hw,e  core);pec);
		wlc_phy_

	/* banwcwlc_hw,ecstat	b_gs *O=e/* entt	brcms_c_flushqu}
}

wc) brcms_ dt brcmss_c_setxband(wlpec)t	b_gs *)re */
}
}

 i < wlcs_cMS_PHY_0;hardwar, chp_dnow dt brcmss_c_setxband(wl *
brcms_crs	ff/*  }
}

wc));
	wlc chp_den	/* bwe'ddreunning  u8 bastcallcE_FA	gat danspec
brcms_
	/* set up the specified band and chans}cms_ dckedtes[at8 basoftw brcmsse Biage 	/* cwlc_hw,e +rC		bw tha
			pec)ands /* switces /
_set_h(cwlc_hw,ecsbrce, inde;

	/* entb~d			>fnitshandunit;

	brcms_dbg_mac80211(rs_dfu, "wcwlc_hw,e  coref//
	rt	b_gs *;d);pec);
		wlc_phy_

	/* banwcwlc_hw,ecstar s_lc_pi)fif_err(wlc wl_r_lihw_ndstation ic }
}

/* switch pppppp2XO?t	b_gs *O=e/* entt	brcms_c_flushqu}
}

wc) brcmpec)t	b_gs *)r, chansp
}

rb    =e_err(wlc!ansp
}

    =e_err(wlc!anspr_lihw_   _ndstation ic }
}

/* switch pppppp2XO?x <  nclaimbRAy_lcsin tthw,eband(wl *
brcmc_flush ++++sshqu}
}

wc) br!mute_tx);O?x < R>pub->unidt brcms-clock\n"et->ratesus_b_chaninsg (20usn ic }
}

HANNEL( >= BRCMatesms_b_mctrl(wr s_lc_pHANNEL(chans		| MCTL_HPS, v2nt filtthe  MMAC E m.aCs for z, removwlc_hw->wlc);
	}
}

/*
 }
}

wc) br!	 cwlc_hw,e +rCz, remresi_clkctl_

wc));
	wlc_r z, removnecect brcm  " %d\cmsh u16 c_anet_h( dutprimaFRMx_al_>unitcalet->rates;
		wlc_pnoresi_>= BRCMcs_b_	} telcha
			pec)ands if neces>buses 	mms_c_seb_x_al{
		if (wO= 0; 	c}phy_tx_set_h(cwlc_hw,ecsbrcmst(stMarkespectsc_sfaciAnonope for bas,Dbcopespec= wlc->bameio ciss if *idt brcms-cloms_dbg_m,lfre
sRAy_transPROT_t(strudndsta.shouR matchaad mac brcms_cnumber:brche mac cwlc_hw,e Bific %s: chseed f u;

	/* enterd			p basic_rate;
	uint i;

	rs_ddfu, "wcwlc_hw,e  coref, "wi; b//
	rt	b_gs * =e_err(wlsp/* write ucode ACK/CTS rate table */
	et our snapshot of macstc)
_prot,s_c_wr->baalmctryn  down,going dckedta (bSHORTSLOT_ON)goingrd			NNEL(chanspec));
		return;
	}

	/
peec));
				 banDe mac going dckedbrcm matc fill in shm ac params struct */
	acp_shm.txonfo cy_t_set_ps_ctrl(wlc);

	/* banwcwlc_hw,ecstar sN)goingrd			fifmacintstcwlc_hw,e +rCz, remb~bwlc)d			>prepaACK/CTSrt ;
t	b_gs *O=e/* entt	brcms_c_flushqumacstc)
CacalRAyeid(strucn t
			f nedlern 	/* engine(s) and poand-S_MAXandULES  t
		 * wh ban i < w		coecb[i].d			>fn)r!	 cwlc_hw,e +rhans(wlc i < w		coecb[i].d			>fnn i < w		coecb[i].hdl)ele}t_c_a ke ceal basw *ssdog timer:brcm ban i < WDarmed	 * wh ban!/* entt	l_timer| wlc-edtimerc)r!	 cwlc_hw,ecore	  wlc-WDarmedfif_err(wlc}_c_a ke cealacaloc_radtimern 	/* cwlc_hw,e +=	/* enterd			>wel_timer|hqumacsthm ac param)fif_err(wlspanspr_limutation ic 

/* switch ppppp,Rshm(MUTEeALL)el* cwlc_hw,e +rCz, remb~d			>fnitshaACK/CTSrt ;

	/z, remb~d			>fnitsh;t, _ite is_c_set_necect brcm().tso
    in odutstat
		ac    =f_err(wlspansN)goingrd			fif_err(wlc_set_h(cwlc_hw,ecsbrcms_S->;own,nt
 *		 
	cascanges ur macintma;

	/* enter/rad
	casu16 txfunfl[NFIFO];
#endicfg->
	cas  acpppanges on beTE +_se  core i}

si; bcnint_us;
	uic_c_}

	/ );
R
hw-Deytxcha,o
54gnAulocstathw-AdC(acisw_>uniuse pec(struct(-1/0/1nAulo/Off/O	NN0x3][8 pec(struct b == shortslot)
		retomb//
	rpec(struc
ratericct b_err(w  < R>pericctassoci macintbwTATUacis
for at != 0eo
BRtry irpec(struc
for at !/mb//
	rofdm_basict b_err(wc_anMeke 6, 12,d || 24 basictrstas if thw-AdC(acisw_>uniuse pec(stpreamhansp(-1/0/1nAulo/Off/O	NN0x3]}

spreamhan:
	 == shPLCPhLctiomb//
	rpreamhan
ratericct b_err(w	 < R>pericctassoci macintbwTATUacis
for at!= 0eo
BRtry irpec(stpreamhans
for at!/ bcnint_us;
	uiN_PCs_N_PC; a >macstN-BRtry irin  (20usn,x,
	/* G	cascpub->		 *ng >		hw, BCMed rateGTL_INd\nouldGand aLEGACY_B/*
			 * BM(
		ac paraing (20 & SUPPtsl_11Nvlifs
	casc==dGand aLEGACY_BN 	cms_c_in-ENOTSUPP0; i < vererio 0eo
_wr->bade baticd, (b2G hateratergr20 		  haterlc->hw, brcm ban i <  if not uptimi*r>band->gmode2Gmatchatering sleepin;hardwar* BM(
		ac parainit_pendinT && "	an i <  if g statOTHERgmodUNITewate]not uptimi*r>band->gmode2Gmmatchatering sleeping statOTHERgmodUNITewate]msclmandatms_c_in-EINVAL0; i < on chananges ur macin(100 -brcm bananges onr z, remov this function ic =
ection->gmodeUSER,s
	casmacstc)
Clear c_}

	/ 	default:	/* ams->tx&);,pnvert fromrsves RXsd, ch (
	casm * wc voiGand aLEGACY_B: whpec(struct b == shortslot)
	O= wlc!rd 11b/gc_}

	/* */
(&gol)eoegac)ec_}

, &);p2XO?xbreakel* cwvoiGand aLRS:O?xbreakel* cwvoiGand a	ret: wh u Accepd,
	ytxchsmd(wl *
eakel* cwvoiGand aONLY: whofdm_basict bd!\n",
	preamhan:
	 == shPLCPhortslcs
	preamhan
ratericct bd!\n",
	*
eakel* cwvoiGand aPERFORMANCE: whpec(struct b == shortslot)
	O s b
pec(struc
ratericct bd!\n",
	ofdm_basict bd!\n",
	preamhan:
	 == shPLCPhortslcs
	preamhan
ratericct bd!\n",
	*
eakel* 
	ytxch: wh u Eenormd(wl *
brcmc));
		return;
	}

	/* Switch baninvali _
	casc%d fill in shm ac params struct */
	,s
	casmac	cms_c_in-ENOTSUPP0;(wlt *if no
	casc=s
	casntst, trueec(struc
	default:
	eec(strucacstc)
Usel baszhytxchaangCc_}

	/ set Fix u);sd mac)lc!rd 11b/gc_}

	/* */
(&cck_ofdm_c_}

, &);p2XO?Fix ofdm_basic	 * whengine(s) and po);sd mac  t
		 * wh?Fix );sc_}

SM
 *>band->rcms_6Mhans(wlc|| );sc_}

SM
 *>band->rcms_12Mhans(wlc|| );sc_}

SM
 *>band->rcms_24Ms for );sc_}

SM
 |=
ectionrcms_te >; 	c}phy_txms_S->;zhytxcha_infms_c_ehastat
		acppppp2% Nssstn sincesd mac(s));sd mac * amsc/
(
		acppppp2% Nssstn sincesc_}

, );sc_}

,
pe(wlc);rt from
		acppppp2% Nssstn sincesc_}

ves RXE matchE mcsbrc;

	/* enter/radn	casu16 txfunfl[NFIFO];
#endirs_dfu, "wi; bc32 n	casc=s	retome_ ban i < rs.count; i++) *>bWL_11N_3x3s fon	casc=sWL_11N_3x3msclmandatn	casc=sWL_11N_2x2;a;up_d maciAGand a	retacstNand Nd\nON brcms_c_set_crad
	casuic =
Gand a	retms(struct  bann	casc==dWL_11N_3x3s fo
		ac paraing (20 = SUPPtsl_HTntardware */
st paraing (20 = SUPPtsl_11N;at
		acppppp2% Nssstags & |=
ectionoid_HTntup_d d) c_ramcnfms_c_s					  zhytxcha || hunt;sincespbrcms_c_set_c_}

	/*mcn_build(&
		acppppp2% Nssstn sincechans(wlc); i < rs.count; i++));a engine(s) and po wlc->bandinit_pe at
		hanamsc/
(
		aceping stati](whw_n sincesmcnc
spe(wlc);

	acppppp2% Nssstn sincesmcnc MCSSET_LENmacstms_c_info rate, inde;


s_c_set_crad->hw,nal_n sinceu16 txfunfl[NFIFO];
#endichans(wlc)cnint_us;
	uic_c_}

	/ *rs_argve been bandlocked dc_}

	/ );,cfg ;dfu, "we;
			ifacstamsc/
(&);,prs_argvert fromen bandlocked dc_}

	/)macstc)
_prot,engishm.d mac((100 -brcm ban();sd maccms_c_*S co);sd macc>
ectionNUMrcmsSAN 	cms_c_in-EINVAL0; i < LFRMiwn,nt
 *		 N_PCs_bast up yet
=c i <  if not upms s;s	amsc/
(&fg , &);vert fromen bandlocked dc_}

	/)mac Fix the ha dc_}
_hwrs_filhw,_sc(srvali cha "aifsM&fg , &
		aceping statt upms s](whw_n since.sd!\nb
pe(wlc i < rs.count; i++))N 	cgotbagood0; i < LFRMiwn,oc_radN_PCs_basFix the hais_mt upduncE_FlushquANNEL(ch up yet
=cOTHERgmodUNITewate;hanamsc/
(&fg , &);vert fromen bandlocked dc_}

	/)mac  Fix the ha dc_}
_hwrs_filhw,_sc(srvali cha(&fg ,he set	  slc);&*  ->he set	  slc);eping statt upms s](whe set	  slc);hw_n since.sd!\nb
p set	  slc); i < rs.count; i++))N 	ccgotbagood0;hy_tx_set_h(-EBADE;16agood:tup_d pplycfg fms_c_ehastatamsc/
(&
		acppppp2% Nssstn sincec &fg ,hee(wlc);rt fromen bandlocked dc_}

	/)mac amsc/
(&
		aceping statt upms s](wpppn sincec &fg ,hee(wlc);rt fromen bandlocked dc_}

	/)mac ms_c_info rate, indeet */
	if (wlofdm_c_}

radbg_ 16 txfunfl[NFIFO];
#endirs_dfu8 romb//
	rbg_ =f_err(wlspt || wlc->bandassoci mmd;

	/ring slee16 ch->nt
 *		 Nssstn sincesc_}

t0]
	 lmandatm*rr		bchppppp2% Nssstn sincesc_}

t0]
	spanspr_liofdm_c_}

radbg_ ic 

/* switch bg_)csbrc;

	/* enter/radn necesu16 txfunfl[NFIFO];
#endicfgc_validceson be_c_valwlc->cmi,20mhz_mac_up(alidcesocstarr(cc necess< 0*S cc necess> MAXCHANNELN 	cms_c_in-EINVAL0; i	retueturn favali dio chans_db| wlc-cmi,valwlc-AN 	cms_c_in-EINVAL0; _t_set_ps_ctrl(wlc)lifsthe hais_mt upduncE_FlushquANNEL(c ban i <  if not upr(wlc->hd = true;
			if (wwlc-AN 	cxpowc-epinl%d: %s: chs_ked!\n",
	lmandatopowc-epinl%d: %s: chs_ke_err(wlcy_txpowc-ppppp2% Nssstndunit(chansp_cha; b
	/z, remcnoids_for)*
	 * sa->ba= w basms_c_ehabefeceitch rate_sit.._SHORTSLOT_ON)set(wc)lifs(anspr_liio chans_gmatch e_ if nopif ->hd = tr); 40Mhz, remov/rad	}
}

void
brpwpec)) = tr);0Mhz, removwlc_hw->wlc);
	}
}

/*
 tandard 11b/g/radio chanspwpec)) = tr);0Mhz, remov (20us slots) if (wl ms_c_info rat;

	/* enter/radc_}
_TXCNTu16 txfunfl[NFIFO];
#endicfgc_vsrlcfgc_vlrlon beTE +a		if ppppprss< 1 ||vsrls> RETRYhortsl_MAX ||211_AClrss< 1 ||vlrls> RETRYhortsl_MAXN 	cms_c_in-EINVAL0; ich e_tR, =
srl;at
		acLR, =
lrl;at *
brcms_M_LFRTXCNTtion ic 

hm.s wlc-SRL>b
		acLR,)d-spenginpp M_EDCpp <shm.aifs < NUMF_/SDCpp++m) {
	w tru
		 cetries[ac] =	SFIELD(w tru
		 cetries[ac],
for atttttttSTAThortsl, s wlc-SRLct 	efore w		 cetries[ac] =	SFIELD(w tru
		 cetries[ac],
for atttttttSTAThLcti>b
		acLR,)d-(wl s_c_set_
		 cetries));
R
auct 1);
ms_c_info ratet */
	if (wl
	/*_t
 *		 n sinceu16 txfunfl[NFIFO];
#endichans	)cnint_us;
	_c_}

	/ *_t
 sve been bandlocked dc_}

	/ *rswlspt || wlc->bandassoci mmd;

	/s truw		ree16 ch->nt
 *		 Nssstn since
	 lmandatms truw		reppppp2% Nssstn sinceacstc)
C*/
	bnfo oegac)sms_c_ehass func 	/* ct
 s

limac(s));

limacac amsc/
(&ct
 s

c_}

, &);

c_}

, );

limac)csbrc;

	/* enter/radn sinceu16 txfunfl[NFIFO];
#endic)cnint_us;
	_c_}

	/ * sve been bandlocked dc_}

	/ ->hw,nal_nsref, "wbcmeenorwlspt || s

limac(>
ectionNUMrcmsSAc	cms_c_in-ENOBUFSacstams->tx&->hw,nal_ns,pnvert from->hw,nal_ns)macstc)
C*/
	bnfo oegac)sms_c_ehass func 	/* ->hw,nal_ns.limac(s));

limacac amsc/
(&->hw,nal_ns.c_}

, &);

c_}

, ->hw,nal_ns.limacmacstc)
meegasms_c_ehacomic vinpw, (bonefnt
 *		 mcn
	/ set Fix 
		ac paraing (20 & SUPPtsl_11Nvl {
	cnint_us;
	uiNssfdm_basmcn
	/ Nssmswh banpowc-set(wassoci mmd;

		mcn
	/ Nssring slee16 ch->nt
 *		 Nss",
	lmandatomcn
	/ Nssring sleppppp2% Nssmsanamsc/
(->hw,nal_ns.mcnc &mcn
	/ Nssstn sincesmcnt0]c
spe(wlc);MCSSET_LENmaced  -us_eenor =	/* enter/rad->hw,nal_n sinceundic)&->hw,nal_nsmac Fix !us_eenor)lc!rd 11b/gofdm_c_}

radbg_ ic  1);
ms_c_inbcmeenorwlrate, index;
	u8 basic_time_cE_F 16 txfunfl[NFIFO];
#endirs_dfcs_b_
ral(wr sreturn;
	}

	/*| MCTL_HPS, v2nt filtt, MMAC TBTTHOLDes ->maCommic_bandw
statset *s_b_mctrl(wr sreturn;
	}

	/*| MCTL_HPS, v2nt filtt)wlrate, index;
	u8 basic_time_uncE_F 16 txfunfl[NFIFO];
#endirs_dfcs_b_MS_Pl(wr sreturn;
	}

	/*| MCTL_HPS, v2nt filtt, ~MMAC TBTTHOLDes ->maCommic_bandw
statset *s_b_mctrl(wr sreturn;
	}

	/*| MCTL_HPS, v2nt filtt)wlrat;

	/* enter/rad  = wlc-annotu16 txfunfl[NFIFO];
#endicfgc_v-annoton be_32nbcniant;swlspt ||-annot* s_c_ 	cms_c_in-EINVAL0; ich e_ppppp2% Nssst  = wlc-annot* f-annot;at *cniant;s* f-annotREGOuoref/ basic_time_cE_F s) if (*s_b_w
stal(wr sreturn;
	}

	/*| MCTL_HPS,tsf_cfprep)c
spe(wlc(*cniant;s*EGOCFPREP_CBIhorIFT)if (*s_b_w
stal(wr sreturn;
	}

	/*| MCTL_HPS,tsf_cfpbcma_t, *cniant;s)ref/ basic_time_uncE_F uct 1);
ms_c_info ratgc_v
	if (wl
	/*ookitimiu16 txfunfl[NFIFO];
#endicf}

sphyidxrs_df_set_h(hm ac if nop	brcmss ratet */
	if (wl/radeec(struc
	defaultu16 txfunfl[NFIFO];
#endic)c8 struc
	defaultrs_dfr srueec(struc
	default:
	etruc
	defaultct_c_adate pec(structs_ofnaangCfeaet_	/*st	ou mk\n"wort,s_c_wr->bdate nt
 *		ly  u8 bas5G hate/*
			 * BM i <  if not uptimi*r>band->gmode)
		rs/* band-spTSLOT_ON)set(wc)lifspowc-set(wassoci mmd;r, chp_dl->;w *ssdog ngis = wldelat !=ic von chanhec(struct			 
}

static ps_ctrl(wlc);r, chp_dunassoci mmd pec(structs_oodutstatard 11b/g/d, chdeec(strucundic)pppppp2X_mute_tx);

s_ de mac ,
	h			brst	ju(raon chaniheunfl[NFIFO];

 at!=(100 -brcm_ ban i < rec(struc
	default:
 b == shortslot)
		retN 	cxpowc-pec(struct b_err(wlc!lmandatopowc-pec(struct hans(wlcn i < rec(struc
	default:
 hans(wlc) == shortslot)
	O v * d ratmst(stmd(strucww *ssdog >unid			f nedlerneed f ;

	/* enterw		coe_md(strucu16 txfunfl[NF pate pachans(wlc wlshan nr *name,c16 txfunfl[NFO];
#ehdlchans(wlc;

	(*d>fn)(et */* nedle)ve been bandlocked dO];
#endit bu16 txfunfl[NFIFO];
#e) rl(wlwb_upt, "wi; f (wlfb_refnaempty *		ry >uniju(raaddion
_duplic macin_prot! 	/* engine(s) and poand-S_MAXandULES  t
		 * wh ban i < w		coecb[i].namet0]:
 b'\0'	 * wh?16 nc/
(
		acw		coecb[i].nameionameihans	rt from
		acw		coecb[i].name) - 1es 	mm i < w		coecb[i].hdl(s)hdls 	mm i < w		coecb[i].d			>fn(s)d>fns 	mmhm.txonfo cc}phy_tx_set_h(-ENOSRcsbrcms_onmd(strucww		coewcwlc_hw,e  f ;

	/* enterw		coe_onmd(strucu16 txfunfl[NF pate pacc wlshan nr *name,hans(wlc);16 txfunfl[NFO];
#ehdlve been bandlocked dO];
#endit bu16 txfunfl[NFIFO];
#e) rl(wlwb_upt, "wi; f rr(chspl =b & Sc
	cms_c_in-ENODATA; a engine(s) and poand-S_MAXandULES  t
		 * wh ban!16 cmp(
		acw		coecb[i].nameionameT && "	awlcn i < w		coecb[i].hdl(ss)hdl >= BRCMams->tx& i < w		coecb[i],pnvert from i < w		coecb[i])mac  mhm.txonfo cc}phy_tx < Lbrcmsouldfimad! 	/* ms_c_in-ENODATA; ne. b+++++*/
	return fa_pip8 *sselchu16 txfuns + 
	brcm *cle)ve been bandpci 
	b *pci
	b = chans>bus;
	} telchas gc_vvendte rrpci
	b->vendte; cgc_v
	brcm rrpci
	b->
	brcm;pf rr(cvendte != PCI_VENDOR-ID_lROADCOM	= BRCprmc));"unkn			fvendte  */%04 (wlc_vendtemac	cms_c_in_err(wlcy_txpec)t	brcm r(wlsp43224_Dwlc-ID_VEN1 || t	brcm r(wlsp43224_CHIP-ID)+BRE matchmacintm ban(t	brcm r(wlsp43224_Dwlc-ID_*S cot	brcm r(wlsp43225_Dwlc2G-ID_)+BRE matchmacintm bant	brcm r(wlsp4313_Dwlc2G-ID || t	brcm r(wlsp4313_CHIP-ID)+BRE matchmacintm ban(t	brcm r(wlsp43236_Dwlc-ID_*S cot	brcm r(wlsp43236_Dwlc2G-ID_)+BRE matchmacint
Cprmc));"unkn			f
	brcm sd/%04 (wlc_
	brcmit);ms_c_in_err(wlne. b+++++*/
	return fa_pip8 *ssesocu16 txfuns + 
	brcm *cle)ve been bandus_b_cpipO];
#ecpipO];
#= &chans>bus;
cpipO];
cstarr(cc ipO];
s>idng(wlspecCHIP-ID_lsp4716)+BRE matchmacint
Cprmc));"unkn			fc ipasd/%04 (wlc_c ipO];
s>idit);ms_c_in_err(wlne.*/
	return fa_pip8 *ssu16 txfuns + 
	brcm *cle)ve beed, ch (chans>bus;
	} ttimim * wc voilspecHOSTTYPE_PCI:+BRE matcheturn fa_pip8 *sselchucle)v; wc voilspecHOSTTYPE_SOC:+BRE matcheturn fa_pip8 *ssesocucle)v; w
	ytxch: whprmc));"unkn			f	} t[timi: %i(wlc_chans>bus;
	} ttimimac	cms_c_in_err(wlcy_ratgc_v
	if (bdc_}
_shm_r *nceu16 txfunfl[NFms_dbg_mac80211(cfg->c_}
on be_c_vLbrcm_pte; cg8lookiroid,sng */
		blc_gMS_TXPWol);t*wlc, u1}

stcnne chs_atic conPLCP SIGNALlfbelCs_basFix isgofdm_c_}
(c_}
o)+BRLbrcm_pte#= M_RT_DIRMAP_A
	 lmandatLbrcm_pte#= M_RT_DIRMAP_B;a;up_d ma a g man roid,s conLS-nibrcmsbrcms_cPLCP SIGNALlfbelCsis
rcmsinefng */c;

ow basms_cvLbrcm.ateset ookiroid(s))_}
_O];
[)_}
] & Brate rcms_c_bsupt,  */c=lookiroid & 0xfct_c_a Fb_read_sSHMolc->hw, (di basms_cvLbrcm *		ry by lookic vinpthbdate Direct-map Tbrcmateset ms_c_in2ate/* entb~OFF))bugging11(cfLbrcm_pte#+x i  */c* 2)ng this shouns c_fidl
	nre te:shouGenre tata[ *e ID  ma a lspCtthw,eb. sT_raa[ glfbelCsissouldc, dt(strma pCta[ *esrst	in m, d asfcf_c brcms_csw, Bon bnumber.e if it is sinlinefgc_
ns c_fidl
	nre teu16 txfunfl[NFIFO];
#endic)cnint_us;
	uiNssfk;
	*u16 chc
spe(cnint_u;
	txh *txhon be_c_va[ *eiC; a e[ *eiC =
le16_to_cpu(txh->TxF[ *eID_*& ~(TXFID_SEQ_c_bs |
p set	  TXFID_QUEUs_c_bs);a e[ *eiC |= "aifsMM(
		achee(wlc); c_fidllimacer
		 << TXFID_SEQ_orIFT)*& TXFID_SEQ_c_bs) |211_ACTX_lspC_t re1);
ms_c_ina[ *eiC; rate, indeu;


s_c_set_cwlc_hw,_timeu16 txfunfl[NFIFO];
#endicfg32 rt*wlc
spe(wlc)g8loreamhan
timims_dfu, "wdur  core);_adate St(ch9.6: hw,sms_cvisdown,ate atenms_cvin_oidBasicRs_cS->;owat ratees
lescn 0en ngiw, al_tba basms_cvbrcms_cimmedi mmlylorevious
rcmsa[ *e   down,FESateset md
brcmsb;
	uiN_sic_c_}
(ndicfr= tr);0M u ACKsa[ *e lan F=in4 F=i2(fc)#+x2(dur)#+x6(c_)#+x4(fcsNN0x3]dur   "aifs	_c_set_cwlc_a[ *e_timeundicfr= tr,loreamhan
timiihans	(DOT11_ACK_LEN#+xFCS_LENmit);ms_c_indur; rate, indeu;


s_c_set_cwlc_cts_timeu16 txfunfl[NFIFO];
#endicfg32 rt*wlc
spe(wlc)g8loreamhan
timims_dfE matcheturn fa_wlc_hw,_timeundicfr= tr,loreamhan
timi); rate, indeu;


s_c_set_cwlc_ba_timeu16 txfunfl[NFIFO];
#endicfg32 rt*wlc
spe(wlcg8loreamhan
timims_df_adate St(ch9.6: hw,sms_cvisdown,ate atenms_cvin_oidBasicRs_cS->;owat ratees
lescn 0en ngiw, al_tba basms_cvbrcms_cimmedi mmlylorevious
rcmsa[ *e   down,FESateset md
brcmsb;
	uiN_sic_c_}
(ndicfr= tr);0M u BA lan F=i32 F=in6(ctl)hdr)#+x4(ba lan)#+x8(bitmap)#+x4(fcsNN0x3]E matcheturn fa_wlc_a[ *e_timeundicfr= tr,loreamhan
timiihans	ifsMDOT11_BA_LEN#+xDOT11_BA_BITMAP_LEN#+hans	ifsxFCS_LENmit)brcms_s_c_set_nempu w_a[ *e_dur()ed shouCwlculchaniheu802.11 MAC hFF)w, DURlfbelCsrma pPDUshouDURlfma a acogMS_a[ *e =in SIFS#+x1 ACKshouDURlfma a a[ *e w, (bfo
	/*te_smss & FI3 SIFS#+x2 ACKs+ nextaa[ gltimeed shouc_}
) 	*PDUnms_cvin_usel_wlc500kbpsshounext_a[ g_lan	nexta*PDUnlangth:t dbytesshouoreamhan
timi	use pec(s/GF ngi *ng/MMcPLCP hFF)w,e if it is sgc_
n_c_set_nempu w_a[ *e_dur(16 txfunfl[NFIFO];
#endicfg32 roid,
spe(wlc)g8loreamhan
timi,eu;

	next_a[ g_lanon be_c_vdurvertfswlsprtfsc=s
radetfsM i <  if rt ;
tur  crtfswl
tur += (_c_)heturn fa_wlc_hw,_timeundicfroid,soreamhan
timi); t  bannext_a[ g_lanox);

s_ Dourcms-cloct
 *		 DURltbag->;2 SIFS#+x2 ACKsmd(wl tur *=i2;;

s_  d) anoc_radSIFS# || wlc-a[ gltimemd(wl tur + crtfswl
 tur +  "	awlcn_c_)heturn fa_wlc_a[ *e_timeundicfroid,soreamhan
timib
p set	 next_a[ g_lanof (wl ms_c_indur; rat/msT_raopposi_cvbrceturn fa_wlc_a[ *e_time if it is sg;


s_c_set_cwlc_a[ *e_lan(16 txfunfl[NFIFO];
#endicfg32 roidt*wlc
spe(wg8loreamhan
timi,eu;

	dur)s_dfu, "wnsyms,. MA_lan, Ndps,.kNdps;dfu, "wroid(s))t*wl2c_}
(c_}
= tr);0asFix isgmcn_c_}
(c_}
= tr)ox);

u, "wmcnf= roidt*wl*& RSPEC rcms_c_bsuptt, "wtoade; i++) *wmcn_2_unt; i++)(mcsNN+))t*wlde;c(c_}
= tr);0
 tur -= PREN_PREAMBLE#+x toade; i++) * PREN_PREAMBLE_EXT);;

s_ pay *adwcwlculchacin8 *ssecn 0eo
,sttegulcrrofdm-brcm_ ban i <  if not uptimi*r>band->gmode2Gmatc tur -= DOT11_OFDM_SIGNAL_EXTENSIO s b
s_ kNdbps *wkbps * 4-brcm_kNdps =	mcn_2_c_}
(mcnc )t*wldis40mhz(c_}
= tr)ihans	ifs)t*wldissgi(c_}
= tr)ox* 4s b
nsyms(s)dur /	Ashm(SYMBOL_TIMEmsanaMA_lan Fbsl1_AC((nsyms(*.kNdps) -
spe(wlc((Ashm(SERVICE_NBITS#+xAshm(TAIL_NBITSox* 1000)ox/u8000;{

}

static isgofdm_c_}
(c_}
= tr)ox);

tur -= Ashm(PREAMBLE_TIMEmsantur -= Ashm(SIGNAL_TIMEmsan(wlNdbps *wMbps * 4-= roid(500Kbps) *n2atrcm_Ndps =wroid(*i2;;

nsyms(s)dur /	Ashm(SYMBOL_TIMEmsanaMA_lan Fbsl1_AC((nsyms(*.Ndps) -
spe(wlc(Ashm(SERVICE_NBITS#+xAshm(TAIL_NBITSoox/u82X_mute_tx);

t ||-reamhan
timi & Brate ortsl_PREAMBLEmatc tur -= Bshm(PLCPhortsl_TIMEmsanlmandatotur -= Bshm(PLCPhTIMEmsanaMA_lan F)dur ouc_}
msan(wldivult:o	  factor:brc2cble oid((1/2 mbps) */sanaMA_lan F)aMA_lan /u8 /u2f (wl ms_c_inaMA_lang this shouR matchmaciaticms_cs*wlc,  d ms_cvisdBRtry in tbycms_cs*wlc,  d t up.shouand->gmodeAretacndic mesMiwn,nt
 *		 N_PC.e if it is s*/
	return favali de teu16 txfunfl[NFIFO];
#endic)g32 rt*wlc , "we;
	,
spe(wl*/
	rverboseve been bandlocked dc_}

	/ *hw_n since;dfu, "wi; tm ban(hateri=uand->gmodeAret_*S cohateri=u i <  if not uptimio)+BRhw_n since truw		reeif nohw_n since;df

static ps_ctrl(wlinit_pendinT+BRhw_n since truw		reeif g statOTHERgmodUNITewate]nohw_n since;df

stsan(wloc_radN_PCss*wlc,  d _PCs_wr->baa acogMS_N_PCs
	brcm */c	cms_c_in_err(wlstc)
_prot,s_cmsse s_of d */wroid(*/asFix isgmcn_c_}
(c= tr)ox);

 ban();*wl*& RSPEC rcms_c_bs)*> cMCS_TABLE(SIZEN 	ccgotbaeenorwlspcms_c_in

sta(hw_n since->mcnc ();*wl*& RSPEC rcms_c_bs)maced  -engine(s) and pohw_n since->d mac  t
		;

 banhw_n since->c_}

SM
 *>b)t*wl2c_}
(cwlc-AN 	cxE matchmacintaeenor:f rr(cverbosevel *
brcmc));
		return;
	}

	/* Switchvali de te:a}

stC*wl*	if  cwmax = ouldblehw_n sinceet our snapshot of cfr= tr);0);ms_c_in_err(wlne. b+++++g32
aMAifs < ing1/radne teu16 txfunfl[NFIFO];
#endic)cnint_us;
	uiN_PCs_nt
_e;
	,
spe(wle(wg32cblsrvalve been bandus_b_
	brcm *cle)fifosz_fixrn;
	}

	; cg8lstft bublsrval*& Nrcms_STF_c_bs)*>> Nrcms_STF_orIFT; cg8lroid(s)blsrval*& Nrcms_rcms_c_bsuptg32 rt*wlomb//
	rismcnf= (ublsrval*& Nrcms_MCS_INUSEvng(wNrcms_MCS_INUSEvomb//
	rissgif= (ublsrval*& Nrcms_SGI_c_bs)*>> Nrcms_SGIhorIFT)omb//
	r	defaultgmcn_bnfo = (ublsrval*& Nrcms_OVERRIDs_MCS_ONLY)hans	ifg(wNrcms_OVERRIDs_MCS_ONLY)ref, "wbcmeenor  core);pec);ismcn)+BRE match(g32)uc_}
ms i < vali chas-clockmbin maci*,stt;te/mcn/stfts_of
	/*eCs_basFix (
		ac paraing (20 & SUPPtsl_11Nvlifsismcn)x);

s_ mcnfbnfo f
	/*eCsfisOFn	cascbrcm_ banstft>Rshm(TXC1_and aSDM	= BRCM*
brcmc));}

	/* Switch banInvali _stfld the se shm ac params struct */
	acp_s-us_eenor =	-EINVAL0;	ccgotbads *;df u16 c_anmcnf32cb_of s*wlcal*c vo, DUP 	casc40fbnfo brcm_ banroid(s=i32	= BRCMates!CHSPEC IS40;
		ret}
}

void
br) ||21		1_AC((stft!=Rshm(TXC1_and aSISO) 1) {
	*lifs(stft!=Rshm(TXC1_and aCDD)i, __func*
brcmc));}

	/* Switch banInvali _mcnf32ld the set shm ac params struct */
	acp_s--us_eenor =	-EINVAL0;	cccgotbads *;df 
	wlcc_anmcnf> 7 mu(raose ptftSDM brcm_
}

static roid(> HIGHESTaSINGLE(STREAM_MCSf (wlc_hw-mcnf> 7 mu(raose ptftSDM brcm__ banstft!=Rshm(TXC1_and aSDM	= BRCMp/* write uaMAifs <;}

	/* Switch (20ute_s
	}

	ax = "SDM 	cascengimcnf%d fill in	t	  shm ac params scfroidacp_s--stft bshm(TXC1_and aSDM;df 
	wlcmute_tx);

	_ada at!=MCS 0-7A
			ose SISO, CDD,w		if mada at!=ookirs *> c3 STBC
r at!/ b

 ban(stft>Rshm(TXC1_and aSTBC) ||21		1_AC(!Brate oTBC_CAP_shmewate 1) {
	*lifs(stft= bshm(TXC1_and aSTBC)i, __func*
brcmc));}

	/* Switch banInvali _STBCld the set shm ac params struct */
	acp_s--us_eenor =	-EINVAL0;	cccgotbads *;df 
	wlc}{

}

static isgofdm_c_}
(c_}
)ox);

 ban(stft!=Rshm(TXC1_and aCDD)lifs(stft!=Rshm(TXC1_and aSISO)	= BRCM*
brcmc));}

	/* Switch banInvali _OFDMld the se shm ac params struct */
	acp_s-us_eenor =	-EINVAL0;	ccgotbads *;df u1

}

static isgcck_c_}
(c_}
)ox);

 ban(nt
_e;
	not uptimi*!>band->gmode2Gmatc
	*lS costft!=Rshm(TXC1_and aSISO)	= BRCM*
brcmc));}

	/* Switch banInvali _CCKld the se shm ac params struct */
	acp_s-us_eenor =	-EINVAL0;	ccgotbads *;df u1

}

sta{
CM*
brcmc));}

	/* Switch banUnkn			fms_cvLimi fill in shm ac params struct */
	acp_sus_eenor =	-EINVAL0;	cgotbads *;df}_c_a f);

	/* bmultipMS_RAit(wlwr->baapec);

	penginon-sisotrstas if t ban(stft!=Rshm(TXC1_and aSISO)lifs(
		c-rs.count; i++) *>b1); 40Mhz, remc));}

	/* Switch banSISObRAit(wlcb	  !SISObcwmax = hw, BCMet our snapshot of cfuct */
	acp_sus_eenor =	-EINVAL0;	cgotbads *;df}_t md
brcmsc_}
msatic ismcn)x);

md
brc|= RSPEC MIMOrcmsmsan(wlForsSTBC populchaniheuSTClfbelCsbrcms_croidt*wl*brcm_ banstft= bshm(TXC1_and aSTBC)= BRCMg8lstc0;	ccstc =inw	 < Nst f (dacogMS_t; i++ts_of
ways 1t!/ b

md
brc|= (stc << RSPEC STChorIFT)ombc}phy_tx_d
brc|= (stf << RSPEC STFhorIFT)omO?Fix odefaultgmcn_bnfo);

md
brc|= RSPEC OVERRIDs_MCS_ONLY;0asFix issgi);

md
brc|= RSPEC ortsl_GI; tm ban(ms_cv!s_c_ 	
	*lifs!eturn favali de teundicfr= tr,lnt
_e;
	not uptimims(stru)+BRE matchc_}
ms iE matchct*wlomds *:
RE matchc_}
msthis shouCompu woPLCP,cb	  bnfo hw, irateact al_ms_cv		iflangth:brcpkt.shouRs_cvisdg man   down,he mac st		iardbmultipMS_wlc500wkbps.shoulcvisdBet f (d11 Mbps ms_cvif net !=ary.shouarokan o	  f (dPRQ.e if te, index;
	u8 basic_cck_plcp_nceu16 txfunfl[NFIFO];
#endic), "wroid_500,hans(wlc)u, "wlangthcfg->*plcpon be_c_vosec  core i8 an:
	0s RXsd, ch (roid_500m * wc voilnd->rcms_1M: whosec  clangth:<< 3cp_su
eakelwc voilnd->rcms_2M: whosec  clangth:<< 2cp_su
eakelwc voilnd->rcms_5M5: whosec  c(langth:<< 4ox/u11;;

 ban(langth:<< 4ox-h(gsec *u11)*>_c_ 	choseccore	 u
eakelwc voilnd->rcms_11M: whosec  c(langth:<< 3ox/u11;;

 ban(langth:<< 3ox-h(gsec *u11)*>_c_= BRCMgseccore	 
 ban(gsec *u11)*-c(langth:<< 3ox> c8)hans	an:
	D11B(PLCPhoIGNAL_LEombc}ph	*
eakel* 
	ytxch: whhanspec));
		return;
	}

	/
peec))8 basic_cck_plcp_nce: unBRtry in tms_cv%d fill in sroid_500mac	cmoid_500 >band->rcms_1M;df osec  clangth:<< 3cp_su
eakelw}_c_a PLCP sigbassbyteeset olcpt0]:
sroid_500 *u5w	 < r (500kbps) *n5 *>b) (100kbps) */_c_a PLCP serbrcm byteeset olcpt1]:
s(u8)c(la |	D11B(PLCPhoIGNAL_LOCKEDes ->maPLCP langth:_c_, littcm *	dianeset olcpt2]:
sosec & 0xff;t olcpt3]:
s(usec >> 8) & 0xff;t >maPLCP CRCc_vset olcpt4]  core olcpt5]  corerat/msR te:a802.11 ms_cvccas  langth: PSDUnlangth:t doctcespbrce, index;
	u8 basic_cempu w_d */
olcp(g32 rt*wlc u, "wlangthcfg->*plcpon be_8wmcnf= (u8)c();*wl*& RSPEC rcms_c_bs);t olcpt0]:
smcn;spt || s*wldis40mhz(cd
br) || (mcnf==i32	)hanolcpt0]:|= MIMO(PLCPh40MHZ;spBrate oET_MIMO(PLCPhLEN(olcp  langth);t olcpt3]:
s s*wldd */olcp3(cd
br)w  < );*wl*almctrynholdsgosel_Nyteeset olcpt3]:|= 0x7w  < _ehasmoooseng,souldsimadte_sprocl&  nd-rveCs_basolcpt4]  cor  < number:brcexit(saci*spaband t; i++) bit 0l& 1s_basolcpt5]  corerat/msR te:a802.11 ms_cvccas  langth: PSDUnlangth:t doctcespbrce, index;
	
n_c_set_nempu w_ofdm_olcp(g32 rt*wlc u32 langthcfg->*plcpon be_8wc_}
_sigbasuptg32 tmp  coref, "wroid(s))t*wl2c_}
(c= tr);0);_adate cnne asms_cvper:802.11a-1999ass  17.3.4.1, w, (blsb
rcmsiransmitin tfirsb>protecc_}
_sigbas(s))_}
_O];
[)_}
] & Brate rcms_c_bsuptams->txolcp  0/*| M_shm_HDR_LENmace| MA_shm_HDR_Srcms(u16 txfuofdm_o_lihdr#e) rlcp  c_}
_sigbas);0);tmp  c(langth:& 0xfff	 << 5;t olcpt2]:|= (tmp >> c_)h& 0xff;t olcpt1]:|= (tmp >> 8)h& 0xff;t olcpt0]:|= tmp & 0xff;trat/msR te:a802.11 ms_cvccas  langth: PSDUnlangth:t doctcespbrce, index;
	u8 basic_cempu w_cck_plcpu16 txfunfl[NFIFO];
#endicfg32 rt*wlc
spin u, "wlangthcfg->*plcpon be, "wroid(s))t*wl2c_}
(c= tr);0);8 basic_cck_plcp_nceundicfroid,slangthcfplcpowlrate, index;
	
n_c_set_nempu w_plcpu16 txfunfl[NFIFO];
#endicfg32 rt*wlc
sp(wlc)u, "wlangthcfg->*plcpon beFix isgmcn_c_}
(c= tr)o
tard 11b/gcempu w_d */
olcp(rt*wlc langthcfplcpowl	

static isgofdm_c_}
(c= tr)o
tard 11b/gcempu w_ofdm_olcp(rt*wlc langthcfplcpowl	

st
tard 11b/gcempu w_cck_plcpundicfr= tr,llangthcfplcpowlratms_s_c_set_nempu w_rtscts_dur()ed shouCwlculchaniheu802.11 MAC hFF)w, DURlfbelCsrma aneRTS ngiCTS a[ *eshouDURlfma normas(RTS/CTS w/_a[ *e =i3 SIFS#+x1 CTS#+xnextaa[ mbatime#+x1 ACKshouDURlfma CTS-TO-SELF w/_a[ *e    =;2 SIFS#########+xnextaa[ mbatime#+x1 ACKshofo *rtspinrts-to-self ngirts/rtsshouctn_c_}
	cmespngirtsnms_cvin_usel_wlc500kbpsshouc_}
) 	nexta*PDUnms_cvin_usel_wlc500kbpsshoua[ *e_lan 	nexta*PDUna[ *e langth:t dbytessho/
gc_
n_c_set_nempu w_rtscts_dur(16 txfunfl[NFIFO];
#endicfacpppatn_bnfo,hans(wlg32 rtn_c_}
,hans(wlg32 a[ *e_roid,s_8wctn_oreamhan
timib
p se(wg8la[ *e_oreamhan
timi,eu;

	a[ *e_lancfacpppbaon be_c_vdurvertfswlsprtfsc=s
radetfsM i <  if rt ;
ates!atn_bnfo)x);

s_ RTS/CTS d(wl tur =i3 *crtfswl
 tur +  "	awlcn_c_)heturn fa_wlc_cts_timeundicfrtn_c_}
,hansspe(wle(wctn_oreamhan
timip2X_mute_tx);

s_ CTS-TO-SELF d(wl tur =i2 *crtfswl
}

 tur +  "awlcn_c_)heturn fa_wlc_a[ *e_timeundicfa[ *e_roid,sa[ *e_oreamhan
timi,hansspea[ *e_lanmac Fix ta)l
 tur +  "	awlcn_c_)heturn fa_wlc_ba_timeundicfa[ *e_roid,hansspe(wle(Brate ortsl_PREAMBLEmwl	

st
tatur +  "	awlcn_c_)heturn fa_wlc_hw,_timeundicfa[ *e_roid,hansspe(wle(sa[ *e_oreamhan
timiit);ms_c_indur; rate, indeuc_v
	if (wlp	brxctl1a_wlcu16 txfunfl[NFIFO];
#endicfg32 rt*wlon be_c_vp	bctl1  core ic_v
wt ;
atesBrate ISLCNshmewat<  if r; 40Mhzwt bshm(TXC1_BW_20MHZ;sp
}

sta{
CM*w:
s s*wld
radbw(c= tr);0

s_ 10MhzsissouldBRtry in ty	/ set  Fix tw <bshm(TXC1_BW_20MHZ	= BRCM*
brcmc));
		return;
	}

	/* p	brxctl1a_wlc:v
wv%dsiss"he se s"ouldBRtry in ty	/brd->;to 20Let oubwacp_s-uwt bshm(TXC1_BW_20MHZ;spc_phy_txt ||isgmcn_c_}
(c= tr)ox);

u, "wmcnf= rt*wl*& RSPEC rcms_c_bsup0

s_ bwc)cnfc_chdte_-timi*isfcf_c brc s*wldp	brxbyte2 _set_hs set  p	bctl1  c s*wldp	brxbyte2(c= tr);0

s_ d->;own,RtrradNy_cvbrcp	bctl1 set  p	bctl1 |= (mcn_Lbrcm[mcn].txpr_liitl3 << 8p2X_mute_txtic isgcck_c_}
(cd
br) ifs!Brate ISLCNshmewat<  if r "	awlifs!Brate ISSSLPNshmewat<  if r; 40Mh_ada ateIn_CCK 	cascLPshmr	def *ads_OFDM M		co maci*bitopw, (bCCKda ateDatasR te. Event ally MIMOshmrwoulCsalso banangC(ac->BSS
 at!= 0it f (mat rat!/ b
s_ 0 >b1Mbps; 1  c2Mbps; 2  c5.5Mbps; 3 >b11Mbps *et  p	bctl1  c tw ||| s*wldcnf(cd
br) <<bshm(TXC1_and aSrIFT)if (
}

sta{chp_dl-gac)sOFDM/CCK *et  sc_vp	bcchs b
lc_gMS_TXPWol)ctl)Ny_cvrs_c_ms_cvp	bcchvLbrcm *et  p	bcchrinthe ha dc_}
_oegac)eol)ctl()t*wl2c_}
(cwlc-AN;;

 banp	bcchri= -1	= BRCM*
brcmc));
		return;
	}

	/* p	brxctl1a_wlc:vwroe_s
	}

	 s"l-gac)sOFDM/CCK c_}
et acp_s-p	bcchrinfo cc}ph
s_ d->;own,RtrradNy_cvbrcp	bctl1 set  p	bctl1   "	awlcntw |||p	bcchr<< 8p |
p e(wlc( s*wldcnf(cd
br) <<bshm(TXC1_and aSrIFT)if (
);ms_c_inp	bctl1msthis shouAdd(cnint_u;
	txhc)cnint_ucck_p_lihdr.shofo *'p'
_oia mu(rabcma_ w, (b802.11 MAC hFF)w,fo *'p'
mu(ra,
	/* enoughdbytesvbrclo_wl hFF)w,_s			ban"pushed" o

ow basthw,ebshofo *hFF)ro_c_==*| M_shm_HDR_LEN#+xD M_TXH_LEN#(D M_TXH_LEN#issouw 104dbytes)ed shof it is sgc_
n_c_set_;
	hdrsuaMAifs <;16 txfunfl[NFIFO];
#endic)cnint_u2%_EDCF_SThw *hwc
sp(wlc)cnint_usk_t(st>*pc)cnint_uscb *scb,eu;

	a[ gc
sp(wlc)u, "wnmss &,eu;

	 ++++,eu;

	next_a[ g_lanon becnint_u2%_EDCF_SThdr#eh;becnint_u;
	txh *txhre i8 *rlcp  plcp_fwlc_hw,[| M_shm_HDR_LEN]ref, "wlancfp_llancfctn_o_llanre ic_vmch,np	bctl, xfts,. Mine tesre ic_vseqrinf,. clrinf,.TATUS_O=e0,na[ *eiC  core i32 rt*wlt2]:
s{band->rcms_1M,band->rcms_1M }re i32 rtn_ct*wlt2]:
s{band->rcms_1M,band->rcms_1M }re acpppusw_rtst b_err(wlcacpppusw_ctst b_err(wlcacpppusw_rtfsc=s_err(wlcacppppec(s_oreamhant2]:
s{bppppp,Rppppp }re i8loreamhan
timit2]:
s{band-ShLcti_PREAMBLE,band-ShLcti_PREAMBLE }re i8lctn_oreamhan
timit2]:
s{band-ShLcti_PREAMBLE,band-ShLcti_PREAMBLE }re i8l*ctn_olcp  ctn_olcp_fwlc_hw,[| M_shm_HDR_LEN]refcnint_u2%_EDCF_STrtst*rtst b & Swlcacpppqos;dfu, "walomb//
	rhwtkmict b_err(we ic_vm */
ctlchuwt bshm(TXC1_BW_20MHZ;s#definefANTCFG_NONE 0xFFe i8lantcchrinANTCFG_NONEre i8lfbantcchrinANTCFG_NONEre i}

sphyctl1astft bore ic_vduriC  core cnint_u2%_EDCF_STtxproid(*txroidt2]ref, "wkre cnint_u2%_EDCF_STtxpO];
#etxpO];
;mb//
	ris_mcn;spic_vm */
rxbwre i8ld */
oreamhan
timims i < lo_wteu802.11 MAC hFF)w, set ht bu16 txfu2%_EDCF_SThdr#e)(prn;oiaif (qos(s)b%_EDCF_STis_;oia_qos(h->a[ *e_nt filttwlstc)
_ompu wolangth:brca[ *e   dbytesvfma ose   dPLCP _ompu TUacis set lan F)prnlanre o_llan  clan#+xFCS_LENwlstc)
G->;oxpO];
#eet oxpO];
#=shm.aifs < SKB_CB(ptwlstc)
 d) PLCP _basolcp  crkb_push(p/*| M_shm_HDR_LENmacstc)
 d) Br*ad_om;ox descriptor:hFF)w, set txh  bu16 txfu;
	txh *)crkb_push(p/*| M_TXH_LEN)uptams->txtxhc)0/*| M_TXH_LEN)up

s_ d->upna[ *eiC if t banoxpO];
stags & &shm.aifs < TX_MAC ASoIGN_SEQ)x);

s_ non-AP STAppeculCsnemac uvoilspC	 ++++ set  Fix  ++++ ==*TX_lspC_t re	= BRCM*
brcmc));
		return;
	}

	/	}

	 s"Switch banASoERT  ++++ ==*TX_lspC!ld the se shm ac params struct */
	acp_s-a[ *eiC  cns c_fidl
	nre teundic) & S, txhoo cc}ute_tx);

	_aeIncrem*		 -clockmacervfma firsb	a[ gm*		 !/ b

 banoxpO];
stags & &shm.aifs < TX_MAC FIRST_FRAGMENT)hans	scb->seqnum[prnpriority]core;

	_aeextracb	a[ gm*		 number:rs_c_a[ *e firsb	!/ b

seqrinle16_to_cpu(h->seq
ctrl)h& FRAGNUMFc_bsuptt
seqr|= (scb->seqnum[prnpriority] <<bSEQNUMForIFT)ombc	h->seq
ctrl = cpu_to_le16(seqp2XO?x-a[ *eiC  c((seq << TXFID_SEQ_orIFT)*& TXFID_SEQ_c_bs) |21"	awlcn ++++ & TXFID_QUEUs_c_bs);a c_phy_ e[ *eiC |=  ++++ & TXFID_QUEUs_c_bsup

s_ d->cms_cignpmq bit rma aic %ktsttx'd   dPS 	casc		if mais = wls if t banb%_EDCF_STis_s = wl(h->a[ *e_nt filtt	hanaclr|=*TXC_IGNOREPMQ;0);txroidt0]:
soxpO];
stnt filt.e tesre txroidt1]:
soxroidt0]:+ 1;0);_adate if ms_cvcc filt aigorithm didn'tdg ma uv a awlc_hw,date roid,s_sel basprimaFRMroid/*
			 * BMtxroidt1]s>idxs< 0)+BRLxroidt1]:
soxroidt0]; a enginPHY_0; k pohw< wlxproids; k
		 * wh s_mcn:
soxroidtk]stags & &shm.aifs < TX_RC_MCS ?hmacia:b_err(wlc!pec);is_mcn)x);


 ban(oxroidtk]stidxs>s_c_ 	c	
	*lifs(oxroidtk]stidxs<hans	l_

wio_lreeif g[oxpO];
steif ]stn_bitroidsi, __funcct*wltk]t hans	
	*ll_

wio_lreeif g[oxpO];
steif ]sthans	
	*lbitroids[oxroidtk]stidx].hw_(100 ;hans	sec(s_oreamhantk]t hans	
	*loxroidtk]sthans	
	*lags & &shm.aifs < TX_RC_USE ortsl_PREAMBLE ?hans	
	*loacia:b_err(wlc!c}ute_tx);

	cct*wltk]t band->rcms_1M;df 
	wlcmute_tx);

	ct*wltk]t baMAifs < ing1/radne teuwpec) i < e;
	,
sppppNrcms_MCS_INUSE ||oxroidtk]stidx);a c_p0Mh_ada ateCt
 *		ly  uly BRtry irp *e in_sel(f maiprimay >unda ateawlc_hw,croidt. Unerioags &  mai = hnms_cvin
owada ateacogMS_(100 -atic cona[ *esrat!/ b
usw_rtst|  "	awlcoxroidtk]sthan
	*lags & &shm.aifs < TX_RC_USE RTS_CTS ?hmacia:b_err(wlc!usw_ctst|  "	awlcoxroidtk]sthan
	*lags & &shm.aifs < TX_RC_USE CTSn->gmECT ?hmacia:b_err(wlp0Mh_ada ate(1) rcms:da ate  decerminef		ifvali chasprimaFRMroid/* ate  		if wlc_hw,croidtsrat!/ b
Fix u);*wldacb ma(ct*wltk]i, __funct*wltk]t band->rcms_1M;df mute_tx);

	pec);is_multicast_ec_ra_ d)r(h-> d)r1i, __funcs_ d->cmxbRAit(wlcanges 	!/ b

ard 11b/gRAisel_antcch_gmatch e_asch ppppp,
sppppppppp,R0,R0,R&antcch,R&fbantcch);df 
	wlcmphy_txphyctl1astft b
		c-rs.coss_op	casntstFix 
		ac paraing (20 & SUPPtsl_11Nvl {
	enginPHY_0; k pohw< wlxproids; k
		 * wh	_ada at!= pplycsiso/cd) codacogMS_t; i++tmcn'spngiofdmda at!=irc s*wlts_ofulocselecMed r at!/ b

 ban(|isgmcn_c_}
(c= trtk]i && "	pe(wle(isgacogMSde; i++(c= trtk]*& RSPEC rcms_c_bs)m ||21		1_AC isgofdm_c_}
(c= trtk]i, 	c	
	*lifs((c= trtk]*& RSPEC OVERRIDs_MCS_ONLY)hans	|| !(c= trtk]*& RSPEC OVERRIDs)i, __funcct*wltk]t&= ~(RSPEC STFhc_bs | RSPEC STChc_bs);a_funcs_ ForsSISObMCS ose STBC  bapossircm *et  	xt ||isgmcn_c_}
(c= trtk]ihans	
	*l&& Brate oTFhoe oTBC_TXuwpec)scbi, __funcMg8lstc0;_funcM < Nst f (dacogMS_t; i++ts_of
ways 1t!/ b

ccstc =inw b

ccct*wltk]t|= (shm(TXC1_and aSTBC << b

cc		RSPEC STFhorIFT) |
p set	    (stc << RSPEC STChorIFT)ombcf mute_t b

ccct*wltk]t|=hansspe(wl(phyctl1astft<< RSPEC STFhorIFT)om	 c_p0Mh	_ada at!=Is_TXPWol);anges ur->BSS ose 40MHZta[ *es? Ifda at!=sow ban pick			  zhsir->BSxbw r at!/ b

 banrd 11b/ = truew| wlc-cvoid
br) 
 b == sh40_MHZ	= BRCMn(wldeytxcha,xbwts_o20in40 SBt!/ b

cm */
ctlchuwt bm */
rxbwt hans	
	*CHSPEC SB_UPPER(anspr_liio chans_gmat b

cc			(hm ac if nopi)ihans	
	*?bshm(TXC1_BW_20MHZ_UPa:bshm(TXC1_BW_20MHZ;st  	xt ||isgmcn_c_}
(c= trtk]i, __funcM_anmcnf32cmu(rabe 40b/w DUP !/ b

cc ban();*wltk]*& RSPEC rcms_c_bs)hansspe(wls=i32	= BRCM

cm */
rxbwt hans	spe(wlshm(TXC1_BW_40MHZ_DUPw b

cchp_duse 	default:	/* bcf mute_tatic ps_ctm */
40rxbwt!=s	ret)BRCM

cm */
rxbwt  ps_ctm */
40rxbw;_funcM_ante_ta_prot,s_cdst	in m,el(f40 Mhzs	/* bcf te_tatic scb->ags & &sSCB IS40)BRCM

cm */
rxbwt  shm(TXC1_BW_40MHZombcf mute_tatic isgofdm_c_}
(c= trtk]i, __funcMtic ps_ctofdm_40rxbwt!=s	ret)BRCM

cm */
rxbwt  ps_ctofdm_40rxbwombcf mute_tatic  wlc-cck_40rxbwt!=s	ret) __funcMm */
rxbwt  ps_ctcck_40rxbwombcf mlc!c}ute_tx);

	c_ada aat!=mcn32cb_o40 b/w  uly.da aat!=Tsse s_opossircm  maiprobasthw,ebnfbnda aat!=a STApduril(fSCANda aat!et  	xt ||();*wltk]*& RSPEC rcms_c_bs)ls=i32	_funcM_anmcnf0s	/* bcf ct*wltk]t bRSPEC MIMOrcmsms
uncMm */
rxbwt  shm(TXC1_BW_20MHZ;spcc_p0Mh	_a_S->;c necesswidtht!/ b

md
brtk]t&= ~RSPEC BW_c_bsuptt
t ||(kcms_c_*S co(kc>_c_=ifsisgmcn_c_}
(c= trtk]i,s for );*wltk]t|= (m */
rxbwt<< RSPEC BW_orIFT)om	 cte_t b

c);*wltk]t|= (m */
ctlchuwt<< RSPEC BW_orIFT)om0Mh	_a_Dt brcmrpec(stGI,souldBRtry in ty	/ set  
md
brtk]t&= ~RSPEC ortsl_GI; tmcMm */
-reamhan
timi  b == shMM_PREAMBLEuptt
t ||oxroidtk]stags & &shm.aifs < TX_RC_GREEN_FIELD)
uncMm */
-reamhan
timi  b == shGF_PREAMBLEup;


 ban(oxroidtk]stags & &shm.aifs < TX_RC_MCS, 	c	
	*lifs(!isgmcn_c_}
(c= trtk]i,s __func*
brcmwarn;
		return;
	}

	/	}

	x = "Switch banIm.aifs < TX_RC_MCS !=sisgmcn_c_}
(c= tr)ld the set sshm ac params struct */
	acp_s-}p;


 banisgmcn_c_}
(c= trtk]i, __funcoreamhan
timitk]t ba */
oreamhan
timims i
	c_ada aat!= baSGIvisdBelecMed,w ban  macid mmda aat!=f (dacogMS_t; i++da aat!et  	xt ||();*wltk]*& RSPEC ortsl_GIihans	
	*l&& isgacogMSde; i++(c= trtk]*& b

cc		RSPEC rcms_c_bs)m b

ccoreamhan
timitk]t b == shMM_PREAMBLEuptt
_p0Mh	_a_peculCsbasbn_ser;angdiUacializeCs_basc!pec);is_mcn_c_}
(c= trt0]_ 	c	
	*lifs(oxpO];
stnt filt.e test0].da aaags & &shm.aifs < TX_RC_USE ortsl_PREAMBLE)m b

coreamhan
timitk]t b == shortsl_PREAMBLE;a c_phyute_tx);

enginPHY_0; k pohw< wlxproids; k
		 * wh	_a_S->;ctrlchuwta_o20Mhzs	/* bcmd
brtk]t&= ~RSPEC BW_c_bsuptt
ct*wltk]t|= (shm(TXC1_BW_20MHZt<< RSPEC BW_orIFT)om0Mh	_a_fma nr_lc)cnf:brcofdm-a[ *esrmu(rafo
	/*opolicias if t	
atesBrate ISNshmewat<  if rl&& isgofdm_c_}
(c= trtk]i, __funcmd
brtk]t&= ~RSPEC oTFhc_bs; b

c);*wltk]t|= phyctl1astft<< RSPEC STFhorIFT;df 
	wlcmphy_txs_ Red->cms_sevfma ose w, (bA*PDU's set txroidt0]

limac(s)0re txroidt1]

limac(s)0retxs_ (2	=->gmECTION,A
			c negasmt*wl*brcmt ||(b%_EDCF_STis_;oia(h->a[ *e_nt filtt ||211_ACb%_EDCF_STis_mgmt(h->a[ *e_nt filtt	 && "e(wl(phylan#>shm acRTSThresh) ifs!is_multicast_ec_ra_ d)r(h-> d)r1i, b
usw_rtst=hmacint
Cs_ (3) PLCP: decerminefPLCP hFF)w, 		ifMAC dur maci,
rcmsa	 * s6 txfu;
	txh *rcms_c_set_nempu w_plcpundicfr= trt0]cfp_llancfplcpowl	s_c_set_nempu w_plcpundicfr= trt1]cfp_llancfplcp_fwlc_hw,mac amsc/
(&txh->FragPLCPFwlc_hw,,hee(wlc);plcp_fwlc_hw,vert fromtxh->FragPLCPFwlc_hw,)macstc)
Langth:fbelCsouw pu    dCCK FBR CRClfbelCs_basFix isgcck_c_}
(cd
brt1]i, __futxh->FragPLCPFwlc_hw,t4]  cphylan#& 0xff;t utxh->FragPLCPFwlc_hw,t5]  c(phylan#& 0xff00m >> 8;phy_txs_ MIMO-rcms: neeifvali chaci*??astataMine tes = isgofdm_c_}
(c= trt0]_ ?hans| MA_shm_HDR_Grcms(u16 txfuofdm_o_lihdr#e) rlcp) :da  olcpt0]acstc)
DURlfbelCsrma aMinwroid(*/asFix !b%_EDCF_STis_pspoll(h->a[ *e_nt filtt && "e(wl!is_multicast_ec_ra_ d)r(h-> d)r1i ifs!usw_rtfsox);

turiC  han
	*ln_c_set_nempu w_a[ *e_dur(ndicfr= trt0]cfpreamhan
timit0],
for attnext_a[ g_lanof (	h->dur maci_iC  ccpu_to_le16(turiCp2X_mute_txtic usw_rtfsox);

 < NAViprotecMBSS enCsbrcnextawlxsthw,ebert f d(wl turiC  han
	*ln_c_)heturn fa_wlc_a[ *e_timeundicfr= trt0]chans	spepreamhan
timit0],
for a	 DOT11_MAX_FRAG_LEN)upt turiC + bRIFS_11N_TIMEmsanh->dur maci_iC  ccpu_to_le16(turiCp2X_mcstc)
DURlfbelCsrma  wlc_hw,croid if t banb%_EDCF_STis_pspoll(h->a[ *e_nt filtt)t utxh->FragDurFwlc_hw,  ch->dur maci_iC;l	

static isgmulticast_ec_ra_ d)r(h-> d)r1i S cusw_rtfsot utxh->FragDurFwlc_hw,  c0;l	

sta);

turiC  ln_c_set_nempu w_a[ *e_dur(ndicfr= trt1],
for attttttpreamhan
timit1]cfnext_a[ g_lanof (	txh->FragDurFwlc_hw,  ccpu_to_le16(turiCp2X_mcstc)
(4)fMAC-HDR: MacTxCt filtLuw if t bana[ gl s_c_ 	caclr|=*TXC_STARTMSDUre);pec);is_multicast_ec_ra_ d)r(h-> d)r1i, b
aclr|=*TXC_IMMEDACKntstFix 
		ac if not uptimi*r>band->gmode)
		rsaclr|=*TXC_FREQgmode)
ntstFix CHSPEC IS40;
		pr_liio chans_gmatch e_ if nopifi, b
aclr|=*TXC_BW_40up

s_ d->cAMIC bit Fixm,el(fms_dbg_maTKIP MIC if t banhwtkmic, b
aclr|=*TXC_AMIC;0);txh->MacTxCt filtLuw  ccpu_to_le16(aclmacstc)
MacTxCt filtHigh *rcmmch(s)0retxs_ S->; wlc_hw,croid preamhanvLimi*brcmt ||(preamhan
timit1]:
 b == shortsl_PREAMBLE) ||211_AC(preamhan
timit1]:
 b == shGF_PREAMBLE)ox);

 ban)t*wl2c_}
(cwlc-t1]i*!>band->rcms_1M;

		mchr|=*TXC_PREAMBLE_DATA_FBhortsl;phy_txs_ MacF[ *eCt filtastatamsc/
(&txh->MacF[ *eCt filt, &h->a[ *e_nt filtvert from_c_)););txh->TxFesTimeNormas( ccpu_to_le16(0);0);txh->TxFesTimeFwlc_hw,  ccpu_to_le16(0macstc)
TxF[ *eRAastatamsc/
(&txh->TxF[ *eRA, &h-> d)r1, ETH_ALEN)up

s_ TxF[ *eID set txh->TxF[ *eID  ccpu_to_le16(e[ *eiC);0);_adate TxSATUS_, Nohas-cloc voi,sttecreasel(f conairsb	a[ gi,stadBRtrres, dtrcmsa[ *e  ban weA
			neeifSS red->cms_ M_LFR cnt's viacms_csATUS_Oreg/*
			 txh->TxSATUS_O=ecpu_to_le16(sATUS_);0);_adate cxtralfbelCsvfma one asA*PDU aggreg maci,cms_cfg ffbelCsv->baadd->BSS
 cmsinefENDvbrcprevious s6 txf/* bsow bat Ft's nemp mahanvinwhe mac./*
			 txh->MaxNMpdS_O=ecpu_to_le16(0););txh->MaxABytes_MRTO=ecpu_to_le16(0););txh->MaxABytes_FBR =ecpu_to_le16(0););txh->MinMBytes  ccpu_to_le16(0macstc)
(5) RTS/CTS: decerminefRTS/CTS PLCP hFF)w, 		ifMAC dur maci,
rcmsaurnish s6 txfu;
	txh *rcms_ RTS PLCP hFF)w, 		ifRTS a[ *e*brcmt ||usw_rtst||pusw_ctsox);

 banusw_rtstifsusw_ctso

		usw_ctst b_err(wl;

enginPHY_0; k po2; k
		 * wh	rtn_ct*wltk]t bthe ha dchans_to_rtn_ct*wl(ndicfr= trtk],
for a	attttttppppp,
spppp	attttttm */
ctlchuw);a c_p0Mhpec);is_ofdm_c_}
(ctn_ct*wlt0]T && "	awlc!(n)t*wl2c_}
(ctn_ct*wlt0]T =>band->rcms_1M; ||21	tttttttch e_PLCPHdr
	default:
 b == shPLCPhLONGi,s __functn_oreamhan
timit0]t b == shortsl_PREAMBLE;a c	mchr|=*TXC_PREAMBLE_RTS_MAINhortsl;phc_p0Mhpec);is_ofdm_c_}
(ctn_ct*wlt1]T && "	awlc!(n)t*wl2c_}
(ctn_ct*wlt1]T =>band->rcms_1M; ||21	tttttttch e_PLCPHdr
	default:
 b == shPLCPhLONGi,s __functn_oreamhan
timit1]t b == shortsl_PREAMBLE;a c	mchr|=*TXC_PREAMBLE_RTS_FBhortsl;ph u16 c_anRTS/CTS addiUacis SS MacTxCt filtLuw if t
 banusw_ctsox);

;txh->MacTxCt filtLuw | ccpu_to_le16(TXC_SENDCTSoo cc}ute_tx);

	txh->MacTxCt filtLuw | ccpu_to_le16(TXC_SENDRTSoo cc	txh->MacTxCt filtLuw | ccpu_to_le16(TXC_LONGFRAME);ph u16 c_anRTS PLCP hFF)w, */c	cmtn_olcp:
soxhacRTSPhyHFF)w,; t
 banusw_ctso_functn_o_llan  cDOT11_CTSnLEN#+xFCS_LEN; t
te_t b

ctn_o_llan  cDOT11_RTSnLEN#+xFCS_LEN; 
tard 11b/gcempu w_plcpundicfrtn_ct*wlt0]cfctn_o_llan  ctn_olcp)up0

s_  wlc_hw,croid defsaci*,stRTS PLCP hFF)w, */c	crd 11b/gcempu w_plcpundicfrtn_ct*wlt1]cfctn_o_llan hans	
ctn_olcp_fwlc_hw,);ph amsc/
(&txh->RTSPLCPFwlc_hw,,
ctn_olcp_fwlc_hw,,
spe(wle(wrt fromtxh->RTSPLCPFwlc_hw,))up0

s_ RTS a[ *e*fbelCs... */c	cmtnt bu16 txfu2%_EDCF_STrtst*)&txh->ctn_a[ *eup0

turiC  ln_c_set_nempu w_rtscts_dur(ndicfgsw_ctscfrtn_ct*wlt0]c
for atttttt ct*wlt0]cfctn_oreamhan
timit0],
for atttttttpreamhan
timit0]cfp_llancfpppppp2X_cmtn->dur maci  ccpu_to_le16(turiCp2X_
s_  wlc_hw,croid defsaci*,stRTS DURlfbelCsd(wl turiC  ln_c_set_nempu w_rtscts_dur(ndicfgsw_ctsc
for atttttt ctn_ct*wlt1]cfc= trt1],
for attttttfctn_oreamhan
timit1],
for attttttfpreamhan
timit1]cfp_llancfpppppp2X_ctxh->RTSDurFwlc_hw,  ccpu_to_le16(turiCp2X t
 banusw_ctsox);

;mtn->a[ *e_nt filt  ccpu_to_le16(hm.aifs < FTYPE_CTL |
p set		shm.aifs < STYPE_CTS)om0Mh	amsc/
(&mtn->ra, &h-> d)r2, ETH_ALEN)uplcmute_tx);

	ctn->a[ *e_nt filt  ccpu_to_le16(hm.aifs < FTYPE_CTL |
p set		shm.aifs < STYPE_RTS)om0Mh	amsc/
(&mtn->ra, &h-> d)r1, 2 *cETH_ALEN)uplcm16 c_anmMine te
* ate   	/*o8*bito: aMinwa[ gir;te/mcn,
* ate   ate o8*bito: rts/rtsir;te/mcn
aat!et  aMine tes |= (is_ofdm_c_}
(ctn_ct*wlt0]T ?hans	| MA_shm_HDR_Grcms(
p setu16 txfuofdm_o_lihdr#e) ctn_olcp) :da  cmtn_olcpt0]T << 8;phyute_tx);

ams->txtxhacRTSPhyHFF)w,  0/*| M_shm_HDR_LENmaceMams->tx&txh->ctn_a[ *e,pnvert from16 txfu2%_EDCF_STrts)maceMams->txtxh->RTSPLCPFwlc_hw,,
0,wrt fromtxh->RTSPLCPFwlc_hw,))up_ctxh->RTSDurFwlc_hw,  c0;phy_t#ifdef SUPPtsl_40MHZstc)
 d) null delimiser;anu		 !/ b ban(oxpO];
stags & &shm.aifs < TX_MAC A*PDU_=ifsisgmcn_c_}
(c= trt)t utxh->RTSPLCPFwlc_hw,[A*PDU_FBR_ & S_DELIM]  han
	*n_c_b/gRmpdS_null_delim_cnttch e_ampdSc)scbcfr= tr,lo_llan)om0#*	dif0);_adate N/*o bat RTS/RTS FB preamhanvLimisv->baon chad,dw
sta
 cmsineffibas((100 /*
			 txh->MacTxCt filtHigh  ccpu_to_le16(ach);0);_adate MMinR tes (bothcms_ Mts 		if [ giolcp:e tes hava
 cmsbean cwlculcheCsouw)/*
			 txh->MainR tes  ccpu_to_le16(aMine tesmacstc)
XtraF[ *eTimisv			 xftst b_[ *etimiuct*wlt1]cfps_ctm */ft)up_xftst|= (_[ *etimiuctn_ct*wlt0]cfps_ctm */ft) << XFTSnRTS_FT_orIFT)om	xftst|= (_[ *etimiuctn_ct*wlt1]cfps_ctm */ft) << XFTSnFBRRTS_FT_orIFT)om	xftst|= CHSPEC CHANNEL;
		pr_liio chans_gmatch e_ if nopifi << b

cc		ttttfXFTSnCHANNELhorIFT;dftxh->XtraF[ *eTimisv ccpu_to_le16(xftsmacstc)
PhyTxCt filtWordeset ookctl) b_[ *etimiuct*wlt0]cfps_ctm */ft);cmt ||(preamhan
timit0]:
 b == shortsl_PREAMBLE) ||211_AC(preamhan
timit0]:
 b == shGF_PREAMBLE)ox);

 ban)t*wl2c_}
(cwlc-t0]i*!>band->rcms_1M;

		ookctl)|  shm(TXChortsl_HDR;phy_txs_ p	brxant	in properly bit shiftedeset ookctl)| ln_c_set_stf_;
	hdrsuookctl_rxantundicfr= trt0]););txh->PhyTxCt filtWorde ccpu_to_le16(ookctlmacstc)
PhyTxCt filtWord_1 !/ b ban == shPhm(11N_CAPewat<  if r; 40Mh_c_vp	bctl1  core
  p	bctl1  c
	if (wlp	brxctl1a_wlcundicfr= trt0]););;txh->PhyTxCt filtWord_1e ccpu_to_le16(ookctl1es 	mp	bctl1  c
	if (wlp	brxctl1a_wlcundicfr= trt1]););;txh->PhyTxCt filtWord_1_Fbre ccpu_to_le16(ookctl1es ;

 banusw_rtst||pusw_ctsox);

mp	bctl1  c
	if (wlp	brxctl1a_wlcundicfrtn_ct*wlt0]To cc	txh->PhyTxCt filtWord_1_Rtsv ccpu_to_le16(ookctl1es 	mmp	bctl1  c
	if (wlp	brxctl1a_wlcundicfrtn_ct*wlt1]To cc	txh->PhyTxCt filtWord_1_FbrRtsv ccpu_to_le16(ookctl1es 	m_p0Mh_ada ateFngimcnfa[ *es, Fixmixed	cas(	def *adeCsf, (bloe_spreamhan)da ateisdgoel(f 		bans	/bra	 * inwnon-zerS MMcasLan if /mada te MMcasFbrLan i_ w, * banunnet !=aryaticms_yv->basepac_}
d
aat!et   banisgmcn_c_}
(c= trt0]T && "	awlc(preamhan
timit0]:
 b == shMM_PREAMBLE,s __funic_vm	caslan Fbsl"aifs	_c_set_cwlc_lsig_lanundicfr= trt0]cfp_llanoo cc	txh->MMcasLan  ccpu_to_le16(a	caslan);a c_p0Mhpec)isgmcn_c_}
(c= trt1]T && "	awlc(preamhan
timit1]:
 b == shMM_PREAMBLE,s __funic_vm	casfbrlan Fbsl"aifs	_c_set_cwlc_lsig_lanundicfr= trt1]cfp_llanoo cc	txh->MMcasFbrLan  ccpu_to_le16(a	casfbrlan)ombc}phy_txac  crkb_
rad ++++_mappel((ptwlt ban(scb->ags & &sSCB WMECAP_=ifsqos(ifswat< edcf_rxop[ac]ox);

u, "wa[ g_durvedurvedur_fwlc_hw,up0

s_ WME: Un cha TXOPcmsresholCsd(wl pec);(oxpO];
stags & &shm.aifs < TX_MAC A*PDU_=ifsa[ gl s_c_ __funa[ g_dur Fbsl"aifs	_c_set_cwlc_a[ *e_timeundicfr= trt0]chans	spreamhan
timit0]cfp_llan)up;


 banrtsox);

m
s_ 1eRTS ngiCTS-to-self a[ *e*brcm			dur Fbsl""aifs	_c_set_cwlc_cts_timeundicfrtn_c= trt0]chans	spettttfctn_oreamhan
timit0]To cc		dur_fwlc_hw, Fbsl""aifs	_c_set_cwlc_cts_timeundicfrtn_c= trt1]chans	spettttfctn_oreamhan
timit1]To cc	tc)
(SIFS#+xCTS)#+xSIFS#+xa[ *e*+xSIFS#+xACK *et  l tur + cle16_to_cpu(mtn->dur maciTo cc		dur_fwlc_hw, + hans	sle16_to_cpu(txh->RTSDurFwlc_hw,)wlc!c}ute_txtic usw_rtfsox);

l tur =ia[ g_duro cc		dur_fwlc_hw, Fnfo ccc}ute_tx);

	c_axa[ *e*+xSIFS#+xACK *et  l tur =ia[ g_duro cc		dur + hans	
	*ln_c_set_nempu w_a[ *e_dur(ndicfr= trt0]c b

cc		ttpreamhan
timit0]cf0);a_funcdur_fwlc_hw, Fbsl""aifs	_c_set_cwlc_a[ *e_timeundicfr= trt1]chans	sp	oreamhan
timit1],
for a		p_llanoo cc		dur_fwlc_hw, + hans	
	*ln_c_set_nempu w_a[ *e_dur(ndicfr= trt1]c b

cc		ttpreamhan
timit1]cf0);a	 
	wlcc_anNEED codaet TxFesTimeNormas((ms_d) */san;txh->TxFesTimeNormas( ccpu_to_le16(n_c_)hdur);wlcc_absl"aanNEED codaet  wlc_hw,croid defsaci*,sbsl"aanTxFesTimeNormas((ms_d)bsl"aa/san;txh->TxFesTimeFwlc_hw, Fbsl""cpu_to_le16(n_c_)hdur_fwlc_hw,);pwlcc_absl"aanon cha rxopdNy_cvmsresholCs(rxopdminusvin
raa[ *esra"aan	defhFF))bsl"aa/san;Fix 
		acedcf_rxop[ac]s>s_(tur -ia[ g_duri, __funcu;

	newa[ gmsresh;a_funcnewa[ gmsresh Fbsl""aifs	_c_set_cwlc_a[ *e_lanundic b

ccr= trt0]cfpreamhan
timit0],
for a 
		acedcf_rxop[ac]s- b

cc	(tur -ia[ g_duri,To cc	tc)
rnegasbimad wlc-a[ gmsresholCsd(wl 	  bannewa[ gmsresh < DOT11_MIN_FRAG_LEN) b

ccnewa[ gmsresh Fbsl"""aifsDOT11_MIN_FRAG_LENo cc	tte_txtic newa[ gmsresh >bsl"""a
		acusr_a[ gmsresh) b

ccnewa[ gmsresh Fbsl"""aifs
		acusr_a[ gmsresho cc	tc)
on cha rlc-a[ gmsresh 		ifdo rxc
on cha d(wl 	  ban
		aca[ gmsresh[ ++++]*!>bsl""aifsn_c_)hnewa[ gmsresh) b

cc
		aca[ gmsresh[ ++++]*=hansspe(wl(_c_)hnewa[ gmsresho ccc}ute_tx);

	c*
brcmwarn;
		return;
	}

	/	}

	x = "Switch b rxopdinvali _engims_cv%d fill inet sshm ac params strfifo_n *es[ ++++]ll inet ss)t*wl2c_}
(cwlc-t0]iacp_s-}p;


 bantur >swat< edcf_rxop[ac]o;

	c*
brcmwarn;
		return;
	}

	/	}

	x = "Switch bch b rxopdexceedeCso_llan %d/%d)dur %d/%dld the set sshm ac params struct */
	the set ssfifo_n *es[ ++++]ll inet ssp_llancf
		aca[ gmsresh[ ++++]ll inet ssdurvewat< edcf_rxop[ac]oombc}phy_txms_c_in0; rate, inde, "wb_c_set_tx;16 txfunfl[NFIFO];
#endic)cnint_usk_t(st>*rkbon becnint_udma_ pa *dma;ef, "wfifo, M_L =	-ENOSPC;becnint_u;
	txh *txhre ic_va[ *eiC#=shNVALIDFID; a ei;
#=snfl[NFas_to_ei;
(rkb_
rad ++++_mappel((rkbooombdma ifosz_fixrn;i[ei;
];t txh  bu16 txfu;
	txh *)(rkbrn;oiaif lt bandmacounapec)l s_c_ __fu_ada ateW bsometimes_gMS_a a[ *e rs_c_aMAifs < aftac stoppel(
	 cmsinef ++++s.=Tsse  uly emac seem_s			bana acogMS_a[ *e
	 cms		ifisdBeem_slikmlyl			bana race.*TX_HEADROOM_peculC
	 cmsen	/* b bat we hava enoughdspacel			h		icms-clse ptray
	 cmsthw,ebn,bsowwarnaticms_ri*isn't. If we'rt:o	  ofdspaceda atei down,tx ril(f || wlc-tx  ++++ isn't stoppe| wlcnda atewe'v_ M_ally goS_a bug;wwarnaloudlyaticmsat happen	.
aat!et  *
brcmwarn;
		return;
	}

	/	}

 = "Ret iveCsa[ *e rtic xsf, (bnodspaceli dDMA ril(et acp_sWARN_ON !b%_EDCF_ST ++++_stoppe| 
		ac parab%_E_hwc
spinet rkb_
rad ++++_mappel((rkboop2X_cms_c_in-ENOSPC;bey_txs_ Wban a BC/MCca[ *e  s	bail(fnemmitin t
ow baslspC	ei;

 cmsviacDMA (NOT PIO),
on cha one asticBSS O];
#as= ppropri mm./*
			 * BMei;
#==*TX_lspC_t re	
x-a[ *eiC  cle16_to_cpu(txh->TxF[ *eIDmacstc)
CemmitslspC	sw, Bncelnumber:i down,SHMca[ *e ID lo_wtaci*if t bana[ *eiC !=shNVALIDFID_ __fu_ada ateTo O];
rm;own,Rne astrcms_clas"wmcastaa[ mbapos}
d
aat!bsow bat Ft can clear m

	_oia bit
aat!et  *
brcmb_w
sta_shm;
		retu, M_lspC_t D,na[ *eiCp2X_mcstM_L =	b_c_set_txei;
(ndicfaifo, rkbo;);_adate T_raonfo hwasonf mais_c_set_txei;
t
owfec)l s	bacausa
 cmsinert:wertn't anycDMA descriptors,cb	  we'v_ almctry
 cms_proted-atic cat. SoaticFt doesvfec)lyellaloudly./*
			 WARN_ON_ONCE(M_L)ms iE matchcce;dmcs*/
	return fasendpktuaMAifs <;16 txfunfl[NFIFO];
#endic)cnint_usk_t(st>*rdu,hans(wlc))cnint_u2%_EDCF_SThw *hw)s_dfu, "wei;
;becnint_uscb *scb truw		repri_scb; a ei;
#=snfl[NFas_to_ei;
(rkb_
rad ++++_mappel((rdu)owl	s_c_set_;
	hdrsuaMAifs <;ndic)tu, sdSc)scbcf0, 1cfaifo, 0twlt ban!b_c_set_tx;ndic)cdu)oX_cms_c_inmacint
Cs_ thw,ebediscs_dedeset dev_kfr_E_rkb_any(rdu););ms_c_in_err(wlne.;


s_c_set_txei;
(16 txfunfl[NFIFO];
#endicfg, "wfifo, cnint_usk_t(st>*pon becnint_udma_ pa *dma ifosz_fixrn;i[ei;
];t , "wrce;dfuc_v ++++;cstM_L =	dma_txeast;ndic)dmacfptwlt banM_L	< 0)+BRwio_lmc));
		rewio_l, "txei;
:b_etal,t
osnfa[ *es !!!et acp);_adate Stop  ++++ ifdDMA ril(l s	full. Red-rv bsome fr_E descriptors,date as=w bsometimes_ret ive_a a[ *e rs_c_aMAifs < aftac inef ++++sdate a>bastoppe|./*
			  ++++ =crkb_
rad ++++_mappel((ptwlt bandmacounapec)l<=*TX_HEADROOM_ifsai;
#<*TX_lspC_t re && "e(wl!i%_EDCF_ST ++++_stoppe| 
		ac parab%_E_hwcf ++++)oX_ci%_EDCF_STstopT ++++ 
		ac parab%_E_hwcf ++++)ms iE matchcce;dmcsg32
the ha dchans_to_rtn_ct*wl(16 txfunfl[NFIFO];
#endicfg32 rt*wlc
spi wl*/
	rusw_rt*wlc uc_vm */
ctlchuw)s_dfu32 rtn_ct*wl  core
 tic usw_rt*wlonchp_duse a[ *e ms_cvas=rtsnms_cv*/c	cmtn_ct*wl  cct*wl;df

static ps_ct if nog	cascifswat< protecMaci->_g ifs!is_cck_c_}
(cd
br)onchp_dUsta11Mbps as_TXPWg protecMacieRTS targMS_ms_cva	if wlc_hw,.
aat!dUstaTXPWs;
	uiN_si dc_}
()alookupt
owfi|| wlc-be(rab_si Mroid/* atemadac ineftargMS_inoc voi11 Mbps issouldB_si .
aat!d6va	if9 Mbps a>baouldus ally BelecMedtbycms_cvBelecMaci,cb	 
	 cmseman  rcms_cOFDM ms_cv_wr->baprotecMal(l s	6stic9 Mbps, 11
a ateisdm

	 robust.
aat!et  mtn_ct*wl  cs;
	uiN_si dc_}
(ndicflnd->rcms_11Mmwl	

st
tac)
_wlculchanRTS ms_cva	if wlc_hw, ms_cvN_sedtonc cona[ *eMroid/* ateRTS mu(rabe s*		 aS_a b_si Mroid acocelit	in ada atent filt a[ *e,pss  9.6strc802.11 t*wl
aat!et  mtn_ct*wl  cs;
	uiN_si dc_}
(ndicfc= tr);0); ban == shPhm(11N_CAPewat<  if r; 40Mhs_ d->crtsttxbwt
ow}

recMBsult:N_PCs_et  mtn_ct*wl &= ~RSPEC BW_c_bsup_fu_ada ateirc s*wl/chans_ wlc_hw, b_o40MHz,w ban sendeRTS nn bothda ate20MHz;c necess(DUP),
oc_rawise sendeRTS nn nt filt c neces
aat!et  t || s*wldis40mhz(cd
br) ifs!is_cck_c_}
(ctn_ct*wl)o_functn_ct*wl |= (shm(TXC1_BW_40MHZ_DUPt<< RSPEC BW_orIFT)om	 te_t b

ctn_ct*wl |= (m */
ctlchuwt<< RSPEC BW_orIFT)om0Mhs_ tick	siso/cd) as_deytxchafngiofdmt!et   banisgofdm_c_}
(ctn_ct*wl)	 * wh	rtn_ct*wlt&= ~RSPEC oTFhc_bs; b

ctn_ct*wl |= (
		c-rs.coss_op	cast<< RSPEC STFhorIFT)om	 } (
);ms_c_inctn_ct*wlwlratms_Un cha s = wlslis}
n   tacval*inoshar->Bmem

ypbrce, index;
	u8 basic_bcn_li_on (16 txfunfl[NFIFO];
#endi)s_dfms_w);

uptemacycDTIM b_o		  zhytxchaif t banwat<  cn_li_dtim *>b1)t  *
brcmb_w
sta_shm;
		retu, M_lsN_LI, 0twlt

st
tard 11bb_w
sta_shm;
		retu, M_lsN_LI,hans(wlc))nwat<  cn_li_dtim << 8p | wat<  cn_li_ cn)wlrate, index;
	
n_c_seb_mctr_tsf(16 txfunfl[NFms_dbg_maendi_hwcfu32 *tsf_l_ptr,
spe(u32 *tsf_h_ptrve been bandus_b_
	brcm *cle)fifosz_ixrn;
	}

	; dfms_mctr ineftsfatimer 	/*,w ban ate o
owgMS_an atomictmctr if t*tsf_l_ptr  cns a_mctr32;}

	/*D11REGOFFS(tsf_timer	/*)owl	*tsf_h_ptr  cns a_mctr32;}

	/*D11REGOFFS(tsf_timerate ))msthis shourec	def 64bit TSF_(100 -as_c_inef16bit TSF_(100 -i down,rx hFF)w,fo *g man own,assumpMacie bat own,TSF_pas, d-i dhFF)w, iopw, (i d65msshoutrcms_cct
 *		ftsf.shofo *6ttttttf5ttttttf4ttttttf4ttttttf3ttttttf2ttttttf1fo *3.......6.......8.......0.......2.......4.......6.......8......0fo *|<---------- tsf_h ----------->||<--- tsf_l -->||<-RxTSFTime ->|shofo *T_raRxTSFTime artaTXPW	/*e(rac_v
its 		ifprovidedtbycown,Rne a.=Tseshoutsf_l  s	fill d-i dbycn_c_seb_mccv, which  s	ds * earlier:i downshourec ive_caic sw, Bncelaftac rx   tacrupt. Onfo own,ate ac c_v
its
te a>baused. Fibasly, ineftsf_h  s	mctr as_c_ineftsfaregis}
r.shof it is sg64bthe ha dcec	def_tsf64(16 txfunfl[NFIFO];
#endicbsl""a16 txfu;
	rxhdr#erxh)s_dfu32 tsf_h,utsf_l;dfuc_vrx_tsf_0_15,vrx_tsf_16_31;0);8 basib_mctr_tsf(
		retu, &tsf_l, &tsf_h);0);rx_tsf_16_31  bu_c_)(tsf_l >> c_););rx_tsf_0_15  ccxh->RxTSFTimecp);_adate a greasac isfatime   dic tes TXPW	/*ac_v
its ,sbshoutsf_l wrapped,bsowdecrem*		 -cloate oc_v
its./*
			 * BMu_c_)tsf_l <vrx_tsf_0_15)x);

mx_tsf_16_31 -=inw b
t || x_tsf_16_31  = 0xffffo_funtsf_h -=inw by_txms_c_inMu_64)tsf_h << 32p | (((g32) x_tsf_16_31 << c_)h+vrx_tsf_0_15)wlrate, index;
	
prepuaMAifs <_sATUS_;16 txfunfl[NFIFO];
#endic)cnint_u;
	rxhdr#erxhc
sp(wlc)cnint_usk_t(st>*pc
sp(wlc)cnint_u2%_EDCF_STrx_sATUS_#erx_sATUS_on be, "wc neces;e i32 rt*wl;e insigb d-c nr *rlcp; dfms_a	 * inwTSF_a	if la(l tn pred-nceloteccx_sATUS_< wlctime =bthe ha dcec	def_tsf64(ndicfcxhoo ccx_sATUS_<  la(l|= RX_FLAGhc_CTIME_START; dfc necess b == shCHAN CHANNEL;cxh->RxC ne);0);rx_sATUS_< N_PCs=hanc necess> c4 ? NLDCF_STgmode)
HZ : NLDCF_STgmode2GHZ;spcx_sATUS_<  reqriX_ci%_EDCF_STc neces_to_erw, Bncy(c neces,vrx_sATUS_< N_PC);0);rx_sATUS_< sigbas(s)
		pr_lirssi_nempu w;
		return if nopicfcxhoo 

s_ noise *rcms_ q al_oteccx_sATUS_< RAit(wlciX_c;cxh->PhyRxSATUS__0l& PRXS0_RXANT_UPSUBgmod) ? 1 : ore
 olcp:
sprn;oia;0);rt*wl  cs;
	uit_nempu w_rt*wl(rxhcfplcpowl	t ||isgmcn_c_}
(c= tr)ox);

cx_sATUS_< c_}
_idxs= rt*wl*& RSPEC rcms_c_bsup

cx_sATUS_< cnne al(l= RX_ENC_HT;t  t || s*wldis40mhz(cd
br)o_funcx_sATUS_< Nw  crcms_INFO_BW_40upc}ute_tx);

sd, ch (rt*wl2c_}
(cwlc-)ox);

c voilnd->rcms_1M: wh
cx_sATUS_< c_}
_idxs= fo cccu
eakelwwc voilnd->rcms_2M: wh
cx_sATUS_< c_}
_idxs= 1o cccu
eakelwwc voilnd->rcms_5M5: wh
cx_sATUS_< c_}
_idxs= 2o cccu
eakelwwc voilnd->rcms_11M: wh
cx_sATUS_< c_}
_idxs= 3o cccu
eakelwwc voilnd->rcms_6M: wh
cx_sATUS_< c_}
_idxs= 4o cccu
eakelwwc voilnd->rcms_9M: wh
cx_sATUS_< c_}
_idxs= 5o cccu
eakelwwc voilnd->rcms_12M: wh
cx_sATUS_< c_}
_idxs= 6o cccu
eakelwwc voilnd->rcms_18M: wh
cx_sATUS_< c_}
_idxs= 7o cccu
eakelwwc voilnd->rcms_24M: wh
cx_sATUS_< c_}
_idxs= 8o cccu
eakelwwc voilnd->rcms_36M: wh
cx_sATUS_< c_}
_idxs= 9o cccu
eakelwwc voilnd->rcms_48M: wh
cx_sATUS_< c_}
_idxs= 1fo cccu
eakelwwc voilnd->rcms_54M: wh
cx_sATUS_< c_}
_idxs= 11o cccu
eakelww
	ytxch: whM*
brcmc));
		return;
	}

	/	}

	 s" bchUnknown c_}
et truct */
	acp_s_p0Mh_ada ateFngi)
Hz,=w bseculCsdecreastaTXPW  dex as_it	in
	 cms	 subd->ctrcms_c2.4Gcroidt. SeelbitroidslfbelC
	 cmstrcs;
	uiN_nde)
Hz_nr_l||inbaMAifs < if.c).
aat!et  t || x_sATUS_< N_PCs== NLDCF_STgmode)
HZo_funcx_sATUS_< c_}
_idxs- b == shLEGACYe)
>rcms_OFFSETom0Mhs_ Decerminefpec(stpreamhanv_PCsc_}
_idxs!et   banisgcck_c_}
(cd
br)ox);

	pec)cxh->PhyRxSATUS__0l& PRXS0_ortslHs for )x_sATUS_< cnn_ags & |= RX_ENC_FLAGhortslPRE;a c_ute_tatic isgofdm_c_}
(c= tr)	 * wh	rx_sATUS_< cnn_ags & |= RX_ENC_FLAGhortslPRE;a c_ute_ta{ whM*
brcmc));
		return;
	}

	/s" bchUnknown m		co macild the se suct */
	acp_s_phy_txt ||olcp3dissgi(olcpt3])oX_cmx_sATUS_< cnn_ags & |= RX_ENC_FLAGhortsl_GI; tmpec)cxh->RxSATUS_1*& RXS_DECERRox);

cx_sATUS_<  la(l|= RX_FLAGhFAILEDhPLCPhCRC;beM*
brcmc));
		return;
	}

	/s" bch RX_FLAGhFAILEDhPLCPhCRC fill in suct */
	acp_}tmpec)cxh->RxSATUS_1*& RXS_FCSERRox);

cx_sATUS_<  la(l|= RX_FLAGhFAILEDhFCS_CRC;beM*
brcmc));
		return;
	}

	/s" bch RX_FLAGhFAILEDhFCS_CRC fill in suct */
	acp_}trate, index;
	
n_c_set_mccvctl(16 txfunfl[NFIFO];
#endic)cnint_u;
	rxhdr#erxhc
spcnint_usk_t(st>*pon be, "wlan_mpdS;efcnint_u2%_EDCF_STrx_sATUS_#rx_sATUS_;efcnint_u2%_EDCF_SThdr#ehdr; tmams->tx&rx_sATUS_,
0,wrt fromrx_sATUS_oacp_prepuaMAifs <_sATUS_;ndicfcxhcfp, &rx_sATUS_oo 

s_ aMAdhFF)w,+bodyllangthcfexclude CRCl		ifplcp:hFF)w, */c	lan_mpdS F)prnlan -*| M_shm_HDR_LEN#-xFCS_LEN; trkb_pull(p/*| M_shm_HDR_LENmac	__rkb_trim(p  lan_mpdSoo 

s_ unmu woiransmitaif t banwat< turnsu= t ded_ei;
sox);

hdr# bu16 txfu2%_EDCF_SThdr#e)prn;oia;0	t banb%_EDCF_STis_s = wl(hdr->a[ *e_nt filtt)t u;8 basib_mu w;
		retucfpppppp2X_}
atamsc/
(hm.aifs < SKB_RXCB(pt, &rx_sATUS_,wrt fromrx_sATUS_oacp_2%_EDCF_STrx_irqsaf+ 
		ac parab%_E_hwcfpowlratms__wlculchana[ *eMdur maci fngiMixed-	cascL-SIG spoofeng,sms_c_i
te number:brcbytesvgoesvi down,langth:fbelCshofo *Fngmulc*g man by HT shm S*wl*v 1.13
ate  lan  c3(nsyms#+xnt; i++t+ 3)#-x3sho/
gc_
n_c_set_nwlc_lsig_lanu16 txfunfl[NFIFO];
#endicfg32 roidspwlc
sp(wlc)fg, "waMA_lanon beu;

	nsyms, lan  c0,wkNdps; tmpec)isgmcn_c_}
(coidspwl)ox);

u, "wmcnf= roidspwl*& RSPEC rcms_c_bsup

, "wtotde; i++nt bumcn_2_txe; i++n(mcn)x+ 1)x+he se s s*wldstc(coidspwl)up_fu_ada ate basthy *adMdur maci _wlculchaci ma ches TXat rat!i,sttegulcgiofdmda t!/ b
s_ 1000Ndbps = kbps * 4t!/ b
kNdpst bacn_2_c_}
(mcn,  s*wldis40mhz(coidspwl)the se s  s*wldissgi(coidspwl)ox* 4o t  t || s*wldstc(coidspwl)l s_c_ 	c	nsyms#Fbsl"aifsCEIL((Ashm_SERVICE_NBITS#+x8 *waMA_lanx+he se sAshm_TAIL_NBITSox* 1000,wkNdps)om	 te_t b

s_ STBC neeisl			h	va eman number:brcsymbols if t	
nsyms#Fbsl"aifs2 *bsl"aifsCEIL((Ashm_SERVICE_NBITS#+x8 *waMA_lanx+he se sAshm_TAIL_NBITSox* 1000,w2 *ckNdps)om b
s_ (+3)#acanu		 fngiHT-SIG(2	=		ifHT-STF(1) if t	nsyms#+ butotde; i++nt+ 3);_fu_ada ate3cbytes/symbol @dl-gac)s6Mbps roid/* ate(-3)#excludal(ld-rvrcm 
its 		iftec)l
its
a t!/ b
lan  c(3 *cnsyms)#-x3w by_txms_c_inM_c_)hlanrerate, index;
	
n_c_set_	ca_prb_rt*dc_}
_Lbrcm(16 txfunfl[NFIFO];
#endicfg, "wf[ *e_lanmn be wlst 16 txfunfl[NFIFcoids	/ srs_dflt;efcnint_unfl[NFIFcoids	/ rn;spi8 roid;dfuc_ve fiy_ptr;spi8 olcpt| M_shm_HDR_LEN]ref_c_vdurvertfswl
u, "wiwlsprtfsc=s
radetfsM i <  if rt ;
rs_dflt =unfl[NFIFcoids	/_
radhwrsM i );0);8 basiIFcoids	/_co/
(rs_dflt, &rsowl	s_c_set_coids	/_acn_on (&rs,c
		c-rs.cotxe; i++nacp);_adate walk_TXPWol);roid Lbrcm 		ifon cha MAC cle)fSHM
 cmsb_si Mroid Lbrcm e fii+sdate/a enginiHY_0; i <vrs.anu		; i
		 * whroid = rt.e testi]*&  == shrcms_c_bsup0

e fiy_ptr  cs;
	uiNdc_}
_shmgoffsmatch e_hwcfc_}
)om b
s_ CwlculchanTXPWProbasRdspwlsefPLCP atic cong man ms_cv*/c	crd 11b/gcempu w_plcpundicfraid,sa[ *e_lancfplcpowl_fu_ada ateCwlculchanTXPWdur maci trcms_cProbasRdspwlseda atea[ mbaplS_#SIFS#atic conMAC
a t!/ b
tur =in_c_)heturn fa_wlc_a[ *e_timeundicfraid,hanssp	and-ShLcti_PREAMBLE,ba[ *e_lanmac  tur + crtfswl b
s_ Un cha rlc-SHMcRcha Tbrcm e fiyWProbasRdspwlsef(100 st!et  *
brcmb_w
sta_shm;
		retu, e fiy_ptr + M_sl_PRshPLCPhPOS,hans(wlc))n_c_)h(olcpt0] + (olcpt1] << 8pop2X_c*
brcmb_w
sta_shm;
		retu, e fiy_ptr + M_sl_PRshPLCPhPOS + 2,hans(wlc))n_c_)h(olcpt2] + (olcpt3] << 8pop2X_c*
brcmb_w
sta_shm;
		retu, e fiy_ptr + M_sl_PRshDURhPOS,hdur);wl}lne.;

heturn fa
radhFF)w,_lanux;
	mn bems_c_inTXOFFrerate, index;
	u8 basic_b = wl_w
sta(16 txfunfl[NFIFO];
#endicbsl""a16 txfusk_t(st>*b = wlc uc_vtimgoffsmacbsl""a_c_vdtimgperiod,l*/
	rbcn0,l*/
	rbcn1ve beet f_thlanre cnint_u2%_EDCF_STtxpO];
#etxpO];
;mb16 txfunfl[NFms_dbg_maendi_hw ifosz_fix;efcnint_u2%_EDCF_SThw *b%_E_hw  c
	if (wlpubM i )rab%_E_hwwlstc)
G->;oxpO];
#eet oxpO];
#=shm.aifs < SKB_CB(b = wlowl_flan  cmin_t(et f_t, s = wlrnlan, lsN_TMPL_LENmac	wat<  cn_rt*wl  c2%_EDCF_ST
radtxproid(b%_E_hwcfoxpO];
)_fix_(100 ;hl	s_c_set_nempu w_plcpundicfwat<  cn_rt*wl,hans(wlc)lan#+xFCS_LEN -*| M_shm_HDR_LEN, s = wlrn;oiaif ltc)
"Regulcg" 		ifc_vMBSS b	  ouldatic4vMBSS *rcms_ Un cha rlc-p	brxctl#atic cons = wlsN_sedtonc conmt*wl*brcm8 basic_b = wl_p	brxctl_rxant_on (ndicfwat<  cn_rt*wl);0); banbcn0; 40Mhs_ w
stal basprobasrdspwlsefi

ow bastemplchanregici*if tc*
brcmb_w
sta_templchaprom;
		_hwcfT_lsN0_TPL_BASE,
for atttt(lan#+x3)*& ~3, s = wlrn;oiaif lths_ w
stals = wlslangth:
owSCRt!et  *
brcmb_w
sta_shm;
		_tu, M_lsN0_FRM_lYTESZ,nM_c_)hlanacp_}tmpec)bcn1v 40Mhs_ w
stal basprobasrdspwlsefi

ow bastemplchanregici*if tc*
brcmb_w
sta_templchaprom;
		_hwcfT_lsN1_TPL_BASE,
for atttt(lan#+x3)*& ~3, s = wlrn;oiaif lths_ w
stals = wlslangth:
owSCRt!et  *
brcmb_w
sta_shm;
		_tu, M_lsN1_FRM_lYTESZ,nM_c_)hlanacp_}ttmpec)timgoffsma !=s0; 40Mh*
brcmb_w
sta_shm;
		_tu, M_TIMBPOS_INBEACONthe se stimgoffsma +xD MB_shm_HDR_LENmaceM*
brcmb_w
sta_shm;
		_tu, M_DOT11_DTIMPERIOD,vdtimgperiod)upc}ute_tx);

*
brcmb_w
sta_shm;
		_tu, M_TIMBPOS_INBEACONthe se slan#+xD MB_shm_HDR_LENmaceM*
brcmb_w
sta_shm;
		_tu, M_DOT11_DTIMPERIOD,v0acp_}trate, index;
	c
	if (wlon cha_b = wl_hw(16 txfunfl[NFIFO];
#endicbsl""awlc)cnint_usk_t(st>*b = wlc uc_vtimgoffsmacbsl""alc)fgc_vdtimgperiodve been bandufl[NFms_dbg_maendi_hw ifosz_fix;efcnint_uus_b_
	brcm *cle)fifosz_ixrn;
	}

	; dfms_Hs_dbg_mab = wlel(f maitsse anges 	!/ bg32 both_vali _= MCMD_lsN0VLD | MCMD_lsN1VLDacstc)
Cprot,s_cbothcmemplchas a>bainause, Fixso sched._an   tacrupt
 ate    b bat w	 * caic _hw, b

ow b s	moutind/*
			 * BM(ns a_mctr32;}

	/*D11REGOFFS(macanmmif r; & both_vali )l s_both_vali )lths_ clear anycprevious s6TUS_#eet  *s a_w
sta32;}

	/*D11REGOFFS(macb

sATUS_o, MI_lsNTPL);0); banwat<   = wl_templchapvirgin; 40Mhwat<   = wl_templchapvirgint b_err(wl	m8 basic_b = wl_w
sta(ndicfb = wlc timgoffsmacvdtimgperiod,lmacicbsl""alc)fmacip2X_
s_ markfb = wl0fvali #eet  *s a_sma32;}

	/*D11REGOFFS(macanmmif r, MCMD_lsN0VLDp2X_cms_c_i;bey_txs_ Cprot, bat aftac schedulel(f con  tacruptcbothctrcms_
 ate    b emplchas a>bast, * busy. Fixouldclear  con  t. & remask/*
			 * BM(ns a_mctr32;}

	/*D11REGOFFS(macanmmif r; & both_vali )l s_both_vali ) 40Mhwat< defmacb

maskl|= MI_lsNTPL2X_cms_c_i;bey_txpec);(ns a_mctr32;}

	/*D11REGOFFS(macanmmif r; & MCMD_lsN0VLDp; 40Mh*
brcmc_b = wl_w
sta(ndicfb = wlc timgoffsmacvdtimgperiod,lmacicbsl""alc)fpppppp2X_cs_ markfb = wl0fvali #eet  *s a_sma32;}

	/*D11REGOFFS(macanmmif r, MCMD_lsN0VLDp2X_cms_c_i;bey_xpec);(ns a_mctr32;}

	/*D11REGOFFS(macanmmif r; & MCMD_lsN1VLDp; 40Mh*
brcmc_b = wl_w
sta(ndicfb = wlc timgoffsmacvdtimgperiod,bsl""alc)fppppp,fmacip2X_
s_ markfb = wl0fvali #eet  *s a_sma32;}

	/*D11REGOFFS(macanmmif r, MCMD_lsN1VLDp2X_cms_c_i;bey_xms_c_i;bthis shouUn cha aic _ = wls atic consys em.shof x;
	c
	if (wlon cha_b = wl(16 txfunfl[NFIFO];
#endi)s_df16 txfunfl[NFbsNFIf 	!bsNIf 	ifosz_fbsNIf ntstFix 
		ac parauplifs(bsNIf ->timi*r>band-S_TYPE_AP ||21		1_AC bsNIf ->timi*r>band-S_TYPE_ADHOCr; 40Mhs_ Clear  consofS_in
maskleet  wat< defmacb

maskl&= ~MI_lsNTPL2X_cpec);wat<   = wlo_funcs_c_i;be	
	if (wlon cha_b = wl_hw(ndicfwat<   = wlc
for atwat<   = wl_timgoffsmacbsl""atwat<   = wl_dtimgperiod)upc}bthix;
	c
	if (wl/radnew_b = wl(16 txfunfl[NFIFO];
#endi, cnint_usk_t(st>*  = wlc
forlc)fgc_vtimgoffsmacvgc_vdtimgperiodve be ban!b = wlo_fucs_c_i;be banwat<   = wlo_fudev_kfr_E_rkb_any(wat<   = wloac	wat<   = wls= _ = wlacstc)
 d) PLCP eet rkb_push(wat<   = wl/*| M_shm_HDR_LENmac	wat<   = wl_timgoffsma:
soimgoffsmaac	wat<   = wl_dtimgperiod =	dtimgperiodwl	s_c_set_on cha_b = wl( i );0thix;
	c
	if (wl/radnew_proba_rdsp(16 txfunfl[NFIFO];
#endicbsl""cnint_usk_t(st>*proba_rdspve be ban!proba_rdspvefucs_c_i;be banwat< proba_rdspvefudev_kfr_E_rkb_any(wat< proba_rdspvac	wat< proba_rdsp F)proba_rdspacstc)
 d) PLCP eet rkb_push(wat< proba_rdsp/*| M_shm_HDR_LENmac	s_c_set_on cha_proba_rdsp(ndicfappppp2Xthix;
	c
	if (wl (20le_proba_rdsp(16 txfunfl[NFIFO];
#endicl*/
	r (20le)s_dfms
 ateprev*		 Rne asas_c_sendie_sprobasrdspwlses by in_sel(f bastimeout
 ate
ow1, Ft can ouldBendeitvi dowat oimana[ *e./*
			 wat< prb_rdsp_timeout F) (20le ?b == shPRB_RESP_TIMEOUT : 1o crd 11bb_w
sta_shm;
		retu, M_PRshMAXTIME, wat< prb_rdsp_timeouto;);_a TODO:atic  (20le) => aisowdeacb mchanrec ivel(fbrcprobasrdque(ra			ratms_W
stalssid b

owshar->Bmem

ypbrce, index;
	
n_c_set_shmgssid_on (16 txfunfl[NFIFO];
#endi, 16 txfunfl[NFbsNFIf 	!cch)n beu8 *ssidptr  cIf ->SSID; f_c_vN_se_= M_SSID; f_8lssidt(s[hm.aifs < MAX_SSID_LEN]re
Cs_ thddel(f basssid f, (bzerS 		ifco/
eitvi 
owshmastatamssmatssidt(s,
0,whm.aifs < MAX_SSID_LENo;);amsc/
(ssidt(s,
ssidptr,cIf ->SSID_lan)up;
s_c_set_nepyto_shm;
		,vN_se, ssidt(s,
hm.aifs < MAX_SSID_LENo;);rd 11bb_w
sta_shm;
		retu, M_SSIDLEN, M_c_)hIf ->SSID_lan)uprate, index;
	
n_c_set_bsNFon cha_proba_rdsp(16 txfunfl[NFIFO];
#endicbsl"(wlc))cnint_unfl[NFbsNFIf 	!cchcbsl"(wlc))cnint_usk_t(st>*proba_rdspcbsl"(wlc))*/
	rsu= t don be, "wlanwl_flan  cmin_t(et f_t, proba_rdsprnlan, lsN_TMPL_LENmacbe bansu= t done	
	if (wlsu= t duaMA__ndewaitM i );0);s_ w
stal basprobasrdspwlsefi

ow bastemplchanregici*if t*
brcmb_w
sta_templchaprom;
		retu, l_PRshTPL_BASE,
for tttt(lan#+x3)*& ~3, proba_rdsprn;oiaif ltc)
w
stal baslangth:trcms_cprobasrdspwlsefa[ mba(+PLCP/-FCS)*if t*
brcmb_w
sta_shm;
		retu, M_PRB_RESP_FRM_LEN, M_c_)hlan)up;
c)
w
stal basSSID 		ifSSID langth:brcm8 basic_shmgssid_on (
		,vcch)cp);_adate W
stalPLCP hFF)w,s 		ifdur macis  maiprobasrdspwlsefa[ mbsdate at aic roidt. UstaTXPWacb al_a[ mbalangth:c	defedtbycowndate PLCP hFF)w, atic concaic 			b_c_set_	ca_prb_rt*dc_}
_Lbrcm()
 cmsby subtracsel(f basPLCP lan if  hddel(f basFCS./*
			 b_c_set_	ca_prb_rt*dc_}
_Lbrcm(ndicbsl""awlc) M_c_)lan#+xFCS_LEN -*| M_shm_HDR_LENmacbe bansu= t done	
	if (wl (20le_aMA( i );0thix;
	c
	if (wlon cha_proba_rdsp(16 txfunfl[NFIFO];
#endic)*/
	rsu= t don be16 txfunfl[NFbsNFIf 	!bsNIf 	ifosz_fbsNIf ntstc)
on cha AP ticIBSS probasrdspwlses if t banwat<  parauplifs(bsNIf ->timi*r>band-S_TYPE_AP ||21		1_AC bsNIf ->timi*r>band-S_TYPE_ADHOCr; 40Mhpec);wat< proba_rdspvefuncs_c_i;be	
	if (wlbsNFon cha_proba_rdsp(ndic)*sNIf , wat< proba_rdspcbsl"l"(wlc))cu= t do;wl}lne.;

heturn b_xmtfifo_sz_gmat16 txfunfl[NFms_dbg_maendi_hwcfu, "wfifo,21		1_Au, "w*blocksve be banai;
#>= Nt re	
x-ms_c_in-EhNVALntst*blocksfifosz_ixrnxmtfifo_sz[ei;
];ttxms_c_in0; ratx;
	
n_c_set_srad d)rma ch(16 txfunfl[NFIFO];
#endic), "waM ch_rdggoffsmacbsl   wlst u8 * d)rve beeturn b_srad d)rma ch(
		retu, aM ch_rdggoffsmac  d)rv;be banaM ch_rdggoffsma*r>bnd->gSSID_OFFSET, b
amsc/
(osz_fbsNIf ->gSSIDc  d)r, ETH_ALEN)upthis shouFla(l'scan    progres,' 			f, (holCsdynamictol);caiibr macishof x;
	c
	if (wlscan_sATrt(16 txfunfl[NFIFO];
#endi)s_df
		pr_liholC_on (
		rn if nopicfshm_HOLD_FOR_SCAN,fmacip2Xthix;
	c
	if (wl/can_sAop(16 txfunfl[NFIFO];
#endi)s_df
		pr_liholC_on (
		rn if nopicfshm_HOLD_FOR_SCAN,fappppp2Xthix;
	c
	if (wlassoci mm_on (16 txfunfl[NFIFO];
#endi, */
	rst_}
)s_df
		<  paraassoci mmd =	st_}
upthis shouWban a rem
ha STA/AP  s	mcmovedtbycMMAifs <, ticwhan i_ can oubloe_w, 	cceptshouA*PDU 
raafir,lohw,ebnfpendie_si dhs_dbg_mah	va 			baninvali  mmd sow batshouwhan lasac o dhs_dbg_mareleasts TXPm,cms_yvcan be	h		icmd= ppropri mmly./hof x;
	c
	if (wlinval_dma_ ktst16 txfunfl[NFms_dbg_maehwc
spi (wlc))cnint_ui%_EDCF_STsta *stac
spi (wlc))x;
	c(*dmaa_wlc_hw,_fn)on becnint_udma_ pa *dmahre , "wiwl enginiHY_0; i <vNt re; i
		 * whdmahHY_ixrn;i[i]2X_cpec)dmahH!= NULLvefundma_walk_ohw,ebn)dmahc)dmaa_wlc_hw,_fn, 16a);wl}lne.;

heturn fa
radcur if (16 txfunfl[NFIFO];
#endi)s_dfms_c_in
		rn if no if ms s;dmcs*/
	return fatx_flush_nemplemmd(16 txfunfl[NFIFO];
#endi)s_df, "wiwlspc)
Kick	DMA codaef  hnycpendie_sA*PDU e/a enginiHY_0; i <vARRAY_SIZE;
		return;i); i
		
	t banwat< turn;i[i]vefundma_kick_tx;ndi< turn;i[i]v;ttxms_c_in!b_c_setx ktpendtot( i );0thix;
	c
	if (wl/rad  = wl_lis}
n_  tacval(16 txfunfl[NFIFO];
#endicfg8   tacval)s_df
		<  cn_li_ cn  c2 tacval; t banwat<  paraupone	
	if (wlbcn_li_on ( i );0thig64bthe ha dtsf_gmat16 txfunfl[NFIFO];
#endi)s_dfu32 tsf_h,utsf_l;dfu64btsf;0);8 basib_mctr_tsf(
		retu, &tsf_l, &tsf_h);0);isfa= tsf_h;);isfa<<= 32;);isfa|=utsf_l;dtxms_c_intsf;0thix;
	c
	if (wltsf_smat16 txfunfl[NFIFO];
#endi, u64btsf)s_dfu32 tsf_h,utsf_l;d	 b_c_set_time_lockM i );0);tsf_l =ntsf;0ntsf_h  butsfa>> 32p; dfms_mctr ineftsfatimer 	/*,w ban ate o
owgMS_an atomictmctr if t*s a_w
sta32;
		return;
	}

	/sD11REGOFFS(tsf_timer	/*),utsf_lo;);rs a_w
sta32;
		return;
	}

	/sD11REGOFFS(tsf_timerate ),utsf_h);d	 b_c_set_time_unlockM i );0ne.;

heturn fasradtxppower(16 txfunfl[NFIFO];
#endic), "wtx wron beu;

	qdbm; dfms_Rcmove 	default:bit 		ifclipt
owwlxsqdbm_(100 -			  dbm_ cmin_t(u;

,wtx wrhouand-S_TXPWR_DB_FACTOR, 0xff););ms_c_in
		pr_litx ower_smatch e_ if nopi,sqdbm,fappppp2Xthi;

heturn fa
radtxppower(16 txfunfl[NFIFO];
#endion beu;

	qdbm; 	*/
	r	default; df
		pr_litx ower_gmatch e_ if nopi, &qdbm,f&	default); dfms_Rc_c_in dbm_ms ss if tms_c_inM;

)( dbm_/uand-S_TXPWR_DB_FACTORowlratms_Prot !=urec ivedfa[ *es if s shouRs_c_inmaci Fixmle)fa[ *es neeifSS b_cproces, d.fapppp
oc_rawise./ho Pa[ * 'bimad'   dic tes wlx. #fa[ *es SS proces, b_fle)fu
eak:o	 .shof it is sx;
	c
	if (wlmccv(16 txfunfl[NFIFO];
#endi, cnint_usk_t(st>*pon becnint_ud
	rxhdr#erxh;efcnint_u2%_EDCF_SThdr#ehwl
u, "wlanre */
	ris_amsdS;e
c_axa[ *e*sATrtopw, ( rxhdr#eteccxh  bu16 txfu;
	rxhdr#e)h(orn;oiaif ltc)
16 iptoff rxhdr#etecrkb_pull(p/*and-S_HWRXOFFif ltc)
MAC ilsertop2 thdcbytesvengia4 hFF)w,s ngiQoS ngiA-MSDU suba[ *es if mpec)cxh->RxSATUS_1*& RXS_PBPRES; 40Mhpec)prnlan < 2)a{ whM*
brcmc));
		return;
	}

	/bsl""aw"Switchmccvchmcvd ru		 of lan %dld the se swat<  paraus strprnlanacp_s-go
ow osswl

	wlcrkb_pull(p/*2acp_}ttmh  bu16 txfu2%_EDCF_SThdr#e)(orn;oia#+xD M_shm_HDR_LENmac	lan  cprnlan; tmpec)cxh->RxSATUS_1*& RXS_FCSERRox);

pec);(wat< filtac_ags & &st r_FCSFAILt)t u;go
ow osswl
y_txs_ _proturec ivedf kt	h	s at leastaa[ mbant filt abelCsd(wlpec)lan < | M_shm_HDR_LEN#+wrt fromh->a[ *e_nt filtt)t ugo
ow osswl

s_ notdBRtrorsel(fA-MSDU d(wlps_amsdS  ccxh->RxSATUS_2*& RXS_AMSDU_c_bsup
pec)isgamsdS)t ugo
ow osswl

n_c_set_mccvctl(ndicfcxhcfp););ms_c_iwl
w oss:

n_c_u_ kt_t(s_fr_E_rkb(powlratms_Prot !=urec ivedfa[ *es if s shouRs_c_inmaci Fixmle)fa[ *es neeifSS b_cproces, d.fapppp
oc_rawise./ho Pa[ * 'bimad'   dic tes wlx. #fa[ *es SS proces, b_fle)fu
eak:o	 .shof it is s*/
	
n_c_seb_mccv(16 txfunfl[NFms_dbg_maendi_hwcfu, "wfifo,l*/
	rbimadon becnint_usk_t(st>*p;becnint_usk_t(st>*next = NULL;becnint_usk_t(stdhFF)hmccv_a[ *eswl

u;

	ns= fo cu;

	bimad_limiss= _imad ? RXBND : -1re */
	rmle)pendie_s b_err(wl;
rkb_ ++++_hFF)FO]itx&rccv_a[ *esif ltc)
gac_raurec ivedfa[ *es if 	do 40Mhs_ !g ma
oc_rasbsome oimanSS run!s!et   bann#>= bimad_limis)t u;8 eakel b
ale)pendie_s bdma_rx(osz_ixrn;i[ei;
], &rccv_a[ *esif 		n++upc}uwhile (ale)pendie_if ltc)
pos}dm

	 rt(ss if 	dma_rxa	 *(osz_ixrn;i[ei;
]if ltc)
proces,  = h a[ *e*brcmrkb_ ++++_walk_saf+ &rccv_a[ *escfp, nextox);

16 txfu;
	rxhdr_le erxh_le;;

16 txfu;
	rxhdr#erxh;ewlcrkb_unlink(p, &rccv_a[ *esif 		rxh_le  bu16 txfu;
	rxhdr_le e)prn;oia;0	tcxh  bu16 txfu;
	rxhdr#e)prn;oia;0);fms_a	xup,rx hFF)w, endineces_#eet  cxh->RxF[ *eSt f  cle16_to_cpu(rxh_le->RxF[ *eSt fif 		rxh->PhyRxSATUS__0l cle16_to_cpu(rxh_le->PhyRxSATUS__0if 		rxh->PhyRxSATUS__1l cle16_to_cpu(rxh_le->PhyRxSATUS__1if 		rxh->PhyRxSATUS__2l cle16_to_cpu(rxh_le->PhyRxSATUS__2if 		rxh->PhyRxSATUS__3l cle16_to_cpu(rxh_le->PhyRxSATUS__3if 		rxh->PhyRxSATUS__4l cle16_to_cpu(rxh_le->PhyRxSATUS__4if 		rxh->PhyRxSATUS__5l cle16_to_cpu(rxh_le->PhyRxSATUS__5if 		rxh->RxSATUS_1* cle16_to_cpu(rxh_le->RxSATUS_1if 		rxh->RxSATUS_2l cle16_to_cpu(rxh_le->RxSATUS_2if 		rxh->RxTSFTime  cle16_to_cpu(rxh_le->RxTSFTimeif 		rxh->RxC ne  cle16_to_cpu(rxh_le->RxC ne);0);
n_c_set_mccv(osz_ixrnndicfpacp_}ttmms_c_inale)pendie_wlratms_sent d-leveln  tacruptcproces,el(
ate  Rs_c_inmaci Fixanoc_ra dpc neeisl			basrd-schedul d.fapppp
oc_rawise./ho   Pa[ * 'bimaded'   dic tes Fixapplic 0le loopsbseculCsbasbimade|./#eet*/
	return fadpc(16 txfunfl[NFIFO];
#endi, */
	rbimade|)s_dfu32 macb

sATUS_;mb16 txfunfl[NFms_dbg_maendi_hw ifosz_fix;efcnint_uus_b_
	brcm *cle)fifosz_ixrn;
	}

	; dfpec)bfl[NF
	brcmmcmovedM i ); 40Mh*
brcmc));}

	/s"Switch bchdFF)hchipet trosz_ixrnus st
 se suct */
	acp_sbfl[NF
own;
		rewlp2X_cms_c_ib_err(wl	y_txs_ grab 		ifclear  consavmd softwa>bain
sATUS_ 
its stataacb

sATUS_ ifosz_fmacb

sATUS_;mbosz_fmacb

sATUS_  core
 bfl[NF
bg_  t;}

	/s"Switchmacb

sATUS_ 0x%xld the (wlc))osz_ixrnus sthmacb

sATUS_);0);WARN_ON macb

sATUS_ & MI_PRQ); ms_PRQ I tacruptcinwnon-MBSS *rctxs_ tx s6TUS_#eet  banaMcb

sATUS_ & MI_TFS; 40Mh*/
	r_etal2X_cpec)n_c_seb_txe;TUS_;ndiretu, bimade|, &_etalt)t u;osz_fmacb

sATUS_ |= MI_TFS2X_cpec)_etalta{ whM*
brcmc));}

	/s"MI_TFS:b_etalet acp_sugo
ow_etal2X_c_phy_txt ||aMcb

sATUS_ & (MI_TBTT | MI_DTIM_TBTT)one	
	if (wltbttM i );0);s_ ATIM w  dow end#eet  banaMcb

sATUS_ & MI_ATIMWINEND; 40Mh*
brcm
bg_  fo;}

	/s"end#of ATIM w  dowet acp_s*s a_sma32;}

	/*D11REGOFFS(macanmmif r, osz_fqvali );t  wat< qvali _= 0wl	y_txs_
 cmsrec ivedf;oia#orent filt a[ *e,pMI_DMAINT	in
	atei dic taci*,stRX_t re   tacrupt
 atet  banaMcb

sATUS_ & MI_DMAINT)X_cpec)n_c_seb_mccv(osz_ix,tRX_t re, bimade|t)t u;osz_fmacb

sATUS_ |= MI_DMAINTo 

s_ noise sampleentllecMedttet  banaMcb

sATUS_ & MI_BG_NOISE)t  watpr_linoise_sample_  tr(osz_ixrn if nopif;_txt ||aMcb

sATUS_ & MI_GP0; 40Mh*
brcmc));}

	/s"SwitchPSM microce aswM chdog_a	fedtat %d "
 se s"(sent ds). Red-_sel(.et trosz_ixrnus strosz_ixrnouw)re
  prb

k_once("%s chPSM WM chdog,hchipi _0x%x,hchiprev 0x%xld the  (wlcuct */
	t aia
radchip_id(osz_ixrnsih)the  (wlcaia
radchiprev(osz_ixrnsih)acp_sbfl[NF_etalmc))or(osz_ixrnndirewlp2X_y_txs_ gptimer timeout eet  banaMcb

sATUS_ & MI_Te	
x-*s a_w
sta32;}

	/*D11REGOFFS(gptimer)cf0);a_f banaMcb

sATUS_ & MI_RFDISABLE) 40Mh*
brcm
bg_  fo;}

	/s"SwitchBMAC DececMedtawc negetonc co"
spi (wlc))" RF Dis 0le Inputet trosz_ixrnus sacp_sbfl[NFrfk	 *asradhw_sATUe(ndirewlp2X_y_txs_ BCNstemplchanin apec) 0le tet  banaMcb

sATUS_ & MI_BsNTPL)be	
	if (wlon cha_b = wlM i );0);s_ it	inn't ds * 		ifneeisl			basrdsched FixmMcb

sATUS_ issoun-zerS if tms_c_inosz_fmacb

sATUS_ ! core
w_etal:
sbfl[NF_etalmc))or(osz_ixrnndirewlp2X_ms_c_inosz_fmacb

sATUS_ ! corethix;
	c
	if (wlO]itx16 txfunfl[NFIFO];
#endi, */
	rmu w_txve been bandus_b_
	brcm *cle)fifoszreturn;
	}

	;efcnint_u2%_EDCF_STc necess*chfifoszre parab%_E_hw->ange.c nedee.c ne; f_c_vio chansre
 bfl[NF
bg_  fo;}

	/s"Switet trosz<  paraus s); dfc net*wl  cch20mhzTc t*wl(ch_fix_(100 );d	 b_c_seblO]itxndiretu, c net*wloo 

s_ un cha s = wlslis}
n   tacval*brcm8 basic_bcn_li_on ( i );0;
c)
w
stalec_ranMS_addres,t
ow}

e:brcm8 basic_set_aMA( i _fbsNIf mac	s_c_set_/rad ssid( i _fbsNIf maccms_ Un cha rsf_cfprep Fixassoci mmd 		ifon if t banwat<  paraassoci mmd ifswat< pparaupox);

u32 bi;0);fms_gMS_s = wlsperiod 		ifconver"wto uS#eet  *i	ifosz_fbsNIf ->at
 *		d ss<   = wl_period << c02X_
s_
l"aanon cha acoceli]it path wculCsred-t rat!ito zhytxcha(100 /*at!et  *s a_w
sta32;}

	/*D11REGOFFS(rsf_cfprep)the  (wlc bi << CFPREP_CBI_orIFT)om0Mhs_ Un cha macan filt PMarel mmd 
its stat	s_c_set_/radpsettrl( i );0_y_tx8 basic_bandieit_ordefed(
		,vc net*wloo 

s_ i]it probasrdspwlseftimeout eet rd 11bb_w
sta_shm;
		retu, M_PRshMAXTIME, wat< prb_rdsp_timeouto;)

s_ i]it wlxsburst rxopdna[ *eburstie_i eet rd 11bb_w
sta_shm;
		retu, M_MBURST_TXOPthe (wlc));
		rehe (wlc)) _rtfs ? (EDCF_AC_VO_TXOP_AP << 5) chMAXFRAMEBURST_TXOP)o;)

s_ i]itializa maximum aicowedf;uty cycle:brcm8 basic_;uty_cycle_smatch , wat< tx_;uty_cycle_ofdm,lmacicfmacip2X_8 basic_;uty_cycle_smatch , wat< tx_;uty_cycle_cck,fppppp,fmacip2Xtxs_
 cmsUn cha some shar->Bmem

yplo_wtacisarel mmd t

 cmswlxsA*PDU rt f aicowedfSS rec ived/*
			 b_c_set_ampdS_shmgon (
		rnampdSoo 

s_  if nt*wlifnde, its stat
	if (wlbsO]itxndioo 

s_ E(20le EDCF 	casc(while  conMACfisdBu= t ded) if t*s a_sma16;}

	/*D11REGOFFS(tfs
ctl)cfIFS_USEEDCFp2X_8 basic_edcf_smapac_ms(ndicfappppp2Xdfms_mctr inefRne asdefsaci*if we hava notdyet ds * so if t banwat< Rne a_rev  s_c_ __fuuc_vrev;_fuuc_vpM chom0Mhrev  cn_c_seb_mcad_shm;
		retu, M_lOM_REVhMAJORowl  pM ch  cn_c_seb_mcad_shm;
		retu, M_lOM_REVhMINORowl  wat< Rne a_rev  anM_v << NBITSM_c_)p | pM chom		snprb

f;
		rewio_l->fx_(efsacithe  (rt from
		rewio_l->fx_(efsaci)/s" u.%u",smsv, pM chp2X_y_txs_ ..ouw M_ally unleash hella(aicow  conMACfo	  ofdsu= t do stat
	if (wl (20le_aMA( i );0ltc)
1u= t d wlc-tx ei;
s 		ifmu woiXPWol); maipreism cac oimantet  banau w_txveu;8 basib_mu w;
		retucfmacip2Xtxs_) (20le iXPWRF Dis 0le Del yatimer if t*s a_w
sta32;}

	/*D11REGOFFS(rfdis 0ledly)/sRFDISABLE_DEFAULTp2Xtxs_
 cmsI]itializa WME pac_mecers;aticms_yvhavan't bean setsby some oc_ra
 cmswec neism (IOVar,lecc)w ban mctr inem as_c_inefms_dbg_m./*
			 * BMGFIELDm
		rewma_refii+st0]cfEDCF_ortsl)l s_c_ __fu_a Uni]itializad;	mctr as_c_HWs!et   		 asre
  enginal  cor as <
hm.aifs < NUM_ACSr as
		
	t	
		rewma_refii+stac]sFbsl"aifs	_c_seb_mcad_shm;
		retu, M_AC_TXLMT_ADDRnal));wl}lne./ofo *T_raanmmci*drivw, enfiyWmoutind. E))or ne asbseculCsbasus queshof it txfunfl[NFIFO];
#e
n_c_set_attach(16 txfunfl[NFO];
#end, 16 txfuns_b_
	brcm *cle)cfu, "wus st
  (wlc))*/
	rpio	cascfu, "w*perron be16 txfunfl[NFIFO];
#endio cu;

	errs= fo cu;

	i, j;mb16 txfunfl[NF pa * paacstc)
 llo_wte 16 txfunfl[NFIFO];
#sATUes		ifits subd6 txfures if 	ndi  c
	if (wlattach_aMllo_(us str&err, 0twlt banndi  = NULLvefugo
ow_eil;mbosz_fwio_l	ifos_fwio_l;mb pa ifoszre paacs# bazhyindd(DEBUG)df
		pO];
F
bg ifosz;
#endif
df
		<  _PCs=n
		rn if sATUet0];mbosz_fcle)fifoszrecle)st_}
upbosz_fws(s)
	;mb paraus s(s)ms s;db para_pio	cas  cpio	cas;df
		<  _PCieit_pendie_s b_err(wlhwat<   = wl_templchapvirgint bmacint
Cs_ topulchan16 txfunfl[NFIFO];
#w, ( zhytxcha(100 s  stat
	if (wlO];
FO]itxndi, us s); dfs_ un cha st_/aparel mmd pac_mecers
			 b_c_set_ap_on ( i );0;
c)
 cmscow levelnattach steps(aic hw 	ccessesvgo
	atei sult, oubm

	 in mcs>ctrcms_cattach)/*
			 errs= 	_c_seb_attach(
		,vcle)cfus strpio	castwlt banerronfugo
ow_eil;m	 b_c_set_protecMaci_on (
		,v == shPROT_N_PAM_OVR, OFFif lt parar_li11ncap 0le >band-S_Phm(11N_CAPewat<  if r; dfs_ dis 0le aicowedf;uty cycle:brcmwat< tx_;uty_cycle_ofdms= fo cwat< tx_;uty_cycle_cck  core
 bfl[NFldstfpr_lic nina_wlc( i );0ltc)
txc nin 1: rxant
0,wtxc nin 2: rxant
1
			 * BMand-S_ISNPhmewat<  if rlifs(w		c-rs.cotxe; i++n *>b1))l  wat< rs.cotxant
=c
		c-rs.coix_txc nin - 1nt
Cs_ tus o
owBMAC drivw, brcmwatpr_listfpc ninaO]itxndire if nopi, 
		c-rs.coix_txc ninc
spi (wlc))
		c-rs.coix_rxc ninif ltc)
puic on some O];
#mcsxchel(f s_c_inefcow attach e/a enginiHY_0; i <vNt re; i
		l  wat< cle)counapec)ti]*ifoszreturnunapec)ti]; tmamsc/
(uw		reperm_ec_ra d)r, &oszreturnec_ra d)r, ETH_ALEN)upmamsc/
(u paraat
_ec_ra d)r, &oszreturnec_ra d)r, ETH_ALEN)upa enginjHY_0; j <vwat< ppara_n if s; j
		 * wh
		<  _PCs=n
		rn if sATUetj]o t  t ||!
	if (wlattach_stfpant_O]itxndiota{ whMerrs= 24cp_sugo
ow_eil2X_c_p_fu_a zhytxchaan fenfaci*w  dows rt f limissleet  wat<  if noCWmint bAshm_CWMIN;t  wat<  if noCWmaxs= shm_CWMAXom0Mhs_ i]it g	casc(100 -			 e banwat<  if no if timi*r>band-Tgmode2Gta{ whMwat<  if nog	cas  cGMODE_AUTO; whM*
brcmt_protecMaci_on (
		,v == shPROT_G_USERthe set sshm ac if nog	cas)2X_c_p_fu_a i]it _nl (20dBRtrors->Bmcas 			 e ban == shPhm(11N_CAPewat<  if r; 40Mhb para_nl (20d= SUPPtsl_11N; whM*
brcmt_protecMaci_on (
		,v == shPROT_N_USERthe sett ss(( para_nl (20d==he sett ss  SUPPtsl_11N) ? WL(11N_2x2 :he sett ss WL(11N_3x3))2X_c_p_fu_a i]it per- _PCszhytxchacoids	/,szh t d wlsN_f nog	cas stat	s_c_sezhytxch_coids	/(
		,v&hm ac if nozhycoids	/)om0Mhs_ a	 * inwix_roids	/ stat	s_c_set_coids	/_filtac(&hm ac if nozhycoids	/cbsl""awl&hm ac if noix_roids	/,fppppp,bsl""awl == shrcmsS_CCK_OFDM,  == shrcms_c_bs,bsl""awl(*/
	)anwat<  para_nl (20d& SUPPtsl_11N)p2X_y_txs_
"aanon cha RAit(wlcanges 	due t

 cmswat< rs.cotxant/txc nin/ant_rx_ovrwc nege/*
			 b_c_set_stfpr_lirxant_on (ndiif ltc)
attach  = h m		coes if 	errs= 	_c_sewlattach_a		coe( i );0_ banerr !=s0;nfugo
ow_eil;m	  ban!b_c_set_timersFO]itxndi, us s)	 * wh
io_lmc));
	rewio_l, "Switch bchieit_timer _eiletet trus st
 se suct */
	acp_serrs= 32;);ugo
ow_eil2X_y_txs_ zh t d wlsroids	/,fg	cas statwat< cmi  cs;
	uit_n neces_mgr_attach(
		);0_ ban!wat< cmi	 * wh
io_lmc));
	rewio_l, "Switch bchn neces_mgr_attach _eilet"
 se s"et trus stsuct */
	acp_serrs= 33;);ugo
ow_eil2X_y_txs_ i]it zhytxchawhan aic pac_mecers
g_mareadl, i.d. < c_}
s	/ stat
	if (wlbsNFzhytxch_O]itxndioo 

s_
 cmsCemplemm_inefndi zhytxchasATUesi]itializwtacis../*
			lhwat<  sNIf ->ndi  cosz;
mbosz_fmimofS_= FT_HT;t osz_fmimo_40txbwt bAUTO; wosz_fofdm_40txbwt bAUTO; wosz_fcck_40txbwt bAUTO; w
	if (wlon cha_mimo_N_ndebwcap(
		,v == shN_BW_20IN2G_40IN5Gif ltc)
Set zhytxcha(100 s trcSGI
			 * BMand-S_SGI_CAP_Phmewatp; 40Mh*
brcmc_htlon cha_sgi_rx(osz,BMand-S_N_SGI_20 |
 sett ss    and-S_N_SGI_40)p2X_yute_tatic and-S_ISSSLPNPhmewat<  if r; 40Mh*
brcmc_htlon cha_sgi_rx(osz,BMand-S_N_SGI_20 |
 sett ss    and-S_N_SGI_40)p2X_yute_ta40Mh*
brcmc_htlon cha_sgi_rx(osz,B0);0_y_tx8 basib_antsel_smatch e_tu, 
		rnasi< RAisel_apec));a_f banperronfu*perr  core
 ms_c_inoszre
w_eil:
s
io_lmc));
	rewio_l, "Switch bch_eilet#w, ( err %dld