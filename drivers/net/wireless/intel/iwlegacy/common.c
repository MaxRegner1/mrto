/******************************************************************************
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2008 - 2011 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * Contact Information:
 *  Intel Linux Wireless <ilw@linux.intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *****************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/etherdevice.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/lockdep.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/skbuff.h>
#include <net/mac80211.h>

#include "common.h"

int
_il_poll_bit(struct il_priv *il, u32 addr, u32 bits, u32 mask, int timeout)
{
	const int interval = 10; /* microseconds */
	int t = 0;

	do {
		if ((_il_rd(il, addr) & mask) == (bits & mask))
			return t;
		udelay(interval);
		t += interval;
	} while (t < timeout);

	return -ETIMEDOUT;
}
EXPORT_SYMBOL(_il_poll_bit);

void
il_set_bit(struct il_priv *p, u32 r, u32 m)
{
	unsigned long reg_flags;

	spin_lock_irqsave(&p->reg_lock, reg_flags);
	_il_set_bit(p, r, m);
	spin_unlock_irqrestore(&p->reg_lock, reg_flags);
}
EXPORT_SYMBOL(il_set_bit);

void
il_clear_bit(struct il_priv *p, u32 r, u32 m)
{
	unsigned long reg_flags;

	spin_lock_irqsave(&p->reg_lock, reg_flags);
	_il_clear_bit(p, r, m);
	spin_unlock_irqrestore(&p->reg_lock, reg_flags);
}
EXPORT_SYMBOL(il_clear_bit);

bool
_il_grab_nic_access(struct il_priv *il)
{
	int ret;
	u32 val;

	/* this bit wakes up the NIC */
	_il_set_bit(il, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);

	/*
	 * These bits say the device is running, and should keep running for
	 * at least a short while (at least as long as MAC_ACCESS_REQ stays 1),
	 * but they do not indicate that embedded SRAM is restored yet;
	 * 3945 and 4965 have volatile SRAM, and must save/restore contents
	 * to/from host DRAM when sleeping/waking for power-saving.
	 * Each direction takes approximately 1/4 millisecond; with this
	 * overhead, it's a good idea to grab and hold MAC_ACCESS_REQUEST if a
	 * series of register accesses are expected (e.g. reading Event Log),
	 * to keep device from sleeping.
	 *
	 * CSR_UCODE_DRV_GP1 register bit MAC_SLEEP == 0 indicates that
	 * SRAM is okay/restored.  We don't check that here because this call
	 * is just for hardware register access; but GP1 MAC_SLEEP check is a
	 * good idea before accessing 3945/4965 SRAM (e.g. reading Event Log).
	 *
	 */
	ret =
	    _il_poll_bit(il, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_VAL_MAC_ACCESS_EN,
			 (CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY |
			  CSR_GP_CNTRL_REG_FLAG_GOING_TO_SLEEP), 15000);
	if (unlikely(ret < 0)) {
		val = _il_rd(il, CSR_GP_CNTRL);
		WARN_ONCE(1, "Timeout waiting for ucode processor access "
			     "(CSR_GP_CNTRL 0x%08x)\n", val);
		_il_wr(il, CSR_RESET, CSR_RESET_REG_FLAG_FORCE_NMI);
		return false;
	}

	return true;
}
EXPORT_SYMBOL_GPL(_il_grab_nic_access);

int
il_poll_bit(struct il_priv *il, u32 addr, u32 mask, int timeout)
{
	const int interval = 10; /* microseconds */
	int t = 0;

	do {
		if ((il_rd(il, addr) & mask) == mask)
			return t;
		udelay(interval);
		t += interval;
	} while (t < timeout);

	return -ETIMEDOUT;
}
EXPORT_SYMBOL(il_poll_bit);

u32
il_rd_prph(struct il_priv *il, u32 reg)
{
	unsigned long reg_flags;
	u32 val;

	spin_lock_irqsave(&il->reg_lock, reg_flags);
	_il_grab_nic_access(il);
	val = _il_rd_prph(il, reg);
	_il_release_nic_access(il);
	spin_unlock_irqrestore(&il->reg_lock, reg_flags);
	return val;
}
EXPORT_SYMBOL(il_rd_prph);

void
il_wr_prph(struct il_priv *il, u32 addr, u32 val)
{
	unsigned long reg_flags;

	spin_lock_irqsave(&il->reg_lock, reg_flags);
	if (likely(_il_grab_nic_access(il))) {
		_il_wr_prph(il, addr, val);
		_il_release_nic_access(il);
	}
	spin_unlock_irqrestore(&il->reg_lock, reg_flags);
}
EXPORT_SYMBOL(il_wr_prph);

u32
il_read_targ_mem(struct il_priv *il, u32 addr)
{
	unsigned long reg_flags;
	u32 value;

	spin_lock_irqsave(&il->reg_lock, reg_flags);
	_il_grab_nic_access(il);

	_il_wr(il, HBUS_TARG_MEM_RADDR, addr);
	value = _il_rd(il, HBUS_TARG_MEM_RDAT);

	_il_release_nic_access(il);
	spin_unlock_irqrestore(&il->reg_lock, reg_flags);
	return value;
}
EXPORT_SYMBOL(il_read_targ_mem);

void
il_write_targ_mem(struct il_priv *il, u32 addr, u32 val)
{
	unsigned long reg_flags;

	spin_lock_irqsave(&il->reg_lock, reg_flags);
	if (likely(_il_grab_nic_access(il))) {
		_il_wr(il, HBUS_TARG_MEM_WADDR, addr);
		_il_wr(il, HBUS_TARG_MEM_WDAT, val);
		_il_release_nic_access(il);
	}
	spin_unlock_irqrestore(&il->reg_lock, reg_flags);
}
EXPORT_SYMBOL(il_write_targ_mem);

const char *
il_get_cmd_string(u8 cmd)
{
	switch (cmd) {
		IL_CMD(N_ALIVE);
		IL_CMD(N_ERROR);
		IL_CMD(C_RXON);
		IL_CMD(C_RXON_ASSOC);
		IL_CMD(C_QOS_PARAM);
		IL_CMD(C_RXON_TIMING);
		IL_CMD(C_ADD_STA);
		IL_CMD(C_REM_STA);
		IL_CMD(C_WEPKEY);
		IL_CMD(N_3945_RX);
		IL_CMD(C_TX);
		IL_CMD(C_RATE_SCALE);
		IL_CMD(C_LEDS);
		IL_CMD(C_TX_LINK_QUALITY_CMD);
		IL_CMD(C_CHANNEL_SWITCH);
		IL_CMD(N_CHANNEL_SWITCH);
		IL_CMD(C_SPECTRUM_MEASUREMENT);
		IL_CMD(N_SPECTRUM_MEASUREMENT);
		IL_CMD(C_POWER_TBL);
		IL_CMD(N_PM_SLEEP);
		IL_CMD(N_PM_DEBUG_STATS);
		IL_CMD(C_SCAN);
		IL_CMD(C_SCAN_ABORT);
		IL_CMD(N_SCAN_START);
		IL_CMD(N_SCAN_RESULTS);
		IL_CMD(N_SCAN_COMPLETE);
		IL_CMD(N_BEACON);
		IL_CMD(C_TX_BEACON);
		IL_CMD(C_TX_PWR_TBL);
		IL_CMD(C_BT_CONFIG);
		IL_CMD(C_STATS);
		IL_CMD(N_STATS);
		IL_CMD(N_CARD_STATE);
		IL_CMD(N_MISSED_BEACONS);
		IL_CMD(C_CT_KILL_CONFIG);
		IL_CMD(C_SENSITIVITY);
		IL_CMD(C_PHY_CALIBRATION);
		IL_CMD(N_RX_PHY);
		IL_CMD(N_RX_MPDU);
		IL_CMD(N_RX);
		IL_CMD(N_COMPRESSED_BA);
	default:
		return "UNKNOWN";

	}
}
EXPORT_SYMBOL(il_get_cmd_string);

#define HOST_COMPLETE_TIMEOUT (HZ / 2)

static void
il_generic_cmd_callback(struct il_priv *il, struct il_device_cmd *cmd,
			struct il_rx_pkt *pkt)
{
	if (pkt->hdr.flags & IL_CMD_FAILED_MSK) {
		IL_ERR("Bad return from %s (0x%08X)\n",
		       il_get_cmd_string(cmd->hdr.cmd), pkt->hdr.flags);
		return;
	}
#ifdef CONFIG_IWLEGACY_DEBUG
	switch (cmd->hdr.cmd) {
	case C_TX_LINK_QUALITY_CMD:
	case C_SENSITIVITY:
		D_HC_DUMP("back from %s (0x%08X)\n",
			  il_get_cmd_string(cmd->hdr.cmd), pkt->hdr.flags);
		break;
	default:
		D_HC("back from %s (0x%08X)\n", il_get_cmd_string(cmd->hdr.cmd),
		     pkt->hdr.flags);
	}
#endif
}

static int
il_send_cmd_async(struct il_priv *il, struct il_host_cmd *cmd)
{
	int ret;

	BUG_ON(!(cmd->flags & CMD_ASYNC));

	/* An asynchronous command can not expect an SKB to be set. */
	BUG_ON(cmd->flags & CMD_WANT_SKB);

	/* Assign a generic callback if one is not provided */
	if (!cmd->callback)
		cmd->callback = il_generic_cmd_callback;

	if (test_bit(S_EXIT_PENDING, &il->status))
		return -EBUSY;

	ret = il_enqueue_hcmd(il, cmd);
	if (ret < 0) {
		IL_ERR("Error sending %s: enqueue_hcmd failed: %d\n",
		       il_get_cmd_string(cmd->id), ret);
		return ret;
	}
	return 0;
}

int
il_send_cmd_sync(struct il_priv *il, struct il_host_cmd *cmd)
{
	int cmd_idx;
	int ret;

	lockdep_assert_held(&il->mutex);

	BUG_ON(cmd->flags & CMD_ASYNC);

	/* A synchronous command can not have a callback set. */
	BUG_ON(cmd->callback);

	D_INFO("Attempting to send sync command %s\n",
	       il_get_cmd_string(cmd->id));

	set_bit(S_HCMD_ACTIVE, &il->status);
	D_INFO("Setting HCMD_ACTIVE for command %s\n",
	       il_get_cmd_string(cmd->id));

	cmd_idx = il_enqueue_hcmd(il, cmd);
	if (cmd_idx < 0) {
		ret = cmd_idx;
		IL_ERR("Error sending %s: enqueue_hcmd failed: %d\n",
		       il_get_cmd_string(cmd->id), ret);
		goto out;
	}

	ret = wait_event_timeout(il->wait_command_queue,
				 !test_bit(S_HCMD_ACTIVE, &il->status),
				 HOST_COMPLETE_TIMEOUT);
	if (!ret) {
		if (test_bit(S_HCMD_ACTIVE, &il->status)) {
			IL_ERR("Error sending %s: time out after %dms.\n",
			       il_get_cmd_string(cmd->id),
			       jiffies_to_msecs(HOST_COMPLETE_TIMEOUT));

			clear_bit(S_HCMD_ACTIVE, &il->status);
			D_INFO("Clearing HCMD_ACTIVE for command %s\n",
			       il_get_cmd_string(cmd->id));
			ret = -ETIMEDOUT;
			goto cancel;
		}
	}

	if (test_bit(S_RFKILL, &il->status)) {
		IL_ERR("Command %s aborted: RF KILL Switch\n",
		       il_get_cmd_string(cmd->id));
		ret = -ECANCELED;
		goto fail;
	}
	if (test_bit(S_FW_ERROR, &il->status)) {
		IL_ERR("Command %s failed: FW Error\n",
		       il_get_cmd_string(cmd->id));
		ret = -EIO;
		goto fail;
	}
	if ((cmd->flags & CMD_WANT_SKB) && !cmd->reply_page) {
		IL_ERR("Error: Response NULL in '%s'\n",
		       il_get_cmd_string(cmd->id));
		ret = -EIO;
		goto cancel;
	}

	ret = 0;
	goto out;

cancel:
	if (cmd->flags & CMD_WANT_SKB) {
		/*
		 * Cancel the CMD_WANT_SKB flag for the cmd in the
		 * TX cmd queue. Otherwise in case the cmd comes
		 * in later, it will possibly set an invalid
		 * address (cmd->meta.source).
		 */
		il->txq[il->cmd_queue].meta[cmd_idx].flags &= ~CMD_WANT_SKB;
	}
fail:
	if (cmd->reply_page) {
		il_free_pages(il, cmd->reply_page);
		cmd->reply_page = 0;
	}
out:
	return ret;
}
EXPORT_SYMBOL(il_send_cmd_sync);

int
il_send_cmd(struct il_priv *il, struct il_host_cmd *cmd)
{
	if (cmd->flags & CMD_ASYNC)
		return il_send_cmd_async(il, cmd);

	return il_send_cmd_sync(il, cmd);
}
EXPORT_SYMBOL(il_send_cmd);

int
il_send_cmd_pdu(struct il_priv *il, u8 id, u16 len, const void *data)
{
	struct il_host_cmd cmd = {
		.id = id,
		.len = len,
		.data = data,
	};

	return il_send_cmd_sync(il, &cmd);
}
EXPORT_SYMBOL(il_send_cmd_pdu);

int
il_send_cmd_pdu_async(struct il_priv *il, u8 id, u16 len, const void *data,
		      void (*callback) (struct il_priv *il,
					struct il_device_cmd *cmd,
					struct il_rx_pkt *pkt))
{
	struct il_host_cmd cmd = {
		.id = id,
		.len = len,
		.data = data,
	};

	cmd.flags |= CMD_ASYNC;
	cmd.callback = callback;

	return il_send_cmd_async(il, &cmd);
}
EXPORT_SYMBOL(il_send_cmd_pdu_async);

/* default: IL_LED_BLINK(0) using blinking idx table */
static int led_mode;
module_param(led_mode, int, S_IRUGO);
MODULE_PARM_DESC(led_mode,
		 "0=system default, " "1=On(RF On)/Off(RF Off), 2=blinking");

/* Throughput		OFF time(ms)	ON time (ms)
 *	>300			25		25
 *	>200 to 300		40		40
 *	>100 to 200		55		55
 *	>70 to 100		65		65
 *	>50 to 70		75		75
 *	>20 to 50		85		85
 *	>10 to 20		95		95
 *	>5 to 10		110		110
 *	>1 to 5			130		130
 *	>0 to 1			167		167
 *	<=0					SOLID ON
 */
static const struct ieee80211_tpt_blink il_blink[] = {
	{.throughput = 0,		.blink_time = 334},
	{.throughput = 1 * 1024 - 1,	.blink_time = 260},
	{.throughput = 5 * 1024 - 1,	.blink_time = 220},
	{.throughput = 10 * 1024 - 1,	.blink_time = 190},
	{.throughput = 20 * 1024 - 1,	.blink_time = 170},
	{.throughput = 50 * 1024 - 1,	.blink_time = 150},
	{.throughput = 70 * 1024 - 1,	.blink_time = 130},
	{.throughput = 100 * 1024 - 1,	.blink_time = 110},
	{.throughput = 200 * 1024 - 1,	.blink_time = 80},
	{.throughput = 300 * 1024 - 1,	.blink_time = 50},
};

/*
 * Adjust led blink rate to compensate on a MAC Clock difference on every HW
 * Led blink rate analysis showed an average deviation of 0% on 3945,
 * 5% on 4965 HW.
 * Need to compensate on the led on/off time per HW according to the deviation
 * to achieve the desired led frequency
 * The calculation is: (100-averageDeviation)/100 * blinkTime
 * For code efficiency the calculation will be:
 *     compensation = (100 - averageDeviation) * 64 / 100
 *     NewBlinkTime = (compensation * BlinkTime) / 64
 */
static inline u8
il_blink_compensation(struct il_priv *il, u8 time, u16 compensation)
{
	if (!compensation) {
		IL_ERR("undefined blink compensation: "
		       "use pre-defined blinking time\n");
		return time;
	}

	return (u8) ((time * compensation) >> 6);
}

/* Set led pattern command */
static int
il_led_cmd(struct il_priv *il, unsigned long on, unsigned long off)
{
	struct il_led_cmd led_cmd = {
		.id = IL_LED_LINK,
		.interval = IL_DEF_LED_INTRVL
	};
	int ret;

	if (!test_bit(S_READY, &il->status))
		return -EBUSY;

	if (il->blink_on == on && il->blink_off == off)
		return 0;

	if (off == 0) {
		/* led is SOLID_ON */
		on = IL_LED_SOLID;
	}

	D_LED("Led blink time compensation=%u\n",
	      il->cfg->led_compensation);
	led_cmd.on =
	    il_blink_compensation(il, on,
				  il->cfg->led_compensation);
	led_cmd.off =
	    il_blink_compensation(il, off,
				  il->cfg->led_compensation);

	ret = il->ops->send_led_cmd(il, &led_cmd);
	if (!ret) {
		il->blink_on = on;
		il->blink_off = off;
	}
	return ret;
}

static void
il_led_brightness_set(struct led_classdev *led_cdev,
		      enum led_brightness brightness)
{
	struct il_priv *il = container_of(led_cdev, struct il_priv, led);
	unsigned long on = 0;

	if (brightness > 0)
		on = IL_LED_SOLID;

	il_led_cmd(il, on, 0);
}

static int
il_led_blink_set(struct led_classdev *led_cdev, unsigned long *delay_on,
		 unsigned long *delay_off)
{
	struct il_priv *il = container_of(led_cdev, struct il_priv, led);

	return il_led_cmd(il, *delay_on, *delay_off);
}

void
il_leds_init(struct il_priv *il)
{
	int mode = led_mode;
	int ret;

	if (mode == IL_LED_DEFAULT)
		mode = il->cfg->led_mode;

	il->led.name =
	    kasprintf(GFP_KERNEL, "%s-led", wiphy_name(il->hw->wiphy));
	il->led.brightness_set = il_led_brightness_set;
	il->led.blink_set = il_led_blink_set;
	il->led.max_brightness = 1;

	switch (mode) {
	case IL_LED_DEFAULT:
		WARN_ON(1);
		break;
	case IL_LED_BLINK:
		il->led.default_trigger =
		    ieee80211_create_tpt_led_trigger(il->hw,
						     IEEE80211_TPT_LEDTRIG_FL_CONNECTED,
						     il_blink,
						     ARRAY_SIZE(il_blink));
		break;
	case IL_LED_RF_STATE:
		il->led.default_trigger = ieee80211_get_radio_led_name(il->hw);
		break;
	}

	ret = led_classdev_register(&il->pci_dev->dev, &il->led);
	if (ret) {
		kfree(il->led.name);
		return;
	}

	il->led_registered = true;
}
EXPORT_SYMBOL(il_leds_init);

void
il_leds_exit(struct il_priv *il)
{
	if (!il->led_registered)
		return;

	led_classdev_unregister(&il->led);
	kfree(il->led.name);
}
EXPORT_SYMBOL(il_leds_exit);

/************************** EEPROM BANDS ****************************
 *
 * The il_eeprom_band definitions below provide the mapping from the
 * EEPROM contents to the specific channel number supported for each
 * band.
 *
 * For example, il_priv->eeprom.band_3_channels[4] from the band_3
 * definition below maps to physical channel 42 in the 5.2GHz spectrum.
 * The specific geography and calibration information for that channel
 * is contained in the eeprom map itself.
 *
 * During init, we copy the eeprom information and channel map
 * information into il->channel_info_24/52 and il->channel_map_24/52
 *
 * channel_map_24/52 provides the idx in the channel_info array for a
 * given channel.  We have to have two separate maps as there is channel
 * overlap with the 2.4GHz and 5.2GHz spectrum as seen in band_1 and
 * band_2
 *
 * A value of 0xff stored in the channel_map indicates that the channel
 * is not supported by the hardware at all.
 *
 * A value of 0xfe in the channel_map indicates that the channel is not
 * valid for Tx with the current hardware.  This means that
 * while the system can tune and receive on a given channel, it may not
 * be able to associate or transmit any frames on that
 * channel.  There is no corresponding channel information for that
 * entry.
 *
 *********************************************************************/

/* 2.4 GHz */
const u8 il_eeprom_band_1[14] = {
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14
};

/* 5.2 GHz bands */
static const u8 il_eeprom_band_2[] = {	/* 4915-5080MHz */
	183, 184, 185, 187, 188, 189, 192, 196, 7, 8, 11, 12, 16
};

static const u8 il_eeprom_band_3[] = {	/* 5170-5320MHz */
	34, 36, 38, 40, 42, 44, 46, 48, 52, 56, 60, 64
};

static const u8 il_eeprom_band_4[] = {	/* 5500-5700MHz */
	100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140
};

static const u8 il_eeprom_band_5[] = {	/* 5725-5825MHz */
	145, 149, 153, 157, 161, 165
};

static const u8 il_eeprom_band_6[] = {	/* 2.4 ht40 channel */
	1, 2, 3, 4, 5, 6, 7
};

static const u8 il_eeprom_band_7[] = {	/* 5.2 ht40 channel */
	36, 44, 52, 60, 100, 108, 116, 124, 132, 149, 157
};

/******************************************************************************
 *
 * EEPROM related functions
 *
******************************************************************************/

static int
il_eeprom_verify_signature(struct il_priv *il)
{
	u32 gp = _il_rd(il, CSR_EEPROM_GP) & CSR_EEPROM_GP_VALID_MSK;
	int ret = 0;

	D_EEPROM("EEPROM signature=0x%08x\n", gp);
	switch (gp) {
	case CSR_EEPROM_GP_GOOD_SIG_EEP_LESS_THAN_4K:
	case CSR_EEPROM_GP_GOOD_SIG_EEP_MORE_THAN_4K:
		break;
	default:
		IL_ERR("bad EEPROM signature," "EEPROM_GP=0x%08x\n", gp);
		ret = -ENOENT;
		break;
	}
	return ret;
}

const u8 *
il_eeprom_query_addr(const struct il_priv *il, size_t offset)
{
	BUG_ON(offset >= il->cfg->eeprom_size);
	return &il->eeprom[offset];
}
EXPORT_SYMBOL(il_eeprom_query_addr);

u16
il_eeprom_query16(const struct il_priv *il, size_t offset)
{
	if (!il->eeprom)
		return 0;
	return (u16) il->eeprom[offset] | ((u16) il->eeprom[offset + 1] << 8);
}
EXPORT_SYMBOL(il_eeprom_query16);

/**
 * il_eeprom_init - read EEPROM contents
 *
 * Load the EEPROM contents from adapter into il->eeprom
 *
 * NOTE:  This routine uses the non-debug IO access functions.
 */
int
il_eeprom_init(struct il_priv *il)
{
	__le16 *e;
	u32 gp = _il_rd(il, CSR_EEPROM_GP);
	int sz;
	int ret;
	int addr;

	/* allocate eeprom */
	sz = il->cfg->eeprom_size;
	D_EEPROM("NVM size = %d\n", sz);
	il->eeprom = kzalloc(sz, GFP_KERNEL);
	if (!il->eeprom)
		return -ENOMEM;

	e = (__le16 *) il->eeprom;

	il->ops->apm_init(il);

	ret = il_eeprom_verify_signature(il);
	if (ret < 0) {
		IL_ERR("EEPROM not found, EEPROM_GP=0x%08x\n", gp);
		ret = -ENOENT;
		goto err;
	}

	/* Make sure driver (instead of uCode) is allowed to read EEPROM */
	ret = il->ops->eeprom_acquire_semaphore(il);
	if (ret < 0) {
		IL_ERR("Failed to acquire EEPROM semaphore.\n");
		ret = -ENOENT;
		goto err;
	}

	/* eeprom is an array of 16bit values */
	for (addr = 0; addr < sz; addr += sizeof(u16)) {
		u32 r;

		_il_wr(il, CSR_EEPROM_REG,
		       CSR_EEPROM_REG_MSK_ADDR & (addr << 1));

		ret =
		    _il_poll_bit(il, CSR_EEPROM_REG,
				 CSR_EEPROM_REG_READ_VALID_MSK,
				 CSR_EEPROM_REG_READ_VALID_MSK,
				 IL_EEPROM_ACCESS_TIMEOUT);
		if (ret < 0) {
			IL_ERR("Time out reading EEPROM[%d]\n", addr);
			goto done;
		}
		r = _il_rd(il, CSR_EEPROM_REG);
		e[addr / 2] = cpu_to_le16(r >> 16);
	}

	D_EEPROM("NVM Type: %s, version: 0x%x\n", "EEPROM",
		 il_eeprom_query16(il, EEPROM_VERSION));

	ret = 0;
done:
	il->ops->eeprom_release_semaphore(il);

err:
	if (ret)
		il_eeprom_free(il);
	/* Reset chip to save power until we load uCode during "up". */
	il_apm_stop(il);
	return ret;
}
EXPORT_SYMBOL(il_eeprom_init);

void
il_eeprom_free(struct il_priv *il)
{
	kfree(il->eeprom);
	il->eeprom = NULL;
}
EXPORT_SYMBOL(il_eeprom_free);

static void
il_init_band_reference(const struct il_priv *il, int eep_band,
		       int *eeprom_ch_count,
		       const struct il_eeprom_channel **eeprom_ch_info,
		       const u8 **eeprom_ch_idx)
{
	u32 offset = il->cfg->regulatory_bands[eep_band - 1];

	switch (eep_band) {
	case 1:		/* 2.4GHz band */
		*eeprom_ch_count = ARRAY_SIZE(il_eeprom_band_1);
		*eeprom_ch_info =
		    (struct il_eeprom_channel *)il_eeprom_query_addr(il,
								     offset);
		*eeprom_ch_idx = il_eeprom_band_1;
		break;
	case 2:		/* 4.9GHz band */
		*eeprom_ch_count = ARRAY_SIZE(il_eeprom_band_2);
		*eeprom_ch_info =
		    (struct il_eeprom_channel *)il_eeprom_query_addr(il,
								     offset);
		*eeprom_ch_idx = il_eeprom_band_2;
		break;
	case 3:		/* 5.2GHz band */
		*eeprom_ch_count = ARRAY_SIZE(il_eeprom_band_3);
		*eeprom_ch_info =
		    (struct il_eeprom_channel *)il_eeprom_query_addr(il,
								     offset);
		*eeprom_ch_idx = il_eeprom_band_3;
		break;
	case 4:		/* 5.5GHz band */
		*eeprom_ch_count = ARRAY_SIZE(il_eeprom_band_4);
		*eeprom_ch_info =
		    (struct il_eeprom_channel *)il_eeprom_query_addr(il,
								     offset);
		*eeprom_ch_idx = il_eeprom_band_4;
		break;
	case 5:		/* 5.7GHz band */
		*eeprom_ch_count = ARRAY_SIZE(il_eeprom_band_5);
		*eeprom_ch_info =
		    (struct il_eeprom_channel *)il_eeprom_query_addr(il,
								     offset);
		*eeprom_ch_idx = il_eeprom_band_5;
		break;
	case 6:		/* 2.4GHz ht40 channels */
		*eeprom_ch_count = ARRAY_SIZE(il_eeprom_band_6);
		*eeprom_ch_info =
		    (struct il_eeprom_channel *)il_eeprom_query_addr(il,
								     offset);
		*eeprom_ch_idx = il_eeprom_band_6;
		break;
	case 7:		/* 5 GHz ht40 channels */
		*eeprom_ch_count = ARRAY_SIZE(il_eeprom_band_7);
		*eeprom_ch_info =
		    (struct il_eeprom_channel *)il_eeprom_query_addr(il,
								     offset);
		*eeprom_ch_idx = il_eeprom_band_7;
		break;
	default:
		BUG();
	}
}

#define CHECK_AND_PRINT(x) ((eeprom_ch->flags & EEPROM_CHANNEL_##x) \
			    ? # x " " : "")
/**
 * il_mod_ht40_chan_info - Copy ht40 channel info into driver's il.
 *
 * Does not set up a command, or touch hardware.
 */
static int
il_mod_ht40_chan_info(struct il_priv *il, enum nl80211_band band, u16 channel,
		      const struct il_eeprom_channel *eeprom_ch,
		      u8 clear_ht40_extension_channel)
{
	struct il_channel_info *ch_info;

	ch_info =
	    (struct il_channel_info *)il_get_channel_info(il, band, channel);

	if (!il_is_channel_valid(ch_info))
		return -1;

	D_EEPROM("HT40 Ch. %d [%sGHz] %s%s%s%s%s(0x%02x %ddBm):"
		 " Ad-Hoc %ssupported\n", ch_info->channel,
		 il_is_channel_a_band(ch_info) ? "5.2" : "2.4",
		 CHECK_AND_PRINT(IBSS), CHECK_AND_PRINT(ACTIVE),
		 CHECK_AND_PRINT(RADAR), CHECK_AND_PRINT(WIDE),
		 CHECK_AND_PRINT(DFS), eeprom_ch->flags,
		 eeprom_ch->max_power_avg,
		 ((eeprom_ch->flags & EEPROM_CHANNEL_IBSS) &&
		  !(eeprom_ch->flags & EEPROM_CHANNEL_RADAR)) ? "" : "not ");

	ch_info->ht40_eeprom = *eeprom_ch;
	ch_info->ht40_max_power_avg = eeprom_ch->max_power_avg;
	ch_info->ht40_flags = eeprom_ch->flags;
	if (eeprom_ch->flags & EEPROM_CHANNEL_VALID)
		ch_info->ht40_extension_channel &=
		    ~clear_ht40_extension_channel;

	return 0;
}

#define CHECK_AND_PRINT_I(x) ((eeprom_ch_info[ch].flags & EEPROM_CHANNEL_##x) \
			    ? # x " " : "")

/**
 * il_init_channel_map - Set up driver's info for all possible channels
 */
int
il_init_channel_map(struct il_priv *il)
{
	int eeprom_ch_count = 0;
	const u8 *eeprom_ch_idx = NULL;
	const struct il_eeprom_channel *eeprom_ch_info = NULL;
	int band, ch;
	struct il_channel_info *ch_info;

	if (il->channel_count) {
		D_EEPROM("Channel map already initialized.\n");
		return 0;
	}

	D_EEPROM("Initializing regulatory info from EEPROM\n");

	il->channel_count =
	    ARRAY_SIZE(il_eeprom_band_1) + ARRAY_SIZE(il_eeprom_band_2) +
	    ARRAY_SIZE(il_eeprom_band_3) + ARRAY_SIZE(il_eeprom_band_4) +
	    ARRAY_SIZE(il_eeprom_band_5);

	D_EEPROM("Parsing data for %d channels.\n", il->channel_count);

	il->channel_info =
	    kzalloc(sizeof(struct il_channel_info) * il->channel_count,
		    GFP_KERNEL);
	if (!il->channel_info) {
		IL_ERR("Could not allocate channel_info\n");
		il->channel_count = 0;
		return -ENOMEM;
	}

	ch_info = il->channel_info;

	/* Loop through the 5 EEPROM bands adding them in order to the
	 * channel map we maintain (that contains additional information than
	 * what just in the EEPROM) */
	for (band = 1; band <= 5; band++) {

		il_init_band_reference(il, band, &eeprom_ch_count,
				       &eeprom_ch_info, &eeprom_ch_idx);

		/* Loop through each band adding each of the channels */
		for (ch = 0; ch < eeprom_ch_count; ch++) {
			ch_info->channel = eeprom_ch_idx[ch];
			ch_info->band =
			    (band ==
			     1) ? NL80211_BAND_2GHZ : NL80211_BAND_5GHZ;

			/* permanently store EEPROM's channel regulatory flags
			 *   and max power in channel info database. */
			ch_info->eeprom = eeprom_ch_info[ch];

			/* Copy the run-time flags so they are there even on
			 * invalid channels */
			ch_info->flags = eeprom_ch_info[ch].flags;
			/* First write that ht40 is not enabled, and then enable
			 * one by one */
			ch_info->ht40_extension_channel =
			    IEEE80211_CHAN_NO_HT40;

			if (!(il_is_channel_valid(ch_info))) {
				D_EEPROM("Ch. %d Flags %x [%sGHz] - "
					 "No traffic\n", ch_info->channel,
					 ch_info->flags,
					 il_is_channel_a_band(ch_info) ? "5.2" :
					 "2.4");
				ch_info++;
				continue;
			}

			/* Initialize regulatory-based run-time data */
			ch_info->max_power_avg = ch_info->curr_txpow =
			    eeprom_ch_info[ch].max_power_avg;
			ch_info->scan_power = eeprom_ch_info[ch].max_power_avg;
			ch_info->min_power = 0;

			D_EEPROM("Ch. %d [%sGHz] " "%s%s%s%s%s%s(0x%02x %ddBm):"
				 " Ad-Hoc %ssupported\n", ch_info->channel,
				 il_is_channel_a_band(ch_info) ? "5.2" : "2.4",
				 CHECK_AND_PRINT_I(VALID),
				 CHECK_AND_PRINT_I(IBSS),
				 CHECK_AND_PRINT_I(ACTIVE),
				 CHECK_AND_PRINT_I(RADAR),
				 CHECK_AND_PRINT_I(WIDE),
				 CHECK_AND_PRINT_I(DFS),
				 eeprom_ch_info[ch].flags,
				 eeprom_ch_info[ch].max_power_avg,
				 ((eeprom_ch_info[ch].
				   flags & EEPROM_CHANNEL_IBSS) &&
				  !(eeprom_ch_info[ch].
				    flags & EEPROM_CHANNEL_RADAR)) ? "" :
				 "not ");

			ch_info++;
		}
	}

	/* Check if we do have HT40 channels */
	if (il->cfg->regulatory_bands[5] == EEPROM_REGULATORY_BAND_NO_HT40 &&
	    il->cfg->regulatory_bands[6] == EEPROM_REGULATORY_BAND_NO_HT40)
		return 0;

	/* Two additional EEPROM bands for 2.4 and 5 GHz HT40 channels */
	for (band = 6; band <= 7; band++) {
		enum nl80211_band ieeeband;

		il_init_band_reference(il, band, &eeprom_ch_count,
				       &eeprom_ch_info, &eeprom_ch_idx);

		/* EEPROM band 6 is 2.4, band 7 is 5 GHz */
		ieeeband =
		    (band == 6) ? NL80211_BAND_2GHZ : NL80211_BAND_5GHZ;

		/* Loop through each band adding each of the channels */
		for (ch = 0; ch < eeprom_ch_count; ch++) {
			/* Set up driver's info for lower half */
			il_mod_ht40_chan_info(il, ieeeband, eeprom_ch_idx[ch],
					      &eeprom_ch_info[ch],
					      IEEE80211_CHAN_NO_HT40PLUS);

			/* Set up driver's info for upper half */
			il_mod_ht40_chan_info(il, ieeeband,
					      eeprom_ch_idx[ch] + 4,
					      &eeprom_ch_info[ch],
					      IEEE80211_CHAN_NO_HT40MINUS);
		}
	}

	return 0;
}
EXPORT_SYMBOL(il_init_channel_map);

/*
 * il_free_channel_map - undo allocations in il_init_channel_map
 */
void
il_free_channel_map(struct il_priv *il)
{
	kfree(il->channel_info);
	il->channel_count = 0;
}
EXPORT_SYMBOL(il_free_channel_map);

/**
 * il_get_channel_info - Find driver's ilate channel info
 *
 * Based on band and channel number.
 */
const struct il_channel_info *
il_get_channel_info(const struct il_priv *il, enum nl80211_band band,
		    u16 channel)
{
	int i;

	switch (band) {
	case NL80211_BAND_5GHZ:
		for (i = 14; i < il->channel_count; i++) {
			if (il->channel_info[i].channel == channel)
				return &il->channel_info[i];
		}
		break;
	case NL80211_BAND_2GHZ:
		if (channel >= 1 && channel <= 14)
			return &il->channel_info[channel - 1];
		break;
	default:
		BUG();
	}

	return NULL;
}
EXPORT_SYMBOL(il_get_channel_info);

/*
 * Setting power level allows the card to go to sleep when not busy.
 *
 * We calculate a sleep command based on the required latency, which
 * we get from mac80211.
 */

#define SLP_VEC(X0, X1, X2, X3, X4) { \
		cpu_to_le32(X0), \
		cpu_to_le32(X1), \
		cpu_to_le32(X2), \
		cpu_to_le32(X3), \
		cpu_to_le32(X4)  \
}

static void
il_build_powertable_cmd(struct il_priv *il, struct il_powertable_cmd *cmd)
{
	const __le32 interval[3][IL_POWER_VEC_SIZE] = {
		SLP_VEC(2, 2, 4, 6, 0xFF),
		SLP_VEC(2, 4, 7, 10, 10),
		SLP_VEC(4, 7, 10, 10, 0xFF)
	};
	int i, dtim_period, no_dtim;
	u32 max_sleep;
	bool skip;

	memset(cmd, 0, sizeof(*cmd));

	if (il->power_data.pci_pm)
		cmd->flags |= IL_POWER_PCI_PM_MSK;

	/* if no Power Save, we are done */
	if (il->power_data.ps_disabled)
		return;

	cmd->flags = IL_POWER_DRIVER_ALLOW_SLEEP_MSK;
	cmd->keep_alive_seconds = 0;
	cmd->debug_flags = 0;
	cmd->rx_data_timeout = cpu_to_le32(25 * 1024);
	cmd->tx_data_timeout = cpu_to_le32(25 * 1024);
	cmd->keep_alive_beacons = 0;

	dtim_period = il->vif ? il->vif->bss_conf.dtim_period : 0;

	if (dtim_period <= 2) {
		memcpy(cmd->sleep_interval, interval[0], sizeof(interval[0]));
		no_dtim = 2;
	} else if (dtim_period <= 10) {
		memcpy(cmd->sleep_interval, interval[1], sizeof(interval[1]));
		no_dtim = 2;
	} else {
		memcpy(cmd->sleep_interval, interval[2], sizeof(interval[2]));
		no_dtim = 0;
	}

	if (dtim_period == 0) {
		dtim_period = 1;
		skip = false;
	} else {
		skip = !!no_dtim;
	}

	if (skip) {
		__le32 tmp = cmd->sleep_interval[IL_POWER_VEC_SIZE - 1];

		max_sleep = le32_to_cpu(tmp);
		if (max_sleep == 0xFF)
			max_sleep = dtim_period * (skip + 1);
		else if (max_sleep >  dtim_period)
			max_sleep = (max_sleep / dtim_period) * dtim_period;
		cmd->flags |= IL_POWER_SLEEP_OVER_DTIM_MSK;
	} else {
		max_sleep = dtim_period;
		cmd->flags &= ~IL_POWER_SLEEP_OVER_DTIM_MSK;
	}

	for (i = 0; i < IL_POWER_VEC_SIZE; i++)
		if (le32_to_cpu(cmd->sleep_interval[i]) > max_sleep)
			cmd->sleep_interval[i] = cpu_to_le32(max_sleep);
}

static int
il_set_power(struct il_priv *il, struct il_powertable_cmd *cmd)
{
	D_POWER("Sending power/sleep command\n");
	D_POWER("Flags value = 0x%08X\n", cmd->flags);
	D_POWER("Tx timeout = %u\n", le32_to_cpu(cmd->tx_data_timeout));
	D_POWER("Rx timeout = %u\n", le32_to_cpu(cmd->rx_data_timeout));
	D_POWER("Sleep interval vector = { %d , %d , %d , %d , %d }\n",
		le32_to_cpu(cmd->sleep_interval[0]),
		le32_to_cpu(cmd->sleep_interval[1]),
		le32_to_cpu(cmd->sleep_interval[2]),
		le32_to_cpu(cmd->sleep_interval[3]),
		le32_to_cpu(cmd->sleep_interval[4]));

	return il_send_cmd_pdu(il, C_POWER_TBL,
			       sizeof(struct il_powertable_cmd), cmd);
}

static int
il_power_set_mode(struct il_priv *il, struct il_powertable_cmd *cmd, bool force)
{
	int ret;
	bool update_chains;

	lockdep_assert_held(&il->mutex);

	/* Don't update the RX chain when chain noise calibration is running */
	update_chains = il->chain_noise_data.state == IL_CHAIN_NOISE_DONE ||
	    il->chain_noise_data.state == IL_CHAIN_NOISE_ALIVE;

	if (!memcmp(&il->power_data.sleep_cmd, cmd, sizeof(*cmd)) && !force)
		return 0;

	if (!il_is_ready_rf(il))
		return -EIO;

	/* scan complete use sleep_power_next, need to be updated */
	memcpy(&il->power_data.sleep_cmd_next, cmd, sizeof(*cmd));
	if (test_bit(S_SCANNING, &il->status) && !force) {
		D_INFO("Defer power set mode while scanning\n");
		return 0;
	}

	if (cmd->flags & IL_POWER_DRIVER_ALLOW_SLEEP_MSK)
		set_bit(S_POWER_PMI, &il->status);

	ret = il_set_power(il, cmd);
	if (!ret) {
		if (!(cmd->flags & IL_POWER_DRIVER_ALLOW_SLEEP_MSK))
			clear_bit(S_POWER_PMI, &il->status);

		if (il->ops->update_chain_flags && update_chains)
			il->ops->update_chain_flags(il);
		else if (il->ops->update_chain_flags)
			D_POWER("Cannot update the power, chain noise "
				"calibration running: %d\n",
				il->chain_noise_data.state);

		memcpy(&il->power_data.sleep_cmd, cmd, sizeof(*cmd));
	} else
		IL_ERR("set power fail, ret = %d", ret);

	return ret;
}

int
il_power_update_mode(struct il_priv *il, bool force)
{
	struct il_powertable_cmd cmd;

	il_build_powertable_cmd(il, &cmd);

	return il_power_set_mode(il, &cmd, force);
}
EXPORT_SYMBOL(il_power_update_mode);

/* initialize to default */
void
il_power_initialize(struct il_priv *il)
{
	u16 lctl;

	pcie_capability_read_word(il->pci_dev, PCI_EXP_LNKCTL, &lctl);
	il->power_data.pci_pm = !(lctl & PCI_EXP_LNKCTL_ASPM_L0S);

	il->power_data.debug_sleep_level_override = -1;

	memset(&il->power_data.sleep_cmd, 0, sizeof(il->power_data.sleep_cmd));
}
EXPORT_SYMBOL(il_power_initialize);

/* For active scan, listen ACTIVE_DWELL_TIME (msec) on each channel after
 * sending probe req.  This should be set long enough to hear probe responses
 * from more than one AP.  */
#define IL_ACTIVE_DWELL_TIME_24    (30)	/* all times in msec */
#define IL_ACTIVE_DWELL_TIME_52    (20)

#define IL_ACTIVE_DWELL_FACTOR_24GHZ (3)
#define IL_ACTIVE_DWELL_FACTOR_52GHZ (2)

/* For passive scan, listen PASSIVE_DWELL_TIME (msec) on each channel.
 * Must be set longer than active dwell time.
 * For the most reliable scan, set > AP beacon interval (typically 100msec). */
#define IL_PASSIVE_DWELL_TIME_24   (20)	/* all times in msec */
#define IL_PASSIVE_DWELL_TIME_52   (10)
#define IL_PASSIVE_DWELL_BASE      (100)
#define IL_CHANNEL_TUNE_TIME       5

static int
il_send_scan_abort(struct il_priv *il)
{
	int ret;
	struct il_rx_pkt *pkt;
	struct il_host_cmd cmd = {
		.id = C_SCAN_ABORT,
		.flags = CMD_WANT_SKB,
	};

	/* Exit instantly with error when device is not ready
	 * to receive scan abort command or it does not perform
	 * hardware scan currently */
	if (!test_bit(S_READY, &il->status) ||
	    !test_bit(S_GEO_CONFIGURED, &il->status) ||
	    !test_bit(S_SCAN_HW, &il->status) ||
	    test_bit(S_FW_ERROR, &il->status) ||
	    test_bit(S_EXIT_PENDING, &il->status))
		return -EIO;

	ret = il_send_cmd_sync(il, &cmd);
	if (ret)
		return ret;

	pkt = (struct il_rx_pkt *)cmd.reply_page;
	if (pkt->u.status != CAN_ABORT_STATUS) {
		/* The scan abort will return 1 for success or
		 * 2 for "failure".  A failure condition can be
		 * due to simply not being in an active scan which
		 * can occur if we send the scan abort before we
		 * the microcode has notified us that a scan is
		 * completed. */
		D_SCAN("SCAN_ABORT ret %d.\n", pkt->u.status);
		ret = -EIO;
	}

	il_free_pages(il, cmd.reply_page);
	return ret;
}

static void
il_complete_scan(struct il_priv *il, bool aborted)
{
	struct cfg80211_scan_info info = {
		.aborted = aborted,
	};

	/* check if scan was requested from mac80211 */
	if (il->scan_request) {
		D_SCAN("Complete scan in mac80211\n");
		ieee80211_scan_completed(il->hw, &info);
	}

	il->scan_vif = NULL;
	il->scan_request = NULL;
}

void
il_force_scan_end(struct il_priv *il)
{
	lockdep_assert_held(&il->mutex);

	if (!test_bit(S_SCANNING, &il->status)) {
		D_SCAN("Forcing scan end while not scanning\n");
		return;
	}

	D_SCAN("Forcing scan end\n");
	clear_bit(S_SCANNING, &il->status);
	clear_bit(S_SCAN_HW, &il->status);
	clear_bit(S_SCAN_ABORTING, &il->status);
	il_complete_scan(il, true);
}

static void
il_do_scan_abort(struct il_priv *il)
{
	int ret;

	lockdep_assert_held(&il->mutex);

	if (!test_bit(S_SCANNING, &il->status)) {
		D_SCAN("Not performing scan to abort\n");
		return;
	}

	if (test_and_set_bit(S_SCAN_ABORTING, &il->status)) {
		D_SCAN("Scan abort in progress\n");
		return;
	}

	ret = il_send_scan_abort(il);
	if (ret) {
		D_SCAN("Send scan abort failed %d\n", ret);
		il_force_scan_end(il);
	} else
		D_SCAN("Successfully send scan abort\n");
}

/**
 * il_scan_cancel - Cancel any currently executing HW scan
 */
int
il_scan_cancel(struct il_priv *il)
{
	D_SCAN("Queuing abort scan\n");
	queue_work(il->workqueue, &il->abort_scan);
	return 0;
}
EXPORT_SYMBOL(il_scan_cancel);

/**
 * il_scan_cancel_timeout - Cancel any currently executing HW scan
 * @ms: amount of time to wait (in milliseconds) for scan to abort
 *
 */
int
il_scan_cancel_timeout(struct il_priv *il, unsigned long ms)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(ms);

	lockdep_assert_held(&il->mutex);

	D_SCAN("Scan cancel timeout\n");

	il_do_scan_abort(il);

	while (time_before_eq(jiffies, timeout)) {
		if (!test_bit(S_SCAN_HW, &il->status))
			break;
		msleep(20);
	}

	return test_bit(S_SCAN_HW, &il->status);
}
EXPORT_SYMBOL(il_scan_cancel_timeout);

/* Service response to C_SCAN (0x80) */
static void
il_hdl_scan(struct il_priv *il, struct il_rx_buf *rxb)
{
#ifdef CONFIG_IWLEGACY_DEBUG
	struct il_rx_pkt *pkt = rxb_addr(rxb);
	struct il_scanreq_notification *notif =
	    (struct il_scanreq_notification *)pkt->u.raw;

	D_SCAN("Scan request status = 0x%x\n", notif->status);
#endif
}

/* Service N_SCAN_START (0x82) */
static void
il_hdl_scan_start(struct il_priv *il, struct il_rx_buf *rxb)
{
	struct il_rx_pkt *pkt = rxb_addr(rxb);
	struct il_scanstart_notification *notif =
	    (struct il_scanstart_notification *)pkt->u.raw;
	il->scan_start_tsf = le32_to_cpu(notif->tsf_low);
	D_SCAN("Scan start: " "%d [802.11%s] "
	       "(TSF: 0x%08X:%08X) - %d (beacon timer %u)\n", notif->channel,
	       notif->band ? "bg" : "a", le32_to_cpu(notif->tsf_high),
	       le32_to_cpu(notif->tsf_low), notif->status, notif->beacon_timer);
}

/* Service N_SCAN_RESULTS (0x83) */
static void
il_hdl_scan_results(struct il_priv *il, struct il_rx_buf *rxb)
{
#ifdef CONFIG_IWLEGACY_DEBUG
	struct il_rx_pkt *pkt = rxb_addr(rxb);
	struct il_scanresults_notification *notif =
	    (struct il_scanresults_notification *)pkt->u.raw;

	D_SCAN("Scan ch.res: " "%d [802.11%s] " "(TSF: 0x%08X:%08X) - %d "
	       "elapsed=%lu usec\n", notif->channel, notif->band ? "bg" : "a",
	       le32_to_cpu(notif->tsf_high), le32_to_cpu(notif->tsf_low),
	       le32_to_cpu(notif->stats[0]),
	       le32_to_cpu(notif->tsf_low) - il->scan_start_tsf);
#endif
}

/* Service N_SCAN_COMPLETE (0x84) */
static void
il_hdl_scan_complete(struct il_priv *il, struct il_rx_buf *rxb)
{

#ifdef CONFIG_IWLEGACY_DEBUG
	struct il_rx_pkt *pkt = rxb_addr(rxb);
	struct il_scancomplete_notification *scan_notif = (void *)pkt->u.raw;
#endif

	D_SCAN("Scan complete: %d channels (TSF 0x%08X:%08X) - %d\n",
	       scan_notif->scanned_channels, scan_notif->tsf_low,
	       scan_notif->tsf_high, scan_notif->status);

	/* The HW is no longer scanning */
	clear_bit(S_SCAN_HW, &il->status);

	D_SCAN("Scan on %sGHz took %dms\n",
	       (il->scan_band == NL80211_BAND_2GHZ) ? "2.4" : "5.2",
	       jiffies_to_msecs(jiffies - il->scan_start));

	queue_work(il->workqueue, &il->scan_completed);
}

void
il_setup_rx_scan_handlers(struct il_priv *il)
{
	/* scan handlers */
	il->handlers[C_SCAN] = il_hdl_scan;
	il->handlers[N_SCAN_START] = il_hdl_scan_start;
	il->handlers[N_SCAN_RESULTS] = il_hdl_scan_results;
	il->handlers[N_SCAN_COMPLETE] = il_hdl_scan_complete;
}
EXPORT_SYMBOL(il_setup_rx_scan_handlers);

u16
il_get_active_dwell_time(struct il_priv *il, enum nl80211_band band,
			 u8 n_probes)
{
	if (band == NL80211_BAND_5GHZ)
		return IL_ACTIVE_DWELL_TIME_52 +
		    IL_ACTIVE_DWELL_FACTOR_52GHZ * (n_probes + 1);
	else
		return IL_ACTIVE_DWELL_TIME_24 +
		    IL_ACTIVE_DWELL_FACTOR_24GHZ * (n_probes + 1);
}
EXPORT_SYMBOL(il_get_active_dwell_time);

u16
il_get_passive_dwell_time(struct il_priv *il, enum nl80211_band band,
			  struct ieee80211_vif *vif)
{
	u16 value;

	u16 passive =
	    (band ==
	     NL80211_BAND_2GHZ) ? IL_PASSIVE_DWELL_BASE +
	    IL_PASSIVE_DWELL_TIME_24 : IL_PASSIVE_DWELL_BASE +
	    IL_PASSIVE_DWELL_TIME_52;

	if (il_is_any_associated(il)) {
		/*
		 * If we're associated, we clamp the maximum passive
		 * dwell time to be 98% of the smallest beacon interval
		 * (minus 2 * channel tune time)
		 */
		value = il->vif ? il->vif->bss_conf.beacon_int : 0;
		if (value > IL_PASSIVE_DWELL_BASE || !value)
			value = IL_PASSIVE_DWELL_BASE;
		value = (value * 98) / 100 - IL_CHANNEL_TUNE_TIME * 2;
		passive = min(value, passive);
	}

	return passive;
}
EXPORT_SYMBOL(il_get_passive_dwell_time);

void
il_init_scan_params(struct il_priv *il)
{
	u8 ant_idx = fls(il->hw_params.valid_tx_ant) - 1;
	if (!il->scan_tx_ant[NL80211_BAND_5GHZ])
		il->scan_tx_ant[NL80211_BAND_5GHZ] = ant_idx;
	if (!il->scan_tx_ant[NL80211_BAND_2GHZ])
		il->scan_tx_ant[NL80211_BAND_2GHZ] = ant_idx;
}
EXPORT_SYMBOL(il_init_scan_params);

static int
il_scan_initiate(struct il_priv *il, struct ieee80211_vif *vif)
{
	int ret;

	lockdep_assert_held(&il->mutex);

	cancel_delayed_work(&il->scan_check);

	if (!il_is_ready_rf(il)) {
		IL_WARN("Request scan called when driver not ready.\n");
		return -EIO;
	}

	if (test_bit(S_SCAN_HW, &il->status)) {
		D_SCAN("Multiple concurrent scan requests in parallel.\n");
		return -EBUSY;
	}

	if (test_bit(S_SCAN_ABORTING, &il->status)) {
		D_SCAN("Scan request while abort pending.\n");
		return -EBUSY;
	}

	D_SCAN("Starting scan...\n");

	set_bit(S_SCANNING, &il->status);
	il->scan_start = jiffies;

	ret = il->ops->request_scan(il, vif);
	if (ret) {
		clear_bit(S_SCANNING, &il->status);
		return ret;
	}

	queue_delayed_work(il->workqueue, &il->scan_check,
			   IL_SCAN_CHECK_WATCHDOG);

	return 0;
}

int
il_mac_hw_scan(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
	       struct ieee80211_scan_request *hw_req)
{
	struct cfg80211_scan_request *req = &hw_req->req;
	struct il_priv *il = hw->priv;
	int ret;

	if (req->n_channels == 0) {
		IL_ERR("Can not scan on no channels.\n");
		return -EINVAL;
	}

	mutex_lock(&il->mutex);
	D_MAC80211("enter\n");

	if (test_bit(S_SCANNING, &il->status)) {
		D_SCAN("Scan already in progress.\n");
		ret = -EAGAIN;
		goto out_unlock;
	}

	/* mac80211 will only ask for one band at a time */
	il->scan_request = req;
	il->scan_vif = vif;
	il->scan_band = req->channels[0]->band;

	ret = il_scan_initiate(il, vif);

out_unlock:
	D_MAC80211("leave ret %d\n", ret);
	mutex_unlock(&il->mutex);

	return ret;
}
EXPORT_SYMBOL(il_mac_hw_scan);

static void
il_bg_scan_check(struct work_struct *data)
{
	struct il_priv *il =
	    container_of(data, struct il_priv, scan_check.work);

	D_SCAN("Scan check work\n");

	/* Since we are here firmware does not finish scan and
	 * most likely is in bad shape, so we don't bother to
	 * send abort command, just force scan complete to mac80211 */
	mutex_lock(&il->mutex);
	il_force_scan_end(il);
	mutex_unlock(&il->mutex);
}

/**
 * il_fill_probe_req - fill in all required fields and IE for probe request
 */

u16
il_fill_probe_req(struct il_priv *il, struct ieee80211_mgmt *frame,
		  const u8 *ta, const u8 *ies, int ie_len, int left)
{
	int len = 0;
	u8 *pos = NULL;

	/* Make sure there is enough space for the probe request,
	 * two mandatory IEs and the data */
	left -= 24;
	if (left < 0)
		return 0;

	frame->frame_control = cpu_to_le16(IEEE80211_STYPE_PROBE_REQ);
	eth_broadcast_addr(frame->da);
	memcpy(frame->sa, ta, ETH_ALEN);
	eth_broadcast_addr(frame->bssid);
	frame->seq_ctrl = 0;

	len += 24;

	/* ...next IE... */
	pos = &frame->u.probe_req.variable[0];

	/* fill in our indirect SSID IE */
	left -= 2;
	if (left < 0)
		return 0;
	*pos++ = WLAN_EID_SSID;
	*pos++ = 0;

	len += 2;

	if (WARN_ON(left < ie_len))
		return len;

	if (ies && ie_len) {
		memcpy(pos, ies, ie_len);
		len += ie_len;
	}

	return (u16) len;
}
EXPORT_SYMBOL(il_fill_probe_req);

static void
il_bg_abort_scan(struct work_struct *work)
{
	struct il_priv *il = container_of(work, struct il_priv, abort_scan);

	D_SCAN("Abort scan work\n");

	/* We keep scan_check work queued in case when firmware will not
	 * report back scan completed notification */
	mutex_lock(&il->mutex);
	il_scan_cancel_timeout(il, 200);
	mutex_unlock(&il->mutex);
}

static void
il_bg_scan_completed(struct work_struct *work)
{
	struct il_priv *il = container_of(work, struct il_priv, scan_completed);
	bool aborted;

	D_SCAN("Completed scan.\n");

	cancel_delayed_work(&il->scan_check);

	mutex_lock(&il->mutex);

	aborted = test_and_clear_bit(S_SCAN_ABORTING, &il->status);
	if (aborted)
		D_SCAN("Aborted scan completed.\n");

	if (!test_and_clear_bit(S_SCANNING, &il->status)) {
		D_SCAN("Scan already completed.\n");
		goto out_settings;
	}

	il_complete_scan(il, aborted);

out_settings:
	/* Can we still talk to firmware ? */
	if (!il_is_ready_rf(il))
		goto out;

	/*
	 * We do not commit power settings while scan is pending,
	 * do it now if the settings changed.
	 */
	il_power_set_mode(il, &il->power_data.sleep_cmd_next, false);
	il_set_tx_power(il, il->tx_power_next, false);

	il->ops->post_scan(il);

out:
	mutex_unlock(&il->mutex);
}

void
il_setup_scan_deferred_work(struct il_priv *il)
{
	INIT_WORK(&il->scan_completed, il_bg_scan_completed);
	INIT_WORK(&il->abort_scan, il_bg_abort_scan);
	INIT_DELAYED_WORK(&il->scan_check, il_bg_scan_check);
}
EXPORT_SYMBOL(il_setup_scan_deferred_work);

void
il_cancel_scan_deferred_work(struct il_priv *il)
{
	cancel_work_sync(&il->abort_scan);
	cancel_work_sync(&il->scan_completed);

	if (cancel_delayed_work_sync(&il->scan_check)) {
		mutex_lock(&il->mutex);
		il_force_scan_end(il);
		mutex_unlock(&il->mutex);
	}
}
EXPORT_SYMBOL(il_cancel_scan_deferred_work);

/* il->sta_lock must be held */
static void
il_sta_ucode_activate(struct il_priv *il, u8 sta_id)
{

	if (!(il->stations[sta_id].used & IL_STA_DRIVER_ACTIVE))
		IL_ERR("ACTIVATE a non DRIVER active station id %u addr %pM\n",
		       sta_id, il->stations[sta_id].sta.sta.addr);

	if (il->stations[sta_id].used & IL_STA_UCODE_ACTIVE) {
		D_ASSOC("STA id %u addr %pM already present"
			" in uCode (according to driver)\n", sta_id,
			il->stations[sta_id].sta.sta.addr);
	} else {
		il->stations[sta_id].used |= IL_STA_UCODE_ACTIVE;
		D_ASSOC("Added STA id %u addr %pM to uCode\n", sta_id,
			il->stations[sta_id].sta.sta.addr);
	}
}

static int
il_process_add_sta_resp(struct il_priv *il, struct il_addsta_cmd *addsta,
			struct il_rx_pkt *pkt, bool sync)
{
	u8 sta_id = addsta->sta.sta_id;
	unsigned long flags;
	int ret = -EIO;

	if (pkt->hdr.flags & IL_CMD_FAILED_MSK) {
		IL_ERR("Bad return from C_ADD_STA (0x%08X)\n", pkt->hdr.flags);
		return ret;
	}

	D_INFO("Processing response for adding station %u\n", sta_id);

	spin_lock_irqsave(&il->sta_lock, flags);

	switch (pkt->u.add_sta.status) {
	case ADD_STA_SUCCESS_MSK:
		D_INFO("C_ADD_STA PASSED\n");
		il_sta_ucode_activate(il, sta_id);
		ret = 0;
		break;
	case ADD_STA_NO_ROOM_IN_TBL:
		IL_ERR("Adding station %d failed, no room in table.\n", sta_id);
		break;
	case ADD_STA_NO_BLOCK_ACK_RESOURCE:
		IL_ERR("Adding station %d failed, no block ack resource.\n",
		       sta_id);
		break;
	case ADD_STA_MODIFY_NON_EXIST_STA:
		IL_ERR("Attempting to modify non-existing station %d\n",
		       sta_id);
		break;
	default:
		D_ASSOC("Received C_ADD_STA:(0x%08X)\n", pkt->u.add_sta.status);
		break;
	}

	D_INFO("%s station id %u addr %pM\n",
	       il->stations[sta_id].sta.mode ==
	       STA_CONTROL_MODIFY_MSK ? "Modified" : "Added", sta_id,
	       il->stations[sta_id].sta.sta.addr);

	/*
	 * XXX: The MAC address in the command buffer is often changed from
	 * the original sent to the device. That is, the MAC address
	 * written to the command buffer often is not the same MAC address
	 * read from the command buffer when the command returns. This
	 * issue has not yet been resolved and this debugging is left to
	 * observe the problem.
	 */
	D_INFO("%s station according to cmd buffer %pM\n",
	       il->stations[sta_id].sta.mode ==
	       STA_CONTROL_MODIFY_MSK ? "Modified" : "Added", addsta->sta.addr);
	spin_unlock_irqrestore(&il->sta_lock, flags);

	return ret;
}

static void
il_add_sta_callback(struct il_priv *il, struct il_device_cmd *cmd,
		    struct il_rx_pkt *pkt)
{
	struct il_addsta_cmd *addsta = (struct il_addsta_cmd *)cmd->cmd.payload;

	il_process_add_sta_resp(il, addsta, pkt, false);

}

int
il_send_add_sta(struct il_priv *il, struct il_addsta_cmd *sta, u8 flags)
{
	struct il_rx_pkt *pkt = NULL;
	int ret = 0;
	u8 data[sizeof(*sta)];
	struct il_host_cmd cmd = {
		.id = C_ADD_STA,
		.flags = flags,
		.data = data,
	};
	u8 sta_id __maybe_unused = sta->sta.sta_id;

	D_INFO("Adding sta %u (%pM) %ssynchronously\n", sta_id, sta->sta.addr,
	       flags & CMD_ASYNC ? "a" : "");

	if (flags & CMD_ASYNC)
		cmd.callback = il_add_sta_callback;
	else {
		cmd.flags |= CMD_WANT_SKB;
		might_sleep();
	}

	cmd.len = il->ops->build_addsta_hcmd(sta, data);
	ret = il_send_cmd(il, &cmd);
	if (ret)
		return ret;
	if (flags & CMD_ASYNC)
		return 0;

	pkt = (struct il_rx_pkt *)cmd.reply_page;
	ret = il_process_add_sta_resp(il, sta, pkt, true);

	il_free_pages(il, cmd.reply_page);

	return ret;
}
EXPORT_SYMBOL(il_send_add_sta);

static void
il_set_ht_add_station(struct il_priv *il, u8 idx, struct ieee80211_sta *sta)
{
	struct ieee80211_sta_ht_cap *sta_ht_inf = &sta->ht_cap;
	__le32 sta_flags;

	if (!sta || !sta_ht_inf->ht_supported)
		goto done;

	D_ASSOC("spatial multiplexing power save mode: %s\n",
		(sta->smps_mode == IEEE80211_SMPS_STATIC) ? "static" :
		(sta->smps_mode == IEEE80211_SMPS_DYNAMIC) ? "dynamic" :
		"disabled");

	sta_flags = il->stations[idx].sta.station_flags;

	sta_flags &= ~(STA_FLG_RTS_MIMO_PROT_MSK | STA_FLG_MIMO_DIS_MSK);

	switch (sta->smps_mode) {
	case IEEE80211_SMPS_STATIC:
		sta_flags |= STA_FLG_MIMO_DIS_MSK;
		break;
	case IEEE80211_SMPS_DYNAMIC:
		sta_flags |= STA_FLG_RTS_MIMO_PROT_MSK;
		break;
	case IEEE80211_SMPS_OFF:
		break;
	default:
		IL_WARN("Invalid MIMO PS mode %d\n", sta->smps_mode);
		break;
	}

	sta_flags |=
	    cpu_to_le32((u32) sta_ht_inf->
			ampdu_factor << STA_FLG_MAX_AGG_SIZE_POS);

	sta_flags |=
	    cpu_to_le32((u32) sta_ht_inf->
			ampdu_density << STA_FLG_AGG_MPDU_DENSITY_POS);

	if (il_is_ht40_tx_allowed(il, &sta->ht_cap))
		sta_flags |= STA_FLG_HT40_EN_MSK;
	else
		sta_flags &= ~STA_FLG_HT40_EN_MSK;

	il->stations[idx].sta.station_flags = sta_flags;
done:
	return;
}

/**
 * il_prep_station - Prepare station information for addition
 *
 * should be called with sta_lock held
 */
u8
il_prep_station(struct il_priv *il, const u8 *addr, bool is_ap,
		struct ieee80211_sta *sta)
{
	struct il_station_entry *station;
	int i;
	u8 sta_id = IL_INVALID_STATION;
	u16 rate;

	if (is_ap)
		sta_id = IL_AP_ID;
	else if (is_broadcast_ether_addr(addr))
		sta_id = il->hw_params.bcast_id;
	else
		for (i = IL_STA_ID; i < il->hw_params.max_stations; i++) {
			if (ether_addr_equal(il->stations[i].sta.sta.addr,
					     addr)) {
				sta_id = i;
				break;
			}

			if (!il->stations[i].used &&
			    sta_id == IL_INVALID_STATION)
				sta_id = i;
		}

	/*
	 * These two conditions have the same outcome, but keep them
	 * separate
	 */
	if (unlikely(sta_id == IL_INVALID_STATION))
		return sta_id;

	/*
	 * uCode is not able to deal with multiple requests to add a
	 * station. Keep track if one is in progress so that we do not send
	 * another.
	 */
	if (il->stations[sta_id].used & IL_STA_UCODE_INPROGRESS) {
		D_INFO("STA %d already in process of being added.\n", sta_id);
		return sta_id;
	}

	if ((il->stations[sta_id].used & IL_STA_DRIVER_ACTIVE) &&
	    (il->stations[sta_id].used & IL_STA_UCODE_ACTIVE) &&
	    ether_addr_equal(il->stations[sta_id].sta.sta.addr, addr)) {
		D_ASSOC("STA %d (%pM) already added, not adding again.\n",
			sta_id, addr);
		return sta_id;
	}

	station = &il->stations[sta_id];
	station->used = IL_STA_DRIVER_ACTIVE;
	D_ASSOC("Add STA to driver ID %d: %pM\n", sta_id, addr);
	il->num_stations++;

	/* Set up the C_ADD_STA command to send to device */
	memset(&station->sta, 0, sizeof(struct il_addsta_cmd));
	memcpy(station->sta.sta.addr, addr, ETH_ALEN);
	station->sta.mode = 0;
	station->sta.sta.sta_id = sta_id;
	station->sta.station_flags = 0;

	/*
	 * OK to call unconditionally, since local stations (IBSS BSSID
	 * STA and broadcast STA) pass in a NULL sta, and mac80211
	 * doesn't allow HT IBSS.
	 */
	il_set_ht_add_station(il, sta_id, sta);

	/* 3945 only */
	rate = (il->band == NL80211_BAND_5GHZ) ? RATE_6M_PLCP : RATE_1M_PLCP;
	/* Turn on both antennas for the station... */
	station->sta.rate_n_flags = cpu_to_le16(rate | RATE_MCS_ANT_AB_MSK);

	return sta_id;

}
EXPORT_SYMBOL_GPL(il_prep_station);

#define STA_WAIT_TIMEOUT (HZ/2)

/**
 * il_add_station_common -
 */
int
il_add_station_common(struct il_priv *il, const u8 *addr, bool is_ap,
		      struct ieee80211_sta *sta, u8 *sta_id_r)
{
	unsigned long flags_spin;
	int ret = 0;
	u8 sta_id;
	struct il_addsta_cmd sta_cmd;

	*sta_id_r = 0;
	spin_lock_irqsave(&il->sta_lock, flags_spin);
	sta_id = il_prep_station(il, addr, is_ap, sta);
	if (sta_id == IL_INVALID_STATION) {
		IL_ERR("Unable to prepare station %pM for addition\n", addr);
		spin_unlock_irqrestore(&il->sta_lock, flags_spin);
		return -EINVAL;
	}

	/*
	 * uCode is not able to deal with multiple requests to add a
	 * station. Keep track if one is in progress so that we do not send
	 * another.
	 */
	if (il->stations[sta_id].used & IL_STA_UCODE_INPROGRESS) {
		D_INFO("STA %d already in process of being added.\n", sta_id);
		spin_unlock_irqrestore(&il->sta_lock, flags_spin);
		return -EEXIST;
	}

	if ((il->stations[sta_id].used & IL_STA_DRIVER_ACTIVE) &&
	    (il->stations[sta_id].used & IL_STA_UCODE_ACTIVE)) {
		D_ASSOC("STA %d (%pM) already added, not adding again.\n",
			sta_id, addr);
		spin_unlock_irqrestore(&il->sta_lock, flags_spin);
		return -EEXIST;
	}

	il->stations[sta_id].used |= IL_STA_UCODE_INPROGRESS;
	memcpy(&sta_cmd, &il->stations[sta_id].sta,
	       sizeof(struct il_addsta_cmd));
	spin_unlock_irqrestore(&il->sta_lock, flags_spin);

	/* Add station to device's station table */
	ret = il_send_add_sta(il, &sta_cmd, CMD_SYNC);
	if (ret) {
		spin_lock_irqsave(&il->sta_lock, flags_spin);
		IL_ERR("Adding station %pM failed.\n",
		       il->stations[sta_id].sta.sta.addr);
		il->stations[sta_id].used &= ~IL_STA_DRIVER_ACTIVE;
		il->stations[sta_id].used &= ~IL_STA_UCODE_INPROGRESS;
		spin_unlock_irqrestore(&il->sta_lock, flags_spin);
	}
	*sta_id_r = sta_id;
	return ret;
}
EXPORT_SYMBOL(il_add_station_common);

/**
 * il_sta_ucode_deactivate - deactivate ucode status for a station
 *
 * il->sta_lock must be held
 */
static void
il_sta_ucode_deactivate(struct il_priv *il, u8 sta_id)
{
	/* Ucode must be active and driver must be non active */
	if ((il->stations[sta_id].
	     used & (IL_STA_UCODE_ACTIVE | IL_STA_DRIVER_ACTIVE)) !=
	    IL_STA_UCODE_ACTIVE)
		IL_ERR("removed non active STA %u\n", sta_id);

	il->stations[sta_id].used &= ~IL_STA_UCODE_ACTIVE;

	memset(&il->stations[sta_id], 0, sizeof(struct il_station_entry));
	D_ASSOC("Removed STA %u\n", sta_id);
}

static int
il_send_remove_station(struct il_priv *il, const u8 * addr, int sta_id,
		       bool temporary)
{
	struct il_rx_pkt *pkt;
	int ret;

	unsigned long flags_spin;
	struct il_rem_sta_cmd rm_sta_cmd;

	struct il_host_cmd cmd = {
		.id = C_REM_STA,
		.len = sizeof(struct il_rem_sta_cmd),
		.flags = CMD_SYNC,
		.data = &rm_sta_cmd,
	};

	memset(&rm_sta_cmd, 0, sizeof(rm_sta_cmd));
	rm_sta_cmd.num_sta = 1;
	memcpy(&rm_sta_cmd.addr, addr, ETH_ALEN);

	cmd.flags |= CMD_WANT_SKB;

	ret = il_send_cmd(il, &cmd);

	if (ret)
		return ret;

	pkt = (struct il_rx_pkt *)cmd.reply_page;
	if (pkt->hdr.flags & IL_CMD_FAILED_MSK) {
		IL_ERR("Bad return from C_REM_STA (0x%08X)\n", pkt->hdr.flags);
		ret = -EIO;
	}

	if (!ret) {
		switch (pkt->u.rem_sta.status) {
		case REM_STA_SUCCESS_MSK:
			if (!temporary) {
				spin_lock_irqsave(&il->sta_lock, flags_spin);
				il_sta_ucode_deactivate(il, sta_id);
				spin_unlock_irqrestore(&il->sta_lock,
						       flags_spin);
			}
			D_ASSOC("C_REM_STA PASSED\n");
			break;
		default:
			ret = -EIO;
			IL_ERR("C_REM_STA failed\n");
			break;
		}
	}
	il_free_pages(il, cmd.reply_page);

	return ret;
}

/**
 * il_remove_station - Remove driver's knowledge of station.
 */
int
il_remove_station(struct il_priv *il, const u8 sta_id, const u8 * addr)
{
	unsigned long flags;

	if (!il_is_ready(il)) {
		D_INFO("Unable to remove station %pM, device not ready.\n",
		       addr);
		/*
		 * It is typical for stations to be removed when we are
		 * going down. Return success since device will be down
		 * soon anyway
		 */
		return 0;
	}

	D_ASSOC("Removing STA from driver:%d  %pM\n", sta_id, addr);

	if (WARN_ON(sta_id == IL_INVALID_STATION))
		return -EINVAL;

	spin_lock_irqsave(&il->sta_lock, flags);

	if (!(il->stations[sta_id].used & IL_STA_DRIVER_ACTIVE)) {
		D_INFO("Removing %pM but non DRIVER active\n", addr);
		goto out_err;
	}

	if (!(il->stations[sta_id].used & IL_STA_UCODE_ACTIVE)) {
		D_INFO("Removing %pM but non UCODE active\n", addr);
		goto out_err;
	}

	if (il->stations[sta_id].used & IL_STA_LOCAL) {
		kfree(il->stations[sta_id].lq);
		il->stations[sta_id].lq = NULL;
	}

	il->stations[sta_id].used &= ~IL_STA_DRIVER_ACTIVE;

	il->num_stations--;

	BUG_ON(il->num_stations < 0);

	spin_unlock_irqrestore(&il->sta_lock, flags);

	return il_send_remove_station(il, addr, sta_id, false);
out_err:
	spin_unlock_irqrestore(&il->sta_lock, flags);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(il_remove_station);

/**
 * il_clear_ucode_stations - clear ucode station table bits
 *
 * This function clears all the bits in the driver indicating
 * which stations are active in the ucode. Call when something
 * other than explicit station management would cause this in
 * the ucode, e.g. unassociated RXON.
 */
void
il_clear_ucode_stations(struct il_priv *il)
{
	int i;
	unsigned long flags_spin;
	bool cleared = false;

	D_INFO("Clearing ucode stations in driver\n");

	spin_lock_irqsave(&il->sta_lock, flags_spin);
	for (i = 0; i < il->hw_params.max_stations; i++) {
		if (il->stations[i].used & IL_STA_UCODE_ACTIVE) {
			D_INFO("Clearing ucode active for station %d\n", i);
			il->stations[i].used &= ~IL_STA_UCODE_ACTIVE;
			cleared = true;
		}
	}
	spin_unlock_irqrestore(&il->sta_lock, flags_spin);

	if (!cleared)
		D_INFO("No active stations found to be cleared\n");
}
EXPORT_SYMBOL(il_clear_ucode_stations);

/**
 * il_restore_stations() - Restore driver known stations to device
 *
 * All stations considered active by driver, but not present in ucode, is
 * restored.
 *
 * Function sleeps.
 */
void
il_restore_stations(struct il_priv *il)
{
	struct il_addsta_cmd sta_cmd;
	struct il_link_quality_cmd lq;
	unsigned long flags_spin;
	int i;
	bool found = false;
	int ret;
	bool send_lq;

	if (!il_is_ready(il)) {
		D_INFO("Not ready yet, not restoring any stations.\n");
		return;
	}

	D_ASSOC("Restoring all known stations ... start.\n");
	spin_lock_irqsave(&il->sta_lock, flags_spin);
	for (i = 0; i < il->hw_params.max_stations; i++) {
		if ((il->stations[i].used & IL_STA_DRIVER_ACTIVE) &&
		    !(il->stations[i].used & IL_STA_UCODE_ACTIVE)) {
			D_ASSOC("Restoring sta %pM\n",
				il->stations[i].sta.sta.addr);
			il->stations[i].sta.mode = 0;
			il->stations[i].used |= IL_STA_UCODE_INPROGRESS;
			found = true;
		}
	}

	for (i = 0; i < il->hw_params.max_stations; i++) {
		if ((il->stations[i].used & IL_STA_UCODE_INPROGRESS)) {
			memcpy(&sta_cmd, &il->stations[i].sta,
			       sizeof(struct il_addsta_cmd));
			send_lq = false;
			if (il->stations[i].lq) {
				memcpy(&lq, il->stations[i].lq,
				       sizeof(struct il_link_quality_cmd));
				send_lq = true;
			}
			spin_unlock_irqrestore(&il->sta_lock, flags_spin);
			ret = il_send_add_sta(il, &sta_cmd, CMD_SYNC);
			if (ret) {
				spin_lock_irqsave(&il->sta_lock, flags_spin);
				IL_ERR("Adding station %pM failed.\n",
				       il->stations[i].sta.sta.addr);
				il->stations[i].used &= ~IL_STA_DRIVER_ACTIVE;
				il->stations[i].used &=
				    ~IL_STA_UCODE_INPROGRESS;
				spin_unlock_irqrestore(&il->sta_lock,
						       flags_spin);
			}
			/*
			 * Rate scaling has already been initialized, send
			 * current LQ command
			 */
			if (send_lq)
				il_send_lq_cmd(il, &lq, CMD_SYNC, true);
			spin_lock_irqsave(&il->sta_lock, flags_spin);
			il->stations[i].used &= ~IL_STA_UCODE_INPROGRESS;
		}
	}

	spin_unlock_irqrestore(&il->sta_lock, flags_spin);
	if (!found)
		D_INFO("Restoring all known stations"
		       " .... no stations to be restored.\n");
	else
		D_INFO("Restoring all known stations" " .... complete.\n");
}
EXPORT_SYMBOL(il_restore_stations);

int
il_get_free_ucode_key_idx(struct il_priv *il)
{
	int i;

	for (i = 0; i < il->sta_key_max_num; i++)
		if (!test_and_set_bit(i, &il->ucode_key_table))
			return i;

	return WEP_INVALID_OFFSET;
}
EXPORT_SYMBOL(il_get_free_ucode_key_idx);

void
il_dealloc_bcast_stations(struct il_priv *il)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&il->sta_lock, flags);
	for (i = 0; i < il->hw_params.max_stations; i++) {
		if (!(il->stations[i].used & IL_STA_BCAST))
			continue;

		il->stations[i].used &= ~IL_STA_UCODE_ACTIVE;
		il->num_stations--;
		BUG_ON(il->num_stations < 0);
		kfree(il->stations[i].lq);
		il->stations[i].lq = NULL;
	}
	spin_unlock_irqrestore(&il->sta_lock, flags);
}
EXPORT_SYMBOL_GPL(il_dealloc_bcast_stations);

#ifdef CONFIG_IWLEGACY_DEBUG
static void
il_dump_lq_cmd(struct il_priv *il, struct il_link_quality_cmd *lq)
{
	int i;
	D_RATE("lq station id 0x%x\n", lq->sta_id);
	D_RATE("lq ant 0x%X 0x%X\n", lq->general_params.single_stream_ant_msk,
	       lq->general_params.dual_stream_ant_msk);

	for (i = 0; i < LINK_QUAL_MAX_RETRY_NUM; i++)
		D_RATE("lq idx %d 0x%X\n", i, lq->rs_table[i].rate_n_flags);
}
#else
static inline void
il_dump_lq_cmd(struct il_priv *il, struct il_link_quality_cmd *lq)
{
}
#endif

/**
 * il_is_lq_table_valid() - Test one aspect of LQ cmd for validity
 *
 * It sometimes happens when a HT rate has been in use and we
 * loose connectivity with AP then mac80211 will first tell us that the
 * current channel is not HT anymore before removing the station. In such a
 * scenario the RXON flags will be updated to indicate we are not
 * communicating HT anymore, but the LQ command may still contain HT rates.
 * Test for this to prevent driver from sending LQ command between the time
 * RXON flags are updated and when LQ command is updated.
 */
static bool
il_is_lq_table_valid(struct il_priv *il, struct il_link_quality_cmd *lq)
{
	int i;

	if (il->ht.enabled)
		return true;

	D_INFO("Channel %u is not an HT channel\n", il->active.channel);
	for (i = 0; i < LINK_QUAL_MAX_RETRY_NUM; i++) {
		if (le32_to_cpu(lq->rs_table[i].rate_n_flags) & RATE_MCS_HT_MSK) {
			D_INFO("idx %d of LQ expects HT channel\n", i);
			return false;
		}
	}
	return true;
}

/**
 * il_send_lq_cmd() - Send link quality command
 * @init: This command is sent as part of station initialization right
 *        after station has been added.
 *
 * The link quality command is sent as the last step of station creation.
 * This is the special case in which init is set and we call a callback in
 * this case to clear the state indicating that station creation is in
 * progress.
 */
int
il_send_lq_cmd(struct il_priv *il, struct il_link_quality_cmd *lq,
	       u8 flags, bool init)
{
	int ret = 0;
	unsigned long flags_spin;

	struct il_host_cmd cmd = {
		.id = C_TX_LINK_QUALITY_CMD,
		.len = sizeof(struct il_link_quality_cmd),
		.flags = flags,
		.data = lq,
	};

	if (WARN_ON(lq->sta_id == IL_INVALID_STATION))
		return -EINVAL;

	spin_lock_irqsave(&il->sta_lock, flags_spin);
	if (!(il->stations[lq->sta_id].used & IL_STA_DRIVER_ACTIVE)) {
		spin_unlock_irqrestore(&il->sta_lock, flags_spin);
		return -EINVAL;
	}
	spin_unlock_irqrestore(&il->sta_lock, flags_spin);

	il_dump_lq_cmd(il, lq);
	BUG_ON(init && (cmd.flags & CMD_ASYNC));

	if (il_is_lq_table_valid(il, lq))
		ret = il_send_cmd(il, &cmd);
	else
		ret = -EINVAL;

	if (cmd.flags & CMD_ASYNC)
		return ret;

	if (init) {
		D_INFO("init LQ command complete,"
		       " clearing sta addition status for sta %d\n",
		       lq->sta_id);
		spin_lock_irqsave(&il->sta_lock, flags_spin);
		il->stations[lq->sta_id].used &= ~IL_STA_UCODE_INPROGRESS;
		spin_unlock_irqrestore(&il->sta_lock, flags_spin);
	}
	return ret;
}
EXPORT_SYMBOL(il_send_lq_cmd);

int
il_mac_sta_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		  struct ieee80211_sta *sta)
{
	struct il_priv *il = hw->priv;
	struct il_station_priv_common *sta_common = (void *)sta->drv_priv;
	int ret;

	mutex_lock(&il->mutex);
	D_MAC80211("enter station %pM\n", sta->addr);

	ret = il_remove_station(il, sta_common->sta_id, sta->addr);
	if (ret)
		IL_ERR("Error removing station %pM\n", sta->addr);

	D_MAC80211("leave ret %d\n", ret);
	mutex_unlock(&il->mutex);

	return ret;
}
EXPORT_SYMBOL(il_mac_sta_remove);

/************************** RX-FUNCTIONS ****************************/
/*
 * Rx theory of operation
 *
 * Driver allocates a circular buffer of Receive Buffer Descriptors (RBDs),
 * each of which point to Receive Buffers to be filled by the NIC.  These get
 * used not only for Rx frames, but for any command response or notification
 * from the NIC.  The driver and NIC manage the Rx buffers by means
 * of idxes into the circular buffer.
 *
 * Rx Queue Indexes
 * The host/firmware share two idx registers for managing the Rx buffers.
 *
 * The READ idx maps to the first position that the firmware may be writing
 * to -- the driver can read up to (but not including) this position and get
 * good data.
 * The READ idx is managed by the firmware once the card is enabled.
 *
 * The WRITE idx maps to the last position the driver has read from -- the
 * position preceding WRITE is the last slot the firmware can place a packet.
 *
 * The queue is empty (no good data) if WRITE = READ - 1, and is full if
 * WRITE = READ.
 *
 * During initialization, the host sets up the READ queue position to the first
 * IDX position, and WRITE to the last (READ - 1 wrapped)
 *
 * When the firmware places a packet in a buffer, it will advance the READ idx
 * and fire the RX interrupt.  The driver can then query the READ idx and
 * process as many packets as possible, moving the WRITE idx forward as it
 * resets the Rx queue buffers with new memory.
 *
 * The management in the driver is as follows:
 * + A list of pre-allocated SKBs is stored in iwl->rxq->rx_free.  When
 *   iwl->rxq->free_count drops to or below RX_LOW_WATERMARK, work is scheduled
 *   to replenish the iwl->rxq->rx_free.
 * + In il_rx_replenish (scheduled) if 'processed' != 'read' then the
 *   iwl->rxq is replenished and the READ IDX is updated (updating the
 *   'processed' and 'read' driver idxes as well)
 * + A received packet is processed and handed to the kernel network stack,
 *   detached from the iwl->rxq.  The driver 'processed' idx is updated.
 * + The Host/Firmware iwl->rxq is replenished at tasklet time from the rx_free
 *   list. If there are no allocated buffers in iwl->rxq->rx_free, the READ
 *   IDX is not incremented and iwl->status(RX_STALLED) is set.  If there
 *   were enough free buffers and RX_STALLED is set it is cleared.
 *
 *
 * Driver sequence:
 *
 * il_rx_queue_alloc()   Allocates rx_free
 * il_rx_replenish()     Replenishes rx_free list from rx_used, and calls
 *                            il_rx_queue_restock
 * il_rx_queue_restock() Moves available buffers from rx_free into Rx
 *                            queue, updates firmware pointers, and updates
 *                            the WRITE idx.  If insufficient rx_free buffers
 *                            are available, schedules il_rx_replenish
 *
 * -- enable interrupts --
 * ISR - il_rx()         Detach il_rx_bufs from pool up to the
 *                            READ IDX, detaching the SKB from the pool.
 *                            Moves the packet buffer from queue to rx_used.
 *                            Calls il_rx_queue_restock to refill any empty
 *                            slots.
 * ...
 *
 */

/**
 * il_rx_queue_space - Return number of free slots available in queue.
 */
int
il_rx_queue_space(const struct il_rx_queue *q)
{
	int s = q->read - q->write;
	if (s <= 0)
		s += RX_QUEUE_SIZE;
	/* keep some buffer to not confuse full and empty queue */
	s -= 2;
	if (s < 0)
		s = 0;
	return s;
}
EXPORT_SYMBOL(il_rx_queue_space);

/**
 * il_rx_queue_update_write_ptr - Update the write pointer for the RX queue
 */
void
il_rx_queue_update_write_ptr(struct il_priv *il, struct il_rx_queue *q)
{
	unsigned long flags;
	u32 rx_wrt_ptr_reg = il->hw_params.rx_wrt_ptr_reg;
	u32 reg;

	spin_lock_irqsave(&q->lock, flags);

	if (q->need_update == 0)
		goto exit_unlock;

	/* If power-saving is in use, make sure device is awake */
	if (test_bit(S_POWER_PMI, &il->status)) {
		reg = _il_rd(il, CSR_UCODE_DRV_GP1);

		if (reg & CSR_UCODE_DRV_GP1_BIT_MAC_SLEEP) {
			D_INFO("Rx queue requesting wakeup," " GP1 = 0x%x\n",
			       reg);
			il_set_bit(il, CSR_GP_CNTRL,
				   CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
			goto exit_unlock;
		}

		q->write_actual = (q->write & ~0x7);
		il_wr(il, rx_wrt_ptr_reg, q->write_actual);

		/* Else device is assumed to be awake */
	} else {
		/* Device expects a multiple of 8 */
		q->write_actual = (q->write & ~0x7);
		il_wr(il, rx_wrt_ptr_reg, q->write_actual);
	}

	q->need_update = 0;

exit_unlock:
	spin_unlock_irqrestore(&q->lock, flags);
}
EXPORT_SYMBOL(il_rx_queue_update_write_ptr);

int
il_rx_queue_alloc(struct il_priv *il)
{
	struct il_rx_queue *rxq = &il->rxq;
	struct device *dev = &il->pci_dev->dev;
	int i;

	spin_lock_init(&rxq->lock);
	INIT_LIST_HEAD(&rxq->rx_free);
	INIT_LIST_HEAD(&rxq->rx_used);

	/* Alloc the circular buffer of Read Buffer Descriptors (RBDs) */
	rxq->bd = dma_alloc_coherent(dev, 4 * RX_QUEUE_SIZE, &rxq->bd_dma,
				     GFP_KERNEL);
	if (!rxq->bd)
		goto err_bd;

	rxq->rb_stts = dma_alloc_coherent(dev, sizeof(struct il_rb_status),
					  &rxq->rb_stts_dma, GFP_KERNEL);
	if (!rxq->rb_stts)
		goto err_rb;

	/* Fill the rx_used queue with _all_ of the Rx buffers */
	for (i = 0; i < RX_FREE_BUFFERS + RX_QUEUE_SIZE; i++)
		list_add_tail(&rxq->pool[i].list, &rxq->rx_used);

	/* Set us so that we have processed and used all buffers, but have
	 * not restocked the Rx queue with fresh buffers */
	rxq->read = rxq->write = 0;
	rxq->write_actual = 0;
	rxq->free_count = 0;
	rxq->need_update = 0;
	return 0;

err_rb:
	dma_free_coherent(&il->pci_dev->dev, 4 * RX_QUEUE_SIZE, rxq->bd,
			  rxq->bd_dma);
err_bd:
	return -ENOMEM;
}
EXPORT_SYMBOL(il_rx_queue_alloc);

void
il_hdl_spectrum_measurement(struct il_priv *il, struct il_rx_buf *rxb)
{
	struct il_rx_pkt *pkt = rxb_addr(rxb);
	struct il_spectrum_notification *report = &(pkt->u.spectrum_notif);

	if (!report->state) {
		D_11H("Spectrum Measure Notification: Start\n");
		return;
	}

	memcpy(&il->measure_report, report, sizeof(*report));
	il->measurement_status |= MEASUREMENT_READY;
}
EXPORT_SYMBOL(il_hdl_spectrum_measurement);

/*
 * returns non-zero if packet should be dropped
 */
int
il_set_decrypted_flag(struct il_priv *il, struct ieee80211_hdr *hdr,
		      u32 decrypt_res, struct ieee80211_rx_status *stats)
{
	u16 fc = le16_to_cpu(hdr->frame_control);

	/*
	 * All contexts have the same setting here due to it being
	 * a module parameter, so OK to check any context.
	 */
	if (il->active.filter_flags & RXON_FILTER_DIS_DECRYPT_MSK)
		return 0;

	if (!(fc & IEEE80211_FCTL_PROTECTED))
		return 0;

	D_RX("decrypt_res:0x%x\n", decrypt_res);
	switch (decrypt_res & RX_RES_STATUS_SEC_TYPE_MSK) {
	case RX_RES_STATUS_SEC_TYPE_TKIP:
		/* The uCode has got a bad phase 1 Key, pushes the packet.
		 * Decryption will be done in SW. */
		if ((decrypt_res & RX_RES_STATUS_DECRYPT_TYPE_MSK) ==
		    RX_RES_STATUS_BAD_KEY_TTAK)
			break;

	case RX_RES_STATUS_SEC_TYPE_WEP:
		if ((decrypt_res & RX_RES_STATUS_DECRYPT_TYPE_MSK) ==
		    RX_RES_STATUS_BAD_ICV_MIC) {
			/* bad ICV, the packet is destroyed since the
			 * decryption is inplace, drop it */
			D_RX("Packet destroyed\n");
			return -1;
		}
	case RX_RES_STATUS_SEC_TYPE_CCMP:
		if ((decrypt_res & RX_RES_STATUS_DECRYPT_TYPE_MSK) ==
		    RX_RES_STATUS_DECRYPT_OK) {
			D_RX("hw decrypt successfully!!!\n");
			stats->flag |= RX_FLAG_DECRYPTED;
		}
		break;

	default:
		break;
	}
	return 0;
}
EXPORT_SYMBOL(il_set_decrypted_flag);

/**
 * il_txq_update_write_ptr - Send new write idx to hardware
 */
void
il_txq_update_write_ptr(struct il_priv *il, struct il_tx_queue *txq)
{
	u32 reg = 0;
	int txq_id = txq->q.id;

	if (txq->need_update == 0)
		return;

	/* if we're trying to save power */
	if (test_bit(S_POWER_PMI, &il->status)) {
		/* wake up nic if it's powered down ...
		 * uCode will wake up, and interrupt us again, so next
		 * time we'll skip this part. */
		reg = _il_rd(il, CSR_UCODE_DRV_GP1);

		if (reg & CSR_UCODE_DRV_GP1_BIT_MAC_SLEEP) {
			D_INFO("Tx queue %d requesting wakeup," " GP1 = 0x%x\n",
			       txq_id, reg);
			il_set_bit(il, CSR_GP_CNTRL,
				   CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
			return;
		}

		il_wr(il, HBUS_TARG_WRPTR, txq->q.write_ptr | (txq_id << 8));

		/*
		 * else not in power-save mode,
		 * uCode will never sleep when we're
		 * trying to tx (during RFKILL, we're not trying to tx).
		 */
	} else
		_il_wr(il, HBUS_TARG_WRPTR, txq->q.write_ptr | (txq_id << 8));
	txq->need_update = 0;
}
EXPORT_SYMBOL(il_txq_update_write_ptr);

/**
 * il_tx_queue_unmap -  Unmap any remaining DMA mappings and free skb's
 */
void
il_tx_queue_unmap(struct il_priv *il, int txq_id)
{
	struct il_tx_queue *txq = &il->txq[txq_id];
	struct il_queue *q = &txq->q;

	if (q->n_bd == 0)
		return;

	while (q->write_ptr != q->read_ptr) {
		il->ops->txq_free_tfd(il, txq);
		q->read_ptr = il_queue_inc_wrap(q->read_ptr, q->n_bd);
	}
}
EXPORT_SYMBOL(il_tx_queue_unmap);

/**
 * il_tx_queue_free - Deallocate DMA queue.
 * @txq: Transmit queue to deallocate.
 *
 * Empty queue by removing and destroying all BD's.
 * Free all buffers.
 * 0-fill, but do not free "txq" descriptor structure.
 */
void
il_tx_queue_free(struct il_priv *il, int txq_id)
{
	struct il_tx_queue *txq = &il->txq[txq_id];
	struct device *dev = &il->pci_dev->dev;
	int i;

	il_tx_queue_unmap(il, txq_id);

	/* De-alloc array of command/tx buffers */
	if (txq->cmd) {
		for (i = 0; i < TFD_TX_CMD_SLOTS; i++)
			kfree(txq->cmd[i]);
	}

	/* De-alloc circular buffer of TFDs */
	if (txq->q.n_bd)
		dma_free_coherent(dev, il->hw_params.tfd_size * txq->q.n_bd,
				  txq->tfds, txq->q.dma_addr);

	/* De-alloc array of per-TFD driver data */
	kfree(txq->skbs);
	txq->skbs = NULL;

	/* deallocate arrays */
	kfree(txq->cmd);
	kfree(txq->meta);
	txq->cmd = NULL;
	txq->meta = NULL;

	/* 0-fill queue descriptor structure */
	memset(txq, 0, sizeof(*txq));
}
EXPORT_SYMBOL(il_tx_queue_free);

/**
 * il_cmd_queue_unmap - Unmap any remaining DMA mappings from command queue
 */
void
il_cmd_queue_unmap(struct il_priv *il)
{
	struct il_tx_queue *txq = &il->txq[il->cmd_queue];
	struct il_queue *q = &txq->q;
	int i;

	if (q->n_bd == 0)
		return;

	while (q->read_ptr != q->write_ptr) {
		i = il_get_cmd_idx(q, q->read_ptr, 0);

		if (txq->meta[i].flags & CMD_MAPPED) {
			pci_unmap_single(il->pci_dev,
					 dma_unmap_addr(&txq->meta[i], mapping),
					 dma_unmap_len(&txq->meta[i], len),
					 PCI_DMA_BIDIRECTIONAL);
			txq->meta[i].flags = 0;
		}

		q->read_ptr = il_queue_inc_wrap(q->read_ptr, q->n_bd);
	}

	i = q->n_win;
	if (txq->meta[i].flags & CMD_MAPPED) {
		pci_unmap_single(il->pci_dev,
				 dma_unmap_addr(&txq->meta[i], mapping),
				 dma_unmap_len(&txq->meta[i], len),
				 PCI_DMA_BIDIRECTIONAL);
		txq->meta[i].flags = 0;
	}
}
EXPORT_SYMBOL(il_cmd_queue_unmap);

/**
 * il_cmd_queue_free - Deallocate DMA queue.
 * @txq: Transmit queue to deallocate.
 *
 * Empty queue by removing and destroying all BD's.
 * Free all buffers.
 * 0-fill, but do not free "txq" descriptor structure.
 */
void
il_cmd_queue_free(struct il_priv *il)
{
	struct il_tx_queue *txq = &il->txq[il->cmd_queue];
	struct device *dev = &il->pci_dev->dev;
	int i;

	il_cmd_queue_unmap(il);

	/* De-alloc array of command/tx buffers */
	if (txq->cmd) {
		for (i = 0; i <= TFD_CMD_SLOTS; i++)
			kfree(txq->cmd[i]);
	}

	/* De-alloc circular buffer of TFDs */
	if (txq->q.n_bd)
		dma_free_coherent(dev, il->hw_params.tfd_size * txq->q.n_bd,
				  txq->tfds, txq->q.dma_addr);

	/* deallocate arrays */
	kfree(txq->cmd);
	kfree(txq->meta);
	txq->cmd = NULL;
	txq->meta = NULL;

	/* 0-fill queue descriptor structure */
	memset(txq, 0, sizeof(*txq));
}
EXPORT_SYMBOL(il_cmd_queue_free);

/*************** DMA-QUEUE-GENERAL-FUNCTIONS  *****
 * DMA services
 *
 * Theory of operation
 *
 * A Tx or Rx queue resides in host DRAM, and is comprised of a circular buffer
 * of buffer descriptors, each of which points to one or more data buffers for
 * the device to read from or fill.  Driver and device exchange status of each
 * queue via "read" and "write" pointers.  Driver keeps minimum of 2 empty
 * entries in each circular buffer, to protect against confusing empty and full
 * queue states.
 *
 * The device reads or writes the data in the queues via the device's several
 * DMA/FIFO channels.  Each queue is mapped to a single DMA channel.
 *
 * For Tx queue, there are low mark and high mark limits. If, after queuing
 * the packet for Tx, free space become < low mark, Tx queue stopped. When
 * reclaiming packets (on 'tx done IRQ), if free space become > high mark,
 * Tx queue resumed.
 *
 * See more detailed info in 4965.h.
 ***************************************************/

int
il_queue_space(const struct il_queue *q)
{
	int s = q->read_ptr - q->write_ptr;

	if (q->read_ptr > q->write_ptr)
		s -= q->n_bd;

	if (s <= 0)
		s += q->n_win;
	/* keep some reserve to not confuse empty and full situations */
	s -= 2;
	if (s < 0)
		s = 0;
	return s;
}
EXPORT_SYMBOL(il_queue_space);


/**
 * il_queue_init - Initialize queue's high/low-water and read/write idxes
 */
static int
il_queue_init(struct il_priv *il, struct il_queue *q, int slots, u32 id)
{
	/*
	 * TFD_QUEUE_SIZE_MAX must be power-of-two size, otherwise
	 * il_queue_inc_wrap and il_queue_dec_wrap are broken.
	 */
	BUILD_BUG_ON(TFD_QUEUE_SIZE_MAX & (TFD_QUEUE_SIZE_MAX - 1));
	/* FIXME: remove q->n_bd */
	q->n_bd = TFD_QUEUE_SIZE_MAX;

	q->n_win = slots;
	q->id = id;

	/* slots_must be power-of-two size, otherwise
	 * il_get_cmd_idx is broken. */
	BUG_ON(!is_power_of_2(slots));

	q->low_mark = q->n_win / 4;
	if (q->low_mark < 4)
		q->low_mark = 4;

	q->high_mark = q->n_win / 8;
	if (q->high_mark < 2)
		q->high_mark = 2;

	q->write_ptr = q->read_ptr = 0;

	return 0;
}

/**
 * il_tx_queue_alloc - Alloc driver data and TFD CB for one Tx/cmd queue
 */
static int
il_tx_queue_alloc(struct il_priv *il, struct il_tx_queue *txq, u32 id)
{
	struct device *dev = &il->pci_dev->dev;
	size_t tfd_sz = il->hw_params.tfd_size * TFD_QUEUE_SIZE_MAX;

	/* Driver ilate data, only for Tx (not command) queues,
	 * not shared with device. */
	if (id != il->cmd_queue) {
		txq->skbs = kcalloc(TFD_QUEUE_SIZE_MAX,
				    sizeof(struct sk_buff *),
				    GFP_KERNEL);
		if (!txq->skbs) {
			IL_ERR("Fail to alloc skbs\n");
			goto error;
		}
	} else
		txq->skbs = NULL;

	/* Circular buffer of transmit frame descriptors (TFDs),
	 * shared with device */
	txq->tfds =
	    dma_alloc_coherent(dev, tfd_sz, &txq->q.dma_addr, GFP_KERNEL);
	if (!txq->tfds)
		goto error;

	txq->q.id = id;

	return 0;

error:
	kfree(txq->skbs);
	txq->skbs = NULL;

	return -ENOMEM;
}

/**
 * il_tx_queue_init - Allocate and initialize one tx/cmd queue
 */
int
il_tx_queue_init(struct il_priv *il, u32 txq_id)
{
	int i, len, ret;
	int slots, actual_slots;
	struct il_tx_queue *txq = &il->txq[txq_id];

	/*
	 * Alloc buffer array for commands (Tx or other types of commands).
	 * For the command queue (#4/#9), allocate command space + one big
	 * command for scan, since scan command is very huge; the system will
	 * not have two scans at the same time, so only one is needed.
	 * For normal Tx queues (all other queues), no super-size command
	 * space is needed.
	 */
	if (txq_id == il->cmd_queue) {
		slots = TFD_CMD_SLOTS;
		actual_slots = slots + 1;
	} else {
		slots = TFD_TX_CMD_SLOTS;
		actual_slots = slots;
	}

	txq->meta =
	    kzalloc(sizeof(struct il_cmd_meta) * actual_slots, GFP_KERNEL);
	txq->cmd =
	    kzalloc(sizeof(struct il_device_cmd *) * actual_slots, GFP_KERNEL);

	if (!txq->meta || !txq->cmd)
		goto out_free_arrays;

	len = sizeof(struct il_device_cmd);
	for (i = 0; i < actual_slots; i++) {
		/* only happens for cmd queue */
		if (i == slots)
			len = IL_MAX_CMD_SIZE;

		txq->cmd[i] = kmalloc(len, GFP_KERNEL);
		if (!txq->cmd[i])
			goto err;
	}

	/* Alloc driver data array and TFD circular buffer */
	ret = il_tx_queue_alloc(il, txq, txq_id);
	if (ret)
		goto err;

	txq->need_update = 0;

	/*
	 * For the default queues 0-3, set up the swq_id
	 * already -- all others need to get one later
	 * (if they need one at all).
	 */
	if (txq_id < 4)
		il_set_swq_id(txq, txq_id, txq_id);

	/* Initialize queue's high/low-water marks, and head/tail idxes */
	il_queue_init(il, &txq->q, slots, txq_id);

	/* Tell device where to find queue */
	il->ops->txq_init(il, txq);

	return 0;
err:
	for (i = 0; i < actual_slots; i++)
		kfree(txq->cmd[i]);
out_free_arrays:
	kfree(txq->meta);
	txq->meta = NULL;
	kfree(txq->cmd);
	txq->cmd = NULL;

	return -ENOMEM;
}
EXPORT_SYMBOL(il_tx_queue_init);

void
il_tx_queue_reset(struct il_priv *il, u32 txq_id)
{
	int slots, actual_slots;
	struct il_tx_queue *txq = &il->txq[txq_id];

	if (txq_id == il->cmd_queue) {
		slots = TFD_CMD_SLOTS;
		actual_slots = TFD_CMD_SLOTS + 1;
	} else {
		slots = TFD_TX_CMD_SLOTS;
		actual_slots = TFD_TX_CMD_SLOTS;
	}

	memset(txq->meta, 0, sizeof(struct il_cmd_meta) * actual_slots);
	txq->need_update = 0;

	/* Initialize queue's high/low-water marks, and head/tail idxes */
	il_queue_init(il, &txq->q, slots, txq_id);

	/* Tell device where to find queue */
	il->ops->txq_init(il, txq);
}
EXPORT_SYMBOL(il_tx_queue_reset);

/*************** HOST COMMAND QUEUE FUNCTIONS   *****/

/**
 * il_enqueue_hcmd - enqueue a uCode command
 * @il: device ilate data point
 * @cmd: a point to the ucode command structure
 *
 * The function returns < 0 values to indicate the operation is
 * failed. On success, it turns the idx (> 0) of command in the
 * command queue.
 */
int
il_enqueue_hcmd(struct il_priv *il, struct il_host_cmd *cmd)
{
	struct il_tx_queue *txq = &il->txq[il->cmd_queue];
	struct il_queue *q = &txq->q;
	struct il_device_cmd *out_cmd;
	struct il_cmd_meta *out_meta;
	dma_addr_t phys_addr;
	unsigned long flags;
	int len;
	u32 idx;
	u16 fix_size;

	cmd->len = il->ops->get_hcmd_size(cmd->id, cmd->len);
	fix_size = (u16) (cmd->len + sizeof(out_cmd->hdr));

	/* If any of the command structures end up being larger than
	 * the TFD_MAX_PAYLOAD_SIZE, and it sent as a 'small' command then
	 * we will need to increase the size of the TFD entries
	 * Also, check to see if command buffer should not exceed the size
	 * of device_cmd and max_cmd_size. */
	BUG_ON((fix_size > TFD_MAX_PAYLOAD_SIZE) &&
	       !(cmd->flags & CMD_SIZE_HUGE));
	BUG_ON(fix_size > IL_MAX_CMD_SIZE);

	if (il_is_rfkill(il) || il_is_ctkill(il)) {
		IL_WARN("Not sending command - %s KILL\n",
			il_is_rfkill(il) ? "RF" : "CT");
		return -EIO;
	}

	spin_lock_irqsave(&il->hcmd_lock, flags);

	if (il_queue_space(q) < ((cmd->flags & CMD_ASYNC) ? 2 : 1)) {
		spin_unlock_irqrestore(&il->hcmd_lock, flags);

		IL_ERR("Restarting adapter due to command queue full\n");
		queue_work(il->workqueue, &il->restart);
		return -ENOSPC;
	}

	idx = il_get_cmd_idx(q, q->write_ptr, cmd->flags & CMD_SIZE_HUGE);
	out_cmd = txq->cmd[idx];
	out_meta = &txq->meta[idx];

	if (WARN_ON(out_meta->flags & CMD_MAPPED)) {
		spin_unlock_irqrestore(&il->hcmd_lock, flags);
		return -ENOSPC;
	}

	memset(out_meta, 0, sizeof(*out_meta));	/* re-initialize to NULL */
	out_meta->flags = cmd->flags | CMD_MAPPED;
	if (cmd->flags & CMD_WANT_SKB)
		out_meta->source = cmd;
	if (cmd->flags & CMD_ASYNC)
		out_meta->callback = cmd->callback;

	out_cmd->hdr.cmd = cmd->id;
	memcpy(&out_cmd->cmd.payload, cmd->data, cmd->len);

	/* At this point, the out_cmd now has all of the incoming cmd
	 * information */

	out_cmd->hdr.flags = 0;
	out_cmd->hdr.sequence =
	    cpu_to_le16(QUEUE_TO_SEQ(il->cmd_queue) | IDX_TO_SEQ(q->write_ptr));
	if (cmd->flags & CMD_SIZE_HUGE)
		out_cmd->hdr.sequence |= SEQ_HUGE_FRAME;
	len = sizeof(struct il_device_cmd);
	if (idx == TFD_CMD_SLOTS)
		len = IL_MAX_CMD_SIZE;

#ifdef CONFIG_IWLEGACY_DEBUG
	switch (out_cmd->hdr.cmd) {
	case C_TX_LINK_QUALITY_CMD:
	case C_SENSITIVITY:
		D_HC_DUMP("Sending command %s (#%x), seq: 0x%04X, "
			  "%d bytes at %d[%d]:%d\n",
			  il_get_cmd_string(out_cmd->hdr.cmd), out_cmd->hdr.cmd,
			  le16_to_cpu(out_cmd->hdr.sequence), fix_size,
			  q->write_ptr, idx, il->cmd_queue);
		break;
	default:
		D_HC("Sending command %s (#%x), seq: 0x%04X, "
		     "%d bytes at %d[%d]:%d\n",
		     il_get_cmd_string(out_cmd->hdr.cmd), out_cmd->hdr.cmd,
		     le16_to_cpu(out_cmd->hdr.sequence), fix_size, q->write_ptr,
		     idx, il->cmd_queue);
	}
#endif

	phys_addr =
	    pci_map_single(il->pci_dev, &out_cmd->hdr, fix_size,
			   PCI_DMA_BIDIRECTIONAL);
	if (unlikely(pci_dma_mapping_error(il->pci_dev, phys_addr))) {
		idx = -ENOMEM;
		goto out;
	}
	dma_unmap_addr_set(out_meta, mapping, phys_addr);
	dma_unmap_len_set(out_meta, len, fix_size);

	txq->need_update = 1;

	if (il->ops->txq_update_byte_cnt_tbl)
		/* Set up entry in queue's byte count circular buffer */
		il->ops->txq_update_byte_cnt_tbl(il, txq, 0);

	il->ops->txq_attach_buf_to_tfd(il, txq, phys_addr, fix_size, 1,
					    U32_PAD(cmd->len));

	/* Increment and update queue's write idx */
	q->write_ptr = il_queue_inc_wrap(q->write_ptr, q->n_bd);
	il_txq_update_write_ptr(il, txq);

out:
	spin_unlock_irqrestore(&il->hcmd_lock, flags);
	return idx;
}

/**
 * il_hcmd_queue_reclaim - Reclaim TX command queue entries already Tx'd
 *
 * When FW advances 'R' idx, all entries between old and new 'R' idx
 * need to be reclaimed. As result, some free space forms.  If there is
 * enough free space (> low mark), wake the stack that feeds us.
 */
static void
il_hcmd_queue_reclaim(struct il_priv *il, int txq_id, int idx, int cmd_idx)
{
	struct il_tx_queue *txq = &il->txq[txq_id];
	struct il_queue *q = &txq->q;
	int nfreed = 0;

	if (idx >= q->n_bd || il_queue_used(q, idx) == 0) {
		IL_ERR("Read idx for DMA queue txq id (%d), idx %d, "
		       "is out of range [0-%d] %d %d.\n", txq_id, idx, q->n_bd,
		       q->write_ptr, q->read_ptr);
		return;
	}

	for (idx = il_queue_inc_wrap(idx, q->n_bd); q->read_ptr != idx;
	     q->read_ptr = il_queue_inc_wrap(q->read_ptr, q->n_bd)) {

		if (nfreed++ > 0) {
			IL_ERR("HCMD skipped: idx (%d) %d %d\n", idx,
			       q->write_ptr, q->read_ptr);
			queue_work(il->workqueue, &il->restart);
		}

	}
}

/**
 * il_tx_cmd_complete - Pull unused buffers off the queue and reclaim them
 * @rxb: Rx buffer to reclaim
 *
 * If an Rx buffer has an async callback associated with it the callback
 * will be executed.  The attached skb (if present) will only be freed
 * if the callback returns 1
 */
void
il_tx_cmd_complete(struct il_priv *il, struct il_rx_buf *rxb)
{
	struct il_rx_pkt *pkt = rxb_addr(rxb);
	u16 sequence = le16_to_cpu(pkt->hdr.sequence);
	int txq_id = SEQ_TO_QUEUE(sequence);
	int idx = SEQ_TO_IDX(sequence);
	int cmd_idx;
	bool huge = !!(pkt->hdr.sequence & SEQ_HUGE_FRAME);
	struct il_device_cmd *cmd;
	struct il_cmd_meta *meta;
	struct il_tx_queue *txq = &il->txq[il->cmd_queue];
	unsigned long flags;

	/* If a Tx command is being handled and it isn't in the actual
	 * command queue then there a command routing bug has been introduced
	 * in the queue management code. */
	if (WARN
	    (txq_id != il->cmd_queue,
	     "wrong command queue %d (should be %d), sequence 0x%X readp=%d writep=%d\n",
	     txq_id, il->cmd_queue, sequence, il->txq[il->cmd_queue].q.read_ptr,
	     il->txq[il->cmd_queue].q.write_ptr)) {
		il_print_hex_error(il, pkt, 32);
		return;
	}

	cmd_idx = il_get_cmd_idx(&txq->q, idx, huge);
	cmd = txq->cmd[cmd_idx];
	meta = &txq->meta[cmd_idx];

	txq->time_stamp = jiffies;

	pci_unmap_single(il->pci_dev, dma_unmap_addr(meta, mapping),
			 dma_unmap_len(meta, len), PCI_DMA_BIDIRECTIONAL);

	/* Input error checking is done when commands are added to queue. */
	if (meta->flags & CMD_WANT_SKB) {
		meta->source->reply_page = (unsigned long)rxb_addr(rxb);
		rxb->page = NULL;
	} else if (meta->callback)
		meta->callback(il, cmd, pkt);

	spin_lock_irqsave(&il->hcmd_lock, flags);

	il_hcmd_queue_reclaim(il, txq_id, idx, cmd_idx);

	if (!(meta->flags & CMD_ASYNC)) {
		clear_bit(S_HCMD_ACTIVE, &il->status);
		D_INFO("Clearing HCMD_ACTIVE for command %s\n",
		       il_get_cmd_string(cmd->hdr.cmd));
		wake_up(&il->wait_command_queue);
	}

	/* Mark as unmapped */
	meta->flags = 0;

	spin_unlock_irqrestore(&il->hcmd_lock, flags);
}
EXPORT_SYMBOL(il_tx_cmd_complete);

MODULE_DESCRIPTION("iwl-legacy: common functions for 3945 and 4965");
MODULE_VERSION(IWLWIFI_VERSION);
MODULE_AUTHOR(DRV_COPYRIGHT " " DRV_AUTHOR);
MODULE_LICENSE("GPL");

/*
 * set bt_coex_active to true, uCode will do kill/defer
 * every time the priority line is asserted (BT is sending signals on the
 * priority line in the PCIx).
 * set bt_coex_active to false, uCode will ignore the BT activity and
 * perform the normal operation
 *
 * User might experience transmit issue on some platform due to WiFi/BT
 * co-exist problem. The possible behaviors are:
 *   Able to scan and finding all the available AP
 *   Not able to associate with any AP
 * On those platforms, WiFi communication can be restored by set
 * "bt_coex_active" module parameter to "false"
 *
 * default: bt_coex_active = true (BT_COEX_ENABLE)
 */
static bool bt_coex_active = true;
module_param(bt_coex_active, bool, S_IRUGO);
MODULE_PARM_DESC(bt_coex_active, "enable wifi/bluetooth co-exist");

u32 il_debug_level;
EXPORT_SYMBOL(il_debug_level);

const u8 il_bcast_addr[ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
EXPORT_SYMBOL(il_bcast_addr);

#define MAX_BIT_RATE_40_MHZ 150	/* Mbps */
#define MAX_BIT_RATE_20_MHZ 72	/* Mbps */
static void
il_init_ht_hw_capab(const struct il_priv *il,
		    struct ieee80211_sta_ht_cap *ht_info,
		    enum nl80211_band band)
{
	u16 max_bit_rate = 0;
	u8 rx_chains_num = il->hw_params.rx_chains_num;
	u8 tx_chains_num = il->hw_params.tx_chains_num;

	ht_info->cap = 0;
	memset(&ht_info->mcs, 0, sizeof(ht_info->mcs));

	ht_info->ht_supported = true;

	ht_info->cap |= IEEE80211_HT_CAP_SGI_20;
	max_bit_rate = MAX_BIT_RATE_20_MHZ;
	if (il->hw_params.ht40_channel & BIT(band)) {
		ht_info->cap |= IEEE80211_HT_CAP_SUP_WIDTH_20_40;
		ht_info->cap |= IEEE80211_HT_CAP_SGI_40;
		ht_info->mcs.rx_mask[4] = 0x01;
		max_bit_rate = MAX_BIT_RATE_40_MHZ;
	}

	if (il->cfg->mod_params->amsdu_size_8K)
		ht_info->cap |= IEEE80211_HT_CAP_MAX_AMSDU;

	ht_info->ampdu_factor = CFG_HT_RX_AMPDU_FACTOR_DEF;
	ht_info->ampdu_density = CFG_HT_MPDU_DENSITY_DEF;

	ht_info->mcs.rx_mask[0] = 0xFF;
	if (rx_chains_num >= 2)
		ht_info->mcs.rx_mask[1] = 0xFF;
	if (rx_chains_num >= 3)
		ht_info->mcs.rx_mask[2] = 0xFF;

	/* Highest supported Rx data rate */
	max_bit_rate *= rx_chains_num;
	WARN_ON(max_bit_rate & ~IEEE80211_HT_MCS_RX_HIGHEST_MASK);
	ht_info->mcs.rx_highest = cpu_to_le16(max_bit_rate);

	/* Tx MCS capabilities */
	ht_info->mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;
	if (tx_chains_num != rx_chains_num) {
		ht_info->mcs.tx_params |= IEEE80211_HT_MCS_TX_RX_DIFF;
		ht_info->mcs.tx_params |=
		    ((tx_chains_num -
		      1) << IEEE80211_HT_MCS_TX_MAX_STREAMS_SHIFT);
	}
}

/**
 * il_init_geos - Initialize mac80211's geo/channel info based from eeprom
 */
int
il_init_geos(struct il_priv *il)
{
	struct il_channel_info *ch;
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel *channels;
	struct ieee80211_channel *geo_ch;
	struct ieee80211_rate *rates;
	int i = 0;
	s8 max_tx_power = 0;

	if (il->bands[NL80211_BAND_2GHZ].n_bitrates ||
	    il->bands[NL80211_BAND_5GHZ].n_bitrates) {
		D_INFO("Geography modes already initialized.\n");
		set_bit(S_GEO_CONFIGURED, &il->status);
		return 0;
	}

	channels =
	    kzalloc(sizeof(struct ieee80211_channel) * il->channel_count,
		    GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	rates =
	    kzalloc((sizeof(struct ieee80211_rate) * RATE_COUNT_LEGACY),
		    GFP_KERNEL);
	if (!rates) {
		kfree(channels);
		return -ENOMEM;
	}

	/* 5.2GHz channels start after the 2.4GHz channels */
	sband = &il->bands[NL80211_BAND_5GHZ];
	sband->channels = &channels[ARRAY_SIZE(il_eeprom_band_1)];
	/* just OFDM */
	sband->bitrates = &rates[IL_FIRST_OFDM_RATE];
	sband->n_bitrates = RATE_COUNT_LEGACY - IL_FIRST_OFDM_RATE;

	if (il->cfg->sku & IL_SKU_N)
		il_init_ht_hw_capab(il, &sband->ht_cap, NL80211_BAND_5GHZ);

	sband = &il->bands[NL80211_BAND_2GHZ];
	sband->channels = channels;
	/* OFDM & CCK */
	sband->bitrates = rates;
	sband->n_bitrates = RATE_COUNT_LEGACY;

	if (il->cfg->sku & IL_SKU_N)
		il_init_ht_hw_capab(il, &sband->ht_cap, NL80211_BAND_2GHZ);

	il->ieee_channels = channels;
	il->ieee_rates = rates;

	for (i = 0; i < il->channel_count; i++) {
		ch = &il->channel_info[i];

		if (!il_is_channel_valid(ch))
			continue;

		sband = &il->bands[ch->band];

		geo_ch = &sband->channels[sband->n_channels++];

		geo_ch->center_freq =
		    ieee80211_channel_to_frequency(ch->channel, ch->band);
		geo_ch->max_power = ch->max_power_avg;
		geo_ch->max_antenna_gain = 0xff;
		geo_ch->hw_value = ch->channel;

		if (il_is_channel_valid(ch)) {
			if (!(ch->flags & EEPROM_CHANNEL_IBSS))
				geo_ch->flags |= IEEE80211_CHAN_NO_IR;

			if (!(ch->flags & EEPROM_CHANNEL_ACTIVE))
				geo_ch->flags |= IEEE80211_CHAN_NO_IR;

			if (ch->flags & EEPROM_CHANNEL_RADAR)
				geo_ch->flags |= IEEE80211_CHAN_RADAR;

			geo_ch->flags |= ch->ht40_extension_channel;

			if (ch->max_power_avg > max_tx_power)
				max_tx_power = ch->max_power_avg;
		} else {
			geo_ch->flags |= IEEE80211_CHAN_DISABLED;
		}

		D_INFO("Channel %d Freq=%d[%sGHz] %s flag=0x%X\n", ch->channel,
		       geo_ch->center_freq,
		       il_is_channel_a_band(ch) ? "5.2" : "2.4",
		       geo_ch->
		       flags & IEEE80211_CHAN_DISABLED ? "restricted" : "valid",
		       geo_ch->flags);
	}

	il->tx_power_device_lmt = max_tx_power;
	il->tx_power_user_lmt = max_tx_power;
	il->tx_power_next = max_tx_power;

	if (il->bands[NL80211_BAND_5GHZ].n_channels == 0 &&
	    (il->cfg->sku & IL_SKU_A)) {
		IL_INFO("Incorrectly detected BG card as ABG. "
			"Please send your PCI ID 0x%04X:0x%04X to maintainer.\n",
			il->pci_dev->device, il->pci_dev->subsystem_device);
		il->cfg->sku &= ~IL_SKU_A;
	}

	IL_INFO("Tunable channels: %d 802.11bg, %d 802.11a channels\n",
		il->bands[NL80211_BAND_2GHZ].n_channels,
		il->bands[NL80211_BAND_5GHZ].n_channels);

	set_bit(S_GEO_CONFIGURED, &il->status);

	return 0;
}
EXPORT_SYMBOL(il_init_geos);

/*
 * il_free_geos - undo allocations in il_init_geos
 */
void
il_free_geos(struct il_priv *il)
{
	kfree(il->ieee_channels);
	kfree(il->ieee_rates);
	clear_bit(S_GEO_CONFIGURED, &il->status);
}
EXPORT_SYMBOL(il_free_geos);

static bool
il_is_channel_extension(struct il_priv *il, enum nl80211_band band,
			u16 channel, u8 extension_chan_offset)
{
	const struct il_channel_info *ch_info;

	ch_info = il_get_channel_info(il, band, channel);
	if (!il_is_channel_valid(ch_info))
		return false;

	if (extension_chan_offset == IEEE80211_HT_PARAM_CHA_SEC_ABOVE)
		return !(ch_info->
			 ht40_extension_channel & IEEE80211_CHAN_NO_HT40PLUS);
	else if (extension_chan_offset == IEEE80211_HT_PARAM_CHA_SEC_BELOW)
		return !(ch_info->
			 ht40_extension_channel & IEEE80211_CHAN_NO_HT40MINUS);

	return false;
}

bool
il_is_ht40_tx_allowed(struct il_priv *il, struct ieee80211_sta_ht_cap *ht_cap)
{
	if (!il->ht.enabled || !il->ht.is_40mhz)
		return false;

	/*
	 * We do not check for IEEE80211_HT_CAP_SUP_WIDTH_20_40
	 * the bit will not set if it is pure 40MHz case
	 */
	if (ht_cap && !ht_cap->ht_supported)
		return false;

#ifdef CONFIG_IWLEGACY_DEBUGFS
	if (il->disable_ht40)
		return false;
#endif

	return il_is_channel_extension(il, il->band,
				       le16_to_cpu(il->staging.channel),
				       il->ht.extension_chan_offset);
}
EXPORT_SYMBOL(il_is_ht40_tx_allowed);

static u16 noinline
il_adjust_beacon_interval(u16 beacon_val, u16 max_beacon_val)
{
	u16 new_val;
	u16 beacon_factor;

	/*
	 * If mac80211 hasn't given us a beacon interval, program
	 * the default into the device.
	 */
	if (!beacon_val)
		return DEFAULT_BEACON_INTERVAL;

	/*
	 * If the beacon interval we obtained from the peer
	 * is too large, we'll have to wake up more often
	 * (and in IBSS case, we'll beacon too much)
	 *
	 * For example, if max_beacon_val is 4096, and the
	 * requested beacon interval is 7000, we'll have to
	 * use 3500 to be able to wake up on the beacons.
	 *
	 * This could badly influence beacon detection stats.
	 */

	beacon_factor = (beacon_val + max_beacon_val) / max_beacon_val;
	new_val = beacon_val / beacon_factor;

	if (!new_val)
		new_val = max_beacon_val;

	return new_val;
}

int
il_send_rxon_timing(struct il_priv *il)
{
	u64 tsf;
	s32 interval_tm, rem;
	struct ieee80211_conf *conf = NULL;
	u16 beacon_int;
	struct ieee80211_vif *vif = il->vif;

	conf = &il->hw->conf;

	lockdep_assert_held(&il->mutex);

	memset(&il->timing, 0, sizeof(struct il_rxon_time_cmd));

	il->timing.timestamp = cpu_to_le64(il->timestamp);
	il->timing.listen_interval = cpu_to_le16(conf->listen_interval);

	beacon_int = vif ? vif->bss_conf.beacon_int : 0;

	/*
	 * TODO: For IBSS we need to get atim_win from mac80211,
	 *       for now just always use 0
	 */
	il->timing.atim_win = 0;

	beacon_int =
	    il_adjust_beacon_interval(beacon_int,
				      il->hw_params.max_beacon_itrvl *
				      TIME_UNIT);
	il->timing.beacon_interval = cpu_to_le16(beacon_int);

	tsf = il->timestamp;	/* tsf is modifed by do_div: copy it */
	interval_tm = beacon_int * TIME_UNIT;
	rem = do_div(tsf, interval_tm);
	il->timing.beacon_init_val = cpu_to_le32(interval_tm - rem);

	il->timing.dtim_period = vif ? (vif->bss_conf.dtim_period ? : 1) : 1;

	D_ASSOC("beacon interval %d beacon timer %d beacon tim %d\n",
		le16_to_cpu(il->timing.beacon_interval),
		le32_to_cpu(il->timing.beacon_init_val),
		le16_to_cpu(il->timing.atim_win));

	return il_send_cmd_pdu(il, C_RXON_TIMING, sizeof(il->timing),
			       &il->timing);
}
EXPORT_SYMBOL(il_send_rxon_timing);

void
il_set_rxon_hwcrypto(struct il_priv *il, int hw_decrypt)
{
	struct il_rxon_cmd *rxon = &il->staging;

	if (hw_decrypt)
		rxon->filter_flags &= ~RXON_FILTER_DIS_DECRYPT_MSK;
	else
		rxon->filter_flags |= RXON_FILTER_DIS_DECRYPT_MSK;

}
EXPORT_SYMBOL(il_set_rxon_hwcrypto);

/* validate RXON structure is valid */
int
il_check_rxon_cmd(struct il_priv *il)
{
	struct il_rxon_cmd *rxon = &il->staging;
	bool error = false;

	if (rxon->flags & RXON_FLG_BAND_24G_MSK) {
		if (rxon->flags & RXON_FLG_TGJ_NARROW_BAND_MSK) {
			IL_WARN("check 2.4G: wrong narrow\n");
			error = true;
		}
		if (rxon->flags & RXON_FLG_RADAR_DETECT_MSK) {
			IL_WARN("check 2.4G: wrong radar\n");
			error = true;
		}
	} else {
		if (!(rxon->flags & RXON_FLG_SHORT_SLOT_MSK)) {
			IL_WARN("check 5.2G: not short slot!\n");
			error = true;
		}
		if (rxon->flags & RXON_FLG_CCK_MSK) {
			IL_WARN("check 5.2G: CCK!\n");
			error = true;
		}
	}
	if ((rxon->node_addr[0] | rxon->bssid_addr[0]) & 0x1) {
		IL_WARN("mac/bssid mcast!\n");
		error = true;
	}

	/* make sure basic rates 6Mbps and 1Mbps are supported */
	if ((rxon->ofdm_basic_rates & RATE_6M_MASK) == 0 &&
	    (rxon->cck_basic_rates & RATE_1M_MASK) == 0) {
		IL_WARN("neither 1 nor 6 are basic\n");
		error = true;
	}

	if (le16_to_cpu(rxon->assoc_id) > 2007) {
		IL_WARN("aid > 2007\n");
		error = true;
	}

	if ((rxon->flags & (RXON_FLG_CCK_MSK | RXON_FLG_SHORT_SLOT_MSK)) ==
	    (RXON_FLG_CCK_MSK | RXON_FLG_SHORT_SLOT_MSK)) {
		IL_WARN("CCK and short slot\n");
		error = true;
	}

	if ((rxon->flags & (RXON_FLG_CCK_MSK | RXON_FLG_AUTO_DETECT_MSK)) ==
	    (RXON_FLG_CCK_MSK | RXON_FLG_AUTO_DETECT_MSK)) {
		IL_WARN("CCK and auto detect");
		error = true;
	}

	if ((rxon->
	     flags & (RXON_FLG_AUTO_DETECT_MSK | RXON_FLG_TGG_PROTECT_MSK)) ==
	    RXON_FLG_TGG_PROTECT_MSK) {
		IL_WARN("TGg but no auto-detect\n");
		error = true;
	}

	if (error)
		IL_WARN("Tuning to channel %d\n", le16_to_cpu(rxon->channel));

	if (error) {
		IL_ERR("Invalid RXON\n");
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(il_check_rxon_cmd);

/**
 * il_full_rxon_required - check if full RXON (vs RXON_ASSOC) cmd is needed
 * @il: staging_rxon is compared to active_rxon
 *
 * If the RXON structure is changing enough to require a new tune,
 * or is clearing the RXON_FILTER_ASSOC_MSK, then return 1 to indicate that
 * a new tune (full RXON command, rather than RXON_ASSOC cmd) is required.
 */
int
il_full_rxon_required(struct il_priv *il)
{
	const struct il_rxon_cmd *staging = &il->staging;
	const struct il_rxon_cmd *active = &il->active;

#define CHK(cond)							\
	if ((cond)) {							\
		D_INFO("need full RXON - " #cond "\n");	\
		return 1;						\
	}

#define CHK_NEQ(c1, c2)						\
	if ((c1) != (c2)) {					\
		D_INFO("need full RXON - "	\
			       #c1 " != " #c2 " - %d != %d\n",	\
			       (c1), (c2));			\
		return 1;					\
	}

	/* These items are only settable from the full RXON command */
	CHK(!il_is_associated(il));
	CHK(!ether_addr_equal_64bits(staging->bssid_addr, active->bssid_addr));
	CHK(!ether_addr_equal_64bits(staging->node_addr, active->node_addr));
	CHK(!ether_addr_equal_64bits(staging->wlap_bssid_addr,
				     active->wlap_bssid_addr));
	CHK_NEQ(staging->dev_type, active->dev_type);
	CHK_NEQ(staging->channel, active->channel);
	CHK_NEQ(staging->air_propagation, active->air_propagation);
	CHK_NEQ(staging->ofdm_ht_single_stream_basic_rates,
		active->ofdm_ht_single_stream_basic_rates);
	CHK_NEQ(staging->ofdm_ht_dual_stream_basic_rates,
		active->ofdm_ht_dual_stream_basic_rates);
	CHK_NEQ(staging->assoc_id, active->assoc_id);

	/* flags, filter_flags, ofdm_basic_rates, and cck_basic_rates can
	 * be updated with the RXON_ASSOC command -- however only some
	 * flag transitions are allowed using RXON_ASSOC */

	/* Check if we are not switching bands */
	CHK_NEQ(staging->flags & RXON_FLG_BAND_24G_MSK,
		active->flags & RXON_FLG_BAND_24G_MSK);

	/* Check if we are switching association toggle */
	CHK_NEQ(staging->filter_flags & RXON_FILTER_ASSOC_MSK,
		active->filter_flags & RXON_FILTER_ASSOC_MSK);

#undef CHK
#undef CHK_NEQ

	return 0;
}
EXPORT_SYMBOL(il_full_rxon_required);

u8
il_get_lowest_plcp(struct il_priv *il)
{
	/*
	 * Assign the lowest rate -- should really get this from
	 * the beacon skb from mac80211.
	 */
	if (il->staging.flags & RXON_FLG_BAND_24G_MSK)
		return RATE_1M_PLCP;
	else
		return RATE_6M_PLCP;
}
EXPORT_SYMBOL(il_get_lowest_plcp);

static void
_il_set_rxon_ht(struct il_priv *il, struct il_ht_config *ht_conf)
{
	struct il_rxon_cmd *rxon = &il->staging;

	if (!il->ht.enabled) {
		rxon->flags &=
		    ~(RXON_FLG_CHANNEL_MODE_MSK |
		      RXON_FLG_CTRL_CHANNEL_LOC_HI_MSK | RXON_FLG_HT40_PROT_MSK
		      | RXON_FLG_HT_PROT_MSK);
		return;
	}

	rxon->flags |=
	    cpu_to_le32(il->ht.protection << RXON_FLG_HT_OPERATING_MODE_POS);

	/* Set up channel bandwidth:
	 * 20 MHz only, 20/40 mixed or pure 40 if ht40 ok */
	/* clear the HT channel mode before set the mode */
	rxon->flags &=
	    ~(RXON_FLG_CHANNEL_MODE_MSK | RXON_FLG_CTRL_CHANNEL_LOC_HI_MSK);
	if (il_is_ht40_tx_allowed(il, NULL)) {
		/* pure ht40 */
		if (il->ht.protection == IEEE80211_HT_OP_MODE_PROTECTION_20MHZ) {
			rxon->flags |= RXON_FLG_CHANNEL_MODE_PURE_40;
			/* Note: control channel is opposite of extension channel */
			switch (il->ht.extension_chan_offset) {
			case IEEE80211_HT_PARAM_CHA_SEC_ABOVE:
				rxon->flags &=
				    ~RXON_FLG_CTRL_CHANNEL_LOC_HI_MSK;
				break;
			case IEEE80211_HT_PARAM_CHA_SEC_BELOW:
				rxon->flags |= RXON_FLG_CTRL_CHANNEL_LOC_HI_MSK;
				break;
			}
		} else {
			/* Note: control channel is opposite of extension channel */
			switch (il->ht.extension_chan_offset) {
			case IEEE80211_HT_PARAM_CHA_SEC_ABOVE:
				rxon->flags &=
				    ~(RXON_FLG_CTRL_CHANNEL_LOC_HI_MSK);
				rxon->flags |= RXON_FLG_CHANNEL_MODE_MIXED;
				break;
			case IEEE80211_HT_PARAM_CHA_SEC_BELOW:
				rxon->flags |= RXON_FLG_CTRL_CHANNEL_LOC_HI_MSK;
				rxon->flags |= RXON_FLG_CHANNEL_MODE_MIXED;
				break;
			case IEEE80211_HT_PARAM_CHA_SEC_NONE:
			default:
				/* channel location only valid if in Mixed mode */
				IL_ERR("invalid extension channel offset\n");
				break;
			}
		}
	} else {
		rxon->flags |= RXON_FLG_CHANNEL_MODE_LEGACY;
	}

	if (il->ops->set_rxon_chain)
		il->ops->set_rxon_chain(il);

	D_ASSOC("rxon flags 0x%X operation mode :0x%X "
		"extension channel offset 0x%x\n", le32_to_cpu(rxon->flags),
		il->ht.protection, il->ht.extension_chan_offset);
}

void
il_set_rxon_ht(struct il_priv *il, struct il_ht_config *ht_conf)
{
	_il_set_rxon_ht(il, ht_conf);
}
EXPORT_SYMBOL(il_set_rxon_ht);

/* Return valid, unused, channel for a passive scan to reset the RF */
u8
il_get_single_channel_number(struct il_priv *il, enum nl80211_band band)
{
	const struct il_channel_info *ch_info;
	int i;
	u8 channel = 0;
	u8 min, max;

	if (band == NL80211_BAND_5GHZ) {
		min = 14;
		max = il->channel_count;
	} else {
		min = 0;
		max = 14;
	}

	for (i = min; i < max; i++) {
		channel = il->channel_info[i].channel;
		if (channel == le16_to_cpu(il->staging.channel))
			continue;

		ch_info = il_get_channel_info(il, band, channel);
		if (il_is_channel_valid(ch_info))
			break;
	}

	return channel;
}
EXPORT_SYMBOL(il_get_single_channel_number);

/**
 * il_set_rxon_channel - Set the band and channel values in staging RXON
 * @ch: requested channel as a pointer to struct ieee80211_channel

 * NOTE:  Does not commit to the hardware; it sets appropriate bit fields
 * in the staging RXON flag structure based on the ch->band
 */
int
il_set_rxon_channel(struct il_priv *il, struct ieee80211_channel *ch)
{
	enum nl80211_band band = ch->band;
	u16 channel = ch->hw_value;

	if (le16_to_cpu(il->staging.channel) == channel && il->band == band)
		return 0;

	il->staging.channel = cpu_to_le16(channel);
	if (band == NL80211_BAND_5GHZ)
		il->staging.flags &= ~RXON_FLG_BAND_24G_MSK;
	else
		il->staging.flags |= RXON_FLG_BAND_24G_MSK;

	il->band = band;

	D_INFO("Staging channel set to %d [%d]\n", channel, band);

	return 0;
}
EXPORT_SYMBOL(il_set_rxon_channel);

void
il_set_flags_for_band(struct il_priv *il, enum nl80211_band band,
		      struct ieee80211_vif *vif)
{
	if (band == NL80211_BAND_5GHZ) {
		il->staging.flags &=
		    ~(RXON_FLG_BAND_24G_MSK | RXON_FLG_AUTO_DETECT_MSK |
		      RXON_FLG_CCK_MSK);
		il->staging.flags |= RXON_FLG_SHORT_SLOT_MSK;
	} else {
		/* Copied from il_post_associate() */
		if (vif && vif->bss_conf.use_short_slot)
			il->staging.flags |= RXON_FLG_SHORT_SLOT_MSK;
		else
			il->staging.flags &= ~RXON_FLG_SHORT_SLOT_MSK;

		il->staging.flags |= RXON_FLG_BAND_24G_MSK;
		il->staging.flags |= RXON_FLG_AUTO_DETECT_MSK;
		il->staging.flags &= ~RXON_FLG_CCK_MSK;
	}
}
EXPORT_SYMBOL(il_set_flags_for_band);

/*
 * initialize rxon structure with default values from eeprom
 */
void
il_connection_init_rx_config(struct il_priv *il)
{
	const struct il_channel_info *ch_info;

	memset(&il->staging, 0, sizeof(il->staging));

	switch (il->iw_mode) {
	case NL80211_IFTYPE_UNSPECIFIED:
		il->staging.dev_type = RXON_DEV_TYPE_ESS;
		break;
	case NL80211_IFTYPE_STATION:
		il->staging.dev_type = RXON_DEV_TYPE_ESS;
		il->staging.filter_flags = RXON_FILTER_ACCEPT_GRP_MSK;
		break;
	case NL80211_IFTYPE_ADHOC:
		il->staging.dev_type = RXON_DEV_TYPE_IBSS;
		il->staging.flags = RXON_FLG_SHORT_PREAMBLE_MSK;
		il->staging.filter_flags =
		    RXON_FILTER_BCON_AWARE_MSK | RXON_FILTER_ACCEPT_GRP_MSK;
		break;
	default:
		IL_ERR("Unsupported interface type %d\n", il->vif->type);
		return;
	}

#if 0
	/* TODO:  Figure out when short_preamble would be set and cache from
	 * that */
	if (!hw_to_local(il->hw)->short_preamble)
		il->staging.flags &= ~RXON_FLG_SHORT_PREAMBLE_MSK;
	else
		il->staging.flags |= RXON_FLG_SHORT_PREAMBLE_MSK;
#endif

	ch_info =
	    il_get_channel_info(il, il->band, le16_to_cpu(il->active.channel));

	if (!ch_info)
		ch_info = &il->channel_info[0];

	il->staging.channel = cpu_to_le16(ch_info->channel);
	il->band = ch_info->band;

	il_set_flags_for_band(il, il->band, il->vif);

	il->staging.ofdm_basic_rates =
	    (IL_OFDM_RATES_MASK >> IL_FIRST_OFDM_RATE) & 0xFF;
	il->staging.cck_basic_rates =
	    (IL_CCK_RATES_MASK >> IL_FIRST_CCK_RATE) & 0xF;

	/* clear both MIX and PURE40 mode flag */
	il->staging.flags &=
	    ~(RXON_FLG_CHANNEL_MODE_MIXED | RXON_FLG_CHANNEL_MODE_PURE_40);
	if (il->vif)
		memcpy(il->staging.node_addr, il->vif->addr, ETH_ALEN);

	il->staging.ofdm_ht_single_stream_basic_rates = 0xff;
	il->staging.ofdm_ht_dual_stream_basic_rates = 0xff;
}
EXPORT_SYMBOL(il_connection_init_rx_config);

void
il_set_rate(struct il_priv *il)
{
	const struct ieee80211_supported_band *hw = NULL;
	struct ieee80211_rate *rate;
	int i;

	hw = il_get_hw_mode(il, il->band);
	if (!hw) {
		IL_ERR("Failed to set rate: unable to get hw mode\n");
		return;
	}

	il->active_rate = 0;

	for (i = 0; i < hw->n_bitrates; i++) {
		rate = &(hw->bitrates[i]);
		if (rate->hw_value < RATE_COUNT_LEGACY)
			il->active_rate |= (1 << rate->hw_value);
	}

	D_RATE("Set active_rate = %0x\n", il->active_rate);

	il->staging.cck_basic_rates =
	    (IL_CCK_BASIC_RATES_MASK >> IL_FIRST_CCK_RATE) & 0xF;

	il->staging.ofdm_basic_rates =
	    (IL_OFDM_BASIC_RATES_MASK >> IL_FIRST_OFDM_RATE) & 0xFF;
}
EXPORT_SYMBOL(il_set_rate);

void
il_chswitch_done(struct il_priv *il, bool is_success)
{
	if (test_bit(S_EXIT_PENDING, &il->status))
		return;

	if (test_and_clear_bit(S_CHANNEL_SWITCH_PENDING, &il->status))
		ieee80211_chswitch_done(il->vif, is_success);
}
EXPORT_SYMBOL(il_chswitch_done);

void
il_hdl_csa(struct il_priv *il, struct il_rx_buf *rxb)
{
	struct il_rx_pkt *pkt = rxb_addr(rxb);
	struct il_csa_notification *csa = &(pkt->u.csa_notif);
	struct il_rxon_cmd *rxon = (void *)&il->active;

	if (!test_bit(S_CHANNEL_SWITCH_PENDING, &il->status))
		return;

	if (!le32_to_cpu(csa->status) && csa->channel == il->switch_channel) {
		rxon->channel = csa->channel;
		il->staging.channel = csa->channel;
		D_11H("CSA notif: channel %d\n", le16_to_cpu(csa->channel));
		il_chswitch_done(il, true);
	} else {
		IL_ERR("CSA notif (fail) : channel %d\n",
		       le16_to_cpu(csa->channel));
		il_chswitch_done(il, false);
	}
}
EXPORT_SYMBOL(il_hdl_csa);

#ifdef CONFIG_IWLEGACY_DEBUG
void
il_print_rx_config_cmd(struct il_priv *il)
{
	struct il_rxon_cmd *rxon = &il->staging;

	D_RADIO("RX CONFIG:\n");
	il_print_hex_dump(il, IL_DL_RADIO, (u8 *) rxon, sizeof(*rxon));
	D_RADIO("u16 channel: 0x%x\n", le16_to_cpu(rxon->channel));
	D_RADIO("u32 flags: 0x%08X\n", le32_to_cpu(rxon->flags));
	D_RADIO("u32 filter_flags: 0x%08x\n", le32_to_cpu(rxon->filter_flags));
	D_RADIO("u8 dev_type: 0x%x\n", rxon->dev_type);
	D_RADIO("u8 ofdm_basic_rates: 0x%02x\n", rxon->ofdm_basic_rates);
	D_RADIO("u8 cck_basic_rates: 0x%02x\n", rxon->cck_basic_rates);
	D_RADIO("u8[6] node_addr: %pM\n", rxon->node_addr);
	D_RADIO("u8[6] bssid_addr: %pM\n", rxon->bssid_addr);
	D_RADIO("u16 assoc_id: 0x%x\n", le16_to_cpu(rxon->assoc_id));
}
EXPORT_SYMBOL(il_print_rx_config_cmd);
#endif
/**
 * il_irq_handle_error - called for HW or SW error interrupt from card
 */
void
il_irq_handle_error(struct il_priv *il)
{
	/* Set the FW error flag -- cleared on il_down */
	set_bit(S_FW_ERROR, &il->status);

	/* Cancel currently queued command. */
	clear_bit(S_HCMD_ACTIVE, &il->status);

	IL_ERR("Loaded firmware version: %s\n", il->hw->wiphy->fw_version);

	il->ops->dump_nic_error_log(il);
	if (il->ops->dump_fh)
		il->ops->dump_fh(il, NULL, false);
#ifdef CONFIG_IWLEGACY_DEBUG
	if (il_get_debug_level(il) & IL_DL_FW_ERRORS)
		il_print_rx_config_cmd(il);
#endif

	wake_up(&il->wait_command_queue);

	/* Keep the restart process from trying to send host
	 * commands by clearing the INIT status bit */
	clear_bit(S_READY, &il->status);

	if (!test_bit(S_EXIT_PENDING, &il->status)) {
		IL_DBG(IL_DL_FW_ERRORS,
		       "Restarting adapter due to uCode error.\n");

		if (il->cfg->mod_params->restart_fw)
			queue_work(il->workqueue, &il->restart);
	}
}
EXPORT_SYMBOL(il_irq_handle_error);

static int
_il_apm_stop_master(struct il_priv *il)
{
	int ret = 0;

	/* stop device's busmaster DMA activity */
	_il_set_bit(il, CSR_RESET, CSR_RESET_REG_FLAG_STOP_MASTER);

	ret =
	    _il_poll_bit(il, CSR_RESET, CSR_RESET_REG_FLAG_MASTER_DISABLED,
			 CSR_RESET_REG_FLAG_MASTER_DISABLED, 100);
	if (ret < 0)
		IL_WARN("Master Disable Timed Out, 100 usec\n");

	D_INFO("stop master\n");

	return ret;
}

void
_il_apm_stop(struct il_priv *il)
{
	lockdep_assert_held(&il->reg_lock);

	D_INFO("Stop card, put in low power state\n");

	/* Stop device's DMA activity */
	_il_apm_stop_master(il);

	/* Reset the entire device */
	_il_set_bit(il, CSR_RESET, CSR_RESET_REG_FLAG_SW_RESET);

	udelay(10);

	/*
	 * Clear "initialization complete" bit to move adapter from
	 * D0A* (powered-up Active) --> D0U* (Uninitialized) state.
	 */
	_il_clear_bit(il, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_INIT_DONE);
}
EXPORT_SYMBOL(_il_apm_stop);

void
il_apm_stop(struct il_priv *il)
{
	unsigned long flags;

	spin_lock_irqsave(&il->reg_lock, flags);
	_il_apm_stop(il);
	spin_unlock_irqrestore(&il->reg_lock, flags);
}
EXPORT_SYMBOL(il_apm_stop);

/*
 * Start up NIC's basic functionality after it has been reset
 * (e.g. after platform boot, or shutdown via il_apm_stop())
 * NOTE:  This does not load uCode nor start the embedded processor
 */
int
il_apm_init(struct il_priv *il)
{
	int ret = 0;
	u16 lctl;

	D_INFO("Init card's basic functions\n");

	/*
	 * Use "set_bit" below rather than "write", to preserve any hardware
	 * bits already set by default after reset.
	 */

	/* Disable L0S exit timer (platform NMI Work/Around) */
	il_set_bit(il, CSR_GIO_CHICKEN_BITS,
		   CSR_GIO_CHICKEN_BITS_REG_BIT_DIS_L0S_EXIT_TIMER);

	/*
	 * Disable L0s without affecting L1;
	 *  don't wait for ICH L0s (ICH bug W/A)
	 */
	il_set_bit(il, CSR_GIO_CHICKEN_BITS,
		   CSR_GIO_CHICKEN_BITS_REG_BIT_L1A_NO_L0S_RX);

	/* Set FH wait threshold to maximum (HW error during stress W/A) */
	il_set_bit(il, CSR_DBG_HPET_MEM_REG, CSR_DBG_HPET_MEM_REG_VAL);

	/*
	 * Enable HAP INTA (interrupt from management bus) to
	 * wake device's PCI Express link L1a -> L0s
	 * NOTE:  This is no-op for 3945 (non-existent bit)
	 */
	il_set_bit(il, CSR_HW_IF_CONFIG_REG,
		   CSR_HW_IF_CONFIG_REG_BIT_HAP_WAKE_L1A);

	/*
	 * HW bug W/A for instability in PCIe bus L0->L0S->L1 transition.
	 * Check if BIOS (or OS) enabled L1-ASPM on this device.
	 * If so (likely), disable L0S, so device moves directly L0->L1;
	 *    costs negligible amount of power savings.
	 * If not (unlikely), enable L0S, so there is at least some
	 *    power savings, even without L1.
	 */
	if (il->cfg->set_l0s) {
		pcie_capability_read_word(il->pci_dev, PCI_EXP_LNKCTL, &lctl);
		if (lctl & PCI_EXP_LNKCTL_ASPM_L1) {
			/* L1-ASPM enabled; disable(!) L0S  */
			il_set_bit(il, CSR_GIO_REG,
				   CSR_GIO_REG_VAL_L0S_ENABLED);
			D_POWER("L1 Enabled; Disabling L0S\n");
		} else {
			/* L1-ASPM disabled; enable(!) L0S */
			il_clear_bit(il, CSR_GIO_REG,
				     CSR_GIO_REG_VAL_L0S_ENABLED);
			D_POWER("L1 Disabled; Enabling L0S\n");
		}
	}

	/* Configure analog phase-lock-loop before activating to D0A */
	if (il->cfg->pll_cfg_val)
		il_set_bit(il, CSR_ANA_PLL_CFG,
			   il->cfg->pll_cfg_val);

	/*
	 * Set "initialization complete" bit to move adapter from
	 * D0U* --> D0A* (powered-up active) state.
	 */
	il_set_bit(il, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_INIT_DONE);

	/*
	 * Wait for clock stabilization; once stabilized, access to
	 * device-internal resources is supported, e.g. il_wr_prph()
	 * and accesses to uCode SRAM.
	 */
	ret =
	    _il_poll_bit(il, CSR_, CSR_GP_  _il_poll_RESET_REG_FLAG_STOP_MASTER);

	ret =
	    _il_poll_bit(il, CSR_RESET, CSR_RESET_R
				break;
			}=KwI
	if (cm]WdKRt acZrnal resourcemourcemourcemourcemourcemourcemourcemourcemourcemcEcZ uCode SRAM.
mRAM.
mRAM.
mRAM.
mRAM.
mRAM.
mRAM.
mRAM.
mRAM.
mcA_REG,
;lcZ>cap |= IEEE8WpM\n", rxon->node_addr);
	D_RADIO("u8[6] bssid_addr: %pM\n", rxon->bssid_addr);
	DZI
	if (cm]WdKmg);
	}

	D3channel_number);

/**
 * il_set_rxon_channel - Set the band and channel values in cZer_addr_equal to NULL0atic u16 noinline
il_adjust_beacoZtx_params = I cZ =
	    _il_pAault aftams =HANNEL_SWITCH_PENDING, &il->status))
		return;

	if (!le32_to_cpu(csa->status) && ligi;3break;
			case IEEE80211_HT_PARAM_CHA_SEC_NONE:
			default:
				/* channel locatiotransition_set_flags_for_band);

/*
 * initialize rxon structure with default values from ee1_band band)
{= values from emG_BAND;3RXON_FLG_CHANNEL_MODE_LEGACY;
	}

	if (il->ops->set_rxon_chain)
		il->ops->set_rxole L0S, so);

	D_ASSOC("rxon flags 0x%X operation mode :0x%X "
		"extension channel offset 0IO_REG_VAL, CSR_RESET, CSR_RESET_R
				break;
			}=KwI
	if (ation complete" bit #2lse
			;3id
il_set_rxon_ht(struct il_priv *il, struct il_ht_config *ht_conf)
{
	_il_set_rxo_GIO_REG_VI6CM\n", rxon->bssid_addr);
	;cR))
			break;
	 &sband->ht_cap, NL80211_BAND_5GHEues from emG_BAND;3RXON_FLG_CHANNr_equal to NULL0atic u16 noinline
il_adjust_beacoZv=KgR("Faile	case IE#>	loif (MSK | RXON_FILTER_ACCEPT_GRP_MSK;
		break;
	default:
		IL_ERR("Unsupported interfanot commit to "r clock stabilQT |mmit to "r clo_costabilQT)
	struct iee		IL_ERR("Unsupported interfammit to "r clock stabilQT)
	sCKEN_BI2S,
		   >pci_dev, REGve->of211_HT_PARAM_CsL_ERR("Unsupport(iand STTterfanot cupport(iand STTt->bss1AG_M	pci,
		d\n",  il_priv *il)
from management bus) reak,
		HORT_SLOT_M, band, _to_cpu(rxon->flags));6(maxand,  *il)
{il,ce->L0S->L1 tr;* OFDL0av;

	ch_info il)
{X_BITo_cpu(il->timing.beacon_init_val),
		leP_MSK;
	l_channel_info *ch_=(maxand, ctor il,ce->
}
EXPORT_SYMBOXON_FLG_nnel_nund>status);

	r;
	CHK(!eOPanaSUPP
		   >0 dBLEDeachanmilliwatconfig);

maxand, c<
 * @il: staging_R   RXON_F_inf TX (cm]rue;
	), danmWtruct ilatus);
;
));
	CHK(!ether_addr_ig);

maxand, c> K;
	l_channeln_chan_off* @il: staging_R   RXON_F_inf TX (cm]rue;ab;

	= vnf li	/* 	structnot maxand,  *K;
	l_channeln_chan_off*;
));
	CHK(!ether_addr_ig);

_BAND_2beaco_rf	/* C
));
	CHK(!etOflags);
m nlquence 0xil_co
		/*_equal_DIS_get_channel_il;
		on->fiiON_FILTER an Rx buffE_MSK |
lags e_20MHZ   RXOCRYPT_MSK;get_channel_inf, band, chan/D;3RXON_F
	/* x* and aic_ratm n\
	}
nals r HW osid_addr)RYPTX_BITinf,initializaSCANNlete" bit to moveCOUNT_LEGol imp devic_params-o = &il->channinfo[0];

	il->stagingg);

X_BITitor il,ce-ESET, CSR_RESD_BITSR_RESx* and a_info(il, b_rates = rates;L0av;

	ch_in *rxon =_channel_info *c;PT_MSK;get_chan_info *ch_i, band, chan transFLG_nnel_nund>status); reset.
	 *(staT_Rvoid *)&maxand,  **
	 * E;
			orim eSx* and anfig);

rnnel for_MSK;get_chan_info *ch_iL0av;

	ch_info T_MSK;get_channel_infL0av;

	ch_info },  il_priv *il)
from management bOT_M, band, ch_done);

voind>btl_info(il, il->band, le16_to_cpl, il->banblse"
 blse"
 =l for.l_polHZ 72= Ba->cADity_rY),
tnot/* vane M2= Ba- &chKIara),
tnot/ne M_ackalizeansitnot/ne M_cCsLlizeansitno}SYMBOXON_cfg->mod_paramC
))blse"
>staging.cap |= I il_apm
	struct iblse"
>staging.cap |= Iaddr);heck if BIOSBT ->moodule par Check iblse"
>stagingg.cap |= I il_apmtatusLG_CHAN*il, T(band)
		leP_MSK;XON_FLG_CCK_MSK) {
	cap |G,
	atim_win));

	returblse"
al &blse"
a) il_irq_hanfn = (void *l_seTet_moot_rxonfo(il,	if ((rxon->node_addr[0]btl_info(,
		HORT_SLOTn
 *
ats_Z   RXO_to_cpu(rxon->flags))u8

	if (!il)
{_addrto_cpl, il->ban*
ats_e"
 *
ats_e"
 =l for.c_rxon_c init	il->stag_rates?n_chaTATSp |G,staEARhaTATS
{
	tno}SYMBOXONociate with any AP, b_rates K;XON_FLG_CCK_M_q[il-SK) {
	aTATSatim_win));

	retur*
ats_e"
tsf;
	sr Chec&*
ats_e"
		defau;_chan_offset) {
K;XON_FLG_CCK_MSK) {
	aTATSatim_win));

	retur*
ats_e"
tsf;
	serror = *
ats_e"
tl,	if ((rxon->node_addr[0]*
ats_Z   RXOgs: 0x%08X\n", lCSR_leeit threshold to maxi));
	D_RADIO("u32 filter_flstop(struct il_priv *il)
{
	logs: 0x%08x\n", le32_to_cpu(rxon->filter_flags));
	_leeiADIO("u8 dev_ty_leei 0x%x\n", rx_leeiADIO("er due X("_leei _bithann));rc: nd_queue);
	}_leeiil_SR_leei(pkt-,}_leeiil_SRbledup_;rctate\n");
)
{
	/* Set the FW erro_SR_leeigs: 0x%08X\n", lCSRt_held*
atscmd_idx];

	txq->time_stamp = jiffies;

	pci_unmap_single(il->pci_dev, dma_unmap_addr(me32 pri =FW_ERRORS)
		\n", pri_t	il->sg_lock)RX	meta-s_chaULL;
	sue to uCoDumo kirue;
y));
on_hnhock, flDIO("u8 dev_tlags%s:if (il-nruct i"false"
 *
 * dea->flags _coex_ac("Loaded firmware version: %s\n", i\n", rxrawe priol,	if ((rxon->node_ad", lCSRt_held*
atsgs: 0x%08X\n", lret = 0;

	/* stop devicme_stamp = jiffies;

	pci_unmap_single(il->pci_dev, dma_unmap_addr( CSR_RESETEing LRand
_singlL, fal>node%s (it */X)ues up Activseqchannel inf L, false);
or interruERRORS)
		\n", u.ret_Z sp.ut, 10ueue);
or inter i"false"
 *
 * dea->flu.ret_Z sp. AP
 *);
or intera->flu.ret_Z sp. AP
 *;
or interru>cfg->mod_a->flu.ret_Z sp.baFLG_CCseqfree(;
or interruERRORS)
		\n", u.ret_Z sp.ut, 1011_banl,	if ((rxon->node_ad", lt il_privid
il_priaddr)isrd*
atscmd_idx];

	txq->tici_un	ch_info = &isrd*
atsnnel_info[0];

	isrd*
atsanl,	i	HORT_SLmacl_infM, h = &sband->channehwl_rx &= ~RXON_FLG_SHORTpe = RXOlockde
}
EXP
or interl, struct il_rx_buf *rx, bEG_FLAannels *annelsci_unmap_single	txq->tiv,  (re	txq; (HW error during stresS->L1qheck iMACuf *r("_rate this de);

_BAND_2beaco_rf	/* CESET, CMACuf *r("add

	-and ON_Fbeacoted(il));
	CHK(!etOaddr_ig);

EG_FLaphyAC_NUMCESET, CMACuf *r("add

	-aEG_FLaphyAC_NUMpu(rxon-)
 */
stab_rates = rates;q hyAC_NUMp-dan-aEG_FLess W/A) */
	il_set_bit(il,rs are:
 *   Able ->qos_l->s.p(s_qos_annm.ac[q].cw_K;
	eBOVE:
				rxon-16(P_CNTRL,cw_K;
f (rate-qos_l->s.p(s_qos_annm.ac[q].cw_Kl->sBOVE:
				rxon-16(P_CNTRL,cw_Kaxf (rate-qos_l->s.p(s_qos_annm.ac[q].aifsi =Fct ieee80ifs (rate-qos_l->s.p(s_qos_annm.ac[q].edcax, op>sBOVE:
				rxon-16((P_CNTRL,, op>* 32)  Able ->qos_l->s.p(s_qos_annm.ac[q].0atic ud1 aram(bt_coex_active, bool, S_IRUGO)rs are:
 *   Abl CMACuf *r("add

ted(il)mode) {
	case NL80211_IFTYPE_macl_infM, ,
		HORT_SLmacl, blaDECRYPT_Mh = &sband->channehwl_rxci_unmap_single	txq->tiv,  (re	txq; (->L1 tr;*ck iMACuf *r("_rate this de trans];

	i_MSKD);
		rMixed0x%04XiMANAGread_wo CMACuf *r("add

	 trau(rxon-r to t  il_priv *il)
from manageme_GPTYPE_macl, blaDECRYPT_Mpriv *il)
{
	unt bOT_Mpkt->ity in PCIe bus L0->L0S-=
	    il_get_channel_info(ireset.
;
}
EXPORT_SYMBOL(il_get_single_channel_number);

/**
 * il_seset) {
K;Xo
		/*_equa * il_	i	HORT_SLmaclunm_g.ofdm_hth = &sband->channehwl_rx &= ~RXON_FLG_SHORTpe = RXOci_unmap_single	txq->tiv,  (re	txq; (->L1t ifo il)
{TER_Ao)
		al), */
	_init_val),
		k iMACuf *r("_rate:_single_, unma	       "sic_rates "sic_rask[0] =e);

_BAND_2beaco_rf	/* CESET,urn 1;				ryging-ding.ofdm_ht_ic_raLAG_INION_Fbeacoted(il))t iv, !ether_addation complete" bit #2l 3500 to b{= valu multiple"sirtu				.ofdm_ht (!iut	 */nline
il_ no-opDisabe to get a-dinss_coa 72g.ofdm_ht_a&&
		if (il->ostrans];

	il->tlags uct il_priv *ilitor >ostrCESET,t iv, !eOPanaSUPP
	dation complete" ;

	il->t .beac	 cpu_to_le1>t .be_rates] =et iv, t bOT_Mpkt->ec\n");

	t iCESET,urn 1;			FT_Rvoid *)&_le1>u(rxon-sic_rates = 0x=
		  ostrCESET, ;

	il->t t *pkt 		 cpu_to_le1>t 	il->staging.ofdm_basi

/**
 * id\n",  CMACuf *r("add

	t ivu(rxon-t iC;
		al), _activ_init_val),
		leset) {
t ifoase NL80211_IFTYPE_maclunm_g.ofdm_htpriv *il)
{id
il_prtddr_set_g.ofdm_hth = &sbanging.flags &= ~RXON_FLG_SHORTpe = RXOci_unu(il->timing.beacon_init_val),
		leP_MSK;
	tm n_il->tlags ul for_M_tm n_c =
	 lHZ 7d\n vers20IO_REr_M_il,ce_tm n_ener statete" ;
bOT_Mpkt->ec\n"he hardwaremaclren;

_g.ofdm_hth = &sband->channehwl_rx &= ~RXON_FLG_SHORTpe = RXOci_unmap_single	txq->tiv,  (re	txq; 
		al), */
	_init_val),
		k iMACuf *r("_rate:_single_, unma	       "sic_rates "sic_rask[0] =ezalloc(s;

	il->!lags uct i

	il->t t *pkt  cpu_to_le1>t 	il->staging.of>band = ch__ac("Ltddr_set_g.ofdm_hthversgs uct eth_zero_unmapa_noting   Abl CMACuf *r("add

ted(il)	al), _activ_init_val),
		ase NL80211_IFTYPE_maclren;

_g.ofdm_ht,
		HORT_SL11_HTl, q_	ch>ity in PCIe bus L0->L0S-XON_FLG_, qngle_cha, qCHANNEL_M < il->channel_count; ilx, bEG_FL)struct Z =
	    _ilree_ofbEG_FL (!->center_freq =
		 FLG_, qn * il_irq_hanNo b);
	CHK	chorygil, sxqted(il));
	CHK(!ehannel, ch)mode) {
	case NL80211_IFTYPE_11_HTl, q_	chprivid
il_prt40)
, q_	ch>ity in PCIe bus L0->L0Scap->ht_su, qn;PT_MSK;gq>t t *pkt
	if (il->disable_ht40)
, q_	chprivgle_strel,ce_ ostr_cmd(struct il_priv *il)
{E:  om eci_unmap_singleel,ce_ ostr *el,ce_ ostr");
	il_printon = &il->staging;

	D_RADIO("RX CONFIG:\n !ether_ad	retuce_ ostr ar_bit(el,ce_ ostr")retuce_ ostrSYMBOumbe   RXORXON_F++eq =
		 E:  om ec and autoetuce_ ostrSYlaDECetuce_ ostr_jiffi_ASS&ANNEL_Msend_ON_FIoetuce_ ostrSYlaDECetuce_ ostr_jiffi_AS+

	serror =etuce_ ostrSYMBOumbd_c init, jiffi_Aon->
	   if BIOSetuce211_banrejSEC_Afo(il, baetuce_ ostrSYMBOumbe jSECRXON_F++eq ONFIG:\n !eAGAIi

/**
 * aetuce_ ostrSYMBOumb_to_cpuRXON_F++eq etuce_ ostrSYlaDECetuce_ ostr_jiffi_AS= jiffi_Asition_set_iftop_mas  RXOCiE_MSK;
#:  om e(ex: t_helfstrucEL_MODnN_FILTERpfdmo thop_mas  RXOCiriv gdr_lwith CTRL_Culrph()
P_CNTetd a_ines i_set_iftop_mas  RXOCiE_MSK;
s from ee() state.
	 
naldil_	 * TODd != %taT_Rure)el, actfw_aster(ilL_Culr
P_CNTetd * TOD an Rx buff64bits channepfdmo t kirAG_MASTERreF_COgs.
	 * =
		 E:  om eitor bit(il, CSR_GP_CNTRL, CSR_GP_CNTESET, CSR_RESt =
	  AG_MASTERreF_COstaging.fl"ed-up ActivL_Culr
P_CNTetd a_ines ifo(il, b_rates = rates;_irq_hanOraLAHANNEAG_MASTERreF_COo preserve aster DMA activity */
	_il_set_bit(il, CSR_RESET, CSR_RESET_REG_FLAG_STOP_MASTER);	/* Stop device's DMA activity */
	ion_set_pm_stop_master(il);

	/* Reset the entire device */
	_il_set_bit(il, CSR_RESET, CSR_RESET_REG_if (il-SW_RESET);

	udelay(10);

	/*
	 *REG_FLAG_INIT_DONE);
}
EXPORT_SYMBOL(_il_a;
	if (il_is_ht40_tx_allowed(il,tuce_ ostr,
		HORT_SLmaclsid_a
_g.ofdm_hth = &sband->channehwl_rx &= ~RXON_FLG_SHORTpe = RXOsf;
	case NL80211_ifsinglnewates "il)
{newp2pci_unmap_single	txq->tiv,  (re	txq; (->L1t ifo
		al), */
	_init_val),
		k iMACuf *r("_rate:_single_, unma	   lnewatesrue;newp2por SW error insic_rates "sic_rask[,lnewates "newp2pc");
	il_newp2pcESET,t iv, !eOPanaSUPP
	dation complete" ;
		 FLG_pe =beaconND_2beaco_rf	/* CESET,on_s	/*
	uh? Bu & PCI_.../
	il_m nlmaybe toppuCodh8021Disabe'rtedflags mid_lw
on_aEAG_MASTERreOL(_i!21Dis/ET,t iv, !eBUSYaddation complete" bi _to_cpuis/ETsic_ratesv, newates;ETsic_rp2poG_CCK_MSK  cpu_to_le1>t newates;ET("Ltddr_set_g.ofdm_hthversgs uct e iv, 0; id\n",  CMACuf *r("add

	t ivu(rxon-t iC;
		al), _activ_init_val),
		leset) {
t ifoase NL80211_IFTYPE_maclsid_a
_g.ofdm_htprivid
i PE_maclflushh = &sband->channehwl_rx &= ~RXON_FLG_SHORTpe = RXOsf;
  e32 EG_FL (!il)
{Xropci_unmap_single	txq->tiv,  (re	txq; (HW error durinHZ 7d\nS= jiffi_A: cosecs	rxojiffi_A(50IO_REG_notific	al), */
	_init_val),
		k iMACuf *r("_rate this de);

_MSK;gq>tt t *p)ddation compl	return;

	if (!le
	if (pto);

/* va, q_ase>status) &&map_singleEG_FLa*qil_clear_b%pM\n", G_CCity */

EXPORT_SYMBOL(iq ar_bit(, qStagq= 0x=
		qSYMBa_GPtrMixeqSYo devGPtr/

EXPORT_SYMBOL(i	il_pend_ON_FIojiffi_A,nHZ 7d\non->
	     d *rxon = (voidflushaEG_FLau(rxon-)pu_dil, bad, channete" 	m_leeit2S,
	eted\n",  CMACuf *r("add

ted(il)	al), _activ_init_val),
		ase NL80211_IFTYPE_maclflush * wake deOraHI_My& PtchdoinHZitswff64bits(ll->st)nHZ 7 l->st.	 * CI_ CSR_HW_e desid_a
L0S\n");HZ 7d\nS_MSK) {ANNEEG_FLaiR_HW_Iemptyswff11_banAG_MASTEht_sin *il)
{
	unt bdr, acstu acEG_FLRN("mac/bssid mcast!\n");cn= true;
	}

	/* , bEG_FL *;gq>t _bit(, qScn=]er_flags));
	EG_FLa*q>t _, q->q; (HW error durinHZ 7d\n; (HW error durin &= = jiffi_Asi(->L1 tr;*ck=
		qSYMBa_GPtrMixeqSYo devGPtr/->
	 , q->pend_G, sizeo &=l, b_rates = rates;HZ 7d\nS=BOVE:
, q->pend_G, siz+NT_LEGosecs	rxojiffi_A(bit(il, CwdlHZ 7d\nis de);

pend_ON_FIo &=,nHZ 7d\non->
	    d *rxQG_FLau( stu atlags%uGos.rxon-)pu_dred-up Actijiffi_A	rxoosecso &= -
, q->pend_G, si)il, b_rav, t bel,ce_ ostr_uct il_priv *b_rates AL, C== !eAGAIitatu0_WARN(     struct _is_hwake deMakn"); PtchdoinHZits
			EEG(_i_MSKf;HZ 7d\nSas
		reweadly e dediscoveELOW:
EG_FLahucpu_etws isHZ 7d\nSaetur.25*HZ 7d\nt_sinasic_raturn DityCK(HZ 7d\no ((HZ 7d\no / 4)hwake deWPtchdoinHZble e_erba arewff64bitseacheSx*EG_FLaf_BIT_u arel->sfahucp
isabe 11_band baAG_MASTEh	 * HI_Mytpure iE_M_rat~RXONMBa thop_mHZble.uct il_priv *bg_ Ptchdoi(HW error durinl->sci_unmap_single	txq->tiv, RN("mac/bssid mca)l->ssi(->L1cnn; (HW error durinHZ 7d\n; ;
	il_printon = &il->staging;

	D_RADIO("RX CONFIG:\n;es;HZ 7d\nS= bit(il, CwdlHZ 7d\n;de);

pendd\nS== 0 CONFIG:\n;es;XON\onimodiD_24G_able to stu atnodeEG_FLa*_GRP_MSK;bdr, acstu acEG_FLRu.csa_noG_CCity */ CONFIG:\n;es;XON\onimodiD_24G_able to oable stu atEG_FL a*_GReturncnransit1cnn!le
	if (pto);

/* va, q_ase>sc_F++lt:
				/skipMSK)40 ms negliG_ableinss_cL_CHANNEEG_FLa*_GRl set nt%pM\n", G_CCity */

EXPORT_SYMBO	RP_MSK;bdr, acstu acEG_FLRu.cscn= )q ONFIG:\nN(     SR_GHZble device'tchdoired-upjiffi_A: cosecs	rxojiffi_A(urn DityCK(HZ 7d\noanl,	if ((rxon->node_adbg_ Ptchdoich_done);

voitup_ Ptchdoi( threshold to maximum (HW error ->L1HZ 7d\nS= bit(il, CwdlHZ 7d\n;dde);

pendd\n, booR_GHZble device'tchdoired--upjiffi_A: cosecs	rxojiffi_A(urn DityCK(HZ 7d\noanl,chan_offd	 lHZ 7e device'tchdoitl,	if ((rxon->node_addrtup_ Ptchdoi * wake deE:  Dde;
			error = e tom(stagir = eial_DIcadly buff64d_a
dstamp)a 32-		rexon->cialE:  Dde;:s from ee tom(stagirAKE_:  Dde;
pr(ilis_MODE_PURE_4XON_FstagirAKEs from eepr(ilis_MODEr = eial_DIcadltpurit(vif ? vif->bss_couct ie32;

vusecs	rxo->timin_to_cpu(rxon->flags))u32 usec))u32 LG_SHORT_SLOT_MSum (H32 EGotr(me32  1;

	u_period ? :LG_BAND_24G_Md ? :LK) {
		if (rx" ;
		 F_Md ? :Lbeacusec->
}
EXPORT_SYMBEGot
		il_chs_DIca/
or intT_SLOT_MS

#d_adbAND_24pend_lize_higR("Unf;
	srcZ =
	   (pto);

/f;
	srcZ =bAND_24pend_tsfAM_Cs)}
}

	  ed--up (pto);

/bAND_24pend_tsfAM_Cs);xon->fl	il_chs_DIca%tT_SLOT_MS

#_adbAND_24pend_lize_low("Unf;
	src	
	   (pto);

/f;
	src	bAND_24pend_tsfAM_Cs);x  struct (EGot
<le
	if (pto);

/bAND_24pend_tsfAM_Cs)}+  1;

	if ((rxon->node_adusecs	rxo->timin * wakstagilis_usugs |=wx_co40  (!tResetucle1>_FLG_eachereceivagingampe, acss_coa 72SK)HWnHZble eON_Fle eON_Fure _setuct i_on->fT_SL1dddbAND_24pend_to_cpu(rxon->flags))u32 tagi))u32 1ddonred-up u32 LG_SHORT_SLOT_MSum (H32 tagi_low_infogil
#_adbAND_24pend_lize_low("Unf;
	src inter i   (pto);

/f;
	srcZ =Z =bAND_24pend_tsfAM_Cs)

	u_pe1ddon_low_in1ddon

#_adbAND_24pend_lize_low("Unf;
	src	
	   (pto);

/f;
	src	bAND_24pend_tsfAM_Cs);x	u_period ? :LG_BAND_24G_Md ? :LK) {
		if (rxme32  1AS= (fogil
#_adbAND_24pend_lize_higR("Unf;
	srcZ =
	   (pto);

/f;
	srcZ =bAND_24pend_tsfAM_Cs))z+NT_LEG(1ddon

#_adbAND_24pend_lize_higR("Unf;
	sr inter i   (pto);

/f;
	srZ =Z =bAND_24pend_tsfAM_Cs)is de);

tagi_low_>e1ddon_low->
}
EA: = tagi_low_-e1ddon_lowl,chan_ );

tagi_low_<e1ddon_low-);
	D_EA: = G_Md ? :L+ tagi_low_-e1ddon_lowl,cD_EA: = l = cs
	if (pto);

/bAND_24pend_tsfAM_Cs);dif
/**
,cD_EA: = l = cs
	if (pto);

/bAND_24pend_tsfAM_Cs);d  struct i			rxon->flrerrupt from management budddbAND_24pend)ag -- cleared on PM_SLEEPiv *il)
{
	unt bEG_FsuspeEV_TYPE_ESS exit tS exitci_unmap_sinEG_FLAG *pLAG =voi_EG_FLAG(S exitc;unmap_single	txq->tiv, EG_Fssertrvl->s(pLAGnsition_set_o NUL1-ASPM ove->d_erroric_ratyst->fgo_FLG_tireuspeEVase IE_set_Note: coadly bd_er PE_macl_HW_IF_24G_MSK,
Note: coaeuspeEVa1-ASPM oFLG_HTirst!iut	siize PE_macl_HW_IF_  Thno k &=rrogw
on_who;
			}=errive-on->fiiONdly blse {_er us) ops._HW_IF_t il_ostop_m3id
band and * D0U* _erD0U* s) ops._HW_ porte Enab");
		reweil_ostop_m3idEE80211_HT_CSR_DBG_HPET_M  struct _is_hw *il)
{
	unt bEG_Frerund_to_cpu(S exit tS exitci_unmap_sinEG_FLAG *pLAG =voi_EG_FLAG(S exitc;unmap_single	txq->tiv, EG_Fssertrvl->s(pLAGnsi il)
{ (prfne M2= CCK_MSK |bit #2l 350ci_dev,d banETRYity_rOUTiv gid uCo(0x41F_t ikeept #2l L0STx1 trrieE_MSK;
s frofe\n");_FLG_C3 CPUase IEEE80211_EG_Fo devGn low p
y))(pLAG_STOP_CF			 TRYity_rOUT, 0x00  Able _L80211__VAL_L0S_sireset.
;
}
!T_L1Ard	il->ops->set_rxoS

# mode :0x%X "
		"extenHW_RFhKIaraSW )q O (prfne M2=   (c1).
;
}
 (prfne M)q O, CSR_RESERFKIaray(10);

	/*
	 *R/**
,cDSW_RESET);

	FKIaray(10);

	/*
	 *
	t < 0prfne MbOT_M (pse IE(
	if (ret < 0,{ (prfne MT_M  struct _is_hwSIMPLE and PM_OPS(il->) ops, t bEG_FsuspeEV, t bEG_FrerundT_M			queue_work(il->) ops)ag -\n"); _seted on PM_SLEEP.
	 * *il)
{id
il_prE_MSK _qos>ity in PCIe bus L0->L0S-XONd *rxon = &il->staging;

	D_RADIO("RX CONFIG:\n");
	 ->qos_l->s.p(s_qos_annm.qos_il->stag0s de);

_MSKqos_l->s.qos__paramC
))	 ->qos_l->s.p(s_qos_annm.qos_il->st|HANNEL_MQOShannel oid
UPDnel EDCAchannel_ion only vaL_CHANNE
))	 ->qos_l->s.p(s_qos_annm.qos_il->st|HMQOShannel oid
TGNchannel_D_QOS("e devQoStnode_FLG_Qosak;
			=%d exteS= valse);
or inte_MSKqos_l->s.qos__param, 	 ->qos_l->s.p(s_qos_annm.qos_il->s  Able _ON_FLG_CCK_M_q[il-SK) {
	QOShannelatim_win));

	returqosto);
_e"
tsf;
	rror = 	 ->qos_l->s.p(s_qos_annm		defau;_}
		il->stagimacl_infig_-eNote: coa_infig_e_erba a bug W/A formacl_infigh = &sband->channehwl_rx &e32 64d_a
dci_unmap_single	txq->tiv,  (re	txq; ( channel && il->band == band)
		return 0= ~RXON_FLG_SHORT_inf)
	inf)= & (re	infn 0= ~RXON_FLG_SHORT_SLOT_MSK;
addr);
	Dinfump_fhp(sid_ad;unmap_single staging RXON flag 0;

	for _il_pe staging ; (HW error durinil->stag0s S->L1 transition.
	chs S->L1tm n_ive->ofdm0si il)
{ t_64d_a
ds= CCK_MSK |	al), */
	_init_val),
		k iMACuf *r("_rate:_for HW or f64d_a
ds valse);_for HW hannel;
		red-up 64d_a
dcnel_ion e-intern(,initializaSCANNlete" bit to moveon->
	 tm n_ive->ofdmRN( k iMACuf *r("
m nlive->oted(il)te" ;
		64d_a
ds&	il_chsw i++) {
	 |G,stHANGE_SMPS | w i++) {
	 |G,stHANGE_);
	D_Rult:
				/Note: coauMODE *il)
{lags on-HTHEuechee->wx_co40 wanta*_GRl 	for _il_pe staging .smps;
	Dinfumsmpso_le1BOL(ion_s	/*
Ree_ecull-> 
/**
4XON_Fs._s	/*_s	/*
IfN\onimodi_le1>is active) , actNote: coadly _s	/*
s->flagd baSM PS _le1>t iOFF );
 nl				rxon->fis_s	/*
c_rxon_ced.21Dis/ET,;
}
EXPORT_SYMBOL(il_get_singlee_channel_number);

/**
 * il_ete" bi 0S\n");
m n\
	}
Note: coadly bKEN_Bng, 0, sizeoFure N_FulZv=KgRm nlfinishe_FLG_64d_a
ds= 0CEPT_GRP_MS!64d_a
ds||		64d_a
ds& w i++) {
	 |G,stHANGE_);
	D_Rult:
ET,;
}
tm n_ive->ongleetion s		IL__d\n;dde	che=_for HW hannel;
		;
(il_set_rxon_channel);

void
il_set_for HW haflags_fo = 0x=
		 il_priv *il, enum nl80211_ban->
	   iMACuf *r("add

	-a;

		ch_v *il, fo(il, ba trans!ether_addaetion s		IL__d\n;dnete" 	;
}
EXPO_to_le1>tt 	il->staging.ofCCK_RSS&ANNEL_M il_priv *il, ei_MSl80211_ban->
	   iMACuf *r("add

	-alse %04X_v *il, fo(il, ba trans!ether_addaetion s		IL__d\n;dnete" 	W/A) */
	il_set_bit(il,rs are:
 *   Abll_set_rxon_chaswit_v *il, s location only vaactive) !
	Dinf_PARAM(band
CESET, ;

	 vaactive) 
	Dinf_PARAM(band
addae t_64d_a
ds=   (c1)el);
	ion only vaL_CHANNEESET, ;
		6inf_PARAM40_K;
us(band
CESET,  ;

	 vaa:  Does not commit t el_number(l = 0;
	u8 min, max;

	if (band ;ET,  ;

	 vaPAR40mhzs=   (c1)el
mRAM.
m;
		6inf_PARAM40_plus(band
CESET,  ;

	 vaa:  Does not commit t el_number(l = 0;
	u8 min, max;

	if (/
u8
;ET,  ;

	 vaPAR40mhzs=   (c1)el
mRAM.
mSET,  ;

	 vaa:  Does not commit t el_number(l = 0;
	u8 min, max;

	if (anne;ET,  ;

	 vaPAR40mhzs= CCK_MSK _ht_config *
,  ;

	 vaPAR40mhzs= CCK_MSKL(ion_s	/*
D	case It0 to 211_channe. Pid if in Mmle1>_Fy _s	/*
ll->r_SYMBOL(MSK;
04X_vging Rinngle stagin21Dis/ET,;lly valid if in Mied mode */
				IL_ERR("invalid exanne;Ebll_se20/40 mixed or pure MSK;
h It0 2.4g_rates:
 * _s	/*
MSK;
 noinL1 tlaaging._rxsiize 2.4g CSR_HW_es	/*
s= valu ht location _FLG_AUTO_DETECT_MSK;
		il->stagin!
	Dh)PT_GRP_MSK;
		break;
	dag0s de ;
bOT_MSK;
		else
		set_for HW O_REr_M_e based on the ch->band
 *REr_M_e ba			il->active_rate for HW haflags_FLG_pe 
 *REr_coex_active, bool, S_IRUGO)rs are:
 *   Abl,;
}
EXPORT_SYE_MSK _bcaDEC *il)min , ba transEXPORT_SYE_MSK _bcaDEC *il)minireset.s		IL__d\n:bll_seT balid 
on_il->staginel));
ANNEd\n",lizeam nlb350cffe\l_p_s	/*
Modieache_COUNxsiize N_FLG_BANmay to ge64d_a
d,_ no-opDcEL_MODEd\n",lizeat0 wx_coNote: coalid s locati
void
il_hd statete" ;
		64d_a
ds&hsw i++) {
	 |G,stHANGE_PS | w i++) {
	 |G,stHANGE_IDLE)el for_MSKhanneln->s.psocZ>cap |ans!(Dinfumociate ww i++) {
	 |G,sPS = 0x=
		 ilSKhanneln->s.psocZ>cap |)
	     zalloc(CE("channel_n and acces mighe {__DISAG_MASTERcrashesfo(il, b_raxon_cht_chan__MSK _pkt->u.csil_priv *b);

rnne
	   iMACuf *r("Eing LzeoFure _leei (&il-ted(il)te" ;
		64d_a
ds& w i++) {
	 |G,stHANGE_ (cm]CESET, CMACuf *r("TX Pand aold=ue;new=e_stream_ba;get_chan_info *csf;
	rroDinfumt_chan(&il-
 *REr_M_e bastatus); re,oDinfumt_chan(&il-t il_priv *il
x=
		 il_pri negl	/* CESET, CMACuf *r("add

	-aON_Fbeacoted(il))tion complete" ;
		tm n_ive->ongletion compl	r;
		ol imp devic_params-o = &il->channinfo[0];

	il->stagiault:
	o
		/*_equa * il_R/**
,cD CSR_RESNN_Fbe-ON_Fure _a 72shortc_rxon_c init.	IL_ERR(
}
 t_64d_a
dault:
	E_MSK _qos>reset.d\n",  CMACuf *r("add

	 trau(rxon-r to t 	al), _activ_init_val),
		leset) {
v *il)
from management bmacl_infigch_done);

vmaclree bassfh = &sband->channehwl_rx &= ~RXON_FLG_SHORTpe = RXOci_unmap_single	txq->tiv,  (re	txq; (HW error during stress 	al), */
	_init_val),
		k iMACuf *r("_rate:_single_, unma	       "sic_rates "sic_rask[0] =eW/A) */
	il_set_bit(il,rs are:
 *   Abl	ch_info = &r _il_pe staging nnel_info[0]map_single staging )reserve new gs =
		 in M (!trch_on_i_MSif ? vifskbPT_GRP_MSK;
	bAND_24skb)offd	v_cap->4skbSK;
	bAND_24skb)SK  cpubAND_24skb>t t *pkt  cpupendG, sizeoam(bt_coex_active, bool, S_IRUGO)rs are:
 *   Abl_M_tm n_c =
	 lHZ 7d\n versop())
 * NO_BAND_2beaco_rf	/* CESET, CMACuf *r("add

	-aON_Fbeacoted(il))	al), _activ_init_val),
		ONFIG:\nN(     /*/40 mixer) --> D0U* s =
		 in M);

	/* ASK >> IL_FIRST_O| RXON_FLG_C0211_suppoy(il->sxon_ULL;
	s:
	o
		/*_equa * il_
ti
void
il_hd statbl CMACuf *r("add

ted(il)	al), _activ_init_val),
		ase NL80211_IFTYPE_maclree bassfpriv *il)
{id
il_pr staginh = &sbanging.flags &= ~RXON_FLG_SHORTpe = RXOci_unmap_single staging RXON flag 0;

	for _il_pe staging ; (uct il_rx_buf *rxbta *s>ssi(uct il_rx_buf *rx_MSK | R *_MSK | R 0;
AWARE_MSK | Ratbl Csxon_("_rate: this de);

_BAly vaL_CHANNE
))FIG:\n");
	 -> valid if in Mi	il_ch_MSK | R-> v_band and o_le1>&ed mode */
				IL_ERR("invalid e;;
	 -> vand ogfxbta_L0atint
		il_ch!!(_MSK | R->
or inter v_band and o_le1>&ed mode */
				IL_ERR(ann_GFfdm_("iSNIO_CHI stagin IL= NL80211oexsufficiint
	 CCK_MSKL(hannel =sic_rates (ch_info->channel);
	il-dm_basic_rarcuil_polctiv_
		ONbta =_rx_buf *rxfi[0]*
a(RXOlo_MSK | R->ting   AT,;
}
ttaCESET, uct il_rx_buf *rxbtae staapRXON fai 0x%bta-> v_faiSK _h->L1maxieee80ress ))	axieee80r el_nul_chs v_fait_vc
/f;
	 VE:
, Aannels &ed mode */
			MCS_TX- &chSTl_gese);
	)f;
	 VE:
}
E mode */
			MCS_TX- &chSTl_geseSH);
;s ))	axieee80r +dmRN(ET, ;
		 v_fait_vc
/rx_lize[1]S== 0SS&ANN	nter v_fait_vc
/rx_lize[2]S== 0 CON	I stagin IL= NL80211oexsufficiint
	   (c1)el
;
		oaxieee80r <dmR CON	I stagin IL= NL80211oexsufficiint
	   (c1)elmRAM.
mRAM.
mcANN	n*
IfNaON_Fcmd)	il_m nl6CM\ntoppuCothr
	CHKaEd\ceANN	n*
ic_raMODEAPedisco   ils_usHEues fbe'rtestFy _s		/*
s->Fure Npnss_cL_   il_ge,edflagae {_s,
Note: coANN	n*
ily bsorrorey busHabd\nSagae/f;
	 xon->n stagin IL= NL80211oexsufficiint
	   (c1)elm_rarcuil_pol_activ_  AT,_CCK_RATES_MASK >> IL_FIRST_CCK_RATE) stagin IL= NL80211oexsufficiint
	   (c1)el il->vif->addr, ETH il->vif}tbl Csxon_("add

ted(il_hw *il)
{
	l_ratone);

void
noags =
h = &sbanging.flags &= ~RXON_FLG_SHORTpe = RXOci_unon_set_inmo thop_mucle1>agae pported, no duriediD_N_FLG_s =
		 in M_BANagae nurn;nnepackels_shPORT_SYZv=KgR(ntE80211_HT IL_FIRST_O| RXON_FLG_C0211_suppoy(il->sxon_ULL;
	s:
 IL_FIRST_->restartag0s S-;Xo
		/*_equa * il_	i	 *il)
{id
il_prbAND_24E_MSK h = &sband->channehwl_rx &= ~RXON_FLG_SHORTpe = RXOci_unmap_single	txq->tiv,  (re	txq; (HW error during stres	_on-64 pendG, sisi(uct il_skfies = skb>t rx_buf *rx_AND_24 (!(rx &>hw_value;

_skb)offFIG:\n");
 iMACuf *r("_rate this deu(il->timing.beacon_init_val),
		leP_MS! cpubAND_24L_CHANNEESET,   d *rxE_MSK if ? vif_FLG_noif ? viure L_CHANNted(il))d	v_cap->4skbSskb)SK NFIG:\nN(     W/A) */
	il_set_bit(il,rs are:
 *   Able_MSK;
	bAND_24skb)offd	v_cap->4skbSK;
	bAND_24skb)SKK  cpubAND_24skb>t skb;es;HZ 7G, sizeo(h = &sband->channemgmt *)skb->l->sc, u.bAND_2.pendG, sisi( cpupendG, sizeon-64AUTO_DETpendG, sitatbl CMACuf *r("add

ted(il)_coex_active, bool, S_IRUGO)rs are:
 *   Abl_;

_BAND_2beaco_rf	/* CESET, CMACuf *r("add

	-and ON_Fbeacoted(il));
	CHK;lete" ;

	RT_SYpoint_s =
		 ->ec\n"he hardwaremacl_MSKinmo_64d_a
dh = &sband->channehwl_rx &= ~RXON_FLG_SHORTpe = RXOsf;
	uct il_rx_buf *rx_MSK | R *_MSK | R &e32 64d_a
sci_unmap_single	txq->tiv,  (re	txq; (->L1 tr;*ck	al), */
	_init_val),
		k iMACuf *r("_rate:_for a
s;

		if (i64d_a
sc Abl_;

_BAND_2alram	/* CESET, CMACuf *r("add

	-aON_Falramted(il))	al), _activ_init_val),
		ONFIG:\nN(     ;
		64d_a
s &e04XitHANGED_QOSCESET,HW error during stress  W/A) */
	il_set_bit(il,rs are:
 *   A))	 ->qos_l->s.qos__paramLG_BMSK | R->qos;ult:
	E_MSK _qos>resetEr_coex_active, bool, S_IRUGO)rs are:
 *   A     ;
		64d_a
s &e04XitHANGED_BEACuppaddr);
	t:
				/FIXME:_m nlbe 11n;

	bAND_24L_CHANN ? location AWARE_MSK | RaL_CHANCRYPT_MpT_GRP_MSbAND_24L_CHANN 	   (c1)elig *
,  ;

	bAND_24L_CHANN 	 CCK_MSK     ;
		64d_a
s &e04XitHANGED_BSSIDCESET, CMACuf *r("BSSID	       "_MSK | R->ting   AL(ion_s	/*
O M)_s i ge64d_, si40 waiONdlLG_brs aedtEG_FL aon s	
m;
pDcEL_MODrted, traffic	 */
	ae {4d_, s.	 * deingampadly bufpDcEL_receivagi(wx_cod, I_My&e-internxsiize Rm nld != %sEAPeonpDcEL_MOae {4d_, s(!iut	MOD S_l)
gs |=poistion),oNote: coa_s =
		 -pDcEL_);

	d		rewly br = ed\nSaetuNote: coadly bd_er uCI_EXP t *ppDcEL_ting .l 35to get aunbrs atEG_FL avifsuel  | dom
	 * DDis/ET,;
}
Es_zero_eMODr_unmap_MSK | R->ting  pT_GRP_RbledbEG_FL _b_il_psua * ,n_chaT	ILl_gSuppPsxoIVE  AL(ion_s	/*
If_MODrted,   _il_polla)HWnRm nlgo
	}
nnedflags ba agleast,pDcEL_MODnlbe  an Rx bc =
	  i)
	 MODrwi_MAr_prpendG/40 mixeHW_es	/*
ve;

	ifauMODnl)
gK i(FIXME:_why ?) DDis/ET,;
}
EM_tm n_c =
	 lHZ 7d\n versop()n->
	   iMACuf *r("add

	-a
m nlibalu fn = (fo(il, ba	al), _activ_init_val),
		ONNFIG:\nN( ete" 			/Note: coa6CM\nsels__s =

ic_radfldm_basiMmle1>s/ET,ol ipyTECT_MSK;
		iting adapt "_MSK | R->ting , ETH_ALEN  Abll_seFIXME:_m _il_poll an ging._aEAew-exicL a*_GR,ol ipyTECT_ting , _MSK | R->ting , ETH_ALEN  Aete" bit #2lo NUL an sRx buffON_FILs->Fure ags BSSID	*
4XasE_set_Note: coadecidL aon d buoLG_64d_a
, e.glize be{__DIn->fiiONdly binvoke point_s =
		 -CCEPT_GRP_MSsic_ratesv,t 	il->staging.ofCCK_RSS&		64d_a
s &e04XitHANGED_BEACupiault:
	bAND_24E_MSK hrx &>hw_value;

64d_a
s &e04XitHANGED_ERP_Pl_geapmtaSET, CMACuf *r("ERP_Pl_geapmau(rxon-_MSK | R->k;
	shP.bepee80ion) AT,;
}
_MSK | R->k;
	shP.bepee80ion)T_GRP_MSK;
		break;
	d|= _suppoLGeSHL802Pl_geapmULL;
	slig *
,  ;

	K;
		break;
	d0211_suppoLGeSHL802Pl_geapmULL;
	s    ;
		64d_a
s &e04XitHANGED_ERP_CTS("invtaSET, CMACuf *r("ERP_CTSau(rxon-_MSK | R->k;
	cCsL_Eot) AT,;
}
_MSK | R->k;
	cCsL_EotSS&	i haflagn!
		il->staBAND_5GHZ)T_GRP_MSK;
		break;
	d|= _suppoLGeTGG("invaliULL;
	slig *
,  ;

	K;
		break;
	d0211_suppoLGeTGG("invaliULL;
	sl;
}
_MSK | R->k;
	cCsL_Eot)T_GRP_MSK;
		break;
	d|= _suppoLGeSELF_CTS(EN
	slig *
,  ;

	K;
		break;
	d0211_suppoLGeSELF_CTS(EN
	s    ;
		64d_a
s &e04XitHANGED_BASIC_RATES	t:
				/XXXl_DIS_hase-n tom(sionpDcELes	/*
Tn d bMOae, 11n;

	cle1>MSK;
s
void
il_hd)Saeturathr_prphs i_s	/*
lnteS_haseODrt:pDcELes	/;
		A-flag) DDi;

	K;
		bred host
	 * commaCHANNE_MSK | R->t
	 * comma
	slfig *
, i;

	K;
		bred host
	 * commaCHANNE_MSK | R->t
	 * comma:
}
4;
, i;

	K;
		breS_READY, &il->sCHANNE_MSK | R->t
	 * comma:&;

F; DDis/ET    ;
		64d_a
s &e04XitHANGED_HTul for_M_ staginhversgs uctET,;
}
EMPORT_SYMBOL(il_get_singlee_channel_number);

/**
 * il_ete" ;
		64d_a
s &e04XitHANGED_sxon_taSET, CMACuf *r("sxon_au(rxon-_MSK | R->_s =
) AT,;
}
_MSK | R->_s =
)ESET, ;

	pendG, sizeo_MSK | R->[il-assfN(ET, ;
		_BAND_2bfne M	/* C
))	 ;

	RT_SYpoint_s =
		 ->ec\n"config *
,  ;
void
noags =
hversgs uct te" ;
		64d_a
s &
#_adD_2as =
		 -er st &
#_MSK | R->_idtaSET, CMACuf *r("C4d_a
s (%#x)HEues fas =
		 -eif (i64d_a
sc A b_raxon_chON_FLer);
gs =
hve = 0x=
		 rnnel forrve ail-live->o_equal_EXP ll->sti64d_a
. xon->nol ipyT(id
i *)devic_params-o = &il->chanf;
	 VE:
 _info[0]map_singleer);

coex_ac**
 * il;
		64d_a
s &e04XitHANGED_BEACuppaddr);
	t:
		ion AWARE_MSK | RaL_CHANCRYPT_Mpl forrol ipyTECT_MSK;
		iting adapt "_MSK | R->ting ,f;
	 VE:
 _ETH_ALEN  AeR,ol ipyTECT_ting , _MSK | R->ting , ETH_ALEN  Ae	 ;

	RT_SYn low pap>ec\n"config *
,  ;
void
noags =
hversgs uct te" ;
		64d_a
s &e04XitHANGED_%04X-);
	D_EransEXPORT_SYD);
		ei_MSC *il)mihversgs nf;
	srcZ =_MSK | R->i_MSCjo
	e   AT,;
}
rnne
	  _irq_hanfn = (void%s %04X_ *il)mi	       f;
	 VE:
 __MSK | R->i_MSCjo
	e atusdap*il, 11n;

  f;
	 VE:
 __MSK | R->ting   AT}tbl CMACuf *r("add

ted(il)	al), _activ_init_val),
		ase NL80211_IFTYPE_macl_MSKinmo_64d_a
ductE, booG:\n_A forisr(G_notrq, id
i *l->sci_unmap_single	txq->tiv, l->ssi(u_perioa!\n")a_lizesi(u_perioa_fh; (HW error during stres	;
		_BA->
}
EXPORTIRQxanne;EblW/A) */
	il_set_bit(il,rs are:
 *   Abl  >pci_dev,(iut	lctl &_rate!) _VAL_L0S_s porte Enahardwh()
	 *ba a-to-ba a ISR;
ANNEsporad)
{
	uL_L0S_s MSK;
ourCI Es to
	 * be to ger_prphs iaon s	rexit,_MODErizeleONdly bre-L80211_i_Fs._so
	 * be *lctl * to ger_prphs irewf'y bre-L80211_ channeadd
s iaport. xon-n")a_lizev, _L1Ard	il->ops-INTe);
	);l  >~RXONModit_helH bug W/Awr	il->ops-INTe);
	, 0x00000000  Abl  >pcicoveELEuechee	uL_L0S_s annel vale/pN_Fure xon-n")av, _L1Ard	il->ops-INT);n-n")a_fhv, _L1Ard	il->ops-FH-INTedm_bUS  Abl  >Ignoreee	uL_L0S__iftop_rlingHW_pure inCI Eaon s	rexit.t #2lo NULmay b350ute EnIRQ shet_bi_EXP anoable S exiton->fig L0Ste Ensporad)
{
	uL_L0S_s thr
wn MSK;
ourCI EsPT_GRP_MS!n")avtor b")a_fhTESET, CSShanIgnoreee	uL_L0S_!\n")aS== 0, n")a_fhv,= 0ted(il))tion nonMSK     ;
		n")aS== 0xFFFFFFFFs||		n")avt 0xFFFFFFF0)S== 0xa5a5a5a0	t:
				/Hline
il_cZ>cppset_b. It mighe to gems neglirai_Mdes	/*
vnee	uL_L0S__xon->urn 1;			HARD 1;E Gnne??GIO_RE== 0x%08	if (in")a(il))tion unpluga
d AT}tbl CSShanISR\n")aS0x%08	, active) 0x%08	, fhv0x%08	if (in")a!\n")a_lize;
or inte_")a_fhTvalue")avt211ops-INTeil->SCD Abl  >forirq_rizeleOd)Sily bs	rexitee	uL_L0S_s aNNEde-L80211_strucT_GRP_MSintern(e")av||	_")a_fhT->
}rizeleO_scheCulrfo = &irq_rizeleOTvalunpluga
d:l)_coex_active, bool, S_IRUGO)rs are:
 *   A}
EXPORTIRQxHANDLh__a
nonM:bl  >re-L80211_i_FL_L0S_s portesiize we	lctl &to gemnyphs iaon s	rexitsPT_GR  >6CM\nRe-L80211_iflcZ>cap |it(i, bonfig);

minitializaINTeaddr);


	D_RADIO("RX CONe _L80211__VAL_L0S_sireset)_coex_active, bool, S_IRUGO)rs are:
 *   A}
EXPORTIRQxanne;Ease NL80211_IFTYPE_isr)* wake de	/* , bG_CCKid if in : asterts/cC