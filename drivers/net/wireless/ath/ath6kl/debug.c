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

#include "core.h"

#include <linux/skbuff.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/export.h>

#include "debug.h"
#include "target.h"

struct ath6kl_fwlog_slot {
	__le32 timestamp;
	__le32 length;

	/* max ATH6KL_FWLOG_PAYLOAD_SIZE bytes */
	u8 payload[0];
};

#define ATH6KL_FWLOG_MAX_ENTRIES 20

#define ATH6KL_FWLOG_VALID_MASK 0x1ffff

void ath6kl_printk(const char *level, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	printk("%sath6kl: %pV", level, &vaf);

	va_end(args);
}
EXPORT_SYMBOL(ath6kl_printk);

void ath6kl_info(const char *fmt, ...)
{
	struct va_format vaf = {
		.fmt = fmt,
	};
	va_list args;

	va_start(args, fmt);
	vaf.va = &args;
	ath6kl_printk(KERN_INFO, "%pV", &vaf);
	trace_ath6kl_log_info(&vaf);
	va_end(args);
}
EXPORT_SYMBOL(ath6kl_info);

void ath6kl_err(const char *fmt, ...)
{
	struct va_format vaf = {
		.fmt = fmt,
	};
	va_list args;

	va_start(args, fmt);
	vaf.va = &args;
	ath6kl_printk(KERN_ERR, "%pV", &vaf);
	trace_ath6kl_log_err(&vaf);
	va_end(args);
}
EXPORT_SYMBOL(ath6kl_err);

void ath6kl_warn(const char *fmt, ...)
{
	struct va_format vaf = {
		.fmt = fmt,
	};
	va_list args;

	va_start(args, fmt);
	vaf.va = &args;
	ath6kl_printk(KERN_WARNING, "%pV", &vaf);
	trace_ath6kl_log_warn(&vaf);
	va_end(args);
}
EXPORT_SYMBOL(ath6kl_warn);

int ath6kl_read_tgt_stats(struct ath6kl *ar, struct ath6kl_vif *vif)
{
	long left;

	if (down_interruptible(&ar->sem))
		return -EBUSY;

	set_bit(STATS_UPDATE_PEND, &vif->flags);

	if (ath6kl_wmi_get_stats_cmd(ar->wmi, 0)) {
		up(&ar->sem);
		return -EIO;
	}

	left = wait_event_interruptible_timeout(ar->event_wq,
						!test_bit(STATS_UPDATE_PEND,
						&vif->flags), WMI_TIMEOUT);

	up(&ar->sem);

	if (left <= 0)
		return -ETIMEDOUT;

	return 0;
}
EXPORT_SYMBOL(ath6kl_read_tgt_stats);

#ifdef CONFIG_ATH6KL_DEBUG

void ath6kl_dbg(enum ATH6K_DEBUG_MASK mask, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	if (debug_mask & mask)
		ath6kl_printk(KERN_DEBUG, "%pV", &vaf);

	trace_ath6kl_log_dbg(mask, &vaf);

	va_end(args);
}
EXPORT_SYMBOL(ath6kl_dbg);

void ath6kl_dbg_dump(enum ATH6K_DEBUG_MASK mask,
		     const char *msg, const char *prefix,
		     const void *buf, size_t len)
{
	if (debug_mask & mask) {
		if (msg)
			ath6kl_dbg(mask, "%s\n", msg);

		print_hex_dump_bytes(prefix, DUMP_PREFIX_OFFSET, buf, len);
	}

	/* tracing code doesn't like null strings :/ */
	trace_ath6kl_log_dbg_dump(msg ? msg : "", prefix ? prefix : "",
				  buf, len);
}
EXPORT_SYMBOL(ath6kl_dbg_dump);

#define REG_OUTPUT_LEN_PER_LINE	25
#define REGTYPE_STR_LEN		100

struct ath6kl_diag_reg_info {
	u32 reg_start;
	u32 reg_end;
	const char *reg_info;
};

static const struct ath6kl_diag_reg_info diag_reg[reg_info;
};

static const struct ath6kl_diag_reg_info diag_reg[reg_info;
};

static const struct ath6kl_diag_reg_info diag_reg[reg_info;
};

static const struct ath6kl_diag_reg_info diag_reg[reg_info;
};

static const struct ath6kl_diag_reg_info diag_reg[ret len)o;
