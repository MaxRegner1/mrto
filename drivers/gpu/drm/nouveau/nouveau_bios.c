/*
 * Copyright 2005-2006 Erik Waling
 * Copyright 2006 Stephane Marchesin
 * Copyright 2007-2009 Stuart Bennett
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <drm/drmP.h>

#include "nouveau_drv.h"
#include "nouveau_reg.h"
#include "dispnv04/hw.h"
#include "nouveau_encoder.h"

#include <linux/io-mapping.h>
#include <linux/firmware.h>

/* these defines are made up */
#define NV_CIO_CRE_44_HEADA 0x0
#define NV_CIO_CRE_44_HEADB 0x3
#define FEATURE_MOBILE 0x10	/* also FEATURE_QUADRO for BMP */

#define EDID1_LEN 128

#define BIOSLOG(sip, fmt, arg...) NV_DEBUG(sip->dev, fmt, ##arg)
#define LOG_OLD_VALUE(x)

struct init_exec {
	bool execute;
	bool repeat;
};

static bool nv_cksum(const uint8_t *data, unsigned int length)
{
	/*
	 * There's a few checksums in the BIOS, so here's a generic checking
	 * function.
	 */
	int i;
	uint8_t sum = 0;

	for (i = 0; i < length; i++)
		sum += data[i];

	if (sum)
		return true;

	return false;
}

static uint16_t clkcmptable(struct nvbios *bios, uint16_t clktable, int pxclk)
{
	int compare_record_len, i = 0;
	uint16_t compareclk, scriptptr = 0;

	if (bios->major_version < 5) /* pre BIT */
		compare_record_len = 3;
	else
		compare_record_len = 4;

	do {
		compareclk = ROM16(bios->data[clktable + compare_record_len * i]);
		if (pxclk >= compareclk * 10) {
			if (bios->major_version < 5) {
				uint8_t tmdssub = bios->data[clktable + 2 + compare_record_len * i];
				scriptptr = ROM16(bios->data[bios->init_script_tbls_ptr + tmdssub * 2]);
			} else
				scriptptr = ROM16(bios->data[clktable + 2 + compare_record_len * i]);
			break;
		}
		i++;
	} while (compareclk);

	return scriptptr;
}

static void
run_digital_op_script(struct drm_device *dev, uint16_t scriptptr,
		      struct dcb_output *dcbent, int head, bool dl)
{
	struct nouveau_drm *drm = nouveau_drm(dev);

	NV_INFO(drm, "0x%04X: Parsing digital output script table\n",
		 scriptptr);
	NVWriteVgaCrtc(dev, 0, NV_CIO_CRE_44, head ? NV_CIO_CRE_44_HEADB :
					         NV_CIO_CRE_44_HEADA);
	nouveau_bios_run_init_table(dev, scriptptr, dcbent, head);

	nv04_dfp_bind_head(dev, dcbent, head, dl);
}

static int call_lvds_manufacturer_script(struct drm_device *dev, struct dcb_output *dcbent, int head, enum LVDS_script script)
{
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct nvbios *bios = &drm->vbios;
	uint8_t sub = bios->data[bios->fp.xlated_entry + script] + (bios->fp.link_c_increment && dcbent->or & DCB_OUTPUT_C ? 1 : 0);
	uint16_t scriptofs = ROM16(bios->data[bios->init_script_tbls_ptr + sub * 2]);

	if (!bios->fp.xlated_entry || !sub || !scriptofs)
		return -EINVAL;

	run_digital_op_script(dev, scriptofs, dcbent, head, bios->fp.dual_link);

	if (script == LVDS_PANEL_OFF) {
		/* off-on delay in ms */
		mdelay(ROM16(bios->data[bios->fp.xlated_entry + 7]));
	}
#ifdef __powerpc__
	/* Powerbook specific quirks */
	if (script == LVDS_RESET &&
	    (dev->pdev->device == 0x0179 || dev->pdev->device == 0x0189 ||
	     dev->pdev->device == 0x0329))
		nv_write_tmds(dev, dcbent->or, 0, 0x02, 0x72);
#endif

	return 0;
}

static int run_lvds_table(struct drm_device *dev, struct dcb_output *dcbent, int head, enum LVDS_script script, int pxclk)
{
	/*
	 * The BIT LVDS table's header has the information to setup the
	 * necessary registers. Following the standard 4 byte header are:
	 * A bitmask byte and a dual-link transition pxclk value for use in
	 * selecting the init script when not using straps; 4 script pointers
	 * for panel power, selected by output and on/off; and 8 table pointers
	 * for panel init, the needed one determined by output, and bits in the
	 * conf byte. These tables are similar to the TMDS tables, consisting
	 * of a list of pxclks and script pointers.
	 */
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct nvbios *bios = &drm->vbios;
	unsigned int outputset = (dcbent->or == 4) ? 1 : 0;
	uint16_t scriptptr = 0, clktable;

	/*
	 * For now we assume version 3.0 table - g80 support will need some
	 * changes
	 */

	switch (script) {
	case LVDS_INIT:
		return -ENOSYS;
	case LVDS_BACKLIGHT_ON:
	case LVDS_PANEL_ON:
		scriptptr = ROM16(bios->data[bios->fp.lvdsmanufacturerpointer + 7 + outputset * 2]);
		break;
	case LVDS_BACKLIGHT_OFF:
	case LVDS_PANEL_OFF:
		scriptptr = ROM16(bios->data[bios->fp.lvdsmanufacturerpointer + 11 + outputset * 2]);
		break;
	case LVDS_RESET:
		clktable = bios->fp.lvdsmanufacturerpointer + 15;
		if (dcbent->or == 4)
			clktable += 8;

		if (dcbent->lvdsconf.use_straps_for_mode) {
			if (bios->fp.dual_link)
				clktable += 4;
			if (bios->fp.if_is_24bit)
				clktable += 2;
		} else {
			/* using EDID */
			int cmpval_24bit = (dcbent->or == 4) ? 4 : 1;

			if (bios->fp.dual_link) {
				clktable += 4;
				cmpval_24bit <<= 1;
			}

			if (bios->fp.strapless_is_24bit & cmpval_24bit)
				clktable += 2;
		}

		clktable = ROM16(bios->data[clktable]);
		if (!clktable) {
			NV_ERROR(drm, "Pixel clock comparison table not found\n");
			return -ENOENT;
		}
		scriptptr = clkcmptable(bios, clktable, pxclk);
	}

	if (!scriptptr) {
		NV_ERROR(drm, "LVDS output init script not found\n");
		return -ENOENT;
	}
	run_digital_op_script(dev, scriptptr, dcbent, head, bios->fp.dual_link);

	return 0;
}

int call_lvds_script(struct drm_device *dev, struct dcb_output *dcbent, int head, enum LVDS_script script, int pxclk)
{
	/*
	 * LVDS operations are multiplexed in an effort to present a single API
	 * which works with two vastly differing underlying structures.
	 * This acts as the demux
	 */

	struct nouveau_drm *drm = nouveau_drm(dev);
	struct nvif_object *device = &drm->client.device.object;
	struct nvbios *bios = &drm->vbios;
	uint8_t lvds_ver = bios->data[bios->fp.lvdsmanufacturerpointer];
	uint32_t sel_clk_binding, sel_clk;
	int ret;

	if (bios->fp.last_script_invoc == (script << 1 | head) || !lvds_ver ||
	    (lvds_ver >= 0x30 && script == LVDS_INIT))
		return 0;

	if (!bios->fp.lvds_init_run) {
		bios->fp.lvds_init_run = true;
		call_lvds_script(dev, dcbent, head, LVDS_INIT, pxclk);
	}

	if (script == LVDS_PANEL_ON && bios->fp.reset_after_pclk_change)
		call_lvds_script(dev, dcbent, head, LVDS_RESET, pxclk);
	if (script == LVDS_RESET && bios->fp.power_off_for_reset)
		call_lvds_script(dev, dcbent, head, LVDS_PANEL_OFF, pxclk);

	NV_INFO(drm, "Calling LVDS script %d:\n", script);

	/* don't let script change pll->head binding */
	sel_clk_binding = nvif_rd32(device, NV_PRAMDAC_SEL_CLK) & 0x50000;

	if (lvds_ver < 0x30)
		ret = call_lvds_manufacturer_script(dev, dcbent, head, script);
	else
		ret = run_lvds_table(dev, dcbent, head, script, pxclk);

	bios->fp.last_script_invoc = (script << 1 | head);

	sel_clk = NVReadRAMDAC(dev, 0, NV_PRAMDAC_SEL_CLK) & ~0x50000;
	NVWriteRAMDAC(dev, 0, NV_PRAMDAC_SEL_CLK, sel_clk | sel_clk_binding);
	/* some scripts set a value in NV_PBUS_POWERCTRL_2 and break video overlay */
	nvif_wr32(device, NV_PBUS_POWERCTRL_2, 0);

	return ret;
}

struct lvdstableheader {
	uint8_t lvds_ver, headerlen, recordlen;
};

static int parse_lvds_manufacturer_table_header(struct drm_device *dev, struct nvbios *bios, struct lvdstableheader *lth)
{
	/*
	 * BMP version (0xa) LVDS table has a simple header of version and
	 * record length. The BIT LVDS table has the typical BIT table header:
	 * version byte, header length byte, record length byte, and a byte for
	 * the maximum number of records that can be held in the table.
	 */

	struct nouveau_drm *drm = nouveau_drm(dev);
	uint8_t lvds_ver, headerlen, recordlen;

	memset(lth, 0, sizeof(struct lvdstableheader));

	if (bios->fp.lvdsmanufacturerpointer == 0x0) {
		NV_ERROR(drm, "Pointer to LVDS manufacturer table invalid\n");
		return -EINVAL;
	}

	lvds_ver = bios->data[bios->fp.lvdsmanufacturerpointer];

	switch (lvds_ver) {
	case 0x0a:	/* pre NV40 */
		headerlen = 2;
		recordlen = bios->data[bios->fp.lvdsmanufacturerpointer + 1];
		break;
	case 0x30:	/* NV4x */
		headerlen = bios->data[bios->fp.lvdsmanufacturerpointer + 1];
		if (headerlen < 0x1f) {
			NV_ERROR(drm, "LVDS table header not understood\n");
			return -EINVAL;
		}
		recordlen = bios->data[bios->fp.lvdsmanufacturerpointer + 2];
		break;
	case 0x40:	/* G80/G90 */
		headerlen = bios->data[bios->fp.lvdsmanufacturerpointer + 1];
		if (headerlen < 0x7) {
			NV_ERROR(drm, "LVDS table header not understood\n");
			return -EINVAL;
		}
		recordlen = bios->data[bios->fp.lvdsmanufacturerpointer + 2];
		break;
	default:
		NV_ERROR(drm,
			 "LVDS table revision %d.%d not currently supported\n",
			 lvds_ver >> 4, lvds_ver & 0xf);
		return -ENOSYS;
	}

	lth->lvds_ver = lvds_ver;
	lth->headerlen = headerlen;
	lth->recordlen = recordlen;

	return 0;
}

static int
get_fp_strap(struct drm_device *dev, struct nvbios *bios)
{
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct nvif_object *device = &drm->client.device.object;

	/*
	 * The fp strap is normally dictated by the "User Strap" in
	 * PEXTDEV_BOOT_0[20:16], but on BMP cards when bit 2 of the
	 * Internal_Flags struct at 0x48 is set, the user strap gets overriden
	 * by the PCI subsystem ID during POST, but not before the previous user
	 * strap has been committed to CR58 for CR57=0xf on head A, which may be
	 * read and used instead
	 */

	if (bios->major_version < 5 && bios->data[0x48] & 0x4)
		return NVReadVgaCrtc5758(dev, 0, 0xf) & 0xf;

	if (drm->client.device.info.family >= NV_DEVICE_INFO_V0_MAXWELL)
		return nvif_rd32(device, 0x001800) & 0x0000000f;
	else
	if (drm->client.device.info.family >= NV_DEVICE_INFO_V0_TESLA)
		return (nvif_rd32(device, NV_PEXTDEV_BOOT_0) >> 24) & 0xf;
	else
		return (nvif_rd32(device, NV_PEXTDEV_BOOT_0) >> 16) & 0xf;
}

static int parse_fp_mode_table(struct drm_device *dev, struct nvbios *bios)
{
	struct nouveau_drm *drm = nouveau_drm(dev);
	uint8_t *fptable;
	uint8_t fptable_ver, headerlen = 0, recordlen, fpentries = 0xf, fpindex;
	int ret, ofs, fpstrapping;
	struct lvdstableheader lth;

	if (bios->fp.fptablepointer == 0x0) {
		/* Most laptop cards lack an fp table. They use DDC. */
		NV_DEBUG(drm, "Pointer to flat panel table invalid\n");
		bios->digital_min_front_porch = 0x4b;
		return 0;
	}

	fptable = &bios->data[bios->fp.fptablepointer];
	fptable_ver = fptable[0];

	switch (fptable_ver) {
	/*
	 * BMP version 0x5.0x11 BIOSen have version 1 like tables, but no
	 * version field, and miss one of the spread spectrum/PWM bytes.
	 * This could affect early GF2Go parts (not seen any appropriate ROMs
	 * though). Here we assume that a version of 0x05 matches this case
	 * (combining with a BMP version check would be better), as the
	 * common case for the panel type field is 0x0005, and that is in
	 * fact what we are reading the first byte of.
	 */
	case 0x05:	/* some NV10, 11, 15, 16 */
		recordlen = 42;
		ofs = -1;
		break;
	case 0x10:	/* some NV15/16, and NV11+ */
		recordlen = 44;
		ofs = 0;
		break;
	case 0x20:	/* NV40+ */
		headerlen = fptable[1];
		recordlen = fptable[2];
		fpentries = fptable[3];
		/*
		 * fptable[4] is the minimum
		 * RAMDAC_FP_HCRTC -> RAMDAC_FP_HSYNC_START gap
		 */
		bios->digital_min_front_porch = fptable[4];
		ofs = -7;
		break;
	default:
		NV_ERROR(drm,
			 "FP table revision %d.%d not currently supported\n",
			 fptable_ver >> 4, fptable_ver & 0xf);
		return -ENOSYS;
	}

	if (!bios->is_mobile) /* !mobile only needs digital_min_front_porch */
		return 0;

	ret = parse_lvds_manufacturer_table_header(dev, bios, &lth);
	if (ret)
		return ret;

	if (lth.lvds_ver == 0x30 || lth.lvds_ver == 0x40) {
		bios->fp.fpxlatetableptr = bios->fp.lvdsmanufacturerpointer +
							lth.headerlen + 1;
		bios->fp.xlatwidth = lth.recordlen;
	}
	if (bios->fp.fpxlatetableptr == 0x0) {
		NV_ERROR(drm, "Pointer to flat panel xlat table invalid\n");
		return -EINVAL;
	}

	fpstrapping = get_fp_strap(dev, bios);

	fpindex = bios->data[bios->fp.fpxlatetableptr +
					fpstrapping * bios->fp.xlatwidth];

	if (fpindex > fpentries) {
		NV_ERROR(drm, "Bad flat panel table index\n");
		return -ENOENT;
	}

	/* nv4x cards need both a strap value and fpindex of 0xf to use DDC */
	if (lth.lvds_ver > 0x10)
		bios->fp_no_ddc = fpstrapping != 0xf || fpindex != 0xf;

	/*
	 * If either the strap or xlated fpindex value are 0xf there is no
	 * panel using a strap-derived bios mode present.  this condition
	 * includes, but is different from, the DDC panel indicator above
	 */
	if (fpstrapping == 0xf || fpindex == 0xf)
		return 0;

	bios->fp.mode_ptr = bios->fp.fptablepointer + headerlen +
			    recordlen * fpindex + ofs;

	NV_INFO(drm, "BIOS FP mode: %dx%d (%dkHz pixel clock)\n",
		 ROM16(bios->data[bios->fp.mode_ptr + 11]) + 1,
		 ROM16(bios->data[bios->fp.mode_ptr + 25]) + 1,
		 ROM16(bios->data[bios->fp.mode_ptr + 7]) * 10);

	return 0;
}

bool nouveau_bios_fp_mode(struct drm_device *dev, struct drm_display_mode *mode)
{
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct nvbios *bios = &drm->vbios;
	uint8_t *mode_entry = &bios->data[bios->fp.mode_ptr];

	if (!mode)	/* just checking whether we can produce a mode */
		return bios->fp.mode_ptr;

	memset(mode, 0, sizeof(struct drm_display_mode));
	/*
	 * For version 1.0 (version in byte 0):
	 * bytes 1-2 are "panel type", including bits on whether Colour/mono,
	 * single/dual link, and type (TFT etc.)
	 * bytes 3-6 are bits per colour in RGBX
	 */
	mode->clock = ROM16(mode_entry[7]) * 10;
	/* bytes 9-10 is HActive */
	mode->hdisplay = ROM16(mode_entry[11]) + 1;
	/*
	 * bytes 13-14 is HValid Start
	 * bytes 15-16 is HValid End
	 */
	mode->hsync_start = ROM16(mode_entry[17]) + 1;
	mode->hsync_end = ROM16(mode_entry[19]) + 1;
	mode->htotal = ROM16(mode_entry[21]) + 1;
	/* bytes 23-24, 27-30 similarly, but vertical */
	mode->vdisplay = ROM16(mode_entry[25]) + 1;
	mode->vsync_start = ROM16(mode_entry[31]) + 1;
	mode->vsync_end = ROM16(mode_entry[33]) + 1;
	mode->vtotal = ROM16(mode_entry[35]) + 1;
	mode->flags |= (mode_entry[37] & 0x10) ?
			DRM_MODE_FLAG_PHSYNC : DRM_MODE_FLAG_NHSYNC;
	mode->flags |= (mode_entry[37] & 0x1) ?
			DRM_MODE_FLAG_PVSYNC : DRM_MODE_FLAG_NVSYNC;
	/*
	 * bytes 38-39 relate to spread spectrum settings
	 * bytes 40-43 are something to do with PWM
	 */

	mode->status = MODE_OK;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	return bios->fp.mode_ptr;
}

int nouveau_bios_parse_lvds_table(struct drm_device *dev, int pxclk, bool *dl, bool *if_is_24bit)
{
	/*
	 * The LVDS table header is (mostly) described in
	 * parse_lvds_manufacturer_table_header(): the BIT header additionally
	 * contains the dual-link transition pxclk (in 10s kHz), at byte 5 - if
	 * straps are not being used for the panel, this specifies the frequency
	 * at which modes should be set up in the dual link style.
	 *
	 * Following the header, the BMP (ver 0xa) table has several records,
	 * indexed by a separate xlat table, indexed in turn by the fp strap in
	 * EXTDEV_BOOT. Each record had a config byte, followed by 6 script
	 * numbers for use by INIT_SUB which controlled panel init and power,
	 * and finally a dword of ms to sleep between power off and on
	 * operations.
	 *
	 * In the BIT versions, the table following the header serves as an
	 * integrated config and xlat table: the records in the table are
	 * indexed by the FP strap nibble in EXTDEV_BOOT, and each record has
	 * two bytes - the first as a config byte, the second for indexing the
	 * fp mode table pointed to by the BIT 'D' table
	 *
	 * DDC is not used until after card init, so selecting the correct table
	 * entry and setting the dual link flag for EDID equipped panels,
	 * requiring tests against the native-mode pixel clock, cannot be done
	 * until later, when this function should be called with non-zero pxclk
	 */
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct nvbios *bios = &drm->vbios;
	int fpstrapping = get_fp_strap(dev, bios), lvdsmanufacturerindex = 0;
	struct lvdstableheader lth;
	uint16_t lvdsofs;
	int ret, chip_version = bios->chip_version;

	ret = parse_lvds_manufacturer_table_header(dev, bios, &lth);
	if (ret)
		return ret;

	switch (lth.lvds_ver) {
	case 0x0a:	/* pre NV40 */
		lvdsmanufacturerindex = bios->data[
					bios->fp.fpxlatemanufacturertableptr +
					fpstrapping];

		/* we're done if this isn't the EDID panel case */
		if (!pxclk)
			break;

		if (chip_version < 0x25) {
			/* nv17 behaviour
			 *
			 * It seems the old style lvds script pointer is reused
			 * to select 18/24 bit colour depth for EDID panels.
			 */
			lvdsmanufacturerindex =
				(bios->legacy.lvds_single_a_script_ptr & 1) ?
									2 : 0;
			if (pxclk >= bios->fp.duallink_transition_clk)
				lvdsmanufacturerindex++;
		} else if (chip_version < 0x30) {
			/* nv28 behaviour (off-chip encoder)
			 *
			 * nv28 does a complex dance of first using byte 121 of
			 * the EDID to choose the lvdsmanufacturerindex, then
			 * later attempting to match the EDID manufacturer and
			 * product IDs in a table (signature 'pidt' (panel id
			 * table?)), setting an lvdsmanufacturerindex of 0 and
			 * an fp strap of the match index (or 0xf if none)
			 */
			lvdsmanufacturerindex = 0;
		} else {
			/* nv31, nv34 behaviour */
			lvdsmanufacturerindex = 0;
			if (pxclk >= bios->fp.duallink_transition_clk)
				lvdsmanufacturerindex = 2;
			if (pxclk >= 140000)
				lvdsmanufacturerindex = 3;
		}

		/*
		 * nvidia set the high nibble of (cr57=f, cr58) to
		 * lvdsmanufacturerindex in this case; we don't
		 */
		break;
	case 0x30:	/* NV4x */
	case 0x40:	/* G80/G90 */
		lvdsmanufacturerindex = fpstrapping;
		break;
	default:
		NV_ERROR(drm, "LVDS table revision not currently supported\n");
		return -ENOSYS;
	}

	lvdsofs = bios->fp.xlated_entry = bios->fp.lvdsmanufacturerpointer + lth.headerlen + lth.recordlen * lvdsmanufacturerindex;
	switch (lth.lvds_ver) {
	case 0x0a:
		bios->fp.power_off_for_reset = bios->data[lvdsofs] & 1;
		bios->fp.reset_after_pclk_change = bios->data[lvdsofs] & 2;
		bios->fp.dual_link = bios->data[lvdsofs] & 4;
		bios->fp.link_c_increment = bios->data[lvdsofs] & 8;
		*if_is_24bit = bios->data[lvdsofs] & 16;
		break;
	case 0x30:
	case 0x40:
		/*
		 * No sign of the "power off for reset" or "reset for panel
		 * on" bits, but it's safer to assume we should
		 */
		bios->fp.power_off_for_reset = true;
		bios->fp.reset_after_pclk_change = true;

		/*
		 * It's ok lvdsofs is wrong for nv4x edid case; dual_link is
		 * over-written, and if_is_24bit isn't used
		 */
		bios->fp.dual_link = bios->data[lvdsofs] & 1;
		bios->fp.if_is_24bit = bios->data[lvdsofs] & 2;
		bios->fp.strapless_is_24bit = bios->data[bios->fp.lvdsmanufacturerpointer + 4];
		bios->fp.duallink_transition_clk = ROM16(bios->data[bios->fp.lvdsmanufacturerpointer + 5]) * 10;
		break;
	}

	/* set dual_link flag for EDID case */
	if (pxclk && (chip_version < 0x25 || chip_version > 0x28))
		bios->fp.dual_link = (pxclk >= bios->fp.duallink_transition_clk);

	*dl = bios->fp.dual_link;

	return 0;
}

int run_tmds_table(struct drm_device *dev, struct dcb_output *dcbent, int head, int pxclk)
{
	/*
	 * the pxclk parameter is in kHz
	 *
	 * This runs the TMDS regs setting code found on BIT bios cards
	 *
	 * For ffs(or) == 1 use the first table, for ffs(or) == 2 and
	 * ffs(or) == 3, use the second.
	 */

	struct nouveau_drm *drm = nouveau_drm(dev);
	struct nvif_object *device = &drm->client.device.object;
	struct nvbios *bios = &drm->vbios;
	int cv = bios->chip_version;
	uint16_t clktable = 0, scriptptr;
	uint32_t sel_clk_binding, sel_clk;

	/* pre-nv17 off-chip tmds uses scripts, post nv17 doesn't */
	if (cv >= 0x17 && cv != 0x1a && cv != 0x20 &&
	    dcbent->location != DCB_LOC_ON_CHIP)
		return 0;

	switch (ffs(dcbent->or)) {
	case 1:
		clktable = bios->tmds.output0_script_ptr;
		break;
	case 2:
	case 3:
		clktable = bios->tmds.output1_script_ptr;
		break;
	}

	if (!clktable) {
		NV_ERROR(drm, "Pixel clock comparison table not found\n");
		return -EINVAL;
	}

	scriptptr = clkcmptable(bios, clktable, pxclk);

	if (!scriptptr) {
		NV_ERROR(drm, "TMDS output init script not found\n");
		return -ENOENT;
	}

	/* don't let script change pll->head binding */
	sel_clk_binding = nvif_rd32(device, NV_PRAMDAC_SEL_CLK) & 0x50000;
	run_digital_op_script(dev, scriptptr, dcbent, head, pxclk >= 165000);
	sel_clk = NVReadRAMDAC(dev, 0, NV_PRAMDAC_SEL_CLK) & ~0x50000;
	NVWriteRAMDAC(dev, 0, NV_PRAMDAC_SEL_CLK, sel_clk | sel_clk_binding);

	return 0;
}

static void parse_script_table_pointers(struct nvbios *bios, uint16_t offset)
{
	/*
	 * Parses the init table segment for pointers used in script execution.
	 *
	 * offset + 0  (16 bits): init script tables pointer
	 * offset + 2  (16 bits): macro index table pointer
	 * offset + 4  (16 bits): macro table pointer
	 * offset + 6  (16 bits): condition table pointer
	 * offset + 8  (16 bits): io condition table pointer
	 * offset + 10 (16 bits): io flag condition table pointer
	 * offset + 12 (16 bits): init function table pointer
	 */

	bios->init_script_tbls_ptr = ROM16(bios->data[offset]);
}

static int parse_bit_A_tbl_entry(struct drm_device *dev, struct nvbios *bios, struct bit_entry *bitentry)
{
	/*
	 * Parses the load detect values for g80 cards.
	 *
	 * offset + 0 (16 bits): loadval table pointer
	 */

	struct nouveau_drm *drm = nouveau_drm(dev);
	uint16_t load_table_ptr;
	uint8_t version, headerlen, entrylen, num_entries;

	if (bitentry->length != 3) {
		NV_ERROR(drm, "Do not understand BIT A table\n");
		return -EINVAL;
	}

	load_table_ptr = ROM16(bios->data[bitentry->offset]);

	if (load_table_ptr == 0x0) {
		NV_DEBUG(drm, "Pointer to BIT loadval table invalid\n");
		return -EINVAL;
	}

	version = bios->data[load_table_ptr];

	if (version != 0x10) {
		NV_ERROR(drm, "BIT loadval table version %d.%d not supported\n",
			 version >> 4, version & 0xF);
		return -ENOSYS;
	}

	headerlen = bios->data[load_table_ptr + 1];
	entrylen = bios->data[load_table_ptr + 2];
	num_entries = bios->data[load_table_ptr + 3];

	if (headerlen != 4 || entrylen != 4 || num_entries != 2) {
		NV_ERROR(drm, "Do not understand BIT loadval table\n");
		return -EINVAL;
	}

	/* First entry is normal dac, 2nd tv-out perhaps? */
	bios->dactestval = ROM32(bios->data[load_table_ptr + headerlen]) & 0x3ff;

	return 0;
}

static int parse_bit_display_tbl_entry(struct drm_device *dev, struct nvbios *bios, struct bit_entry *bitentry)
{
	/*
	 * Parses the flat panel table segment that the bit entry points to.
	 * Starting at bitentry->offset:
	 *
	 * offset + 0  (16 bits): ??? table pointer - seems to have 18 byte
	 * records beginning with a freq.
	 * offset + 2  (16 bits): mode table pointer
	 */
	struct nouveau_drm *drm = nouveau_drm(dev);

	if (bitentry->length != 4) {
		NV_ERROR(drm, "Do not understand BIT display table\n");
		return -EINVAL;
	}

	bios->fp.fptablepointer = ROM16(bios->data[bitentry->offset + 2]);

	return 0;
}

static int parse_bit_init_tbl_entry(struct drm_device *dev, struct nvbios *bios, struct bit_entry *bitentry)
{
	/*
	 * Parses the init table segment that the bit entry points to.
	 *
	 * See parse_script_table_pointers for layout
	 */
	struct nouveau_drm *drm = nouveau_drm(dev);

	if (bitentry->length < 14) {
		NV_ERROR(drm, "Do not understand init table\n");
		return -EINVAL;
	}

	parse_script_table_pointers(bios, bitentry->offset);
	return 0;
}

static int parse_bit_i_tbl_entry(struct drm_device *dev, struct nvbios *bios, struct bit_entry *bitentry)
{
	/*
	 * BIT 'i' (info?) table
	 *
	 * offset + 0  (32 bits): BIOS version dword (as in B table)
	 * offset + 5  (8  bits): BIOS feature byte (same as for BMP?)
	 * offset + 13 (16 bits): pointer to table containing DAC load
	 * detection comparison values
	 *
	 * There's other things in the table, purpose unknown
	 */

	struct nouveau_drm *drm = nouveau_drm(dev);
	uint16_t daccmpoffset;
	uint8_t dacver, dacheaderlen;

	if (bitentry->length < 6) {
		NV_ERROR(drm, "BIT i table too short for needed information\n");
		return -EINVAL;
	}

	/*
	 * bit 4 seems to indicate a mobile bios (doesn't suffer from BMP's
	 * Quadro identity crisis), other bits possibly as for BMP feature byte
	 */
	bios->feature_byte = bios->data[bitentry->offset + 5];
	bios->is_mobile = bios->feature_byte & FEATURE_MOBILE;

	if (bitentry->length < 15) {
		NV_WARN(drm, "BIT i table not long enough for DAC load "
			       "detection comparison table\n");
		return -EINVAL;
	}

	daccmpoffset = ROM16(bios->data[bitentry->offset + 13]);

	/* doesn't exist on g80 */
	if (!daccmpoffset)
		return 0;

	/*
	 * The first value in the table, following the header, is the
	 * comparison value, the second entry is a comparison value for
	 * TV load detection.
	 */

	dacver = bios->data[daccmpoffset];
	dacheaderlen = bios->data[daccmpoffset + 1];

	if (dacver != 0x00 && dacver != 0x10) {
		NV_WARN(drm, "DAC load detection comparison table version "
			       "%d.%d not known\n", dacver >> 4, dacver & 0xf);
		return -ENOSYS;
	}

	bios->dactestval = ROM32(bios->data[daccmpoffset + dacheaderlen]);
	bios->tvdactestval = ROM32(bios->data[daccmpoffset + dacheaderlen + 4]);

	return 0;
}

static int parse_bit_lvds_tbl_entry(struct drm_device *dev, struct nvbios *bios, struct bit_entry *bitentry)
{
	/*
	 * Parses the LVDS table segment that the bit entry points to.
	 * Starting at bitentry->offset:
	 *
	 * offset + 0  (16 bits): LVDS strap xlate table pointer
	 */

	struct nouveau_drm *drm = nouveau_drm(dev);

	if (bitentry->length != 2) {
		NV_ERROR(drm, "Do not understand BIT LVDS table\n");
		return -EINVAL;
	}

	/*
	 * No idea if it's still called the LVDS manufacturer table, but
	 * the concept's close enough.
	 */
	bios->fp.lvdsmanufacturerpointer = ROM16(bios->data[bitentry->offset]);

	return 0;
}

static int
parse_bit_M_tbl_entry(struct drm_device *dev, struct nvbios *bios,
		      struct bit_entry *bitentry)
{
	/*
	 * offset + 2  (8  bits): number of options in an
	 * 	INIT_RAM_RESTRICT_ZM_REG_GROUP opcode option set
	 * offset + 3  (16 bits): pointer to strap xlate table for RAM
	 * 	restrict option selection
	 *
	 * There's a bunch of bits in this table other than the RAM restrict
	 * stuff that we don't use - their use currently unknown
	 */

	/*
	 * Older bios versions don't have a sufficiently long table for
	 * what we want
	 */
	if (bitentry->length < 0x5)
		return 0;

	if (bitentry->version < 2) {
		bios->ram_restrict_group_count = bios->data[bitentry->offset + 2];
		bios->ram_restrict_tbl_ptr = ROM16(bios->data[bitentry->offset + 3]);
	} else {
		bios->ram_restrict_group_count = bios->data[bitentry->offset + 0];
		bios->ram_restrict_tbl_ptr = ROM16(bios->data[bitentry->offset + 1]);
	}

	return 0;
}

static int parse_bit_tmds_tbl_entry(struct drm_device *dev, struct nvbios *bios, struct bit_entry *bitentry)
{
	/*
	 * Parses the pointer to the TMDS table
	 *
	 * Starting at bitentry->offset:
	 *
	 * offset + 0  (16 bits): TMDS table pointer
	 *
	 * The TMDS table is typically found just before the DCB table, with a
	 * characteristic signature of 0x11,0x13 (1.1 being version, 0x13 being
	 * length?)
	 *
	 * At offset +7 is a pointer to a script, which I don't know how to
	 * run yet.
	 * At offset +9 is a pointer to another script, likewise
	 * Offset +11 has a pointer to a table where the first word is a pxclk
	 * frequency and the second word a pointer to a script, which should be
	 * run if the comparison pxclk frequency is less than the pxclk desired.
	 * This repeats for decreasing comparison frequencies
	 * Offset +13 has a pointer to a similar table
	 * The selection of table (and possibly +7/+9 script) is dictated by
	 * "or" from the DCB.
	 */

	struct nouveau_drm *drm = nouveau_drm(dev);
	uint16_t tmdstableptr, script1, script2;

	if (bitentry->length != 2) {
		NV_ERROR(drm, "Do not understand BIT TMDS table\n");
		return -EINVAL;
	}

	tmdstableptr = ROM16(bios->data[bitentry->offset]);
	if (!tmdstableptr) {
		NV_ERROR(drm, "Pointer to TMDS table invalid\n");
		return -EINVAL;
	}

	NV_INFO(drm, "TMDS table version %d.%d\n",
		bios->data[tmdstableptr] >> 4, bios->data[tmdstableptr] & 0xf);

	/* nv50+ has v2.0, but we don't parse it atm */
	if (bios->data[tmdstableptr] != 0x11)
		return -ENOSYS;

	/*
	 * These two scripts are odd: they don't seem to get run even when
	 * they are not stubbed.
	 */
	script1 = ROM16(bios->data[tmdstableptr + 7]);
	script2 = ROM16(bios->data[tmdstableptr + 9]);
	if (bios->data[script1] != 'q' || bios->data[script2] != 'q')
		NV_WARN(drm, "TMDS table script pointers not stubbed\n");

	bios->tmds.output0_script_ptr = ROM16(bios->data[tmdstableptr + 11]);
	bios->tmds.output1_script_ptr = ROM16(bios->data[tmdstableptr + 13]);

	return 0;
}

struct bit_table {
	const char id;
	int (* const parse_fn)(struct drm_device *, struct nvbios *, struct bit_entry *);
};

#define BIT_TABLE(id, funcid) ((struct bit_table){ id, parse_bit_##funcid##_tbl_entry })

int
bit_table(struct drm_device *dev, u8 id, struct bit_entry *bit)
{
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct nvbios *bios = &drm->vbios;
	u8 entries, *entry;

	if (bios->type != NVBIOS_BIT)
		return -ENODEV;

	entries = bios->data[bios->offset + 10];
	entry   = &bios->data[bios->offset + 12];
	while (entries--) {
		if (entry[0] == id) {
			bit->id = entry[0];
			bit->version = entry[1];
			bit->length = ROM16(entry[2]);
			bit->offset = ROM16(entry[4]);
			bit->data = ROMPTR(dev, entry[4]);
			return 0;
		}

		entry += bios->data[bios->offset + 9];
	}

	return -ENOENT;
}

static int
parse_bit_table(struct nvbios *bios, const uint16_t bitoffset,
		struct bit_table *table)
{
	struct drm_device *dev = bios->dev;
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct bit_entry bitentry;

	if (bit_table(dev, table->id, &bitentry) == 0)
		return table->parse_fn(dev, bios, &bitentry);

	NV_INFO(drm, "BIT table '%c' not found\n", table->id);
	return -ENOSYS;
}

static int
parse_bit_structure(struct nvbios *bios, const uint16_t bitoffset)
{
	int ret;

	/*
	 * The only restriction on parsing order currently is having 'i' first
	 * for use of bios->*_version or bios->feature_byte while parsing;
	 * functions shouldn't be actually *doing* anything apart from pulling
	 * data from the image into the bios struct, thus no interdependencies
	 */
	ret = parse_bit_table(bios, bitoffset, &BIT_TABLE('i', i));
	if (ret) /* info? */
		return ret;
	if (bios->major_version >= 0x60) /* g80+ */
		parse_bit_table(bios, bitoffset, &BIT_TABLE('A', A));
	parse_bit_table(bios, bitoffset, &BIT_TABLE('D', display));
	ret = parse_bit_table(bios, bitoffset, &BIT_TABLE('I', init));
	if (ret)
		return ret;
	parse_bit_table(bios, bitoffset, &BIT_TABLE('M', M)); /* memory? */
	parse_bit_table(bios, bitoffset, &BIT_TABLE('L', lvds));
	parse_bit_table(bios, bitoffset, &BIT_TABLE('T', tmds));

	return 0;
}

static int parse_bmp_structure(struct drm_device *dev, struct nvbios *bios, unsigned int offset)
{
	/*
	 * Parses the BMP structure for useful things, but does not act on them
	 *
	 * offset +   5: BMP major version
	 * offset +   6: BMP minor version
	 * offset +   9: BMP feature byte
	 * offset +  10: BCD encoded BIOS version
	 *
	 * offset +  18: init script table pointer (for bios versions < 5.10h)
	 * offset +  20: extra init script table pointer (for bios
	 * versions < 5.10h)
	 *
	 * offset +  24: memory init table pointer (used on early bios versions)
	 * offset +  26: SDR memory sequencing setup data table
	 * offset +  28: DDR memory sequencing setup data table
	 *
	 * offset +  54: index of I2C CRTC pair to use for CRT output
	 * offset +  55: index of I2C CRTC pair to use for TV output
	 * offset +  56: index of I2C CRTC pair to use for flat panel output
	 * offset +  58: write CRTC index for I2C pair 0
	 * offset +  59: read CRTC index for I2C pair 0
	 * offset +  60: write CRTC index for I2C pair 1
	 * offset +  61: read CRTC index for I2C pair 1
	 *
	 * offset +  67: maximum internal PLL frequency (single stage PLL)
	 * offset +  71: minimum internal PLL frequency (single stage PLL)
	 *
	 * offset +  75: script table pointers, as described in
	 * parse_script_table_pointers
	 *
	 * offset +  89: TMDS single link output A table pointer
	 * offset +  91: TMDS single link output B table pointer
	 * offset +  95: LVDS single link output A table pointer
	 * offset + 105: flat panel timings table pointer
	 * offset + 107: flat panel strapping translation table pointer
	 * offset + 117: LVDS manufacturer panel config table pointer
	 * offset + 119: LVDS manufacturer strapping translation table pointer
	 *
	 * offset + 142: PLL limits table pointer
	 *
	 * offset + 156: minimum pixel clock for LVDS dual link
	 */

	struct nouveau_drm *drm = nouveau_drm(dev);
	uint8_t *bmp = &bios->data[offset], bmp_version_major, bmp_version_minor;
	uint16_t bmplength;
	uint16_t legacy_scripts_offset, legacy_i2c_offset;

	/* load needed defaults in case we can't parse this info */
	bios->digital_min_front_porch = 0x4b;
	bios->fmaxvco = 256000;
	bios->fminvco = 128000;
	bios->fp.duallink_transition_clk = 90000;

	bmp_version_major = bmp[5];
	bmp_version_minor = bmp[6];

	NV_INFO(drm, "BMP version %d.%d\n",
		 bmp_version_major, bmp_version_minor);

	/*
	 * Make sure that 0x36 is blank and can't be mistaken for a DCB
	 * pointer on early versions
	 */
	if (bmp_version_major < 5)
		*(uint16_t *)&bios->data[0x36] = 0;

	/*
	 * Seems that the minor version was 1 for all major versions prior
	 * to 5. Version 6 could theoretically exist, but I suspect BIT
	 * happened instead.
	 */
	if ((bmp_version_major < 5 && bmp_version_minor != 1) || bmp_version_major > 5) {
		NV_ERROR(drm, "You have an unsupported BMP version. "
				"Please send in your bios\n");
		return -ENOSYS;
	}

	if (bmp_version_major == 0)
		/* nothing that's currently useful in this version */
		return 0;
	else if (bmp_version_major == 1)
		bmplength = 44; /* exact for 1.01 */
	else if (bmp_version_major == 2)
		bmplength = 48; /* exact for 2.01 */
	else if (bmp_version_major == 3)
		bmplength = 54;
		/* guessed - mem init tables added in this version */
	else if (bmp_version_major == 4 || bmp_version_minor < 0x1)
		/* don't know if 5.0 exists... */
		bmplength = 62;
		/* guessed - BMP I2C indices added in version 4*/
	else if (bmp_version_minor < 0x6)
		bmplength = 67; /* exact for 5.01 */
	else if (bmp_version_minor < 0x10)
		bmplength = 75; /* exact for 5.06 */
	else if (bmp_version_minor == 0x10)
		bmplength = 89; /* exact for 5.10h */
	else if (bmp_version_minor < 0x14)
		bmplength = 118; /* exact for 5.11h */
	else if (bmp_version_minor < 0x24)
		/*
		 * Not sure of version where pll limits came in;
		 * certainly exist by 0x24 though.
		 */
		/* length not exact: this is long enough to get lvds members */
		bmplength = 123;
	else if (bmp_version_minor < 0x27)
		/*
		 * Length not exact: this is long enough to get pll limit
		 * member
		 */
		bmplength = 144;
	else
		/*
		 * Length not exact: this is long enough to get dual link
		 * transition clock.
		 */
		bmplength = 158;

	/* checksum */
	if (nv_cksum(bmp, 8)) {
		NV_ERROR(drm, "Bad BMP checksum\n");
		return -EINVAL;
	}

	/*
	 * Bit 4 seems to indicate either a mobile bios or a quadro card --
	 * mobile behaviour consistent (nv11+), quadro only seen nv18gl-nv36gl
	 * (not nv10gl), bit 5 that the flat panel tables are present, and
	 * bit 6 a tv bios.
	 */
	bios->feature_byte = bmp[9];

	if (bmp_version_major < 5 || bmp_version_minor < 0x10)
		bios->old_style_init = true;
	legacy_scripts_offset = 18;
	if (bmp_version_major < 2)
		legacy_scripts_offset -= 4;
	bios->init_script_tbls_ptr = ROM16(bmp[legacy_scripts_offset]);
	bios->extra_init_script_tbl_ptr = ROM16(bmp[legacy_scripts_offset + 2]);

	if (bmp_version_major > 2) {	/* appears in BMP 3 */
		bios->legacy.mem_init_tbl_ptr = ROM16(bmp[24]);
		bios->legacy.sdr_seq_tbl_ptr = ROM16(bmp[26]);
		bios->legacy.ddr_seq_tbl_ptr = ROM16(bmp[28]);
	}

	legacy_i2c_offset = 0x48;	/* BMP version 2 & 3 */
	if (bmplength > 61)
		legacy_i2c_offset = offset + 54;
	bios->legacy.i2c_indices.crt = bios->data[legacy_i2c_offset];
	bios->legacy.i2c_indices.tv = bios->data[legacy_i2c_offset + 1];
	bios->legacy.i2c_indices.panel = bios->data[legacy_i2c_offset + 2];

	if (bmplength > 74) {
		bios->fmaxvco = ROM32(bmp[67]);
		bios->fminvco = ROM32(bmp[71]);
	}
	if (bmplength > 88)
		parse_script_table_pointers(bios, offset + 75);
	if (bmplength > 94) {
		bios->tmds.output0_script_ptr = ROM16(bmp[89]);
		bios->tmds.output1_script_ptr = ROM16(bmp[91]);
		/*
		 * Never observed in use with lvds scripts, but is reused for
		 * 18/24 bit panel interface default for EDID equipped panels
		 * (if_is_24bit not set directly to avoid any oscillation).
		 */
		bios->legacy.lvds_single_a_script_ptr = ROM16(bmp[95]);
	}
	if (bmplength > 108) {
		bios->fp.fptablepointer = ROM16(bmp[105]);
		bios->fp.fpxlatetableptr = ROM16(bmp[107]);
		bios->fp.xlatwidth = 1;
	}
	if (bmplength > 120) {
		bios->fp.lvdsmanufacturerpointer = ROM16(bmp[117]);
		bios->fp.fpxlatemanufacturertableptr = ROM16(bmp[119]);
	}
#if 0
	if (bmplength > 143)
		bios->pll_limit_tbl_ptr = ROM16(bmp[142]);
#endif

	if (bmplength > 157)
		bios->fp.duallink_transition_clk = ROM16(bmp[156]) * 10;

	return 0;
}

static uint16_t findstr(uint8_t *data, int n, const uint8_t *str, int len)
{
	int i, j;

	for (i = 0; i <= (n - len); i++) {
		for (j = 0; j < len; j++)
			if (data[i + j] != str[j])
				break;
		if (j == len)
			return i;
	}

	return 0;
}

void *
olddcb_table(struct drm_device *dev)
{
	struct nouveau_drm *drm = nouveau_drm(dev);
	u8 *dcb = NULL;

	if (drm->client.device.info.family > NV_DEVICE_INFO_V0_TNT)
		dcb = ROMPTR(dev, drm->vbios.data[0x36]);
	if (!dcb) {
		NV_WARN(drm, "No DCB data found in VBIOS\n");
		return NULL;
	}

	if (dcb[0] >= 0x42) {
		NV_WARN(drm, "DCB version 0x%02x unknown\n", dcb[0]);
		return NULL;
	} else
	if (dcb[0] >= 0x30) {
		if (ROM32(dcb[6]) == 0x4edcbdcb)
			return dcb;
	} else
	if (dcb[0] >= 0x20) {
		if (ROM32(dcb[4]) == 0x4edcbdcb)
			return dcb;
	} else
	if (dcb[0] >= 0x15) {
		if (!memcmp(&dcb[-7], "DEV_REC", 7))
			return dcb;
	} else {
		/*
		 * v1.4 (some NV15/16, NV11+) seems the same as v1.5, but
		 * always has the same single (crt) entry, even when tv-out
		 * present, so the conclusion is this version cannot really
		 * be used.
		 *
		 * v1.2 tables (some NV6/10, and NV15+) normally have the
		 * same 5 entries, which are not specific to the card and so
		 * no use.
		 *
		 * v1.2 does have an I2C table that read_dcb_i2c_table can
		 * handle, but cards exist (nv11 in #14821) with a bad i2c
		 * table pointer, so use the indices parsed in
		 * parse_bmp_structure.
		 *
		 * v1.1 (NV5+, maybe some NV4) is entirely unhelpful
		 */
		NV_WARN(drm, "No useful DCB data in VBIOS\n");
		return NULL;
	}

	NV_WARN(drm, "DCB header validation failed\n");
	return NULL;
}

void *
olddcb_outp(struct drm_device *dev, u8 idx)
{
	u8 *dcb = olddcb_table(dev);
	if (dcb && dcb[0] >= 0x30) {
		if (idx < dcb[2])
			return dcb + dcb[1] + (idx * dcb[3]);
	} else
	if (dcb && dcb[0] >= 0x20) {
		u8 *i2c = ROMPTR(dev, dcb[2]);
		u8 *ent = dcb + 8 + (idx * 8);
		if (i2c && ent < i2c)
			return ent;
	} else
	if (dcb && dcb[0] >= 0x15) {
		u8 *i2c = ROMPTR(dev, dcb[2]);
		u8 *ent = dcb + 4 + (idx * 10);
		if (i2c && ent < i2c)
			return ent;
	}

	return NULL;
}

int
olddcb_outp_foreach(struct drm_device *dev, void *data,
		 int (*exec)(struct drm_device *, void *, int idx, u8 *outp))
{
	int ret, idx = -1;
	u8 *outp = NULL;
	while ((outp = olddcb_outp(dev, ++idx))) {
		if (ROM32(outp[0]) == 0x00000000)
			break; /* seen on an NV11 with DCB v1.5 */
		if (ROM32(outp[0]) == 0xffffffff)
			break; /* seen on an NV17 with DCB v2.0 */

		if ((outp[0] & 0x0f) == DCB_OUTPUT_UNUSED)
			continue;
		if ((outp[0] & 0x0f) == DCB_OUTPUT_EOL)
			break;

		ret = exec(dev, data, idx, outp);
		if (ret)
			return ret;
	}

	return 0;
}

u8 *
olddcb_conntab(struct drm_device *dev)
{
	u8 *dcb = olddcb_table(dev);
	if (dcb && dcb[0] >= 0x30 && dcb[1] >= 0x16) {
		u8 *conntab = ROMPTR(dev, dcb[0x14]);
		if (conntab && conntab[0] >= 0x30 && conntab[0] <= 0x40)
			return conntab;
	}
	return NULL;
}

u8 *
olddcb_conn(struct drm_device *dev, u8 idx)
{
	u8 *conntab = olddcb_conntab(dev);
	if (conntab && idx < conntab[2])
		return conntab + conntab[1] + (idx * conntab[3]);
	return NULL;
}

static struct dcb_output *new_dcb_entry(struct dcb_table *dcb)
{
	struct dcb_output *entry = &dcb->entry[dcb->entries];

	memset(entry, 0, sizeof(struct dcb_output));
	entry->index = dcb->entries++;

	return entry;
}

static void fabricate_dcb_output(struct dcb_table *dcb, int type, int i2c,
				 int heads, int or)
{
	struct dcb_output *entry = new_dcb_entry(dcb);

	entry->type = type;
	entry->i2c_index = i2c;
	entry->heads = heads;
	if (type != DCB_OUTPUT_ANALOG)
		entry->location = !DCB_LOC_ON_CHIP; /* ie OFF CHIP */
	entry->or = or;
}

static bool
parse_dcb20_entry(struct drm_device *dev, struct dcb_table *dcb,
		  uint32_t conn, uint32_t conf, struct dcb_output *entry)
{
	struct nouveau_drm *drm = nouveau_drm(dev);
	int link = 0;

	entry->type = conn & 0xf;
	entry->i2c_index = (conn >> 4) & 0xf;
	entry->heads = (conn >> 8) & 0xf;
	entry->connector = (conn >> 12) & 0xf;
	entry->bus = (conn >> 16) & 0xf;
	entry->location = (conn >> 20) & 0x3;
	entry->or = (conn >> 24) & 0xf;

	switch (entry->type) {
	case DCB_OUTPUT_ANALOG:
		/*
		 * Although the rest of a CRT conf dword is usually
		 * zeros, mac biosen have stuff there so we must mask
		 */
		entry->crtconf.maxfreq = (dcb->version < 0x30) ?
					 (conf & 0xffff) * 10 :
					 (conf & 0xff) * 10000;
		break;
	case DCB_OUTPUT_LVDS:
		{
		uint32_t mask;
		if (conf & 0x1)
			entry->lvdsconf.use_straps_for_mode = true;
		if (dcb->version < 0x22) {
			mask = ~0xd;
			/*
			 * The laptop in bug 14567 lies and claims to not use
			 * straps when it does, so assume all DCB 2.0 laptops
			 * use straps, until a broken EDID using one is produced
			 */
			entry->lvdsconf.use_straps_for_mode = true;
			/*
			 * Both 0x4 and 0x8 show up in v2.0 tables; assume they
			 * mean the same thing (probably wrong, but might work)
			 */
			if (conf & 0x4 || conf & 0x8)
				entry->lvdsconf.use_power_scripts = true;
		} else {
			mask = ~0x7;
			if (conf & 0x2)
				entry->lvdsconf.use_acpi_for_edid = true;
			if (conf & 0x4)
				entry->lvdsconf.use_power_scripts = true;
			entry->lvdsconf.sor.link = (conf & 0x00000030) >> 4;
			link = entry->lvdsconf.sor.link;
		}
		if (conf & mask) {
			/*
			 * Until we even try to use these on G8x, it's
			 * useless reporting unknown bits.  They all are.
			 */
			if (dcb->version >= 0x40)
				break;

			NV_ERROR(drm, "Unknown LVDS configuration bits, "
				      "please report\n");
		}
		break;
		}
	case DCB_OUTPUT_TV:
	{
		if (dcb->version >= 0x30)
			entry->tvconf.has_component_output = conf & (0x8 << 4);
		else
			entry->tvconf.has_component_output = false;

		break;
	}
	case DCB_OUTPUT_DP:
		entry->dpconf.sor.link = (conf & 0x00000030) >> 4;
		entry->extdev = (conf & 0x0000ff00) >> 8;
		switch ((conf & 0x00e00000) >> 21) {
		case 0:
			entry->dpconf.link_bw = 162000;
			break;
		case 1:
			entry->dpconf.link_bw = 270000;
			break;
		default:
			entry->dpconf.link_bw = 540000;
			break;
		}
		switch ((conf & 0x0f000000) >> 24) {
		case 0xf:
		case 0x4:
			entry->dpconf.link_nr = 4;
			break;
		case 0x3:
		case 0x2:
			entry->dpconf.link_nr = 2;
			break;
		default:
			entry->dpconf.link_nr = 1;
			break;
		}
		link = entry->dpconf.sor.link;
		break;
	case DCB_OUTPUT_TMDS:
		if (dcb->version >= 0x40) {
			entry->tmdsconf.sor.link = (conf & 0x00000030) >> 4;
			entry->extdev = (conf & 0x0000ff00) >> 8;
			link = entry->tmdsconf.sor.link;
		}
		else if (dcb->version >= 0x30)
			entry->tmdsconf.slave_addr = (conf & 0x00000700) >> 8;
		else if (dcb->version >= 0x22)
			entry->tmdsconf.slave_addr = (conf & 0x00000070) >> 4;
		break;
	case DCB_OUTPUT_EOL:
		/* weird g80 mobile type that "nv" treats as a terminator */
		dcb->entries--;
		return false;
	default:
		break;
	}

	if (dcb->version < 0x40) {
		/* Normal entries consist of a single bit, but dual link has
		 * the next most significant bit set too
		 */
		entry->duallink_possible =
			((1 << (ffs(entry->or) - 1)) * 3 == entry->or);
	} else {
		entry->duallink_possible = (entry->sorconf.link == 3);
	}

	/* unsure what DCB version introduces this, 3.0? */
	if (conf & 0x100000)
		entry->i2c_upper_default = true;

	entry->hasht = (entry->extdev << 8) | (entry->location << 4) |
			entry->type;
	entry->hashm = (entry->heads << 8) | (link << 6) | entry->or;
	return true;
}

static bool
parse_dcb15_entry(struct drm_device *dev, struct dcb_table *dcb,
		  uint32_t conn, uint32_t conf, struct dcb_output *entry)
{
	struct nouveau_drm *drm = nouveau_drm(dev);

	switch (conn & 0x0000000f) {
	case 0:
		entry->type = DCB_OUTPUT_ANALOG;
		break;
	case 1:
		entry->type = DCB_OUTPUT_TV;
		break;
	case 2:
	case 4:
		if (conn & 0x10)
			entry->type = DCB_OUTPUT_LVDS;
		else
			entry->type = DCB_OUTPUT_TMDS;
		break;
	case 3:
		entry->type = DCB_OUTPUT_LVDS;
		break;
	default:
		NV_ERROR(drm, "Unknown DCB type %d\n", conn & 0x0000000f);
		return false;
	}

	entry->i2c_index = (conn & 0x0003c000) >> 14;
	entry->heads = ((conn & 0x001c0000) >> 18) + 1;
	entry->or = entry->heads; /* same as heads, hopefully safe enough */
	entry->location = (conn & 0x01e00000) >> 21;
	entry->bus = (conn & 0x0e000000) >> 25;
	entry->duallink_possible = false;

	switch (entry->type) {
	case DCB_OUTPUT_ANALOG:
		entry->crtconf.maxfreq = (conf & 0xffff) * 10;
		break;
	case DCB_OUTPUT_TV:
		entry->tvconf.has_component_output = false;
		break;
	case DCB_OUTPUT_LVDS:
		if ((conn & 0x00003f00) >> 8 != 0x10)
			entry->lvdsconf.use_straps_for_mode = true;
		entry->lvdsconf.use_power_scripts = true;
		break;
	default:
		break;
	}

	return true;
}

static
void merge_like_dcb_entries(struct drm_device *dev, struct dcb_table *dcb)
{
	/*
	 * DCB v2.0 lists each output combination separately.
	 * Here we merge compatible entries to have fewer outputs, with
	 * more options
	 */

	struct nouveau_drm *drm = nouveau_drm(dev);
	int i, newentries = 0;

	for (i = 0; i < dcb->entries; i++) {
		struct dcb_output *ient = &dcb->entry[i];
		int j;

		for (j = i + 1; j < dcb->entries; j++) {
			struct dcb_output *jent = &dcb->entry[j];

			if (jent->type == 100) /* already merged entry */
				continue;

			/* merge heads field when all other fields the same */
			if (jent->i2c_index == ient->i2c_index &&
			    jent->type == ient->type &&
			    jent->location == ient->location &&
			    jent->or == ient->or) {
				NV_INFO(drm, "Merging DCB entries %d and %d\n",
					 i, j);
				ient->heads |= jent->heads;
				jent->type = 100; /* dummy value */
			}
		}
	}

	/* Compact entries merged into others out of dcb */
	for (i = 0; i < dcb->entries; i++) {
		if (dcb->entry[i].type == 100)
			continue;

		if (newentries != i) {
			dcb->entry[newentries] = dcb->entry[i];
			dcb->entry[newentries].index = newentries;
		}
		newentries++;
	}

	dcb->entries = newentries;
}

static bool
apply_dcb_encoder_quirks(struct drm_device *dev, int idx, u32 *conn, u32 *conf)
{
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct dcb_table *dcb = &drm->vbios.dcb;

	/* Dell Precision M6300
	 *   DCB entry 2: 02025312 00000010
	 *   DCB entry 3: 02026312 00000020
	 *
	 * Identical, except apparently a different connector on a
	 * different SOR link.  Not a clue how we're supposed to know
	 * which one is in use if it even shares an i2c line...
	 *
	 * Ignore the connector on the second SOR link to prevent
	 * nasty problems until this is sorted (assuming it's not a
	 * VBIOS bug).
	 */
	if (nv_match_device(dev, 0x040d, 0x1028, 0x019b)) {
		if (*conn == 0x02026312 && *conf == 0x00000020)
			return false;
	}

	/* GeForce3 Ti 200
	 *
	 * DCB reports an LVDS output that should be TMDS:
	 *   DCB entry 1: f2005014 ffffffff
	 */
	if (nv_match_device(dev, 0x0201, 0x1462, 0x8851)) {
		if (*conn == 0xf2005014 && *conf == 0xffffffff) {
			fabricate_dcb_output(dcb, DCB_OUTPUT_TMDS, 1, 1, 1);
			return false;
		}
	}

	/* XFX GT-240X-YA
	 *
	 * So many things wrong here, replace the entire encoder table..
	 */
	if (nv_match_device(dev, 0x0ca3, 0x1682, 0x3003)) {
		if (idx == 0) {
			*conn = 0x02001300; /* VGA, connector 1 */
			*conf = 0x00000028;
		} else
		if (idx == 1) {
			*conn = 0x01010312; /* DVI, connector 0 */
			*conf = 0x00020030;
		} else
		if (idx == 2) {
			*conn = 0x01010310; /* VGA, connector 0 */
			*conf = 0x00000028;
		} else
		if (idx == 3) {
			*conn = 0x02022362; /* HDMI, connector 2 */
			*conf = 0x00020010;
		} else {
			*conn = 0x0000000e; /* EOL */
			*conf = 0x00000000;
		}
	}

	/* Some other twisted XFX board (rhbz#694914)
	 *
	 * The DVI/VGA encoder combo that's supposed to represent the
	 * DVI-I connector actually point at two different ones, and
	 * the HDMI connector ends up paired with the fferent oa= big , 1, 1, 1);
			return false;
		}
	}= n= 0x0000000e; /* EOL */
			*conf = 0x0*
	 * The DVI/VG			returANA4 	u8 0x16able_point EOL *onf.sor.link;
		}
		l(Ooruct 	if (&& dcb[ i, 	/* upposed tolelf(idx == 0) {
			*conn = 0x02001300; 615VGA, connect2605 */
			*conf = 0x00000028;
		} else
		if2(idx == 1) {
			*conn =EOL */
			*conf = 0x0connector 0 */
			*conf = 0x00020030;
		} else
		if (idx == 2) {
			*conn = 0x01010310; /* VGA, connector 0 */
			*conf = 0x00000028;
		} else
	4 co(idx == 3) {
			*conn = 0x02022362; /* HDMI, connector 2 */
			*conf = 0x00020010;
		} else {
	132conn = 0x0000000e; /*  0x01010312; /* DVI, 0000000;
		}
	}

	/* Some other twisted XFX board (rhbz#694914)
	 *
	 * The DVI/VGA enfdo#50830:00000e; /*  maybe su8 0x16abssu HDMI oes backw use nn == 0x02026312 && *conf == 0x002fffff384nnectc791 */
			*conf = 0x00T_TMDS, GeForce3 Tonnen LVDS;
		} else {
1 (idxry->type =*conf = 0x00T_TMDS, GeForce34
1 (1n LVDS;
		} else e 0x31dxry->type =*conf = 0x02T_TMDS, GeForce3 T1 (12 LVDS;
		} else {
0 (idxrm_device *dev, struct dcb_t2c,
	
lse;
		}
	}

	onf)
{
	ruct nouveau_drm *drm = nouveaP structure for use = type;
	entry- entry 2: 0202->fmaxv as vred wll_s; /* sam020h t_s; /*ion M ? 3 :thingp[14def _ue;
}
pc__GA enAn, * uMac G4ED)
		nn == 0xof026cNA4es->l */

	stru("P;
}
Mac4,5" */
			lse;
		}
	}

	/* XFX GT-240X-YA
	 *
	 * So t spll_s; /* things lse;
		}
	}

	/* XFX GT-240X-YA
	 *
	
	caseany tpll_s; /* t2 14;
	entryxrm_d= ROM16(b/* point	}
CB dasanefront_porc == 1se;
		}
	}

	/* XFX GT-240X-YA
	 *
	
	casearies %d  os->legacy.i2c_indices.tv = any thing == 0x0204_tv_/
	biofyn", table-s->legacy.i2c_indices.p)4;
		)			lse;
		}
	}

	/* XFX GT-240X-YA
	 *
	 Vntry(dc%d  os->legacy.i2c_indices.tvtvntry(dc%d  pll_s; /* t0 headss is longt1_script_ptr = ROM16(bmp[91]|6) |NV_ERROR(drm, "Pixel clock com)			lse;
		}
	}

	/* XFX GT-240X-YA
	 *
	 * Sotry(dc%d  os->legacy.i2c_indices.tvn).
	ntry(dc%d  pll_s; /* t1ruct nvbios *bio
		  uint3t conn, uint32_t conf, struct du8 *outp))
le ((outp = olddcb_ *dcb = &drm->vbios.dcb;

	/* Dell Precision M6300
	 *   DCB entry 2: 02025312 00000010
	 	 dcb#69491400000070) >> 4;
		brbrea CB v2.0 */
		if: CB v2.0 */

		ret dcb#69n = CB v2.0 */

		ret2 *c having  lonnn, u32 *conf)
{
	struct nouveautp &struct&tablej = i + 1; j < dcb->entriex = i2c;
	entry->heads = head	ient->heads |=201,0000)%02d:)%08x)%08x(connautp structtabley[i];
			0000070) >> 4;
		brbrgs wron et)
		rent32_t conn,] <= 0x4p structtabl + 9];
;
	}
	case DCron et)
		rent32_t conn,] <= 0x4p structtabl + 9];
;
	}
*
		 drm_device *dev1conn &does*_versiof ((ou/y problems un6)
		bmuency (oncv != TVy
		+  89ion <nk_posoes o use 0x000boguse pointex0231ms, so as23212 ,nk_posoARN(tau_drm * is ->versio
	}
	if (btry[1];
	&
			    j0X-YA
	 *
	 Ventrie%d  & 0x0e000000) >>  1:
		clktable = biosconn & 0x001c0000) >k_nr;	u8 *dcb = olddcb_t dcb_t2c,
	
ntryfoin_0000e; /* table segment for po = type;
	entry- entry 2: t0202->fmaxv as v= omap[1 vers{ }		struct dx))) {ions prce(ulength:s lonown br/* cha Den->crt00000e; /* 		   , work)
 1, 1, ble_poNV5+, maybe soes 		}
		oARN	return -l_mi foinMP mi/
			*confoAR+  89	 */
,e_st0caklcant biosed e 0x000b_CLK)t ftp)) (idx == 0) {!02026312 && *conit_entry = 0x03920020)
70002020a2 */
			l00)
			continue;

t		if (newentries != ;
			000t	dcb->entry0000e; /*"
					entryxrm DVI/VGA enn"DCB head0000e; /*  mfpt_pail versioe-l_mi t
	ifk* uppupstructtolelve
			ve stulece(de DV:eturANA4 	oonf & 0x4 |o prbuointeriignific
{
mi t
	be	oonf & 0x4 |0000e; /*			 ROM1t +  59: r0x00osedinkworociouveao prbuopt, wsthings, orcy->iuniqur
	 * TV00e; /*  maex(idx == l00)
			continue;

t		if (newentries !=  < c(i2c 000t	dcb->entryx001c0000
	}
*
		c(i2c= outp);ries].int	dcb->entry0000e; /*dcb-dxder_q
		}
	}

	/* *
		 *ap[c(i *
oldd*ap[c(i dcbon an;ies].int	dcb->entry0000e; /*dcb*ap[c(i d- DCB_ODVI/VGA en lonow simuveamblems ane con0000e; /*,9: Ttroyms until this 
ructure.
	-f 0x11h = 0x4bitso throARos |rae currentlmdst,ntries; (idx == 0) {i;

	tab[0] >= 0x30 && coonntab + conntait_entry return conntab;
] = dcb-cb_conn(14)
	 ;	u8 t nvbios *bio
		  uint3truct nouveau_drm *drm = nouveaP structure for use = type;
	enm->vbios.dcb;

	/* Dell Precision M6300
	 *   DCB entry 2: 0202->fmaxv as v= o 2: t,= 0x30		structan;irks(st6) {
		u8 *conntab = ROMPTRNULLts
		 * the poinot a-201,osed e if (btry[a[bios->off=et + 10];
MPturn false;
		}
	}

	onf)
{
	ruct n", table-
	return -ENOENT;
}

st	bios->data[tmdstableptr] >> 4, bioy->i2c_upper sure tha 000tNULL;32(bio00tNULLos->data[t0000070) >> 4=io00tNULdstexec)(struct drm_dev", ta *dc, 		  uint3t connr a DCB
	 * thus nobut 2.1+028, pointetry->iobutinteirintemjent->,_matc
	 * guaroAtee snt3>locat0000)ie parse_8: writ 0x1682 26: 	ve stom= parsidx == 0) {bug 14567 lies andion <dcb)
{
	/*
	 * DCB v2.0] <= 0x4	/*
	 * Tump TV00e; /*  entryt nouveau_dlo>,_0)  ROMthe in == 0))) {
		if= 0x0000#69n = cb_conntab(dseen on an NV11 with Dtab(NULL!=sk;
		i
	/* *
		onntab + conntab[1][3] k <<
				ient->heads |=y->i#69n %02d:)%04x= 100; /* autp 8_t *dtab(NUL)
	retu	case DC	ient->heads |=y->i#69n %02d:)%08x= 100; /* autp 8_tv2.tab(NUL)
	retDVI/V	ntryfoin_0000e; /* tble-
	rev, struct nvbios *bios, arse_nv17_hwsq_uf)
{try *bitentry)
{
	/*
	 * Parses the pointer to the Tn)
{
	inthwsq_an't paros, onn & 0x00'i' first
	(structry is a compar"HWSQ"?)
	 *
	 * o the coe option st nouve,*confoARt 0x1682 26urn w how to
	b, int gnificanst bit st masu_d for Iable thTC pairuallydefac handenci()
	 *
1304)ntry is veau_le thuf)
{  18:_drmfortnd SC paiti up ,*confstable pointhan4)
	 *
14DCB ens = 0;

	for (i = 0; i < dcb->entries; i++) {
		s the poinif_obj bmp*/*
	 * 2025312 (dev, drm->vbiobj bm;ion_minor; 18:__to_ for ;eded defaulhwsq_ int fo */
	bistructios->data[biosIOS\nhwsq_an't pn(stronn & /*
	 * Bit 4 seems tToodrm yt nouvea: 	HWhTC pairualnch of bitmp_versinterat's onn &hing that's currentt16_tlen]);18:__to_ for  os->fmaxvco =hwsq_an't pDAC load dete;18:__to_ for  !=s3n\n");
		return -EINVAentry->iHWhTC pairual682 26urn %d\n",
		bios->data[tmdstableptr] >> 4, bioLrsee poD)
		e;
}
RTC pair to microf)
{eptr + 1hwsq_ int fo */
	B_LOwsq_an't pDAC2DACelds th; 18:__to_ for ;e
	 * t pDTC pairuallydefac  == inif_wrx15)m->vb, )
	 *
1304p 8_tv2.->fmaxvco =hwsq_ int fo */
	L)
	re;18:__to_ for  et]);

	 *  for  uf)
{  == l00)
			continue 18:__to_ for ;b->e= <<
		inif_wrx15)m->vb, )
	 *
14DCDACip 8_tv2.->fmaxvco =hwsq_ int fo */
	DACi drm_d	/*
	 * rpoioinoptrPBUS_DEBUG_4  == inif_wrx15)m->vb, ptrPBUS_DEBUG_4,oinif_rdx15)m->vb, ptrPBUS_DEBUG_4uct 20)8e *dev, struct nvbios *bios, arse_nv17_hw_TC pairua_uf)
{itentry)
{
	/*
	 * Parse0; /* P structure for use = tyard --
	MP0b_ffero use, */
	rD)
	,-l_mi a microf)
{ arsea comt000re_a_sc
	 * TV0efac e thGPIO etcdev);
	uinn).
		 *how to
ic ib_ffero use1 = ROM16do_matchgle_a_scr: 	ve s offset +  

	struct nou microf)
{ t nouveaa 5: BWARNu_le th"HWSQ"?)
	 *
	 *dstableptclen); i++) {
hwsq_)
	 *
	 *[vers{ 'H', 'W', 'S', 'Q' }		sclen);os, szersurn enthwsq_)
	 *
	 *
		strucOwsq_an't p + 1hwsq_o */
	B_L i, j;

->fmaxvco table-s->l* 10,
hwsq_)
	 *
	 *es z= ROMPTRNhwsq_o */
	L;
}

statiions prcsent, susx1682 26entry->

statiarse_nv17_hwsq_uf)
{try *bi", table->iOwsq_an't pDACsz t0 heb = i++) {
	entries;ble-_thish = ripts(dev);
	u8 *dcb = NULL;

	if (drm->client.device.info.family > NV_DEVICE_s->type != NVBIOS_BIT)
		return -ENOclen); i++) {
ipts_)
	[vers{0; /)
	 ,sk;
	,sk;
	,sk;
	,sk;
	,sk;
	,sk;
	,sk;DCD};eded defaulacy_i2c_of;eded defaulnewo */
	bistrucsearchr != 0ptrP8_t_SIZEios->data[biosfp.iptsL;
}

statia[biosfp.iptsios-= 0x000searchr !\n");
newo */
	B_L i, j;

6_t bmplength;
	uint1searchr !e0; /* etu	pts_)
	,cb[0] >= 0x!newo */
	_device *devp[0]) =	an't pDA=lnewo */
	bis 0) {!020-EINVA6_t bmplength;
	uint1or_m1_LEN)turn ret;
	}

	searchr !=-i2c_offs) =	an't pder_quirkptr] >> 4, bioFBWARNor_moios-failed\n"
}

statia[biosfp.ipts t16_t bmplength;
	uin int idx, u32 *c0ptIoff+ 10](dev);
	u8 *dcb = NULL;

	if (drm->client.device.info.family > NV_DEVICE_s->type !km_= NVBIOS_BIT)nvxx_= NV(25312 (dev, drm->vbICE_s->type != NVBIgacy.iIT)
		return -ENb->entriegacy.i
	return entry;
}

 != NV)
	respin_, 8)ROM16(&gacy.ie0000k
	regacy.ie0k;
		}ry biregacy.ie0kata[bi_t bmpleng	regacy.ie0R(drm, "B_t bmpurn 	regacy.ie0et, &BIT_TABLE"B_t bmp}

	if (et, &	regacy.ie0v !=BIT_TABLE"B_t bmp}

	if (v !=;s->data[bios
stao */
	_n");
gacy.ie0rged in + 10];
	e;);
gacy.ie0o */
	B_La[bios
stao */
	",
		bios->!)
{
	int ret;

	/*
	gacy.i
	gacy.ie0o */
	B+ 6dcb[2]);
		u8 *ea[bios
mpao */
	_n");
gacy.ie0rged in + 10];
MP;);
gacy.ie0o */
	B_La[bios
mpao */
	",
		bios->!)
{
	ind int offset)", tagacy.i
	gacy.ie0o */
	rm_device *dev,re, rep int (*family >run_!= NVROM16(dev);
	u8 *dcb = NULL;

	if (drm->client.device.info.family > NV_DEVICE_s->type != NVBIOS_BIT)
		return -ENOdev, ++) {ions prcReor) -	 *  {
	(strOM160.bmp_versionidx,ev = c(strO=nt = bios->doffset, &BIT_TABLEe mi = offseel taif (barse_nv17_hw_TC pairua_uf)
{i", table-
	r bios->doffs 0;
ute>fp.fpxlatemanufasmp_versioinvocc_of;edpxlatemanufactROM16(s->d=t that shouldce *dev)
{
	nt idx, u32 *coentries;ble-_poat's(dev);
	u8 *dcb = NULL;

	if (drm->client.device.info.family > NV_DEVICE_I things,htotal	dcb = ROMPTR(dev, drm->vbios.data[0x36]=);
	if (!dcb) {
		NVESLAL;
}

stati 4) |
		htotal  in +RstrVgaC= cnf == 0300; 6ICE_htotal |=, "NRstrVgaC= cnf == 0300; 7)ntry->d)rse_dCE_htotal |=, "NRstrVgaC= cnf == 0300; 7)ntry-rbrenk <CE_htotal |=, "NRstrVgaC= cnf == 0300;25)ntry->d)rse_1dxryhtotal |=, "NRstrVgaC= cnf == 0300;41)ntry->d)rse_11	rev, stru(htotal !=sk)ep int (*family >= NVROM16(dev);
	u8 *dcb = NULL;

	if (drm->client.device.info.family > NV_DEVICE_s->type != NVBIOS_BIT)
		return -ENOdev, ++ons prc * for lev(entev);PCI dcb = e nn == 0x!dcb->pLL;

roup_count = bios-!ptIoff+ 10](LL;
)that's current>data[bron et)
		rent3	ruct n", table-
	reLE('M', M)); /* memory? bios-!>doffset, &BIT_TABL)cb->eneturn -s->dcb[0] >= LK)t f; /* exact for GA en offset +  2 0;
ut] >=disuct dbmp_version 0;
utery->crtconf. enversunrsionat reisrn -POSTmi an all o == 0) {!0ntries;ble-_poat's(LL;
)n");
		r] >> 4, bioAdd
		r028,  offialiffeUT_TV:
"s->na co)) {
	== 4 || bmp.%d\n",
	ersion 0;
utery-d merge}a[bron etfamily >run_!= NVROM16(ab = ROMPTRM', M)); /* memory? b enfminor < 0x10oos->le
			o can'u,  offcsent, sor)s CR4B
	 * These two et, &BIT_TABLEe mi,
	ersionis_
		retuin +RstrVgaC= cnf == 030		rCIO_CRE_4B)ntry-4ions prcsel
ic isyat'ms-l_mi p_f_m_ntev);minvco = 128000;
	bios->	 * These two is_
		retupointers et, &BIT_TABLE('A5 M)); / et)
		refp;
	de	ruct n", table-
	rs prcselow subSC paitre not stM16 0;
utermp_version 0;
utery- 4) |
		t n