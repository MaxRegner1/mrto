
/*
 *  linux/drivers/scsi/esas2r/esas2r_flash.c
 *      For use with ATTO ExpressSAS R6xx SAS/SATA RAID controllers
 *
 *  Copyright (c) 2001-2013 ATTO Technology, Inc.
 *  (mailto:linuxdrivers@attotech.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * NO WARRANTY
 * THE PROGRAM IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED INCLUDING, WITHOUT
 * LIMITATION, ANY WARRANTIES OR CONDITIONS OF TITLE, NON-INFRINGEMENT,
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE. Each Recipient is
 * solely responsible for determining the appropriateness of using and
 * distributing the Program and assumes all risks associated with its
 * exercise of rights under this Agreement, including but not limited to
 * the risks and costs of program errors, damage to or loss of data,
 * programs or equipment, and unavailability or interruption of operations.
 *
 * DISCLAIMER OF LIABILITY
 * NEITHER RECIPIENT NOR ANY CONTRIBUTORS SHALL HAVE ANY LIABILITY FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING WITHOUT LIMITATION LOST PROFITS), HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OR DISTRIBUTION OF THE PROGRAM OR THE EXERCISE OF ANY RIGHTS GRANTED
 * HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#include "esas2r.h"

/* local macro defs */
#define esas2r_nvramcalc_cksum(n)     \
	(esas2r_calc_byte_cksum((u8 *)(n), sizeof(struct esas2r_sas_nvram), \
				SASNVR_CKSUM_SEED))
#define esas2r_nvramcalc_xor_cksum(n)  \
	(esas2r_calc_byte_xor_cksum((u8 *)(n), \
				    sizeof(struct esas2r_sas_nvram), 0))

#define ESAS2R_FS_DRVR_VER 2

static struct esas2r_sas_nvram default_sas_nvram = {
	{ 'E',	'S',  'A',  'S'			     }, /* signature          */
	SASNVR_VERSION,                                 /* version            */
	0,                                              /* checksum           */
	31,                                             /* max_lun_for_target */
	SASNVR_PCILAT_MAX,                              /* pci_latency        */
	SASNVR1_BOOT_DRVR,                              /* options1           */
	SASNVR2_HEARTBEAT   | SASNVR2_SINGLE_BUS        /* options2           */
	| SASNVR2_SW_MUX_CTRL,
	SASNVR_COAL_DIS,                                /* int_coalescing     */
	SASNVR_CMDTHR_NONE,                             /* cmd_throttle       */
	3,                                              /* dev_wait_time      */
	1,                                              /* dev_wait_count     */
	0,                                              /* spin_up_delay      */
	0,                                              /* ssp_align_rate     */
	{ 0x50, 0x01, 0x08, 0x60,                       /* sas_addr           */
	  0x00, 0x00, 0x00, 0x00 },
	{ SASNVR_SPEED_AUTO },                          /* phy_speed          */
	{ SASNVR_MUX_DISABLED },                        /* SAS multiplexing   */
	{ 0 },                                          /* phy_flags          */
	SASNVR_SORT_SAS_ADDR,                           /* sort_type          */
	3,                                              /* dpm_reqcmd_lmt     */
	3,                                              /* dpm_stndby_time    */
	0,                                              /* dpm_active_time    */
	{ 0 },                                          /* phy_target_id      */
	SASNVR_VSMH_DISABLED,                           /* virt_ses_mode      */
	SASNVR_RWM_DEFAULT,                             /* read_write_mode    */
	0,                                              /* link down timeout  */
	{ 0 }                                           /* reserved           */
};

static u8 cmd_to_fls_func[] = {
	0xFF,
	VDA_FLASH_READ,
	VDA_FLASH_BEGINW,
	VDA_FLASH_WRITE,
	VDA_FLASH_COMMIT,
	VDA_FLASH_CANCEL
};

static u8 esas2r_calc_byte_xor_cksum(u8 *addr, u32 len, u8 seed)
{
	u32 cksum = seed;
	u8 *p = (u8 *)&cksum;

	while (len) {
		if (((uintptr_t)addr & 3) == 0)
			break;

		cksum = cksum ^ *addr;
		addr++;
		len--;
	}
	while (len >= sizeof(u32)) {
		cksum = cksum ^ *(u32 *)addr;
		addr += 4;
		len -= 4;
	}
	while (len--) {
		cksum = cksum ^ *addr;
		addr++;
	}
	return p[0] ^ p[1] ^ p[2] ^ p[3];
}

static u8 esas2r_calc_byte_cksum(void *addr, u32 len, u8 seed)
{
	u8 *p = (u8 *)addr;
	u8 cksum = seed;

	while (len--)
		cksum = cksum + p[len];
	return cksum;
}

/* Interrupt callback to process FM API write requests. */
static void esas2r_fmapi_callback(struct esas2r_adapter *a,
				  struct esas2r_request *rq)
{
	struct atto_vda_flash_req *vrq = &rq->vrq->flash;
	struct esas2r_flash_context *fc =
		(struct esas2r_flash_context *)rq->interrupt_cx;

	if (rq->req_stat == RS_SUCCESS) {
		/* Last request was successful.  See what to do now. */
		switch (vrq->sub_func) {
		case VDA_FLASH_BEGINW:
			if (fc->sgc.cur_offset == NULL)
				goto commit;

			vrq->sub_func = VDA_FLASH_WRITE;
			rq->req_stat = RS_PENDING;
			break;

		case VDA_FLASH_WRITE:
commit:
			vrq->sub_func = VDA_FLASH_COMMIT;
			rq->req_stat = RS_PENDING;
			rq->interrupt_cb = fc->interrupt_cb;
			break;

		default:
			break;
		}
	}

	if (rq->req_stat != RS_PENDING)
		/*
		 * All done. call the real callback to complete the FM API
		 * request.  We should only get here if a BEGINW or WRITE
		 * operation failed.
		 */
		(*fc->interrupt_cb)(a, rq);
}

/*
 * Build a flash request based on the flash context.  The request status
 * is filled in on an error.
 */
static void build_flash_msg(struct esas2r_adapter *a,
			    struct esas2r_request *rq)
{
	struct esas2r_flash_context *fc =
		(struct esas2r_flash_context *)rq->interrupt_cx;
	struct esas2r_sg_context *sgc = &fc->sgc;
	u8 cksum = 0;

	/* calculate the checksum */
	if (fc->func == VDA_FLASH_BEGINW) {
		if (sgc->cur_offset)
			cksum = esas2r_calc_byte_xor_cksum(sgc->cur_offset,
							   sgc->length,
							   0);
		rq->interrupt_cb = esas2r_fmapi_callback;
	} else {
		rq->interrupt_cb = fc->interrupt_cb;
	}
	esas2r_build_flash_req(a,
			       rq,
			       fc->func,
			       cksum,
			       fc->flsh_addr,
			       sgc->length);

	esas2r_rq_free_sg_lists(rq, a);

	/*
	 * remember the length we asked for.  we have to keep track of
	 * the current amount done so we know how much to compare when
	 * doing the verification phase.
	 */
	fc->curr_len = fc->sgc.length;

	if (sgc->cur_offset) {
		/* setup the S/G context to build the S/G table  */
		esas2r_sgc_init(sgc, a, rq, &rq->vrq->flash.data.sge[0]);

		if (!esas2r_build_sg_list(a, rq, sgc)) {
			rq->req_stat = RS_BUSY;
			return;
		}
	} else {
		fc->sgc.length = 0;
	}

	/* update the flsh_addr to the next one to write to  */
	fc->flsh_addr += fc->curr_len;
}

/* determine the method to process the flash request */
static bool load_image(struct esas2r_adapter *a, struct esas2r_request *rq)
{
	/*
	 * assume we have more to do.  if we return with the status set to
	 * RS_PENDING, FM API tasks will continue.
	 */
	rq->req_stat = RS_PENDING;
	if (test_bit(AF_DEGRADED_MODE, &a->flags))
		/* not suppported for now */;
	else
		build_flash_msg(a, rq);

	return rq->req_stat == RS_PENDING;
}

/*  boot image fixer uppers called before downloading the image. */
static void fix_bios(struct esas2r_adapter *a, struct esas2r_flash_img *fi)
{
	struct esas2r_component_header *ch = &fi->cmp_hdr[CH_IT_BIOS];
	struct esas2r_pc_image *pi;
	struct esas2r_boot_header *bh;

	pi = (struct esas2r_pc_image *)((u8 *)fi + ch->image_offset);
	bh =
		(struct esas2r_boot_header *)((u8 *)pi +
					      le16_to_cpu(pi->header_offset));
	bh->device_id = cpu_to_le16(a->pcid->device);

	/* Recalculate the checksum in the PNP header if there  */
	if (pi->pnp_offset) {
		u8 *pnp_header_bytes =
			((u8 *)pi + le16_to_cpu(pi->pnp_offset));

		/* Identifier - dword that starts at byte 10 */
		*((u32 *)&pnp_header_bytes[10]) =
			cpu_to_le32(MAKEDWORD(a->pcid->subsystem_vendor,
					      a->pcid->subsystem_device));

		/* Checksum - byte 9 */
		pnp_header_bytes[9] -= esas2r_calc_byte_cksum(pnp_header_bytes,
							      32, 0);
	}

	/* Recalculate the checksum needed by the PC */
	pi->checksum = pi->checksum -
		       esas2r_calc_byte_cksum((u8 *)pi, ch->length, 0);
}

static void fix_efi(struct esas2r_adapter *a, struct esas2r_flash_img *fi)
{
	struct esas2r_component_header *ch = &fi->cmp_hdr[CH_IT_EFI];
	u32 len = ch->length;
	u32 offset = ch->image_offset;
	struct esas2r_efi_image *ei;
	struct esas2r_boot_header *bh;

	while (len) {
		u32 thislen;

		ei = (struct esas2r_efi_image *)((u8 *)fi + offset);
		bh = (struct esas2r_boot_header *)((u8 *)ei +
						   le16_to_cpu(
							   ei->header_offset));
		bh->device_id = cpu_to_le16(a->pcid->device);
		thislen = (u32)le16_to_cpu(bh->image_length) * 512;

		if (thislen > len)
			break;

		len -= thislen;
		offset += thislen;
	}
}

/* Complete a FM API request with the specified status. */
static bool complete_fmapi_req(struct esas2r_adapter *a,
			       struct esas2r_request *rq, u8 fi_stat)
{
	struct esas2r_flash_context *fc =
		(struct esas2r_flash_context *)rq->interrupt_cx;
	struct esas2r_flash_img *fi = fc->fi;

	fi->status = fi_stat;
	fi->driver_error = rq->req_stat;
	rq->interrupt_cb = NULL;
	rq->req_stat = RS_SUCCESS;

	if (fi_stat != FI_STAT_IMG_VER)
		memset(fc->scratch, 0, FM_BUF_SZ);

	esas2r_enable_heartbeat(a);
	clear_bit(AF_FLASH_LOCK, &a->flags);
	return false;
}

/* Process each phase of the flash download process. */
static void fw_download_proc(struct esas2r_adapter *a,
			     struct esas2r_request *rq)
{
	struct esas2r_flash_context *fc =
		(struct esas2r_flash_context *)rq->interrupt_cx;
	struct esas2r_flash_img *fi = fc->fi;
	struct esas2r_component_header *ch;
	u32 len;
	u8 *p, *q;

	/* If the previous operation failed, just return. */
	if (rq->req_stat != RS_SUCCESS)
		goto error;

	/*
	 * If an upload just completed and the compare length is non-zero,
	 * then we just read back part of the image we just wrote.  verify the
	 * section and continue reading until the entire image is verified.
	 */
	if (fc->func == VDA_FLASH_READ
	    && fc->cmp_len) {
		ch = &fi->cmp_hdr[fc->comp_typ];

		p = fc->scratch;
		q = (u8 *)fi                    /* start of the whole gob     */
		    + ch->image_offset          /* start of the current image */
		    + ch->length                /* end of the current image   */
		    - fc->cmp_len;              /* where we are now           */

		/*
		 * NOTE - curr_len is the exact count of bytes for the read
		 *        even when the end is read and its not a full buffer
		 */
		for (len = fc->curr_len; len; len--)
			if (*p++ != *q++)
				goto error;

		fc->cmp_len -= fc->curr_len; /* # left to compare    */

		/* Update fc and determine the length for the next upload */
		if (fc->cmp_len > FM_BUF_SZ)
			fc->sgc.length = FM_BUF_SZ;
		else
			fc->sgc.length = fc->cmp_len;

		fc->sgc.cur_offset = fc->sgc_offset +
				     ((u8 *)fc->scratch - (u8 *)fi);
	}

	/*
	 * This code uses a 'while' statement since the next component may
	 * have a length = zero.  This can happen since some components are
	 * not required.  At the end of this 'while' we set up the length
	 * for the next request and therefore sgc.length can be = 0.
	 */
	while (fc->sgc.length == 0) {
		ch = &fi->cmp_hdr[fc->comp_typ];

		switch (fc->task) {
		case FMTSK_ERASE_BOOT:
			/* the BIOS image is written next */
			ch = &fi->cmp_hdr[CH_IT_BIOS];
			if (ch->length == 0)
				goto no_bios;

			fc->task = FMTSK_WRTBIOS;
			fc->func = VDA_FLASH_BEGINW;
			fc->comp_typ = CH_IT_BIOS;
			fc->flsh_addr = FLS_OFFSET_BOOT;
			fc->sgc.length = ch->length;
			fc->sgc.cur_offset = fc->sgc_offset +
					     ch->image_offset;
			break;

		case FMTSK_WRTBIOS:
			/*
			 * The BIOS image has been written - read it and
			 * verify it
			 */
			fc->task = FMTSK_READBIOS;
			fc->func = VDA_FLASH_READ;
			fc->flsh_addr = FLS_OFFSET_BOOT;
			fc->cmp_len = ch->length;
			fc->sgc.length = FM_BUF_SZ;
			fc->sgc.cur_offset = fc->sgc_offset
					     + ((u8 *)fc->scratch -
						(u8 *)fi);
			break;

		case FMTSK_READBIOS:
no_bios:
			/*
			 * Mark the component header status for the image
			 * completed
			 */
			ch->status = CH_STAT_SUCCESS;

			/* The MAC image is written next */
			ch = &fi->cmp_hdr[CH_IT_MAC];
			if (ch->length == 0)
				goto no_mac;

			fc->task = FMTSK_WRTMAC;
			fc->func = VDA_FLASH_BEGINW;
			fc->comp_typ = CH_IT_MAC;
			fc->flsh_addr = FLS_OFFSET_BOOT
					+ fi->cmp_hdr[CH_IT_BIOS].length;
			fc->sgc.length = ch->length;
			fc->sgc.cur_offset = fc->sgc_offset +
					     ch->image_offset;
			break;

		case FMTSK_WRTMAC:
			/* The MAC image has been written - read and verify */
			fc->task = FMTSK_READMAC;
			fc->func = VDA_FLASH_READ;
			fc->flsh_addr -= ch->length;
			fc->cmp_len = ch->length;
			fc->sgc.length = FM_BUF_SZ;
			fc->sgc.cur_offset = fc->sgc_offset
					     + ((u8 *)fc->scratch -
						(u8 *)fi);
			break;

		case FMTSK_READMAC:
no_mac:
			/*
			 * Mark the component header status for the image
			 * completed
			 */
			ch->status = CH_STAT_SUCCESS;

			/* The EFI image is written next */
			ch = &fi->cmp_hdr[CH_IT_EFI];
			if (ch->length == 0)
				goto no_efi;

			fc->task = FMTSK_WRTEFI;
			fc->func = VDA_FLASH_BEGINW;
			fc->comp_typ = CH_IT_EFI;
			fc->flsh_addr = FLS_OFFSET_BOOT
					+ fi->cmp_hdr[CH_IT_BIOS].length
					+ fi->cmp_hdr[CH_IT_MAC].length;
			fc->sgc.length = ch->length;
			fc->sgc.cur_offset = fc->sgc_offset +
					     ch->image_offset;
			break;

		case FMTSK_WRTEFI:
			/* The EFI image has been written - read and verify */
			fc->task = FMTSK_READEFI;
			fc->func = VDA_FLASH_READ;
			fc->flsh_addr -= ch->length;
			fc->cmp_len = ch->length;
			fc->sgc.length = FM_BUF_SZ;
			fc->sgc.cur_offset = fc->sgc_offset
					     + ((u8 *)fc->scratch -
						(u8 *)fi);
			break;

		case FMTSK_READEFI:
no_efi:
			/*
			 * Mark the component header status for the image
			 * completed
			 */
			ch->status = CH_STAT_SUCCESS;

			/* The CFG image is written next */
			ch = &fi->cmp_hdr[CH_IT_CFG];

			if (ch->length == 0)
				goto no_cfg;
			fc->task = FMTSK_WRTCFG;
			fc->func = VDA_FLASH_BEGINW;
			fc->comp_typ = CH_IT_CFG;
			fc->flsh_addr = FLS_OFFSET_CPYR - ch->length;
			fc->sgc.length = ch->length;
			fc->sgc.cur_offset = fc->sgc_offset +
					     ch->image_offset;
			break;

		case FMTSK_WRTCFG:
			/* The CFG image has been written - read and verify */
			fc->task = FMTSK_READCFG;
			fc->func = VDA_FLASH_READ;
			fc->flsh_addr = FLS_OFFSET_CPYR - ch->length;
			fc->cmp_len = ch->length;
			fc->sgc.length = FM_BUF_SZ;
			fc->sgc.cur_offset = fc->sgc_offset
					     + ((u8 *)fc->scratch -
						(u8 *)fi);
			break;

		case FMTSK_READCFG:
no_cfg:
			/*
			 * Mark the component header status for the image
			 * completed
			 */
			ch->status = CH_STAT_SUCCESS;

			/*
			 * The download is complete.  If in degraded mode,
			 * attempt a chip reset.
			 */
			if (test_bit(AF_DEGRADED_MODE, &a->flags))
				esas2r_local_reset_adapter(a);

			a->flash_ver = fi->cmp_hdr[CH_IT_BIOS].version;
			esas2r_print_flash_rev(a);

			/* Update the type of boot image on the card */
			memcpy(a->image_type, fi->rel_version,
			       sizeof(fi->rel_version));
			complete_fmapi_req(a, rq, FI_STAT_SUCCESS);
			return;
		}

		/* If verifying, don't try reading more than what's there */
		if (fc->func == VDA_FLASH_READ
		    && fc->sgc.length > fc->cmp_len)
			fc->sgc.length = fc->cmp_len;
	}

	/* Build the request to perform the next action */
	if (!load_image(a, rq)) {
error:
		if (fc->comp_typ < fi->num_comps) {
			ch = &fi->cmp_hdr[fc->comp_typ];
			ch->status = CH_STAT_FAILED;
		}

		complete_fmapi_req(a, rq, FI_STAT_FAILED);
	}
}

/* Determine the flash image adaptyp for this adapter */
static u8 get_fi_adap_type(struct esas2r_adapter *a)
{
	u8 type;

	/* use the device ID to get the correct adap_typ for this HBA */
	switch (a->pcid->device) {
	case ATTO_DID_INTEL_IOP348:
		type = FI_AT_SUN_LAKE;
		break;

	case ATTO_DID_MV_88RC9580:
	case ATTO_DID_MV_88RC9580TS:
	case ATTO_DID_MV_88RC9580TSE:
	case ATTO_DID_MV_88RC9580TL:
		type = FI_AT_MV_9580;
		break;

	default:
		type = FI_AT_UNKNWN;
		break;
	}

	return type;
}

/* Size of config + copyright + flash_ver images, 0 for failure. */
static u32 chk_cfg(u8 *cfg, u32 length, u32 *flash_ver)
{
	u16 *pw = (u16 *)cfg - 1;
	u32 sz = 0;
	u32 len = length;

	if (len == 0)
		len = FM_BUF_SZ;

	if (flash_ver)
		*flash_ver = 0;

	while (true) {
		u16 type;
		u16 size;

		type = le16_to_cpu(*pw--);
		size = le16_to_cpu(*pw--);

		if (type != FBT_CPYR
		    && type != FBT_SETUP
		    && type != FBT_FLASH_VER)
			break;

		if (type == FBT_FLASH_VER
		    && flash_ver)
			*flash_ver = le32_to_cpu(*(u32 *)(pw - 1));

		sz += size + (2 * sizeof(u16));
		pw -= size / sizeof(u16);

		if (sz > len - (2 * sizeof(u16)))
			break;
	}

	/* See if we are comparing the size to the specified length */
	if (length && sz != length)
		return 0;

	return sz;
}

/* Verify that the boot image is valid */
static u8 chk_boot(u8 *boot_img, u32 length)
{
	struct esas2r_boot_image *bi = (struct esas2r_boot_image *)boot_img;
	u16 hdroffset = le16_to_cpu(bi->header_offset);
	struct esas2r_boot_header *bh;

	if (bi->signature != le16_to_cpu(0xaa55)
	    || (long)hdroffset >
	    (long)(65536L - sizeof(struct esas2r_boot_header))
	    || (hdroffset & 3)
	    || (hdroffset < sizeof(struct esas2r_boot_image))
	    || ((u32)hdroffset + sizeof(struct esas2r_boot_header) > length))
		return 0xff;

	bh = (struct esas2r_boot_header *)((char *)bi + hdroffset);

	if (bh->signature[0] != 'P'
	    || bh->signature[1] != 'C'
	    || bh->signature[2] != 'I'
	    || bh->signature[3] != 'R'
	    || le16_to_cpu(bh->struct_length) <
	    (u16)sizeof(struct esas2r_boot_header)
	    || bh->class_code[2] != 0x01
	    || bh->class_code[1] != 0x04
	    || bh->class_code[0] != 0x00
	    || (bh->code_type != CODE_TYPE_PC
		&& bh->code_type != CODE_TYPE_OPEN
		&& bh->code_type != CODE_TYPE_EFI))
		return 0xff;

	return bh->code_type;
}

/* The sum of all the WORDS of the image */
static u16 calc_fi_checksum(struct esas2r_flash_context *fc)
{
	struct esas2r_flash_img *fi = fc->fi;
	u16 cksum;
	u32 len;
	u16 *pw;

	for (len = (fi->length - fc->fi_hdr_len) / 2,
	     pw = (u16 *)((u8 *)fi + fc->fi_hdr_len),
	     cksum = 0;
	     len;
	     len--, pw++)
		cksum = cksum + le16_to_cpu(*pw);

	return cksum;
}

/*
 * Verify the flash image structure.  The following verifications will
 * be performed:
 *              1)  verify the fi_version is correct
 *              2)  verify the checksum of the entire image.
 *              3)  validate the adap_typ, action and length fields.
 *              4)  validate each component header. check the img_type and
 *                  length fields
 *              5)  validate each component image.  validate signatures and
 *                  local checksums
 */
static bool verify_fi(struct esas2r_adapter *a,
		      struct esas2r_flash_context *fc)
{
	struct esas2r_flash_img *fi = fc->fi;
	u8 type;
	bool imgerr;
	u16 i;
	u32 len;
	struct esas2r_component_header *ch;

	/* Verify the length - length must even since we do a word checksum */
	len = fi->length;

	if ((len & 1)
	    || len < fc->fi_hdr_len) {
		fi->status = FI_STAT_LENGTH;
		return false;
	}

	/* Get adapter type and verify type in flash image */
	type = get_fi_adap_type(a);
	if ((type == FI_AT_UNKNWN) || (fi->adap_typ != type)) {
		fi->status = FI_STAT_ADAPTYP;
		return false;
	}

	/*
	 * Loop through each component and verify the img_type and length
	 * fields.  Keep a running count of the sizes sooze we can verify total
	 * size to additive size.
	 */
	imgerr = false;

	for (i = 0, len = 0, ch = fi->cmp_hdr;
	     i < fi->num_comps;
	     i++, ch++) {
		bool cmperr = false;

		/*
		 * Verify that the component header has the same index as the
		 * image type.  The headers must be ordered correctly
		 */
		if (i != ch->img_type) {
			imgerr = true;
			ch->status = CH_STAT_INVALID;
			continue;
		}

		switch (ch->img_type) {
		case CH_IT_BIOS:
			type = CODE_TYPE_PC;
			break;

		case CH_IT_MAC:
			type = CODE_TYPE_OPEN;
			break;

		case CH_IT_EFI:
			type = CODE_TYPE_EFI;
			break;
		}

		switch (ch->img_type) {
		case CH_IT_FW:
		case CH_IT_NVR:
			break;

		case CH_IT_BIOS:
		case CH_IT_MAC:
		case CH_IT_EFI:
			if (ch->length & 0x1ff)
				cmperr = true;

			/* Test if component image is present  */
			if (ch->length == 0)
				break;

			/* Image is present - verify the image */
			if (chk_boot((u8 *)fi + ch->image_offset, ch->length)
			    != type)
				cmperr = true;

			break;

		case CH_IT_CFG:

			/* Test if component image is present */
			if (ch->length == 0) {
				cmperr = true;
				break;
			}

			/* Image is present - verify the image */
			if (!chk_cfg((u8 *)fi + ch->image_offset + ch->length,
				     ch->length, NULL))
				cmperr = true;

			break;

		default:

			fi->status = FI_STAT_UNKNOWN;
			return false;
		}

		if (cmperr) {
			imgerr = true;
			ch->status = CH_STAT_INVALID;
		} else {
			ch->status = CH_STAT_PENDING;
			len += ch->length;
		}
	}

	if (imgerr) {
		fi->status = FI_STAT_MISSING;
		return false;
	}

	/* Compare fi->length to the sum of ch->length fields */
	if (len != fi->length - fc->fi_hdr_len) {
		fi->status = FI_STAT_LENGTH;
		return false;
	}

	/* Compute the checksum - it should come out zero */
	if (fi->checksum != calc_fi_checksum(fc)) {
		fi->status = FI_STAT_CHKSUM;
		return false;
	}

	return true;
}

/* Fill in the FS IOCTL response data from a completed request. */
static void esas2r_complete_fs_ioctl(struct esas2r_adapter *a,
				     struct esas2r_request *rq)
{
	struct esas2r_ioctl_fs *fs =
		(struct esas2r_ioctl_fs *)rq->interrupt_cx;

	if (rq->vrq->flash.sub_func == VDA_FLASH_COMMIT)
		esas2r_enable_heartbeat(a);

	fs->driver_error = rq->req_stat;

	if (fs->driver_error == RS_SUCCESS)
		fs->status = ATTO_STS_SUCCESS;
	else
		fs->status = ATTO_STS_FAILED;
}

/* Prepare an FS IOCTL request to be sent to the firmware. */
bool esas2r_process_fs_ioctl(struct esas2r_adapter *a,
			     struct esas2r_ioctl_fs *fs,
			     struct esas2r_request *rq,
			     struct esas2r_sg_context *sgc)
{
	u8 cmdcnt = (u8)ARRAY_SIZE(cmd_to_fls_func);
	struct esas2r_ioctlfs_command *fsc = &fs->command;
	u8 func = 0;
	u32 datalen;

	fs->status = ATTO_STS_FAILED;
	fs->driver_error = RS_PENDING;

	if (fs->version > ESAS2R_FS_VER) {
		fs->status = ATTO_STS_INV_VERSION;
		return false;
	}

	if (fsc->command >= cmdcnt) {
		fs->status = ATTO_STS_INV_FUNC;
		return false;
	}

	func = cmd_to_fls_func[fsc->command];
	if (func == 0xFF) {
		fs->status = ATTO_STS_INV_FUNC;
		return false;
	}

	if (fsc->command != ESAS2R_FS_CMD_CANCEL) {
		if ((a->pcid->device != ATTO_DID_MV_88RC9580
		     || fs->adap_type != ESAS2R_FS_AT_ESASRAID2)
		    && (a->pcid->device != ATTO_DID_MV_88RC9580TS
			|| fs->adap_type != ESAS2R_FS_AT_TSSASRAID2)
		    && (a->pcid->device != ATTO_DID_MV_88RC9580TSE
			|| fs->adap_type != ESAS2R_FS_AT_TSSASRAID2E)
		    && (a->pcid->device != ATTO_DID_MV_88RC9580TL
			|| fs->adap_type != ESAS2R_FS_AT_TLSASHBA)) {
			fs->status = ATTO_STS_INV_ADAPTER;
			return false;
		}

		if (fs->driver_ver > ESAS2R_FS_DRVR_VER) {
			fs->status = ATTO_STS_INV_DRVR_VER;
			return false;
		}
	}

	if (test_bit(AF_DEGRADED_MODE, &a->flags)) {
		fs->status = ATTO_STS_DEGRADED;
		return false;
	}

	rq->interrupt_cb = esas2r_complete_fs_ioctl;
	rq->interrupt_cx = fs;
	datalen = le32_to_cpu(fsc->length);
	esas2r_build_flash_req(a,
			       rq,
			       func,
			       fsc->checksum,
			       le32_to_cpu(fsc->flash_addr),
			       datalen);

	if (func == VDA_FLASH_WRITE
	    || func == VDA_FLASH_READ) {
		if (datalen == 0) {
			fs->status = ATTO_STS_INV_FUNC;
			return false;
		}

		esas2r_sgc_init(sgc, a, rq, rq->vrq->flash.data.sge);
		sgc->length = datalen;

		if (!esas2r_build_sg_list(a, rq, sgc)) {
			fs->status = ATTO_STS_OUT_OF_RSRC;
			return false;
		}
	}

	if (func == VDA_FLASH_COMMIT)
		esas2r_disable_heartbeat(a);

	esas2r_start_request(a, rq);

	return true;
}

static bool esas2r_flash_access(struct esas2r_adapter *a, u32 function)
{
	u32 starttime;
	u32 timeout;
	u32 intstat;
	u32 doorbell;

	/* Disable chip interrupts awhile */
	if (function == DRBL_FLASH_REQ)
		esas2r_disable_chip_interrupts(a);

	/* Issue the request to the firmware */
	esas2r_write_register_dword(a, MU_DOORBELL_IN, function);

	/* Now wait for the firmware to process it */
	starttime = jiffies_to_msecs(jiffies);

	if (test_bit(AF_CHPRST_PENDING, &a->flags) ||
	    test_bit(AF_DISC_PENDING, &a->flags))
		timeout = 40000;
	else
		timeout = 5000;

	while (true) {
		intstat = esas2r_read_register_dword(a, MU_INT_STATUS_OUT);

		if (intstat & MU_INTSTAT_DRBL) {
			/* Got a doorbell interrupt.  Check for the function */
			doorbell =
				esas2r_read_register_dword(a, MU_DOORBELL_OUT);
			esas2r_write_register_dword(a, MU_DOORBELL_OUT,
						    doorbell);
			if (doorbell & function)
				break;
		}

		schedule_timeout_interruptible(msecs_to_jiffies(100));

		if ((jiffies_to_msecs(jiffies) - starttime) > timeout) {
			/*
			 * Iimeout.  If we were requesting flash access,
			 * indicate we are done so the firmware knows we gave
			 * up.  If this was a REQ, we also need to re-enable
			 * chip interrupts.
			 */
			if (function == DRBL_FLASH_REQ) {
				esas2r_hdebug("flash access timeout");
				esas2r_write_register_dword(a, MU_DOORBELL_IN,
							    DRBL_FLASH_DONE);
				esas2r_enable_chip_interrupts(a);
			} else {
				esas2r_hdebug("flash release timeout");
			}

			return false;
		}
	}

	/* if we're done, re-enable chip interrupts */
	if (function == DRBL_FLASH_DONE)
		esas2r_enable_chip_interrupts(a);

	return true;
}

#define WINDOW_SIZE ((signed int)MW_DATA_WINDOW_SIZE)

bool esas2r_read_flash_block(struct esas2r_adapter *a,
			     void *to,
			     u32 from,
			     u32 size)
{
	u8 *end = (u8 *)to;

	/* Try to acquire access to the flash */
	if (!esas2r_flash_access(a, DRBL_FLASH_REQ))
		return false;

	while (size) {
		u32 len;
		u32 offset;
		u32 iatvr;

		if (test_bit(AF2_SERIAL_FLASH, &a->flags2))
			iatvr = MW_DATA_ADDR_SER_FLASH + (from & -WINDOW_SIZE);
		else
			iatvr = MW_DATA_ADDR_PAR_FLASH + (from & -WINDOW_SIZE);

		esas2r_map_data_window(a, iatvr);
		offset = from & (WINDOW_SIZE - 1);
		len = size;

		if (len > WINDOW_SIZE - offset)
			len = WINDOW_SIZE - offset;

		from += len;
		size -= len;

		while (len--) {
			*end++ = esas2r_read_data_byte(a, offset);
			offset++;
		}
	}

	/* Release flash access */
	esas2r_flash_access(a, DRBL_FLASH_DONE);
	return true;
}

bool esas2r_read_flash_rev(struct esas2r_adapter *a)
{
	u8 bytes[256];
	u16 *pw;
	u16 *pwstart;
	u16 type;
	u16 size;
	u32 sz;

	sz = sizeof(bytes);
	pw = (u16 *)(bytes + sz);
	pwstart = (u16 *)bytes + 2;

	if (!esas2r_read_flash_block(a, bytes, FLS_OFFSET_CPYR - sz, sz))
		goto invalid_rev;

	while (pw >= pwstart) {
		pw--;
		type = le16_to_cpu(*pw);
		pw--;
		size = le16_to_cpu(*pw);
		pw -= size / 2;

		if (type == FBT_CPYR
		    || type == FBT_ || type == eqj:R6xR6Un_;

	sztype == FBT_ || type == eqj:R6xR6Un_;

	sztype == FBT_ || type == eqj:R6xR6Un_;

	sztype == FBT_ || type == eqj:ROT_ || type == eqj:R6xR6Un_;

	sztype == FBT_ || type == eqj:ROT_ || type == eqj:R6xR6Un_;

	sztyp (lk*= FBT_ || type == eqj:ROT_+w--;
		size = le16#'->length = datalen;

		if (!esas2r'Bst tth
	:R6UAeqj:R6xR6Un_;

	sztype == FBT_ || type == eqj#:R6xR6M;
	u16R6gE16xR6M-;
		sk*= FB		if (OMMIT)
		esas2Oth
	 * for the E.xi6xR6xR6xR6xR6xR6U

		ixk4HR6xR6wBdsas2e2Oth
	 * for the E.xi6xR6xR6xR6xR6xR6U

		ixk4HR6xR6wBdsas2e2Oth
	 * for the E.xi6xR6xR6xR6xR6xR6ete the FM API
		 * request.  We should only get here if a BEGINW or WRITE
		 * operation failed.
		 */
		(*fc->i if a BEGINW or WRITE
		 * operation failed.
		 */
		(*fc->i if a BEGINW or WRITE
		 * operation failed.
		 */
		 filled in on an error.
 */
static void build_flash_msg(struct esas2r_adapter *a,
			    struct esas2r_request *rq)
{
	struct esas2r_flash_context *fc =
		(struct esas2r_flash_context *)rq->interrupt_cx;
	struct esas2r_sg_conts2r_flash_context *)rq->interrupt_cx;
	struct esas2r_sg_conts2r_flash_context *)rq->interrupt_cx;
	struct esas2r_text *)rq->interrupt_cx;
	struct esas2r_text *)rq->interrupt_cx;
	struct esas2r_text *)rq->interrupt_cx;
	struct I;
		6xcxk:R6b== DRE}

bool esas2r_read_flash_rev(struct e_sg_conts2rRi6xR6xR6g	:R6UAeqj2 datalen;

	fs->statvxR6P
			    sxi6xen;

	fsk:R6brt_requesct e_si6xR6Uorb_addr,
			       sgc->length);

	esas2r_rq_free_sg_lists(rq, a);

	/*
	 * remember the length we ave to keep track of
	 * the current amount done so we know how much to compare when
	 * doing the verification pe know how much to compare when
	 * doing the verification pe know how much to compare when
	 * doing the verific_stat)
{
	struct esas2r_flash_context *fc =
		(struct esas2r_flash_context *)rq->interrupt_cx;
	struct esas2r_flaO>curr_len; /* # left to compare    */

		/* Update fc and determine the length for the next uploa,_res:R6PR - szpanlash_ROM
	     chr_rnext ;

	   */
r = FLSthe le co2r_f16 *addr +*/
		(*fc->i if au32 len = r WRITE
		 * operation failed.
		 */
		 filled *)((u8 *)fi + fc->fi_hdr_leled *)((u8 *)fi + fc->* Verify thesg(struct= FI_AT_UNKNT_ || type f the 
{
	slen;
	u1			goto errpare    */e ESASloo_REQ))a (len =TL requestsas2r_adapter *a,
			gnaturet *	fs->sext */
	s2Oth
	 *    void *to,
			     uext *fc =
		(struct esas2r_flash_context *)rq->inxt *f (!load_imagw */;
	else
	ED;
			strucct esas2r_sg_conts2r_	len),
	     cksum = 0;
	     len;
_conmparust return. */
	if (rq->req_stat != RS_SUbCESS)
		goto error;

	/*
	 * If an ue16_to_cpu(*pw);

e knolash image structure.ngth is non-0xAA55strucct esas2r_sg_conts2r_	ength fird checksu=>status = FI_S the FMhen
	 * dou32 len = le"he F" length esas2r_flaO>ize;
	u32ength fird checksu=>status = FIi->lthe FMhenrupt_cx;
	struct esas2r_fl1);
		len = sSo Recalcul = nent head+= ch-r offsesev   /*ecksslen = sr_rserrupicfair thteto ad Recalcu		}
	}

	/* n;
	u8 *p, *q;

	/* If the previ
_conmpe FMh{
		fi-error;

	/*
ploama thncheckst) {
			iver_nense dHINE_IA32ed reMhen
	 * dou32 len = le"nent32-(lk true;
} esas2r_flaO>
			iver_nense dHINE_IA64ed reMhen
	 * dou32 len = le"nentitanium true;
} esas2r_flaO>
			iver_nense dHINE_X64ed reMhen
	 * dou32 len = le"nent64-(lk true;
} esas2r_flaO>
			iver_nense dHINE_EBCed reMhen
	 * dou32 len = le"nentEBC true;
} esas2r_flaO>
			_len = fc->sct esas2r_sg_conts2 + szize;
	u32 sz;
der *ch;
	u32 le thejump =
			cpu_to_TL request tst read back part of the image we just wrote.  verif	 the
	 * sectiknow htruct esas* sectiion fail       htruct esaage wOW_SIZdap_ty    CATOR_ct T	struct esas2r_iore image is verifiedr WRITEnts2rRi6xR6xR6hen
	 * dou32 len = le"norpare    */e"len; len; len--)
			if ype =R];
			ch-	case CHc->flsh_NV2   param*/
	s2r_e a BEGingype =physibreaNV2   di) {
		ecksumfc and detsZdae cparam*/
	s2 dev_waas2r_sg_,8 *boot_imgen = saddr +*/
		(*fc->iS mulst.  Wei) {
r WRITE
		 * operation failed.*/
		res =  the currownSH + (from & -Weqj:S mulsse;
	hdae= eqj:R6xR6Un_;

	sztuext *fc =
		(struct esas2r_flasqj:S multext *)rq->inNV2pcid->deviceTO },                         t) {
		ct esas2r_flashNV2   p];
	ses a 'w), \
	mgen = s true;upWeqj:S mulsse;
	hdae=false;
	}

	if (fsc->comms = 	 * requesS muls-	case Cart) {
upWeqj:S mulsse;
	hdae=fader *ch = es =  tt esas2r_flash_context *)rq->interruNV2   	timeou     
		     || fs->adap_typS mulsu8 cksum = 0;

	/* calculate the checksum */
	if (fc->func == VDA_FLASH_BEGINW) {
		if (sgc->cur_offset)
			cksum = esas2ATTO_DID_M esas2r_fmapi_callback;
	} elsel{
		rq->interrupt_cb = fc->inserrupt_cb;
	}
	esas2r_
build_flash_req(a,
			       rq,
			       fc->func,
		->length);

	esas2r_rq_free_sg_lists(rq, a);

	/*
	 * remember the length we asked for.  we have t
		->length);

	esas2r_rq_fret done so we know how much to compare whe length we asked for.  weter_:st_bit(AF2_S muls-	case Cart) whe length we asked for.  wet done:st__len = fc->sgc.length;

	if (sgc->cur_offset) {
		/* setup 
	} elsenp_header_bNV2   CESS;>status = ID_M esas2r_fmapi_callback;e FMheA_WINDOW_r_fla
	st= fc->curr_len;:R6UAeqj:uffer
		 */
	r_fla
	st= fc->curr_leninvalid_rev;

	whave
			 * up.  I;upWeqj:S mulsse;
	hdae=fal}			if ype WEDWORD_INVAr_fnFS_DRVS mult=
			cpulate t's=physibreaNV2  ength for aTA_A             NV2   n 0;2r_rnp_headaddr +*/
		(*fc->iS mulsesas2 esas2r_calc_byte_cksum(pnp_header_bytes,
							   ,e FMhenrupt_cx;
	s          *      S
			|| fs->adap_t          * uch     ber thstptr_t)V_DR */
		8flash_co*stptr_t)V_DRsh_res back puct stptr_t)V_DR */
		0lled *)((u8) {
		if (sgc->cur_offset)
			cksum = esas2ATTO_D
		type = FI_AT_UNKNWN;
		break;
	}

	ret:R6xR6Un_;

	sztuextrownSH + (from & -Weqj:S mulsse;
	hdae= eqj:R6xR6Un_;

	sztuextn   fc->flsh_ uchqj:S mule f the->img_type-	cas'			 fi + chetZE -es, FLS_OFnsc->flash_ad* phy_flags    || typpWeqj:S mulsse;
	hdae=false;
	}

	if (fsc->co sz = 0 stptr_t)V_DR */
		0l, nscstptr_t), 8* the curstptr_t)V_DR */
		0lstruct5w how muchstptr_t)V_DR */
		1r_flash_img *fi = stptr_t)V_DR */
		2r_flash_8sas2r_compstptr_t)V_DR */
		3]ic voF0)_flash6esas2r_comppstptr_t)V_DR */
		3]ic vo0F) = stptr_t)V_DRsh_res	1r)ELL_OUT,
		ppWeqj:S mulsse;
	hdae=false;
	}

	if (fsc->coS_OFnscH_CANCEL
};

s>*addr;
		aINUPtimelsh_ scH_CANCEL
};

s=*addr;
		aINUPtime thensc->flash_=d* phy_flags   ;hensc(a);
	clearnsc(a);
	cle-/* spin_up_delay      */
	;
2 sz = 0;
	S multen       /* ssp_align_rate     */
	 chip inwEDWORD_INNV2   , FL uchqj:S mule mware knows we gave
			 * up.  If this S_OUT);

		if (intstat & MU_INTSTAT_DRBL) {se VDA_FLASH_WRITE:OCK, &a->flags);     /* sas_addr      TE:OCK, &a->xt *)rq->inNV2pcid-K, &a->     /* ssp_align_rate     */
	 chipTO_D
		type = FILEGACY_SGEAF_CHPRST_PENDING, &a
		->lenet) {
		u8 	break;

eqj:unterrupt_cxSGEAct T |cid->devi     /* ssp_align_rate     */
	 ch		->lenet) {
		u8 	r_t)V_D e.ngth is n64(xcxk:R6

	aTA_A_phystypeu64!= RS_SUpe;
:R6

	aTA_A	 ch	e;
	u32 sz;->lenet) {pren;
	.MODEectikeqj:unterrupt_cx     /* ssp_align_rate     */
	 ch		->lenet) {pren;
	.r_t)V_D e.ngth is n64(xcxk:R6

	aTA_A_physxcxkypeu64!= RS_SUpe;
:R6

	aTA_A	 ch	e)
		timeout = 40000;
	else
	S mulsu8 cksum; If this was a REQ, we also neeommand != ESAS2R_FS_Vas the same  aTA_A NV2  e ified s NV2   n 0s2r_sg_,89580Tot_imgen = sadr +*/
		(*fc->iS muls-	case Ca WRITE
		 * operation failed.	|| fs->adap_t          * uchqj:S mule m*/
		rs= 	 *n_;

	sztuextnfields
 *          Ems
 */
stanate each component Sms
 */
stanate each comes and
Ams
 */
stanate each com3onent Sm) {
		ct esas2r_flashs2r_sg_bNV2   C each co" ch	e;
	u32uext* spin_up_delay      */
	) {
		ct esas2r_flashs2r_sg_bNV2   (a);
	cl" ch	e;
	u32uextnsc->flash_ad* phy_flags    || tyct esas2r_flashs2r_sg_bNV2   ->flash" ch	e;
	u32 sz;heA_WINDOW_r_fla
	st= fc->curr_len;:rs= 	 *_flaO>i	if (sgc-s= 	  *n_;

 || tyct esas2r_flash), \
	mgen = s true;(*fc->iS mulsheA_mgen = sart) w->command !-s= 
			if ype S)
{
	straTA_A NV2  cb;
	gen = sad no_eCH_STAT_ wriSET_CPYRheAsTot_imgen = ype NV2  c
			 it>cmp_hdr[fsgc.lengtdCH_STAT_INphysibreaNV2   n 0s2r_sg_engthInAT_ wr ask,ate thAS r_t)V_D  wriabr_SIZEdaddr +fs->adap_typS mulsheA_mgen = sa WRITE
		 * operation failed.	|| fs->adap_t          * uchqj:S mule mcess timr_enable_chip_interrupts(a);

	ruffer
		 */
	r_fla
	st= fc->curr_len	* uch       /* SAS mul;henscstptr_t)m3on|lash_F;henscstptr_t)m4]gth we avec->leng & (W);henscstptr_t)m5]sg_lists(rc->leng & (W);henscstptr_t)m6]uchqj:hile *bus			reber;henscstptr_t)m7]uchqj:hile *devf    a-fs->adap_typS mulsgeA_mgen = sa WRITE
		 * operation faq = (u8 *)fi                        *      S
		 thstptr_t)	8fla) {
				cinr ask */
staticpy calc_fimgen = s	siz
			cpulate t,      te thAS0);
	}_t)V_D  *a,first
				cmp sz = 0 stptr_t)	0l, qj:S mulscstptr_t), 8* t	*     uch       /* SAS mul;he sz = 0 S mulscstptr_t)	0l,  stptr_t)	0l, 8vxR6P
			    sxi6xfm_approcess. */
static void fw_download_proc(struct esas2r_	case C                    /* star	return false;
	}

	rq->interrupt_oad */
		if (fc->cmp_len > FM_BUF fc->curcmp_len > ber thjled *)((u8 *)fi +_BIOS:
		case CH_IT_MAC:TO_D
		tyand_heA_WINDOW_	for (len = fc->curr_l 
	} elsecurr			*enlp];
yfunc nd
			TO_STS_INV_FUNC;
		r/* Idente;
	}

	if (fsc->co sz = 0 VER)
		imeou       /* ssp_align_rate	}

	rq->W);heith the status selse;
	->sta selse;tion */
	if= evice_id = cpu_;  */

		/*
		 * NOTE - curr_lwhile (true) {
		intsEFI igth;
			fie;
	}	if (flaoot_img;
UNClags   _1fc->he correct a=mppswnload_proc(struct esas2his  correct  S_f;c->he c	return fV_FUNCNUMet dPS_V1;c->he c
	}

	if (s2r_adapteswnload_proc(struct esaed and)
	    || (hdroffset of thef we are comparing the size to thbytes fo;secs(jiffies);

	if (test_bit(AF_CHPRST_PENDING,fset of thef we are comparing the size to thest_bit()FI igth;
			fie;he
		 aoot_img;
UNCACtheOWN:FFSED8RC9580TT_INVALID;
		es, FL:
		case CH_IT_ 0, atgoto error;

	TL request uext ype = CODE thtatungth esas2ref we are comparing the si	TO_STS_INer_error Ad for      ch->= ATTO_H_STAffsetepenfi_su16 *)cf = (strucase FMTSK_WRTMAC:
			/* The MAC
S)
		fs->status LASH_Vstruct e thtier_error Ad for     nent= ATTO_H_STAffsetepenfi_su16 *)cf = (strucase FMTSK_WRTMAC:
			/* _WRTCF
S)
		fs->status LASH_VstreODE thtier_error
n = sS* The MACength
		*e for mod esas,ref weturn false;
	}

on
n = stem_dod esast head+= Firstenp_header_bCRCREQ))
		rVALIDsDWO
n = sszpanlash_ROMy that tt and
			eat(a);
	clear= 0xFF) {
		fs->statr_error (a, MU_Dth->len
			 *m in the PNP ows we gave
			 * up.  Ip interr)
{
	s= VDA_F88RC9580Ts     Thend
			STAT_FAILED;
		}sgc_offset;c->he c
there */
		if (fc->func ==VDA_FLASH_READ
		    && fc->The EFI image is written next */
		get_fi_adap_type(st *	fs->sext */
	_adapter *a)
{
	u8 typ       Ip inS>pcid->deontext *)}_t)V_D nd
			STA called before d left to compard and)
	    |img;
UNCACthUPSZ:FFSEGegc.cur_ofh->imand
			eatCFG:

					    != type)
				cmper		eatENDINo compar;
			return fg_type)	return fals	;
		return =alse;
	}

	if (   Ip inResas2# left to compare    */
ermine Z;

	if (flr	re calnd
		 sz = 0ngth;

	if (len   dou32 len = l
 0)
		len = FM_BUF_SZ;

