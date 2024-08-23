/*
 * Driver for USB Mass Storage compliant devices
 *
 * Current development and maintenance by:
 *   (c) 1999-2002 Matthew Dharm (mdharm-usb@one-eyed-alien.net)
 *
 * Developed with the assistance of:
 *   (c) 2000 David L. Brown, Jr. (usb-storage@davidb.org)
 *   (c) 2000 Stephen J. Gowdy (SGowdy@lbl.gov)
 *   (c) 2002 Alan Stern <stern@rowland.org>
 *
 * Initial work by:
 *   (c) 1999 Michael Gee (michael@linuxspecific.com)
 *
 * This driver is based on the 'USB Mass Storage Class' document. This
 * describes in detail the protocol used to communicate with such
 * devices.  Clearly, the designers had SCSI and ATAPI commands in
 * mind when they created this document.  The commands are all very
 * similar to commands in the SCSI-II and ATAPI specifications.
 *
 * It is important to note that in a number of cases this class
 * exhibits class-specific exemptions from the USB specification.
 * Notably the usage of NAK, STALL and ACK differs from the norm, in
 * that they are used to communicate wait, failed and OK on commands.
 *
 * Also, for certain devices, the interrupt endpoint is used to convey
 * status of a command.
 *
 * Please see http://www.one-eyed-alien.net/~mdharm/linux-usb for more
 * information about this driver.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/sched.h>
#include <linux/gfp.h>
#include <linux/errno.h>
#include <linux/export.h>

#include <linux/usb/quirks.h>

#include <scsi/scsi.h>
#include <scsi/scsi_eh.h>
#include <scsi/scsi_device.h>

#include "usb.h"
#include "transport.h"
#include "protocol.h"
#include "scsiglue.h"
#include "debug.h"

#include <linux/blkdev.h>
#include "../../scsi/sd.h"
#include "usb_boost.h"


/***********************************************************************
 * Data transfer routines
 ***********************************************************************/

/*
 * This is subtle, so pay attention:
 * ---------------------------------
 * We're very concerned about races with a command abort.  Hanging this code
 * is a sure fire way to hang the kernel.  (Note that this discussion applies
 * only to transactions resulting from a scsi queued-command, since only
 * these transactions are subject to a scsi abort.  Other transactions, such
 * as those occurring during device-specific initialization, must be handled
 * by a separate code path.)
 *
 * The abort function (usb_storage_command_abort() in scsiglue.c) first
 * sets the machine state and the ABORTING bit in us->dflags to prevent
 * new URBs from being submitted.  It then calls usb_stor_stop_transport()
 * below, which atomically tests-and-clears the URB_ACTIVE bit in us->dflags
 * to see if the current_urb needs to be stopped.  Likewise, the SG_ACTIVE
 * bit is tested to see if the current_sg scatter-gather request needs to be
 * stopped.  The timeout callback routine does much the same thing.
 *
 * When a disconnect occurs, the DISCONNECTING bit in us->dflags is set to
 * prevent new URBs from being submitted, and usb_stor_stop_transport() is
 * called to stop any ongoing requests.
 *
 * The submit function first verifies that the submitting is allowed
 * (neither ABORTING nor DISCONNECTING bits are set) and that the submit
 * completes without errors, and only then sets the URB_ACTIVE bit.  This
 * prevents the stop_transport() function from trying to cancel the URB
 * while the submit call is underway.  Next, the submit function must test
 * the flags to see if an abort or disconnect occurred during the submission
 * or before the URB_ACTIVE bit was set.  If so, it's essential to cancel
 * the URB if it hasn't been cancelled already (i.e., if the URB_ACTIVE bit
 * is still set).  Either way, the function must then wait for the URB to
 * finish.  Note that the URB can still be in progress even after a call to
 * usb_unlink_urb() returns.
 *
 * The idea is that (1) once the ABORTING or DISCONNECTING bit is set,
 * either the stop_transport() function or the submitting function
 * is guaranteed to call usb_unlink_urb() for an active URB,
 * and (2) test_and_clear_bit() prevents usb_unlink_urb() from being
 * called more than once or from being called during usb_submit_urb().
 */

/*
 * This is the completion handler which will wake us up when an URB
 * completes.
 */
static void usb_stor_blocking_completion(struct urb *urb)
{
	struct completion *urb_done_ptr = urb->context;

	complete(urb_done_ptr);
}

/*
 * This is the common part of the URB message submission code
 *
 * All URBs from the usb-storage driver involved in handling a queued scsi
 * command _must_ pass through this function (or something like it) for the
 * abort mechanisms to work properly.
 */
static int usb_stor_msg_common(struct us_data *us, int timeout)
{
	struct completion urb_done;
	long timeleft;
	int status;

	/* don't submit URBs during abort processing */
	if (test_bit(US_FLIDX_ABORTING, &us->dflags))
		return -EIO;

	/* set up data structures for the wakeup system */
	init_completion(&urb_done);

	/* fill the common fields in the URB */
	us->current_urb->context = &urb_done;
	us->current_urb->transfer_flags = 0;

	/*
	 * we assume that if transfer_buffer isn't us->iobuf then it
	 * hasn't been mapped for DMA.  Yes, this is clunky, but it's
	 * easier than always having the caller tell us whether the
	 * transfer buffer has already been mapped.
	 */
	if (us->current_urb->transfer_buffer == us->iobuf)
		us->current_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	us->current_urb->transfer_dma = us->iobuf_dma;

	/* submit the URB */
	status = usb_submit_urb(us->current_urb, GFP_NOIO);
	if (status) {
		/* something went wrong */
		return status;
	}

	/*
	 * since the URB has been submitted successfully, it's now okay
	 * to cancel it
	 */
	set_bit(US_FLIDX_URB_ACTIVE, &us->dflags);

	/* did an abort occur during the submission? */
	if (test_bit(US_FLIDX_ABORTING, &us->dflags)) {

		/* cancel the URB, if it hasn't been cancelled already */
		if (test_and_clear_bit(US_FLIDX_URB_ACTIVE, &us->dflags)) {
			usb_stor_dbg(us, "-- cancelling URB\n");
			usb_unlink_urb(us->current_urb);
		}
	}
 
	/* wait for the completion of the URB */
	timeleft = wait_for_completion_interruptible_timeout(
			&urb_done, timeout ? : MAX_SCHEDULE_TIMEOUT);
 
	clear_bit(US_FLIDX_URB_ACTIVE, &us->dflags);

	if (timeleft <= 0) {
		usb_stor_dbg(us, "%s -- cancelling URB\n",
			     timeleft == 0 ? "Timeout" : "Signal");
		usb_kill_urb(us->current_urb);
	}

	/* return the URB status */
	return us->current_urb->status;
}

/*
 * Transfer one control message, with timeouts, and allowing early
 * termination.  Return codes are usual -Exxx, *not* USB_STOR_XFER_xxx.
 */
int usb_stor_control_msg(struct us_data *us, unsigned int pipe,
		 u8 request, u8 requesttype, u16 value, u16 index, 
		 void *data, u16 size, int timeout)
{
	int status;

	usb_stor_dbg(us, "rq=%02x rqtype=%02x value=%04x index=%02x len=%u\n",
		     request, requesttype, value, index, size);

	/* fill in the devrequest structure */
	us->cr->bRequestType = requesttype;
	us->cr->bRequest = request;
	us->cr->wValue = cpu_to_le16(value);
	us->cr->wIndex = cpu_to_le16(index);
	us->cr->wLength = cpu_to_le16(size);

	/* fill and submit the URB */
	usb_fill_control_urb(us->current_urb, us->pusb_dev, pipe, 
			 (unsigned char*) us->cr, data, size, 
			 usb_stor_blocking_completion, NULL);
	status = usb_stor_msg_common(us, timeout);

	/* return the actual length of the data transferred if no error */
	if (status == 0)
		status = us->current_urb->actual_length;
	return status;
}
EXPORT_SYMBOL_GPL(usb_stor_control_msg);

/*
 * This is a version of usb_clear_halt() that allows early termination and
 * doesn't read the status from the device -- this is because some devices
 * crash their internal firmware when the status is requested after a halt.
 *
 * A definitive list of these 'bad' devices is too difficult to maintain or
 * make complete enough to be useful.  This problem was first observed on the
 * Hagiwara FlashGate DUAL unit.  However, bus traces reveal that neither
 * MacOS nor Windows checks the status after clearing a halt.
 *
 * Since many vendors in this space limit their testing to interoperability
 * with these two OSes, specification violations like this one are common.
 */
int usb_stor_clear_halt(struct us_data *us, unsigned int pipe)
{
	int result;
	int endp = usb_pipeendpoint(pipe);

	if (usb_pipein (pipe))
		endp |= USB_DIR_IN;

	result = usb_stor_control_msg(us, us->send_ctrl_pipe,
		USB_REQ_CLEAR_FEATURE, USB_RECIP_ENDPOINT,
		USB_ENDPOINT_HALT, endp,
		NULL, 0, 3*HZ);

	if (result >= 0)
		usb_reset_endpoint(us->pusb_dev, endp);

	usb_stor_dbg(us, "result = %d\n", result);
	return result;
}
EXPORT_SYMBOL_GPL(usb_stor_clear_halt);


/*
 * Interpret the results of a URB transfer
 *
 * This function prints appropriate debugging messages, clears halts on
 * non-control endpoints, and translates the status to the corresponding
 * USB_STOR_XFER_xxx return code.
 */
static int interpret_urb_result(struct us_data *us, unsigned int pipe,
		unsigned int length, int result, unsigned int partial)
{
	usb_stor_dbg(us, "Status code %d; transferred %u/%u\n",
		     result, partial, length);
	switch (result) {

	/* no error code; did we send all the data? */
	case 0:
		if (partial != length) {
			usb_stor_dbg(us, "-- short transfer\n");
			return USB_STOR_XFER_SHORT;
		}

		usb_stor_dbg(us, "-- transfer complete\n");
		return USB_STOR_XFER_GOOD;

	/* stalled */
	case -EPIPE:
		/*
		 * for control endpoints, (used by CB[I]) a stall indicates
		 * a failed command
		 */
		if (usb_pipecontrol(pipe)) {
			usb_stor_dbg(us, "-- stall on control pipe\n");
			return USB_STOR_XFER_STALLED;
		}

		/* for other sorts of endpoint, clear the stall */
		usb_stor_dbg(us, "clearing endpoint halt for pipe 0x%x\n",
			     pipe);
		if (usb_stor_clear_halt(us, pipe) < 0)
			return USB_STOR_XFER_ERROR;
		return USB_STOR_XFER_STALLED;

	/* babble - the device tried to send more than we wanted to read */
	case -EOVERFLOW:
		usb_stor_dbg(us, "-- babble\n");
		return USB_STOR_XFER_LONG;

	/* the transfer was cancelled by abort, disconnect, or timeout */
	case -ECONNRESET:
		usb_stor_dbg(us, "-- transfer cancelled\n");
		return USB_STOR_XFER_ERROR;

	/* short scatter-gather read transfer */
	case -EREMOTEIO:
		usb_stor_dbg(us, "-- short read transfer\n");
		return USB_STOR_XFER_SHORT;

	/* abort or disconnect in progress */
	case -EIO:
		usb_stor_dbg(us, "-- abort or disconnect in progress\n");
		return USB_STOR_XFER_ERROR;

	/* the catch-all error case */
	default:
		usb_stor_dbg(us, "-- unknown error\n");
		return USB_STOR_XFER_ERROR;
	}
}

/*
 * Transfer one control message, without timeouts, but allowing early
 * termination.  Return codes are USB_STOR_XFER_xxx.
 */
int usb_stor_ctrl_transfer(struct us_data *us, unsigned int pipe,
		u8 request, u8 requesttype, u16 value, u16 index,
		void *data, u16 size)
{
	int result;

	usb_stor_dbg(us, "rq=%02x rqtype=%02x value=%04x index=%02x len=%u\n",
		     request, requesttype, value, index, size);

	/* fill in the devrequest structure */
	us->cr->bRequestType = requesttype;
	us->cr->bRequest = request;
	us->cr->wValue = cpu_to_le16(value);
	us->cr->wIndex = cpu_to_le16(index);
	us->cr->wLength = cpu_to_le16(size);

	/* fill and submit the URB */
	usb_fill_control_urb(us->current_urb, us->pusb_dev, pipe, 
			 (unsigned char*) us->cr, data, size, 
			 usb_stor_blocking_completion, NULL);
	result = usb_stor_msg_common(us, 0);

	return interpret_urb_result(us, pipe, size, result,
			us->current_urb->actual_length);
}
EXPORT_SYMBOL_GPL(usb_stor_ctrl_transfer);

/*
 * Receive one interrupt buffer, without timeouts, but allowing early
 * termination.  Return codes are USB_STOR_XFER_xxx.
 *
 * This routine always uses us->recv_intr_pipe as the pipe and
 * us->ep_bInterval as the interrupt interval.
 */
static int usb_stor_intr_transfer(struct us_data *us, void *buf,
				  unsigned int length)
{
	int result;
	unsigned int pipe = us->recv_intr_pipe;
	unsigned int maxp;

	usb_stor_dbg(us, "xfer %u bytes\n", length);

	/* calculate the max packet size */
	maxp = usb_maxpacket(us->pusb_dev, pipe, usb_pipeout(pipe));
	if (maxp > length)
		maxp = length;

	/* fill and submit the URB */
	usb_fill_int_urb(us->current_urb, us->pusb_dev, pipe, buf,
			maxp, usb_stor_blocking_completion, NULL,
			us->ep_bInterval);
	result = usb_stor_msg_common(us, 0);

	return interpret_urb_result(us, pipe, length, result,
			us->current_urb->actual_length);
}

/*
 * Transfer one buffer via bulk pipe, without timeouts, but allowing early
 * termination.  Return codes are USB_STOR_XFER_xxx.  If the bulk pipe
 * stalls during the transfer, the halt is automatically cleared.
 */
int usb_stor_bulk_transfer_buf(struct us_data *us, unsigned int pipe,
	void *buf, unsigned int length, unsigned int *act_len)
{
	int result;

	usb_stor_dbg(us, "xfer %u bytes\n", length);

	/* fill and submit the URB */
	usb_fill_bulk_urb(us->current_urb, us->pusb_dev, pipe, buf, length,
		      usb_stor_blocking_completion, NULL);
	result = usb_stor_msg_common(us, 0);

	/* store the actual length of the data transferred */
	if (act_len)
		*act_len = us->current_urb->actual_length;
	return interpret_urb_result(us, pipe, length, result, 
			us->current_urb->actual_length);
}
EXPORT_SYMBOL_GPL(usb_stor_bulk_transfer_buf);

/*
 * Transfer a scatter-gather list via bulk transfer
 *
 * This function does basically the same thing as usb_stor_bulk_transfer_buf()
 * above, but it uses the usbcore scatter-gather library.
 */
static int usb_stor_bulk_transfer_sglist(struct us_data *us, unsigned int pipe,
		struct scatterlist *sg, int num_sg, unsigned int length,
		unsigned int *act_len)
{
	int result;

	/* don't submit s-g requests during abort processing */
	if (test_bit(US_FLIDX_ABORTING, &us->dflags))
		return USB_STOR_XFER_ERROR;

	/* initialize the scatter-gather request block */
	usb_stor_dbg(us, "xfer %u bytes, %d entries\n", length, num_sg);
	result = usb_sg_init(&us->current_sg, us->pusb_dev, pipe, 0,
			sg, num_sg, length, GFP_NOIO);
	if (result) {
		usb_stor_dbg(us, "usb_sg_init returned %d\n", result);
		return USB_STOR_XFER_ERROR;
	}

	/*
	 * since the block has been initialized successfully, it's now
	 * okay to cancel it
	 */
	set_bit(US_FLIDX_SG_ACTIVE, &us->dflags);

	/* did an abort occur during the submission? */
	if (test_bit(US_FLIDX_ABORTING, &us->dflags)) {

		/* cancel the request, if it hasn't been cancelled already */
		if (test_and_clear_bit(US_FLIDX_SG_ACTIVE, &us->dflags)) {
			usb_stor_dbg(us, "-- cancelling sg request\n");
			usb_sg_cancel(&us->current_sg);
		}
	}

	/* wait for the completion of the transfer */
	usb_sg_wait(&us->current_sg);
	clear_bit(US_FLIDX_SG_ACTIVE, &us->dflags);

	result = us->current_sg.status;
	if (act_len)
		*act_len = us->current_sg.bytes;
	return interpret_urb_result(us, pipe, length, result,
			us->current_sg.bytes);
}

/*
 * Common used function. Transfer a complete command
 * via usb_stor_bulk_transfer_sglist() above. Set cmnd resid
 */
int usb_stor_bulk_srb(struct us_data* us, unsigned int pipe,
		      struct scsi_cmnd* srb)
{
	unsigned int partial;
	int result;

#ifdef CONFIG_MEDIATEK_SOLUTION
	usb_boost();
#endif
	result = usb_stor_bulk_transfer_sglist(us, pipe, scsi_sglist(srb),
				      scsi_sg_count(srb), scsi_bufflen(srb),
				      &partial);

	scsi_set_resid(srb, scsi_bufflen(srb) - partial);
	return result;
}
EXPORT_SYMBOL_GPL(usb_stor_bulk_srb);

/*
 * Transfer an entire SCSI command's worth of data payload over the bulk
 * pipe.
 *
 * Note that this uses usb_stor_bulk_transfer_buf() and
 * usb_stor_bulk_transfer_sglist() to achieve its goals --
 * this function simply determines whether we're going to use
 * scatter-gather or not, and acts appropriately.
 */
int usb_stor_bulk_transfer_sg(struct us_data* us, unsigned int pipe,
		void *buf, unsigned int length_left, int use_sg, int *residual)
{
	int result;
	unsigned int partial;

	/* are we scatter-gathering? */
	if (use_sg) {
		/* use the usb core scatter-gather primitives */
		result = usb_stor_bulk_transfer_sglist(us, pipe,
				(struct scatterlist *) buf, use_sg,
				length_left, &partial);
		length_left -= partial;
	} else {
		/* no scatter-gather, just make the request */
		result = usb_stor_bulk_transfer_buf(us, pipe, buf, 
				length_left, &partial);
		length_left -= partial;
	}

	/* store the residual and return the error code */
	if (residual)
		*residual = length_left;
	return result;
}
EXPORT_SYMBOL_GPL(usb_stor_bulk_transfer_sg);

/***********************************************************************
 * Transport routines
 ***********************************************************************/

/*
 * There are so many devices that report the capacity incorrectly,
 * this routine was written to counteract some of the resulting
 * problems.
 */
static void last_sector_ha|HR=wR=ktw<vnt us