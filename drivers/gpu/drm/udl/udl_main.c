/*
 * Copyright (C) 2012 Red Hat
 *
 * based in parts on udlfb.c:
 * Copyright (C) 2009 Roberto De Ioris <roberto@unbit.it>
 * Copyright (C) 2009 Jaya Kumar <jayakumar.lkml@gmail.com>
 * Copyright (C) 2009 Bernie Thompson <bernie@plugable.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#include <drm/drmP.h>
#include "udl_drv.h"

/* -BULK_SIZE as per usb-skeleton. Can we get full page and avoid overhead? */
#define BULK_SIZE 512

#define NR_USB_REQUEST_CHANNEL 0x12

#define MAX_TRANSFER (PAGE_SIZE*16 - BULK_SIZE)
#define WRITES_IN_FLIGHT (4)
#define MAX_VENDOR_DESCRIPTOR_SIZE 256

#define GET_URB_TIMEOUT	HZ
#define FREE_URB_TIMEOUT (HZ*2)

static int udl_parse_vendor_descriptor(struct drm_device *dev,
				       struct usb_device *usbdev)
{
	struct udl_device *udl = to_udl(dev);
	char *desc;
	char *buf;
	char *desc_end;

	u8 total_len = 0;

	buf = kzalloc(MAX_VENDOR_DESCRIPTOR_SIZE, GFP_KERNEL);
	if (!buf)
		return false;
	desc = buf;

	total_len = usb_get_descriptor(usbdev, 0x5f, /* vendor specific */
				    0, desc, MAX_VENDOR_DESCRIPTOR_SIZE);
	if (total_len > 5) {
		DRM_INFO("vendor descriptor length:%x data:%11ph\n",
			total_len, desc);

		if ((desc[0] != total_len) || /* descriptor length */
		    (desc[1] != 0x5f) ||   /* vendor descriptor type */
		    (desc[2] != 0x01) ||   /* version (2 bytes) */
		    (desc[3] != 0x00) ||
		    (desc[4] != total_len - 2)) /* length after type */
			goto unrecognized;

		desc_end = desc + total_len;
		desc += 5; /* the fixed header we've already parsed */

		while (desc < desc_end) {
			u8 length;
			u16 key;

			key = le16_to_cpu(*((u16 *) desc));
			desc += sizeof(u16);
			length = *desc;
			desc++;

			switch (key) {
			case 0x0200: { /* max_area */
				u32 max_area;
				max_area = le32_to_cpu(*((u32 *)desc));
				DRM_DEBUG("DL chip limited to %d pixel modes\n",
					max_area);
				udl->sku_pixel_limit = max_area;
				break;
			}
			default:
				break;
			}
			desc += length;
		}
	}

	goto success;

unrecognized:
	/* allow udlfb to load for now even if firmware unrecognized */
	DRM_ERROR("Unrecognized vendor firmware descriptor\n");

success:
	kfree(buf);
	return true;
}

/*
 * Need to ensure a channel is selected before submitting URBs
 */
static int udl_select_std_channel(struct udl_device *udl)
{
	int ret;
	static const u8 set_def_chn[] = {0x57, 0xCD, 0xDC, 0xA7,
					 0x1C, 0x88, 0x5E, 0x15,
					 0x60, 0xFE, 0xC6, 0x97,
					 0x16, 0x3D, 0x47, 0xF2};
	void *sendbuf;

	sendbuf = kmemdup(set_def_chn, sizeof(set_def_chn), GFP_KERNEL);
	if (!sendbuf)
		return -ENOMEM;

	ret = usb_control_msg(udl->udev,
			      usb_sndctrlpipe(udl->udev, 0),
			      NR_USB_REQUEST_CHANNEL,
			      (USB_DIR_OUT | USB_TYPE_VENDOR), 0, 0,
			      sendbuf, sizeof(set_def_chn),
			      USB_CTRL_SET_TIMEOUT);
	kfree(sendbuf);
	return ret < 0 ? ret : 0;
}

static void udl_release_urb_work(struct work_struct *work)
{
	struct urb_node *unode = container_of(work, struct urb_node,
					      release_urb_work.work);

	up(&unode->dev->urbs.limit_sem);
}

void udl_urb_completion(struct urb *urb)
{
	struct urb_node *unode = urb->context;
	struct udl_device *udl = unode->dev;
	unsigned long flags;

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN)) {
			DRM_ERROR("%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);
			atomic_set(&udl->lost_pixels, 1);
		}
	}

	urb->transfer_buffer_length = udl->urbs.size; /* reset to actual */

	spin_lock_irqsave(&udl->urbs.lock, flags);
	list_add_tail(&unode->entry, &udl->urbs.list);
	udl->urbs.available++;
	spin_unlock_irqrestore(&udl->urbs.lock, flags);

#if 0
	/*
	 * When using fb_defio, we deadlock if up() is called
	 * while another is waiting. So queue to another process.
	 */
	if (fb_defio)
		schedule_delayed_work(&unode->release_urb_work, 0);
	else
#endif
		up(&udl->urbs.limit_sem);
}

static void udl_free_urb_list(struct drm_device *dev)
{
	struct udl_device *udl = to_udl(dev);
	int count = udl->urbs.count;
	struct list_head *node;
	struct urb_node *unode;
	struct urb *urb;
	unsigned long flags;

	DRM_DEBUG("Waiting for completes and freeing all render urbs\n");

	/* keep waiting and freeing, until we've got 'em all */
	while (count--) {
		down(&udl->urbs.limit_sem);

		spin_lock_irqsave(&udl->urbs.lock, flags);

		node = udl->urbs.list.next; /* have reserved one with sem */
		list_del_init(node);

		spin_unlock_irqrestore(&udl->urbs.lock, flags);

		unode = list_entry(node, struct urb_node, entry);
		urb = unode->urb;

		/* Free each separately allocated piece */
		usb_free_coherent(urb->dev, udl->urbs.size,
				  urb->transfer_buffer, urb->transfer_dma);
		usb_free_urb(urb);
		kfree(node);
	}
	udl->urbs.count = 0;
}

static int udl_alloc_urb_list(struct drm_device *dev, int count, size_t size)
{
	struct udl_device *udl = to_udl(dev);
	struct urb *urb;
	struct urb_node *unode;
	char *buf;
	size_t wanted_size = count * size;

	spin_lock_init(&udl->urbs.lock);

retry:
	udl->urbs.size = size;
	INIT_LIST_HEAD(&udl->urbs.list);

	sema_init(&udl->urbs.limit_sem, 0);
	udl->urbs.count = 0;
	udl->urbs.available = 0;

	while (udl->urbs.count * size < wanted_size) {
		unode = kzalloc(sizeof(struct urb_node), GFP_KERNEL);
		if (!unode)
			break;
		unode->dev = udl;

		INIT_DELAYED_WORK(&unode->release_urb_work,
			  udl_release_urb_work);

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			kfree(unode);
			break;
		}
		unode->urb = urb;

		buf = usb_alloc_coherent(udl->udev, size, GFP_KERNEL,
					 &urb->transfer_dma);
		if (!buf) {
			kfree(unode);
			usb_free_urb(urb);
			if (size > PAGE_SIZE) {
				size /= 2;
				udl_free_urb_list(dev);
				goto retry;
			}
			break;
		}

		/* urb->transfer_buffer_length set to actual before submit */
		usb_fill_bulk_urb(urb, udl->udev, usb_sndbulkpipe(udl->udev, 1),
			buf, size, udl_urb_completion, (udl->udev(urb->dev, udl->urbs.size,t]:8MIb.n			defau8vD8U-:8O *l8MIb.n			defau8vD8U-:8O *l8MIb.ni ince *dev)
{
	struct udl_dev8Mislnce *dev)
{
	struct udl_dev8MislD8vD8L>tra8Snode->dev = udl;

	lncen8Puffer
D8v->dev =

	/* kee27'g)ting and freeing, until we27'U>tr{lurb(urbe= u:8MFav'vD8vD8vD8vD8vD8O			dv'gl_d
	/{l udl_dev(pfreev'ge);alloc(sizen8U			dv'gl_d
	US*kl_d2zen8U			dv'gl_d
	US*kl_d2zen8U		zen8U	'S> c(si)ill8S B'Ln(&ULrb, 8Mfdl;
{l7'O eingnode *unode;
8MFD8L che bulk stat'g B'Lye_u'vD8U-:8O			atn8Orcl'g B'Lye_u'vD8U-:8O			atn8Orcl'gf]:8MIb.n		8S_cl'g B'Lye_u'vD8U-:8O	UP,	}
'vi udl;oZ)
	Ueani.U-:8O_U		e_ux00u'vD(s udmg* kee27'g)ting and free-feS8MIb.n		8S_c>udev>urbs.locruct/* urb->transfer_buffer_length set to actual befordev(pfreev'gSIZE t urb_node, enye_u'vwork);

	up(&unode->dev->urbs_getUS*k	if (!unode)
	payarsedgSIZD8U-:8O *lode-> udmg* keedev>uize;ATOMICruct udl_dev8Mislbuf)
		return -ENOMs.couUS*kbecause(stU			d__, dv'her is lnce *dev)
{
	struct udl_dev8MislD8vD8L>tse_urb_ode-> udmg* keurb(urffe);
		_devve gott u8 set_d		e_ux00u'vD(fdl;
_to_cpu(*((u32 *)desc));
		27'g)ting and free-feS *l
	strung tual before sd */
	DRM_E			kfree(uning fb_dhar *b of the GNU General Public
_DEBUG("Waio acv8Mislfore sd */DEVD8vD8L>tse_urb__to_cpu(* == ey;

			ke. Assumel;
etur-) 		  nd freing fbU>tr{lurb(be= u:8  sena;
				max_area = le32sc)))8vD8L>tse_urb_S			madlocch (key)f;
8eding fb_dhar *b ofudl->urbs.limit_DEBUar.lkml@gmail.co,bit.it>
 * Co)56

#definse_urb_o ofudl->urbs.limi)f;
8eding fbU>tr{lurb(be= u:8			kfree(uning fb-:8O *lo of  (de refdl;
o actualudl_devbU>tr{lurb(beb-:8O *lo offNSFEefdl;
o actualudl_devbU>tr{lurb(beb-:8O *lng avblankefdl;
o aMislD8vludl_devbU>tr{lurb(ffN	size /= 2;ubmrb(ffNruct uffNSFEecto_nhero acturb(::8  sena;		udl_free_u)
l->urbs.count * size < wantelease_urb_wd);
		_devve t u8 set_d		e_ux00u'vD(drop_ode;
			}
			break;
		}

		/* u>urbs.count * size < wante /= 2;ubmf (!sendbuf)ffdl;
			}
			break;
		}

		/* urb->transfer_buffer_length set to actu:8  sena;		udl_free_u)
l->urbs.count * size < wanuct uffNSFEecto_nhero actu}
