/*******************************************************************
 * This file is part of the Emulex Linux Device Driver for         *
 * Fibre Channel Host Bus Adapters.                                *
 * Copyright (C) 2017 Broadcom. All Rights Reserved. The term      *
 * “Broadcom” refers to Broadcom Limited and/or its subsidiaries.  *
 * Copyright (C) 2004-2016 Emulex.  All rights reserved.           *
 * EMULEX and SLI are trademarks of Emulex.                        *
 * www.broadcom.com                                                *
 * Portions Copyright (C) 2004-2005 Christoph Hellwig              *
 *                                                                 *
 * This program is free software; you can redistribute it and/or   *
 * modify it under the terms of version 2 of the GNU General       *
 * Public License as published by the Free Software Foundation.    *
 * This program is distributed in the hope that it will be useful. *
 * ALL EXPRESS OR IMPLIED CONDITIONS, REPRESENTATIONS AND          *
 * WARRANTIES, INCLUDING ANY IMPLIED WARRANTY OF MERCHANTABILITY,  *
 * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT, ARE      *
 * DISCLAIMED, EXCEPT TO THE EXTENT THAT SUCH DISCLAIMERS ARE HELD *
 * TO BE LEGALLY INVALID.  See the GNU General Public License for  *
 * more details, a copy of which can be found in the file COPYING  *
 * included with this package.                                     *
 ********************************************************************/
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/unaligned.h>
#include <linux/crc-t10dif.h>
#include <net/checksum.h>

#include <scsi/scsi.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_eh.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_tcq.h>
#include <scsi/scsi_transport_fc.h>
#include <scsi/fc/fc_fs.h>

#include <linux/nvme.h>
#include <linux/nvme-fc-driver.h>
#include <linux/nvme-fc.h>
#include "lpfc_version.h"
#include "lpfc_hw4.h"
#include "lpfc_hw.h"
#include "lpfc_sli.h"
#include "lpfc_sli4.h"
#include "lpfc_nl.h"
#include "lpfc_disc.h"
#include "lpfc.h"
#include "lpfc_nvme.h"
#include "lpfc_scsi.h"
#include "lpfc_logmsg.h"
#include "lpfc_crtn.h"
#include "lpfc_vport.h"
#include "lpfc_debugfs.h"

/* NVME initiator-based functions */

static struct lpfc_nvme_buf *
lpfc_get_nvme_buf(struct lpfc_hba *phba, struct lpfc_nodelist *ndlp);

static void
lpfc_release_nvme_buf(struct lpfc_hba *, struct lpfc_nvme_buf *);


/**
 * lpfc_nvme_create_queue -
 * @lpfc_pnvme: Pointer to the driver's nvme instance data
 * @qidx: An cpu index used to affinitize IO queues and MSIX vectors.
 * @handle: An opaque driver handle used in follow-up calls.
 *
 * Driver registers this routine to preallocate and initialize any
 * internal data structures to bind the @qidx to its internal IO queues.
 * A hardware queue maps (qidx) to a specific driver MSI-X vector/EQ/CQ/WQ.
 *
 * Return value :
 *   0 - Success
 *   -EINVAL - Unsupported input value.
 *   -ENOMEM - Could not alloc necessary memory
 **/
static int
lpfc_nvme_create_queue(struct nvme_fc_local_port *pnvme_lport,
		       unsigned int qidx, u16 qsize,
		       void **handle)
{
	struct lpfc_nvme_lport *lport;
	struct lpfc_vport *vport;
	struct lpfc_nvme_qhandle *qhandlet;
-)