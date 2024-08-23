/*******************************************************************************
 *
 * This file contains the Linux/SCSI LLD virtual SCSI initiator driver
 * for emulated SAS initiator ports
 *
 * © Copyright 2011-2013 Datera, Inc.
 *
 * Licensed to the Linux Foundation under the General Public License (GPL) version 2.
 *
 * Author: Nicholas A. Bellinger <nab@risingtidesystems.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 ****************************************************************************/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/configfs.h>
#include <scsi/scsi.h>
#include <scsi/scsi_tcq.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_cmnd.h>

#include <target/target_core_base.h>
#include <target/target_core_fabric.h>

#include "tcm_loop.h"

#define to_tcm_loop_hba(hba)	container_of(hba, struct tcm_loop_hba, dev)

static struct workqueue_struct *tcm_loop_workqueue;
static struct kmem_cache *tcm_loop_cmd_cache;

static int tcm_loop_hba_no_cnt;

static int tcm_loop_queue_status(struct se_cmd *se_cmd);

/*
 * Called from struct target_core_fabric_ops->check_stop_free()
 */
static int tcm_loop_check_stop_free(struct se_cmd *se_cmd)
{
	return transport_generic_free_cmd(se_cmd, 0);
}

static void tcm_loop_release_cmd(struct se_cmd *se_cmd)
{
	struct tcm_loop_cmd *tl_cmd = container_of(se_cmd,
				struct tcm_loop_cmd, tl_se_cmd);

	kmem_cache_free(tcm_loop_cmd_cache, tl_cmd);
}

static int tcm_loop_show_info(struct seq_file *m, struct Scsi_Host *host)
{
	seq_printf(m, "tcm_loop_proc_info()\n");
	return 0;
}

static int tcm_loop_driver_probe(struct device *);
static int tcm_loop_driver_remove(struct device *);

static int pseudo_lld_bus_match(struct device *dev,
				struct device_driver *dev_driver)
{
	return 1;
}

static struct bus_type tcm_loop_lld_bus = {
	.name			= "tcm_loop_bus",
	.match			= pseudo_lld_bus_match,
	.probe			= tcm_loop_driver_probe,
	.remove			= tcm_loop_driver_remove,
};

static struct device_driver tcm_loop_driverfs = {
	.name			= "tcm_loop",
	.bus			= &tcm_loop_lld_bus,
};
/*
 * Used with root_device_register() in tcm_loop_alloc_core_bus() below
 */
static struct device *tcm_loop_primary;

static void tcm_loop_submission_work(struct work_struct *work)
{
	struct tcm_loop_cmd *tl_cmd =
		container_of(work, struct tcm_loop_cmd, work);
	struct se_cmd *se_cmd = &tl_cmd->tl_se_cmd;
	struct scsi_cmnd *sc = tl_cmd->sc;
	struct tcm_loop_nexus *tl_nexus;
	struct tcm_loop_hba *tl_hba;
	struct tcm_loop_tpg *tl_tpg;
	struct scatterlist *sgl_bidi = NULL;
	u32 sgl_bidi_count = 0, transfer_length;
	int rc;

	tl_hba = *(struct tcm_loop_hba **)shost_priv(sc->device->host);
	tl_tpg = &tl_hba->tl_hba_tpgs[sc->device->id];

	/*
	 * Ensure that this tl_tpg reference from the incoming sc->device->id
	 * has already been configured via tcm_loop_make_naa_tpg().
	 */
	if (!tl_tpg->tl_hba) {
		set_host_byte(sc, DID_NO_CONNECT);
		goto out_done;
	}
	if (tl_tpg->tl_transport_status == TCM_TRANSPORT_OFFLINE) {
		set_host_byte(sc, DID_TRANSPORT_DISRUPTED);
		goto out_done;
	}
	tl_nexus = tl_tpg->tl_nexus;
	if (!tl_nexus) {
		scmd_printk(KERN_ERR, sc, "TCM_Loop I_T Nexus"
				" does not exist\n");
		set_host_byte(sc, DID_ERROR);
		goto out_done;
	}
	if (scsi_bidi_cmnd(sc)) {
		struct scsi_data_buffer *sdb = scsi_in(sc);

		sgl_bidi = sdb->table.sgl;
		sgl_bidi_count = sdb->table.nents;
		se_cmd->se_cmd_flags |= SCF_BIDI;

	}

	transfer_length = scsi_transfer_length(sc);
	if (!scsi_prot_sg_count(sc) &&
	    scsi_get_prot_op(sc) != SCSI_PROT_NORMAL) {
		se_cmd->prot_pto = true;
		/*
		 * loopback transport doesn't support
		 * WRITE_GENERATE, READ_STRIP protection
		 * information operations, go ahead unprotected.
		 */
		transfer_length = scsi_bufflen(sc);
	}

	se_cmd->tag = tl_cmd->sc_cmd_tag;
	rc = target_submit_cmd_map_sgls(se_cmd, tl_nexus->se_sess, sc->cmnd,
			&tl_cmd->tl_sense_buf[0], tl_cmd->sc->device->lun,
			transfer_length, TCM_SIMPLE_TAG,
			sc->sc_data_direction, 0,
			scsi_sglist(sc), scsi_sg_count(sc),
			sgl_bidi, sgl_bidi_count,
			scsi_prot_sglist(sc), scsi_prot_sg_count(sc));
	if (rc < 0) {
		set_host_byte(sc, DID_NO_CONNECT);
		goto out_done;
	}
	return;

out_done:
	kmem_cache_free(tcm_loop_cmd_cache, tl_cmd);
	sc->scsi_done(sc);
	return;
}

/*
 * ->queuecommand can be and usually is called from interrupt context, so
 * defer the actual submission to a workqueue.
 */
static int tcm_loop_queuecommand(struct Scsi_Host *sh, struct scsi_cmnd *sc)
{
	struct tcm_loop_cmd *tl_cmd;

	pr_debug("tcm_loop_queuecommand() %d:%d:%d:%llu got CDB: 0x%02x"
		" scsi_buf_len: %u\n", sc->device->host->host_no,
		sc->device->id, sc->device->channel, sc->device->lun,
		sc->cmnd[0], scsi_bufflen(sc));

	tl_cmd = kmem_cache_zalloc(tcm_loop_cmd_cache, GFP_ATOMIC);
	if (!tl_cmd) {
		pr_err("Unable to allocate struct tcm_loop_cmd\n");
		set_host_byte(sc, DID_ERROR);
		sc->scsi_done(sc);
		return 0;
	}

	tl_cmd->sc = sc;
	tl_cmd->sc_cmd_tag = sc->request->tag;
	INIT_WORK(&tl_cmd->work, tcm_loop_submission_work);
	queue_work(tcm_loop_workqueue, &tl_cmd->work);
	return 0;
}

/*
 * Called from SCSI EH process context to issue a LUN_RESET TMR
 * to struct scsi_device
 */
static int tcm_loop_issue_tmr(struct tcm_loop_tpg *tl_tpg,
			      u64 lun, int task, enum tcm_tmreq_table tmr)
{
	struct 	qucmd->wz2cmd(sC(sC(sC(sC(sC(sC(sC(sC(sC(sC(sC(sC(sC(sC(sC(sC(sC(sC(sC(sC(sC(sC(shoic intzWLHUm tcm_twyLHyLHyLHy to)
{
	structwUR