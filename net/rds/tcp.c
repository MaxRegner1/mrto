/*
 * Copyright (c) 2006 Oracle.  All rights reserved.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * OpenIB.org BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/in.h>
#include <linux/module.h>
#include <net/tcp.h>
#include <net/net_namespace.h>
#include <net/netns/generic.h>

#include "rds.h"
#include "tcp.h"

/* only for info exporting */
static DEFINE_SPINLOCK(rds_tcp_tc_list_lock);
static LIST_HEAD(rds_tcp_tc_list);
static unsigned int rds_tcp_tc_count;

/* Track rds_tcp_connection structs so they can be cleaned up */
static DEFINE_SPINLOCK(rds_tcp_conn_lock);
static LIST_HEAD(rds_tcp_conn_list);

static struct kmem_cache *rds_tcp_conn_slab;

static int rds_tcp_skbuf_handler(struct ctl_table *ctl, int write,
				 void __user *buffer, size_t *lenp,
				 loff_t *fpos);

static int rds_tcp_min_sndbuf = SOCK_MIN_SNDBUF;
static int rds_tcp_min_rcvbuf = SOCK_MIN_RCVBUF;

static struct ctl_table rds_tcp_sysctl_table[] = {
#define	RDS_TCP_SNDBUF	0
	{
		.procname       = "rds_tcp_sndbuf",
		/* data is per-net pointer */
		.maxlen         = sizeof(int),
		.mode           = 0644,
		.proc_handler   = rds_tcp_skbuf_handler,
		.extrc