/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*
 * Author: Xiao Wang <xiao.wang@mediatek.com>
 */
#include <linux/list.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched/clock.h> /* local_clock() */
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/netdevice.h>
#include <linux/random.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/syscore_ops.h>
#if defined(CONFIG_MTK_AEE_FEATURE)
#include <mt-plat/aee.h>
#endif

#include "ccci_config.h"
#include "ccci_core.h"
#include "modem_sys.h"
#include "ccci_bm.h"
#include "ccci_platform.h"
#include "ccci_hif_cldma.h"
#include "md_sys1_platform.h"
#include "cldma_reg.h"
#include "modem_reg_base.h"
#include "ccci_fsm.h"
#include "ccci_port.h"

#if defined(CLDMA_TRACE) || defined(CCCI_SKB_TRACE)
#define CREATE_TRACE_POINTS
#include "modem_cldma_events.h"
#endif
unsigned int trace_sample_time = 200000000;

/* CLDMA setting */
/* always keep this in mind:
 * what if there are more than 1 modems using CLDMA...
 */
/*
 * we use this as rgpd->data_allow_len,
 * so skb length must be >= this size,
 * check ccci_bm.c's skb pool design.
 * channel 3 is for network in normal mode,
 * but for mdlogger_ctrl in exception mode,
 * so choose the max packet size.
 */
#if MD_GENERATION >= (6293)
static int net_rx_queue_buffer_size[CLDMA_RXQ_NUM] = { NET_RX_BUF };
static int normal_rx_queue_buffer_size[CLDMA_RXQ_NUM] = { 0 };
static int net_rx_queue_buffer_number[CLDMA_RXQ_NUM] = { 512 };
static int net_tx_queue_buffer_number[CLDMA_TXQ_NUM] = { 256, 256, 256, 256 };
static int normal_rx_queue_buffer_number[CLDMA_RXQ_NUM] = { 0 };
static int normal_tx_queue_buffer_number[CLDMA_TXQ_NUM] = { 0, 0, 0, 0 };

static int net_rx_queue2ring[CLDMA_RXQ_NUM] = { 0 };
static int net_tx_queue2ring[CLDMA_TXQ_NUM] = { 0, 1, 2, 3 };
static int normal_rx_queue2ring[CLDMA_RXQ_NUM] = { -1 };
static int normal_tx_queue2ring[CLDMA_TXQ_NUM] = { -1, -1, -1, -1 };
static int net_rx_ring2queue[NET_RXQ_NUM] = { 0 };
static int net_tx_ring2queue[NET_TXQ_NUM] = { 0, 1, 2, 3 };
static int normal_rx_ring2queue[NORMAL_RXQ_NUM];
static int normal_tx_ring2queue[NORMAL_TXQ_NUM];

#define NET_TX_QUEUE_MASK 0xF	/* 0, 1, 2, 3 */
#define NET_RX_QUEUE_MASK 0x1	/* 0 */
#define NORMAL_TX_QUEUE_MASK 0x0
#define NORMAL_RX_QUEUE_MASK 0x0
#define NONSTOP_QUEUE_MASK 0xFF /* all stop */
#define NONSTOP_QUEUE_MASK_32 0xFFFFFFFF /* all stop */

#define IS_NET_QUE(md_id, qno) (1)
#define NET_TX_FIRST_QUE	0
#else
static int net_rx_queue_buffer_size[CLDMA_RXQ_NUM] = {
	0, 0, 0, NET_RX_BUF, NET_RX_BUF, NET_RX_BUF, 0, NET_RX_BUF };
static int normal_rx_queue_buffer_size[CLDMA_RXQ_NUM] = {
	SKB_4K, SKB_4K, SKB_4K, SKB_4K, 0, 0, SKB_4K, 0 };
static int net_rx_queue_buffer_number[CLDMA_RXQ_NUM] = {
	0, 0, 0, 256, 256, 64, 0, 16 };
static int net_tx_queue_buffer_number[CLDMA_TXQ_NUM] = {
	0, 0, 0, 256, 256, 64, 0, 16 };
static int normal_rx_queue_buffer_number[CLDMA_RXQ_NUM] = {
	16, 16, 16, 16, 0, 0, 16, 0 };
static int normal_tx_queue_buffer_number[CLDMA_TXQ_NUM] = {
	16, 16, 16, 16, 0, 0, 16, 0 };

static int net_rx_queue2ring[CLDMA_RXQ_NUM] = {
	-1, -1, -1, 0, 1, 2, -1, 3 };
static int net_tx_queue2ring[CLDMA_TXQ_NUM] = {
	-1, -1, -1, 0, 1, 2, -1, 3 };
static int normal_rx_queue2ring[CLDMA_RXQ_NUM] = {
	0, 1, 2, 3, -1, -1, 4, -1 };
static int normal_tx_queue2ring[CLDMA_TXQ_NUM] = {
	0, 1, 2, 3, -1, -1, 4, -1 };
static int net_rx_ring2queue[NET_RXQ_NUM] = {
	3, 4, 5, 7 };
static int net_tx_ring2queue[NET_TXQ_NUM] = {
	3, 4, 5, 7 };
static int normal_rx_ring2queue[NORMAL_RXQ_NUM] = {
	0, 1, 2, 3, 6 };
static int normal_tx_ring2queue[NORMAL_TXQ_NUM] = {
	0, 1, 2, 3, 6 };

#define NET_TX_QUEUE_MASK 0xB8	/* 3, 4, 5, 7 */
#define NET_RX_QUEUE_MASK 0xB8	/* 3, 4, 5, 7 */
#define NORMAL_TX_QUEUE_MASK 0x4F	/* 0, 1, 2, 3, 6 */
#define NORMAL_RX_QUEUE_MASK 0x4F	/* 0, 1, 2, 3, 6 */
/* Rx, for convenience, queue 0,1,2,3 are non-stop */
#define NONSTOP_QUEUE_MASK 0xF0
#define NONSTOP_QUEUE_MASK_32 0xF0F0F0F0

#define NET_TX_FIRST_QUE	3
#define IS_NET_QUE(md_id, qno) \
	((ccci_md_in_ee_dump(md_id) == 0) \
	&& ((1<<qno) & NET_RX_QUEUE_MASK))
#endif

/* mp1 1, mp2 0, ro 1 */
#define UIDMASK 0x80000000

#define TAG "cldma"

static unsigned int g_cd_uid_mask_count;

/*for mt6763 ao_misc_cfg RW type set/clear register issue*/
static inline void cldma_write32_ao_misc(struct md_cd_ctrl *md_ctrl,
	u32 reg, u32 val)
{
#if (MD_GENERATION == 6293)
	u32 reg2, reg2_val;

	if (reg == CLDMA_AP_L2RIMSR0)
		reg2 = CLDMA_AP_L2RIMCR0;
	else if (reg == CLDMA_AP_L2RIMCR0)
		reg2 = CLDMA_AP_L2RIMSR0;
	else
		return;

	reg2_val = cldma_read32(md_ctrl->cldma_ap_ao_base, reg2);
	reg2_val &= ~val;
	cldma_write32(md_ctrl->cldma_ap_ao_base, reg2, reg2_val);
#endif
	cldma_write32(md_ctrl->cldma_ap_ao_base, reg, val);
}

static inline void cldma_tgpd_set_data_ptr(struct cldma_tgpd *tgpd,
	dma_addr_t data_ptr)
{
	unsigned char val = 0;

	cldma_write32(&tgpd->data_buff_bd_ptr, 0,
		(u32)data_ptr);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	val = cldma_read8(&tgpd->msb.msb_byte, 0);
	val &= 0xF0;
	val |= ((data_ptr >> 32) & 0xF);
	cldma_write8(&tgpd->msb.msb_byte, 0, val);
#endif
	CCCI_DEBUG_LOG(MD_SYS1, TAG, "%s:%pa, 0x%x, val=0x%x\n",
		__func__, &data_ptr, tgpd->msb.msb_byte, val);
}

static inline void cldma_tgpd_set_next_ptr(struct cldma_tgpd *tgpd,
	dma_addr_t next_ptr)
{
	unsigned char val = 0;

	cldma_write32(&tgpd->next_gpd_ptr, 0, (u32)next_ptr);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	val = cldma_read8(&tgpd->msb.msb_byte, 0);
	val &= 0x0F;
	val |= (((next_ptr >> 32) & 0xF) << 4);
	cldma_write8(&tgpd->msb.msb_byte, 0, val);
#endif
	CCCI_DEBUG_LOG(MD_SYS1, TAG, "%s:%pa, 0x%x, val=0x%x\n",
		__func__, &next_ptr, tgpd->msb.msb_byte, val);
}

static inline void cldma_rgpd_set_data_ptr(struct cldma_rgpd *rgpd,
	dma_addr_t data_ptr)
{
	unsigned char val = 0;

	cldma_write32(&rgpd->data_buff_bd_ptr, 0, (u32)data_ptr);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	val = cldma_read8(&rgpd->msb.msb_byte, 0);
	val &= 0xF0;
	val |= ((data_ptr >> 32) & 0xF);
	cldma_write8(&rgpd->msb.msb_byte, 0, val);
#endif
	CCCI_DEBUG_LOG(MD_SYS1, TAG, "%s:%pa, 0x%x, val=0x%x\n",
		__func__, &data_ptr, rgpd->msb.msb_byte, val);
}

static inline void cldma_rgpd_set_next_ptr(struct cldma_rgpd *rgpd,
	dma_addr_t next_ptr)
{
	unsigned char val = 0;

	cldma_write32(&rgpd->next_gpd_ptr, 0, (u32)next_ptr);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	val = cldma_read8(&rgpd->msb.msb_byte, 0);
	val &= 0x0F;
	val |= (((next_ptr >> 32) & 0xF) << 4);
	cldma_write8(&rgpd->msb.msb_byte, 0, val);
#endif
	CCCI_DEBUG_LOG(MD_SYS1, TAG, "%s:%pa, 0x%x, val=0x%x\n",
		__func__, &next_ptr, rgpd->msb.msb_byte, val);
}

static inline void cldma_tbd_set_data_ptr(struct cldma_tbd *tbd,
	dma_addr_t data_ptr)
{
	unsigned char val = 0;

	cldma_write32(&tbd->data_buff_ptr, 0, (u32)data_ptr);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	val = cldma_read8(&tbd->msb.msb_byte, 0);
	val &= 0xF0;
	val |= ((data_ptr >> 32) & 0xF);
	cldma_write8(&tbd->msb.msb_byte, 0, val);
#endif
	CCCI_DEBUG_LOG(MD_SYS1, TAG,
	"%s:%pa, 0x%x, val=0x%x\n", __func__,
	&data_ptr, tbd->msb.msb_byte, val);
}

static inline void cldma_tbd_set_next_ptr(struct cldma_tbd *tbd,
	dma_addr_t next_ptr)
{
	unsigned char val = 0;

	cldma_write32(&tbd->next_bd_ptr, 0, (u32)next_ptr);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	val = cldma_read8(&tbd->msb.msb_byte, 0);
	val &= 0x0F;
	val |= (((next_ptr >> 32) & 0xF) << 4);
	cldma_write8(&tbd->msb.msb_byte, 0, val);
#endif
	CCCI_DEBUG_LOG(MD_SYS1, TAG, "%s:%pa, 0x%x, val=0x%x\n",
		__func__, &next_ptr, tbd->msb.msb_byte, val);
}

static inline void cldma_rbd_set_data_ptr(struct cldma_rbd *rbd,
	dma_addr_t data_ptr)
{
	unsigned char val = 0;

	cldma_write32(&rbd->data_buff_ptr, 0, (u32)data_ptr);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	val = cldma_read8(&rbd->msb.msb_byte, 0);
	val &= 0xF0;
	val |= ((data_ptr >> 32) & 0xF);
	cldma_write8(&rbd->msb.msb_byte, 0, val);
#endif
	CCCI_DEBUG_LOG(MD_SYS1, TAG, "%s:%pa, 0x%x, val=0x%x\n",
		__func__, &data_ptr, rbd->msb.msb_byte, val);
}

static inline void cldma_rbd_set_next_ptr(struct cldma_rbd *rbd,
	dma_addr_t next_ptr)
{
	unsigned char val = 0;

	cldma_write32(&rbd->next_bd_ptr, 0, (u32)next_ptr);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	val = cldma_read8(&rbd->msb.msb_byte, 0);
	val &= 0x0F;
	val |= (((next_ptr >> 32) & 0xF) << 4);
	cldma_write8(&rbd->msb.msb_byte, 0, val);
#endif
	CCCI_DEBUG_LOG(MD_SYS1, TAG, "%s:%pa, 0x%x, val=0x%x\n",
		__func__, &next_ptr, rbd->msb.msb_byte, val);
}


static void cldma_dump_gpd_queue(struct md_cd_ctrl *md_ctrl,
	unsigned int qno, unsigned int dir)
{
	unsigned int *tmp;
	struct cldma_request *req = NULL;
#ifdef CLDMA_DUMP_BD
	struct cldma_request *req_bd = NULL;
#endif
	struct cldma_rgpd *rgpd;

	if (dir & 1 << OUT) {
		/* use request's link head to traverse */
		CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
			" dump txq %d, tr_done=%p, tx_xmit=0x%p\n", qno,
			md_ctrl->txq[qno].tr_done->gpd,
			md_ctrl->txq[qno].tx_xmit->gpd);
		list_for_each_entry(req, &md_ctrl->txq[qno].tr_ring->gpd_ring,
			entry) {
			tmp = (unsigned int *)req->gpd;
			CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
				" 0x%p: %X %X %X %X\n", req->gpd,
				*tmp, *(tmp + 1), *(tmp + 2), *(tmp + 3));
#ifdef CLDMA_DUMP_BD
			list_for_each_entry(req_bd, &req->bd, entry) {
				tmp = (unsigned int *)req_bd->gpd;
				CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
					"-0x%p: %X %X %X %X\n", req_bd->gpd,
					*tmp, *(tmp + 1), *(tmp + 2),
					*(tmp + 3));
			}
#endif
		}
	}
	if (dir & 1 << IN) {
		/* use request's link head to traverse */
		/*maybe there is more txq than rxq*/
		if (qno >= CLDMA_RXQ_NUM) {
			CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
				"invalid rxq%d\n", qno);
			return;
		}
		CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
			" dump rxq %d, tr_done=%p, rx_refill=0x%p\n", qno,
			md_ctrl->rxq[qno].tr_done->gpd,
			md_ctrl->rxq[qno].rx_refill->gpd);
		list_for_each_entry(req, &md_ctrl->rxq[qno].tr_ring->gpd_ring,
			entry) {
			tmp = (unsigned int *)req->gpd;
			CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
				" 0x%p/0x%p: %X %X %X %X\n", req->gpd, req->skb,
				*tmp, *(tmp + 1), *(tmp + 2), *(tmp + 3));
			rgpd = (struct cldma_rgpd *)req->gpd;
			if ((cldma_read8(&rgpd->gpd_flags, 0) & 0x1) == 0
				&& req->skb) {
				tmp = (unsigned int *)req->skb->data;
				CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
					" 0x%p: %X %X %X %X\n", req->skb->data,
					*tmp, *(tmp + 1), *(tmp + 2),
					*(tmp + 3));
			}
		}
	}
}

static void cldma_dump_all_tx_gpd(struct md_cd_ctrl *md_ctrl)
{
	int i;

	for (i = 0; i < QUEUE_LEN(md_ctrl->txq); i++)
		cldma_dump_gpd_queue(md_ctrl, i, 1 << OUT);
}

#if TRAFFIC_MONITOR_INTERVAL
void md_cd_traffic_monitor_func(unsigned long data)
{
	int i;
	struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)data;
	struct ccci_hif_traffic *tinfo = &md_ctrl->traffic_info;
	unsigned long q_rx_rem_nsec[CLDMA_RXQ_NUM] = {0};
	unsigned long isr_rem_nsec;

	CCCI_ERROR_LOG(-1, TAG,
		"[%s] g_cd_uid_mask_count = %u\n",
		__func__, g_cd_uid_mask_count);

	ccci_port_dump_status(md_ctrl->md_id);
	CCCI_REPEAT_LOG(md_ctrl->md_id, TAG,
		"Tx active %d\n", md_ctrl->txq_active);
	for (i = 0; i < QUEUE_LEN(md_ctrl->txq); i++) {
		if (md_ctrl->txq[i].busy_count != 0) {
			CCCI_REPEAT_LOG(md_ctrl->md_id, TAG,
				"Txq%d busy count %d\n", i,
				md_ctrl->txq[i].busy_count);
			md_ctrl->txq[i].busy_count = 0;
		}
		CCCI_REPEAT_LOG(md_ctrl->md_id, TAG,
			"Tx:%d-%d\n",
			md_ctrl->tx_pre_traffic_monitor[i],
			md_ctrl->tx_traffic_monitor[i]);
	}

	i = NET_TX_FIRST_QUE;
	if (i + 3 < CLDMA_TXQ_NUM)
		CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
			"net Txq%d-%d(status=0x%x):%d-%d, %d-%d, %d-%d, %d-%d\n",
			i, i + 3, cldma_read32(md_ctrl->cldma_ap_pdn_base,
			CLDMA_AP_UL_STATUS),
			md_ctrl->tx_pre_traffic_monitor[i],
			md_ctrl->tx_traffic_monitor[i],
			md_ctrl->tx_pre_traffic_monitor[i + 1],
			md_ctrl->tx_traffic_monitor[i + 1],
			md_ctrl->tx_pre_traffic_monitor[i + 2],
			md_ctrl->tx_traffic_monitor[i + 2],
			md_ctrl->tx_pre_traffic_monitor[i + 3],
			md_ctrl->tx_traffic_monitor[i + 3]);

	isr_rem_nsec = (tinfo->latest_isr_time == 0 ? 0
		: do_div(tinfo->latest_isr_time, 1000000000));

	CCCI_REPEAT_LOG(md_ctrl->md_id, TAG,
		"Rx ISR %lu.%06lu, active %d\n",
		(unsigned long)tinfo->latest_isr_time,
		isr_rem_nsec / 1000, md_ctrl->rxq_active);

	for (i = 0; i < QUEUE_LEN(md_ctrl->rxq); i++) {
		q_rx_rem_nsec[i] =
			(tinfo->latest_q_rx_isr_time[i] == 0 ?
			0 :
			do_div(tinfo->latest_q_rx_isr_time[i], 1000000000));
		CCCI_REPEAT_LOG(md_ctrl->md_id, TAG,
			"RX:%lu.%06lu, %d\n",
			(unsigned long)tinfo->latest_q_rx_isr_time[i],
			q_rx_rem_nsec[i] / 1000,
			md_ctrl->rx_traffic_monitor[i]);
	}

#ifdef ENABLE_CLDMA_TIMER
	CCCI_REPEAT_LOG(md_ctrl->md_id, TAG,
	"traffic(tx_timer): [3]%llu %llu, [4]%llu %llu, [5]%llu %llu\n",
	md_ctrl->txq[3].timeout_start, md_ctrl->txq[3].timeout_end,
	md_ctrl->txq[4].timeout_start, md_ctrl->txq[4].timeout_end,
	md_ctrl->txq[5].timeout_start, md_ctrl->txq[5].timeout_end);

	CCCI_REPEAT_LOG(md_ctrl->md_id, TAG,
	"traffic(tx_done_timer): CLDMA_AP_L2TIMR0=0x%x   [3]%d %llu, [4]%d %llu, [5]%d %llu\n",
	cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TIMR0),
	md_ctrl->tx_done_last_count[3],
	md_ctrl->tx_done_last_start_time[3],
	md_ctrl->tx_done_last_count[4],
	md_ctrl->tx_done_last_start_time[4],
	md_ctrl->tx_done_last_count[5],
	md_ctrl->tx_done_last_start_time[5]);
#endif
	ccci_channel_dump_packet_counter(md_ctrl->md_id, tinfo);
	ccci_dump_skb_pool_usage(md_ctrl->md_id);

	if ((jiffies - md_ctrl->traffic_stamp) / HZ <=
		TRAFFIC_MONITOR_INTERVAL * 2)
		mod_timer(&md_ctrl->traffic_monitor,
			jiffies + TRAFFIC_MONITOR_INTERVAL * HZ);
}
#endif

static void cldma_dump_packet_history(struct md_cd_ctrl *md_ctrl)
{
	int i;

	for (i = 0; i < QUEUE_LEN(md_ctrl->txq); i++) {
		CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
			"Current txq%d pos: tr_done=%x, tx_xmit=%x\n", i,
			(unsigned int)md_ctrl->txq[i].tr_done->gpd_addr,
			(unsigned int)md_ctrl->txq[i].tx_xmit->gpd_addr);
	}
	for (i = 0; i < QUEUE_LEN(md_ctrl->rxq); i++) {
		CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
			"Current rxq%d pos: tr_done=%x, rx_refill=%x\n", i,
			(unsigned int)md_ctrl->rxq[i].tr_done->gpd_addr,
			(unsigned int)md_ctrl->rxq[i].rx_refill->gpd_addr);
	}
	ccci_md_dump_log_history(md_ctrl->md_id,
		&md_ctrl->traffic_info, 1, QUEUE_LEN(md_ctrl->txq),
		QUEUE_LEN(md_ctrl->rxq));
}

static void cldma_dump_queue_history(struct md_cd_ctrl *md_ctrl,
	unsigned int qno)
{
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
		"Current txq%d pos: tr_done=%x, tx_xmit=%x\n", qno,
		(unsigned int)md_ctrl->txq[qno].tr_done->gpd_addr,
		(unsigned int)md_ctrl->txq[qno].tx_xmit->gpd_addr);

	if (qno >= CLDMA_RXQ_NUM) {
		CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
			"invalid rxq%d\n", qno);
		return;
	}

	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
		"Current rxq%d pos: tr_done=%x, rx_refill=%x\n", qno,
		(unsigned int)md_ctrl->rxq[qno].tr_done->gpd_addr,
		(unsigned int)md_ctrl->rxq[qno].rx_refill->gpd_addr);
	ccci_md_dump_log_history(md_ctrl->md_id,
		&md_ctrl->traffic_info, 0, qno, qno);
}

static int cldma_queue_broadcast_state(struct md_cd_ctrl *md_ctrl,
	enum HIF_STATE state, enum DIRECTION dir, int index)
{
	ccci_port_queue_status_notify(md_ctrl->md_id,
		md_ctrl->hif_id, index, dir, state);
	return 0;
}

#ifdef ENABLE_CLDMA_TIMER
static void cldma_timeout_timer_func(unsigned long data)
{
	struct md_cd_queue *queue = (struct md_cd_queue *)data;
	struct ccci_modem *md = queue->modem;
	struct ccci_port *port;
	unsigned long long port_full = 0, i;

	if (MD_IN_DEBUG(md))
		return;

	ccci_md_dump_port_status(md);
	md_cd_traffic_monitor_func((unsigned long)md);
	ccci_hif_dump_status(CLDMA_HIF_ID, DUMP_FLAG_CLDMA, 1 << queue->index);

	CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
		"CLDMA no response, force assert md by CCIF_INTERRUPT\n");
	md->ops->force_assert(md, MD_FORCE_ASSERT_BY_MD_NO_RESPONSE);
}
#endif

#if MD_GENERATION == (6293)
/*
 * AP_L2RISAR0 register is different from others.
 * its valid bit is 0,8,16,24
 * So we have to gather/scatter it to match other registers
 */
static inline u32 cldma_reg_bit_gather(u32 reg_s)
{
	u32 reg_g = 0;
	u32 i = 0;

	while (reg_s) {
		reg_g |= ((reg_s & 0x1) << i);
		reg_s = reg_s >> 8;
		i++;
	}

	return reg_g;
}
static inline u32 cldma_reg_bit_scatter(u32 reg_g)
{
	u32 reg_s = 0;
	u32 i = 0;

	while (reg_g && i < 4) {
		reg_s |= ((reg_g & 0x1) << (8 * i));
		reg_g = reg_g >> 1;
		i++;
	}

	return reg_s;
}
#endif

/* may be called from workqueue or NAPI or tasklet (queue0) context,
 * only NAPI and tasklet with blocking=false
 */
static int cldma_gpd_rx_collect(struct md_cd_queue *queue,
	int budget, int blocking)
{
	struct md_cd_ctrl *md_ctrl =
		(struct md_cd_ctrl *)ccci_hif_get_by_id(queue->hif_id);

	struct cldma_request *req;
	struct cldma_rgpd *rgpd;
	struct ccci_header ccci_h;
#if MD_GENERATION >= (6293)
	struct lhif_header lhif_h;
#endif
	struct sk_buff *skb = NULL;
	struct sk_buff *new_skb = NULL;
	int ret = 0, count = 0, rxbytes = 0;
	int over_budget = 0, skb_handled = 0, retry = 0;
	unsigned long long skb_bytes = 0;
	unsigned long flags;
	char using_napi = (ccci_md_get_cap_by_id(md_ctrl->md_id) & MODEM_CAP_NAPI);

	unsigned int L2RISAR0 = 0;
	unsigned long time_limit = jiffies + 2;
	unsigned int l2qe_s_offset = CLDMA_RX_QE_OFFSET;

#ifdef CLDMA_TRACE
	unsigned long long port_recv_time = 0;
	unsigned long long skb_alloc_time = 0;
	unsigned long long total_handle_time = 0;
	unsigned long long temp_time = 0;
	unsigned long long total_time = 0;
	unsigned int rx_interal;
	static unsigned long long last_leave_time[CLDMA_RXQ_NUM]
	= { 0 };
	static unsigned int sample_time[CLDMA_RXQ_NUM] = { 0 };
	static unsigned int sample_bytes[CLDMA_RXQ_NUM] = { 0 };

	total_time = sched_clock();
	if (last_leave_time[queue->index] == 0)
		rx_interal = 0;
	else
		rx_interal = total_time - last_leave_time[queue->index];
#endif

again:
	while (1) {
#ifdef CLDMA_TRACE
		total_handle_time = port_recv_time = sched_clock();
#endif
		req = queue->tr_done;
		rgpd = (struct cldma_rgpd *)req->gpd;
		if (!((cldma_read8(&rgpd->gpd_flags, 0) & 0x1) == 0
			&& req->skb)) {
			ret = ALL_CLEAR;

			break;
		}

		new_skb = ccci_alloc_skb(queue->tr_ring->pkt_size, 0, blocking);
		if (unlikely(!new_skb)) {
			CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
				"alloc skb fail on q%d, retry!\n",
				queue->index);
			ret = LOW_MEMORY;
			return ret;
		}

		skb = req->skb;
		/* update skb */
		spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
		if (req->data_buffer_ptr_saved != 0) {
			dma_unmap_single(ccci_md_get_dev_by_id(md_ctrl->md_id),
				req->data_buffer_ptr_saved,
				skb_data_size(skb), DMA_FROM_DEVICE);
			req->data_buffer_ptr_saved = 0;
		}
		spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
		/*init skb struct*/
		skb->len = 0;
		skb_reset_tail_pointer(skb);
		/*set data len*/
		skb_put(skb, rgpd->data_buff_len);
		skb_bytes = skb->len;
#ifdef ENABLE_FAST_HEADER
		if (queue->fast_hdr.gpd_count == 0) {
			ccci_h = *((struct ccci_header *)skb->data);
			queue->fast_hdr =
				*((struct ccci_fast_header *)skb->data);
		} else {
			queue->fast_hdr.seq_num++;
			--queue->fast_hdr.gpd_count;
			if (queue->fast_hdr.has_hdr_room)
				memcpy(skb->data, &queue->fast_hdr,
					sizeof(struct ccci_header));
			else
				memcpy(skb_push(skb,
					sizeof(struct ccci_header)),
					&queue->fast_hdr,
					sizeof(struct ccci_header));
			ccci_h = *((struct ccci_header *)skb->data);
		}
#if MD_GENERATION >= (6293)
#error 6293 should not enable fast header!
#endif
#else
#if MD_GENERATION >= (6293)
		lhif_h = *((struct lhif_header *)skb->data);
		memset(&ccci_h, 0, sizeof(ccci_h));
		memcpy(&ccci_h, &lhif_h, sizeof(lhif_h));
		ccci_h.channel = lhif_h.netif;
#else
		ccci_h = *((struct ccci_header *)skb->data);
#endif
#endif
		/* check wakeup source */
		if (atomic_cmpxchg(&md_ctrl->wakeup_src, 1, 0) == 1) {
			md_ctrl->wakeup_count++;
			CCCI_NOTICE_LOG(md_ctrl->md_id, TAG,
			"CLDMA_MD wakeup source:(%d/%d/%x)(%u)\n",
			queue->index, ccci_h.channel, ccci_h.reserved,
			 md_ctrl->wakeup_count);
		}
		CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
			"recv Rx msg (%x %x %x %x) rxq=%d len=%d\n",
			ccci_h.data[0], ccci_h.data[1],
			*(((u32 *)&ccci_h) + 2),
			ccci_h.reserved, queue->index,
			rgpd->data_buff_len);
		/* upload skb */
		if (using_napi) {
			ccci_md_recv_skb(md_ctrl->md_id,
					md_ctrl->hif_id, skb);
			ret = 0;
		} else {
#ifdef CCCI_SKB_TRACE
			skb->tstamp = sched_clock();
#endif
			ccci_skb_enqueue(&queue->skb_list, skb);
			ret = 0;
		}
#ifdef CLDMA_TRACE
		port_recv_time =
			((skb_alloc_time = sched_clock()) - port_recv_time);
#endif

		if (ret >= 0 || ret == -CCCI_ERR_DROP_PACKET) {
			/* mark cldma_request as available */
			req->skb = NULL;
			cldma_rgpd_set_data_ptr(rgpd, 0);
			/* step forward */
			queue->tr_done =
				cldma_ring_step_forward(queue->tr_ring, req);
			/* update log */
#if TRAFFIC_MONITOR_INTERVAL
			md_ctrl->rx_traffic_monitor[queue->index]++;
#endif
			rxbytes += skb_bytes;
			ccci_md_add_log_history(&md_ctrl->traffic_info, IN,
				(int)queue->index, &ccci_h,
				(ret >= 0 ? 0 : 1));
#if MD_GENERATION <= (6292)
			ccci_channel_update_packet_counter(
				md_ctrl->traffic_info.logic_ch_pkt_cnt,
				&ccci_h);
			ccci_md_check_rx_seq_num(md_ctrl->md_id,
				&md_ctrl->traffic_info, &ccci_h, queue->index);
#endif
			/* refill */
			req = queue->rx_refill;
			rgpd = (struct cldma_rgpd *)req->gpd;
			spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
			req->data_buffer_ptr_saved =
				dma_map_single(
					ccci_md_get_dev_by_id(md_ctrl->md_id),
					new_skb->data,
					skb_data_size(new_skb),
					DMA_FROM_DEVICE);
			if (dma_mapping_error(
				ccci_md_get_dev_by_id(md_ctrl->md_id),
				req->data_buffer_ptr_saved)) {
				CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
					"error dma mapping\n");
				req->data_buffer_ptr_saved = 0;
				spin_unlock_irqrestore(
					&md_ctrl->cldma_timeout_lock,
					flags);
				ccci_free_skb(new_skb);
				wake_up_all(&queue->rx_wq);
				return -1;
			}
			spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock,
				flags);
			cldma_rgpd_set_data_ptr(rgpd,
				req->data_buffer_ptr_saved);
			cldma_write16(&rgpd->data_buff_len, 0, 0);
			/* set HWO, no need to hold ring_lock as no racer */
			cldma_write8(&rgpd->gpd_flags, 0, 0x81);
			/* mark cldma_request as available */
			req->skb = new_skb;
			/* step forward */
			queue->rx_refill =
				cldma_ring_step_forward(queue->tr_ring, req);
			skb_handled = 1;
		} else {
			/* undo skb, as it remains in buffer and
			 * will be handled later
			 */
			CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
				"rxq%d leave skb %p in ring, ret = 0x%x\n",
				queue->index, skb, ret);
			/* no need to retry if port refused to recv */
			if (ret == -CCCI_ERR_PORT_RX_FULL)
				ret = ONCE_MORE;
			else
				ret = ALL_CLEAR; /*maybe never come here*/
			ccci_free_skb(new_skb);
			break;
		}
#ifdef CLDMA_TRACE
		temp_time = sched_clock();
		skb_alloc_time = temp_time - skb_alloc_time;
		total_handle_time = temp_time - total_handle_time;
		trace_cldma_rx(queue->index, 0, count,
			port_recv_time, skb_alloc_time,
			total_handle_time, skb_bytes);
#endif
		/* resume cldma rx if necessary,
		 * avoid cldma rx is inactive for long time
		 */
		spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
		if (!(cldma_read32(md_ctrl->cldma_ap_ao_base,
			CLDMA_AP_SO_STATUS)
			& (1 << queue->index))) {
			cldma_write32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_SO_RESUME_CMD,
				CLDMA_BM_ALL_QUEUE & (1 << queue->index));
			cldma_read32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_SO_RESUME_CMD); /* dummy read */
		}
		spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);

		count++;
		if (count % 8 == 0)
			wake_up_all(&queue->rx_wq);
		/* check budget, only NAPI and queue0 are allowed to
		 * reach budget, as they can be scheduled again
		 */
		if ((count >= budget ||
			time_after_eq(jiffies, time_limit))
			&& !blocking) {
			over_budget = 1;
			ret = ONCE_MORE;
			CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
				"rxq%d over budget or timeout, count = %d\n",
				queue->index, count);
			break;
		}
	}
	wake_up_all(&queue->rx_wq);
	/*
	 * do not use if(count == RING_BUFFER_SIZE) to resume Rx queue.
	 * resume Rx queue every time. we may not handle all RX ring
	 * buffer at one time due to
	 * user can refuse to receive patckets. so when a queue is stopped
	 * after it consumes all
	 * GPD, there is a chance that "count" never reaches ring buffer
	 * size and the queue is stopped
	 * permanentely.
	 */
	spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
	if (md_ctrl->rxq_active & (1 << queue->index)) {
		/* resume Rx queue */
		if (!(cldma_read32(md_ctrl->cldma_ap_ao_base,
			CLDMA_AP_SO_STATUS) & (1 << queue->index))) {
			cldma_write32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_SO_RESUME_CMD,
				CLDMA_BM_ALL_QUEUE & (1 << queue->index));
			cldma_read32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_SO_RESUME_CMD); /* dummy read */
		}
		/* greedy mode */
		L2RISAR0 = cldma_read32(md_ctrl->cldma_ap_pdn_base,
					CLDMA_AP_L2RISAR0);
#if MD_GENERATION == (6293)
		L2RISAR0 = cldma_reg_bit_gather(L2RISAR0);
		l2qe_s_offset = CLDMA_RX_QE_OFFSET * 8;
#endif
		if ((L2RISAR0 & CLDMA_RX_INT_DONE & (1 << queue->index))
			&& !(!blocking && ret == ONCE_MORE))
			retry = 1;
		else
			retry = 0;
		/* where are we going */
		if (retry) {
			/* ACK interrupt */
			cldma_write32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_L2RISAR0,
				((1 << queue->index) << l2qe_s_offset));
			cldma_write32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_L2RISAR0, (1 << queue->index));
			/* clear IP busy register wake up cpu case */
			cldma_write32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_CLDMA_IP_BUSY,
				cldma_read32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_CLDMA_IP_BUSY));
			spin_unlock_irqrestore(
				&md_ctrl->cldma_timeout_lock, flags);
			goto again;
		}
	}
	spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);

#ifdef CLDMA_TRACE
	if (count) {
		last_leave_time[queue->index] = sched_clock();
		total_time = last_leave_time[queue->index] - total_time;
		sample_time[queue->index] += (total_time + rx_interal);
		sample_bytes[queue->index] += rxbytes;
		trace_cldma_rx_done(queue->index, rx_interal, total_time,
			count, rxbytes, 0, ret);
		if (sample_time[queue->index] >= trace_sample_time) {
			trace_cldma_rx_done(queue->index, 0, 0, 0, 0,
				sample_time[queue->index],
				sample_bytes[queue->index]);
			sample_time[queue->index] = 0;
			sample_bytes[queue->index] = 0;
		}
	} else {
		trace_cldma_error(queue->index, -1, 0, __LINE__);
	}
#endif
	return ret;
}

static int cldma_net_rx_push_thread(void *arg)
{
	struct sk_buff *skb = NULL;
	struct md_cd_queue *queue = (struct md_cd_queue *)arg;
	struct md_cd_ctrl *md_ctrl =
		(struct md_cd_ctrl *)ccci_hif_get_by_id(queue->hif_id);
#ifdef CCCI_SKB_TRACE
	struct ccci_per_md *per_md_data =
		ccci_get_per_md_data(md_ctrl->md_id);
#endif
	int count = 0;
	int ret;

	while (!kthread_should_stop()) {
		if (skb_queue_empty(&queue->skb_list.skb_list)) {
			cldma_queue_broadcast_state(md_ctrl, RX_FLUSH,
				IN, queue->index);
			count = 0;
			ret = wait_event_interruptible(queue->rx_wq,
				!skb_queue_empty(&queue->skb_list.skb_list));
			if (ret == -ERESTARTSYS)
				continue;
		}

		skb = ccci_skb_dequeue(&queue->skb_list);
		if (!skb)
			continue;

#ifdef CCCI_SKB_TRACE
		per_md_data->netif_rx_profile[6] = sched_clock();
		if (count > 0)
			skb->tstamp = sched_clock();
#endif
		ccci_md_recv_skb(md_ctrl->md_id, md_ctrl->hif_id, skb);
		count++;
#ifdef CCCI_SKB_TRACE
		per_md_data->netif_rx_profile[6] =
			sched_clock() - per_md_data->netif_rx_profile[6];
		per_md_data->netif_rx_profile[5] = count;
		trace_ccci_skb_rx(per_md_data->netif_rx_profile);
#endif
	}
	return 0;
}

static void cldma_rx_done(struct work_struct *work)
{
	struct md_cd_queue *queue =
		container_of(work, struct md_cd_queue, cldma_rx_work);

	struct md_cd_ctrl *md_ctrl =
		(struct md_cd_ctrl *)ccci_hif_get_by_id(queue->hif_id);
	int ret;

	md_ctrl->traffic_info.latest_q_rx_time[queue->index]
		= local_clock();
	ret =
		queue->tr_ring->handle_rx_done(queue, queue->budget, 1);
	/* enable RX_DONE interrupt */
	cldma_write32_ao_misc(md_ctrl, CLDMA_AP_L2RIMCR0,
		(CLDMA_RX_INT_DONE & (1 << queue->index)) |
		(CLDMA_RX_INT_QUEUE_EMPTY
		& ((1 << queue->index) << CLDMA_RX_QE_OFFSET)));
}

/* this function may be called from both workqueue and ISR (timer) */
static int cldma_gpd_bd_tx_collect(struct md_cd_queue *queue,
	int budget, int blocking)
{
	struct md_cd_ctrl *md_ctrl =
		(struct md_cd_ctrl *)ccci_hif_get_by_id(queue->hif_id);
	unsigned long flags;
	struct cldma_request *req;
	struct cldma_request *req_bd;
	struct cldma_tgpd *tgpd;
	struct cldma_tbd *tbd;
	struct ccci_header *ccci_h;
	int count = 0;
	struct sk_buff *skb_free;
	int need_resume = 0;
	int resume_done = 0;

	while (1) {
		spin_lock_irqsave(&queue->ring_lock, flags);
		req = queue->tr_done;
		tgpd = (struct cldma_tgpd *)req->gpd;
		if (!((tgpd->gpd_flags & 0x1) == 0 && req->skb)) {
			spin_unlock_irqrestore(&queue->ring_lock, flags);
			/* resume channel because cldma HW may stop now*/
			spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
			if (!resume_done && (tgpd->gpd_flags & 0x1)
				&& (md_ctrl->txq_active
				& (1 << queue->index))) {
				if (!(cldma_read32(md_ctrl->cldma_ap_pdn_base,
					CLDMA_AP_UL_STATUS)
					& (1 << queue->index))) {
					cldma_write32(
						md_ctrl->cldma_ap_pdn_base,
						CLDMA_AP_UL_RESUME_CMD,
						CLDMA_BM_ALL_QUEUE &
						(1 << queue->index));
					CCCI_REPEAT_LOG(md_ctrl->md_id, TAG,
						"resume txq %d\n",
						queue->index);
				}
			}
			spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock,
			flags);
			break;
		}
		/* network does not has IOC override needs */
		tgpd->non_used = 2;
		/* update counter */
		queue->budget++;
		/* update BD */
		list_for_each_entry(req_bd, &req->bd, entry) {
			tbd = req_bd->gpd;
			if (tbd->non_used == 1) {
				tbd->non_used = 2;
				dma_unmap_single(
					ccci_md_get_dev_by_id(md_ctrl->md_id),
					req_bd->data_buffer_ptr_saved,
					tbd->data_buff_len, DMA_TO_DEVICE);
			}
		}
		/* save skb reference */
		skb_free = req->skb;
		/* mark cldma_request as available */
		req->skb = NULL;
		/* step forward */
		queue->tr_done = cldma_ring_step_forward(queue->tr_ring, req);
		if (likely(ccci_md_get_cap_by_id(md_ctrl->md_id) &
			MODEM_CAP_TXBUSY_STOP)) {
			if (queue->budget > queue->tr_ring->length / 8)
				cldma_queue_broadcast_state(md_ctrl, TX_IRQ,
				OUT, queue->index);
		}
		spin_unlock_irqrestore(&queue->ring_lock, flags);
		count++;
#if MD_GENERATION >= (6293)
		ccci_h = (struct ccci_header *)(skb_push(skb_free,
			sizeof(struct ccci_header)));
#else
		ccci_h = (struct ccci_header *)skb_free->data;
#endif
		/* check wakeup source */
		if (atomic_cmpxchg(&md_ctrl->wakeup_src, 1, 0) == 1) {
			md_ctrl->wakeup_count++;
			CCCI_NOTICE_LOG(md_ctrl->md_id, TAG,
				"CLDMA_AP wakeup source:(%d/%d)(%u)\n",
				queue->index, ccci_h->channel,
				md_ctrl->wakeup_count);
		}
		CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
			"harvest Tx msg (%x %x %x %x) txq=%d len=%d\n",
			ccci_h->data[0], ccci_h->data[1],
			*(((u32 *) ccci_h) + 2), ccci_h->reserved,
			queue->index, tgpd->data_buff_len);
		ccci_channel_update_packet_counter(
			md_ctrl->traffic_info.logic_ch_pkt_cnt, ccci_h);
		ccci_free_skb(skb_free);
#if TRAFFIC_MONITOR_INTERVAL
		md_ctrl->tx_traffic_monitor[queue->index]++;
#endif
		/* check if there is any pending TGPD with HWO=1
		 * & UL status is 0
		 */
		spin_lock_irqsave(&queue->ring_lock, flags);
		req = cldma_ring_step_backward(queue->tr_ring,
			queue->tx_xmit);
		tgpd = (struct cldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			need_resume = 1;
		spin_unlock_irqrestore(&queue->ring_lock, flags);
		/* resume channel */
		spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
		if (need_resume &&
			md_ctrl->txq_active & (1 << queue->index)) {
			if (!(cldma_read32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_UL_STATUS) & (1 << queue->index))) {
				cldma_write32(md_ctrl->cldma_ap_pdn_base,
					CLDMA_AP_UL_RESUME_CMD,
					CLDMA_BM_ALL_QUEUE
					& (1 << queue->index));
				resume_done = 1;
				CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
					"resume txq %d in tx done\n",
					queue->index);
			}
		}
		spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
	}
#if MD_GENERATION == (6293)
		/* clear IP busy register to avoid md can't sleep*/
		if (cldma_read32(md_ctrl->cldma_ap_pdn_base,
			CLDMA_AP_CLDMA_IP_BUSY)) {
			cldma_write32(md_ctrl->cldma_ap_pdn_base,
			CLDMA_AP_CLDMA_IP_BUSY,
				cldma_read32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_CLDMA_IP_BUSY));
			CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
				"CLDMA_IP_BUSY = 0x%x\n",
				cldma_read32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_CLDMA_IP_BUSY));
		}
#endif
	if (count)
		wake_up_nr(&queue->req_wq, count);
	return count;
}

/* this function may be called from both workqueue and ISR (timer) */
static int cldma_gpd_tx_collect(struct md_cd_queue *queue,
	int budget, int blocking)
{
	struct md_cd_ctrl *md_ctrl =
		(struct md_cd_ctrl *)ccci_hif_get_by_id(queue->hif_id);
	unsigned long flags;
	struct cldma_request *req;
	struct cldma_tgpd *tgpd;
	struct ccci_header *ccci_h;
	int count = 0;
	struct sk_buff *skb_free;
	dma_addr_t dma_free;
	unsigned int dma_len;

	while (1) {
		spin_lock_irqsave(&queue->ring_lock, flags);
		req = queue->tr_done;
		tgpd = (struct cldma_tgpd *)req->gpd;
		if (!((tgpd->gpd_flags & 0x1) == 0 && req->skb)) {
			spin_unlock_irqrestore(&queue->ring_lock, flags);
			break;
		}
		/* restore IOC setting */
		if (req->ioc_override & 0x80) {
			if (req->ioc_override & 0x1)
				tgpd->gpd_flags |= 0x80;
			else
				tgpd->gpd_flags &= 0x7F;
			CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
				"TX_collect: qno%d, req->ioc_override=0x%x,tgpd->gpd_flags=0x%x\n",
				queue->index, req->ioc_override,
				tgpd->gpd_flags);
		}
		tgpd->non_used = 2;
		/* update counter */
		queue->budget++;
		/* save skb reference */
		dma_free = req->data_buffer_ptr_saved;
		dma_len = tgpd->data_buff_len;
		skb_free = req->skb;
		/* mark cldma_request as available */
		req->skb = NULL;
		/* step forward */
		queue->tr_done =
			cldma_ring_step_forward(queue->tr_ring, req);
		if (likely(ccci_md_get_cap_by_id(md_ctrl->md_id)
			& MODEM_CAP_TXBUSY_STOP))
			cldma_queue_broadcast_state(md_ctrl, TX_IRQ,
				OUT, queue->index);
		spin_unlock_irqrestore(&queue->ring_lock, flags);
		count++;
		/*
		 * After enabled NAPI, when free skb,
		 * cosume_skb() will eventually called
		 * nf_nat_cleanup_conntrack(),
		 * which will call spin_unlock_bh() to let softirq to run.
		 * so there is a chance a Rx softirq is triggered
		 * (cldma_rx_collect)
		 * and if it's a TCP packet, it will send ACK
		 * another Tx is scheduled which will require
		 * queue->ring_lock, cause a deadlock!
		 *
		 * This should not be an issue any more,
		 * after we start using dev_kfree_skb_any() instead of
		 * dev_kfree_skb().
		 */
		dma_unmap_single(ccci_md_get_dev_by_id(md_ctrl->md_id),
			dma_free, dma_len, DMA_TO_DEVICE);
#if MD_GENERATION >= (6293)
		ccci_h = (struct ccci_header *)(skb_push(skb_free,
			sizeof(struct ccci_header)));
#else
		ccci_h = (struct ccci_header *)skb_free->data;
#endif
		/* check wakeup source */
		if (atomic_cmpxchg(&md_ctrl->wakeup_src, 1, 0) == 1) {
			md_ctrl->wakeup_count++;
			CCCI_NOTICE_LOG(md_ctrl->md_id, TAG,
				"CLDMA_AP wakeup source:(%d/%d)(%u)\n",
				queue->index, ccci_h->channel,
				md_ctrl->wakeup_count);
		}
		CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
				"harvest Tx msg (%x %x %x %x) txq=%d len=%d\n",
				ccci_h->data[0], ccci_h->data[1],
				*(((u32 *) ccci_h) + 2), ccci_h->reserved,
				queue->index, skb_free->len);
		ccci_channel_update_packet_counter(
			md_ctrl->traffic_info.logic_ch_pkt_cnt, ccci_h);
		ccci_free_skb(skb_free);
#if TRAFFIC_MONITOR_INTERVAL
		md_ctrl->tx_traffic_monitor[queue->index]++;
#endif
		/* resume channel */
		spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
		if (md_ctrl->txq_active & (1 << queue->index)) {
			if (!(cldma_read32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_UL_STATUS) & (1 << queue->index)))
				cldma_write32(md_ctrl->cldma_ap_pdn_base,
					CLDMA_AP_UL_RESUME_CMD,
					CLDMA_BM_ALL_QUEUE
					& (1 << queue->index));
		}
		spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
	}
	if (count)
		wake_up_nr(&queue->req_wq, count);
	return count;
}

static void cldma_tx_queue_empty_handler(struct md_cd_queue *queue)
{
	struct md_cd_ctrl *md_ctrl =
		(struct md_cd_ctrl *)ccci_hif_get_by_id(queue->hif_id);
	unsigned long flags;
	struct cldma_request *req;
	struct cldma_tgpd *tgpd;
	int pending_gpd = 0;

	/* ACK interrupt */
	cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TISAR0,
			((1 << queue->index) << CLDMA_TX_QE_OFFSET));

	if (md_ctrl->txq_active & (1 << queue->index)) {
		/* check if there is any pending TGPD with HWO=1 */
		spin_lock_irqsave(&queue->ring_lock, flags);
		req = cldma_ring_step_backward(queue->tr_ring,
				queue->tx_xmit);
		tgpd = (struct cldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pending_gpd = 1;
		spin_unlock_irqrestore(&queue->ring_lock, flags);
		/* resume channel */
		spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
		if (pending_gpd &&
		   !(cldma_read32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_UL_STATUS) & (1 << queue->index))) {
			cldma_write32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_UL_RESUME_CMD,
				CLDMA_BM_ALL_QUEUE & (1 << queue->index));
			CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
				"resume txq %d in tx empty\n", queue->index);
		}
#if MD_GENERATION == (6293)
		if (!pending_gpd &&
			!(cldma_read32(md_ctrl->cldma_ap_pdn_base,
			CLDMA_AP_UL_STATUS) & (1 << queue->index)) &&
			cldma_read32(md_ctrl->cldma_ap_pdn_base,
			CLDMA_AP_CLDMA_IP_BUSY)) {
			cldma_write32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_CLDMA_IP_BUSY,
			cldma_read32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_CLDMA_IP_BUSY));
			cldma_read32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_CLDMA_IP_BUSY);
		}
#endif
		spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
	}

}

static void cldma_tx_done(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct md_cd_queue *queue =
		container_of(dwork, struct md_cd_queue, cldma_tx_work);
	struct md_cd_ctrl *md_ctrl =
		(struct md_cd_ctrl *)ccci_hif_get_by_id(queue->hif_id);
	int count;
	unsigned int L2TISAR0 = 0;
#ifdef CLDMA_TRACE
	unsigned long long total_time = 0;
	unsigned int tx_interal;
	static unsigned long long leave_time[CLDMA_TXQ_NUM] = { 0 };

	total_time = sched_clock();
	leave_time[queue->index] = total_time;
	if (leave_time[queue->index] == 0)
		tx_interal = 0;
	else
		tx_interal = total_time - leave_time[queue->index];
#endif
#if TRAFFIC_MONITOR_INTERVAL
	md_ctrl->tx_done_last_start_time[queue->index] = local_clock();
#endif

	count = queue->tr_ring->handle_tx_done(queue, 0, 0);
	if (count && md_ctrl->tx_busy_warn_cnt)
		md_ctrl->tx_busy_warn_cnt = 0;

#if TRAFFIC_MONITOR_INTERVAL
	md_ctrl->tx_done_last_count[queue->index] = count;
#endif
	/* greedy mode */
	L2TISAR0 = cldma_read32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_L2TISAR0);
	if (L2TISAR0 & CLDMA_TX_INT_QUEUE_EMPTY
		& ((1 << queue->index) << CLDMA_TX_QE_OFFSET))
		cldma_tx_queue_empty_handler(queue);
	if (L2TISAR0 & CLDMA_TX_INT_DONE &
		(1 << queue->index)) {
		/* ACK interrupt */
		cldma_write32(md_ctrl->cldma_ap_pdn_base,
			CLDMA_AP_L2TISAR0, (1 << queue->index));
		if (IS_NET_QUE(md_ctrl->md_id, queue->index))
			queue_delayed_work(queue->worker,
				&queue->cldma_tx_work,
				msecs_to_jiffies(1000 / HZ));
		else
			queue_delayed_work(queue->worker,
				&queue->cldma_tx_work, msecs_to_jiffies(0));
	} else {
#ifndef CLDMA_NO_TX_IRQ
		unsigned long flags;
		/* enable TX_DONE interrupt */
		spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
		if (md_ctrl->txq_active & (1 << queue->index))
			cldma_write32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_L2TIMCR0,
				(CLDMA_TX_INT_DONE & (1 << queue->index)) |
				(CLDMA_TX_INT_QUEUE_EMPTY
				& ((1 << queue->index) << CLDMA_TX_QE_OFFSET)));
		spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
#endif
	}

#ifdef CLDMA_TRACE
	if (count) {
		leave_time[queue->index] = sched_clock();
		total_time = leave_time[queue->index] - total_time;
		trace_cldma_tx_done(queue->index, tx_interal,
			total_time, count);
	} else {
		trace_cldma_error(queue->index, -1, 0, __LINE__);
	}
#endif
}

static void cldma_rx_ring_init(struct md_cd_ctrl *md_ctrl,
	struct cldma_ring *ring)
{
	int i;
	struct cldma_request *item, *first_item = NULL;
	struct cldma_rgpd *gpd = NULL, *prev_gpd = NULL;
	unsigned long flags;

	if (ring->type == RING_GPD) {
		for (i = 0; i < ring->length; i++) {
			item =
			kzalloc(sizeof(struct cldma_request), GFP_KERNEL);
			item->gpd = dma_pool_alloc(md_ctrl->gpd_dmapool,
				GFP_KERNEL, &item->gpd_addr);
			if (item->gpd == NULL) {
				CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
					"%s:dma_pool_alloc fail\n", __func__);
				kfree(item);
				return;
			}
			item->skb = ccci_alloc_skb(ring->pkt_size, 1, 1);
			if (item->skb == NULL) {
				CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
					"%s:alloc skb fail,stop init\n",
					__func__);
				dma_pool_free(md_ctrl->gpd_dmapool,
					item->gpd, item->gpd_addr);
				kfree(item);
				return;
			}
			gpd = (struct cldma_rgpd *)item->gpd;
			memset(gpd, 0, sizeof(struct cldma_rgpd));
			spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
			item->data_buffer_ptr_saved =
				dma_map_single(
				ccci_md_get_dev_by_id(md_ctrl->md_id),
				item->skb->data, skb_data_size(item->skb),
				DMA_FROM_DEVICE);
			if (dma_mapping_error(
				ccci_md_get_dev_by_id(md_ctrl->md_id),
				item->data_buffer_ptr_saved)) {
				CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
					"error dma mapping\n");
				item->data_buffer_ptr_saved = 0;
				spin_unlock_irqrestore(
					&md_ctrl->cldma_timeout_lock,
					flags);
				ccci_free_skb(item->skb);
				dma_pool_free(md_ctrl->gpd_dmapool,
					item->gpd, item->gpd_addr);
				kfree(item);
				return;
			}
			spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock,
				flags);
			cldma_rgpd_set_data_ptr(gpd,
				item->data_buffer_ptr_saved);
			gpd->data_allow_len = ring->pkt_size;
			gpd->gpd_flags = 0x81;	/* IOC|HWO */
			if (i == 0)
				first_item = itemq;
	struct cld[e_time[CLDMA_TXQ_NUM] = { 0 };

	total_time = sched_clock();
	leave_time[queue->index] = total_time;
	if (leave_tileave_tileave_tileave_tileave_tileave_tileave_tileave_tileave_tila5MA_AP wakeup source:(%d/%d)(%u)\n",
				queue->index, ccci_h->channel,
				md_ctrl->wakeup_count);
		}
		CCCI_DEBU?_la?_itF cldma_tgpd _CAP_TXBUSY_STOP)) {
			if (queue->budget > queue->tr_ring->length / 8)
				cldma_queue_broadcast_state(md_ctrl, TX_IRQ,
				OUT, 
			if (i == 0rgpdd_id, TAHst_state(md_ctrl, TX_IRQ,
				OUT, 
			if (i == 0rgpdd_id, T_?_A_AP_CLDMA_IP_BUSY));
			Cm					item-C/[cci_header *)(skb_push(skb_free,
			sizeof(struct ccci_header)));
#else
		ccci_h = (struct ccci_header *)skb_free-_DEBU?_la?_itFatai[check wakeup source */
		if (atomic_cmpxchg(&md_ctrl->wakeup_src, 1, 0) == 1) {
			md_ctrl->wakeup_count++;
			CCCt_state(md_ctr}
		CCCI_DEBU?_la?_lpnt++;
			CCCt_state(md_ctr}
		CCCI_DEBU?_la?_lpnt++;
			CCCtqqueue->&& (tDMAedtrl->md_id, TAG,
					"%s:all&& (tDMA(md_ctrl->md_id, TAG,
					"%s:alhannel */
ail,stop init\n",
					__fun				& (1 << queue-b
ail,stop init\nb
					__fuee(md_ctrl->gpd_dmapool,
					item->gpd, item->gpd_addr);
				kfree(item);
				r n;
			}
			gpd = (struct cldma_rgpd *_lock_irqsave(&
			memset(gpd, 0, sizeof(struct cldma_rgpd));
			spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
			item->data_buffer_ptr_saved =
				dma_map_single(
				cc-%dci_md_get_dev_by_id(md_ctrl->md_iil\n", __func__ctrl->wakeup_count);
		}
		CCCI_DEBex);
		}
#if MD_GENERATION =			flags);
				ccci_ftree_skb(item->skb);
				dma_tool_free(mending_gpd &&
			if (liTX_IRQ,	OUT, 
			if (i == 0rgpdd_id, T_?_A_AP_CLDMA_IP_BUSY));
			Ct					item-C/[cci_header *)(skb_push(skb_free,
			sizeof(struct ccci_headerbd			sizeof(struct ccci_header)));
#else
		ccci_h = (struct ccci_header *)skb_free-_DEBU?_la?_itFatait[check wakeup sout					item-C/[cci_tc_cmpxchg(&md_ctrl->wakeup_src,e(md_ctrl->gpd_dmapool,
			_B			item->gpd, item->gpd_addr);
				kfree(item);
				r n;
			}
			gpd = (struct cldma_rgpd *_lock_irqsave(&
			memset(gpd, 0, sizeof(struct cldma_rgpd));
			spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
			item->data_buffer_ptr_saved =
				dma_map_single(
				cc-%dci_md_get_dev_by_id(md_ctrl->md_iil\n", __func__ctrl->wakeup_count);
		}
		CCCI_DEBex);
		}
#if MD_GENERATION =			flags);
				ccci_ftree_skb(item->skb);
				dma_tool_free(mending_gpd &&
			if (2 TX_IRQ,
BDP	OUT, 
			if (i == 0rgpdd_id, T_?_A_AP_CLDMA_IP_BUSY));
			Ct					item-C/[cci_header *)(skb_push(skb_free,
			sizeof(struct ccci_headerbd			sizeof(struct ccci_header)));
#else
		ccci_h = (struct ccci_header *)skb_free-_DEBU?_la?_itFatait[check TX_Ii_h{
			if (tx_doxd_i 1{
		->gp)) {he32(md_ctm->gpdj item-jgpdMAX_B	ldma + 1m-jee(item);	edtrl->md_n;
			}
			gpd = (struct cldma_rgpd *_lockk_irqsave(&
			me	edtrl->(gpd, 0, sizeof(struct _lockk%d/%d)(%u)\n",
				queue->i_irqsave(&md_edtrl->(gpd,l,
				md_ct			iedtrl->(gpd, 0		item->data_bbuffer_ptr_saved =
				dma_map_single(
					cc-%dci_md_get_dev_by_id(md_ctrl->md_iil\n", __func__cttrl->wakeup_count)trl->waedtrl->_count)t;
		}
		CCCCI_DEB	b;
		}
#if MD_GENERAeue-)edtrl->(gpd,		CCCC	ccci_fbe_skb(item->skb);
				dma_tbl_free(mt			ij (i == 0rgp);
			Ct					iteength / 8tr *)(skb_p	edtrl->(gpd,l,
				md_ctIP_BUSY))			dma_tbl		item-C/[cci_headeb*)(skb_p	edtrl->(gpd,l,
				md_cteof(struct ccci_edtrl->(g)));
#else

		ccci_h = (stredtrl->(g)));
md_ctrl->bd			siz	nit\nb
			bd		CCCI_DEBt ccb, req);
		if , TX_IEOL(md_ctrl->up sout					item-C/[cci_tc_cmpxchg(&md_ctrl->wakeup_sr	if (ret >= 0 || ret == -CCCI_ERR_DROP_PACpi =%p -> b_free-_=%pmd_cted whier *)skb_free-_DEBUc, 1, 0) == 1) {
			md_ef CLDswitchree-_ue->tx_xmit);
		tgpd = (struct cldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pending_gpd = 1;
		(&queue->ring_lock, flags);
\n",
				quedircldmaUT#ifdef CCCpd *gpd = NULL, *pre& INT_ULL;;
	stMASK_timeout_lnormalpdn_base,
	yed_wo->pkt_si_lave_>> 8pd_flags = 0x81;= 0rgpe,
			CLDMA_AP}
			spe_tileave_t				ctrl->w[			spe				ctre,
		2l->w[_ctrl->gpd_dm]eck TX_Ie,
		is & CLDMueueexcep(!((tgn_base,
	yd_ctr CCCpd *gpd = NULL, *pre& I MODEMULL;;
	stMASK_ 0rgpe,
			CLDMA_AP}
			spe_tileave_t	ormal	ctrl->w[			spe	ormal	ctre,
		2l->w[_ctrl->gpd_dm]eck memcpy(&ccci_h, &lhLDMA_AP}
			sp_tileave_t	ormal	ctrl->w[			sp	ormal	ctre,
		2l->w[_ctrl->gpd_dm]eck m	strLOG(mdqueue-chg(&> queu&_h, &lhLDMA_APskb_free-_!kthrea&queue->ring_lock, th / 8)
eck chg(&md_ctrl->wakags);
empty\n", queuewakags);
empty\n"nentely.
, ccci_h->channel,
			LOG(md_ctrn",
				quedircldmIN#ifdef CCCpd *gpd = NULL, *pre& INT_RLL;;
	stMASK_timeout_lnormalpdn_base,
	yed_wo->pkt_si_lave_>> 8pd_flags = 0x81;= 0rgpe,
			CLDMA_AP}
			spe_tileave_t				rtrl->w[			spe				rtre,
		2l->w[_ctrl->gpd_dm]eck TX_Ie,
		is & CLDMueueexcep(!((tgn_base,
	yd_ctr CCCpd *gpd = NULL, *pre& I MODEMRLL;;
	stMASK_ 0rgpe,
			CLDMA_AP}
			spe_tileave_t	ormal	rtrl->w[			spe	ormal	rtre,
		2l->w[_ctrl->gpd_dm]eck memcpy(&ccci_h, &lhLDMA_AP}
			sp_tileave_t	ormal	rtrl->w[			sp	ormal	rtre,
		2l->w[_ctrl->gpd_dm]eck m	strLOG(mdqueue-chg(&> queu&_h, &lhLDMA_APskb_free-_!kthrea&queue->ring_lock, th / 8)
eck chg(&md_ctrl->wakags);
empty\n" = temp_timkags);
empty\n"nentely.
, ccci_h->channel,
			LOG(e_delqueuewakeup be flushe4) {
tma_fse,
if (ret >= 0 || ret == -CCCI_ERR_DROP_"& CLDM_lock switch(md_ctto %pmd_c
EUE
					& (1 <<				quedir,
, ccci_h->chan)->index) << CLDMA_RX_QE_OFef CLDkeup_count++;
			C		tgpd = (struct cldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pending_gpd = 1;
	
				md_ef CLDswitchree-_u flags);
eof(sWORK(ime[queue->indrx] = sche->indrx]rl->s);
		CLDMA
 */od_dqueuerl->mofueue0) contu32 i me[quCLDMA		clqueueconcur conl
mdbutdqueuerl->smofue,
	samuCLDMAu32 i me[qu musurce:queuewlockntiNERATasCLDMA
oueudct(si) {
mcont_AP_			el
		 x]rl->ve & x]rl->NERATION(count) {
		ly.
A_FROM 0 && req("md%rl->%CE
	iferLOG(md_cWQ_UNBOUND | WQ_MEM_RECLAIMma_m_lockk%d/%d)(%uCCCI_ + 1a_tx_done(struct w< queue->ef CLDkeup_nfo, IN,
				(int
quest *req;
	struc			if (dma>hanRLL;;
	stMAX_LENndex));
eup_nt;
ef CLDhe32ldma_ap_ao_base,
	DMA_TX_INT_QUEUE_EMPTY
				& ((1 << queue->indexma_ap_ao_bd, skbmd_nd, skb)runyed_wor				!skb_queue_emp((1 << ROP_PAe->indrxq%dma_tx_done(struct wif (ret >= 0 || ret == -CCCI_ERR_DR (md_ctr
	if=%pmd_c
EUE
					& (1 <<ime[queue->indrx] = s)->index) << CLDMA_RX_QEtOFef CLDkeup_count++;
			C		tgpd = (struct cldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pending_gpd = 1;
	
				md_ef CLDswitchree-_u flags);
(count) {
		ly.
	A_FROM 0 && req("md%rlt>%CE
	iferLOG(mWQ_UNBOUND | WQ_MEM_RECLAIMG(m| %s:dma_pool_a	dma_l? WQ_HIGHPRI :dexOG(m1dex]
		= loCCCI_ + 1a_tx_done(struct weof(sDELAYEDsWORK(ime[queue->indtx] = sche->indtx]rl->s);
if (ret >= 0 || ret == -CCCI_ERR_DR (td_ctr
	if=%pmd_c
EUE
					& (1 <<ime[queue->indclock();
# >= (6293)
#er			itemIMshou
eup_			spu&_h, &lhL {
			i			spi);
(count)L {
			i			sp.	if (!((t				&queL {
			i			spl->md);
(count)L {
			i			sp.0)
			skb-_);
				dmaw_skb));
(count)L {
			i ccci_queue-(count)L {
			irl->rn_cnt
				kfreCLDMA_RX_QEone(qucldm_count++;
			CCCt_state(md_++;
		;
#if TRAFFIC_MONITOR_INTERV_RX_QEldmEone(que_skb(
		dma_ndexone(qucldm_OR_INTERV_RX_QEldmE 1;
	freCLDMA_RX_QEdise(qucldm_count++;
			CCCt_state(md_++;
		;
#if TRAFFIC_MONITOR_INTERV_RX_QEldmEone(que_s->tx_traffidexdise(qucldm_OR_INTERV_RX_QEldmE 1;
	freCLDMA_RX_QEdise(qucldm_nosync_count++;
			CCCt_state(md_++;
		;
#if TRAFFIC_MONITOR_INTERV_RX_QEldmEone(que_s->tx_traffidexv_by_id(gpd->gpd_flagsi%d/%r,
so_STATdise(qucldm_nosync
#else
	;
STATdise(qucldmsi%d/%r,
sysl->makeuphaDMA_(md_ctrlise(qucldm_nosync_OR_INTERV_RX_QEldmE 1;
	frex) << CLDMA_RX_QE_OF
	iferi ccci_count++;
			CCCt_state(md_ct[queqno++;
_error(
				ccci_md_get_dev_buct md_e(&md_ctn",
	nor *)skb->darely.
, cccE
	if OR_INTERVmd_[	no]. {
		leave_tp_tileave_tmd_[	no].e->indrx] = s)->G(md_ctrl->mdasklkb)hueu{
			mdITOR_INTERV_RX_QEmd_0_daskunc__);
P_BUSdasklkb)hueu{
			mdITOR_INTERV_RX_QEmd_0_daskunct
				kfreCLDMA__weak+;
			CC dma ;
	DCM_count++;
			CCCt_state(md_++;
index) << CLDMA_RX_QEldmEelse
cb_count++;
			CCCt_state(md_++;
			CCCt_	if (q
		md_ctrl->tx_buM <<  = 0; <<  =& (1 << otal_time otal_tim_REGd_ctrl-telyL2ex, tx_intemd_ctrleue->worker,
				&queue->cldma_tx_work, msecs_to_jiffiedma_read32(md_ctrlunc_			CLDMA_AP_L2RISAR0, (1 << queue->index));
			/* c	int count = 0ctrlunc_		buM <
				&queue->cldma_tx_work, msecs_to_jiffiedma_read32(md_Mrlunc_			CM <
				&queue->cldma_tx_work, msecs_ !(!bloc	int count = 0;rlunc
		;
#if TRAF_emptTOR_INTERVAL
		md_ct_traffidexif
			ccci_skb_enqueue(&queue->skb_list, tes;
		t {
skb);
			L2(%xCLDMA_xCLDMen);
		sk =& (1 << otal_time otbuM <<  = 0; <unc_IP_BUSYif (ret >= 0 || ret == -CCCI_ERR_DROP_PAu32 i IRQ	L2(%xCLDMA_xCLDMen);
		sk =& (1 << otal_time otbuM <<  = 0; <unce_time[queue->index] - tot (md_ctrl-_ge~(md_Mrlunc2TISAR0 & Cuct *work)
{
	struct dela32(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_ase,
				C_ge~(mR_Mrlunc_			C_tim_REGrl->cldma_ap_pdn_scatt,
				CLDMA_AP;
P_BUSase,
				C_ge~(mR_Mrlunc_			C_tim_REGrl-			C_timl->cldma_ap		if (md_ctrl->txq_active & _ptr_idexif
		_ptr_saved =
				dma_map_singl Au32 i Tx UM] = A_xCLDMn);
		s		&queue->cldma_tx_work, msecs_to_jiffiedma_read32(3d_ctrlu
		s		&queue->cldma_tx_work, msecs_to_jiffiedma_read32(3d_ctrppingp		if (e,
				CLDMA_AP_CLDMA__ptr_idexif
		_ptr_saved =
				dma_map_singl Au32 i Rx UM] = A_xCLDMn);
		s		&queue->cldma_tx_work, msecs_to_jiffiedma_read32(3R_ctrlu
		s		&queue->cldma_tx_work, msecs_to_jiffiedma_read32(3R_ctrpping
_error(
				ccci_md_get_dev_bung, nire
		 !base,
				CLDMA_AP_CLDMA_IP_Bidexstore (md_ctrl->txq_active & (1 <idexstore (e,
				CLDMA_AP_CLDMA_L;
	struct idexstore (md_ctrl->txq_active & L;
	struct i	first_
		md_ctrl->tcoda_vers!(( time. da_vers!((
				&queue->cldma_tx_work, msecs_to_jiffies(0)NUM] = { 0 };

CODA_VERSci_queue->in nire
		 . da_vers!((
	*)sk_timeouuffer_ptr_saved =
				dma_map_single(
	     "norTx  = Rx<  =& (1 <=%X<  =& (1 1=%X<  =R (1 <=%X<  =R (1 1=%X<  =d_Mrl=%X<  =R Mrl=%X< CODA_VERSci_=%Xn);
		sk     		&queue->cldma_tx_work, msecs_to_jiffies(0)));
	} else {
#ifnde
		sk     		&queue->cldma_tx_work, msecs_to_jiffies(0)));
	} else {
#ifn1e
		sk     		&queue->cldma_tx_work, msecs_to_jiffies(0)));
	} else {R#ifnde
		sk     		&queue->cldma_tx_work, msecs_to_jiffies(0)));
	} else {R#ifn1e
		sk     		&queue->cldma_tx_work, msecs_to_jiffies(0)));
	} else {d_Mrlu
		sk     		&queue->cldma_tx_work, msecs_ !(!blocking));
	} else {R#Mrlu
		sk     		&queue->cldma_tx_work, msecs_to_jiffies(0)));
	} else0 };

CODA_VERSci_q			siz;
			CC dma ;
	DCM_ate(md_+eck wak}->cldma_ap		if (md_ctr)x]++;
#endiAP_SO_RESUME_C_id, TAG,
	ldm_uffer_RESU_sing dev_kfe (md_ctrl->txq_active & (1 <i)->cldma_ap_pdn_dex, -1, 0,ueue_delayed_wodate (md_ctrl->txq_active & (1 <i)	siz;
		))
			queue_delayed_work(qu TX_Iick) == trl->cldma_ap_pdn_base,
				CLDMA_AP_L2TIMCR0,
				(CLDMA_TX_INT_DONE & (1 << (md_ctrlunc_m->gpd, item->gpdL;
	stLENndex, -1, 0, )kfree(item);
	if (md_ctrl->txq_active & (1 <			sp_uct cldi))x]++;
#end93)
#er			itemIMshou	CLDMA_TX_INT_QUEUE_EMPTY
				& ((i	tbd->non_dex, -1, 0, [i].L {
			irl-->non_ flags;
	struct cllllldma_requuuudel_			spuave_tp_tileave_t0, [i].L {
			i			spi);
		rqrestore(&md_ctrl->cldma_timeout_lock, flP_"&_sta del_			sp

stptrx);
pree = req->Ct_	if = req->_tileave_t0, [i].L {
			i			spi);
		r= sched_clo TX_Ilise(queue->index, tx_interal,
d_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TISAR0,
			((1 << qu {d_MSa_ring **ring)
{
	int i;
	struct cldiitem, *fiirst_item = NULL;
	struct cldmaa_rgpd *gpirev_gpd = NULL;
	unsigned longCLDMA_TX_INT_QUEUE_EMPTY
				& ((i	tcllllldma_r ef CLDMA_TRACE
	if = req->tileave_t0, [i]. {
		leave_tp>_tileave_t0, [i].store(&md_ctrl->cldldma_timeout_lock, flags);
#endif
ctIP_BUSY))	dma_r ef CLDMA_TRACE
	if = req->tileave_t0, [i]. {
		leave_tp>_tileave_t0, [i].store(&md_ctrl->cldldma_timeout_lock, 0endif
ct(md_ctrl->cldma_ap_pdn_base,
				CLDMA_AP_td_ctr		tgpd
	if=%		cldCCt_	if)		CCCI_DEB
	if (md_ctrl-
*fiirst_item = NULL;
	struct l-
*fiirpd *gpirev_gpd = NULL;
	unsigned l,
d_ctrl->rl->cldma_timeout_lock,_tileave_t0, [i]+eck wak}-gp		if (e,
			)x]++;
#endiAP_SO_RESUME_C_id, TAG,
	ldm_uffer_RESU_Ring dev_kfe (e,
				CLDMA_AP_CLDMA_IP_Bi)_queue *queue)ick)R== trl->cldma_ap_pdn_base,
				CLDMA_AP_L2TIMCR0,
				(CLDMA_TX_INT_DONE al_time otal_tim_REG)qu TX_Is[queue->index] += rxbytes;
		trace_cldma_rx_pdn_base,
				CLDMA_AP_L2TIMCR0,
				(CLDMA_TX_INT_DONR0 = 0;
#ifdef CLDMA_TRACE
	unsigned long long total_time = 0;
	unsigned int tx_interal;
	s->gpd, item->gpdL;
	stLENndex, -1, r, )kfree(item);
	ifbase,
				CLDMA_AP_CLDMA_IP_BUSY));
		iitem|		sk    e (e,
				CLDMA_AP_CLDMA_L;
	struct 			sp_ucpd *gpirev_gpd = N>ring_lock, flabd->nontileave_t0_hif_get_by_id(queue->hiisrloc(mdi]->non_ flags;
	struct cllllX_Ilise(questruct curceL;
	struct lx, tx_interal,
d_ctrl->cldma_a_header *ccci_h;
king));
	} else {R#MSa_ring **ring)
{_CLDMA_IP_BUSY));
		iiteming **ring)
{_CLDMA_L;
	struct cldmaa_rgpd *gpirev_gpd = N>ring_lock, flagsllllX_Ila_write32(md_ctLDMA_TRACE
	unsigned long long to !(!blocking));
	} else {R#MSrlunc_mllX_always, ccci_step_feue->n_locapimd_ctLDMA_TRACOF
	iferi ccci_ate(md_ct[)		CCCI_DE}BUc, 1, 0) == ldma_		}
_ue->ringisr(		CCCrq, CLDMA*0)
	ruct cldma_tgpd *)req->gpd;
		if  gpd->gpd_flags & 0x1)_pkt_c;
if (ret >= 0 || ret == -CCCI_ERR_DR (u32 i IRQ!tal_timtileave_t0_hif_get_by_id(queuisrloc(ma_write32(md_ctrl-	_RX_QEldmEelse
cb_ate(md_+eck
		req IRQ_HANDLED->index) << CLDMA_RX_QEldmEelse)));
}

/* this function may be called fromreq->gpd;
		if ((tnd ISR (timer) */
static int cld(md_ct_RX_QEldmEelse;
	
				md_ldmEelse
cb_ate(md_+ecfreCLDMA__weak+>> 8a_ti_id(qncyty(&qmay bpr_no <<e("[ && /la_wr] %rl->csoursup&md_ed!md_ctrl->md_id),freCLDMA__weak+_RX_QEd> 8a] += rxb_count++;
			CCCt_state(md_++;
indeCLDMA_RX_QEnt++;
		md_ctrchueupd = 1;uct cldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pepd = 1;
		(&queue- queumema] +=on>gpd- quedbg		skb->tstmsize;
umema
		));
->gpd_flags = 0x81ckingSMEM_USER_RAW_MDif (reBG;
		(&queue- queumema] +=on>gpdssedbg		skbb->tstmsize;
umema
		));
->gpd_flags = 0x81ckinggSMEM_USER_RAW_MDSSreBG;
		(&queue- que
		if (count > 0)
			skb->tstamp = sched_clock();
#endif
		ccc			CChed_bgEd> 8areq)a_wex)) |
		(CLDhed_bgEd> 8areq);_buct md_I_ERROR,>skb c__);
				dma_pool_fr >= (6293)
#er			itemIMshou
eueqnol->cldma_ap)
			& MODEM_CAP_TXBUSY_STOP))
			cl "%rl & 0x%psree = rrl->md_iil\nbuiltin_a_		}
_akeues, 0endifd_ctrl->gpd_dmapool,
					item->gpd, item->gpd_addr);
			
		 *opMD,
	 ==urceR0 & CLDrleue-ioc_overridetileave_t0, __LINE__)_ge~P_CLDMA_IP_BUSY);

			dobd->npdn_base,
				CLDMA_AP_L2TIMCR0,
				(CLDMA_TX_INT_DONer_ofOP			CL P_CLDMA_IP_BUSY);

			lX_Ila_write32(md_ct		&queue->cldma_tx_work, msecs_to_jiffies(0)NUM] = { er_ofOP			C)_h->data[1],
				*(((u32 *) ccci as avaueue *quedma_r MA_TRACE
	unsigned long long total_time = 0;
	unsigneder_of(dworueue->in(++loc(siz% flagags=*)skb->dad)
			& MODEM_CAP_TXBUSY_STOP))
			cldma_qu *opM ==	unsi,emd_ctr=%xI_ERROR				cldma_wrmd_I_ERROR)		CCC)
			MEM__CA_		cP_TXBUSY_STOP))
			cldma_quD> 8ata EX		dotal_time pdn_dex_bgEd> 8areq)aSY));
		ta[eBG_DUMP_SMEM		cldma_re>tstutil_mema>> 8pd_flags = 0x81cking));f (reUMP_MEM_eUMPm_lockk%d- quedbg->timeg toviewovirm_lockk%d- quedbg->f (d_tileave_tilutil_mema>> 8pd_flags = 0x81cking));f (reUMP_MEM_eUMPm_lockk%dssedbg->timeg toviewovirm_lockk%dssedbg->f (d_tileaI_DEB/*_flagsd> 8adebuga] += rxb_ate(md_+ea_rx_done(qud> 8a] += rxb_ate(md_+eck f_get_by_id(= 16lagag	cldma_r/d(md_ctrnd firmd_ct
			EMIm_lock * OnRATION  befeq->EE_lock *d_ctLD>> 8a_ti_id(qncyt)_h->da (6R (d(CONFIG_MTK_AEE_FE(dwRE l,
d_aACEateexcep(!((g ti(,stop me ,stop meDMA_AP_md1:\nUNKNOWN Excep(!((\n *opM ==	unsiy_id(ed.ree = req-DB_OP(sDEFAULT)_queue *quest as availCI_DE}BUc gpd->gpdma_ *)sk;e-ioc_overridetileave_tr, __LINE__)_ge~P_CLDMA_IP_BUSY);

			dobd->npdn_base,
				CLDMA_AP_L2TIMCR0,
				(CLDMA_TX_INT_DONSO_ofOP			CL P_CLDMA_IP_BUSY);

			l		&queue->cldma_tx_work, msecs_to_jiffies(0)NUM] = { SO_ofOP			C);lX_Ila_write32(md_->data[1],
				*(((u32 *) ccci as avaueue *quedma_r MA_TRACE
	unsigned long long to !(!blockingNUM] = { SO_of(dworueue->in(++loc(siz% flagags=*)skb->dad)
			& MODEM_CAP_TXBUSY_STOP))
			cldma_qu *opMR==	unsi,emd_ctr=%xI_ERROR				cldma_wrmd_I_ERROR)		CCC)
			MEM__CA_		cP_TXBUSY_STOP))
			cldma_quD> 8ata EX		dotal_time pdn_t_by_id< 5lagag	c||et_by_id(q12lagag	)ongCLDMA_dex_bgEd> 8areq)cldmaa_rg);
		ta[eBG_DUMP_SMEM		cldma_rve_tilutil_mema>> 8pd_flags = 0x81cking)));f (reUMP_MEM_eUMPm_lockkk%d- quedbg->timeg toviewovirm_lockkk%d- quedbg->f (d_tileavve_tilutil_mema>> 8pd_flags = 0x81cking)));f (reUMP_MEM_eUMPm_lockkk%dssedbg->timeg toviewovirm_lockkk%dssedbg->f (d_tileaaI_DEB/*_flagsd> 8adebuga] += rxb_ate(md_+ea_rx_done(qud> 8a] += rxb_ate(md_+eck f_get_by_id(= 16lagag	cldma_r/d(md_ctrnd firmd_ct
			EMIm_lock * OnRATION  befeq->EE_lock *d_ctLD>> 8a_ti_id(qncyt)_h->da (6R (d(CONFIG_MTK_AEE_FE(dwRE l,
d_aACEateexcep(!((g ti(,stop me ,stop meDMA_AP_md1:\nUNKNOWN Excep(!((\n *opMR==	unsiy_id(ed.ree = req-DB_OP(sDEFAULT)_queue *quest as availCI_DE}BUc gpd->gpdma_ *)sk;e-X_Is[queuON  L2=urceL3lx, tx_intst_lock, flags);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE & (1 << P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE R (1 << P_CLDMA_IDMA_P_Brl-_error(
				ccci_md_get_dev_bk, flags);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE & (1 1< P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE R (1 1< P_CLDMA_IDMA_P_Brl-ueue *qu_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3& (1 << P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3R (1 << P_CLDMA_IDMA_P_Brl-u_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3& (1 1< P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3R (1 1< P_CLDMA_IDMA_P_Brl-lX_Ilise(queON  L2=urceL3lx, tx_intst_lock, flags);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE & MSa_r P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		_header *ccci_h;
kin;
	} else {R#MSa_r P_CLDMA_IDMA_P_Brl-_error(
				ccci_md_get_dev_bk, flags);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE & MSa1< P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
 !(!blockin;
	} else {R#MSa1< P_CLDMA_IDMA_P_Brl-ueue *qu_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3& MSa_r P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3R MSa_r P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3& MSa1< P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3R MSa1< P_CLDMA_IDMA_P_Brl-lX_I *opM			sp
md_->d (6293)
#er			itemIMshou->gpd	noritem-	nor_gpd = NULcldmam-	no++idexdel_			spu_tileave_t0, [	no].L {
			i			spi);ueue *quU?_la?_itF cldma_tgpd _CAP_TXBUSY_STOP)) {
			if (queueddr);
			
		flush_step__lock, fladise(qucldm_OR_INTE
			flushEelse)TOR_INTERV_RX_QEldmE = s)->G->gpd, item->gpdL;
	stLENndex, -1, 0, )kfree(eueddushEMA_TRACE
	if _tileave_t0, [i].store(&md_ctr)->G->gpd, item->gpdL;
	stLENndex, -1, r, )kfree(item)/*q0l->cut_locstaticasklkb,n_loceags dush_free);
#f (i == 0rgnd Iinb));
	flushEelse)TOR_INTERVr, [i].store(rx] = s)->G(	freCLDMA_RX_QE *ope->bude;
		md_ctrchueupd = 1;uct cldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pepd = 1;
		uct md_I_ERRORkb c__);
				dma_pool_fr	(&queue- queumema] +=on>gpd- quedbg		skb->tstmsize;
umema
		));
->gpd_flags = 0x81ckingSMEM_USER_RAW_MDif (reBG;
		(&queue- queumema] +=on>gpdssedbg		skb->tstmsize;
umema
		));
->gpd_flags = 0x81ckingSMEM_USER_RAW_MDSSreBG;
		(&queue- que
		if (count > 0)
			skb->tstamp = sched_clock();
#endif
		ccc			CChed_bgEd> 8areq)a_wex)) |
		(CLDhed_bgEd> 8areq);_ap)
			& MODEM_CAP_TXBUSY_STOP))
			cl "%rl & 0x%psree = rrl->md_iil\nbuiltin_a_		}
_akeues, 0endifd_ctrl->gpd_dmapool,
					item->gpd, item->gpd_addr);
			
		 *opMD,
	 ==urceR0 & CLDrmdbutdnon- *opMR==onDrleue-ioc_overridetileave_t0, __LINE__)_ge~P_CLDMA_IP_BUSY);

			dobd->npdn_base,
				CLDMA_AP_L2TIMCR0,
				(CLDMA_TX_INT_DONer_ofOP			CL P_CLDMA_IP_BUSY);

			l		&queue->cldma_tx_work, msecs_to_jiffies(0)NUM] = { er_ofOP			C)_lX_Ila_write32(md_->data[1],
				*(((u32 *) ccci as avaueue *quedma_r MA_TRACE
	unsigned long long total_time = 0;
	unsigneder_of(dworueue->in(++loc(siz% flagags=*)skb->dad)
			& MODEM_CAP_TXBUSY_STOP))
			cldma_qu *opM ==	unsi E,emd_ctr=%xI_ERROR				cldma_wrmd_I_ERROR)		CCC)
			MEM__CA_		cP_TXBUSY_STOP))
			cldma_quD> 8ata EX		dotal_time pdn_dex_bgEd> 8areq)
dmaa_rg);
		ta[eBG_DUMP_SMEM		cldma_re>tstutil_mema>> 8pd_flags = 0x81cking));f (reUMP_MEM_eUMPm_lockk%d- quedbg->timeg toviewovirm_lockk%d- quedbg->f (d_tileave_tilutil_mema>> 8pd_flags = 0x81cking));f (reUMP_MEM_eUMPm_lockk%dssedbg->timeg toviewovirm_lockk%dssedbg->f (d_tileaI_DEB/*_flagsd> 8adebuga] += rxb_ate(md_+ea_rx_done(qud> 8a] += rxb_ate(md_+eck f_get_by_id(= 16lagag	cldma_r/d(md_ctrnd firmd_ct
			EMIm_lock * OnRATION  befeq->EE_lock *d_ctLD>> 8a_ti_id(qncyt)_h->da (6R (d(CONFIG_MTK_AEE_FE(dwRE l,
d_aACEateexcep(!((g ti(,stop me ,stop meDMA_AP_md1:\nUNKNOWN Excep(!((\n *opM ==	unsiy_>gpEEy_id(ed.ree = req-DB_OP(sDEFAULT)_queue *quest as availCI_DE}BUc gpd->gpdma_ *)sk;e-ioc_overridetileave_tr, __LINE__)_((tg~(P_CLDMA_IP_BUSY);
e& I NofOP	;;
	stMASK_
			dobd->npdn_base,
				CLDMA_AP_L2TIMCR0,
				(CLDMA_TX_INT_DONSO_ofOP			CLMA_TX_INT_A_IP_BUSY);
e& I NofOP	;;
	stMASK_			l		&queue->cldma_tx_work, msecs_to_jiffies(0)NUM] = { SO_ofOP			C);lX_Ila_write32(md_->data[1],
				*(((u32 *) ccci as avaueue *quedma_r MA_TRACE
	unsigned long long to !(!blockinggNUM] = { SO_of(dwore& I NofOP	;;
	stMASKueue->in(++loc(siz% flagags=*)skb->dad)
			& MODEM_CAP_TXBUSY_STOP))
			cldma_qu *opMR==	unsi E,emd_ctr=%xI_ERROR				cldma_wrmd_I_ERROR)		CCC)
			MEM__CA_		cP_TXBUSY_STOP))
			cldma_quD> 8ata EX		dotal_time pdn_dex_bgEd> 8areq)aSY));
		ta[eBG_DUMP_SMEM		cldma_re>tstutil_mema>> 8pd_flags = 0x81cking));f (reUMP_MEM_eUMPm_lockk%d- quedbg->timeg toviewovirm_lockk%d- quedbg->f (d_tileave_tilutil_mema>> 8pd_flags = 0x81cking));f (reUMP_MEM_eUMPm_lockk%dssedbg->timeg toviewovirm_lockk%dssedbg->f (d_tileaI_DEB/* _flagsd> 8adebuga] += rxb_ate(md_+e a_rx_done(qud> 8a] += rxb_ate(md_+eck f_get_by_id(= 16lagag	cldma_r/d(md_ctrnd firmd_ct
			EMIm_lock * OnRATION  befeq->EE_lock *d_ctLD>> 8a_ti_id(qncyt)_h->da (6R (d(CONFIG_MTK_AEE_FE(dwRE l,
d_aACEateexcep(!((g ti(,stop me ,stop meDMA_AP_md1:\nUNKNOWN Excep(!((\n *opMR==	unsiy_>gpEEy_id(ed.ree = req-DB_OP(sDEFAULT)_queue *quest as availCI_DE}BUc gpd->gpdma_ *)sk;e-X_Is[queuON  L2=urceL3lx, tx_intsmdbutdnon- *opMR==onDrleue-i, flags);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE & (1 << P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE R (1 << P_CLDMA_IDMA_P_Be& I NofOP	;;
	stMASK_32rl-_error(
				ccci_md_get_dev_bk, flags);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE & (1 1< P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE R (1 1< P_CLDMA_IDMA_P_Be& I NofOP	;;
	stMASK_32rl-_eue *qu_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3& (1 << P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3R (1 << P_CLDMA_IDMA_P_Be& I NofOP	;;
	stMASK_32rl-u_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3& (1 1< P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3R (1 1< P_CLDMA_IDMA_P_Be& I NofOP	;;
	stMASK_32rl-uX_Ilise(queON  L2=urceL3lx, tx_intsmdbutdnon- *opMR==onDrleue-i, flags);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE & MSa_r P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		_header *ccci_h;
	int count = 0;Sa_ring ring)
{_CLDMA_IP_BU|LDMA_AP_CLDMA_L;
	struct 			s|LDMA_AP_CLDMA__ptr_ie& I NofOP	;;
	stMASK_32rl-_error(
				ccci_md_get_dev_bk, flags);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE & MSa1< P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
 !(!blockin;
	} else {R#MSa1< P_CLDMA_IDMA_P_Brl-ueue *qu_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3& MSa_r P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3R MSa<< P_CLDMA_IDMA_P_Be& I NofOP	;;
	stMASK_32rl-u_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3& MSa1< P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3R MSa1< P_CLDMA_IDMA_P_Be& I NofOP	;;
	stMASK_32rl-quU?_la?_itF cldma_tgpd _CAP_TXBUSY_STOP)) {
			if (quaddr);
		}ueue->worker,
				&queue->cldmaex) << CLDMA_flags [quequeue_emp_cloc
		md_ctrchueupd = 1;uct cldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pepd = 1;
	
C	ccci_ftileave_t0,queue_empty_handp meDMAitem->stileave_t0,queue_empty_hand_
				ccci_ftileave_tr,queue_empty_handp meDMAitem->stileave_tr,queue_empty_hand_
				ccci_ftileave_t0,qprequeue_empty_handp meDMAitem->stileave_t0,qprequeue_empty_hand)
		}u>cldma_aCLDMA_RX_QE_eci_f
		md_ctrchueupd = 1;uct cldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pepd = 1;
	
C)
			& MODEM_CAP_TXBUSY_STOP))
			cl "%rl & 0x%psree = rrl->md_iil\nbuiltin_a_		}
_akeues, 0endi
index))
			queue_delayed_work(queve_til_eci__seq_num _tileave_t0_hif_get_by)nt);
	return count;
}

static vo_flags [quequeue_emp_clocP_CLDMHIF_IDrl-ueue *qt);
	(or(
				ccci_md_gt_dev_bx_done(queOUT->dae& nt;
 R_CMD,s);
 ueunsa (!((trepsonldma_rx_RX_QEgs);
		if (pending_gpd &&
 !(!blockingNUM] = { SO_CFGf CLDMA_TRACE
	unsigned long long to !(!blockingNUM] = { SO_CFG)U|L0x5rl-uX_Ione(queSPLIA__Nma_rx_RX_QEgs);
		if (pending_gpd &&
 !(!blockingNUM] = { BUS_CFGf CLDMA_TRACE
	unsigned long long to !(!blockingNUM] = { BUS_CFG)U|L0x02rl-_er#endiONFIG_ARCH_M] = DDR_T_64BITock, flags);
		if (pending_gpd &&
		   !(cldma_read32UL_CFGf CLDDMA_TRACE
	unsigned long long total_time = 0;
	unsignedUL_CFG)U|L0x40rl-	_RX_QEgs);
		if (pending_gpd &&
 !(!bloc NUM] = { SO_CFGf CLDDMA_TRACE
	unsigned long long to !(!blocking)NUM] = { SO_CFG)U|L0x4lunc2TISAR0uX_Ilise(quedebug IDma_rx_RX_QEgs);
		if (pending_gpd &&
 !(!blockinNUM] = { et >= ID_ENndex));
P_BUS/*nd fig MTUx] +y_>gp93a_rx_RX_QEgs);
		if (pending_gpd &&
 !(!blockinNUM] = { eL_MTU_SIZEc NUM] = { MTU_SIZE)d_ctrlone(quensiy_>gp93a_rx_RX_QEgs);
		if (pending_gpd &&
 !(!bloc NUM] = { SO_CFGf CLMA_TRACE
	unsigned long long to !(!blockingNUM] = { SO_CFG)U|L0x1unc2TISAR0freCLDMA_RX_QE *cci_
		md_ctrchueupd = 1;uct em->skb c__);
				dma_pool_fr	(&queuegpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pepd = 1;
	
C)
			& MODEM_CAP_TXBUSY_STOP))
			cl "%rl & 0x%psree = rrl->md_iil\nbuiltin_a_		}
_akeues, 0endiLMA_TRAone(qucldm_OR_INTEndifd_ctrl->gpd_dmapool,
					item->gpd, item->gpd_addr);
			
		 et, ccci_akeues,ma_rx->gpd, item->gpdL;
	stLENndex, -1, 0, )kfree(item)em->gpef CLDswitchree-_u_tileave_t0, [i]+eck MA_TRACEg		ite	qu ACK iakeusigned long long total_time = 0;tileave_t0, [i].& (1 <= 0;tileave_t0, [i]._ctrl->trl->wakeup_sr	MA_TRACEg		ite	qu ACK iakeu_bksigned long long to !(!blockingtileave_t0, [i].& (1 <= 0;tileave_t0, [i]._ctrl->trl->wakeup_sr}>G->gpd, item->gpdL;
	stLENndex, -1, r, )kfree(item)em->gpef CLDswitchree-_u_tileave_tr, [i]+eck MA_TRACEg		iterqu ACK iakeusigned long long to !(!blockingtileave_tr, [i].& (1 <= 0;tileave_tr, [i]._ctrl->trl->wakeup_sr}>Gdelqt;
 ,s);
 rl->va_rxwmb(
			
		 *cci_aN  R0 & CLDrmCLDMATxr		tgpd
iN  be	 *cci			((tsk *dwoERATIONtileave_t0, _ *cci			erridetileave_t0, __LINE__|= P_CLDMA_IP_BUSY);
l-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONSO_of(RT			CL P_CLDMA_IP_BUSY);

			MA_TRACE
	unsigned long long total_time = 0X_INT_DONSO_of(RT			C);lX_Ila_write32(md_etileave_tr, __LINE__|= P_CLDMA_IP_BUSY);
l-	X_Ione(queL2 IP_B,eL;
	struct lurce_ptr_lx, tx_intst_lo_time[queue->index] - totk, flags);
		if (pending_gpd &&
		   !(cldma_read32t cldma_ringpd = NULLDMA_IP_BU|LDMA_APm = NULL;
	struct cld|LDMA_APm = NUL_ptr_il-ueue *qu_RX_QEgs);
		_header *ccci_h;
	int count = 0;ma_ringpd = N_CLDMA_IP_BU|LDMA_AP_CLDMA_L;
	struct 			|LDMA_AP_CLDMA__ptr_il-	X_Ione(queON  L3lx, tx_intst_lock, flags);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3cldma_r P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3& MCa1< P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3R Mma_r P_CLDMA_IDMA_P_Brl-	_RX_QEgs);
		if (pending_gpd &&
		   !(cl
_TX_INT_DONE3R MCa1< P_CLDMA_IDMA_P_Brl-	U?_la?_itF cldma_tgpd _CAP_TXBUSY_STOP)) {
			if (quaddr);
		}ueCLDMA_flags cd =ON Q_eci__else)
		md_ctrchueupd = 1;uct 
		md_ctrl->tSO_CFGl-quU&queuegpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pepd = 1;
	
CX_Ire- *cci_	unsiy_lock, fla_eci_fAP_TXBUSYpd = 1;
	
				md_ ccci_ate(md_SYpd = 1;
	
	SO_CFG_r MA_TRACE
	unsigned long long to !(!blockinNUM] = { SO_CFG);gp		if(SO_CFG_&L0x1us=*)skb->dadelqs);
 	if (!((teudct(sstep__locuuffer_ptr_saved =
				dma_map_single(
"Ene(queAPeOUT	unsiy_id(ed. R += rxbycact(sce:qrote.tSO_CFGx);
xree = rSO_CFG);gpdone(qud> 8a] += rxb_ate(md_+eck _RX_QEgs);
		if (pending_gpd &&
 !(!bloc NUM] = { SO_CFGf CLDMA_TRACE
	unsigned long long to !(!blockingNUM] = { SO_CFG)U|L0x05)->G(	fre
delonRAT			OUd_ctma_fg lonl->c *oppe2(md_CLDMA_flags [queq			pef CL)
		md_ctrchueupd = 1th /um DIRECcci_mdirf (item->skb == NULL) {
				CCCI_ERLOG(mdid, TAG,
					"%s:alhannel */
kb c__);
				dma_pool_fr	(&queuegpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pepd = 1;
	
C	gpd-ircldmaUT#ifdef->gpd, item->gpdL;
	stLENndex, -1, 0, )kfree(item);d_ctrl->gpd_dmapool,
					it0, [i].l->wa>gpd_addr);
				kLOG(mdqueue-chg(&> queuking)_tileave_t0, [i].LDMA_APskb_free-_!kthrea&queue->ring_lock, th / 8)
eck ;tileave_t0, [i]._ctrl->imkags);
e;tileave_t0, [i]._ queuewakags);
e;tileave_t0, [i].nentely.
	k ;tileave_t0, [i]._ctchannel,
			LO);
	PACKET_HISTORY etPTH;
e;tileave_t0_hif_get_by__ qhitgpdy[cci[i]>rn_cnt
				k	

		ccc->budachr> queuags,king)_tileave_t0, [i].LDMA_APskb_free-_!kthre / 8)
cldma_rex);
		}
#if MD_GENERATION =ags(gpd,		CCCC_RX_QEgs);
8(&ending_gpd &&
	p meDMA_APMA_TRACE
	8(&ending_gpd &&
	p mtclllll& ~0x1unc	me pdn_dexeave_t0, [i].LDMA_APskgpd_clllll	!apool,
			_B		DMA_APMA_TRAt					iteength / 8tr *) lunc_mll_RX_QEgs);
16(&endingue_broadcT, 
p me 0unc	me pdn_ags(gla5Mcldma_rve_tileave_tileags(gla5Mtileavvags(gla5(mdid, TAGleaI_DEBI_DEBU?_la?_itF cldma_tgpd _CAP_TXBUSY0, [i].l->wa>gpd_AGleaddr);
				}OG(md_ctrn",
dircldmIN#ifdefdma_timeout_lock,
	rpd,		def->gpd, item->gpdL;
	stLENndex, -1, r, )kfree(item);d_ctrl->gpd_dmapool,
					itr, [i].l->wa>gpd_AGleaddr);
				kLOG(mdqueue-chg(&> queuking)_tileave_tr, [i].LDMA_APskb_free-_!kthrea&queue->ring_lock, th / 8)
eck ;tileave_tr, [i]._ctrl->imkags);
e;tileave_tr, [i].l= temp_timkags););
	PACKET_HISTORY etPTH;
e;tileave_t0_hif_get_by_r qhitgpdy[cci[i]>rn_cnt
				k	

		ccc->budachr> queuags,king)_tileave_tr, [i].LDMA_APskb_free-_!kthre / 8)
cldma_rrtrl->cldma_timeout_lock,
		ags(gpd,		CCCC_RX_QEgs);
8(&rnding_gpd &&
	p mectrl,unc_mll_RX_QEgs);
16(&rndingue_broadcT, 
p me 0unc	me pdn_ags(gla5_ *)item->data_bbags(gla5nel,
[queue->in	
			_eci__= (s_pox, txeags(gla5MtileavI_DEBI_DEBU?_la?_itF cldma_tgpd _CAP_TXBUSYr, [i].l->wa>gpd_AGleaddr);
				k		ccc->budachr> queuags,king)_tileave_tr, [i].LDMA_APskb_free-_!kthre / 8)
cldma_rrtrl->cldma_timeout_lock,
		ags(gpd,		CCCCpdn_ags(gla5_0		item->data_bbcount++;
			C		tgpd = (st_requuuu)_tileave_tr, [i]ue->in	/*whichr		tgp*d_ctLDd)
			& MODEM_CAP_TXBUSY_STOP))
			cldma_q	_qu a5_itemtatiR0 & CLDM_lockree = req->>Ct_tx_done(struct weavvags(gla5(md			DMA_FROM_DEV= req->est *req;
	struc			if (dm= req->_mapping_eCCpdn_ags(gla5_0		item->data_bbbuffer_ptr_saved =
				dma_map_single(
				r_ptr			DMA_FROM_DEy_id(md_ctrl->md_id),
			)t;
		}
		CCCCCI_DEB	vags(gtem);
				return;
			}
			spspin_unlock_irqrestoreng->pkt_size;
			gpd->gpe(
				r =
				dma_map *_lockkbags(gla5nepd_se_lockkb_data_ptr(gpd,ags(gla5Me_lockkbM] =er_ptr_saved);
					gpd->data_allow_len = ringng->pkt_size;
			gpd->gpe(
				r =
				dma_map *_lockkbags(g*/
			if (i == 0)
				first_itbbuffer_ptr_savede(
				r =
				dma_map_single(
				r_UM] = { 0 };

	total_time =)t;
		}
		CCCCCI_DEB	veout_lock,		iteength / 8rr *)(skb_p	ags(g*/
			if (i == 0)
				tileavI_DEBI_DE}BUc, 1, 0) == ln++;
			C *opeef CL)
		md_ctrchueupd = 1th
		md_ctrchueuqno)(s /um DIRECcci_mdirf (itcldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pepd = 1;
		uct ERROR,>	if (q
		md_ctr	dma_pool_fr
C	gpd-ircldmaUTodat	nor>=dL;
	stLENndex, -1, 0, )	DMA
		req -uffer_pt_INVALID_L;
	stINDEX;
C	gpd-ircldmINodat	nor>=dL;
	stLENndex, -1, r, )	DMA
		req -uffer_pt_INVALID_L;
	stINDEX;

C	gpd-ircldmINkb->dadellise(questruct curceL;
	struct lx, tx_interal,
d_ctrl->gpd_dmapool,
					item->gpd, item->gpd_addr);
				_RX_QEgs);
		_header *ccci_h;
	int count = 0;Sa_ring *ring)
{_CLDMA_IP_BUSY));
		qno+teming *ring)
{_CLDMA_L;
	struct cldma_rgpd *gpqno+ev_gpd = N>ring_lock, flagsllioc_overrideetileave_tr, __LINE__)_((ttg~(P_CLDMA_IP_BUSY);
e& ));
		qno+tagslldobd->n_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TISAR0,
		X_INT_DONSO_ofOP			CLMA_TTX_INT_A_IP_BUSY);
e& ));
		qno+t; CLDMA_TRACE
	unsigned long long toP_L2TISAR0,
		X_INT_DONSO_ofOP			C);lX_Ila_write32(md_euedma_r MA_TRACE
	unsigned long long to !(!blockinggNUM] = { SO_of(dwore& ));
		qno+		CCC)
			& MODEM_CAP_TXBUSY_STOP))
			cldma_qu *opMR==	unsi & CLDM_l,emd_ctr=%xI_ERROR				cldma_wrqno)_	if = reqERRORee(				} gpd->gpdma_ *)sk;e-	U?_la?_itF cldma_tgpd _CAP_TXBUSY_STOP)) {
			if (quaddr);
		E}BU
		req 0;, 1, 0) == ln++;
			C *CK ief CL)
		md_ctrchueupd = 1th
		md_ctrchueuqno)(s /um DIRECcci_mdirf (itcldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pepd = 1;
		== NULL) {
				CCCI_ERLOG(mdid, TAG,
					"%s:alock,
	rpd,		 c__);
				dma_pool_fr	 /um MD_of(dEd_fl 0) efr
C	gpd-ircldmaUTodat	nor>=dL;
	stLENndex, -1, 0, )	DMA
		req -uffer_pt_INVALID_L;
	stINDEX;
C	gpd-ircldmINodat	nor>=dL;
	stLENndex, -1, r, )	DMA
		req -uffer_pt_INVALID_L;
	stINDEX;

C	gpd-ircldmINkb->dadel_eci_MR==md_ct	if (i(md_euLOG(mdqueue-chg(&> queu&tileave_tr, [	no].LDMA_APskb_free-_!kthra&queue->ring_lock, th / 8)
eck tileave_tr, [	no].LDMrl->imkags);
etileave_tr, [	no].l= temp_timkags););
	PACKET_HISTORY etPTH;
etileave_t0_hif_get_by_r qhitgpdy[cci[	no]>rn_cnt
				k	
		ccc->budachr> queuags, _tileave_t0, [	no].LDMA_APskb_free-_!kthr / 8)
cldma_rtrl->cldma_timeout_lock,
		ags(gpd,		CCC_RX_QEgs);
8(&rnding_gpd &&
	p mectrl,unc_ml_RX_QEgs);
16(&rndingue_broadcT, 
p me 0unc	meags(gla5nel,
[queue->i
			_eci__= (s_pox, txeags(gla5Mtile}>dadelone(que& CLDMurceRe->index, tx_interal,
d_ctrl->gpd_dmapool,
					item->gpd, item->gpd_addr);
				_fl 0) e(md			DMfsm>skb)_fl 0) eck();
#endif
		ccc	 pdn_dex 0) e( *)WAITol,
TO_ofOPodatdex 0) e( *)G(dEDe->idatdex 0) e( *)INVALID)bd->n_ctrl->CEg		iterqu ACK iakeusigned long long to !(!blockingetileave_tr, [	no].& (1 <= 0; tileave_tr, [	no].LDMrl->(gpd,l,
				md_cctrl->cldma_ap_pdn_base, CLDMA_AP_L2TISAR0,
		X_INT_DONSO_of(RT			CLMA_TTX_INT_A_IP_BUSY);
e& ));
		qno+t; CLDMA_TRAcldma_a_header *ccci_h;
king)int count = 0;ma_ring *ring)
{_CLDMA_IP_BUSY));
		qno+teming *ring)
{_CLDMA_L;
	struct  _rgpd *gpqno+ing *v_gpd = N>ring_lock, flagslllMA_TRACE
	unsigned long long toP_L2TISAR0,
		X_INT_DONSO_of(RT			C);lX_Ila_write32(md_eeetileave_tr, __LINE__|=ing *ring)
{A_IP_BUSY);
e& ));
		qno+t; CL}e-	U?_la?_itF cldma_tgpd _CAP_TXBUSY_STOP)) {
			if (quaddr);
		E}BU
		req 0;, 1, 0) == lnli->iuct E		md_ wDkeup_count++;
			Creq->gpd;
		if (item->	if (0uX_Ilo NOT touch=	unsi HW ad_ctrpowctronata md_eX_Iepd };
=	unsi ] += rxby] +=on>g/	
CX_IreCCCI_EIRQ	md_edma_r reCCCI_cldm_OR_INTERV_RX_QEldmE 1,e->ringisr<= 0;tileave_t_RX_QEldmE &&
	p "X_INT_DO"dex]
		= );gp		if	if)bd->nuffer_ptr_saved =
				dma_map_single(
	"reCCCI_EX_INT_DOEIRQ(%d) UM] = 			cldma_wOR_INTERV_RX_QEldmE 1,e	if)		CC
		req 	if (q}ock, fladise(qucldm_OR_INTE
		BU
		req 0;, 1, 0) == ln++;
			CgNE__mpd _
		md_ctrchueupd = 1th
		md_ctrchueuqnof (itcldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pepd = 1;
		
		md_ctr	dma_pool_fr
C	gpd	nor>=dL;
	stLENndex, -1, r, )	DMA
		req -uffer_pt_INVALID_L;
	stINDEX;

if (ret >= 0 || ret == -CCCI_ERR_DROP_"gNE_ mpd rona& CLDM_lsstep_%pmd_c
EUEno)__tileave_tmd_[	no].e->indrx] = s)->Gd_ctrl->gpd_dmapool,
					item->gpd, item->gpd_addr);
			pdn_dexeave_tr, __LINE__) ));
		tileave_tr, [	no].& (1 )	DMAe->indrx] = seri ccci_ate(md_ctqno+		CU?_la?_itF cldma_tgpd _CAP_TXBUSY_STOP)) {
			if (quaddr);
		E
		req 0;, 1, 0) == ln++;
			Ccldma_room_
		md_ctrchueupd = 1th
		md_ctrchueuqnof (itcldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pepd = 1;
	
C	gpd	nor>=dL;
	stLENndex, -1, 0, )	DMA
		req -uffer_pt_INVALID_L;
	stINDEX;
C
		req tileave_t0, [	no].nentel;, 1,delonRATrun
		is atic, skbmnd Iexf = * asA
 *STATflushEelsesi%d/t
(md_CLDMA_flaSTOP) [quef
		md_ctrchueupd = 1;uct cldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pepd = 1;
	_error(
				ccci_md_get_dev_b
		md_ctrchueuicnt
				k	
		md_ctrl->t	if (ql->t	ifry_r 100ng
_error(
				ccci_m(u32 *) ccc	ifry_r 5 (qgpd->gpdmary_>)skb->darely.
MA_TRACE
	unsigned long long to !(!blockingNUM] = { SO_of(dworueue->in(ing)
{A_IP_BUSY);
e& 	if)bdma_l&&slllMA_TRACE
	unsigned long long toP_L2TISAR0,
	NUM] = { NUM] =x_interas=*)skb->dad)
			& MODEM_CAP_TXBUSY_STOP))
			cldma_(u32 i rxemd_ctrlismoff,t	ifry=_l,e { NUM] =x_interx);
x,e { >riof(dwox);
xree = rc	ifryf CLDMA_TRACE
	unsigned long long toP_L2TISAR0,
	NUM] = { NUM] =x_interaf CLDMA_TRACE
	unsigned long long toP_L2TISAR0,
	NUM] = { SO_of(dworagslll as avail}			_fA_TR(20)		CC
		ry--;c__);
P_BUSdeltouch=ta u32 i to	flush_ON  0)
		 & 0xta to	DOEmd_edma_r MA_TRACE
	unsigned long longign		   !(cldma_read32UL_of(dworueu->gpd, item-(ing)
{A_IP_BUSY);
e& 	if)bdat>gpdL;
	stLENndex, -1, r, )keue-ee(item)->in(ing)
{A_IP_BUSY);
e& 	if)b_uct cldi))x]+dad)
			& MODEM_CAP_TXBUSY_STOP))
			cldma_(ta u32 i txq=_llism_LINE_,oceagsDOErxecollect!"ct[)		CCC;
			CgNE__mpd __TXBUSY_Spd = 1thi
				}OG((qgpd->gpdmary_>)skb->darely.
MA_TRACE
	unsigned long longign		   !(cl0,
	NUM] = { er_of(dworueue->in(ing)
{A_IP_BUSY);
e& 	if)bdma_
sk    datMA_TRACE
	unsigned long long toP_L2TISAR0,
		X_INT_DONNUM] =x_interas=*)skb->dad)
			& MODEM_CAP_TXBUSY_STOP))
			cldma_(ta u32 i txemd_ctrlismoff,t	ifry=_l,e { NUM] =x_interx);
x,eor(Triof(dwox);
x,e { >riof(dwox);
xree = rc	ifryf			&queue->cldma_tx_work, msecs_to_jiffies(0)NUM] = { 0 };

x_interaf CLDMA_TRACE
	unsigned long longign		   !(cl0,
	NUM] = { er_of(dworf CLDMA_TRACE
	unsigned long long toP_L2TISAR0,
	NUM] = { SO_of(dworagslll as avail}			->in(dmary_% fl		dma_ndexd)
			& MODEM_CAP_TXBUSY_STOP))
			cldma_(ta u32 i txeism_LINE_,o	ifry=_l,e { NUM] =x_interx);
x,eor(Triof(dwox);
x,e { >riof(dwox);
xree = rc	ifryf			&queue->cldma_tx_work, msecs_to_jiffies(0)NUM] = { 0 };

x_interaf CLDMA_TRACE
	unsigned long longign		   !(cl0,
	NUM] = { er_of(dworf CLDMA_TRACE
	unsigned long long toP_L2TISAR0,
	NUM] = { SO_of(dworagslll_fA_TR(20)		CCC
		ry--;c__);
				k			if	ifry_rma_l&&			&queue->cldma_tx_work, msecs_to_jiffies(0)NUM] = { 0 };

x_interar *)skb->dauffer_ptr_saved =
				dma_map_single(
	"%s:lqt;
 md txerl->i_id(ed.ree trl->md_id),
	;
			Cueue_empty_handl->md(kb-_);
				dmawx]
		= );gpdone(qud> 8a] += rxb_ate(md_+eck(md_ctrl->m)
			& MODEM_CAP_TXBUSY_STOP))
			cldma_(%s:lmd txerl->ree trl->md_id),
}
				md_ copepd = 1;
	
CX_I6.l_eci_Mmd_ct	if (i(md_e_flags [queq			pef CL)_TXBUSY_Spd = 1thaUT#;
xv_by * se,
eeism_ _id,mnd di(!((tbetweeq tilpowct_offMurceu32 i IRQ.by * ad_ctr
 *telyaeu32 i IRQ,rn",
 *powctrofrror befeq->u32 iby * sasklkbl->c {
			md1thse, sasklkbl
iN  tely0ctma_fue->d_ctu32 iby * ] += rxb,Murcesours{
			mdsstep& CLDMto	C dma R_CM.by * T	is 
iN  [quvDMur HWO=0 R_CMDi%dmd_ct	if (i(urcecaSTATaa& CLDby * bed_ct *oppe2.
so_wATflush R_CMDe,
eeto	kiN  		is missdwoERATeRe->index, tx_int.ERATIONtilegs [queq			pef CL)_TXBUSY_Spd = 1thINndi
index		md_hwa_eci_fAP_TXBUSYif
		ccc, 1,del		is aspd_flagsi%side_tx_donel->wa>gpdATIO 0) == ln++x		md_pd,lbdout_loce	qureCCCI_(count++;
			C		tgpd = (st,		== NULL) {
				CCCI_ER	qureC
static iskroadc *skb,k	
		md_ctrl->tioc_ovtx_ide;uct cldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			petx_donepd = 1;
		== NULL) {
		hannel */
kb tatic iskb_shueedet_by *t_by =iskb_sht_by(la5Mtilln++xudl-rq);_b== NULL) {
		hbnel bd;_b== NULL) {
				CCCI_ER	qureC_bd;__error(
				ccci_m(u32 *) ccc(&queue- quehe->(i(m- quehl-ueue *qt);
	or(
				ccci_m(u32 *) ccc- queh->cldma_time quehe->(i(m)la5nepd_s;_b=kb_pull(la5
sttem->sdma_time quehe->(ii)_queue *qudelnetwtep_foe>csourhasAIOC ovtx_ideoceagst_locif (ret >= 0 || ret == -CCCI_ERR_DROP_"SGIO, _CM=%p, -rq)s=_l,elen=_l,ehe->len=_lree = r	qureC(gpd,,rn_by->ndl-rq)s, la5nel,
eDMAikb_he->len(la5Mil-	X_Ilink firt Ba to	la5's 0)
		_loc	qureC_bd(mdqueue-chg(&> queu&	qureC(gb*)(ska&queue->ring_lock, th / 8)
eckX_Ilink a_tg Ba to	-rq)s' 0)
		_loc->gpdxudl-rq)(md-1;+xudl-rq)gpdn_by->ndl-rq)s;+xudl-rq)ee(item)
		md_ctrl->t-rq)T, 
;gpdCLDMA*-rq)T,
				def_get_udl-rq)(mmd-1kb->dad-rq)T, 
 =iskb_he->len(la5M;>dad-rq)T,
		 =iskbnepd_s;_bk(md_ctrl->mAikb_-rq)TtA*-rq) =in_by->-rq)s ++xudl-rq);_>dad-rq)T, 
 =iskb_-rq)T(gpd,-rq)M;>dad-rq)T,
		 =iskb_-rq)T,
		es, -rq)M;>da}			tbd(md	qureC_bd(gpd,		CCif (ret >= 0 || ret == -CCCI_ERR_DROP_PASGIO, BM=%p, -rq)_l,e-rq)T, 
=%		cldCtb*)(sk	xudl-rq),e-rq)T, 
M;>daX_Iupd) e(Ba md_e		qureC_bd(gtem);
				return;
			}
			    in_unlock_irqre->pkt_size;
			gpd->gpAP_TXBUSYif
		c,>dad-rq)T,
		,e-rq)T, 
, A_APmOtr_saved);
		gpd->data_allow_len =->pkt_size;
			gpd->gpAP_TXBUSYif
		c,>dad	qureC_bd(gtem);
				return;
			k_timeouuffer_ptr_saved =
				dma_map_single(
	r_UM] = { 0 };

	total_time 
		req -1;>da}			) {
		hbn		iteength / 8tb*)(sk		qureC_bd(gtem);
				return;
			k				_RX_QEgs);
16(&ebingue_broadcT, 
p me -rq)T, 
M;>daebingnon	));d(md1;>daX_Is[queuEOLma_rx_pdn_base,
	8(&ebingbpd &&
	p meDMA_MA_TRACE
	8(&ebingbpd &&
	p m)b_u~0x1unc	m
		 *epy_>gward md_e		qureC_bd(mdqueue> queu	qureC_bd(g> que.nexf =thra&queue->ring_lock, th / 8)
eck}		
		 et,EOLma_rxpdn_base,
	8(&ebingbpd &&
	p meDMAMA_TRACE
	8(&ebingbpd &&
	p m)b|L0x1uncrex);
			qureC(gpd,eckX_Iupd) e(_CMD_lock, flags);
		i&endingue_broadcT, 
p me la5nel,
;
	_error(
				ccci_m(u32 *) ccc-RX_QEgs);
8(&endingnetifp me e quehngue_b[0])_queue *qudelmp1 1,lmp2 me ro 1D_loc	gpdla5nemaep_& UIDMASK_timeoglagsuidataskrERRORee;>daendingps
[quexflag;ak}-gpendingnon	));d(md1;>d
		 et,HWOD_locd_ctrl->g_CAP_TXBUSY_STOP)) {
			if (q
			pdn_dexeave_tt, __LINE__) ));
		tx_done(struc)rx_pdn_base,
	8(&ending_gpd &&
	p meDMA_MA_TRACE
	8(&ending_gpd &&
	p mtb|L0x1uncrU?_la?_itF _CAP_TXBUSY_STOP)) {
			if (q
			delmaep_) {
				CCCI_EasAavid(e(que_loc	qureC(gla5(mdla5		E
		req 0;, 1,del		is aspd_flagsi%side_tx_donel->wa>gpdATIO 0) == ln++x		md_pd,lut_loce	qureCCCI_(count++;
			C		tgpd = (st,		== NULL) {
				CCCI_ER	qureC
static iskroadc *skb,k	
		md_ctrl->tioc_ovtx_ide;uct cldma_t) {
		hannel */
kb tatic igpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			petx_donepd = 1;
	_error(
				ccci_m(u32 *) ccc(&queue- quehe->(i(m- quehl-ueue *qtrex);
			qureC(gpd,eckX_Iovtx_ideoxudr / AIOC  ettd_ct_loc	gpdioc_ovtx_ide_&L0x8skb->dadelback		trudr / AIOC  ettd_ct_locc	qureC(gioc_ovtx_ide_=L0x8sb|L(!!(ending_gpd &&
	_&L0x8skd);
		gpdioc_ovtx_ide_&L0x1ndexdending_gpd &&
	_|=L0x8s);
	IP_BUSY)ending_gpd &&
	_&=L0x7Feck}		
		upd) e(_CMD_lo);
	or(
				ccci_m(u32 *) ccc- queh->cldma_time quehe->(i(m)la5nepd_s;_b=kb_pull(la5
sttem->sdma_time quehe->(ii)_queue *qu	qureC(gtem);
				return;
			}
		    in_unlock_irqre->pkt_size;
			gpd->gpAP_TXBUSYif
		c,>dadla5nepd_se la5nel,
e A_APmOtr_saved);
	gpd->data_allow_len =->pkt_size;
			gpd->gpAP_TXBUSYif
		c,>dad	qureC(gtem);
				return;
			k_timeouffer_ptr_saved =
				dma_map_single(
	"UM] = { 0 };

	total_time
		req -1;>d}
				md_t					iteength / 8tr *) 	qureC(gtem);
				return;
			k;
	_RX_QEgs);
16(&endingue_broadcT, 
p me la5nel,
;
	_error(
				ccci_m(u32 *) ccc-RX_QEgs);
8(&endingnetifp me e quehngue_b[0])_queue *quendingnon	));d(md1;>d
	ERATe et,HWOERATeSTAT_STOP)) {
			if (q to	aCLDMA_id,mnd di(onct
						md_ cop.by * T	is f (q must ERvtx T_CMD ettd_c,EasAevenby * t
					m_ _esume opera(!((ldma_restaiN  can	 *cci_sk *dwo nexfby * HWO=1 T_CMDerrlast T_CMDwasAjust finished.ERATIONd_ctrl->g_CAP_TXBUSY_STOP)) {
			if (q
			pdn_dexeave_tt, __LINE__) ));
		tx_done(struc)rx_pdn_base,
	8(&ending_gpd &&
	p meDMA_MA_TRACE
	8(&ending_gpd &&
	p mtb|L0x1uncrU?_la?_itF _CAP_TXBUSY_STOP)) {
			if (q
			delmaep_) {
				CCCI_EasAavid(e(que_loc	qureC(gla5(mdla5		E
		req 0;, 1, 0) == ln++;
			C k *M_DEV
		md_ctrchueupd = 1th
eueqno,b tatic iskroadc *skb,h
eueskb_-rom_pool,h
euebf (qdwof (itcldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pepd = 1;
		== NULL;
			C		tgpd = (st;_b== NULL) {
				CCCI_ER	qureC (ql->t	if[queue-dma_time quehe->(i(- quehl-	
		md_ctrl->tioc_ovtx_ide[queue-c__);
				dma_pool_fr	
		md_ctrl->t	queytes[queue-dma_time que
				re & 0x1
		;
		if did, TA_er#endiMA_APmRACEe-dm) == c__);
				dma_	dma_	ast_[quvDloc(mdDMA_APm cldma]->n= { 0 };e-dm) == c__);
			
euesamplDloc(mdDMA_APm cldma] = { 0 };e-dm) == c__);
			
euesamplDleytesdDMA_APm cldma] = { 0 };e-c__);
				dma_	dma_totalloc(ma_w0fr	
		md_ctrl->t	qux, txall-ueue *qt);
#endiMA_APmRACEe-totalloc(ma_ws{
		2(md_ctrl-	pdn_	ast_[quvDloc(md	no]>rma_ndex	qux, txala_w0fr	IP_BUSY	qux, txala_wtotalloc(ma-_	ast_[quvDloc(md	no]l-ueue *qtr	ccci_f&- quehp me ltem->sdma_time quehe->(ii)_qu;
	return count;
}

static vo->in(jiffies[- tileave_t0eue_empdm)mp) / HZ >DMA_return count;
}

static ndexmod_			spu_tileave_t0eue_empty_handpDMA_jiffies[+	return count;
}

static  * HZrl-	tileave_t0eue_empdm)mpa_wjiffiesl-ueue *qtr	gpd	nor>=dL;
	stLENndex, -1, 0, )	b->darely.
-uffer_pt_INVALID_L;
	stINDEX;
C	goto	__EXIT_FUN;ak}-gpioc_ovtx_ide_=L0x0;oc	gpdla5_-rom_pooll&&	skb_he->room_la5Mc0		iET_SKB_PAD)bd->n
		;
		if d(dma_time que
				re & 0x1)=kb_push(skb,k	MAitem->sdma_time que
				re & 0kd);
		gpdlikely(
		;
		inepe->atag== 0		ufferBUF_MAGIC	)ongCioc_ovtx_ide_=L
		;
		ineioc_ovtx_ide;DMAikb_pull(la5
sttem->sdma_time que
				re & 0kd);
(md_ct	CCif (ret >= 0 || ret == -CCCI_ERR_DROP_PA k * reCCCI_:	skb %peSTAT#enault value!md_ctla5M;>cc- queh->c*ldma_time quehe->(i(m)la5nepd_s;_b= (st_r _tileave_t0, [	no];oc	queytes[qula5nel,
;>co	ifry:>Gd_ctrl->gpd_dmapooltx_donel->wa>gpd_addr);
				delq *STATd_dmapoEasAnetwtep_reCCirATaaf (q in	 oftd_dROP_ * caSTATaapot / ialade->l (q
ck *d_cif (ret >= 0 || ret == -CCCI_ERR_DROP_"gelyaeTx_reCrona&%d eave=_l,e	queytes[qu%Xmd_c
EUEno)_tx_donenentel,e	queytes);oc	qureC[quest *req queue;oc	gpdtx_donenentel_>)sl&&		qureC(gla5(m		item->data->pkt_siince	qu eq_num  ret == -CCCI_ER_tileave_t0_hif_get_byROP_Pldma_time quehe->(i(m)la5nepd_s
				delqt;
 ,s);
 rl->va_rxxwmb(
				tx_donenentel--;c_>est *req;
	strucut_loce	qureCCCI_(= (st,		ad	qureCctla5,tioc_ovtx_ide;nc	m
		 *epy_>gward md_e	est *req queue}
			s) {
			->wa *ep__>gward(est *req;
	str) 	qureCk;e-	U?_la?_itF cldma_tgpd _Ctx_donel->wa>gpd_addr);
				delupd) e(logD_lo);
	return count;
}

static voindex))
			quprequeue_empty_hand[tx_done(stru]ee;>dae quechunnel_upd) e_packkb)ERRORe = rintileave_t0_hif_get_by.log_gech_			ipreqcOR,>&- queh)_queue *qua->pkt_siadda>ggqhitgpdy _tileave_t0_hif_get_bythaUTROP_Plx, )tx_done(stru,>&- quehe 0unc	m
	ER_ * make surATT_CMDesfue->y byDe,
eROP_ * ose,
wiTATse,
eeism_id,mnd di(oncbetweeqOP_ * portsIovtxhse, same_tx_do.OP_ * one porteismjust  ettd_ctT_CM,Murose,
 portER_ * mayDeapoE_esumedhse, tx_do.OP_ */			delp		mue}			side_of d_ctrl->gpd_dmapowtoER_ * aCLDMAdise(qd_ctIRQ	too_	dmaOP_ */			d_ctrl->gpd_dmapool,
					item->gpd, item->gpd_addr);
				pdn_dexeave_tt, __LINE__) ));
		tno+te{_->d (6293)
#er			itemIMshou		pdn_IS_iET_L;
  ret == -CCCI_ERtno+te{_a_wrqst *req, item- *cci_=af (all(md_ctrl-	euedma_r mod_			spu_qst *req, item-			sp)(skb_pjiffies[+	NUM] = CTIVE_T * HZrl-		CCif (ret >= 0 || ret == -CCCI_ERR_DROP_PAP_mdxeave_tt, __LINE_=_l,etno%d ,ch_l,emd_rm-			sp				cldma_wretileave_t0, __LINE_,eqno,b  reqE queh.chunnel,e	if)		CCuedma_r eue->i}nt
				k	

ndex))
			queue_delayed_work(qu			pdn_dexeave_tt, _ *cci			cldma_r/d(_esume Txr		tgpd*d_ctLDpdn_!(MA_TRACE
	unsigned long long toP_L2TISAR0,
		
	NUM] = { er_of(dwor &0,
		
	));
		qno+tab  reqE, flags);
		i0,
		
	igned long long toP_L2TISAR0,
		
	NUM] = { er_RESUME			CLMA_TTTTX_INT_A_IP_BUSY);
e&0,
		
	));
		qno+t		CCueMA_TRACE
	unsigned long long toP_L2TISAR0,
		
NUM] = { er_RESUME			Ct		CCuX_Ila_write32(to	Cte3tATaanon-
			e(que,s);
 *d_ctL(md_ctrl->mAcctrl->cldma_ap_pdn_base, CLDMA_AP_L2TISAR0,
			NUM] = { er_of(RT			CLMA_TTTX_INT_A_IP_BUSY);
t		CCueMA_TRACE
	unsigned long long toP_L2TISAR0,
		
NUM] = { er_of(RT			C);lX_Ila_write32(md_eeeNtileave_t0, _ *cci			er1ue->i}ntL(md_ctrl->mA
	ER__ * [NOTICE] Do->t	ifreq UM] =ER__ * SKBrhasAbeeq p		munto	C lonlchui
eDMAy * Howevxb,Mpdnt, __LINE__ismdise(queDMAy * that means				md_ copy_>gpsome cISAR0,
	 * anMA_RX_Qn_loceags_esume agui
.0,
	 * T	is packagpd
iN  be	>roppe2(byD_RX_Q.0,
	 */meouuffer& MODEM_CAP_TXBUSY_STOP))
			cldma_quch=_llqno=_ll_RX_Qnmaybe	 *op,l		is packagpd
iN  be	>roppe2!	cldma_wrE queh.chunnel,eqno+		CC}e-	U?_la?_itF cldma_tgpd _CAP_TXBUSY_STOP)) {
			if (quaddr);
		E}md_ctrl->m	gpdlikely(->pkt_size;
c topd->gpAP_TXBUSYif
		ce&0,
	MODEM_C { TXnter_ofOPtab  r_STOP)ef CLDbroadcast_ 0) eck();
#e
		X_FULLdma_wraUTR_tx_done(struct weU?_la?_itF cldma_tgpd _Ctx_donel->wa>gpd_addr);
				delC dma ma_resta_ctrl_free);
#MA_TRACE
	unsigned long long toP_L2TISAR0,
		X_INT_DONer_of(dwor & ));
		tno+te{_	CCif (ret >= 0 || ret == -CCCI_ERR_DROP_PAuch=_llqno=_lleave slot 0ldma_read32UL_of(dwox);
xree = rcrE queh.chunnel,eqno,	CCueMA_TRACE
	unsigned long long toP_L2TISAR0,
		X_INT_DONer_of(dwort		CCutx_donenee_dERRORee;>da
ndex))
			queue_delayed_work(qu		(md_ctrl->mA);
#MA_TRACE
	unsigned long long toP_L2TISAR0,
		X_INT_DONt cldR0r & ));
		tno+t-		CCif (rREPEAT 0 || ret == -CCCI_ERR_DROP_PAP_ch=_llqno=_lleave slot 0ldma_read32t cldR0x);
xree = rcrrE queh.chunnel,eqno,	CCueeMA_TRACE
	unsigned long long toP_L2TISAR0,
		
NUM] = { t cldR0r)qu			pdn_++ndex))
			queue_delayed_worr 100g	cldma_r)
			& MODEM_CAP_TXBUSY_STOP))
			cldma_q	"tx eue_:Ila_
=	unsi anMA_CMD a_ctrtal_time =)
			MEM__CA_		cP_TXBUSY_STOP))
			cldma_q	"tx eue_:Ila_
=	unsi anMA_CMD a_ctrtal_time =_TXBUSY_Sopsnep> 8a a_ctrcP_CLDMHIF_IDldma_q	eUMP_FLAGr			it,d-1k;dma_r/ddma_r * aee_kernel_elay->wa ti(__FI#er_,dma_r * __LINer_, DB_OP(sDEFAULT,dma_r * "g lon", "TX eue_edebugl_time = *d_ctL(
a_r/d(_esume chunnel *d_ctLd_ctrl->gpd_dmapool,
					item->gpd, item->gpd_addr);
				Dpdn_!(MA_TRACE
	unsigned long long toP_L2TISAR0,
		X_INT_DONer_of(dwor & ));
		tx_done(struc))rl->mAcctrl->cldma_ap_pdn_base, CLDMA_AP_L2TISAR0,
			NUM] = { er_RESUME			CLMA_TTTX_INT_A_IP_BUSY);
e&0,
		
));
		tx_done(struc);-		CCif (rREPEAT 0 || ret == -CCCI_ERR_DROP_PAP__esume t, M_lsi(tsk *tla5ree = rcrrtx_done(struct weaI_DEBU?_la?_itF cldma_tgpd _CAP_TXBUSYem->gpd, item->gpd_= rcrddr);
				}O);
#endiMA_APndex] - tot>est *req;
	strucut_loce	qurl->(= (st, me 0,>&	if)		t
				k	
pdn_bf (qdwofcldma_rma_r qt;
_event_x, tx_inti(qucexclusNE_(est *rereC_ws,king)dtx_donenentel_>)sr)qu			pdn_rma_r= -ERESf(RTSYS
cldma_rrely.
-EINTR;-		CCgoto	__EXIT_FUN;ak		}O);
#endiMA_APmRACEe-		t_id,ex		md__len =Eno)_E queh.chunnel,e	if, __LINer_)		t
				k	
Cgoto		ifryqu		(md_ctrl->mArely.
-Enter;
	CCgoto	__EXIT_FUN;ak	}OG((
	__EXIT_FUN:qt);
#endiMA_APmRACEe-pdn_?_iikely(	if)_timeoufferet >= 0 || ret == -CCCI_ERR_DROP_PAt, __LINE_=_l,etno=_llism0,>rop ch_l packagp,	if=			cldma_wOR_INTERV0, __LINE_,eqno,_E queh.chunnel,e	ifM;>dae_id,ex		md__len =Eno)_E queh.chunnel,->mArel, __LINer_)			(md_ctrl->m	ast_[quvDloc(md	no]>rws{
		2(md_ctrl-	-totalloc(ma_w	ast_[quvDloc(md	no]>-wtotalloc(mt weUamplDloc(mdtx_done(stru] += (totalloc(ma+t	qux, txal)t weUamplDleytesdtx_done(stru] += 	queytes;>dae_id,ex		md_tx=Eno)_E queh.chunnel,etileave_t0, [	no].nentel,		ad	qux, txal,wtotalloc(m,		ad	queytesp me 0unc	m	gpdlamplDloc(mdtx_done(stru] >= e_id,elamplDloc(m
cldma_e_id,ex		md_tx=Eno)_E queh.chunnel,e0,e0,e0,e0,-		CClamplDloc(mdtx_done(stru],-		CClamplDleytesdtx_done(stru])qu			lamplDloc(mdtx_done(stru] queue->i
amplDleytesdtx_done(stru] rk(qu		(c__);
				k	
		req 	if ( 1, 0) == CLDMA_flaSTOP)rxq0_sasklkbkb-_);
				dma 0)
	f (itcldma_tgpd *)req->gpd;
		if  gpd->gpd_flags & 0x1)pd_s;_bl->t	if (q== NULL;
			C		tgpd = (st;__b= (st_r _tileave_tr, [0]l-	tileave_t0eue_empt_by.l) est_qdrx]oc(mdtx_done(stru]
>n= f (all(md_ctrl-	rely.
est *req;
	strucut_locerqurl->(= (st,	CCutx_donenentel,e0);gp		if	ifcldmaNCE_MORE l,
sasklkb_hi_s{
			md_CAP_TXBUSYem->gprxq0_sask)fr	IP_B pdn_?_iikely(	ifcldmLOW	MEMORY)kb->dadeRxerl->ianMAemptyex, tx_inte
iN  be	one(qulsi(tstep& CLDmd_e	est *_else)tileave_tr, [tx_done(stru]. = ser,	CCu&tileave_tr, [tx_done(stru].e->indrx] = s)->G(md_ct	CCdelone(questruct curceL;
	struct lx, tx_interal,
MA_TRAcldma_a_header *ccci_h;
	int count = 0;ma_ring*ring)
{_CLDMA_IP_BUSY));
		qx_done(struc)eming ring)
{_CLDMA_L;
	struct  _rgpd *gpqx_done(strucing v_gpd = N>ring_lock, flags_cif (ret >= 0 || ret == -CCCI_ERR_DROP_"rxq0 sasklkbl_esult 			clde	ifM;> 1,de_LIrually,elength_ismda_
=ddr)'s priv3tATargumentTIO 0) == ln++ndex		md_hif_p> 8a a_ctrc
		md_ctrchueupd = 1tr	 /um MODEM_eUMP_FLAG=ddr),h
euelengthf (itcldma_tgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pepd = 1;
		uct Ct_t_bit};
=_w0fr	
		md_ctrl->t-ircl d *gpaUTo| d *gpIN
	
C	gpdreq)aSYeUMP_FLAGr			it(item)em->gpd> 8a] += rxb_ate(md_+eck t_bit};
=_wl,
			LO	(c_	gpdreq)aSYeUMP_FLAGrL;
	st_ndext_bit};
=_w0x1;c_	gpdreq)aSYeUMP_FLAGrL;
	st__1kb->dat_bit};
=_w0x3);
		gpdlength_!ma_ndexd-ircl l,
			LO	(c_)
			MEM__CA_		cP_TXBUSY_STOP))
			cldma(%s:lt_bit};
=_w			clderl->md_iilt_bit};
;
	
C	gpd	_bit};
=_md-1kb->daem->gpd> 8apackkb)hitgpdy ate(md_+eck em->gpd> 8a			p	qugpd ate(md_+eck(md_ctrl->m->gpd, item-	_bit};
=dat>gpdL;
	stLENndex, -1, 0, )kfree(item);em->gpd> 8aest *_hitgpdy ate(md_ct[)		CCCem->gpd> 8a_gpdef CL)_TXBUSY, Ct_dirf		CCut_bit};
=d= ~ct cldi)qu		(c__)_	gpdreq)aSYeUMP_FLAGr- t_of(dwor imeouffer& MODEM_CAP_TXBUSY_STOP))
			cldma_(D> 8aAPeu32 i IRQD a_ctrtal_ti_er#endiONFIG_MTK_GIC_V3_EXTdmam_cldm_p> 8a a_ctrcAP_TXBUSYem->gpldmE 1)		t
				k	}	BU
		req 0;, 1,O 0) == dma_time quehd =opsme quehd =em->gpopsm= ime. k *M_DE_r _tile	C k *M_DEldm.gNE__mpd _r _tile	CgNE__mpd ldm.cldma_room_r _tile	Ccldma_roomldm. *opeef CL_r _tile	C *opeef CLldm. *CK ief CL_r _tile	C *CK ief CLldm.p> 8a a_ctr_r _tile		md_hif_p> 8a a_ctr,
}
	
ln++x quec		md_hif_keup_
		md_ctrchueupd = 1th
		md_ctrchueuif
		c (itcldma_tdevid,enodpd nodpdmdid, TAG,
					gpd *)req->gpd;
		i
	_error(
				ccci_md_get_dev_b,
					gpdhwet_by *mdet_by  ((tgpd->gpd_flhwet_by *)->pkt_size;
hwet_by(md_ 1;
		== NULL) {
		hwet_by *hwet_by  ((tgpd->gpd) {
		hwet_by *)mdet_bynepd =hwet_by		t
				k	em->skb-	tileavedmdkzA_FRO(ttem->sdma_timgpd *)req-), GFP_KERNEL
			pdn_dexeave(m		item->datauffer_ptr_saved-1
			cldma_(%s:A_FROetileavey_id(md_ctrl->md_id),
	
		req -1;>d}
	mccci_ftileavep me ltem->sdma_timgpd *)req-))di
index))
		opsm= &e quehd =em->gpopsl-	tileave_tmd_ 1_r md_ 1l-	tileave_tpd = 1_r pd = 1di
inodpdmdof_f(st_compati(qucnodp(item, item, "medi3tAk,mdg lon"
			pdn_nodpdm		item->dataufferBOOTUPM_CAP_TX))
			cldma_(elay->w:n_lomedi3tAk,mdg lonsi(tdtrtal_timekeave ate(md_+eck 
		req -1;>d}
	mP_TXBUSYem->gpldmE &&
	_= IRQFPmRIGGERr& NE;
	mP_TXBUSYem->gpldmE 1_r ldmEof_parse_urcata__nodp,e0);gp		ifmP_TXBUSYem->gpldmE 1_r*)skb->dauffer_ptr_saved =
))
			cl "no	C lonlldm  1_ et,i(tdtrtal_timekeave ate(md_+eck 
		req -1;>d}
_error(
				ccci_md_get_dev_b_pdn_base, CLDMA_AP_L2TISA  ((tgCLDMA__iomcc *)(hwet_byse, CLDMA_AP_L2TISA);
	mP_TXBUSYem->gp to !(!blo  ((tgCLDMA__iomcc *)(hwet_byse, CLDMA_A !(!blo);
	mP_TXBUSYem->gpmdAP_L2TISA  ((tgCLDMA__iomcc *)(hwet_byse, CLDMmdAP_L2TISA);
	mP_TXBUSYem->gpmdA !(!blo  ((tgCLDMA__iomcc *)(hwet_byse, CLDMmdA !(!blo);gp		ifmP_TXBUSYem->gpA_AP_L2TISA  		item |mingmP_TXBUSYem->gp to !(!blo  		item |mingmP_TXBUSYem->gpmdAP_L2TISA  		item |mingmP_TXBUSYem->gpmdA !(!blo  		itemkb->dauffer_ptr_saved =
))
			cl "no	C lonl] += rxby et,i(tdtrtal_timekeave ate(md_+eck 
		req -1;>d}
_d_ct	CmP_TXBUSYem->gp to !(!blo  dof_ioma__nodp,e0);gpmP_TXBUSYem->gpA_AP_L2TISA  dof_ioma__nodp,e1);gp		ifmP_TXBUSYem->gpA_AP_L2TISA  		item |mingmP_TXBUSYem->gp to !(!blo  		itemkb->dauffer_ptr_saved =
))
			cl "no	C lonl] += rxby et,i(tdtrtal_timekeave ate(md_+eck 
		req -1;>d}
_d				k	OR_INTERV0, __LINE_	erridetileave_tr, __LINE_	erridesasklkb_keup_CAP_TXBUSYem->gprxq0_sask,ingmP_TSTOP)rxq0_sasklkb, kb-_);
				dmawx]
		= );g>Gd_ctrl->gpdeup_CAP_TXBUSYem->gp) {
			if (q
			->gpd, item->gpdL;
	stLENndex, -1, 0, )kfree(

	;
			Cef CLDsma_tipdeup_CAP_TXBUSY0, [i]dma_wOR_INTERVpd = 1thaUTct[)		C->gpd, item->gpdL;
	stLENndex, -1, r, )kfree(

	;
			Cef CLDsma_tipdeup_CAP_TXBUSYr, [i]dma_wOR_INTERVpd = 1thINct[)		
	mP_TXBUSYem->gpldmE = ser}
		    A_FROMstep& CLD("md%P_TSTOP) = serldma_wWQ_UNBOUNDo| WQ_MEM_RECLAIMo| WQ_HIGHPRIdma_w1, md_ 1 +e1);gpINIT_WORK_CAP_TXBUSYem->gpldmE = s,e->ringidmE = s)		
	atomempdep_CAP_TXBUSYem->gpidmEone(qul,e1);gu;
	return count;
}

static vo-eup_			spu_tileave_t0eue_empty_handrl-	tileave_t0eue_empty_hand.	if (!((t=

	;
			Cueue_empty_handl->mdl-	tileave_t0eue_empty_hand.0)
		= kb-_);
				dmawx]
		= ;
_d				k	OR_INTERV0,ueue_delayed_work(queve_tilpd [pd = 1] rkgCLDMA*wx]
		= ;
E
		req 0;, 1,del
 *p		muni ializa(!((s whichrtakes	too_much=oc(mae,
ee*/
ln++;
			Cl) e_keup_
		md_ctrchueupd = 1f (item->skb == NULLgpd *)req->gpd;
		if ((tgpd->gpd_flags & 0x1) && req->skb)
			pepd = 1;
	
	atomempdep_CAP_TXBUSYwakeu8a rc,e0);gp && r_eci__ eq_num _tileave_t0_hif_get_by)
	
CX_Ikeupdmd_ct	if (is(md_etileave_t_gpd->gpooll= in_upool_Cte3tA(") {
				CCCI__M] ldma_->pkt_size;
			gpd->gpAP_TXBUSYif
		c,>dattem->sdma_time		md_t			), 16,e0);gp		ifmP_TXBUSY_gpd->gpooll=		itemkb->dauffer_ptr_saved =
BUSY_STOP))
			cldma_(%s-%d:in_upool_Cte3tAy_id(md_ctrl->md_i, __LINer_)			 
		req -1;>d}
	->gpd, item->gpdiET_m cldmakfree(item)INIT_LIST_HEAD _tileave_tnite	qumd_c[i].b_free-_)			 tileave_tnite	qumd_c[i].length_
			snite	quef CLDb				renumber[nite	qumd_c2ef CL[i]]TA_er#endiMA_APiET_m _BD		 tileave_tnite	qumd_c[i].gpd_ apool,
			_B				 tileave_tnite	qumd_c[i].ut_loce	qureCCCI__
			s&x		md_pd,lbdout_loce	qureCCCI_			 tileave_tnite	qumd_c[i].ut_loce	qurl->im			s&x		md_pd,lbdo	qucollect));
P_BUS tileave_tnite	qumd_c[i].gpd_ apool,
						 tileave_tnite	qumd_c[i].ut_loce	qureCCCI__
			s&x		md_pd,lut_loce	qureCCCI_			 tileave_tnite	qumd_c[i].ut_loce	qurl->im			s&x		md_pd,l	qucollect));
ue *qua-		md_tx		->wakeup_tileavep	CCu&tileave_tnite	qumd_c[i])			 ufferet >= 0 || ret == -CCCI_ERR_DROP_PAnite	qumd_c %d:_%pmd_c ip	CCu&tileave_tnite	qumd_c[i])			}
	->gpd, item->gpdiET_R cldmakfree(item)INIT_LIST_HEAD _tileave_tniterqumd_c[i].b_free-_)			 tileave_tniterqumd_c[i].length_
			sniterquef CLDb				renumber[niterqumd_c2ef CL[i]]TA	 tileave_tniterqumd_c[i].			if (d_
			sniterquef CLDb				ref (d[niterqumd_c2ef CL[i]]TA	 tileave_tniterqumd_c[i].gpd_ apool,
						 tileave_tniterqumd_c[i].ut_locerqurl->im			s&x		md_pd,lrqucollect))MAe->indrx]	->wakeup_tileavep _tileave_tniterqumd_c[i])			 ufferet >= 0 || ret == -CCCI_ERR_DROP_PAniterqumd_c %d:_%pmd_c ip	CCu&tileave_tniterqumd_c[i])			}
	->gpd, item->gpdi MODEMm cldmakfree(item)INIT_LIST_HEAD _tileave_tnormale	qumd_c[i].b_free-_)			 tileave_tnormale	qumd_c[i].length_
			snormale	quef CLDb				renumber[normale	qumd_c2ef CL[i]]TA_era_
sktileave_tnormale	qumd_c[i].gpd_ apool,
			_B				 tileave_tnormale	qumd_c[i].ut_loce	qureCCCI__
			s&x		md_pd,lbdout_loce	qureCCCI_			 tileave_tnormale	qumd_c[i].ut_loce	qurl->im			s&x		md_pd,lbdo	qucollect));
P_BUS tileave_tnormale	qumd_c[i].gpd_ apool,
						 tileave_tnormale	qumd_c[i].ut_loce	qureCCCI__
			s&x		md_pd,lut_loce	qureCCCI_			 tileave_tnormale	qumd_c[i].ut_loce	qurl->im			s&x		md_pd,l	qucollect));
ue *qua-		md_tx		->wakeup_tileavep _tileave_tnormale	qumd_c[i])			 ufferet >= 0 || ret == -CCCI_ERR_DROP_PAnormale	qumd_c %d:_%pmd_c ip	CCu&tileave_tnormale	qumd_c[i])			}
	->gpd, item->gpdi MODEMR cldmakfree(item)INIT_LIST_HEAD _tileave_tnormalerqumd_c[i].b_free-_)			 tileave_tnormalerqumd_c[i].length_
			snormalerquef CLDb				renumber[normalerqumd_c2ef CL[i]]TA	 tileave_tnormalerqumd_c[i].			if (d_
			snormalerquef CLDb				ref (d[normalerqumd_c2ef CL[i]]TA	 tileave_tnormalerqumd_c[i].gpd_ apool,
		