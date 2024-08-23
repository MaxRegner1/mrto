/*
 * net/sched/sch_generic.c	Generic packet scheduler routines.
 *
 *		This program is free software; you can redistribute it and/or
 *		modify it under the terms of the GNU General Public License
 *		as published by the Free Software Foundation; either version
 *		2 of the License, or (at your option) any later version.
 *
 * Authors:	Alexey Kuznetsov, <kuznet@ms2.inr.ac.ru>
 *              Jamal Hadi Salim, <hadi@cyberus.ca> 990601
 *              - Ingress support
 */

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/rtnetlink.h>
#include <linux/init.h>
#include <linux/rcupdate.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/if_vlan.h>
#include <net/sch_generic.h>
#include <net/pkt_sched.h>
#include <net/dst.h>
#include <trace/events/qdisc.h>

/* Qdisc to use by default */
const struct Qdisc_ops *default_qdisc_ops = &pfifo_fast_ops;
EXPORT_SYMBOL(default_qdisc_ops);

/* Main transmission queue. */

/* Modifications to data participating in scheduling must be protected with
 * qdisc_lock(qdisc) spinlock.
 *
 * The idea is the following:
 * - enqueue, dequeue are serialized via qdisc root lock
 * - ingress filtering is also serialized via qdisc root lock
 * - updates to tree and tree walking are only done under the rtnl mutex.
 */

static inline int dev_requeue_skb(struct sk_buff *skb, struct Qdisc *q)
{
	q->gso_skb = skb;
	q->qstats.requeues++;
	qdisc_qstats_backlog_inc(q, skb);
	q->q.qlen++;	/* it's still part of the queue */
	__netif_schedule(q);

	return 0;
}

static void try_bulk_dequeue_skb(struct Qdisc *q,
				 struct sk_buff *skb,
				 const struct netdev_queue *txq,
				 int *packets)
{
	int bytelimit = qdisc_avail_bulklimit(txq) - skb->len;

	while (bytelimit > 0) {
		struct sk_buff *nskb = q->dequeue(q);

		if (!nskb)
			break;

		bytelimit -= nskb->len; /* covers GSO len */
		skb->next = nskb;
		skb = nskb;
		(*packets)++; /* GSO counts as one pkt */
	}
	skb->next = NULL;
}

/* This variant of try_bulk_dequeue_skb() makes sure
 * all skbs in the chain are for the same txq
 */
static void try_bulk_dequeue_skb_slow(struct Qdisc *q,
				      struct sk_buff *skb,
				      int *packets)
{
	int mapping = skb_get_queue_mapping(skb);
	struct sk_buff *nskb;
	int cnt = 0;

	do {
		nskb = q->dequeue(q);
		if (!nskb)
			break;
		if (unlikely(skb_get_queue_mapping(nskb) != mapping)) {
			q->skb_bad_txq = nskb;
			qdisc_qstats_backlog_inc(q, nskb);
			q->q.qlen++;
			break;
		}
		skb->next = nskb;
		skb = nskb;
	} while (++cnt < 8);
	(*packets) += cnt;
	skb->next = NULL;
}

/* Note that dequeue_skb can possibly return a SKB list (via skb->next).
 * A requeued skb (via q->gso_skb) can also be a SKB list.
 */
static struct sk_buff *dequeue_skb(struct Qdisc *q, bool *validate,
				   int *packets)
{
	struct sk_buff *skb = q->gso_skb;
	const struct netdev_queue *txq = q->dev_queue;

	*packets = 1;
	if (unlikely(skb)) {
		/* skb in gso_skb were already validated */
		*validate = false;
		/* check the reason of requeuing without tx lock first */
		txq = skb_get_tx_queue(txq->dev, skb);
		if (!netif_xmit_frozen_or_stopped(txq)) {
			q->gso_skb = NULL;
			qdisc_qstats_backlog_dec(q, skb);
			q->q.qlen--;
		} else
			skb = NULL;
		goto trace;
	}
	*validate = true;
	skb = q->skb_bad_txq;
	if (unlikely(skb)) {
		/* check the reason of requeuing without tx lock first */
		txq = skb_get_tx_queue(txq->dev, skb);
		if (!netif_xmit_frozen_or_stopped(txq)) {
			q->skb_bad_txq = NULL;
			qdisc_qstats_backlog_dec(q, skb);
			q->q.qlen--;
			goto bulk;
		}
		skb = NULL;
		goto trace;
	}
	if (!(q->flags & TCQ_F_ONETXQUEUE) ||
	    !netif_xmit_frozen_or_stopped(txq))
		skb = q->dequeue(q);
	if (skb) {
bulk:
		if (qdisc_may_bulk(q))
			try_bulk_dequeue_skb(q, skb, txq, packets);
		else
			try_bulk_dequeue_skb_slow(q, skb, packets);
	}
trace:
	trace_qdisc_dequeue(q, txq, *packets, skb);
	return skb;
}

/*
 * Transmit possibly several skbs, and handle the return status as
 * required. Owning running seqcount bit guarantees that
 * only one CPU can execute this function.
 *
 * Returns to the caller:
 *				0  - queue is empty or throttled.
 *				>0 - queue is not empty.
 */
int sch_direct_xmit(struct sk_buff *skb, struct Qdisc *q,
		    struct net_device *dev, struct netdev_queue *txq,
		    spinlock_t *root_lock, bool validate)
{
	int ret = NETDEV_TX_BUSY;

	/* And release qdisc */
	spin_unlock(root_lock);

	/* Note that we validate skb (GSO, checksum, ...) outside of locks */
	if (validate)
		skb = validate_xmit_skb_list(skb, dev);

	if (likely(skb)) {
		HARD_TX_LOCK(dev, txq, smp_processor_id());
		if (!netif_xmit_frozen_or_stopped(txq))
			skb = dev_hard_start_xmit(skb, dev, txq, &ret);

		HARD_TX_UNLOCK(dev, txq);
	} else {
		spin_lock(root_lock);
		return qdisc_qlen(q);
	}
	spin_lock(root_lock);

	if (dev_xmit_complete(ret)) {
		/* Driver sent out skb successfully or skb was consumed */
		ret = qdisc_qlen(q);
	} else {
		/* Driver returned NETDEV_TX_BUSY - requeue skb */
		if (unlikely(ret != NETDEV_TX_BUSY))
			net_warn_ratelimited("BUG %s code %d qlen %d\n",
					     dev->name, ret, q->q.qlen);

		ret = dev_requeue_skb(skb, q);
	}

	if (ret && netif_xmit_frozen_or_stopped(txq))
		ret = 0;

	return ret;
}

/*
 * NOTE: Called under qdisc_lock(q) with locally disabled BH.
 *
 * running seqcount guarantees only one CPU can process
 * this qdisc at a time. qdisc_lock(q) serializes queue accesses for
 * this queue.
 *
 *  netif_tx_lock serializes accesses to device driver.
 *
 *  qdisc_lock(q) and netif_tx_lock are mutually exclusive,
 *  if one is grabbed, another must be free.
 *
 * Note, that this procedure can be called by a watchdog timer
 *
 * Returns to the caller:
 *				0  - queue is empty or throttled.
 *				>0 - queue is not empty.
 *
 */
static inline int qdisc_restart(struct Qdisc *q, int *packets)
{
	struct netdev_queue *txq;
	struct net_device *dev;
	spinlock_t *root_lock;
	struct sk_buff *skb;
	bool validate;

	/* Dequeue packet */
	skb = dequeue_skb(q, &validate, packets);
	if (unlikely(!skb))
		return 0;

	root_lock = qdisc_lock(q);
	dev = qdisc_dev(q);
	txq = skb_get_tx_queue(dev, skb);

	return sch_direct_xmit(skb, q, dev, txq, root_lock, validate);
}

void __qdisc_run(struct Qdisc *q)
{
	int quota = dev_tx_weight;
	int packets;

	while (qdisc_restart(q, &packets)) {
		/*
		 * Ordered by possible occurrence: Postpone processing if
		 * 1. we've exceeded packet quota
		 * 2. another process needs the CPU;
		 */
		quota -= packets;
		if (quota <= 0 || need_resched()) {
			__netif_schedule(q);
			break;
		}
	}

	qdisc_run_end(q);
}

unsigned long dev_trans_start(struct net_device *dev)
{
	unsigned long val, res;
	unsigned int i;

	if (is_vlan_dev(dev))
		dev = vlan_dev_real_dev(dev);
	res = netdev_get_tx_queue(dev, 0)->trans_start;
	for (i = 1; i < dev->num_tx_queues; i++) {
		val = netdev_get_tx_queue(dev, i)->trans_start;
		if (val && time_after(val, res))
			res = val;
	}

	return res;
}
EXPORT_SYMBOL(dev_trans_start);

static void dev_watchdog(unsigned long arg)
{
	struct net_device *dev = (struct net_device *)arg;

	netif_tx_lock(dev);
	if (!qdisc_tx_is_noop(dev)) {
		if (netif_device_present(dev) &&
		    netif_running(dev) &&
		    netif_carrier_ok(dev)) {
			int some_queue_timedout = 0;
			unsigned int i;
			unsigned long trans_start;

			for (i = 0; i < dev->num_tx_queues; i++) {
				struct netdev_queue *txq;

				txq = netdev_get_tx_queue(dev, i);
				trans_start = txq->trans_start;
				if (netif_xmit_stopped(txq) &&
				    time_after(jiffies, (trans_start +
							 dev->watchdog_timeo))) {
					some_queue_timedout = 1;
					txq->trans_timeout++;
					break;
				}
			}

			if (some_queue_timedout) {
				WARN_ONCE(1, KERN_INFO "NETDEV WATCHDOG: %s (%s): transmit queue %u timed out\n",
				       dev->name, netdev_drivername(dev), i);
				dev->netdev_ops->ndo_tx_timeout(dev);
			}
			if (!mod_timer(&dev->watchdog_timer,
				       round_jiffies(jiffies +
						     dev->watchdog_timeo)))
				dev_hold(dev);
		}
	}
	netif_tx_unlock(dev);

	dev_put(dev);
}

void __netdev_watchdog_up(struct net_device *dev)
{
	if (dev->netdev_ops->ndo_tx_timeout) {
		if (dev->watchdog_timeo <= 0)
			dev->watchdog_timeo = 5*HZ;
		if (!mod_timer(&dev->watchdog_timer,
			       round_jiffies(jiffies + dev->watchdog_timeo)))
			dev_hold(dev);
	}
}

static void dev_watchdog_up(struct net_device *dev)
{
	__netdev_watchdog_up(dev);
}

static void dev_watchdog_down(struct net_device *dev)
{
	netif_tx_lock_bh(dev);
	if (del_timer(&dev->watchdog_timer))
		dev_put(dev);
	netif_tx_unlock_bh(dev);
}

/**
 *	netif_carrier_on - set carrier
 *	@dev: network device
 *
 * Device has detected that carrier.
 */
void netif_carrier_on(struct net_device *dev)
{
	if (test_and_clear_bit(__LINK_STATE_NOCARRIER, &dev->state)) {
		if (dev->reg_state == NETREG_UNINITIALIZED)
			return;
		atomic_inc(&dev->carrier_changes);
		linkwatch_fire_event(dev);
		if (netif_running(dev))
			__netdev_watchdog_up(dev);
	}
}
EXPORT_SYMBOL(netif_carrier_on);

/**
 *	netif_carrier_off - clear carrier
 *	@dev: network device
 *
 * Device has detected loss of carrier.
 */
void netif_carrier_off(struct net_device *dev)
{
	if (!test_and_set_bit(__LINK_STATE_NOCARRIER, &dev->state)) {
		if (dev->reg_state == NETREG_UNINITIALIZED)
			return;
		atomic_inc(&dev->carrier_changes);
		linkwatch_fire_event(dev);
	}
}
EXPORT_SYMBOL(netif_carrier_off);

/* "NOOP" scheduler: the best scheduler, recommended for all interfaces
   under all circumstances. It is difficult to invent anything faster or
   cheaper.
 */

static int noop_enqueue(struct sk_buff *skb, struct Qdisc *qdisc,
			struct sk_buff **to_free)
{
	__qdisc_drop(ruct s0M@q,<q,<Sircumsruc*1c<q,<R p)~aM@q,<k_r(