/*
   drbd_receiver.c

   This file is part of DRBD by Philipp Reisner and Lars Ellenberg.

   Copyright (C) 2001-2008, LINBIT Information Technologies GmbH.
   Copyright (C) 1999-2008, Philipp Reisner <philipp.reisner@linbit.com>.
   Copyright (C) 2002-2008, Lars Ellenberg <lars.ellenberg@linbit.com>.

   drbd is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   drbd is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with drbd; see the file COPYING.  If not, write to
   the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include <linux/module.h>

#include <linux/uaccess.h>
#include <net/sock.h>

#include <linux/drbd.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/in.h>
#include <linux/mm.h>
#include <linux/memcontrol.h>
#include <linux/mm_inline.h>
#include <linux/slab.h>
#include <uapi/linux/sched/types.h>
#include <linux/sched/signal.h>
#include <linux/pkt_sched.h>
#define __KERNEL_SYSCALLS__
#include <linux/unistd.h>
#include <linux/vmalloc.h>
#include <linux/random.h>
#include <linux/string.h>
#include <linux/scatterlist.h>
#include "drbd_int.h"
#include "drbd_protocol.h"
#include "drbd_req.h"
#include "drbd_vli.h"

#define PRO_FEATURES (DRBD_FF_TRIM|DRBD_FF_THIN_RESYNC|DRBD_FF_WSAME)

struct packet_info {
	enum drbd_packet cmd;
	unsigned int size;
	unsigned int vnr;
	void *data;
};

enum finish_epoch {
	FE_STILL_LIVE,
	FE_DESTROYED,
	FE_RECYCLED,
};

static int drbd_do_features(struct drbd_connection *connection);
static int drbd_do_auth(struct drbd_connection *connection);
static int drbd_disconnected(struct drbd_peer_device *);
static void conn_wait_active_ee_empty(struct drbd_connection *connection);
static enum finish_epoch drbd_may_finish_epoch(struct drbd_connection *, struct drbd_epoch *, enum epoch_event);
static int e_end_block(struct drbd_work *, int);


#define GFP_TRY	(__GFP_HIGHMEM | __GFP_NOWARN)

/*
 * some helper functions to deal with single linked page lists,
 * page->private being our "next" pointer.
 */

/* If at least n pages are linked at head, get n pages off.
 * Otherwise, don't modify head, and return NULL.
 * Locking is the responsibility of the caller.
 */
static struct page *page_chain_del(struct page **head, int n)
{
	struct page *page;
	struct page *tmp;

	BUG_ON(!n);
	BUG_ON(!head);

	page = *head;

	if (!page)
		return NULL;

	while (page) {
		tmp = page_chain_next(page);
		if (--n == 0)
			break; /* found sufficient pages */
		if (tmp == NULL)
			/* insufficient pages, don't use any of them. */
			return NULL;
		page = tmp;
	}

	/* add end of list marker for the returned list */
	set_page_private(page, 0);
	/* actual return value, and adjustment of head */
	page = *head;
	*head = tmp;
	return page;
}

/* may be used outside of locks to find the tail of a (usually short)
 * "private" page chain, before adding it back to a global chain head
 * with page_chain_add() under a spinlock. */
static struct page *page_chain_tail(struct page *page, int *len)
{
	struct page *tmp;
	int i = 1;
	while ((tmp = page_chain_next(page)))
		++i, page = tmp;
	if (len)
		*len = i;
	return page;
}

static int page_chain_free(struct page *page)
{
	struct page *tmp;
	int i = 0;
	page_chain_for_each_safe(page, tmp) {
		put_page(page);
		++i;
	}
	return i;
}

static void page_chain_add(struct page **head,
		struct page *chain_first, struct page *chain_last)
{
#if 1
	struct page *tmp;
	tmp = page_chain_tail(chain_first, NULL);
	BUG_ON(tmp != chain_last);
#endif

	/* add chain to head */
	set_page_private(chain_last, (unsigned long)*head);
	*head = chain_first;
}

static struct page *__drbd_alloc_pages(struct drbd_device *device,
				       unsigned int number)
{
	struct page *page = NULL;
	struct page *tmp = NULL;
	unsigned int i = 0;

	/* Yes, testing drbd_pp_vacant outside the lock is racy.
	 * So what. It saves a spin_lock. */
	if (drbd_pp_vacant >= number) {
		spin_lock(&drbd_pp_lock);
		page = page_chain_del(&drbd_pp_pool, number);
		if (page)
			drbd_pp_vacant -= number;
		spin_unlock(&drbd_pp_lock);
		if (page)
			return page;
	}

	/* GFP_TRY, because we must not cause arbitrary write-out: in a DRBD
	 * "criss-cross" setup, that might cause write-out on some other DRBD,
	 * which in turn might block on the other node at this very place.  */
	for (i = 0; i < number; i++) {
		tmp = alloc_page(GFP_TRY);
		if (!tmp)
			break;
		set_page_private(tmp, (unsigned long)page);
		page = tmp;
	}

	if (i == number)
		return page;

	/* Not enough pages immediately available this time.
	 * No need to jump around here, drbd_alloc_pages will retry this
	 * function "soon". */
	if (page) {
		tmp = page_chain_tail(page, NULL);
		spin_lock(&drbd_pp_lock);
		page_chain_add(&drbd_pp_pool, page, tmp);
		drbd_pp_vacant += i;
		spin_unlock(&drbd_pp_lock);
	}
	return NULL;
}

static void reclaim_finished_net_peer_reqs(struct drbd_device *device,
					   struct list_head *to_be_freed)
{
	struct drbd_peer_request *peer_req, *tmp;

	/* The EEs are always appended to the end of the list. Since
	   they are sent in order over the wire, they have to finish
	   in order. As soon as we see the first not finished we can
	   stop to examine the list... */

	list_for_each_entry_safe(peer_req, tmp, &device->net_ee, w.list) {
		if (drbd_peer_req_has_active_page(peer_req))
			break;
		list_move(&peer_req->w.list, to_be_freed);
	}
}

static void drbd_reclaim_net_peer_reqs(struct drbd_device *device)
{
	LIST_HEAD(reclaimed);
	struct drbd_peer_request *peer_req, *t;

	spin_lock_irq(&device->resource->req_lock);
	reclaim_finished_net_peer_reqs(device, &reclaimed);
	spin_unlock_irq(&device->resource->req_lock);
	list_for_each_entry_safe(peer_req, t, &reclaimed, w.list)
		drbd_free_net_peer_req(device, peer_req);
}

static void conn_reclaim_net_peer_reqs(struct drbd_connection *connection)
{
	struct drbd_peer_device *peer_device;
	int vnr;

	rcu_read_lock();
	idr_for_each_entry(&connection->peer_devices, peer_device, vnr) {
		struct drbd_device *device = peer_device->device;
		if (!atomic_read(&device->pp_in_use_by_net))
			continue;

		kref_get(&device->kref);
		rcu_read_unlock();
		drbd_reclaim_net_peer_reqs(device);
		kref_put(&device->kref, drbd_destroy_device);
		rcu_read_lock();
	}
	rcu_read_unlock();
}

/**
 * drbd_alloc_pages() - Returns @number pages, retries forever (or until signalled)
 * @device:	DRBD device.
 * @number:	number of pages requested
 * @retry:	whether to retry, if not enough pages are available right now
 *
 * Tries to allocate number pages, first from our own page pool, then from
 * the kernel.
 * Possibly retry until DRBD frees sufficient pages somewhere else.
 *
 * If this allocation would exceed the max_buffers setting, we throttle
 * allocation (schedule_timeout) to give the system some room to breathe.
 *
 * We do not use max-buffers as hard limit, because it could lead to
 * congestion and further to a distributed deadlock during online-verify or
 * (checksum based) resync, if the max-buffers, socket buffer sizes and
 * resync-rate settings are mis-configured.
 *
 * Returns a page chain linked via page->private.
 */
struct page *drbd_alloc_pages(struct drbd_peer_device *peer_device, unsigned int number,
			      bool retry)
{
	struct drbd_device *device = peer_device->device;
	struct page *page = NULL;
	struct net_conf *nc;
	DEFINE_WAIT(wait);
	unsigned int mxb;

	rcu_read_lock();
	nc = rcu_dereference(peer_device->connection->net_conf);
	mxb = nc ? nc->max_buffers : 1000000;
	rcu_read_unlock();

	if (atomic_read(&device->pp_in_use) < mxb)
		page = __drbd_alloc_pages(device, number);

	/* Try to keep the fast path fast, but occasionally we need
	 * to reclaim the pages we lended to the network stack. */
	if (page && atomic_read(&device->pp_in_use_by_net) > 512)
		drbd_reclaim_net_peer_reqs(device);

	while (page == NULL) {
		prepare_to_wait(&drbd_pp_wait, &wait, TASK_INTERRUPTIBLE);

		drbd_reclaim_net_peer_reqs(device);

		if (atomic_read(&device->pp_in_use) < mxb) {
			page = __drbd_alloc_pages(device, number);
			if (page)
				break;
		}

		if (!retry)
			break;

		if (signal_pending(current)) {
			drbd_warn(device, "drbd_alloc_pages interrupted!\n");
			break;
		}

		if (schedule_timeout(HZ/10) == 0)
			mxb = UINT_MAX;
	}
	finish_wait(&drbd_pp_wait, &wait);

	if (page)
		atomic_add(number, &device->pp_in_use);
	return page;
}

/* Must not be used from irq, as that may deadlock: see drbd_alloc_pages.
 * Is also used from inside an other spin_lock_irq(&resource->req_lock);
 * Either links the page chain back to the global pool,
 * or returns all pages to the system. */
static void drbd_free_pages(struct drbd_device *device, struct page *page, int is_net)
{
	atomic_t *a = is_net ? &device->pp_in_use_by_net : &device->pp_in_use;
	int i;

	if (page == NULL)
		return;

	if (drbd_pp_vacant > (DRBD_MAX_BIO_SIZE/PAGE_SIZE) * drbd_minor_count)
		i = page_chain_free(page);
	else {
		struct page *tmp;
		tmp = page_chain_tail(page, &i);
		spin_lock(&drbd_pp_lock);
		page_chain_add(&drbd_pp_pool, page, tmp);
		drbd_pp_vacant += i;
		spin_unlock(&drbd_pp_lock);
	}
	i = atomic_sub_return(i, a);
	if (i < 0)
		drbd_warn(device, "ASSERTION FAILED: %s: %d < 0\n",
			is_net ? "pp_in_use_by_net" : "pp_in_use", i);
	wake_up(&drbd_pp_wait);
}

/*
You need to hold the req_lock:
 _drbd_wait_ee_list_empty()

You must not have the req_lock:
 drbd_free_peer_req()
 drbd_alloc_peer_req()
 drbd_free_peer_reqs()
 drbd_ee_fix_bhs()
 drbd_finish_peer_reqs()
 drbd_clear_done_ee()
 drbd_wait_ee_list_empty()
*/

/* normal: payload_size == request size (bi_size)
 * w_same: payload_size == logical_block_size
 * trim: payload_size == 0 */
struct drbd_peer_request *
drbd_alloc_peer_req(struct drbd_peer_device *peer_device, u64 id, sector_t sector,
		    unsigned int request_size, unsigned int payload_size, gfp_t gfp_mask) __must_hold(local)
{
	struct drbd_device *device = peer_device->device;
	struct drbd_peer_request *peer_req;
	struct page *page = NULL;
	unsigned nr_pages = (payload_size + PAGE_SIZE -1) >> PAGE_SHIFT;

	if (drbd_insert_fault(device, DRBD_FAULT_AL_EE))
		return NULL;

	peer_req = mempool_alloc(drbd_ee_mempool, gfp_mask & ~__GFP_HIGHMEM);
	if (!peer_req) {
		if (!(gfp_mask & __GFP_NOWARN))
			drbd_err(device, "%s: allocation failed\n", __func__);
		return NULL;
	}

	if (nr_pages) {
		page = drbd_alloc_pages(peer_device, nr_pages,
					gfpflags_allow_blocking(gfp_mask));
		if (!page)
			goto fail;
	}

	memset(peer_req, 0, sizeof(*peer_req));
	INIT_LIST_HEAD(&peer_req->w.list);
	drbd_clear_interval(&peer_req->i);
	peer_req->i.size = request_size;
	peer_req->i.sector = sector;
	peer_req->submit_jif = jiffies;
	peer_req->peer_device = peer_device;
	peer_req->pages = page;
	/*
	 * The block_id is opaque to the receiver.  It is not endianness
	 * converted, and sent back to the sender unchanged.
	 */
	peer_req->block_id = id;

	return peer_req;

 fail:
	mempool_free(peer_req, drbd_ee_mempool);
	return NULL;
}

void __drbd_free_peer_req(struct drbd_device *device, struct drbd_peer_request *peer_req,
		       int is_net)
{
	might_sleep();
	if (peer_req->flags & EE_HAS_DIGEST)
		kfree(peer_req->digest);
	drbd_free_pages(device, peer_req->pages, is_net);
	D_ASSERT(device, atomic_read(&peer_req->pending_bios) == 0);
	D_ASSERT(device, drbd_interval_empty(&peer_req->i));
	if (!expect(!(peer_req->flags & EE_CALL_AL_COMPLETE_IO))) {
		peer_req->flags &= ~EE_CALL_AL_COMPLETE_IO;
		drbd_al_complete_io(device, &peer_req->i);
	}
	mempool_free(peer_req, drbd_ee_mempool);
}

int drbd_free_peer_reqs(struct drbd_device *device, struct list_head *list)
{
	LIST_HEAD(work_list);
	struct drbd_peer_request *peer_req, *t;
	int count = 0;
	int is_net = list == &device->net_ee;

	spin_lock_irq(&device->resource->req_lock);
	list_splice_init(list, &work_list);
	spin_unlock_irq(&device->resource->req_lock);

	list_for_each_entry_safe(peer_req, t, &work_list, w.list) {
		__drbd_free_peer_req(device, peer_req, is_net);
		count++;
	}
	return count;
}

/*
 * See also comments in _req_mod(,BARRIER_ACKED) and receive_Barrier.
 */
static int drbd_finish_peer_reqs(struct drbd_device *device)
{
	LIST_HEAD(work_list);
	LIST_HEAD(reclaimed);
	struct drbd_peer_request *peer_req, *t;
	int err = 0;

	spin_lock_irq(&device->resource->req_lock);
	reclaim_finished_net_peer_reqs(device, &reclaimed);
	list_splice_init(&device->done_ee, &work_list);
	spin_unlock_irq(&device->resource->req_lock);

	list_for_each_entry_safe(peer_req, t, &reclaimed, w.list)
		drbd_free_net_peer_req(device, peer_req);

	/* possible callbacks here:
	 * e_end_block, and e_end_resync_block, e_send_superseded.
	 * all ignore the last argument.
	 */
	list_for_each_entry_safe(peer_req, t, &work_list, w.list) {
		int err2;

		/* list_del not necessary, next/prev members not touched */
		err2 = peer_req->w.cb(&peer_req->w, !!err);
		if (!err)
			err = err2;
		drbd_free_peer_req(device, peer_req);
	}
	wake_up(&device->ee_wait);

	return err;
}

static void _drbd_wait_ee_list_empty(struct drbd_device *device,
				     struct list_head *head)
{
	DEFINE_WAIT(wait);

	/* avoids spin_lock/unlock
	 * and calling prepare_to_wait in the fast path */
	while (!list_empty(head)) {
		prepare_to_wait(&device->ee_wait, &wait, TASK_UNINTERRUPTIBLE);
		spin_unlock_irq(&device->resource->req_lock);
		io_schedule();
		finish_wait(&device->ee_wait, &wait);
		spin_lock_irq(&device->resource->req_lock);
	}
}

static void drbd_wait_ee_list_empty(struct drbd_device *device,
				    struct list_head *head)
{
	spin_lock_irq(&device->resource->req_lock);
	_drbd_wait_ee_list_empty(device, head);
	spin_unlock_irq(&device->resource->req_lock);
}

static int drbd_recv_short(struct socket *sock, void *buf, size_t size, int flags)
{
	struct kvec iov = {
		.iov_base = buf,
		.iov_len = size,
	};
	struct msghdr msg = {
		.msg_flags = (flags ? flags : MSG_WAITALL | MSG_NOSIGNAL)
	};
	return kernel_recvmsg(sock, &msg, &iov, 1, size, msg.msg_flags);
}

static int drbd_recv(struct drbd_connection *connection, void *buf, size_t size)
{
	int rv;

	rv = drbd_recv_short(connection->data.socket, buf, size, 0);

	if (rv < 0) {
		if (rv == -ECONNRESET)
			drbd_info(connection, "sock was reset by peer\n");
		else if (rv != -ERESTARTSYS)
			drbd_err(connection, "sock_recvmsg returned %d\n", rv);
	} else if (rv == 0) {
		if (test_bit(DISCONNECT_SENT, &connection->flags)) {
			long t;
			rcu_read_lock();
			t = rcu_dereference(connection->net_conf)->ping_timeo * HZ/10;
			rcu_read_unlock();

			t = wait_event_timeout(connection->ping_wait, connection->cstate < C_WF_REPORT_PARAMS, t);

			if (t)
				goto out;
		}
		drbd_info(connection, "sock was shut down by peer\n");
	}

	if (rv != size)
		conn_request_state(connection, NS(conn, C_BROKEN_PIPE), CS_HARD);

out:
	return rv;
}

static int drbd_recv_all(struct drbd_connection *connection, void *buf, size_t size)
{
	int err;

	err = drbd_recv(connection, buf, size);
	if (err != size) {
		if (err >= 0)
			err = -EIO;
	} else
		err = 0;
	return err;
}

static int drbd_recv_all_warn(struct drbd_connection *connection, void *buf, size_t size)
{
	int err;

	err = drbd_recv_all(connection, buf, size);
	if (err && !signal_pending(current))
		drbd_warn(connection, "short read (expected size %d)\n", (int)size);
	return err;
}

/* quoting tcp(7):
 *   On individual connections, the socket buffer size must be set prior to the
 *   listen(2) or connect(2) calls in order to have it take effect.
 * This is our wrapper to do so.
 */
static void drbd_setbufsize(struct socket *sock, unsigned int snd,
		unsigned int rcv)
{
	/* open coded SO_SNDBUF, SO_RCVBUF */
	if (snd) {
		sock->sk->sk_sndbuf = snd;
		sock->sk->sk_userlocks |= SOCK_SNDBUF_LOCK;
	}
	if (rcv) {
		sock->sk->sk_rcvbuf = rcv;
		sock->sk->sk_userlocks |= SOCK_RCVBUF_LOCK;
	}
}

static struct socket *drbd_try_connect(struct drbd_connection *connection)
{
	const char *what;
	struct socket *sock;
	struct sockaddr_in6 src_in6;
	struct sockaddr_in6 peer_in6;
	struct net_conf *nc;
	int err, peer_addr_len, my_addr_len;
	int sndbuf_size, rcvbuf_size, connect_int;
	int disconnect_on_error = 1;

	rcu_read_lock();
	nc = rcu_dereference(connection->net_conf);
	if (!nc) {
		rcu_read_unlock();
		return NULL;
	}
	sndbuf_size = nc->sndbuf_size;
	rcvbuf_size = nc->rcvbuf_size;
	connect_int = nc->connect_int;
	rcu_read_unlock();

	my_addr_len = min_t(int, connection->my_addr_len, sizeof(src_in6));
	memcpy(&src_in6, &connection->my_addr, my_addr_len);

	if (((struct sockaddr *)&connection->my_addr)->sa_family == AF_INET6)
		src_in6.sin6_port = 0;
	else
		((struct sockaddr_in *)&src_in6)->sin_port = 0; /* AF_INET & AF_SCI */

	peer_addr_len = min_t(int, connection->peer_addr_len, sizeof(src_in6));
	memcpy(&peer_in6, &connection->peer_addr, peer_addr_len);

	what = "sock_create_kern";
	err = sock_create_kern(&init_net, ((struct sockaddr *)&src_in6)->sa_family,
			       SOCK_STREAM, IPPROTO_TCP, &sock);
	if (err < 0) {
		sock = NULL;
		goto out;
	}

	sock->sk->sk_rcvtimeo =
	sock->sk->sk_sndtimeo = connect_int * HZ;
	drbd_setbufsize(sock, sndbuf_size, rcvbuf_size);

       /* explicitly bind to the configured IP as source IP
	*  for the outgoing connections.
	*  This is needed for multihomed hosts and to be
	*  able to use lo: interfaces for drbd.
	* Make sure to use 0 as port number, so linux selects
	*  a free one dynamically.
	*/
	what = "bind before connect";
	err = sock->ops->bind(sock, (struct sockaddr *) &src_in6, my_addr_len);
	if (err < 0)
		goto out;

	/* connect may fail, peer not yet available.
	 * stay C_WF_CONNECTION, don't go Disconnecting! */
	disconnect_on_error = 0;
	what = "connect";
	err = sock->ops->connect(sock, (struct sockaddr *) &peer_in6, peer_addr_len, 0);

out:
	if (err < 0) {
		if (sock) {
			sock_release(sock);
			sock = NULL;
		}
		switch (-err) {
			/* timeout, busy, signal pending */
		case ETIMEDOUT: case EAGAIN: case EINPROGRESS:
		case EINTR: case ERESTARTSYS:
			/* peer not (yet) available, network problem */
		case ECONNREFUSED: case ENETUNREACH:
		case EHOSTDOWN:    case EHOSTUNREACH:
			disconnect_on_error = 0;
			break;
		default:
			drbd_err(connection, "%s failed, err = %d\n", what, err);
		}
		if (disconnect_on_error)
			conn_request_state(connection, NS(conn, C_DISCONNECTING), CS_HARD);
	}

	return sock;
}

struct accept_wait_data {
	struct drbd_connection *connection;
	struct socket *s_listen;
	struct completion door_bell;
	void (*original_sk_state_change)(struct sock *sk);

};

static void drbd_incoming_connection(struct sock *sk)
{
	struct accept_wait_data *ad = sk->sk_user_data;
	void (*state_change)(struct sock *sk);

	state_change = ad->original_sk_state_change;
	if (sk->sk_state == TCP_ESTABLISHED)
		complete(&ad->door_bell);
	state_change(sk);
}

static int prepare_listen_socket(struct drbd_connection *connection, struct accept_wait_data *ad)
{
	int err, sndbuf_size, rcvbuf_size, my_addr_len;
	struct sockaddr_in6 my_addr;
	struct socket *s_listen;
	struct net_conf *nc;
	const char *what;

	rcu_read_lock();
	nc = rcu_dereference(connection->net_conf);
	if (!nc) {
		rcu_read_unlock();
		return -EIO;
	}
	sndbuf_size = nc->sndbuf_size;
	rcvbuf_size = nc->rcvbuf_size;
	rcu_read_unlock();

	my_addr_len = min_t(int, connection->my_addr_len, sizeof(struct sockaddr_in6));
	memcpy(&my_addr, &connection->my_addr, my_addr_len);

	what = "sock_create_kern";
	err = sock_create_kern(&init_net, ((struct sockaddr *)&my_addr)->sa_family,
			       SOCK_STREAM, IPPROTO_TCP, &s_listen);
	if (err) {
		s_listen = NULL;
		goto out;
	}

	s_listen->sk->sk_reuse = SK_CAN_REUSE; /* SO_REUSEADDR */
	drbd_setbufsize(s_listen, sndbuf_size, rcvbuf_size);

	what = "bind before listen";
	err = s_listen->ops->bind(s_listen, (struct sockaddr *)&my_addr, my_addr_len);
	if (err < 0)
		goto out;

	ad->s_listen = s_listen;
	write_lock_bh(&s_listen->sk->sk_callback_lock);
	ad->original_sk_state_change = s_listen->sk->sk_state_change;
	s_listen->sk->sk_state_change = drbd_incoming_connection;
	s_listen->sk->sk_user_data = ad;
	write_unlock_bh(&s_listen->sk->sk_callback_lock);

	what = "listen";
	err = s_listen->ops->listen(s_listen, 5);
	if (err < 0)
		goto out;

	return 0;
out:
	if (s_listen)
		sock_release(s_listen);
	if (err < 0) {
		if (err != -EAGAIN && err != -EINTR && err != -ERESTARTSYS) {
			drbd_err(connection, "%s failed, err = %d\n", what, err);
			conn_request_state(connection, NS(conn, C_DISCONNECTING), CS_HARD);
		}
	}

	return -EIO;
}

static void unregister_state_change(struct sock *sk, struct accept_wait_data *ad)
{
	write_lock_bh(&sk->sk_callback_lock);
	sk->sk_state_change = ad->original_sk_state_change;
	sk->sk_user_data = NULL;
	write_unlock_bh(&sk->sk_callback_lock);
}

static struct socket *drbd_wait_for_connect(struct drbd_connection *connection, struct accept_wait_data *ad)
{
	int timeo, connect_int, err = 0;
	struct socket *s_estab = NULL;
	struct net_conf *nc;

	rcu_read_lock();
	nc = rcu_dereference(connection->net_conf);
	if (!nc) {
		rcu_read_unlock();
		return NULL;
	}
	connect_int = nc->connect_int;
	rcu_read_unlock();

	timeo = connect_int * HZ;
	/* 28.5% random jitter */
	timeo += (prandom_u32() & 1) ? timeo / 7 : -timeo / 7;

	err = wait_for_completion_interruptible_timeout(&ad->door_bell, timeo);
	if (err <= 0)
		return NULL;

	err = kernel_accept(ad->s_listen, &s_estab, 0);
	if (err < 0) {
		if (err != -EAGAIN && err != -EINTR && err != -ERESTARTSYS) {
			drbd_err(connection, "accept failed, err = %d\n", err);
			conn_request_state(connection, NS(conn, C_DISCONNECTING), CS_HARD);
		}
	}

	if (s_estab)
		unregister_state_change(s_estab->sk, ad);

	return s_estab;
}

static int decode_header(struct drbd_connection *, void *, struct packet_info *);

static int send_first_packet(struct drbd_connection *connection, struct drbd_socket *sock,
			     enum drbd_packet cmd)
{
	if (!conn_prepare_command(connection, sock))
		return -EIO;
	return conn_send_command(connection, sock, cmd, 0, NULL, 0);
}

static int receive_first_packet(struct drbd_connection *connection, struct socket *sock)
{
	unsigned int header_size = drbd_header_size(connection);
	struct packet_info pi;
	struct net_conf *nc;
	int err;

	rcu_read_lock();
	nc = rcu_dereference(connection->net_conf);
	if (!nc) {
		rcu_read_unlock();
		return -EIO;
	}
	sock->sk->sk_rcvtimeo = nc->ping_timeo * 4 * HZ / 10;
	rcu_read_unlock();

	err = drbd_recv_short(sock, connection->data.rbuf, header_size, 0);
	if (err != header_size) {
		if (err >= 0)
			err = -EIO;
		return err;
	}
	err = decode_header(connection, connection->data.rbuf, &pi);
	if (err)
		return err;
	return pi.cmd;
}

/**
 * drbd_socket_okay() - Free the socket if its connection is not okay
 * @sock:	pointer to the pointer to the socket.
 */
static bool drbd_socket_okay(struct socket **sock)
{
	int rr;
	char tb[4];

	if (!*sock)
		return false;

	rr = drbd_recv_short(*sock, tb, 4, MSG_DONTWAIT | MSG_PEEK);

	if (rr > 0 || rr == -EAGAIN) {
		return true;
	} else {
		sock_release(*sock);
		*sock = NULL;
		return false;
	}
}

static bool connection_established(struct drbd_connection *connection,
				   struct socket **sock1,
				   struct socket **sock2)
{
	struct net_conf *nc;
	int timeout;
	bool ok;

	if (!*sock1 || !*sock2)
		return false;

	rcu_read_lock();
	nc = rcu_dereference(connection->net_conf);
	timeout = (nc->sock_check_timeo ?: nc->ping_timeo) * HZ / 10;
	rcu_read_unlock();
	schedule_timeout_interruptible(timeout);

	ok = drbd_socket_okay(sock1);
	ok = drbd_socket_okay(sock2) && ok;

	return ok;
}

/* Gets called if a connection is established, or if a new minor gets created
   in a connection */
int drbd_connected(struct drbd_peer_device *peer_device)
{
	struct drbd_device *device = peer_device->device;
	int err;

	atomic_set(&device->packet_seq, 0);
	device->peer_seq = 0;

	device->state_mutex = peer_device->connection->agreed_pro_version < 100 ?
		&peer_device->connection->cstate_mutex :
		&device->own_state_mutex;

	err = drbd_send_sync_param(peer_device);
	if (!err)
		err = drbd_send_sizes(peer_device, 0, 0);
	if (!err)
		err = drbd_send_uuids(peer_device);
	if (!err)
		err = drbd_send_current_state(peer_device);
	clear_bit(USE_DEGR_WFC_T, &device->flags);
	clear_bit(RESIZE_PENDING, &device->flags);
	atomic_set(&device->ap_in_flight, 0);
	mod_timer(&device->request_timer, jiffies + HZ); /* just start it here. */
	return err;
}

/*
 * return values:
 *   1 yes, we have a valid connection
 *   0 oops, did not work out, please try again
 *  -1 peer talks different language,
 *     no point in trying again, please go standalone.
 *  -2 We do not have a network config...
 */
static int conn_connect(struct drbd_connection *connection)
{
	struct drbd_socket sock, msock;
	struct drbd_peer_device *peer_device;
	struct net_conf *nc;
	int vnr, timeout, h;
	bool discard_my_data, ok;
	enum drbd_state_rv rv;
	struct accept_wait_data ad = {
		.connection = connection,
		.door_bell = COMPLETION_INITIALIZER_ONSTACK(ad.door_bell),
	};

	clear_bit(DISCONNECT_SENT, &connection->flags);
	if (conn_request_state(connection, NS(conn, C_WF_CONNECTION), CS_VERBOSE) < SS_SUCCESS)
		return -2;

	mutex_init(&sock.mutex);
	sock.sbuf = connection->data.sbuf;
	sock.rbuf = connection->data.rbuf;
	sock.socket = NULL;
	mutex_init(&msock.mutex);
	msock.sbuf = connection->meta.sbuf;
	msock.rbuf = connection->meta.rbuf;
	msock.socket = NULL;

	/* Assume that the peer only understands protocol 80 until we know better.  */
	connection->agreed_pro_version = 80;

	if (prepare_listen_socket(connection, &ad))
		return 0;

	do {
		struct socket *s;

		s = drbd_try_connect(connection);
		if (s) {
			if (!sock.socket) {
				sock.socket = s;
				send_first_packet(connection, &sock, P_INITIAL_DATA);
			} else if (!msock.socket) {
				clear_bit(RESOLVE_CONFLICTS, &connection->flags);
				msock.socket = s;
				send_first_packet(connection, &msock, P_INITIAL_META);
			} else {
				drbd_err(connection, "Logic error in conn_connect()\n");
				goto out_release_sockets;
			}
		}

		if (connection_established(connection, &sock.socket, &msock.socket))
			break;

retry:
		s = drbd_wait_for_connect(connection, &ad);
		if (s) {
			int fp = receive_first_packet(connection, s);
			drbd_socket_okay(&sock.socket);
			drbd_socket_okay(&msock.socket);
			switch (fp) {
			case P_INITIAL_DATA:
				if (sock.socket) {
					drbd_warn(connection, "initial packet S crossed\n");
					sock_release(sock.socket);
					sock.socket = s;
					goto randomize;
				}
				sock.socket = s;
				break;
			case P_INITIAL_META:
				set_bit(RESOLVE_CONFLICTS, &connection->flags);
				if (msock.socket) {
					drbd_warn(connection, "initial packet M crossed\n");
					sock_release(msock.socket);
					msock.socket = s;
					goto randomize;
				}
				msock.socket = s;
				break;
			default:
				drbd_warn(connection, "Error receiving initial packet\n");
				sock_release(s);
randomize:
				if (prandom_u32() & 1)
					goto retry;
			}
		}

		if (connection->cstate <= C_DISCONNECTING)
			goto out_release_sockets;
		if (signal_pending(current)) {
			flush_signals(current);
			smp_rmb();
			if (get_t_state(&connection->receiver) == EXITING)
				goto out_release_sockets;
		}

		ok = connection_established(connection, &sock.socket, &msock.socket);
	} while (!ok);

	if (ad.s_listen)
		sock_release(ad.s_listen);

	sock.socket->sk->sk_reuse = SK_CAN_REUSE; /* SO_REUSEADDR */
	msock.socket->sk->sk_reuse = SK_CAN_REUSE; /* SO_REUSEADDR */

	sock.socket->sk->sk_allocation = GFP_NOIO;
	msock.socket->sk->sk_allocation = GFP_NOIO;

	sock.socket->sk->sk_priority = TC_PRIO_INTERACTIVE_BULK;
	msock.socket->sk->sk_priority = TC_PRIO_INTERACTIVE;

	/* NOT YET ...
	 * sock.socket->sk->sk_sndtimeo = connection->net_conf->timeout*HZ/10;
	 * sock.socket->sk->sk_rcvtimeo = MAX_SCHEDULE_TIMEOUT;
	 * first set it to the P_CONNECTION_FEATURES timeout,
	 * which we set to 4x the configured ping_timeout. */
	rcu_read_lock();
	nc = rcu_dereference(connection->net_conf);

	sock.socket->sk->sk_sndtimeo =
	sock.socket->sk->sk_rcvtimeo = nc->ping_timeo*4*HZ/10;

	msock.socket->sk->sk_rcvtimeo = nc->ping_int*HZ;
	timeout = nc->timeout * HZ / 10;
	discard_my_data = nc->discard_my_data;
	rcu_read_unlock();

	msock.socket->sk->sk_sndtimeo = timeout;

	/* we don't want delays.
	 * we use TCP_CORK where appropriate, though */
	drbd_tcp_nodelay(sock.socket);
	drbd_tcp_nodelay(msock.socket);

	connection->data.socket = sock.socket;
	connection->meta.socket = msock.socket;
	connection->last_received = jiffies;

	h = drbd_do_features(connection);
	if (h <= 0)
		return h;

	if (connection->cram_hmac_tfm) {
		/* drbd_request_state(device, NS(conn, WFAuth)); */
		switch (drbd_do_auth(connection)) {
		case -1:
			drbd_err(connection, "Authentication of peer failed\n");
			return -1;
		case 0:
			drbd_err(connection, "Authentication of peer failed, trying again.\n");
			return 0;
		}
	}

	connection->data.socket->sk->sk_sndtimeo = timeout;
	connection->data.socket->sk->sk_rcvtimeo = MAX_SCHEDULE_TIMEOUT;

	if (drbd_send_protocol(connection) == -EOPNOTSUPP)
		return -1;

	/* Prevent a race between resync-handshake and
	 * being promoted to Primary.
	 *
	 * Grab and release the state mutex, so we know that any current
	 * drbd_set_role() is finished, and any incoming drbd_set_role
	 * will see the STATE_SENT flag, and wait for it to be cleared.
	 */
	idr_for_each_entry(&connection->peer_devices, peer_device, vnr)
		mutex_lock(peer_device->device->state_mutex);

	/* avoid a race with conn_request_state( C_DISCONNECTING ) */
	spin_lock_irq(&connection->resource->req_lock);
	set_bit(STATE_SENT, &connection->flags);
	spin_unlock_irq(&connection->resource->req_lock);

	idr_for_each_entry(&connection->peer_devices, peer_device, vnr)
		mutex_unlock(peer_device->device->state_mutex);

	rcu_read_lock();
	idr_for_each_entry(&connection->peer_devices, peer_device, vnr) {
		struct drbd_device *device = peer_device->device;
		kref_get(&device->kref);
		rcu_read_unlock();

		if (discard_my_data)
			set_bit(DISCARD_MY_DATA, &device->flags);
		else
			clear_bit(DISCARD_MY_DATA, &device->flags);

		drbd_connected(peer_device);
		kref_put(&device->kref, drbd_destroy_device);
		rcu_read_lock();
	}
	rcu_read_unlock();

	rv = conn_request_state(connection, NS(conn, C_WF_REPORT_PARAMS), CS_VERBOSE);
	if (rv < SS_SUCCESS || connection->cstate != C_WF_REPORT_PARAMS) {
		clear_bit(STATE_SENT, &connection->flags);
		return 0;
	}

	drbd_thread_start(&connection->ack_receiver);
	/* opencoded create_singlethread_workqueue(),
	 * to be able to use format string arguments */
	connection->ack_sender =
		alloc_ordered_workqueue("drbd_as_%s", WQ_MEM_RECLAIM, connection->resource->name);
	if (!connection->ack_sender) {
		drbd_err(connection, "Failed to create workqueue ack_sender\n");
		return 0;
	}

	mutex_lock(&connection->resource->conf_update);
	/* The discard_my_data flag is a single-shot modifier to the next
	 * connection attempt, the handshake of which is now well underway.
	 * No need for rcu style copying of the whole struct
	 * just to clear a single value. */
	connection->net_conf->discard_my_data = 0;
	mutex_unlock(&connection->resource->conf_update);

	return h;

out_release_sockets:
	if (ad.s_listen)
		sock_release(ad.s_listen);
	if (sock.socket)
		sock_release(sock.socket);
	if (msock.socket)
		sock_release(msock.socket);
	return -1;
}

static int decode_header(struct drbd_connection *connection, void *header, struct packet_info *pi)
{
	unsigned int header_size = drbd_header_size(connection);

	if (header_size == sizeof(struct p_header100) &&
	    *(__be32 *)header == cpu_to_be32(DRBD_MAGIC_100)) {
		struct p_header100 *h = header;
		if (h->pad != 0) {
			drbd_err(connection, "Header padding is not zero\n");
			return -EINVAL;
		}
		pi->vnr = be16_to_cpu(h->volume);
		pi->cmd = be16_to_cpu(h->command);
		pi->size = be32_to_cpu(h->length);
	} else if (header_size == sizeof(struct p_header95) &&
		   *(__be16 *)header == cpu_to_be16(DRBD_MAGIC_BIG)) {
		struct p_header95 *h = header;
		pi->cmd = be16_to_cpu(h->command);
		pi->size = be32_to_cpu(h->length);
		pi->vnr = 0;
	} else if (header_size == sizeof(struct p_header80) &&
		   *(__be32 *)header == cpu_to_be32(DRBD_MAGIC)) {
		struct p_header80 *h = header;
		pi->cmd = be16_to_cpu(h->command);
		pi->size = be16_to_cpu(h->length);
		pi->vnr = 0;
	} else {
		drbd_err(connection, "Wrong magic value 0x%08x in protocol version %d\n",
			 be32_to_cpu(*(__be32 *)header),
			 connection->agreed_pro_version);
		return -EINVAL;
	}
	pi->data = header + header_size;
	return 0;
}

static void drbd_unplug_all_devices(struct drbd_connection *connection)
{
	if (current->plug == &connection->receiver_plug) {
		blk_finish_plug(&connection->receiver_plug);
		blk_start_plug(&connection->receiver_plug);
	} /* else: maybe just schedule() ?? */
}

static int drbd_recv_header(struct drbd_connection *connection, struct packet_info *pi)
{
	void *buffer = connection->data.rbuf;
	int err;

	err = drbd_recv_all_warn(connection, buffer, drbd_header_size(connection));
	if (err)
		return err;

	err = decode_header(connection, buffer, pi);
	connection->last_received = jiffies;

	return err;
}

static int drbd_recv_header_maybe_unplug(struct drbd_connection *connection, struct packet_info *pi)
{
	void *buffer = connection->data.rbuf;
	unsigned int size = drbd_header_size(connection);
	int err;

	err = drbd_recv_short(connection->data.socket, buffer, size, MSG_NOSIGNAL|MSG_DONTWAIT);
	if (err != size) {
		/* If we have nothing in the receive buffer now, to reduce
		 * application latency, try to drain the backend queues as
		 * quickly as possible, and let remote TCP know what we have
		 * received so far. */
		if (err == -EAGAIN) {
			drbd_tcp_quickack(connection->data.socket);
			drbd_unplug_all_devices(connection);
		}
		if (err > 0) {
			buffer += err;
			size -= err;
		}
		err = drbd_recv_all_warn(connection, buffer, size);
		if (err)
			return err;
	}

	err = decode_header(connection, connection->data.rbuf, pi);
	connection->last_received = jiffies;

	return err;
}
/* This is blkdev_issue_flush, but asynchronous.
 * We want to submit to all component volumes in parallel,
 * then wait for all completions.
 */
struct issue_flush_context {
	atomic_t pending;
	int error;
	struct completion done;
};
struct one_flush_context {
	struct drbd_device *device;
	struct issue_flush_context *ctx;
};

static void one_flush_endio(struct bio *bio)
{
	struct one_flush_context *octx = bio->bi_private;
	struct drbd_device *device = octx->device;
	struct issue_flush_context *ctx = octx->ctx;

	if (bio->bi_status) {
		ctx->error = blk_status_to_errno(bio->bi_status);
		drbd_info(device, "local disk FLUSH FAILED with status %d\n", bio->bi_status);
	}
	kfree(octx);
	bio_put(bio);

	clear_bit(FLUSH_PENDING, &device->flags);
	put_ldev(device);
	kref_put(&device->kref, drbd_destroy_device);

	if (atomic_dec_and_test(&ctx->pending))
		complete(&ctx->done);
}

static void submit_one_flush(struct drbd_device *device, struct issue_flush_context *ctx)
{
	struct bio *bio = bio_alloc(GFP_NOIO, 0);
	struct one_flush_context *octx = kmalloc(sizeof(*octx), GFP_NOIO);
	if (!bio || !octx) {
		drbd_warn(device, "Could not allocate a bio, CANNOT ISSUE FLUSH\n");
		/* FIXME: what else can I do now?  disconnecting or detaching
		 * really does not help to improve the state of the world, either.
		 */
		kfree(octx);
		if (bio)
			bio_put(bio);

		ctx->error = -ENOMEM;
		put_ldev(device);
		kref_put(&device->kref, drbd_destroy_device);
		return;
	}

	octx->device = device;
	octx->ctx = ctx;
	bio_set_dev(bio, device->ldev->backing_bdev);
	bio->bi_private = octx;
	bio->bi_end_io = one_flush_endio;
	bio->bi_opf = REQ_OP_FLUSH | REQ_PREFLUSH;

	device->flush_jif = jiffies;
	set_bit(FLUSH_PENDING, &device->flags);
	atomic_inc(&ctx->pending);
	submit_bio(bio);
}

static void drbd_flush(struct drbd_connection *connection)
{
	if (connection->resource->write_ordering >= WO_BDEV_FLUSH) {
		struct drbd_peer_device *peer_device;
		struct issue_flush_context ctx;
		int vnr;

		atomic_set(&ctx.pending, 1);
		ctx.error = 0;
		init_completion(&ctx.done);

		rcu_read_lock();
		idr_for_each_entry(&connection->peer_devices, peer_device, vnr) {
			struct drbd_device *device = peer_device->device;

			if (!get_ldev(device))
				continue;
			kref_get(&device->kref);
			rcu_read_unlock();

			submit_one_flush(device, &ctx);

			rcu_read_lock();
		}
		rcu_read_unlock();

		/* Do we want to add a timeout,
		 * if disk-timeout is set? */
		if (!atomic_dec_and_test(&ctx.pending))
			wait_for_completion(&ctx.done);

		if (ctx.error) {
			/* would rather check on EOPNOTSUPP, but that is not reliable.
			 * don't try again for ANY return value != 0
			 * if (rv == -EOPNOTSUPP) */
			/* Any error is already reported by bio_endio callback. */
			drbd_bump_write_ordering(connection->resource, NULL, WO_DRAIN_IO);
		}
	}
}

/**
 * drbd_may_finish_epoch() - Applies an epoch_event to the epoch's state, eventually finishes it.
 * @device:	DRBD device.
 * @epoch:	Epoch object.
 * @ev:		Epoch event.
 */
static enum finish_epoch drbd_may_finish_epoch(struct drbd_connection *connection,
					       struct drbd_epoch *epoch,
					       enum epoch_event ev)
{
	int epoch_size;
	struct drbd_epoch *next_epoch;
	enum finish_epoch rv = FE_STILL_LIVE;

	spin_lock(&connection->epoch_lock);
	do {
		next_epoch = NULL;

		epoch_size = atomic_read(&epoch->epoch_size);

		switch (ev & ~EV_CLEANUP) {
		case EV_PUT:
			atomic_dec(&epoch->active);
			break;
		case EV_GOT_BARRIER_NR:
			set_bit(DE_HAVE_BARRIER_NUMBER, &epoch->flags);
			break;
		case EV_BECAME_LAST:
			/* nothing to do*/
			break;
		}

		if (epoch_size != 0 &&
		    atomic_read(&epoch->active) == 0 &&
		    (test_bit(DE_HAVE_BARRIER_NUMBER, &epoch->flags) || ev & EV_CLEANUP)) {
			if (!(ev & EV_CLEANUP)) {
				spin_unlock(&connection->epoch_lock);
				drbd_send_b_ack(epoch->connection, epoch->barrier_nr, epoch_size);
				spin_lock(&connection->epoch_lock);
			}
#if 0
			/* FIXME: dec unacked on connection, once we have
			 * something to count pending connection packets in. */
			if (test_bit(DE_HAVE_BARRIER_NUMBER, &epoch->flags))
				dec_unacked(epoch->connection);
#endif

			if (connection->current_epoch != epoch) {
				next_epoch = list_entry(epoch->list.next, struct drbd_epoch, list);
				list_del(&epoch->list);
				ev = EV_BECAME_LAST | (ev & EV_CLEANUP);
				connection->epochs--;
				kfree(epoch);

				if (rv == FE_STILL_LIVE)
					rv = FE_DESTROYED;
			} else {
				epoch->flags = 0;
				atomic_set(&epoch->epoch_size, 0);
				/* atomic_set(&epoch->active, 0); is already zero */
				if (rv == FE_STILL_LIVE)
					rv = FE_RECYCLED;
			}
		}

		if (!next_epoch)
			break;

		epoch = next_epoch;
	} while (1);

	spin_unlock(&connection->epoch_lock);

	return rv;
}

static enum write_ordering_e
max_allowed_wo(struct drbd_backing_dev *bdev, enum write_ordering_e wo)
{
	struct disk_conf *dc;

	dc = rcu_dereference(bdev->disk_conf);

	if (wo == WO_BDEV_FLUSH && !dc->disk_flushes)
		wo = WO_DRAIN_IO;
	if (wo == WO_DRAIN_IO && !dc->disk_drain)
		wo = WO_NONE;

	return wo;
}

/**
 * drbd_bump_write_ordering() - Fall back to an other write ordering method
 * @connection:	DRBD connection.
 * @wo:		Write ordering method to try.
 */
void drbd_bump_write_ordering(struct drbd_resource *resource, struct drbd_backing_dev *bdev,
			      enum write_ordering_e wo)
{
	struct drbd_device *device;
	enum write_ordering_e pwo;
	int vnr;
	static char *write_ordering_str[] = {
		[WO_NONE] = "none",
		[WO_DRAIN_IO] = "drain",
		[WO_BDEV_FLUSH] = "flush",
	};

	pwo = resource->write_ordering;
	if (wo != WO_BDEV_FLUSH)
		wo = min(pwo, wo);
	rcu_read_lock();
	idr_for_each_entry(&resource->devices, device, vnr) {
		if (get_ldev(device)) {
			wo = max_allowed_wo(device->ldev, wo);
			if (device->ldev == bdev)
				bdev = NULL;
			put_ldev(device);
		}
	}

	if (bdev)
		wo = max_allowed_wo(bdev, wo);

	rcu_read_unlock();

	resource->write_ordering = wo;
	if (pwo != resource->write_ordering || wo == WO_BDEV_FLUSH)
		drbd_info(resource, "Method to ensure write ordering: %s\n", write_ordering_str[resource->write_ordering]);
}

static void drbd_issue_peer_discard(struct drbd_device *device, struct drbd_peer_request *peer_req)
{
	struct block_device *bdev = device->ldev->backing_bdev;

	if (blkdev_issue_zeroout(bdev, peer_req->i.sector, peer_req->i.size >> 9,
			GFP_NOIO, 0))
		peer_req->flags |= EE_WAS_ERROR;

	drbd_endio_write_sec_final(peer_req);
}

static void drbd_issue_peer_wsame(struct drbd_device *device,
				  struct drbd_peer_request *peer_req)
{
	struct block_device *bdev = device->ldev->backing_bdev;
	sector_t s = peer_req->i.sector;
	sector_t nr = peer_req->i.size >> 9;
	if (blkdev_issue_write_same(bdev, s, nr, GFP_NOIO, peer_req->pages))
		peer_req->flags |= EE_WAS_ERROR;
	drbd_endio_write_sec_final(peer_req);
}


/**
 * drbd_submit_peer_request()
 * @device:	DRBD device.
 * @peer_req:	peer request
 * @rw:		flag field, see bio->bi_opf
 *
 * May spread the pages to multiple bios,
 * depending on bio_add_page restrictions.
 *
 * Returns 0 if all bios have been submitted,
 * -ENOMEM if we could not allocate enough bios,
 * -ENOSPC (any better suggestion?) if we have not been able to bio_add_page a
 *  single page to an empty bio (which should never happen and likely indicates
 *  that the lower level IO stack is in some way broken). This has been observed
 *  on certain Xen deployments.
 */
/* TODO allocate from our own bio_set. */
int drbd_submit_peer_request(struct drbd_device *device,
			     struct drbd_peer_request *peer_req,
			     const unsigned op, const unsigned op_flags,
			     const int fault_type)
{
	struct bio *bios = NULL;
	struct bio *bio;
	struct page *page = peer_req->pages;
	sector_t sector = peer_req->i.sector;
	unsigned data_size = peer_req->i.size;
	unsigned n_bios = 0;
	unsigned nr_pages = (data_size + PAGE_SIZE -1) >> PAGE_SHIFT;
	int err = -ENOMEM;

	/* TRIM/DISCARD: for now, always use the helper function
	 * blkdev_issue_zeroout(..., discard=true).
	 * It's synchronous, but it does the right thing wrt. bio splitting.
	 * Correctness first, performance later.  Next step is to code an
	 * asynchronous variant of the same.
	 */
	if (peer_req->flags & (EE_IS_TRIM|EE_WRITE_SAME)) {
		/* wait for all pending IO completions, before we start
		 * zeroing things out. */
		conn_wait_active_ee_empty(peer_req->peer_device->connection);
		/* add it to the active list now,
		 * so we can find it to present it in debugfs */
		peer_req->submit_jif = jiffies;
		peer_req->flags |= EE_SUBMITTED;

		/* If this was a resync request from receive_rs_deallocated(),
		 * it is already on the sync_ee list */
		if (list_empty(&peer_req->w.list)) {
			spin_lock_irq(&device->resource->req_lock);
			list_add_tail(&peer_req->w.list, &device->active_ee);
			spin_unlock_irq(&device->resource->req_lock);
		}

		if (peer_req->flags & EE_IS_TRIM)
			drbd_issue_peer_discard(device, peer_req);
		else /* EE_WRITE_SAME */
			drbd_issue_peer_wsame(device, peer_req);
		return 0;
	}

	/* In most cases, we will only need one bio.  But in case the lower
	 * level restrictions happen to be different at this offset on this
	 * side than those of the sending peer, we may need to submit the
	 * request in more than one bio.
	 *
	 * Plain bio_alloc is good enough here, this is no DRBD internally
	 * generated bio, but a bio allocated on behalf of the peer.
	 */
next_bio:
	bio = bio_alloc(GFP_NOIO, nr_pages);
	if (!bio) {
		drbd_err(device, "submit_ee: Allocation of a bio failed (nr_pages=%u)\n", nr_pages);
		goto fail;
	}
	/* > peer_req->i.sector, unless this is the first bio */
	bio->bi_iter.bi_sector = sector;
	bio_set_dev(bio, device->ldev->backing_bdev);
	bio_set_op_attrs(bio, op, op_flags);
	bio->bi_private = peer_req;
	bio->bi_end_io = drbd_peer_request_endio;

	bio->bi_next = bios;
	bios = bio;
	++n_bios;

	page_chain_for_each(page) {
		unsigned len = min_t(unsigned, data_size, PAGE_SIZE);
		if (!bio_add_page(bio, page, len, 0))
			goto next_bio;
		data_size -= len;
		sector += len >> 9;
		--nr_pages;
	}
	D_ASSERT(device, data_size == 0);
	D_ASSERT(device, page == NULL);

	atomic_set(&peer_req->pending_bios, n_bios);
	/* for debugfs: update timestamp, mark as submitted */
	peer_req->submit_jif = jiffies;
	peer_req->flags |= EE_SUBMITTED;
	do {
		bio = bios;
		bios = bios->bi_next;
		bio->bi_next = NULL;

		drbd_generic_make_request(device, fault_type, bio);
	} while (bios);
	return 0;

fail:
	while (bios) {
		bio = bios;
		bios = bios->bi_next;
		bio_put(bio);
	}
	return err;
}

static void drbd_remove_epoch_entry_interval(struct drbd_device *device,
					     struct drbd_peer_request *peer_req)
{
	struct drbd_interval *i = &peer_req->i;

	drbd_remove_interval(&device->write_requests, i);
	drbd_clear_interval(i);

	/* Wake up any processes waiting for this peer request to complete.  */
	if (i->waiting)
		wake_up(&device->misc_wait);
}

static void conn_wait_active_ee_empty(struct drbd_connection *connection)
{
	struct drbd_peer_device *peer_device;
	int vnr;

	rcu_read_lock();
	idr_for_each_entry(&connection->peer_devices, peer_device, vnr) {
		struct drbd_device *device = peer_device->device;

		kref_get(&device->kref);
		rcu_read_unlock();
		drbd_wait_ee_list_empty(device, &device->active_ee);
		kref_put(&device->kref, drbd_destroy_device);
		rcu_read_lock();
	}
	rcu_read_unlock();
}

static int receive_Barrier(struct drbd_connection *connection, struct packet_info *pi)
{
	int rv;
	struct p_barrier *p = pi->data;
	struct drbd_epoch *epoch;

	/* FIXME these are unacked on connection,
	 * not a specific (peer)device.
	 */
	connection->current_epoch->barrier_nr = p->barrier;
	connection->current_epoch->connection = connection;
	rv = drbd_may_finish_epoch(connection, connection->current_epoch, EV_GOT_BARRIER_NR);

	/* P_BARRIER_ACK may imply that the corresponding extent is dropped from
	 * the activity log, which means it would not be resynced in case the
	 * R_PRIMARY crashes now.
	 * Therefore we must send the barrier_ack after the barrier request was
	 * completed. */
	switch (connection->resource->write_ordering) {
	case WO_NONE:
		if (rv == FE_RECYCLED)
			return 0;

		/* receiver context, in the writeout path of the other node.
		 * avoid potential distributed deadlock */
		epoch = kmalloc(sizeof(struct drbd_epoch), GFP_NOIO);
		if (epoch)
			break;
		else
			drbd_warn(connection, "Allocation of an epoch failed, slowing down\n");
			/* Fall through */

	case WO_BDEV_FLUSH:
	case WO_DRAIN_IO:
		conn_wait_active_ee_empty(connection);
		drbd_flush(connection);

		if (atomic_read(&connection->current_epoch->epoch_size)) {
			epoch = kmalloc(sizeof(struct drbd_epoch), GFP_NOIO);
			if (epoch)
				break;
		}

		return 0;
	default:
		drbd_err(connection, "Strangeness in connection->write_ordering %d\n",
			 connection->resource->write_ordering);
		return -EIO;
	}

	epoch->flags = 0;
	atomic_set(&epoch->epoch_size, 0);
	atomic_set(&epoch->active, 0);

	spin_lock(&connection->epoch_lock);
	if (atomic_read(&connection->current_epoch->epoch_size)) {
		list_add(&epoch->list, &connection->current_epoch->list);
		connection->current_epoch = epoch;
		connection->epochs++;
	} else {
		/* The current_epoch got recycled while we allocated this one... */
		kfree(epoch);
	}
	spin_unlock(&connection->epoch_lock);

	return 0;
}

/* quick wrapper in case payload size != request_size (write same) */
static void drbd_csum_ee_size(struct crypto_ahash *h,
			      struct drbd_peer_request *r, void *d,
			      unsigned int payload_size)
{
	unsigned int tmp = r->i.size;
	r->i.size = payload_size;
	drbd_csum_ee(h, r, d);
	r->i.size = tmp;
}

/* used from receive_RSDataReply (recv_resync_read)
 * and from receive_Data.
 * data_size: actual payload ("data in")
 * 	for normal writes that is bi_size.
 * 	for discards, that is zero.
 * 	for write same, it is logical_block_size.
 * both trim and write same have the bi_size ("data len to be affected")
 * as extra argument in the packet header.
 */
static struct drbd_peer_request *
read_in_block(struct drbd_peer_device *peer_device, u64 id, sector_t sector,
	      struct packet_info *pi) __must_hold(local)
{
	struct drbd_device *device = peer_device->device;
	const sector_t capacity = drbd_get_capacity(device->this_bdev);
	struct drbd_peer_request *peer_req;
	struct page *page;
	int digest_size, err;
	unsigned int data_size = pi->size, ds;
	void *dig_in = peer_device->connection->int_dig_in;
	void *dig_vv = peer_device->connection->int_dig_vv;
	unsigned long *data;
	struct p_trim *trim = (pi->cmd == P_TRIM) ? pi->data : NULL;
	struct p_trim *wsame = (pi->cmd == P_WSAME) ? pi->data : NULL;

	digest_size = 0;
	if (!trim && peer_device->connection->peer_integrity_tfm) {
		digest_size = crypto_ahash_digestsize(peer_device->connection->peer_integrity_tfm);
		/*
		 * FIXME: Receive the incoming digest into the receive buffer
		 *	  here, together with its struct p_data?
		 */
		err = drbd_recv_all_warn(peer_device->connection, dig_in, digest_size);
		if (err)
			return NULL;
		data_size -= digest_size;
	}

	/* assume request_size == data_size, but special case trim and wsame. */
	ds = data_size;
	if (trim) {
		if (!expect(data_size == 0))
			return NULL;
		ds = be32_to_cpu(trim->size);
	} else if (wsame) {
		if (data_size != queue_logical_block_size(device->rq_queue)) {
			drbd_err(peer_device, "data size (%u) != drbd logical block size (%u)\n",
				data_size, queue_logical_block_size(device->rq_queue));
			return NULL;
		}
		if (data_size != bdev_logical_block_size(device->ldev->backing_bdev)) {
			drbd_err(peer_device, "data size (%u) != backend logical block size (%u)\n",
				data_size, bdev_logical_block_size(device->ldev->backing_bdev));
			return NULL;
		}
		ds = be32_to_cpu(wsame->size);
	}

	if (!expect(IS_ALIGNED(ds, 512)))
		return NULL;
	if (trim || wsame) {
		if (!expect(ds <= (DRBD_MAX_BBIO_SECTORS << 9)))
			return NULL;
	} else if (!expect(ds <= DRBD_MAX_BIO_SIZE))
		return NULL;

	/* even though we trust out peer,
	 * we sometimes have to double check. */
	if (sector + (ds>>9) > capacity) {
		drbd_err(device, "request from peer beyond end of local disk: "
			"capacity: %llus < sector: %llus + size: %u\n",
			(unsigned long long)capacity,
			(unsigned long long)sector, ds);
		return NULL;
	}

	/* GFP_NOIO, because we must not cause arbitrary write-out: in a DRBD
	 * "criss-cross" setup, that might cause write-out on some other DRBD,
	 * which in turn might block on the other node at this very place.  */
	peer_req = drbd_alloc_peer_req(peer_device, id, sector, ds, data_size, GFP_NOIO);
	if (!peer_req)
		return NULL;

	peer_req->flags |= EE_WRITE;
	if (trim) {
		peer_req->flags |= EE_IS_TRIM;
		return peer_req;
	}
	if (wsame)
		peer_req->flags |= EE_WRITE_SAME;

	/* receive payload size bytes into page chain */
	ds = data_size;
	page = peer_req->pages;
	page_chain_for_each(page) {
		unsigned len = min_t(int, ds, PAGE_SIZE);
		data = kmap(page);
		err = drbd_recv_all_warn(peer_device->connection, data, len);
		if (drbd_insert_fault(device, DRBD_FAULT_RECEIVE)) {
			drbd_err(device, "Fault injection: Corrupting data on receive\n");
			data[0] = data[0] ^ (unsigned long)-1;
		}
		kunmap(page);
		if (err) {
			drbd_free_peer_req(device, peer_req);
			return NULL;
		}
		ds -= len;
	}

	if (digest_size) {
		drbd_csum_ee_size(peer_device->connection->peer_integrity_tfm, peer_req, dig_vv, data_size);
		if (memcmp(dig_in, dig_vv, digest_size)) {
			drbd_err(device, "Digest integrity check FAILED: %llus +%u\n",
				(unsigned long long)sector, data_size);
			drbd_free_peer_req(device, peer_req);
			return NULL;
		}
	}
	device->recv_cnt += data_size >> 9;
	return peer_req;
}

/* drbd_drain_block() just takes a data block
 * out of the socket input buffer, and discards it.
 */
static int drbd_drain_block(struct drbd_peer_device *peer_device, int data_size)
{
	struct page *page;
	int err = 0;
	void *data;

	if (!data_size)
		return 0;

	page = drbd_alloc_pages(peer_device, 1, 1);

	data = kmap(page);
	while (data_size) {
		unsigned int len = min_t(int, data_size, PAGE_SIZE);

		err = drbd_recv_all_warn(peer_device->connection, data, len);
		if (err)
			break;
		data_size -= len;
	}
	kunmap(page);
	drbd_free_pages(peer_device->device, page, 0);
	return err;
}

static int recv_dless_read(struct drbd_peer_device *peer_device, struct drbd_request *req,
			   sector_t sector, int data_size)
{
	struct bio_vec bvec;
	struct bvec_iter iter;
	struct bio *bio;
	int digest_size, err, expect;
	void *dig_in = peer_device->connection->int_dig_in;
	void *dig_vv = peer_device->connection->int_dig_vv;

	digest_size = 0;
	if (peer_device->connection->peer_integrity_tfm) {
		digest_size = crypto_ahash_digestsize(peer_device->connection->peer_integrity_tfm);
		err = drbd_recv_all_warn(peer_device->connection, dig_in, digest_size);
		if (err)
			return err;
		data_size -= digest_size;
	}

	/* optimistically update recv_cnt.  if receiving fails below,
	 * we disconnect anyways, and counters will be reset. */
	peer_device->device->recv_cnt += data_size>>9;

	bio = req->master_bio;
	D_ASSERT(peer_device->device, sector == bio->bi_iter.bi_sector);

	bio_for_each_segment(bvec, bio, iter) {
		void *mapped = kmap(bvec.bv_page) + bvec.bv_offset;
		expect = min_t(int, data_size, bvec.bv_len);
		err = drbd_recv_all_warn(peer_device->connection, mapped, expect);
		kunmap(bvec.bv_page);
		if (err)
			return err;
		data_size -= expect;
	}

	if (digest_size) {
		drbd_csum_bio(peer_device->connection->peer_integrity_tfm, bio, dig_vv);
		if (memcmp(dig_in, dig_vv, digest_size)) {
			drbd_err(peer_device, "Digest integrity check FAILED. Broken NICs?\n");
			return -EINVAL;
		}
	}

	D_ASSERT(peer_device->device, data_size == 0);
	return 0;
}

/*
 * e_end_resync_block() is called in ack_sender context via
 * drbd_finish_peer_reqs().
 */
static int e_end_resync_block(struct drbd_work *w, int unused)
{
	struct drbd_peer_request *peer_req =
		container_of(w, struct drbd_peer_request, w);
	struct drbd_peer_device *peer_device = peer_req->peer_device;
	struct drbd_device *device = peer_device->device;
	sector_t sector = peer_req->i.sector;
	int err;

	D_ASSERT(device, drbd_interval_empty(&peer_req->i));

	if (likely((peer_req->flags & EE_WAS_ERROR) == 0)) {
		drbd_set_in_sync(device, sector, peer_req->i.size);
		err = drbd_send_ack(peer_device, P_RS_WRITE_ACK, peer_req);
	} else {
		/* Record failure to sync */
		drbd_rs_failed_io(device, sector, peer_req->i.size);

		err  = drbd_send_ack(peer_device, P_NEG_ACK, peer_req);
	}
	dec_unacked(device);

	return err;
}

static int recv_resync_read(struct drbd_peer_device *peer_device, sector_t sector,
			    struct packet_info *pi) __releases(local)
{
	struct drbd_device *device = peer_device->device;
	struct drbd_peer_request *peer_req;

	peer_req = read_in_block(peer_device, ID_SYNCER, sector, pi);
	if (!peer_req)
		goto fail;

	dec_rs_pending(device);

	inc_unacked(device);
	/* corresponding dec_unacked() in e_end_resync_block()
	 * respective _drbd_clear_done_ee */

	peer_req->w.cb = e_end_resync_block;
	peer_req->submit_jif = jiffies;

	spin_lock_irq(&device->resource->req_lock);
	list_add_tail(&peer_req->w.list, &device->sync_ee);
	spin_unlock_irq(&device->resource->req_lock);

	atomic_add(pi->size >> 9, &device->rs_sect_ev);
	if (drbd_submit_peer_request(device, peer_req, REQ_OP_WRITE, 0,
				     DRBD_FAULT_RS_WR) == 0)
		return 0;

	/* don't care for the reason here */
	drbd_err(device, "submit failed, triggering re-connect\n");
	spin_lock_irq(&device->resource->req_lock);
	list_del(&peer_req->w.list);
	spin_unlock_irq(&device->resource->req_lock);

	drbd_free_peer_req(device, peer_req);
fail:
	put_ldev(device);
	return -EIO;
}

static struct drbd_request *
find_request(struct drbd_device *device, struct rb_root *root, u64 id,
	     sector_t sector, bool missing_ok, const char *func)
{
	struct drbd_request *req;

	/* Request object according to our peer */
	req = (struct drbd_request *)(unsigned long)id;
	if (drbd_contains_interval(root, sector, &req->i) && req->i.local)
		return req;
	if (!missing_ok) {
		drbd_err(device, "%s: failed to find request 0x%lx, sector %llus\n", func,
			(unsigned long)id, (unsigned long long)sector);
	}
	return NULL;
}

static int receive_DataReply(struct drbd_connection *connection, struct packet_info *pi)
{
	struct drbd_peer_device *peer_device;
	struct drbd_device *device;
	struct drbd_request *req;
	sector_t sector;
	int err;
	struct p_data *p = pi->data;

	peer_device = conn_peer_device(connection, pi->vnr);
	if (!peer_device)
		return -EIO;
	device = peer_device->device;

	sector = be64_to_cpu(p->sector);

	spin_lock_irq(&device->resource->req_lock);
	req = find_request(device, &device->read_requests, p->block_id, sector, false, __func__);
	spin_unlock_irq(&device->resource->req_lock);
	if (unlikely(!req))
		return -EIO;

	/* hlist_del(&req->collision) is done in _req_may_be_done, to avoid
	 * special casing it there for the various failure cases.
	 * still no race with drbd_fail_pending_reads */
	err = recv_dless_read(peer_device, req, sector, pi->size);
	if (!err)
		req_mod(req, DATA_RECEIVED);
	/* else: nothing. handled from drbd_disconnect...
	 * I don't think we may complete this just yet
	 * in case we are "on-disconnect: freeze" */

	return err;
}

static int receive_RSDataReply(struct drbd_connection *connection, struct packet_info *pi)
{
	struct drbd_peer_device *peer_device;
	struct drbd_device *device;
	sector_t sector;
	int err;
	struct p_data *p = pi->data;

	peer_device = conn_peer_device(connection, pi->vnr);
	if (!peer_device)
		return -EIO;
	device = peer_device->device;

	sector = be64_to_cpu(p->sector);
	D_ASSERT(device, p->block_id == ID_SYNCER);

	if (get_ldev(device)) {
		/* data is submitted to disk within recv_resync_read.
		 * corresponding put_ldev done below on error,
		 * or in drbd_peer_request_endio. */
		err = recv_resync_read(peer_device, sector, pi);
	} else {
		if (__ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not write resync data to local disk.\n");

		err = drbd_drain_block(peer_device, pi->size);

		drbd_send_ack_dp(peer_device, P_NEG_ACK, p, pi->size);
	}

	atomic_add(pi->size >> 9, &device->rs_sect_in);

	return err;
}

static void restart_conflicting_writes(struct drbd_device *device,
				       sector_t sector, int size)
{
	struct drbd_interval *i;
	struct drbd_request *req;

	drbd_for_each_overlap(i, &device->write_requests, sector, size) {
		if (!i->local)
			continue;
		req = container_of(i, struct drbd_request, i);
		if (req->rq_state & RQ_LOCAL_PENDING ||
		    !(req->rq_state & RQ_POSTPONED))
			continue;
		/* as it is RQ_POSTPONED, this will cause it to
		 * be queued on the retry workqueue. */
		__req_mod(req, CONFLICT_RESOLVED, NULL);
	}
}

/*
 * e_end_block() is called in ack_sender context via drbd_finish_peer_reqs().
 */
static int e_end_block(struct drbd_work *w, int cancel)
{
	struct drbd_peer_request *peer_req =
		container_of(w, struct drbd_peer_request, w);
	struct drbd_peer_device *peer_device = peer_req->peer_device;
	struct drbd_device *device = peer_device->device;
	sector_t sector = peer_req->i.sector;
	int err = 0, pcmd;

	if (peer_req->flags & EE_SEND_WRITE_ACK) {
		if (likely((peer_req->flags & EE_WAS_ERROR) == 0)) {
			pcmd = (device->state.conn >= C_SYNC_SOURCE &&
				device->state.conn <= C_PAUSED_SYNC_T &&
				peer_req->flags & EE_MAY_SET_IN_SYNC) ?
				P_RS_WRITE_ACK : P_WRITE_ACK;
			err = drbd_send_ack(peer_device, pcmd, peer_req);
			if (pcmd == P_RS_WRITE_ACK)
				drbd_set_in_sync(device, sector, peer_req->i.size);
		} else {
			err = drbd_send_ack(peer_device, P_NEG_ACK, peer_req);
			/* we expect it to be marked out of sync anyways...
			 * maybe assert this?  */
		}
		dec_unacked(device);
	}

	/* we delete from the conflict detection hash _after_ we sent out the
	 * P_WRITE_ACK / P_NEG_ACK, to get the sequence number right.  */
	if (peer_req->flags & EE_IN_INTERVAL_TREE) {
		spin_lock_irq(&device->resource->req_lock);
		D_ASSERT(device, !drbd_interval_empty(&peer_req->i));
		drbd_remove_epoch_entry_interval(device, peer_req);
		if (peer_req->flags & EE_RESTART_REQUESTS)
			restart_conflicting_writes(device, sector, peer_req->i.size);
		spin_unlock_irq(&device->resource->req_lock);
	} else
		D_ASSERT(device, drbd_interval_empty(&peer_req->i));

	drbd_may_finish_epoch(peer_device->connection, peer_req->epoch, EV_PUT + (cancel ? EV_CLEANUP : 0));

	return err;
}

static int e_send_ack(struct drbd_work *w, enum drbd_packet ack)
{
	struct drbd_peer_request *peer_req =
		container_of(w, struct drbd_peer_request, w);
	struct drbd_peer_device *peer_device = peer_req->peer_device;
	int err;

	err = drbd_send_ack(peer_device, ack, peer_req);
	dec_unacked(peer_device->device);

	return err;
}

static int e_send_superseded(struct drbd_work *w, int unused)
{
	return e_send_ack(w, P_SUPERSEDED);
}

static int e_send_retry_write(struct drbd_work *w, int unused)
{
	struct drbd_peer_request *peer_req =
		container_of(w, struct drbd_peer_request, w);
	struct drbd_connection *connection = peer_req->peer_device->connection;

	return e_send_ack(w, connection->agreed_pro_version >= 100 ?
			     P_RETRY_WRITE : P_SUPERSEDED);
}

static bool seq_greater(u32 a, u32 b)
{
	/*
	 * We assume 32-bit wrap-around here.
	 * For 24-bit wrap-around, we would have to shift:
	 *  a <<= 8; b <<= 8;
	 */
	return (s32)a - (s32)b > 0;
}

static u32 seq_max(u32 a, u32 b)
{
	return seq_greater(a, b) ? a : b;
}

static void update_peer_seq(struct drbd_peer_device *peer_device, unsigned int peer_seq)
{
	struct drbd_device *device = peer_device->device;
	unsigned int newest_peer_seq;

	if (test_bit(RESOLVE_CONFLICTS, &peer_device->connection->flags)) {
		spin_lock(&device->peer_seq_lock);
		newest_peer_seq = seq_max(device->peer_seq, peer_seq);
		device->peer_seq = newest_peer_seq;
		spin_unlock(&device->peer_seq_lock);
		/* wake up only if we actually changed device->peer_seq */
		if (peer_seq == newest_peer_seq)
			wake_up(&device->seq_wait);
	}
}

static inline int overlaps(sector_t s1, int l1, sector_t s2, int l2)
{
	return !((s1 + (l1>>9) <= s2) || (s1 >= s2 + (l2>>9)));
}

/* maybe change sync_ee into interval trees as well? */
static bool overlapping_resync_write(struct drbd_device *device, struct drbd_peer_request *peer_req)
{
	struct drbd_peer_request *rs_req;
	bool rv = false;

	spin_lock_irq(&device->resource->req_lock);
	list_for_each_entry(rs_req, &device->sync_ee, w.list) {
		if (overlaps(peer_req->i.sector, peer_req->i.size,
			     rs_req->i.sector, rs_req->i.size)) {
			rv = true;
			break;
		}
	}
	spin_unlock_irq(&device->resource->req_lock);

	return rv;
}

/* Called from receive_Data.
 * Synchronize packets on sock with packets on msock.
 *
 * This is here so even when a P_DATA packet traveling via sock overtook an Ack
 * packet traveling on msock, they are still processed in the order they have
 * been sent.
 *
 * Note: we don't care for Ack packets overtaking P_DATA packets.
 *
 * In case packet_seq is larger than device->peer_seq number, there are
 * outstanding packets on the msock. We wait for them to arrive.
 * In case we are the logically next packet, we update device->peer_seq
 * ourselves. Correctly handles 32bit wrap around.
 *
 * Assume we have a 10 GBit connection, that is about 1<<30 byte per second,
 * about 1<<21 sectors per second. So "worst" case, we have 1<<3 == 8 seconds
 * for the 24bit wrap (historical atomic_t guarantee on some archs), and we have
 * 1<<9 == 512 seconds aka ages for the 32bit wrap around...
 *
 * returns 0 if we may process the packet,
 * -ERESTARTSYS if we were interrupted (by disconnect signal). */
static int wait_for_and_update_peer_seq(struct drbd_peer_device *peer_device, const u32 peer_seq)
{
	struct drbd_device *device = peer_device->device;
	DEFINE_WAIT(wait);
	long timeout;
	int ret = 0, tp;

	if (!test_bit(RESOLVE_CONFLICTS, &peer_device->connection->flags))
		return 0;

	spin_lock(&device->peer_seq_lock);
	for (;;) {
		if (!seq_greater(peer_seq - 1, device->peer_seq)) {
			device->peer_seq = seq_max(device->peer_seq, peer_seq);
			break;
		}

		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}

		rcu_read_lock();
		tp = rcu_dereference(peer_device->connection->net_conf)->two_primaries;
		rcu_read_unlock();

		if (!tp)
			break;

		/* Only need to wait if two_primaries is enabled */
		prepare_to_wait(&device->seq_wait, &wait, TASK_INTERRUPTIBLE);
		spin_unlock(&device->peer_seq_lock);
		rcu_read_lock();
		timeout = rcu_dereference(peer_device->connection->net_conf)->ping_timeo*HZ/10;
		rcu_read_unlock();
		timeout = schedule_timeout(timeout);
		spin_lock(&device->peer_seq_lock);
		if (!timeout) {
			ret = -ETIMEDOUT;
			drbd_err(device, "Timed out waiting for missing ack packets; disconnecting\n");
			break;
		}
	}
	spin_unlock(&device->peer_seq_lock);
	finish_wait(&device->seq_wait, &wait);
	return ret;
}

/* see also bio_flags_to_wire()
 * DRBD_REQ_*, because we need to semantically map the flags to data packet
 * flags and back. We may replicate to other kernel versions. */
static unsigned long wire_flags_to_bio_flags(u32 dpf)
{
	return  (dpf & DP_RW_SYNC ? REQ_SYNC : 0) |
		(dpf & DP_FUA ? REQ_FUA : 0) |
		(dpf & DP_FLUSH ? REQ_PREFLUSH : 0);
}

static unsigned long wire_flags_to_bio_op(u32 dpf)
{
	if (dpf & DP_DISCARD)
		return REQ_OP_WRITE_ZEROES;
	else
		return REQ_OP_WRITE;
}

static void fail_postponed_requests(struct drbd_device *device, sector_t sector,
				    unsigned int size)
{
	struct drbd_interval *i;

    repeat:
	drbd_for_each_overlap(i, &device->write_requests, sector, size) {
		struct drbd_request *req;
		struct bio_and_error m;

		if (!i->local)
			continue;
		req = container_of(i, struct drbd_request, i);
		if (!(req->rq_state & RQ_POSTPONED))
			continue;
		req->rq_state &= ~RQ_POSTPONED;
		__req_mod(req, NEG_ACKED, &m);
		spin_unlock_irq(&device->resource->req_lock);
		if (m.bio)
			complete_master_bio(device, &m);
		spin_lock_irq(&device->resource->req_lock);
		goto repeat;
	}
}

static int handle_write_conflicts(struct drbd_device *device,
				  struct drbd_peer_request *peer_req)
{
	struct drbd_connection *connection = peer_req->peer_device->connection;
	bool resolve_conflicts = test_bit(RESOLVE_CONFLICTS, &connection->flags);
	sector_t sector = peer_req->i.sector;
	const unsigned int size = peer_req->i.size;
	struct drbd_interval *i;
	bool equal;
	int err;

	/*
	 * Inserting the peer request into the write_requests tree will prevent
	 * new conflicting local requests from being added.
	 */
	drbd_insert_interval(&device->write_requests, &peer_req->i);

    repeat:
	drbd_for_each_overlap(i, &device->write_requests, sector, size) {
		if (i == &peer_req->i)
			continue;
		if (i->completed)
			continue;

		if (!i->local) {
			/*
			 * Our peer has sent a conflicting remote request; this
			 * should not happen in a two-node setup.  Wait for the
			 * earlier peer request to complete.
			 */
			err = drbd_wait_misc(device, i);
			if (err)
				goto out;
			goto repeat;
		}

		equal = i->sector == sector && i->size == size;
		if (resolve_conflicts) {
			/*
			 * If the peer request is fully contained within the
			 * overlapping request, it can be considered overwritten
			 * and thus superseded; otherwise, it will be retried
			 * once all overlapping requests have completed.
			 */
			bool superseded = i->sector <= sector && i->sector +
				       (i->size >> 9) >= sector + (size >> 9);

			if (!equal)
				drbd_alert(device, "Concurrent writes detected: "
					       "local=%llus +%u, remote=%llus +%u, "
					       "assuming %s came first\n",
					  (unsigned long long)i->sector, i->size,
					  (unsigned long long)sector, size,
					  superseded ? "local" : "remote");

			peer_req->w.cb = superseded ? e_send_superseded :
						   e_send_retry_write;
			list_add_tail(&peer_req->w.list, &device->done_ee);
			queue_work(connection->ack_sender, &peer_req->peer_device->send_acks_work);

			err = -ENOENT;
			goto out;
		} else {
			struct drbd_request *req =
				container_of(i, struct drbd_request, i);

			if (!equal)
				drbd_alert(device, "Concurrent writes detected: "
					       "local=%llus +%u, remote=%llus +%u\n",
					  (unsigned long long)i->sector, i->size,
					  (unsigned long long)sector, size);

			if (req->rq_state & RQ_LOCAL_PENDING ||
			    !(req->rq_state & RQ_POSTPONED)) {
				/*
				 * Wait for the node with the discard flag to
				 * decide if this request has been superseded
				 * or needs to be retried.
				 * Requests that have been superseded will
				 * disappear from the write_requests tree.
				 *
				 * In addition, wait for the conflicting
				 * request to finish locally before submitting
				 * the conflicting peer request.
				 */
				err = drbd_wait_misc(device, &req->i);
				if (err) {
					_conn_request_state(connection, NS(conn, C_TIMEOUT), CS_HARD);
					fail_postponed_requests(device, sector, size);
					goto out;
				}
				goto repeat;
			}
			/*
			 * Remember to restart the conflicting requests after
			 * the new peer request has completed.
			 */
			peer_req->flags |= EE_RESTART_REQUESTS;
		}
	}
	err = 0;

    out:
	if (err)
		drbd_remove_epoch_entry_interval(device, peer_req);
	return err;
}

/* mirrored write */
static int receive_Data(struct drbd_connection *connection, struct packet_info *pi)
{
	struct drbd_peer_device *peer_device;
	struct drbd_device *device;
	struct net_conf *nc;
	sector_t sector;
	struct drbd_peer_request *peer_req;
	struct p_data *p = pi->data;
	u32 peer_seq = be32_to_cpu(p->seq_num);
	int op, op_flags;
	u32 dp_flags;
	int err, tp;

	peer_device = conn_peer_device(connection, pi->vnr);
	if (!peer_device)
		return -EIO;
	device = peer_device->device;

	if (!get_ldev(device)) {
		int err2;

		err = wait_for_and_update_peer_seq(peer_device, peer_seq);
		drbd_send_ack_dp(peer_device, P_NEG_ACK, p, pi->size);
		atomic_inc(&connection->current_epoch->epoch_size);
		err2 = drbd_drain_block(peer_device, pi->size);
		if (!err)
			err = err2;
		return err;
	}

	/*
	 * Corresponding put_ldev done either below (on various errors), or in
	 * drbd_peer_request_endio, if we successfully submit the data at the
	 * end of this function.
	 */

	sector = be64_to_cpu(p->sector);
	peer_req = read_in_block(peer_device, p->block_id, sector, pi);
	if (!peer_req) {
		put_ldev(device);
		return -EIO;
	}

	peer_req->w.cb = e_end_block;
	peer_req->submit_jif = jiffies;
	peer_req->flags |= EE_APPLICATION;

	dp_flags = be32_to_cpu(p->dp_flags);
	op = wire_flags_to_bio_op(dp_flags);
	op_flags = wire_flags_to_bio_flags(dp_flags);
	if (pi->cmd == P_TRIM) {
		D_ASSERT(peer_device, peer_req->i.size > 0);
		D_ASSERT(peer_device, op == REQ_OP_WRITE_ZEROES);
		D_ASSERT(peer_device, peer_req->pages == NULL);
	} else if (peer_req->pages == NULL) {
		D_ASSERT(device, peer_req->i.size == 0);
		D_ASSERT(device, dp_flags & DP_FLUSH);
	}

	if (dp_flags & DP_MAY_SET_IN_SYNC)
		peer_req->flags |= EE_MAY_SET_IN_SYNC;

	spin_lock(&connection->epoch_lock);
	peer_req->epoch = connection->current_epoch;
	atomic_inc(&peer_req->epoch->epoch_size);
	atomic_inc(&peer_req->epoch->active);
	spin_unlock(&connection->epoch_lock);

	rcu_read_lock();
	nc = rcu_dereference(peer_device->connection->net_conf);
	tp = nc->two_primaries;
	if (peer_device->connection->agreed_pro_version < 100) {
		switch (nc->wire_protocol) {
		case DRBD_PROT_C:
			dp_flags |= DP_SEND_WRITE_ACK;
			break;
		case DRBD_PROT_B:
			dp_flags |= DP_SEND_RECEIVE_ACK;
			break;
		}
	}
	rcu_read_unlock();

	if (dp_flags & DP_SEND_WRITE_ACK) {
		peer_req->flags |= EE_SEND_WRITE_ACK;
		inc_unacked(device);
		/* corresponding dec_unacked() in e_end_block()
		 * respective _drbd_clear_done_ee */
	}

	if (dp_flags & DP_SEND_RECEIVE_ACK) {
		/* I really don't like it that the receiver thread
		 * sends on the msock, but anyways */
		drbd_send_ack(peer_device, P_RECV_ACK, peer_req);
	}

	if (tp) {
		/* two primaries implies protocol C */
		D_ASSERT(device, dp_flags & DP_SEND_WRITE_ACK);
		peer_req->flags |= EE_IN_INTERVAL_TREE;
		err = wait_for_and_update_peer_seq(peer_device, peer_seq);
		if (err)
			goto out_interrupted;
		spin_lock_irq(&device->resource->req_lock);
		err = handle_write_conflicts(device, peer_req);
		if (err) {
			spin_unlock_irq(&device->resource->req_lock);
			if (err == -ENOENT) {
				put_ldev(device);
				return 0;
			}
			goto out_interrupted;
		}
	} else {
		update_peer_seq(peer_device, peer_seq);
		spin_lock_irq(&device->resource->req_lock);
	}
	/* TRIM and WRITE_SAME are processed synchronously,
	 * we wait for all pending requests, respectively wait for
	 * active_ee to become empty in drbd_submit_peer_request();
	 * better not add ourselves here. */
	if ((peer_req->flags & (EE_IS_TRIM|EE_WRITE_SAME)) == 0)
		list_add_tail(&peer_req->w.list, &device->active_ee);
	spin_unlock_irq(&device->resource->req_lock);

	if (device->state.conn == C_SYNC_TARGET)
		wait_event(device->ee_wait, !overlapping_resync_write(device, peer_req));

	if (device->state.pdsk < D_INCONSISTENT) {
		/* In case we have the only disk of the cluster, */
		drbd_set_out_of_sync(device, peer_req->i.sector, peer_req->i.size);
		peer_req->flags &= ~EE_MAY_SET_IN_SYNC;
		drbd_al_begin_io(device, &peer_req->i);
		peer_req->flags |= EE_CALL_AL_COMPLETE_IO;
	}

	err = drbd_submit_peer_request(device, peer_req, op, op_flags,
				       DRBD_FAULT_DT_WR);
	if (!err)
		return 0;

	/* don't care for the reason here */
	drbd_err(device, "submit failed, triggering re-connect\n");
	spin_lock_irq(&device->resource->req_lock);
	list_del(&peer_req->w.list);
	drbd_remove_epoch_entry_interval(device, peer_req);
	spin_unlock_irq(&device->resource->req_lock);
	if (peer_req->flags & EE_CALL_AL_COMPLETE_IO) {
		peer_req->flags &= ~EE_CALL_AL_COMPLETE_IO;
		drbd_al_complete_io(device, &peer_req->i);
	}

out_interrupted:
	drbd_may_finish_epoch(connection, peer_req->epoch, EV_PUT | EV_CLEANUP);
	put_ldev(device);
	drbd_free_peer_req(device, peer_req);
	return err;
}

/* We may throttle resync, if the lower device seems to be busy,
 * and current sync rate is above c_min_rate.
 *
 * To decide whether or not the lower device is busy, we use a scheme similar
 * to MD RAID is_mddev_idle(): if the partition stats reveal "significant"
 * (more than 64 sectors) of activity we cannot account for with our own resync
 * activity, it obviously is "busy".
 *
 * The current sync rate used here uses only the most recent two step marks,
 * to have a short time average so we can react faster.
 */
bool drbd_rs_should_slow_down(struct drbd_device *device, sector_t sector,
		bool throttle_if_app_is_waiting)
{
	struct lc_element *tmp;
	bool throttle = drbd_rs_c_min_rate_throttle(device);

	if (!throttle || throttle_if_app_is_waiting)
		return throttle;

	spin_lock_irq(&device->al_lock);
	tmp = lc_find(device->resync, BM_SECT_TO_EXT(sector));
	if (tmp) {
		struct bm_extent *bm_ext = lc_entry(tmp, struct bm_extent, lce);
		if (test_bit(BME_PRIORITY, &bm_ext->flags))
			throttle = false;
		/* Do not slow down if app IO is already waiting for this extent,
		 * and our progress is necessary for application IO to complete. */
	}
	spin_unlock_irq(&device->al_lock);

	return throttle;
}

bool drbd_rs_c_min_rate_throttle(struct drbd_device *device)
{
	struct gendisk *disk = device->ldev->backing_bdev->bd_contains->bd_disk;
	unsigned long db, dt, dbdt;
	unsigned int c_min_rate;
	int curr_events;

	rcu_read_lock();
	c_min_rate = rcu_dereference(device->ldev->disk_conf)->c_min_rate;
	rcu_read_unlock();

	/* feature disabled? */
	if (c_min_rate == 0)
		return false;

	curr_events = (int)part_stat_read(&disk->part0, sectors[0]) +
		      (int)part_stat_read(&disk->part0, sectors[1]) -
			atomic_read(&device->rs_sect_ev);

	if (atomic_read(&device->ap_actlog_cnt)
	    || curr_events - device->rs_last_events > 64) {
		unsigned long rs_left;
		int i;

		device->rs_last_events = curr_events;

		/* sync speed average over the last 2*DRBD_SYNC_MARK_STEP,
		 * approx. */
		i = (device->rs_last_mark + DRBD_SYNC_MARKS-1) % DRBD_SYNC_MARKS;

		if (device->state.conn == C_VERIFY_S || device->state.conn == C_VERIFY_T)
			rs_left = device->ov_left;
		else
			rs_left = drbd_bm_total_weight(device) - device->rs_failed;

		dt = ((long)jiffies - (long)device->rs_mark_time[i]) / HZ;
		if (!dt)
			dt++;
		db = device->rs_mark_left[i] - rs_left;
		dbdt = Bit2KB(db/dt);

		if (dbdt > c_min_rate)
			return true;
	}
	return false;
}

static int receive_DataRequest(struct drbd_connection *connection, struct packet_info *pi)
{
	struct drbd_peer_device *peer_device;
	struct drbd_device *device;
	sector_t sector;
	sector_t capacity;
	struct drbd_peer_request *peer_req;
	struct digest_info *di = NULL;
	int size, verb;
	unsigned int fault_type;
	struct p_block_req *p =	pi->data;

	peer_device = conn_peer_device(connection, pi->vnr);
	if (!peer_device)
		return -EIO;
	device = peer_device->device;
	capacity = drbd_get_capacity(device->this_bdev);

	sector = be64_to_cpu(p->sector);
	size   = be32_to_cpu(p->blksize);

	if (size <= 0 || !IS_ALIGNED(size, 512) || size > DRBD_MAX_BIO_SIZE) {
		drbd_err(device, "%s:%d: sector: %llus, size: %u\n", __FILE__, __LINE__,
				(unsigned long long)sector, size);
		return -EINVAL;
	}
	if (sector + (size>>9) > capacity) {
		drbd_err(device, "%s:%d: sector: %llus, size: %u\n", __FILE__, __LINE__,
				(unsigned long long)sector, size);
		return -EINVAL;
	}

	if (!get_ldev_if_state(device, D_UP_TO_DATE)) {
		verb = 1;
		switch (pi->cmd) {
		case P_DATA_REQUEST:
			drbd_send_ack_rp(peer_device, P_NEG_DREPLY, p);
			break;
		case P_RS_THIN_REQ:
		case P_RS_DATA_REQUEST:
		case P_CSUM_RS_REQUEST:
		case P_OV_REQUEST:
			drbd_send_ack_rp(peer_device, P_NEG_RS_DREPLY , p);
			break;
		case P_OV_REPLY:
			verb = 0;
			dec_rs_pending(device);
			drbd_send_ack_ex(peer_device, P_OV_RESULT, sector, size, ID_IN_SYNC);
			break;
		default:
			BUG();
		}
		if (verb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		/* drain possibly payload */
		return drbd_drain_block(peer_device, pi->size);
	}

	/* GFP_NOIO, because we must not cause arbitrary write-out: in a DRBD
	 * "criss-cross" setup, that might cause write-out on some other DRBD,
	 * which in turn might block on the other node at this very place.  */
	peer_req = drbd_alloc_peer_req(peer_device, p->block_id, sector, size,
			size, GFP_NOIO);
	if (!peer_req) {
		put_ldev(device);
		return -ENOMEM;
	}

	switch (pi->cmd) {
	case P_DATA_REQUEST:
		peer_req->w.cb = w_e_end_data_req;
		fault_type = DRBD_FAULT_DT_RD;
		/* application IO, don't drbd_rs_begin_io */
		peer_req->flags |= EE_APPLICATION;
		goto submit;

	case P_RS_THIN_REQ:
		/* If at some point in the future we have a smart way to
		   find out if this data block is completely deallocated,
		   then we would do something smarter here than reading
		   the block... */
		peer_req->flags |= EE_RS_THIN_REQ;
	case P_RS_DATA_REQUEST:
		peer_req->w.cb = w_e_end_rsdata_req;
		fault_type = DRBD_FAULT_RS_RD;
		/* used in the sector offset progress display */
		device->bm_resync_fo = BM_SECT_TO_BIT(sector);
		break;

	case P_OV_REPLY:
	case P_CSUM_RS_REQUEST:
		fault_type = DRBD_FAULT_RS_RD;
		di = kmalloc(sizeof(*di) + pi->size, GFP_NOIO);
		if (!di)
			goto out_free_e;

		di->digest_size = pi->size;
		di->digest = (((char *)di)+sizeof(struct digest_info));

		peer_req->digest = di;
		peer_req->flags |= EE_HAS_DIGEST;

		if (drbd_recv_all(peer_device->connection, di->digest, pi->size))
			goto out_free_e;

		if (pi->cmd == P_CSUM_RS_REQUEST) {
			D_ASSERT(device, peer_device->connection->agreed_pro_version >= 89);
			peer_req->w.cb = w_e_end_csum_rs_req;
			/* used in the sector offset progress display */
			device->bm_resync_fo = BM_SECT_TO_BIT(sector);
			/* remember to report stats in drbd_resync_finished */
			device->use_csums = true;
		} else if (pi->cmd == P_OV_REPLY) {
			/* track progress, we may need to throttle */
			atomic_add(size >> 9, &device->rs_sect_in);
			peer_req->w.cb = w_e_end_ov_reply;
			dec_rs_pending(device);
			/* drbd_rs_begin_io done when we sent this request,
			 * but accounting still needs to be done. */
			goto submit_for_resync;
		}
		break;

	case P_OV_REQUEST:
		if (device->ov_start_sector == ~(sector_t)0 &&
		    peer_device->connection->agreed_pro_version >= 90) {
			unsigned long now = jiffies;
			int i;
			device->ov_start_sector = sector;
			device->ov_position = sector;
			device->ov_left = drbd_bm_bits(device) - BM_SECT_TO_BIT(sector);
			device->rs_total = device->ov_left;
			for (i = 0; i < DRBD_SYNC_MARKS; i++) {
				device->rs_mark_left[i] = device->ov_left;
				device->rs_mark_time[i] = now;
			}
			drbd_info(device, "Online Verify start sector: %llu\n",
					(unsigned long long)sector);
		}
		peer_req->w.cb = w_e_end_ov_req;
		fault_type = DRBD_FAULT_RS_RD;
		break;

	default:
		BUG();
	}

	/* Throttle, drbd_rs_begin_io and submit should become asynchronous
	 * wrt the receiver, but it is not as straightforward as it may seem.
	 * Various places in the resync start and stop logic assume resync
	 * requests are processed in order, requeuing this on the worker thread
	 * introduces a bunch of new code for synchronization between threads.
	 *
	 * Unlimited throttling before drbd_rs_begin_io may stall the resync
	 * "forever", throttling after drbd_rs_begin_io will lock that extent
	 * for application writes for the same time.  For now, just throttle
	 * here, where the rest of the code expects the receiver to sleep for
	 * a while, anyways.
	 */

	/* Throttle before drbd_rs_begin_io, as that locks out application IO;
	 * this defers syncer requests for some time, before letting at least
	 * on request through.  The resync controller on the receiving side
	 * will adapt to the incoming rate accordingly.
	 *
	 * We cannot throttle here if remote is Primary/SyncTarget:
	 * we would also throttle its application reads.
	 * In that case, throttling is done on the SyncTarget only.
	 */

	/* Even though this may be a resync request, we do add to "read_ee";
	 * "sync_ee" is only used for resync WRITEs.
	 * Add to list early, so debugfs can find this request
	 * even if we have to sleep below. */
	spin_lock_irq(&device->resource->req_lock);
	list_add_tail(&peer_req->w.list, &device->read_ee);
	spin_unlock_irq(&device->resource->req_lock);

	update_receiver_timing_details(connection, drbd_rs_should_slow_down);
	if (device->state.peer != R_PRIMARY
	&& drbd_rs_should_slow_down(device, sector, false))
		schedule_timeout_uninterruptible(HZ/10);
	update_receiver_timing_details(connection, drbd_rs_begin_io);
	if (drbd_rs_begin_io(device, sector))
		goto out_free_e;

submit_for_resync:
	atomic_add(size >> 9, &device->rs_sect_ev);

submit:
	update_receiver_timing_details(connection, drbd_submit_peer_request);
	inc_unacked(device);
	if (drbd_submit_peer_request(device, peer_req, REQ_OP_READ, 0,
				     fault_type) == 0)
		return 0;

	/* don't care for the reason here */
	drbd_err(device, "submit failed, triggering re-connect\n");

out_free_e:
	spin_lock_irq(&device->resource->req_lock);
	list_del(&peer_req->w.list);
	spin_unlock_irq(&device->resource->req_lock);
	/* no drbd_rs_complete_io(), we are dropping the connection anyways */

	put_ldev(device);
	drbd_free_peer_req(device, peer_req);
	return -EIO;
}

/**
 * drbd_asb_recover_0p  -  Recover after split-brain with no remaining primaries
 */
static int drbd_asb_recover_0p(struct drbd_peer_device *peer_device) __must_hold(local)
{
	struct drbd_device *device = peer_device->device;
	int self, peer, rv = -100;
	unsigned long ch_self, ch_peer;
	enum drbd_after_sb_p after_sb_0p;

	self = device->ldev->md.uuid[UI_BITMAP] & 1;
	peer = device->p_uuid[UI_BITMAP] & 1;

	ch_peer = device->p_uuid[UI_SIZE];
	ch_self = device->comm_bm_set;

	rcu_read_lock();
	after_sb_0p = rcu_dereference(peer_device->connection->net_conf)->after_sb_0p;
	rcu_read_unlock();
	switch (after_sb_0p) {
	case ASB_CONSENSUS:
	case ASB_DISCARD_SECONDARY:
	case ASB_CALL_HELPER:
	case ASB_VIOLENTLY:
		drbd_err(device, "Configuration error.\n");
		break;
	case ASB_DISCONNECT:
		break;
	case ASB_DISCARD_YOUNGER_PRI:
		if (self == 0 && peer == 1) {
			rv = -1;
			break;
		}
		if (self == 1 && peer == 0) {
			rv =  1;
			break;
		}
		/* Else fall through to one of the other strategies... */
	case ASB_DISCARD_OLDER_PRI:
		if (self == 0 && peer == 1) {
			rv = 1;
			break;
		}
		if (self == 1 && peer == 0) {
			rv = -1;
			break;
		}
		/* Else fall through to one of the other strategies... */
		drbd_warn(device, "Discard younger/older primary did not find a decision\n"
		     "Using discard-least-changes instead\n");
	case ASB_DISCARD_ZERO_CHG:
		if (ch_peer == 0 && ch_self == 0) {
			rv = test_bit(RESOLVE_CONFLICTS, &peer_device->connection->flags)
				? -1 : 1;
			break;
		} else {
			if (ch_peer == 0) { rv =  1; break; }
			if (ch_self == 0) { rv = -1; break; }
		}
		if (after_sb_0p == ASB_DISCARD_ZERO_CHG)
			break;
	case ASB_DISCARD_LEAST_CHG:
		if	(ch_self < ch_peer)
			rv = -1;
		else if (ch_self > ch_peer)
			rv =  1;
		else /* ( ch_self == ch_peer ) */
		     /* Well, then use something else. */
			rv = test_bit(RESOLVE_CONFLICTS, &peer_device->connection->flags)
				? -1 : 1;
		break;
	case ASB_DISCARD_LOCAL:
		rv = -1;
		break;
	case ASB_DISCARD_REMOTE:
		rv =  1;
	}

	return rv;
}

/**
 * drbd_asb_recover_1p  -  Recover after split-brain with one remaining primary
 */
static int drbd_asb_recover_1p(struct drbd_peer_device *peer_device) __must_hold(local)
{
	struct drbd_device *device = peer_device->device;
	int hg, rv = -100;
	enum drbd_after_sb_p after_sb_1p;

	rcu_read_lock();
	after_sb_1p = rcu_dereference(peer_device->connection->net_conf)->after_sb_1p;
	rcu_read_unlock();
	switch (after_sb_1p) {
	case ASB_DISCARD_YOUNGER_PRI:
	case ASB_DISCARD_OLDER_PRI:
	case ASB_DISCARD_LEAST_CHG:
	case ASB_DISCARD_LOCAL:
	case ASB_DISCARD_REMOTE:
	case ASB_DISCARD_ZERO_CHG:
		drbd_err(device, "Configuration error.\n");
		break;
	case ASB_DISCONNECT:
		break;
	case ASB_CONSENSUS:
		hg = drbd_asb_recover_0p(peer_device);
		if (hg == -1 && device->state.role == R_SECONDARY)
			rv = hg;
		if (hg == 1  && device->state.role == R_PRIMARY)
			rv = hg;
		break;
	case ASB_VIOLENTLY:
		rv = drbd_asb_recover_0p(peer_device);
		break;
	case ASB_DISCARD_SECONDARY:
		return device->state.role == R_PRIMARY ? 1 : -1;
	case ASB_CALL_HELPER:
		hg = drbd_asb_recover_0p(peer_device);
		if (hg == -1 && device->state.role == R_PRIMARY) {
			enum drbd_state_rv rv2;

			 /* drbd_change_state() does not sleep while in SS_IN_TRANSIENT_STATE,
			  * we might be here in C_WF_REPORT_PARAMS which is transient.
			  * we do not need to wait for the after state change work either. */
			rv2 = drbd_change_state(device, CS_VERBOSE, NS(role, R_SECONDARY));
			if (rv2 != SS_SUCCESS) {
				drbd_khelper(device, "pri-lost-after-sb");
			} else {
				drbd_warn(device, "Successfully gave up primary role.\n");
				rv = hg;
			}
		} else
			rv = hg;
	}

	return rv;
}

/**
 * drbd_asb_recover_2p  -  Recover after split-brain with two remaining primaries
 */
static int drbd_asb_recover_2p(struct drbd_peer_device *peer_device) __must_hold(local)
{
	struct drbd_device *device = peer_device->device;
	int hg, rv = -100;
	enum drbd_after_sb_p after_sb_2p;

	rcu_read_lock();
	after_sb_2p = rcu_dereference(peer_device->connection->net_conf)->after_sb_2p;
	rcu_read_unlock();
	switch (after_sb_2p) {
	case ASB_DISCARD_YOUNGER_PRI:
	case ASB_DISCARD_OLDER_PRI:
	case ASB_DISCARD_LEAST_CHG:
	case ASB_DISCARD_LOCAL:
	case ASB_DISCARD_REMOTE:
	case ASB_CONSENSUS:
	case ASB_DISCARD_SECONDARY:
	case ASB_DISCARD_ZERO_CHG:
		drbd_err(device, "Configuration error.\n");
		break;
	case ASB_VIOLENTLY:
		rv = drbd_asb_recover_0p(peer_device);
		break;
	case ASB_DISCONNECT:
		break;
	case ASB_CALL_HELPER:
		hg = drbd_asb_recover_0p(peer_device);
		if (hg == -1) {
			enum drbd_state_rv rv2;

			 /* drbd_change_state() does not sleep while in SS_IN_TRANSIENT_STATE,
			  * we might be here in C_WF_REPORT_PARAMS which is transient.
			  * we do not need to wait for the after state change work either. */
			rv2 = drbd_change_state(device, CS_VERBOSE, NS(role, R_SECONDARY));
			if (rv2 != SS_SUCCESS) {
				drbd_khelper(device, "pri-lost-after-sb");
			} else {
				drbd_warn(device, "Successfully gave up primary role.\n");
				rv = hg;
			}
		} else
			rv = hg;
	}

	return rv;
}

static void drbd_uuid_dump(struct drbd_device *device, char *text, u64 *uuid,
			   u64 bits, u64 flags)
{
	if (!uuid) {
		drbd_info(device, "%s uuid info vanished while I was looking!\n", text);
		return;
	}
	drbd_info(device, "%s %016llX:%016llX:%016llX:%016llX bits:%llu flags:%llX\n",
	     text,
	     (unsigned long long)uuid[UI_CURRENT],
	     (unsigned long long)uuid[UI_BITMAP],
	     (unsigned long long)uuid[UI_HISTORY_START],
	     (unsigned long long)uuid[UI_HISTORY_END],
	     (unsigned long long)bits,
	     (unsigned long long)flags);
}

/*
  100	after split brain try auto recover
    2	C_SYNC_SOURCE set BitMap
    1	C_SYNC_SOURCE use BitMap
    0	no Sync
   -1	C_SYNC_TARGET use BitMap
   -2	C_SYNC_TARGET set BitMap
 -100	after split brain, disconnect
-1000	unrelated data
-1091   requires proto 91
-1096   requires proto 96
 */

static int drbd_uuid_compare(struct drbd_device *const device, enum drbd_role const peer_role, int *rule_nr) __must_hold(local)
{
	struct drbd_peer_device *const peer_device = first_peer_device(device);
	struct drbd_connection *const connection = peer_device ? peer_device->connection : NULL;
	u64 self, peer;
	int i, j;

	self = device->ldev->md.uuid[UI_CURRENT] & ~((u64)1);
	peer = device->p_uuid[UI_CURRENT] & ~((u64)1);

	*rule_nr = 10;
	if (self == UUID_JUST_CREATED && peer == UUID_JUST_CREATED)
		return 0;

	*rule_nr = 20;
	if ((self == UUID_JUST_CREATED || self == (u64)0) &&
	     peer != UUID_JUST_CREATED)
		return -2;

	*rule_nr = 30;
	if (self != UUID_JUST_CREATED &&
	    (peer == UUID_JUST_CREATED || peer == (u64)0))
		return 2;

	if (self == peer) {
		int rct, dc; /* roles at crash time */

		if (device->p_uuid[UI_BITMAP] == (u64)0 && device->ldev->md.uuid[UI_BITMAP] != (u64)0) {

			if (connection->agreed_pro_version < 91)
				return -1091;

			if ((device->ldev->md.uuid[UI_BITMAP] & ~((u64)1)) == (device->p_uuid[UI_HISTORY_START] & ~((u64)1)) &&
			    (device->ldev->md.uuid[UI_HISTORY_START] & ~((u64)1)) == (device->p_uuid[UI_HISTORY_START + 1] & ~((u64)1))) {
				drbd_info(device, "was SyncSource, missed the resync finished event, corrected myself:\n");
				drbd_uuid_move_history(device);
				device->ldev->md.uuid[UI_HISTORY_START] = device->ldev->md.uuid[UI_BITMAP];
				device->ldev->md.uuid[UI_BITMAP] = 0;

				drbd_uuid_dump(device, "self", device->ldev->md.uuid,
					       device->state.disk >= D_NEGOTIATING ? drbd_bm_total_weight(device) : 0, 0);
				*rule_nr = 34;
			} else {
				drbd_info(device, "was SyncSource (peer failed to write sync_uuid)\n");
				*rule_nr = 36;
			}

			return 1;
		}

		if (device->ldev->md.uuid[UI_BITMAP] == (u64)0 && device->p_uuid[UI_BITMAP] != (u64)0) {

			if (connection->agreed_pro_version < 91)
				return -1091;

			if ((device->ldev->md.uuid[UI_HISTORY_START] & ~((u64)1)) == (device->p_uuid[UI_BITMAP] & ~((u64)1)) &&
			    (device->ldev->md.uuid[UI_HISTORY_START + 1] & ~((u64)1)) == (device->p_uuid[UI_HISTORY_START] & ~((u64)1))) {
				drbd_info(device, "was SyncTarget, peer missed the resync finished event, corrected peer:\n");

				device->p_uuid[UI_HISTORY_START + 1] = device->p_uuid[UI_HISTORY_START];
				device->p_uuid[UI_HISTORY_START] = device->p_uuid[UI_BITMAP];
				device->p_uuid[UI_BITMAP] = 0UL;

				drbd_uuid_dump(device, "peer", device->p_uuid, device->p_uuid[UI_SIZE], device->p_uuid[UI_FLAGS]);
				*rule_nr = 35;
			} else {
				drbd_info(device, "was SyncTarget (failed to write sync_uuid)\n");
				*rule_nr = 37;
			}

			return -1;
		}

		/* Common power [off|failure] */
		rct = (test_bit(CRASHED_PRIMARY, &device->flags) ? 1 : 0) +
			(device->p_uuid[UI_FLAGS] & 2);
		/* lowest bit is set when we were primary,
		 * next bit (weight 2) is set when peer was primary */
		*rule_nr = 40;

		/* Neither has the "crashed primary" flag set,
		 * only a replication link hickup. */
		if (rct == 0)
			return 0;

		/* Current UUID equal and no bitmap uuid; does not necessarily
		 * mean this was a "simultaneous hard crash", maybe IO was
		 * frozen, so no UUID-bump happened.
		 * This is a protocol change, overload DRBD_FF_WSAME as flag
		 * for "new-enough" peer DRBD version. */
		if (device->state.role == R_PRIMARY || peer_role == R_PRIMARY) {
			*rule_nr = 41;
			if (!(connection->agreed_features & DRBD_FF_WSAME)) {
				drbd_warn(peer_device, "Equivalent unrotated UUIDs, but current primary present.\n");
				return -(0x10000 | PRO_VERSION_MAX | (DRBD_FF_WSAME << 8));
			}
			if (device->state.role == R_PRIMARY && peer_role == R_PRIMARY) {
				/* At least one has the "crashed primary" bit set,
				 * both are primary now, but neither has rotated its UUIDs?
				 * "Can not happen." */
				drbd_err(peer_device, "Equivalent unrotated UUIDs, but both are primary. Can not resolve this.\n");
				return -100;
			}
			if (device->state.role == R_PRIMARY)
				return 1;
			return -1;
		}

		/* Both are secondary.
		 * Really looks like recovery from simultaneous hard crash.
		 * Check which had been primary before, and arbitrate. */
		switch (rct) {
		case 0: /* !self_pri && !peer_pri */ return 0; /* already handled */
		case 1: /*  self_pri && !peer_pri */ return 1;
		case 2: /* !self_pri &&  peer_pri */ return -1;
		case 3: /*  self_pri &&  peer_pri */
			dc = test_bit(RESOLVE_CONFLICTS, &connection->flags);
			return dc ? -1 : 1;
		}
	}

	*rule_nr = 50;
	peer = device->p_uuid[UI_BITMAP] & ~((u64)1);
	if (self == peer)
		return -1;

	*rule_nr = 51;
	peer = device->p_uuid[UI_HISTORY_START] & ~((u64)1);
	if (self == peer) {
		if (connection->agreed_pro_version < 96 ?
		    (device->ldev->md.uuid[UI_HISTORY_START] & ~((u64)1)) ==
		    (device->p_uuid[UI_HISTORY_START + 1] & ~((u64)1)) :
		    peer + UUID_NEW_BM_OFFSET == (device->p_uuid[UI_BITMAP] & ~((u64)1))) {
			/* The last P_SYNC_UUID did not get though. Undo the last start of
			   resync as sync source modifications of the peer's UUIDs. */

			if (connection->agreed_pro_version < 91)
				return -1091;

			device->p_uuid[UI_BITMAP] = device->p_uuid[UI_HISTORY_START];
			device->p_uuid[UI_HISTORY_START] = device->p_uuid[UI_HISTORY_START + 1];

			drbd_info(device, "Lost last syncUUID packet, corrected:\n");
			drbd_uuid_dump(device, "peer", device->p_uuid, device->p_uuid[UI_SIZE], device->p_uuid[UI_FLAGS]);

			return -1;
		}
	}

	*rule_nr = 60;
	self = device->ldev->md.uuid[UI_CURRENT] & ~((u64)1);
	for (i = UI_HISTORY_START; i <= UI_HISTORY_END; i++) {
		peer = device->p_uuid[i] & ~((u64)1);
		if (self == peer)
			return -2;
	}

	*rule_nr = 70;
	self = device->ldev->md.uuid[UI_BITMAP] & ~((u64)1);
	peer = device->p_uuid[UI_CURRENT] & ~((u64)1);
	if (self == peer)
		return 1;

	*rule_nr = 71;
	self = device->ldev->md.uuid[UI_HISTORY_START] & ~((u64)1);
	if (self == peer) {
		if (connection->agreed_pro_version < 96 ?
		    (device->ldev->md.uuid[UI_HISTORY_START + 1] & ~((u64)1)) ==
		    (device->p_uuid[UI_HISTORY_START] & ~((u64)1)) :
		    self + UUID_NEW_BM_OFFSET == (device->ldev->md.uuid[UI_BITMAP] & ~((u64)1))) {
			/* The last P_SYNC_UUID did not get though. Undo the last start of
			   resync as sync source modifications of our UUIDs. */

			if (connection->agreed_pro_version < 91)
				return -1091;

			__drbd_uuid_set(device, UI_BITMAP, device->ldev->md.uuid[UI_HISTORY_START]);
			__drbd_uuid_set(device, UI_HISTORY_START, device->ldev->md.uuid[UI_HISTORY_START + 1]);

			drbd_info(device, "Last syncUUID did not get through, corrected:\n");
			drbd_uuid_dump(device, "self", device->ldev->md.uuid,
				       device->state.disk >= D_NEGOTIATING ? drbd_bm_total_weight(device) : 0, 0);

			return 1;
		}
	}


	*rule_nr = 80;
	peer = device->p_uuid[UI_CURRENT] & ~((u64)1);
	for (i = UI_HISTORY_START; i <= UI_HISTORY_END; i++) {
		self = device->ldev->md.uuid[i] & ~((u64)1);
		if (self == peer)
			return 2;
	}

	*rule_nr = 90;
	self = device->ldev->md.uuid[UI_BITMAP] & ~((u64)1);
	peer = device->p_uuid[UI_BITMAP] & ~((u64)1);
	if (self == peer && self != ((u64)0))
		return 100;

	*rule_nr = 100;
	for (i = UI_HISTORY_START; i <= UI_HISTORY_END; i++) {
		self = device->ldev->md.uuid[i] & ~((u64)1);
		for (j = UI_HISTORY_START; j <= UI_HISTORY_END; j++) {
			peer = device->p_uuid[j] & ~((u64)1);
			if (self == peer)
				return -100;
		}
	}

	return -1000;
}

/* drbd_sync_handshake() returns the new conn state on success, or
   CONN_MASK (-1) on failure.
 */
static enum drbd_conns drbd_sync_handshake(struct drbd_peer_device *peer_device,
					   enum drbd_role peer_role,
					   enum drbd_disk_state peer_disk) __must_hold(local)
{
	struct drbd_device *device = peer_device->device;
	enum drbd_conns rv = C_MASK;
	enum drbd_disk_state mydisk;
	struct net_conf *nc;
	int hg, rule_nr, rr_conflict, tentative, always_asbp;

	mydisk = device->state.disk;
	if (mydisk == D_NEGOTIATING)
		mydisk = device->new_state_tmp.disk;

	drbd_info(device, "drbd_sync_handshake:\n");

	spin_lock_irq(&device->ldev->md.uuid_lock);
	drbd_uuid_dump(device, "self", device->ldev->md.uuid, device->comm_bm_set, 0);
	drbd_uuid_dump(device, "peer", device->p_uuid,
		       device->p_uuid[UI_SIZE], device->p_uuid[UI_FLAGS]);

	hg = drbd_uuid_compare(device, peer_role, &rule_nr);
	spin_unlock_irq(&device->ldev->md.uuid_lock);

	drbd_info(device, "uuid_compare()=%d by rule %d\n", hg, rule_nr);

	if (hg == -1000) {
		drbd_alert(device, "Unrelated data, aborting!\n");
		return C_MASK;
	}
	if (hg < -0x10000) {
		int proto, fflags;
		hg = -hg;
		proto = hg & 0xff;
		fflags = (hg >> 8) & 0xff;
		drbd_alert(device, "To resolve this both sides have to support at least protocol %d and feature flags 0x%x\n",
					proto, fflags);
		return C_MASK;
	}
	if (hg < -1000) {
		drbd_alert(device, "To resolve this both sides have to support at least protocol %d\n", -hg - 1000);
		return C_MASK;
	}

	if    ((mydisk == D_INCONSISTENT && peer_disk > D_INCONSISTENT) ||
	    (peer_disk == D_INCONSISTENT && mydisk    > D_INCONSISTENT)) {
		int f = (hg == -100) || abs(hg) == 2;
		hg = mydisk > D_INCONSISTENT ? 1 : -1;
		if (f)
			hg = hg*2;
		drbd_info(device, "Becoming sync %s due to disk states.\n",
		     hg > 0 ? "source" : "target");
	}

	if (abs(hg) == 100)
		drbd_khelper(device, "initial-split-brain");

	rcu_read_lock();
	nc = rcu_dereference(peer_device->connection->net_conf);
	always_asbp = nc->always_asbp;
	rr_conflict = nc->rr_conflict;
	tentative = nc->tentative;
	rcu_read_unlock();

	if (hg == 100 || (hg == -100 && always_asbp)) {
		int pcount = (device->state.role == R_PRIMARY)
			   + (peer_role == R_PRIMARY);
		int forced = (hg == -100);

		switch (pcount) {
		case 0:
			hg = drbd_asb_recover_0p(peer_device);
			break;
		case 1:
			hg = drbd_asb_recover_1p(peer_device);
			break;
		case 2:
			hg = drbd_asb_recover_2p(peer_device);
			break;
		}
		if (abs(hg) < 100) {
			drbd_warn(device, "Split-Brain detected, %d primaries, "
			     "automatically solved. Sync from %s node\n",
			     pcount, (hg < 0) ? "peer" : "this");
			if (forced) {
				drbd_warn(device, "Doing a full sync, since"
				     " UUIDs where ambiguous.\n");
				hg = hg*2;
			}
		}
	}

	if (hg == -100) {
		if (test_bit(DISCARD_MY_DATA, &device->flags) && !(device->p_uuid[UI_FLAGS]&1))
			hg = -1;
		if (!test_bit(DISCARD_MY_DATA, &device->flags) && (device->p_uuid[UI_FLAGS]&1))
			hg = 1;

		if (abs(hg) < 100)
			drbd_warn(device, "Split-Brain detected, manually solved. "
			     "Sync from %s node\n",
			     (hg < 0) ? "peer" : "this");
	}

	if (hg == -100) {
		/* FIXME this log message is not correct if we end up here
		 * after an attempted attach on a diskless node.
		 * We just refuse to attach -- well, we drop the "connection"
		 * to that disk, in a way... */
		drbd_alert(device, "Split-Brain detected but unresolved, dropping connection!\n");
		drbd_khelper(device, "split-brain");
		return C_MASK;
	}

	if (hg > 0 && mydisk <= D_INCONSISTENT) {
		drbd_err(device, "I shall become SyncSource, but I am inconsistent!\n");
		return C_MASK;
	}

	if (hg < 0 && /* by intention we do not use mydisk here. */
	    device->state.role == R_PRIMARY && device->state.disk >= D_CONSISTENT) {
		switch (rr_conflict) {
		case ASB_CALL_HELPER:
			drbd_khelper(device, "pri-lost");
			/* fall through */
		case ASB_DISCONNECT:
			drbd_err(device, "I shall become SyncTarget, but I am primary!\n");
			return C_MASK;
		case ASB_VIOLENTLY:
			drbd_warn(device, "Becoming SyncTarget, violating the stable-data"
			     "assumption\n");
		}
	}

	if (tentative || test_bit(CONN_DRY_RUN, &peer_device->connection->flags)) {
		if (hg == 0)
			drbd_info(device, "dry-run connect: No resync, would become Connected immediately.\n");
		else
			drbd_info(device, "dry-run connect: Would become %s, doing a %s resync.",
				 drbd_conn_str(hg > 0 ? C_SYNC_SOURCE : C_SYNC_TARGET),
				 abs(hg) >= 2 ? "full" : "bit-map based");
		return C_MASK;
	}

	if (abs(hg) >= 2) {
		drbd_info(device, "Writing the whole bitmap, full sync required after drbd_sync_handshake.\n");
		if (drbd_bitmap_io(device, &drbd_bmio_set_n_write, "set_n_write from sync_handshake",
					BM_LOCKED_SET_ALLOWED))
			return C_MASK;
	}

	if (hg > 0) { /* become sync source. */
		rv = C_WF_BITMAP_S;
	} else if (hg < 0) { /* become sync target */
		rv = C_WF_BITMAP_T;
	} else {
		rv = C_CONNECTED;
		if (drbd_bm_total_weight(device)) {
			drbd_info(device, "No resync, but %lu bits in bitmap!\n",
			     drbd_bm_total_weight(device));
		}
	}

	return rv;
}

static enum drbd_after_sb_p convert_after_sb(enum drbd_after_sb_p peer)
{
	/* ASB_DISCARD_REMOTE - ASB_DISCARD_LOCAL is vTzleep while inMARuct drbd_peer_device *peer_device,
					   enum drbd_role peer_role,
					   enum drbd_disk_state 0zync request, Wweight(device) : 0, 0);
				*rule_nr = 34;
			} else {
				drbd_info(device, "was     peer != UUID_JUST_CREATED)
		return -2;

	*rule_nr = 30;
	if (self != UUID_JUST_CREATED &&
	    (peer == UUID_JUST_CREATED || peer == (u64)0))
		return 2;

	ull sync rbd_info(device, "was SyncTarget (failed to write sync_uuid)\n");
				*rule_nr = 37;
			}

			retur drbd_hg < 0) { WY_START] & ~((u64)1)) == (device->p_uuid[UI_BITMAP] & ~((u64)1)) &&
			    (device;
}

/* drbd_sync_handshake() returns the new conn state on success, or
   CONN_MASK (-1) on failure.
 */
static enum drbd_conns drbd_sync_handshake(struct drbd_drbdM1GwbGwbGwbGPer_2p(zTHIN_REQ:
		/Tc_handshake(strucj>HIN_REQ:
)1))) {
				drbd_info(device, "was SyncSource, missed the resync finished event, cic enum dryself:\n");
				drbd_uuid_move_history(device);
				device->ldev->md.uuid[UI_HISTORY_START] = deviight(deviccHISRY_START] = deO_n(peer_device,0&JUST_CR	     "automatically solved. Sync from %s node\n",
			     pcount, (hg < 0) ? "peer" : "this");
		)bGwbGwbGgA		/Tc_handshake(strucj>HIN &&
	    (PW-1000) {
		drbd_alert(device, "To resolve this both sides have to support at leastzbGwbGgA		/Tc_GwbGUce"
				 o that disk, in a way... */
		drbd_alert(device, "Split-Brain detected but unresolved, dropping connection!\n");
		drbd_khelper(device, "split-brain");
		return evi)
		drbd_khelper(device, "initial-split-brain");

	rcu_read_lock();
	nc = rcu_deree->p_u:
		/Tc_haW ~((u64)1)) == (device->p_uuid[UI_BITMAP] & ~((u64)1)) &&
			    (device->ldev->mdess, or
  s hard crash", maybe IO was
		 * frozen, so no UUID-bump happened.
		 * This is a protocol change,tected, manually solved. "
			     "Sync from %s node\n",
			     (hg < 0) ? "peer" : "this");
	}

	if (hg == -100) {
		/* FIXME this log message is not correct  disTzc from %s nodGwdeerbGP whenjWbGwbGh>Siwce->p_uuid[UI_BITMAP];
				device->p_uuid[UI_BITMAP] = 0UL;

				drbd_uuid_dump(de from %s ne->ldezITMAP];
				dwg == -100);

		switch (pcount) {
		case 0:
			hg = drbd_asb_recover_0p(peer_devicezd_uuid_dump(dGg	if   c fromBrain deterTzuid_dump(dGg	 turn C_jG0er_ happened.
		 * T1rn C_jG0er_ ha	 turn CG0er_ ha	 turn CG0er_ ha	 turn CG0er_ ha	 turn CG0er>p_uuid[NSISTE+"Yp_io(devic2er_ ha	wbGh>Siwce->p p_uuid[NSISTE+"YpwbGh>Siwce->pf (hgffromBrain deterTzui the  hg*2;
		nodGwdeerbGP when p_cf & CF_e, "dry-run con& mydiskplaces in the resync start and stop7 "I shall u_rea
receiver, {
	case	peer_rump(de from %uuid[UI_HISTOer nodettle_if_app_ Throttle "
			     "ump(de from %goto out_free_ock);
	list_ds;

	rcu_reashall(de from %s ne->ldezITMAP];
testimary *] = deviirever", !hrotviceOCOL_UPe thact drbctor,turn 2;

	ull sync 64)1);
	if (self == e_ock);cf & CF_eull syist_d);
	turn 2;

	ull sync 64)1);
	if (self == e_o that disk, in a wway... */
		drbd_alertit-Brain detected but u
receivef   c  !hrl(&peer_req->w.list, &d Wweight(d "
			     ""ng thpaASB_Dsyncser_reqs
		if"eq->w.li_info(dak;
	I_HISTORY__ */
hg > 0c_GwbGe_ock);crt(device, "Spliturn C_jG0er_) !hrl(&precover_1p  -  Rd Wweight(d "
			     ""ng thpaASB_Dsyncser_reqs
		if"if (self-0>Si_info(dak;
	I_HISTORY__ */
hg > 0c_GwbGe_ock);crt(device, "Spliturn C_jG0e1_) !hrl(&precover_11  -  Rd Wweight(d "
			     ""ng thpaASB_Dsyncser_reqs
		if"if (self-1>Si_info(dak;
	I_HISTORY__ */
hg > 0c_GwbGe_ock);crt(device, "Spliturn C_jG0e2_) !hrl(&precover_12  -  Rd Wweight(d "
			     ""ng thpaASB_Dsyncser_reqs
		if"if (self-2>Si_info(dak;
	I_HISTORY__ */
hg > 0c_GwbGe_ock);nodGwdeerbGP when  ASl(&pdGwdeerbGP when -  Rd Wweight(d "
			     ""ng thpaASB_Dsyncser_reqs
		if"asb_recomyTED &&info(dak;
	I_HISTORY__ */
hg > 0c_GwbGe_ock);nowbGh>Siwce->p !hrl(&pwbGh>Siwce->pf-  Rd Wweight(d "
			     ""ng thpaASB_Dsyncser_reqs
		if"illow-wbG->Siwce->p&info(dak;
	I_HISTORY__ */
hg > 0c_GwbGe_ock);= -cmp(ump(de from %gol(&pump(de from %uu-  Rd Wweight(d "
			     ""ng thpaASB_Dsyncser_reqs
		if"aD &-ump(de fr-m %&info(dak;
	I_HISTORY__ */
hg > 0c_GwbGe_oUI_HISTORY_START],
	 = deviiall(de from %s> DRI shall tch _	(unsip, fun_write f00;
it(RE NULL;
device->k;

	call(de fr m %oe fhmn_writvert_a CNULL s hd(&devic;

	call(de fr m %oe fhmn_writdev->md.uSK;
	nywaysse AotviceOCOL_UPe throm %s  atturn C_submit_peer_;in");
wibrain vice->kRT] nosubmit_for_ C_e;
			to outY_END;om %s l %d\nm %oe fhmver_timfor_ C NULL;_n_wri = pecount) {
		case 0:
		;

		swiSECT_itch (ump(de from %go0, CRYPTOdevG_A(!di)
			eviiIS_ERR(count) {
		case 0:)				proto, t) {
		case 0:
			hg = dd Wweight(d "
			     ""ce->k;

	-ump(de fr-m %K;
	}
tuid[UI_Fedous.\n");"ump(de from %info(dak;
	I_HISTORY_c_GwbGe_otch _	(un
		;

		switch _ shoul	(un(count) {
		case 0:)eashallecover_0p(device->tch _	(un
			}
KERNEL)eashallecoveuid_ddevice->tch _	(un
			}
KERNEL)easha
		  allecover_0 ASallecoveuiuu-  Rd Wweight(d "
			     ""A(devicand of, "f(&devt at;

	call(de fr cISTOnnectoth abd_info(dak;
	I_HISTORY_c_GwbGwbGe_AP] = 0UL;

			device->rs_markI_BITMAP];
				)
			}
KERNEL)easa
		 AP] = 0UL;

	drbd_info(devi "
			     ""A(devicand of,", -= 0UL;

	toth abd_info(ak;
	I_HISTORY_c_GbGe_mutexsk, in 64)1);
	if (DIGESmutexinfomutexsk, in 64)1);
	if (not find aL;

_ 1) {
info>p_uuid[UI_B = ct-Brain detected bunfoMAP] = 0UL;

			->p_uuid[UI_B;Ge_AP] = 0UL;

&peer_req->w.li : "req->w;e_AP] = 0UL;

&p_DISCARD_LOCALcrt(device, "Spliturn C_jG0er_);e_AP] = 0UL;

&p_DISCARD_1OCALcrt(device, "Spliturn C_jG0e1_);e_AP] = 0UL;

&p_DISCARD_2OCALcrt(device, "Spliturn C_jG0e2_);e_AP] = 0UL;

&pwbGh>Siwce->p p_nowbGh>Siwce->p* to thaasTART_stillertit-Brain detected bu, AP] = 0UL;

	nfomutexsY_START 64)1);
	if (not find aL;

_ 1) {
infomutexsY_START 64)1);
	if (DIGESmutexinf(hg

		sw&& chitch (64)1);
	if (count) {
		case 0:)eask&& c(64)1);
	if (allecover_)eask&& c(64)1);
	if (allecovevvf (hg4)1);
	if (count) {
		case 0:g = hg*2) {
		case 0: (hg4)1);
	if (allecover_0p(allecover_ (hg4)1);
	if (allecoveuid_dallecoveuifinished  -cmp(>p_uuid[UI_B&pump(de from % "ump(de from %)to attach0) {
 "
			     ""ce->k;

	-ump(de fr-m %:K;
sides haveall(de from %s>  ?eall(de from %enum(none)= nc->rce, sectoe_ */()eask&& c(>p_uuid[UI_B)easc ? -1 : 1;I_HISTORY__ */
hg > 0:;
	}

	if (hg > 0 && I_HISTORY_:(hg

		sw&& chitch (count) {
		case 0:)eask&& c(allecover_)eask&& c(allecovevvf (hg4)1to one o device "
			     "NSe "
	, C 0zync requrenc->md.ne-D)easc ? -1 VE_CONFLICT 			    fun	    
s_begp, pem %enalistUUIDs whnali
ritde? -1			ret (m %enalilf =="")
ritttttttttERR_PTR(SYNC_)aybeSS_IN_TRANgid[Uwsecg
ritttttttttevice->g

		s tch  ptr,aybeitnnectic;
		}ok.CONSISTENT = -100);

		switch (p manua

		swiSECT_tor: %llafce "
isk_s)
		return -2;

	*rule_nr drbailed  30;
m % "bailed  30;
nali			     (hg ;

		switch (p 0: (asa
		 m %s> D4)1)) &&
	ump(dGg	 0:
		;

		swiSECT_itch (m %go0, CRYPTOdevG_A(!di)
		eviiIS_ERR( 0:)				pr_SYNC_TARGET use Bievice,  (device \";
s"ust_;
	(1;
			n if Reales ham %golalistPTR_ERR( 0:)	d_sync_hand 0: (hSYNC);
			 0: (		 * This is aARTor
			gs)
{
	_om %s ,tected, manually solved. "
			     "Sync from %s node\n",
			  rbd_as "f(&d = ct-Brain detDIGESr "f disTzclu\n",
					(unsi
	(self >rs_mDRI shall p p_ESULt(allP_NOIO); may bdeviT_BUFFEREST:
	d_sys_if_app_ Thre "
			     " "f(&dP_N);

	drbd_urn -)_recover_0s solv
alert(devicsll through */
		calu\n"-=csll  EE_APPLs_mD
[UI_HISTOer nodec ? -1 : 1vice->p* ct-foveunknevi_volle_t*tex -2;

	ctMap
   -2	C_) {
  " Ueviunknevi volle_>p*>p* W2: /ax -2;

		if dd are seviexist { /* become sain viogresoutY_END;n vbd_de-2;

		if dd ar elseio(), ayssectMap
   -2	C_) {
  "s thalve ce->k "Ton vbd_d* !selin_ioe, knevrbd_"Ton vde-2;

	yet_a Ieio(), retu   " ARTor
on vse>p* ct{
  "s_a Onceon vde-2;

		if dd aro requeuino	peerdrain viuino	peerdrio(),>p* aysse_submit_p -2;

	ctMap
   -2	C_) {
  ":\n");
ior offset prdircome s.>p*>p* (e f00;
/**
 ");
		if (daybe_app		ifm_HISTap
  ed.) {
		drbd_info(dct-foveunknevi_volle_,tected, manually solved. "
			     "Sync from %s node\n",
			  D)
		retur "
			     "";
	om %s   protoc" Uevivolle_t%u,_connectio_sync_Tap
  edEE_HASlyafter an cmdlali) {
			/*	 * which in c ? -1 ARTor
			gs)
{
	_om %s , "
			     ",
	.
		 * This is a protoco 30;Paramrb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		  (hg < codearam_95ead;e arbitrary wrihif C_jGOIO);;
			GOIO);exp_max_szg == -100);

		switch (pvis ree 0:
			hg = d= -100);

		switch (pay ste 0:
			hg = d= -100)P];
				de>p_uuid[UI_BITMAP] = 0UL;

				drbd_ueq->w.cb sk
				de>p_ub sk
				dp(peer_deAP] b sk
				dp(peer (hg4)sty wriapv = ct-Brain dete resync start andd_ueq->w.cfifo_ "f(&d e>p_uplandp(peer_deAP] plandp(peer disTzcfifo_lu\n",
0 disTzcu_rea
r_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOct-foveunknevi_volle_, "
			     ",
	.
	)
			hg = hg*2;
		drbd_info(de
	exp_max_sz g =apv <top7 ?ers_markI_BITMA< codearam)
have t:iapv =top8 ?ers_markI_BITMA< codearam)
haaaa+  ne->ldezITMAP];

have t:iapv <to94 ?ers_markI_BITMA< codearam_89)
have t:ir_reqv will5= pers_markI_BITMA< codearam_95&& mydisker, {
	caseexp_max_sz				pr_SYNC_TARGET use B 30;Param	om %s  toou64)1:  protoc" %u,_== 0)
c" <to%u bytesafter an at] = now;
	exp_max_sz	;
[UI_HISTOer nodeuct drbd_pv <top8				prhif C_jGOIOg =rs_markI_BITMA< codearam);
[U;
			GOIO",
					(un*tehif C_jGOIO  "automaticalapv <to94				prhif C_jGOIOg =rs_markI_BITMA< codearam_p logic;
			GOIO",
					(un*tehif C_jGOIO  "rd as it may seem.;
			GOIO",=ount =? "peer" : hif C_jGOIOg =rs_markI_BITMA< codearam_95&& ic;
			GOIO",
					(un*tehif C_jGOIO  "rd as it may seem.;
			GOIO",=ount =?->mdes the "cOIO"vis reem %e  " ay stem %ee->bm : "this");
	}mem
	mypwhiis reem %go0, 2p*  ne->ldezITMAP];
&& mytle_if_app_ Throttlerbd_info(device, "was S ",,ehif C_jGOIO in  */

	list_s;

	rcu_reafomutexsk, in 64)1);
	if (not find aL;

_ 1) {
info>p_uuid[UI_B = (device, "Split-Brain detected buin  */
geG:
		if (ch_pe				prAP] b sk
				dp(kzvice->rs_markI_BITMAb sk
				)
			}
KERNEL)eassa
		 AP] b sk
				)			protsums = true;
		} elomutexsY_START 64)1);
	if (not find aL;

_ 1) {
infopr_SYNC_TARGET use BA(devicand of,", -b sk
				dtoth abd_info(dse if (pi->cmd ==wbGe_o>p_ub sk
				dp(		case 1:
			hb sk
				 ==weAP] b sk
				dp(e>p_ub sk
				sip, AP] b sk
				 (notove_nnectomBrain deterTzuinotove_nnec)odeuct drbd_pv >top8				prdrbd_pv =top8				prGgA		/
			GOIO">  ne->ldezITMAP];
)0));
			GOIO",=oun-data"
			  _TARGET use Biis re-m %Kof,wsecg_NOIO);tive |	"* !selfnve %u,_accept { /_devicp tha%u byteous.\n");
;
			GOIO); ne->ldezITMAP];
&& ");
tle_ifer nodeto(ak;
	, "pTORY_c_Gwrom simtle_if_app_ Throttlerbd_info(device, "was S ",whiis reem %go;
			GOIOlating the
	list_d(ak;
	, "pTORY_c_Gwr/ion === 0)
(pee terESU00) {I_B { /->ldev/ beforrom sioreer !SS_I[UI_te->p that lfo(l/->ldevd as it may seem.
whiis reem %[;
			GOIO-1]",=ount =rotwhiis reem %[;
			GOIO-1]", : 1;
	? "peerr_reqv wil89= pe{_Gwr/ion =egin_i== 0)
(pee terESU00) {I_B { s/->ldev/ beforrom sioreer !SS_I[UI_te->p that lfo(l/->ldevd as it may seem.
whiis reem %[ ne->ldezITMAP];
-1]",=ount =rod as it may seem.
whay stem %[ ne->ldezITMAP];
-1]",=ount =ro
whiis reem %[ ne->ldezITMAP];
-1]", 0t =ro
whay stem %[ ne->ldezITMAP];
-1]", 0c_GwbGe_ock);= -cmp(>p_uuid[UI_B&piis reem %go
whiis reem %)				prGgA		/_DISCARD_REMO"pTO",=oice);
	struct drbdn-data"
			  _TARGET use BDifbd_alt iis re-m %Kser_reqs. me=\";
s"urbd_=\";
s"ous.\n");"tttep_uuid[UI_B&piis reem %go
whiis reem %)odeto(ak;
	I_HISTORY_c_Gw[UI_SIvis ree 0:
		 manua

		swiSECT_tor: %llafceT && mydisk  ,whiis reem %goBiis re-m %olating theIS_ERR(vis ree 0:uid[UI_BIvis ree 0:
			hg = dto(ak;
	I_HISTORY_c_Gw[UI_SbGe_ock);eqv wil89=-0x1 -cmp(>p_uuid[UI_B&pay stem %m.
whay stem %)				prGgA		/_DISCARD_REMO"pTO",=oice);
	struct drbdn-data"
			  _TARGET use BDifbd_alt ay st-m %Kser_reqs. me=\";
s"urbd_=\";
s"ous.\n");"tttep_uuid[UI_B&pay stem %m.
whay stem %)odeto(ak;
	I_HISTORY_c_Gw[UI_SIay ste 0:
		 manua

		swiSECT_tor: %llafceT && mydisk  ,whay stem %m."ay st-m %olating theIS_ERR(ay ste 0:uid[UI_BIay ste 0:
			hg = dto(ak;
	I_HISTORY_c_Gw[UI_SbGe_ock);eqv wo94  ASlP] b sk
				)			proAP] b sk
				 (c planwitif tomBrain deterTzuic planwitif latingAP] b sk
				 (c delaee de\n",omBrain deterTzuic delaee de\n"latingAP] b sk
				 (c fin_e de\n",omBrain deterTzuic fin_e de\n"latingAP] b sk
				 (c max_nnectomBrain deterTzuic max_nnecvice, "fifo_lu\n",
(AP] b sk
				 (c planwitif t* 10p*  LEEP_TIME) / HZating thefifo_lu\n"!p(		case 1codelanws out_frd[UI_BIAP] plandp(fifo_vice->fifo_lu\n)odeto(a
		 AP] planrd[UI_BI
			  _TARGET use Bdevice-Kof,fifo_ "f(&d toth adevice->otsums = true;
		} eloo(ak;
	I_HISTORY_c_Gw[[UI_SIUI_SbGe_ock);vis ree 0:
0))ay ste 0:u			proAP] = 0UL;

			dzvice->rs_markI_BITMAP];
				)
			}
KERNEL)eas	sa
		 AP] = 0UL;

	drbd_pr_SYNC_TARGET use BA(devicand of,", -= 0UL;

	toth abd_info(o(ak;
	I_HISTORY_c_Gw[UI_Gw[MAP] = 0UL;

			->p_uuid[UI_B;Ge__ock);vis ree 0:	drbd_pr1 -cpy(AP] = 0UL;

&piis reem %go
whiis reem %)odeto(AP] = 0UL;

&piis reem %_lendp(1 -lenypwhiis reem %)k = odeto(g

		sw&& chitch (countce, "Split-Brain detvis ree 0:	odeto(countce, "Split-Brain detvis ree 0:dp(vis ree 0:;bd_pr_SYNCp(peer_device);drbd_iis re-m %: \";
s"ous.o
whiis reem %)odetoSB_DISCARDy ste 0:u			pror1 -cpy(AP] = 0UL;

&pay stem %m.
whay stem %)odeto(AP] = 0UL;

&pay stem %_lendp(1 -lenypwhay stem %)k = odeto(g

		sw&& chitch (countce, "Split-Brain detDy ste 0:uodeto(countce, "Split-Brain detay ste 0:
		ay ste 0:;bd_pr_SYNCp(peer_device);drbd_ay st-m %: \";
s"ous.o
whay stem %)odetoSB_DI thaasTART_stillertit-Brain detected bu, AP] = 0UL;

	nfoEATED || peelP] b sk
				)			pr thaasTART_stillert		case 1:
			hb sk
				,SlP] b sk
				)nfoEtsums = true;
		} eD || peelP] planrd[UI_>p_uplandp(		case 1codelanws;	pr thaasTART_stillert		case 1codelanws,SlP] planr;_GbGe_mutexsY_START 64)1);
	if (not find aL;

_ 1) {
inforce, sectoe_ */()eas peelP] = 0UL;

		prk&& c(>p_uuid[UI_B)eask&& c(>p_ub sk
				)nfok&& c(>p_uplanr;_asc ? -1 : 1;, "pTORY_:|| peelP] b sk
				)			prtsums = true;
		} elk&& c(lP] b sk
				)nfo}e_mutexsY_START 64)1);
	if (not find aL;

_ 1) {
infoI_HISTOer nod I_HISTORY_:(hk&& c(lP] planr;_G peelP] b sk
				)			prtsums = true;
		} elk&& c(lP] b sk
				)nfo}e_mutexsY_START 64)1);
	if (not find aL;

_ 1) {
info/ brom sUevi discardness:_act: "biter_devicepeer_bitsc requtio_synreacaseticaay ste 0:
f ==ok.CONShg

		sw&& chitch (6y ste 0:uode/ befor&& cr offvis ree 0:daguid[Uicaay ste 0:
evice, "	u64 _"ToONShg

		sw&& chitch (vis ree 0:	odeg4)1to one o devicerbd_info(device, "was S "NSe "
	, C 0zync requrenc->md.ne-D)easc ? -1 VE_CONFLICT retu ybe IO de\umenve difbd_
}

mor
on and12.5%{
		drbd_inrbd_aretu_if_difbd_
			 no rablyTED)
		return -2;

	*rule_nr
rbailed  30;
s,Ss"waor_t a,Ss"waor_t be, "Ca"waor_t d;_G peeaDARY)
0))b write sy roles at intea wob		drea -ob		: (b -oar;_G peed wo(a>>3m %s d wo(b>>3mto attachreturn -2;

	*C		 no rable difbd_alersior%sn iflus vs. iflusous.oser an attY_START] & ~((u64)1)a,S(device->ldev->md.uui	.
		 * This is a protocors_msrb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		  (hg < rs_mser" : "this");
	}	  (hg o_qlim	->intect-Brain dete resynM_OFFSET == (device->p_uu;
		->qlim				return

	if eterESUeocal_lu\n"d intDS_UNCHANGwbGwba"waor_t p	GOIO);p_uGOIO);p_cGOIO);my_uGOIOGwba"waor_t ctionOIO);cu_jGOIO  "is aldsc", 0c  alrecalup hereu\n" NULL;= (dev

	if ds  hg*2f dsfea
r_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOct-foveunknevi_volle_, "
			     ",
	.
	)
			hg = hg*2;
		drbd_info(de	cu_jGOIO
		 manugeG:ca	drityt		case 1 req_b		c)dGg	ifGOIO
		be64n deterTzuid_lu\n)odep_uGOIO
		be64n deterTzuiu_lu\n)odep_cGOIO
		be64n deterTzuic_lu\n)odfo/ brom s+
		;
device->'sup hereu\n"Uevinev.er_bin =egin_ievice ? ap
  e _"Towheet prn =acceptuSK;
.CONSh doing a %GOIO",
	_	(unsip, */
geG:
		if (ch_pe				pr that disk, in a wwmy_uGOIO... */
		drbd_alert		case 1:
			hb sk
				)	hb sk
GOIO  "r	}

	if (hg > 0 && my	retu_if_difbd_
			 no rablyTn -2;

	*leck wlevelp -2;

	rs_msdes have p	GOIO); manugeG:max_ca	drityt		case 1:
		)	d_syretu_if_difbd_
			 no rablyTn -2;

	*usk wo one o) {IOIO"" flag set,p_uGOIO);my_uGOIO) 1;
		}
ybe Iequtiodevi elsei "
			 C_MAsevin");
wibr_== 0)
c"
or_ Cearam_==eer) {
	choos;
deviESUim	ifUI_HISTORY_START] & ~(("pTO",=oice);
	struct drbdn =ro
_uGOIO...ESULe, _zero(my_uGOIO);p_uGOIO) 1;
		}
Ns, o shr, &cax -2;

	ad_lousable d

	cdur { /* becom.r an aBu,  (dew/_deSUe shr, & { /ake.\nas
	c WY_STARpeer_dectionOIO
		 manulP] bal_lu\nmay seem.;	case 1:
		);p_uGOIO);0)eassa
		ctionOIO
<;cu_jGOIO		drbde, "self", device->ldev->md.OUTD)1))) {
	de, "self", device-"pTO"<s");
		)bGwbu-  Rd Wweight(dn -2;

	*Tevice->'sup hereu\n"tiodoo sevic! (evice< ifluSs"waors)ous.\n");
(device->ldev->md.uuctionOIO);(device->ldev->md.uucu_jGOIO)odetog4)1to one o devicerbd_info(device, "was S "NSe "
	, C 0zync requrenc->md.ne-D)eas>otsums = true;
		} eloI_HISTOer nodetbGe_ock);my_uGOIO.!=;p_uGOIO)-  Rd eq->w.cb sk
				de>p_ub sk
				_deAP] b sk
				dp(peer (tingAP] b sk
				dp(kzvice->rs_markI_BITMAb sk
				)
			}
KERNEL)easssa
		 AP] b sk
				)			pror_SYNC_TARGET use BA(devicand of,", -b sk
				dtoth abd_info(dotsums = true;
		} eloose if (pi->cmd ==w[UI_Gw[mutexsk, in 64)1);
	if (not find aL;

_ 1) {
info_o>p_ub sk
				dp(		case 1:
			hb sk
				 ==wweAP] b sk
				dp(e>p_ub sk
				siingAP] b sk
				 (b sk
GOIO =;p_uGOIOice, "pthaasTART_stillert		case 1:
			hb sk
				,SlP] b sk
				)nfoEomutexsY_START 64)1);
	if (not find aL;

_ 1) {
infoprrce, sectoe_ */()eas	sk&& c(>p_ub sk
				)nf Rd Wweigp(peer_device)P !sess l u_lu\n tha%luSs"waorsous.\n");"(device->ldev-)my_uGOIO) 1w[UI_Gwtsums = true;
		} eD || doing a bd_imax_biofGOIO
		beain deterTzuimax_biofGOIOuode/ bLee->p_app_ Th		 no r_oneuedearameters() device"no locaterESUeocal_lu\n().er_sbIoreer !.\nbctor->stateQUEUEc.",
_e, "dry_uuid_d(&doneue iser_sb_app_ Th		 no r_oneuedearameters()LOCKE00;
be sur
on atbd_infer_sb_app_caterESUeocal_lu\n() nosREQ_e, "drys((u64)1stateoneueice = p dsf
		be16n deterTzuidds  hg*2 in  */
geG:
		if (ch_pe				pr_app_ Th		 no r_oneuedearameters(ay seem.;	case 1:
		);o&& ic;d
		 manucaterESUeocal_lu\n(ay seem.;dsf,(peer)nfoEtsums = true;
		} eISTORYdd_warS_ERRORuid[UI_HISTOer nodet manumvice, true;
		} eD "peer" : fun_writelse n_write,,ievice ? acceptuSKvice->'su* get th*reu\n.n_writelydis NOT acceptuSKvice->s baTOnnecp hereu\n test_bitnr =ice->p) {
	lde\nron andESUe  (d  (ev-...n_writest_bAte Iequstillain vice->kknevs
mor
obd_"Tomy	}

	ifMAsetturn C]&1))
bd_"TowK;
	nyw_diske resy up sain andERIMAR.n_writSo/akeIequc_lu\n"tioite, n andIequdonOIO);deviEice) ike((u64)1)1;
			utiode;
	*my*udonOIO
f ==sevick wldiskUI_HICKE0ISTO & ~((u6u64)1)Heckver,aybehywayssif (zerot get thoeu\n test_btakedIequ(usk -c1];
d C_)abaTOnnecp hereu\n-brarr(d ~((u6u64)1)Unite, of,cd(&sdeviuuid[UI_BIce->pt_n_wrdIemIMAR.n_writIutY_END;eer !.\nARTor
on equcdiscardly_n_wri =	ba"waor_t ctionOIO =;p_cGOIO
?:;p_uGOIO
?:;p_GOIO  "r_app_ Th		 no r_oneuedearameters(ay seem.peer_do)eassa
		ctionOIO
,=oun-data"/ bIRTor
	if (deuid[UI_BIknevrI_B_TRApeer_deautomaticalctionOIO
,=ocu_jGOIO)-data"/ bI_B_TRAnd upoeer_deautomaticalcu_jGOIO	!=turn rponOIO
,=oun-data"ttachreturn -2;

	*IRTor
d_n_write, f (deu-2;

	rs_me->p_udevice!=tme:ifluSs"waors)!ous.\n");
(device->ldev->md.uuctionOIO);(device->ldev->md.uucu_jGOIO)odetautomaticalctionOIO
<;cu_jGOIO		d(enum drbd_after_sb_p peer)
{
	/u-  Rd Wweight(dn -2;

	*Tevice->'sup-2;

	rs_metiodoo sevic! (evice< ifluSs"waors);up-mo	   vi else!ous.\n");
(device->ldev->md.uuctionOIO);(device->ldev->md.uucu_jGOIO)odetog4)1to one o devicerbd_info(device, "was S "NSe "
	, C 0zync requrenc->md.ne-D)eas>oI_HISTOer nodetbPRIMARY)
		/ bIp) liev;
device->,aybs hav*t*teIupon'BIce->pt_ get thoeu\ndERIMARs hav*t*te.\na resro requeuu\n-brarr(ds hav*t*teIupoIce->pt_ get thoeu\n,lse S->ldev->,s hav*t* e  " hI_BITMAP] _devin_wrs hav*t*teIupoIce->pt_ get thoeu\n,lse Pelf_pri &&av*t* e  " hI_BITMAP] _devin_wri &&av*t* econnectiolde\nron andEy_ get thoeu\n &&av*/ Rd Wweig);
	my_ca	drityt		case, ctionOIO	nfoEATED || peegeG:
		if (ch_pe				prned.
		 * This is knevi_lu\n"!p(	manugeG:ca	drityt		case 1is is baTOnne_b		c)u-  Rd W		 * This is knevi_lu\n"p(	manugeG:ca	drityt		case 1is is baTOnne_b		c)eas>oldsc", 1 1w[UI_Gwtsums = true;
		} eD ||STORY_START] & ~(("pTO">oice);
	struct drbdn-dataSTORbe64n deterTzuic_lu\n)"!p
	de, "smanugeG:ca	drityt		case 1 req_b		c) %s ldsc)e{_Gwr/ion =ce->pdifbd_alt rs_ms,
			bablyice-> &&av*tevicp thaknevrEy_", -eu\n. { /* be Wweig);ndors_msr, "Lost last 0m.;dsf	nfoEATE_info(devi  "_bctor,turnREST:
_PENDrennect: Would become(hg)ew conndd_warS_GREW		d(enum drbd_afte"pTO",=oic;
		)bGwbu				prGgA		/_DISCARD_REMOpddev->md.(device, "Splits have "self", device->ldev->md.evice)) {
			drbd_eISTORYdsf & DDSF_NO;
	(!di)
I_BI
			  p(peer_device)R) __muof,", -+
		er did[Unote->sad_lo--r == e-bctonbd_info(doevice->p>oI_tove_rn C_j_deSUe_growtrue;
		} elobPRIMAe->p>);
	turn
	(!di_AFTERcu_dnect: Would becomnfoEATED ||c ? -1 : 1vic* This is a protocos resrb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		  (hg < s reser" : "this");
	} 1;
*< s re  "is ai,  1) {
d s rese, : 1;
_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOct-foveunknevi_volle_, "
			     ",
	.
	)
			hg = hg*2;
		drbd_info(dedep_u re			device-_array(UI_EX{
	D>ldeT:
,=rs_mark*< s re)
			}
NOIO in  */
		 s re)			pr_SYNC_TARGET use Bdevice-Kof,p_u re	toth abd_info(I_HISTOfaIMA} eD || hg & 0xff;
ce->p_ug >>  UI_EX{
	D>ldeT:
d_aler
			peer = de		be64n deterTzuiuer = d)nf Rk&& c( doing a %s re);
, device->ldev- =;p_u re  
er_role == R_Pevice-"pTO"<s");
		)bGwb %s d_DISCARD_REMOpddev_warn0zyKLESS			drbve "self", device->ldev<md.(device, "Splits _sb(enum drbd_after_sb_p peer)
{
	/* A
ew conn state 
d s reASK (-1) on fa) {
IZE], device->p_uuid[UI_FLAGS])				pr_SYNC_TARGET use Bieviit(RE "
			 nd up

	cad_loc}

	*rule_n=%016llXafter an at(device->ldev->md.uun state 
d s reinfo(g4)1to one o devicerbd_info(device, "was S "NSe "
	, C 0zync requrenc->md.ne-D)eas>I_HISTOer nodeuct drbdgeG:
		if (ch_pe				prnlt rkip_ the "cice,  = Rd W		 * Thd_afte"pTO",=oic;
		)bGwblits hacountce, "Split-Brain dete resync start and sto90lits ha		case 1is is 	if (self == peer)
	,=oic enJUST_CRE)1))) {
	d	( %s resync.",
		id[8);

	drbd_kip_ the "cice, 	    (PW-1000) {
		drbd_alAccept->l", -h}

	*rule_n,
		eearTRAnd u_kips the "cm %s );

	spin_lockresync finished event, cic enbctor,ryself:\e->p>o"bctor,ryself:_uuid_ protocos resevice);
				deviceTESldev->md.u	spinways_asbp;

	mydisk = devi= peer), IZE], device->p_uuu	spinways_asbp;

	mydisk = deviconf *nc0u	spinways_a);
	device_NS2(ay seem.;

	ifD_UP_TOn co
,=pddeifD_UP_TOn co
 (-1) o	md.VERBOSE,(peer)nfoEt manumvice, true;
		} e		 1) {
d s rese, device->wtsums = true;
		} eDutomaticalself", device->ldev<md.(device, "Splits 	sb(enum drbd_after_sb_p peer)
{
	/tmap, fulelse t_n_write, self_priin vice->krom scre00) {al", -h}

	*rule_ns 	sb( hg m(peer_de 1) {
d s rese, ays_a);
	
d s remay seem.
ZE], device->p_uuu	sp?->mdesBevice"n =(dev( hg n vde here
		 LOCKEer_tim wai	} etilsevieY &&: "bi
ew cd.u drbdclom !selide 1000); NULL;
tiof thvice. Te;
	equtmUI_Fas aif
ew c.\nas
		peer = p_uuid_p -e",
nnectuid_d(&de he.te fevice ? s cr of
ew c", -b sk 1000). { /* bmutexsk, inenum drbd_aft_mutexinfomutexsY_STARTenum drbd_aft_mutexinfoSTORY_START] & ~(("pTO">=oic;
		)bGwblit"self", device->ldev<md.(device, "Sp)_de 1) {
d s rese|, ays_a);
	
d s remay seem.
ZE], device->p_uuu	sfoSTOR 1) {
d s resto attach	pentos resr		drbd_al protocr  1) {
d_bit(COto"r;_asc ? -1 : 1FLICT->p* ct-(devidevice)*teCt-(devsuSKvice->'suvi, -T, deviclom !se1000); ? d(&dstill-T, vi, >p* @ps:		The 1000); ==s {
	byuSKvice->INCONSISTENT unand ays_a)000); t-(devideviceunand ays_a)000);pse, "Cunand ays_a)000);mp* toISTENT && peer_disk > Dc_tab[de		ap, [ice);
	struct drbd] =oice);
	struct drbd,p, [ic;
		)bGwb]his");
		)bGwb,
p, [ic%d anren on suc]his")%d anren on suT,p, [ic%d anren on suT]his")%d anren on sud,p, [ic0zync requren]his")TEAR_DOWN,  alCcu_TWORK_FAILURE,(er_de[icVERIFYuc]hhhhhhhis")VERIFYuT,p, [ic_STA]hhhis")_STA,p,}eafoms. 0xfps. eafoms."pTO",Dc_tab[ps."pTOselfms.to, fflps.r_sbelfms.r_sb_plps.ened.
fms.tddev_lps.u_read_ms.u_re_plps.edread_ms.count)s" : (ps.rn nt)s" |lps.usk t)s"r;_asc ? -1 mp* vic* This is a protocoo oideviceb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		  (hg < o oidevicer" : "this");
	} nand ays_a)000);madeifval;	}

	if (hg d_aft_rv that;
_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOer node)
			hg = hg*2;
		drbd_info(dedemade. 0xfbeain deterTzuimas
	rcuval. 0xfbeain deterTzuivalu	sfoSTORuuid[UI_HISTORY_START] & ~((urbd_info(device, "was SyncTarge* A
ew comutexsissk, iedTenum drbd_aft_mutexi				pr_SYNC);ndorr o plyr, "Lost last SSSTARce->p_u_ST_CHG)eas>I_HISTOy *] = dmas
CALcrt(devidevicemas
	rcuvalCALcrt(devidevicevalu	sfo"thisrateliNULL;idevicest last md.VERBOSE,(madeifval);
, SYNC);ndorr o plyr, "Lost last rc)dGg	 manumvice, true;
		} ||c ? -1 : 1vic* This is a protocoo oig4)1tdeviceb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not < o oidevicer" : "this");
	} nand ays_a)000);madeifval;	}

	if (hg d_aft_rv that;
made. 0xfbeain deterTzuimas
	rcuval. 0xfbeain deterTzuivalu	sfoSTORuuid[UI_HISTORY_START] & ~((uce, "was SyncTarge* A
ew comutexsissk, iedTuce, "was Syncd_aft_mutexi				prg4)1td;ndorr o plyratelimit_staSSSTARce->p_u_ST_CHG)eas>I_HISTOy *] = dmas
CALcrt(devidevicemas
	rcuvalCALcrt(devidevicevalu	sfo"thisg4)1to one o deviceatelimit_stamadeifvalt md.VERBOSE |lmd. ~((u_ONLY |lmd.IGN.OUTD_FAIL	odeg4)1td;ndorr o plyratelimit_starc)dGg	c ? -1 : 1vic* This is a protocodeviceb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		  (hg < devicer" : "this");
	} nand ays_a)000);o,,iesbd_asb_)000);o disk states.\n",
		  re0l_0)
			drb;o disk chg d_aft_ hg*2fcs  hg*2  "is athat;
_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOct-foveunknevi_volle_, "
			     ",
	.
	)
			hg = hg*2;
		drbd_info(dedepasb_)000). 0xfbeain deterTzui)000))dGg	c 0l_0)
			drbg = hg*2;

	rcu_read_lock hg*2;

	rcu_rec = rcu_dereferenc			pr  0l_0)
			drbg = device->ldev-sync.",
		id[4 ?ed.(device, "Spl:B_DISCARD_LOC  "r_app_0) {
		drbd_al  0l f (deu sk 1000)g =;
side states.\n",
r(  0l_0)
			drb)u	sp?->mrr_conflict;
	tentativenot find ao oi_asb_retde?ry:
	ose, ns_if_app_ Tadidevicest lasinforice);
			break;
		case 1not find ao oi_asb_re>mdesIbeSS_Ifset prptate p devicrdri(drb_ protocr th Tad,kUI_Hout)er_bitl Tady= dcid are sclos;
deviimit(&drbd_aguid[er_bin =ydis I_BIl  -eT_CREhvi"bitnvert_afterlockos."pTO"<is")TEAR_DOWN/
[UI_HISTOernc rISTETre>mdesIbe Iequtiodevi"");
 p ce, "Oct-forma     "us: "bitdevice->k;_wrs t_btransit drbdtuid_d.(device, "Sple sD_UP_TOn co
. Fhg ce, y (0solves t_b);
)sk) __mu_statary w Pauskd 30;
ifMAsybe IO UI_ { /_frptusk-/s t_bunptusk- __mueY &&s_BITM) {
	"rom sytzbG",tdevice->k;_wrnr =s t_btransit drdtuid_d.evice, "Sple sD_UP_TOn co
; ==	BM_.er_bterlockkos.pddev_warn(device, "Spl|| os.pddev_warnice)) {
			d A
ew co  0l_0)
			drbg =sD_UP_TOn co
; A
ew coos."pTO">oic;
		)bGwblit"os.	drbg =sD_UP_TOn co
tmap, fuleke.\nas
	(bule_nr ) bits in bitmap!\ce->ktioegin_i w  __mn_writ		eeara     "ARTor
olve upto) {
-nte, no arbd_a hg0) {  "Atturn Co(),  NULL;
do			     drbd_ onceon vdce->kreacass_acttocturn C __mcorre
		 * n_writItnr =ice->p NULL;=  __mer-ptuskdhere a, heckver,asoe.\n_writcanI_BIARTor
on equcdiscardly_fUI_HISTOR hg*2;

	rc"pTO">oic;
		)bGwblitr an at hg*2;

	rc"pTO"<te on success,uid[UI_0l_0)
			drbg =rn(device, "Sp 1;
		}
ybe hg*2;

	rp NULL;, no c WY_STARLAGS_submit_peer_ test_bitnexplicitbiter_if->p uiode;
	itof thvicec_handshtest_bMaybeOCKEer_tim f thvi	itoup,odoo?fUI_HItomaticalos."pTO">=te on success, its hav hg*2;

	rc"pTO",=oic;
		)bGwbu			prGgA		/support at leastzbGwbGgA		/"<is		case 1ns  oth a)
I_BI_app_ Ttove_f thvicetrue;
		} eloI_HISTO"To resolve dessxplicitfvis reof thvicecer_if-cait_stateopSs"waornreacase_afterlockos."pTO"=is")VERIFYuTlit"os.	drbg =sD_UP_TOn co
; A
ew co hg*2;

	rc"pTO",=oic;
		)bGwblit"  0l_0)
			drbg =sD_UP_TOn co
rd[UI_>v_out_ofice, "	penttrue;
		} el_app_ Ttove_f thvicetrue;
		} elI_HISTOy *] = d/_d* !sesaysdIequddrbgequt	     drbd_,_conle"n =(h, &ci
	equupto) {
eer_bitsse_sis_BIppens_conle"n vdce->kegin_i(h, &son =ce->pam %s nu drbdod[er_biap!\n =(h, &c.\nas
	tl Tady= [UI_ad_lo_submndshterite fARTor
on equno arbd_a hg0) { =pddehteritT equer_tim I_BIceppen,sybe IO ce->ktioaa prolt iissand of,_app_afterlockos.pddev_warnUP_TOn co
; A"  0l_0)
			drbg =sD_(device, "Splits _sb(os."pTO"=is");
		)bGwblit" hg*2;

	rc"pTO">oicon success,uid[I_0l_0)
			drbg =rnUP_TOn co
	sfoSTORns("pTO",=oice);
	struct drbdn =rns("pTO",s");
		)bGwbGwd_lock hg*2;

	rc"pTO",=oicAHEADn =rns("pTO",s")BEHINDre>mdesTODO:teritlock peer = p_uun_write, p_uuce->ks reA!= effecttocks re)er_bit* e bI_FLAG_bmio_seened.
fu6u6 esIbe Iequerdriuid[UI_BIce->pgoo {
			drf ==tl Tady=c WY_STARtmap!
rn C_subce->k;_ {all00); G_bmio_sbiterw,odryTRAnd u"negr_i00)"_ad_lom
eer_biANDlelse h}

	*rbitPelf_pri possiblyituizen,sad_loSS_Ifspdcif-cer_bi"effecttoc"ks re,e Iequer_tim Is, o benreacase,"  0llri bultusker_bin = elseiaysse_subs res,tdev reque get thoe

	rc
fu6u6 esIne IequerolarioLOCKEtl Tady= < 0)->stateimit(&drbd_harder_binev rwe  protoc" _subsnsui_CREAbs res	(1;rotocos resr)c
fu6u6 esSr_tim weelfnv no cNULL;
deis,tde;
	eq:UI_BID_SET_ALLimit(&drbd_iser_*a protocos resr)Etl Tady,tdev rweel_tim Isice ? ad {albranmiof (drbn C_sate bI_Fl %d\nmG_bmio_f "snsui_CREAbs res"ro requece->ktoreer rbn C_sequerdriequc}

	*rbitD_write, Pelf_pr.er_btefoSTORY_START]>ldev- it" hg*2;

	rc>ldev->md.u_dereferenlits _sb(geG:
		i_if_devicest last rcu_dereferenc				prnlt crc  alh		 no rsk) __mui = pe	}
yben ==T_CREhvi) {al", -hmit(&drbd_rd_disrhhiskos."pTO"<oic;
		)bGwbu; pe	}
yben =hadsevieT_CREhvi) {hmit(&drbdtest_bp_uu[UI_ p devierdrsl", bitmG_bmiss_auddrbgrd_disrh|=ckos."pTO"=is");
		)bGwblitr an at at( hg*2;

	rcu_rec = rcu_dereferene(hg)e	os.	drbg =sD_u_dereferenc	; pe	}
yben =ha->p), so) {
	t	     drbd_,_asse_subce->kRT] ) {
test_b/* by ithat lUpToD00);ad_lo--pri-self:TED &grd_disrh|=c
		return 2;SIDER;
	(!dinect: Would becomnfoE	}
yben =hads) {
	plC_WFc WY_STARtmasse_subad_nrwo one o) {t_for_ C_statpam %s nby""ngh", m00)"_orn"ngh", m00)-r-mo	 "grd_disrh|=ckos."pTO"=is");
		)bGwblitr ad	( hg*2;

	rc"pTO">is")%d anren on sudlitr ad	t hg*2;

	rc"pTO"< pcount, (hg < ))= e_ock);clist_dns("pTO",s(device, "was SyncSr, "Lost last  hg*2;

	rcr_sb,"  0l_0)
			drb)= e_otsums = true;
		} eISTORns("pTO",=oic_STAu			proAs("pTO",s");
		)bGwbGwprGgA		/_DISCARD_REMOu_rec = rcu_dereferenc			prel_app_/* by_devicest last NS(;

	ifD_FAILwbu	} elobPRIMA lock hg*2;

	rcu_rec = rcu_dereferenc			pr"
			  _TARGET use BDirecmG_bmioprocte, o requece->kerdrio ==tbI_Feder_device-> hg*2;

	rcu_rec arn0zyKLESSvice->I_0l_0)
			drbg =rn0zyKLESSvice-bPRIMARY)
		_info(devi  "_bctor,turn 2;

	ull sync rbd_info(device, "was SyncTargete->p>oI_HISTOer nodetrod as it may seem.os("pTO",=oice);
	struct drbdnodetrog4)1to one o devicerbd_info(device, "was S "NSe "
	, C 0zync requrenc->md.ne-D)eas>ooI_HISTOer nodetr}o resolve rr_conflict;
	tentativenot find ao oi_asb_rerlockos.i"!p(	manu Tadidevicest lasi.ite->ak;
	, dryodegctor,turn 2;SIDER;
	(!dinect: Would becomnfons.to, fflphg*2;

	rcr_sbnfons.tddev_lre0l_0)
			drb;o ns.count)s" : (phg*2;

	rcrn nt)s" |lphg*2;

	rcusk t)s"r;_rlockkns."pTO"=is");
		)bGwbl|| ns("pTO",=oice);T_CR	   )  ASls.	drbg =sD_u_dereferenc =rns(	drbg = device-ction_aft_tmpcu_read_cs  hg*2",s"d.VERBOSE +skos."pTO"<oic;
		)bGwb  ASls."pTO">=oic;
		)bGwbl? 0 or
d.ne-D)easSTORns(pddev_warnice)) {
		lit"sdevicuspdndcetrue;
		  ASls."pTO"=is");
		)bGwblit"os."pTO"<oic;
		)bGwb  As _sb(
		returnNEWice-_le_n,
ct: Would becomtmap, fulDoice,  (dew tl_ TtstatHISTENbu	 hg aa pbooo) {ce->Ite f00;
it(RE (dew tseqs 	sb( hg temUI_a_ievt	u64 _"Tages!ri =	baice);
			break;
		case 1not find ao oi_asb_reor_SYNC_TARGET use BAbI_FTRAnC"
			 C_cevice, _saw IO;ad_lo0;
it(REC    drbd_ ce->r_devicetl_gctorerbd_info(device, "was S_reor_SYNCsbp;
ctioc}

	*rtrue;
		} eIgctor,turnNEWice-_le_n,
ct: Would becomnfo(g4)1to one o devicerbd_info(device, "was S "NS2e "
	, C viceOCOL_ERROR, cuspnc0u->md.ne-D)eas>I_HISTOer nodeuco"thisways_a);
	deviceGET use esbdcs  hg*2,(peer)nfons_if_app_ Tadidevicest lasinforice);
			break;
		case 1not find ao oi_asb_re>mSTOR"th<aSSSSUCCESS		{fo(g4)1to one o devicerbd_info(device, "was S "NSe "
	, C 0zync requrenc->md.ne-D)eas>I_HISTOer nodeuct drbdos."pTO">oice);
	struct drbdn-dataSTORns."pTO">oic;
		)bGwblit" hg*2;

	rc"pTO"< pco;
		)bGwblitr an at hg*2;

	rc	drbg!= rcu_dereferene)e{_Gwr/ion =lfnv (device,ce->kRT] ce, yet= dcid are smndsh { /* be 	}
Nowadaysd_device->saev r/* bTRAnauerdriene s peer = r_sb_  "
 have ser_reqolve 	drbge sUpToD00);ad_lode;
	** be Wweig);ndos resr		devbGgA		/; be Wweig);ndoc}

	*r devicerbd_info(demnfoEATED ||gctor,turne, "dry-run con,
ct: Would becomnfg	 manumvice, true;
		}  al 1) {
 c WY_STARLind-caior,ll0ors_m_s		 C_h { /* g	c ? -1 : 1vic* This is a protocode, "s remb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		  (hg < rsldev- r" : "this");
	;
_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOer node)
			hg = hg*2;
		drbd_info(dedewai	_eY &&	/_DISCARm_HI_wai	,s 	sb(enum drbd_afte"pTO",=oice);on sule_ne(hg)ew cenum drbd_afte"pTO",=oicBEHINDe(hg)ew cenum drbd_afte"pTO"<s");
		)bGwb %sg)ew cenum drbd_afte>ldev<md.u_dereferencre>mdesd as it may seem.(enum drbd_afte"pTO",=oice);on sule_ne	} /* g	desHe	;
deviways_asbp;
 fun	    s((u64ytzbG,e get thoer_timfow cLe, _ benrotaTARLine sdeviseqioryafterlockgeG:
		i_if_devicest last rcu_dereferenc				prways_asbp;

	mydisk = devi= peer), be64n deterTzuiuer u	} elways_asbp;

	mydisk = deviconf *nc0UL)= e_ottach	pentos resr		drbd_al 1) {
d_ %s ns re"_reor_SYNCtstat_ Ttoveest last m on suTARGET)= e_otsums = true;
		} ebPRIMAe->_SYNC_TARGET use BIRTorreqobitsle_ne	drbd_!r_deviasc ? -1 : 1FLICT->p*  protocoresync plC_W>p*>p* R ? -1 :saev r [UI, 1saev ranset prlf:  -2	C_equevicepe p_uuiuevgattockSYNC_>p* ctdriup s	tothure. {
		drbd_info(
 protocoresync plC_W(an not satisfy peer's read request, , arbitrary wrieu\n testttttdevice->ldev->*ptate))
		bm_xbd_
	tx_rae, "Cunbitrary wri;
			GOIO",
 may bdeviT_BUFFEREST:
 -r ad	tsatishif C_jGOIOerbd_info(device, "was S_reounbitrary wrinum_	u6dp p_ESULt(rs_m_tm.;
			GOIO"/=rs_mark*<).\n");"ttttttc->bm_	u6dp -tc->	u6d_off);
)reounbitrary wrilfnv =inum_	u6dp *=rs_mark*<)  "is au_reafolocklfnv ! =rs_m				pr_SYNC_TARd request, , "%s:lfnv (%u) ! =rs_m (%u)side __fun	__,ilfnv,=rs_m)eas>I_HISTOer nodeucolocklfnv write sy rolesOy *]tle_if_app_ Throttlerbd_info(device, "was S ",,elfnv in  */

	list_s;

	rcu_reafo/supportmerge_lelerbd_info(deviuest, , c->	u6d_off);
,inum_	u6dp ",eviasc->	u6d_off);
 +=inum_	u6dp;asc->res_off);
 = c->	u6d_off);
 *=conS_PERELONGin  */
c->res_off);
 >tc->bm_olveist_c->res_off);
 = c->bm_olveviasc ? -1 1 1vic* This disk stateresync ctdridcbpugeG:ctdrkI_BITMA< cdisnote->por>*pe, "Cc ? -1 (disk stateresync ctdr)Tzuienctdreqo& 0x0f	nfvic* This is adcbpugeG:tstatHI_BITMA< cdisnote->por>*pe, "Cc ? -1 (zuienctdreqo& 0x80) ! =: 1vic* This is adcbpugeG:pateressHI_BITMA< cdisnote->por>*pe, "Cc ? -1 (zuienctdreqo>> 4)o& 0x7 1FLICT->p*  prvportrlcoress>p*>p* R ? -1 :saev r [UI, 1saev ranset prlf:  -2	C_equevicepe p_uuiuevgattockSYNC_>p* ctdriup s	tothure. {
		drbd_info(
 prvportrlcoress(an not satisfy peer's read request, ,
		I_BITMA< cdisnote->por>*p testte))
		bm_xbd_
	tx_ra testunbitrary wrilene, "Can not resst Tam bp;as 1;
lookwitif ;as 1;
rl;as 1;
tmd;e arbitrarydev->s = c->bes_off);
;e arbitrarydev->O  "is atoggsb_pldcbpugeG:tstatH<)  "is aha->  "is aolveviasresst Tam_ the(&bs.o
whardrailen,sdcbpugeG:pateressHp))= e_ress0xfbesst Tam_g;
	turs(&bs.o&lookwitif , 64 in  */
ress0<ite sy rolesOer nod | hg &ha->pxfbess;=ha->p> 0c s +=irl,atoggsb_pl!toggsb				prress0xfvliqueardr	turs(&rl,alookwitif 	} eISTORress0<rite syy rolesOer nod |_info(oggsb				pr	Og =r +irl -1GwprGgA		e">=oc->bm_olvei			pr"
			  _TARd request, , "resync pri-fdew (e:ifu) conle"ueardreqoor>RLhrom %s side 		} eloose if (pi nodetr}o rlways_abm_s;
	turs(rbd_info(deviuest, , se 		} elbGe_ock);ha->p< olvei			pr"			  _TARd request, , "resync ueardreqoSYNC_: h:%d b:%d la:0x%08llx l:%u/%uous.\n");ha->, olve,alookwitif .\n");(unbitrary wr)(bs."ur.b -o
whardr).\n");(unbitrary wr)bs.buf_len	} eloI_HISTOer nodetbGoE	}
yben =    um) {all 64 olve,aasTART 0c >> 64 equ"undefirar";fUI_HISTOR ike((
ress0<i64 e syylookwitif  >>xfbess;
doevice->plookwitif  , 0c_Gwha->p-=aolvevias_ress0xfbesst Tam_g;
	turs(&bs.o&tmd, 64*teha->	} eISTORress0<ite syy rolesOer nod>plookwitif  |=c
mp <<aha->  "wha->p+xfbess;
d}
asc->res_off);
 = s;
dbm_xbd_
	tx	turn de	u6d_off);
(ceviasc ? -1 (s ! =c->bm_olvei 1FLICT->p* ueardr	turync c>p*>p* R ? -1 :saev r [UI, 1saev ranset prlf:  -2	C_equevicepe p_uuiuevgattockSYNC_>p* ctdriup s	tothure. {
		drbd_info(
ueardr	turync c(an not satisfy peer's read request, ,
		I_BITMA< cdisnote->por>*p teste))
		bm_xbd_
	tx_ra tesunbitrary wrilene, "CgA		/cbpugeG:ctdrkp)_p peLE_VLviclveist_c ? -1  prvportrlcoress(, "Lost last  , cailen*ters_mark*<)cre>mdesset prvaria&&s_BIds) {
	iiscamenvc" Uevievaluait_ster_biap!\ce->p) {
	 < 0)->stsc requ[UI_t -1ic;
		}that l"best"er_bidur { /all d(&d
		rsice = p 		  _TARd request, , " protocoresync c:iunknevi enctdreqo%uous. zuienctdreq	odeg4)1to one o devicerbd_info(device, "was S "NSe "
	, C viceOCOL_ERRORc->md.ne-D)easc ? -1 VE_CONFLIrbd_aINFO_bm_xbd_
devisTED)
		return -2;

	*rule_nr
rrbailed  30;
dircome state))
		bm_xbd_
	tx_rae, "C/ionK;
	n_tim itbtakede sdransf prlf "plC_Wtext"grd_darbitrary wrihif C_jGOIO_if_app_hif C_jGOIOe elset{
				drbd_irue;
		vice, "was S_reounbitrary wri;
			GOIO",
 may bdeviT_BUFFEREST:
 -ehif C_jGOIO  "unbitrary wriplC_WF= Rdhif C_jGOIO_* (DIV_ROUNrnUP(c->bm_	u6dpgo;
			GOIOlk = lk st_c->rm_	u6dp *=rs_markdevice->ldev-)  "unbitrary wri at l = c->bytess>  + c->bytess1]  "unbitrary wrirre>mdes at l cevice, t lzero.beforrom sioreer :afterlock at l =rite sy roles a>mdespon'BIreUI_F
ybe_sync_isnote->afterlock at l >xfplC_We sy roles a>mdes at l <fplC_W.E0ISTO Uevipri-fdew,kegin_iftere_ifk at l > UINAP];
/1000		dre at l / (zlC_W/1000	)
have ttttttttttttttttt: (1000 es at l /fplC_Were>mSTOR" > 1000	 sy e, d000viasce, d000 -ed.
f_app_0) {
		drbd_al%s resync devis [Bytes(om %s s)]:iplC_WF%u(%u),>RLhr%u(%u),>"s _sb( " at l %u;nc_isnotei		n iu.%u%%ous.\n")dircome st\n")c->bytess1], c->om %s ss1],\n")c->bytess0], c->om %s ss0],\n") at l, r/10, r % d0i 1FLICT Sinceo.\nas
		pocte,TRAndhe resfieim fuid_leck waddnote-quno higt p,
b( lf uid[UI_BImat !seybe IO crocte, i sior32 resE0Iu &soevi64 olv
b( 0Iu &sotscdev->asci
	equlittle"endia&. (Undelse  " At>ascbyte st Tam,
b( begin
{
	_ad_lo_subleckm sbyte...)leke.\nn_tim use reg"endia&
w c.\nl_tim Isice ? crocte, i sfuid_devisegt iskeddnote e sdevileckm ,
b( lnoevo rsthat lagnosd_ine sdevi32 vs 64 olve	eqsue. 
 co  oless :s s	tothure, 1syben =succte,fu"bit protoc" i
.CONS* This is a protocoresyncmb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		  (hg bm_xbd_
	tx_c  "is au_reafo_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOer node)
			hg = hg*2;
		drbd_info(dedeays_abm_STARTenum d, " protoc resyncs. 				deviceSEldev->md.u	spCT younas
	id[Uosice ? s _uuiddit dral d(t-of- __murr(drma    teritlocyounact: "bit);
 olve	dur { / Iequsher !e = pc", (	  (hg bm_xbd_
	txi			pr.bm_olve_if_app_bm_olveirue;
		,	pr.bm_	u6dp p__app_bm_	u6dpirue;
		,	p}od | hg(;;n-dataSTOR {
			/ =riPiconf *e syytle_if protocoresync plC_W(, "Lost last   = now;
	"this");((uc);
doevic STOR {
			/ =riPiCOMPRESSiceconf *ee{_Gwr/ioMAYBE: satheyE0ISTO tK;
	nywspeak cro ? sto90i &&av*tasse_subM_OFFSE	equenCREAd!ri =	b	I_BITMA< cdisnote->por>*p : "this");
	;
taSTOR {
	GOIO">  may bdeviT_BUFFEREST:
 -e_app_hif C_jGOIOece, "was S_c			pr"
			  _TARGET use BReUI_FCBesync om %s  toou6de\nbd_info(doele_ifer nodeto(ak;
	d(todetr}o rlSTOR {
	GOIO"<=ers_mark*<)c			pr"
			  _TARGET use BReUI_FCBesync om %s  toousevicOR :%u)side  {
	GOIOinfo(doele_ifer nodeto(ak;
	d(todetr}o rltle_if_app_ Throttlerbd_info(device, "was S ",,e {
	GOIOinfo(d the
	list_dtttttttak;
	d(todetrtle_if_eardr	turync c(, "Lost last  , &c,e {
	GOIOinfo(bPRIMARY)
		ttachreturn -2;

	* protocoresync: 		/ neiet prReUI_FBesMnc noprReUI_FCBesMnc (equ0x%x)de  {
			/)odetrtle_ifer nodetoak;
	d(todetbGe_oc.om %s ss {
			/ =riPiconf *]++nfo(g.bytess {
			/ =riPiconf *]p+xf_app_hif C_jGOIOece, "was S_ +  {
	GOIOod |_infotle_<=oun-data"infotle_<ite syy(ak;
	d(todetrb TakodetbGoEtle_if_app_ Throhif C_erbd_info(device, "was S ",i	} eISTOR
	list_dak;
	d(todebGe_INFO_bm_xbd_
devisTn -2;

	* protoc"((uc);
foSTORY_START] & ~(("pTO"= pcount, (hg < )-data

	if (hg d_aft_rv that;
Etle_if_app_);ndoresyncmrue;
		} eISTOR
	list_dak;
	d(toder/ioOmi smd.ORDERED_ad_lo_stioeg00); ransit drdno arbd_adif STARs{ /* be"thisways_ao one o devicest last NS( "
	, C e);on sule_n)t md.VERBOSE	} eId as it may seem.rv =toSSSSUCCESS	; eDutomaticalself", device-"pTO"!=oice);T_CR	   ) ap, fulad_nrwr =ice->po one o) {C 0zync requren test_bset prth Tadswr =ice->per_ic->l",t	u64 SYNC_s/->lde_SYNCp(peer_device);n== 0)
c" cd_aft (%s) nrwo rotocoresyncafter an atratelimit",
r(self", device-"pTO)u	sp?-Etle_if0via	d(t:deays_abm_Y_STARTenum d in  */
	tle_	d(enum drbd_afte"pTO",=oice);T_CR	   )eor_SYNCtstat_ Ttoveest last m on success,ueasc ? -1 u_reavic* This is a protocodkicmb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Cttachreturatelimit_sta"dkic) { =unknevi opt dral om %s  type %d, ln id!ous.\n"  {
			/,e {
	GOIOinfasc ? -1 ARTor
_ TmC_Wnne_om %s , "
			     ",
	.
vic* This is a protocoUnplugR-mo	 mb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "C/ioMakedsur
owe'->pa ied/all _subTCPup

	cassoci00)der_bind_lo_subp

	co one o] )  { =unplugL;= (devrateltcp_quicka iect-Brain detp

	.s, iet)dGg	c ? -1 : 1vic* This is a protocoout_ofice, mb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		  (hg p_b			brdesc r" : "this");
	;
_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOer node)
			hg = hg*2;
		drbd_info(dedesnd_chalself", device-"pTO) ap,eer !C e);on sule_n:p,eer !C e);, (hg < :p,eer !C BEHIND:detrb Takodedefault:der_SYNC_TARGET use BAs it  FAILwb cd_aft  =;
,_== 0)
c": WFbitsle_n|WFBesMncT|Behindous.\n");ratelimit",
r(self", device-"pTO)u	sp?-
, SYNC);toout_ofice, mGET use be64n deterTzuis"waor),fbeain deterTzuiblkGOIOi	} ||c ? -1 : 1vic* This is a protocoos2;
a(deviceemb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "p_b			brdesc r" : "this");
				    "no local data.\n");

		"waor_t 	"waor  "is anow;
	tle_if0via
_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOer node)
			hg = hg*2;
		drbd_info(dedes"waorn		be64n deterTzuis"waor)

		OIO
		beain deterTzuiblkGOIOidedeaecoos2pdnd { true;
		} ||drbdgeG:
		if (ch_pe				pran not satisfy peo one oead reqo o;
rrbailedill-T" : REQ_OP_WRITE_ZEROES= e_ot reqo o_if_app_vice-_t reqo o(, "Lost last ID;on sER,Ss"waor" flag set,ve sOIO);0
			}
NOIO in   */
			devo on-data"tsums = true;
		} eloI_HISTOer->cmd ==w} e_ot reqo o->w.cb_ife_;ndo Ttove_b			b;e_ot reqo o->submi _jif
		jiff->p;e_ot reqo o-> hg*2"|= EE_IS_TRIM= e_orr_conflict;
	tentativenot find ao oi_asb_rer	lievi deltail(&t reqo o->w.liev,
ct: Wouldtove_e		} elrice);
			break;
		case 1not find ao oi_asb_re>m	atomici deR {
	GOIO">> 9,
ct: Wouldos2s"wa_	c)eas>tle_if_app_)ubmi _fy peo one omay seem.
y peo o,-T");0
	 (deviAULT_RS_WR)od |_infotle)-  Rd er_conflict;
	tentativenot find ao oi_asb_rer		lievidel(&t reqo o->w.liev_rer		rice);
			break;
		case 1not find ao oi_asb_re>m	l_app_/& cht reqo o(ay seem.
y peo o_rer		tsums = true;
		} elotle_if0vit_dak;
	toth} elbGe_oc, "sna iedTenum d) 1;
		}
No tsums = t)nvert_aGs l cvickry w _SYNC_nd oyself:2s"w_f tal()er an a ==	BM_a ==ays_aos_cdiscard fin) (devbPRIMARY)
toth:der_SYNCos_cdiscard finuest, , s"waor)

	 Wweig);ndodrb_ex(, "Lost last Pcu_d_ACK,Ss"waor" sOIO);ID;on sERu	sp?-
,atomici deRGOIO">> 9,
ct: Wouldos2s"wa_inr;_asc ? -1 u_reavic*  not s
					/ { "is au= 0)
_omylof ;as nbitrary wripktjGOIO  "is a(*fn)mb && __ratelimit(&drbd_rtate))
			drbd_err(dev	.
v;ic* This *  not s
					/ ratelimd"was ler[de		ap,[Pn con]	hhhhis{ 1,=rs_mark		    "p_s
		),a protocoD

	c},p,[Pn con;
	sLY]	hhhhis{ 1,=rs_mark		    "p_s
		),a protocoD

	R plyc},p,[PnRS_ con;
	sLY]hhhis{ 1,=rs_mark		    "p_s
		),a protocoRSD

	R plyc} ,p,[PnBARRIER]	hhhhis{ 0,=rs_mark		    "p_barrier),a protocoBarrierc} ,p,[PnBonf *]	hhhhis{ 1,=0,wo rotocoresyncc} ,p,[PnCOMPRESSiceconf *]his{ 1,=0,wo rotocoresyncc} ,p,[PnUNPLUG;
	MOTE]hhhis{ 0,=0,wo rotocoUnplugR-mo	 c},p,[Pn con;
	QUEST]hhhhis{ 0,=rs_mark		    "p_b			bro o_,a protocoD

	R one oe},p,[PnRS_ con;
	QUEST]his{ 0,=rs_mark		    "p_b			bro o_,a protocoD

	R one oe},p,[Pnon sut drb]	hhhhis{ 1,=0,wo rotocobitsParam_},p,[Pnon sut drb89]hhhhis{ 1,=0,wo rotocobitsParam_},p,[PnviceOCOL]hhhhhhhhis{ 1,=rs_mark		    "p_cro ?col_,a protococro ?col_},p,[Pnle_nS]	hhhhis{ 0,=rs_mark		    "p_s rest,a protocos res	},p,[PnoIZES]	hhhhis{ 0,=rs_mark		    "p_rs_mst,a protocors_ms	},p,[PnoTATE]	hhhhis{ 0,=rs_mark		    "p_r000)),a protocor_aft },p,[PnoTATE_CHG;
	Q]hhhis{ 0,=rs_mark		    "p_o oidevic),a protocoo oidevice},p,[Pnon sule_n]hhhhhhhis{ 0,=rs_mark		    "p_os s re)
	 protocode, "s ree},p,[PnOV;
	QUEST]hhhhhhis{ 0,=rs_mark		    "p_b			bro o_,a protocoD

	R one oe},p,[PnOV;
	sLY]hhhhhhhhis{ 1,=rs_mark		    "p_b			bro o_,a protocoD

	R one oe},p,[PnCSUMnRS_
	QUEST]his{ 1,=rs_mark		    "p_b			bro o_,a protocoD

	R one oe},p,[PnRS_THIN;
	Q]hhhhhis{ 0,=rs_mark		    "p_b			bro o_,a protocoD

	R one oe},p,[PnDELAYnvicBE]hhhhhis{ 0,=rs_mark		    "p_delayocrobe93)
	 protocodkips},p,[PnOUT_O);on s]hhhhhis{ 0,=rs_mark		    "p_b			brdesc)
	 protocoout_ofice, e},p,[PnC2;

ST_CHG;
	Q]his{ 0,=rs_mark		    "p_o oidevic),a protocoo oig4)1tdevic_},p,[PnviceOCOL_UP co
]his{ 1,=rs_mark		    "p_cro ?col_,a protococro ?col_},p,[PnTRIM]	hhhhis{ 0,=rs_mark		    "p_trim),a protocoD

	c},p,[PnRS_ Eev->CAGwb]hhis{ 0,=rs_mark		    "p_b			brdesc)
	 protocoos2;
a(deviceec},p,[Pn->p_u]	hhhhis{ 1,=rs_mark		    "p_wmit_),a protocoD

	c},pv;ic* This rbd_adateemb && __ratelimit(&drbd_ratelimit_se, "Can not <drbd_err(depi

		OIO_t 	hs}  al)ubehif C_=rs_m fterls au_reafoconle"dgeG:o device 64)1);
	if (norotocr)_p peUNNrenc			pr*  not s
					/ bailed*		/= e_ottachth Tadoc}

	*r deG:cerT 64)1);
	if (norotocr)

	  1) {
_ protocr_UI_ { ucatothsi "
			     "_app_ Throhif C__maybe_unplug in   */
_app_ Throhif C__maybe_unplugi "
			     "&iceist_dak;
	u_r_d(tod
rrb	/ =vent, cimd"was ler[pi.imd]in   */
;
	ike((
pi.imd stoARRAYnoIZE(nt, cimd"was ler) %s !imd-> S_c			pr"_SYNC_TARatelimit_sta"Un== 0)
c" p

	com %s  %s (0x%04x)de flag 		/nit_
pi.imd) ",
.		/)odetrak;
	u_r_d(todelbGe_o	hs		}gmd->pktjGOIO  "lSTOR {.		/ =riPioIZES_	d(64)1);
	if (e resynM_OFFSET == (device->p_uuer		rhs +=irs_mark		    "o_qlim)  "lSTOR {.GOIO"> rhs 	d(!imd->u= 0)
_omylof c			pr"_SYNC_TARatelimit_sta"No tmylof _== 0)
c" %s l:%dous.\n"); 		/nit_
pi.imd) ",
.GOIOinfo(dak;
	u_r_d(todelbG"lSTOR {.GOIO"< rhsc			pr"_SYNC_TARatelimit_sta"%s:iun== 0)
c" om %s  now;
	t= 0)
c":%dt protoc":%dous.\n"); 		/nit_
pi.imd) "( wr)rhs ",
.GOIOinfo(dak;
	u_r_d(todelbGG"lSTORrhsc			pr" 1) {
_ protocr_UI_ { ucatothsi "
			     "_app_ Throa(dhretu	} elotle_if_app_ Throa(dhretui "
			     ",
.s");((rhscnfo(d the
	list_ddak;
	u_r_d(todel	 {.GOIO"-= 	hs}delbGG"l 1) {
_ protocr_UI_ { ucatothsi "
			     "imd-> S_eas>tle_ifimd-> Si "
			     "&ice} eISTOR
	li			pr"_SYNC_TARatelimit_sta"SYNC_t protoreqo%
,_=n id ln id!ous.\n"); 		/nit_
pi.imd) "SYN ",
.GOIOinfo(dak;
	u_r_d(todelbG"}
y roles a>hhhhu_r_d(t:deg4)1to one o devicece, "was S "NSe "
	, C viceOCOL_ERRORc->md.ne-D)ea}ic* This rbd_ag4)1tdisce, "wamb && __ratelimit(&drbd_ratelimit_se, "Can not satisfy peer's read request, "
	&& peer_disk > Doc  "is avnreafolock64)1);
	if (cd_aft  is")%d NDALONEe sy roles a>mdesW\nas
	tb
		}tha_statpdevicleanupbd_inf-hmit(&drbd_losshteritMakedsur
o manumakeeo one oeknevs
tb
		}thathteritUs: "bitCKEer_tim b64)1sSS_If",t	u64 tothure 1000); l Tady,er_biap!\rom sioreer o.\nas
	not,in = ex	itoupnvert_er_bterg4)1to one o devicece, "was S "NSe "
	, C u_TWORK_FAILUREc->md.ne-D)ea>mdesdrb_ protocr uid[UI_BIcleanoupnanyB_TRApeitnrdis I_BIillerbd_a "Siet pr(devratelth TadideopT 64)1);
	if (drb_ protocr in  */
64)1);
	if (drb_);ndeli			prdestroy_	u6koneue
64)1);
	if (drb_);ndeli;
rrbai1);
	if (drb_);ndel p(peer ("}
y_app_/& chs, iece, "was S_;_asccu_ TadiSTART in  dr_/* _eaca_	*rryT 64)1);
	if (d request, sbd_asb_st last ich 			pran not satiscal data.\n");g = hg*2;
		drbd_info(de		krefugeG;
		case 1kref)eas>Icu_ TadiY_START)

	 Wweigdisce, "waederbd_info(demnfoEkrefupuG;
		case 1krefe statesestroy_nfo(demnfoEccu_ TadiSTART in }
>Icu_ TadiY_START)

n  */
	lievice, yTuce, "was Sync}

	*r epoch->liev_)eor_SYNC_TARatelimit_sta"As it ION FAILwb: ce, "was Sync}

	*r epoch->liev I_BIce, ybd_infodessk, nosmor
oee', o requeflri i
	equsafede sSETetpdeviepoch_rs_m fteratomici);
(uce, "was Sync}

	*r epoch->epoch_rs_mnc0u	spce, "was Syn);nd.s {
_anyyself:2y;
 = faIMA} 
f_app_0) {
atelimit_sta"Cmit(&drbd_clos;dr_devias */
64)1_segt is_r_sbece, "was S_ p peer)
{
	/* A 64)1_segt is_pddeece, "was S_ ->md.UNKNOWN/
[U64)1_rry_d(t) {
_rbd_iace, mce, "was S_;_aser_conflict;
	t64)1);
	if (not find ao oi_asb_reroc		}g4)1);
	if (cd_aftin  */
oc	>=oicUN;
		)bGwbu
[U_g4)1to one o devicece, "was S "NSe "
	, C UN;
		)bGwbut md.VERBOSE	} forice);
			break;
64)1);
	if (not find ao oi_asb_ren  */
oc	 is")0zync requrenc
o(g4)1to one o devicece, "was S "NSe "
	, C %d NDALONEet md.VERBOSE |lmd.ne-D)ea}ic* This  wri;weigdisce, "waedean not satisfy peer's read request, e, "Can not satiscal data.\n");g = hg*2;
		drbd_info(de	 nbitrary wriiea>mdeswai	}Uevi get thoacttoheyEno ceer { /* ber_conflict;
	tentativenot find ao oi_asb_rer_ttachrei	_ee_lievice, yTst last tentativeacttoc_e		} e_ttachrei	_ee_lievice, yTst last tentativetove_e		} e_ttachrei	_ee_lievice, yTst last tentative Tadiesinforice);
			break;
		case 1not find ao oi_asb_re>mdesWriuiUI_BIce->pp

	can notFSET tK;
	n_tim  (dew uiodoer_bigetpdevios2pdnd { _cwri;evi no 0_aguidhteritritOn m on suTARGETin =uiUI_BIce->panypp

	can notFSET describ { er_bit* equecend {  RSD

	R one o'son =ce->p);nthteritritOn m on success, et pE	equnopp

	can notFSE tK;
	trdrbser_bit* equePnRS_ con;
	sLY b			bs tK;
	nywsenv no eque 30;
de\nthteritrAnm I_i i
	equce, _sedsum_ p devirefd_alc =  u&&s_inr of
ew*  (devic_LRU. Tee (devic_LRU	trdrbs deviwh_sb_op:  -2	C_enclud { er_bitn vde he-IO,_conle"n vdos2pdnd { _cwri_devitrdrbs devib			bser_bito requeflr.r(devratelos_calc lottlenfo(demnfot: Wouldos2 at l = 0nfot: Wouldos2 oth a = 0nfoatomici);
(ut: Wouldos2pdnd { _cwrnc0u	spwakeeup(ut: Wouldm_HI_wai	idedeael_UI_d_
de, m
		case 1notvic_UI_d_ueasc tvic_UI_d__ Sikdevice->ldev-)enum d) 1;
deswai	}Ueviall w_e_;ndos
			o o,-w_e_;ndorss
			o o,-w_);ndorarrier[er_binumakeeo tvic_o one oeetc.econnecr =iegin_ibsro reque	u6kel oneuerbn C_hat l"calc led"r(devratelflush_	u6koneue
 rbd_info(device, "was Syn);ndel_	u6kmnfg	 manuf thviht reqo os(enum d) 1;
desT eque->lde 	u6koneue flush_equevcte,_pri sinceo manuf thviht reqo os()s _sbmtzbG=ce->peqsueuuiu	u64 aguidh Tee [UI_bevice" manuf thviht reqo os() eqs   uevcte,_prde sSEclC_WFn;
	
ey w _SYNCf thviht reqo os().r(devratelflush_	u6koneue
 rbd_info(device, "was Syn);ndel_	u6kmnfg	/* Isice ? uiUAt>aguid[" manuf thviht reqo os() r =ice->ppopulaTARLi!
rn Caguid via" manurry_gctor,ones.\n"bm().r(devratelos_calc lottlenfo(demnf Rk&& c( doing a %s re);
, device->ldev- =;peer (n  */
	sdevicuspdndcetrue;
		c
o(tl_gctorerbd_info(device, "was S_reg	 manumvice, true;
		} ||drbdgeG:
		if (ch_pe				prstateresync finished event, cicyself:2copy_pages.\n");"self:_uuid_disce, "waeds. 				deviceCHANGEdev->md.u	spitsums = true;
		} eD ||des cp_clos;
asser leer o p cdndpage pages cevibe"uefd_rse_a Ispon'Ber_binfnv no use SO_LrenER,Sbultusk appa
	*rbitesE0evibe"uefd_rse}Uever_bimor
on evi20ue->ldes (dev-e oeUI_d Is0ISTOed)c
fu6u6 esAct: "bitn =uin'BIcas
	Ueviexactbitnev reque",t	u64 * Tck uid[Ulves t_btsumpage()tmap!\r leer o urirefd_alc =o requse pages rtzbG=cert_er_bteri_if_app_/& cht reqo osTst last tentativen;
	
e in  */
i)eor_SYNCp(peer_device)n;
	
eyI_BIce, y, kiickry%u 	*rrie
side ce} ei_ifatomici Tadm
		case 1pp_ t_usk_by_net in  */
i)eor_SYNCp(peer_device)pp_ t_usk_by_net  =;d
	t= 0)
c" 0side ce} ei_ifatomici Tadm
		case 1pp_ t_usk in  */
i)eor_SYNCp(peer_device)pp_ t_usk  =;d
	t= 0)
c" 0side ce} 
Id as it may seem.lievice, yTuentative Tadiesi in d as it may seem.lievice, yTuentativeacttoc_e		 in d as it may seem.lievice, yTuentativetove_e		 in d as it may seem.lievice, yTuentative [UI_e		 inasc ? -1 : 1FLICT
rite fid[Uort PRO.VERSION_MIN no PRO.VERSION_MAXh Tee cro ?col_iissand
_bin =0evie res 	C_equqiorkry w e resync start and.>p*>p* M_OFFSE	 hg*2"asse_subSETeroc" arrayEer_tim b64enough roid_UevifuFFSE>p* enhalc m &&s_ p deviwas SyncS cro ?cole p_uupossiblS clugins...>p*>p* Morterw,odheynas
	t= 0)
c" _hat lzerotmap!\ARTor
d. {
		drbd_info(f_app_);ndoM_OFFSETmb && __ratelimit(&drbd_ratelimit_se, "Can not satiss, iet *s		b;e_I_BITMA< cdelimit_snM_OFFSET *p} for		b =vect-Brain detp

	;e_p		}g4)1t{	eear:2commp_uece, "was S "sasb_rerlock!p/
[UI_HISTOer nodemem);
(");0
	rs_mark*<)cre	p 1pro ?col__nrw	}gpun debeai(PRO.VERSION_MINcre	p 1pro ?col__axw	}gpun debeai(PRO.VERSION_MAXcre	p 1M_OFFSE  hg*2",sgpun debeai(PRO.FEATURESueasc ? -1 g4)1td;ndocommp_uece, "was S "sasb, P);
		)bGION_FEATURES
	rs_mark*<),(peernc0u	sFLICT
ritc ? -1 values:
_bit*1 y sbdn =ce->pamh", m{hmit(&drbdt_bit*0 ooppgo;im I_BI	u64 _"T, cleer ot = pgC_W>p*  -1,ce->kt lke 	dffd_alt language,
_bit* enopstill-inr ryTRAnaguid["cleer ogha_stndadeve. {
		drbd_info( satiscooM_OFFSETmb && __ratelimit(&drbd_ratelimit_se, "CdesAs it   get tho=	}g4)1);
	if ( protocr h { /* bI_BITMA< cdelimit_snM_OFFSET *p} rbailedill-t= 0)
 =irs_mark		    "< cdelimit_snM_OFFSET);"Can not <drbd_err(depi

	ls au_reafotle_if_app_);ndoM_OFFSETmce, "was S_reo */

	list_s;

	rc0eafotle_if_app_ Throhif C_e "
			     "&ice} e */

	list_s;

	rc0eafoSTOR {.		/ != P);
		)bGION_FEATURES				pr_SYNC_TARatelimit_sta"S= 0)
c" Cmit(&drbdF_OFFSET <drbd_,t protoc": %s (0x%04x)ous.\n") 		/nit_
pi.imd) ",
.		/)odetI_HISTOe1;deuct drbd {.GOIO"!= e= 0)
				pr_SYNC_TARatelimit_sta"S= 0)
c" Cmit(&drbdF_OFFSET lengthn iu,t protoc": %uafter an at e= 0)
 ",
.GOIOinfo(I_HISTOe1;deuct " : "t.p

	;e_tle_if_app_ Throa(dhretui "
			     ",
	t= 0)
e} e */

	list_s;

	rc0eafop 1pro ?col__nrw	}beain deterTzuipro ?col__nrcre	p 1pro ?col__axw	}beain deterTzuipro ?col__axinfoSTORp 1pro ?col__axw	rite syp 1pro ?col__axw	}zuipro ?col__nreafoSTORPRO.VERSION_MAX <fp 1pro ?col__nrw%sg) at PRO.VERSION_MIN > zuipro ?col__axi
(dak;
	t	  mpatod
r64)1);
	if (e resync start and p_ESULt(iwrncPRO.VERSION_MAX, zuipro ?col__axi;
r64)1);
	if (e resynM_OFFSET = PRO.FEATURES &}beain deterTzuiM_OFFSE  hg*2)} 
f_app_0) {
atelimit_sta"Has SyncS succte,fu":>"s _sb( "A resye",t	u64 cro ?col_iissand %dous.(64)1);
	if (e resync start and)} 
f_app_0) {
atelimit_sta"F_OFFSE	 hg*2"enCREAdo_seero ?col_leve":>0x%x%s%s%s.after an 64)1);
	if (e resynM_OFFSETer an 64)1);
	if (e resynM_OFFSET == (deviceTRIM ? " TRIM" :>"ter an 64)1);
	if (e resynM_OFFSET == (deviceTHIN;
	on s ? " THIN;
	on s" :>"ter an 64)1);
	if (e resynM_OFFSET == (device->p_u ? " WRITE_>p_u" :r an 64)1);
	if (e resynM_OFFSET ? "" :>"enonedeviasc ? -1 1 a>ht	  mpat:
r_SYNC_TARatelimit_sta"t	  mpatiblS  (de 	dalimis:>"s _sb("Ifid[Uort %d-;d
	ce->ked[Uorts %d-;dafter  at PRO.VERSION_MINncPRO.VERSION_MAX,
ew co  1pro ?col__nr, zuipro ?col__axi;
rI_HISTOe1;duct#STO!defirarn 2;FIG_CRYPTO_HMAC	  AS!defirarn 2;FIG_CRYPTO_HMAC_MODULE)	drbd_info( satiscooauthmb && __ratelimit(&drbd_ratelimit_se, "C_SYNC_TARatelimit_sta"T equkernelio ==buiim with
		} 2;FIG_CRYPTO_HMACer_devic_SYNC_TARatelimit_sta"You Isice ? uisCREAb'cram-hmac-alg'y w _SYN-"pTfer_devicI_HISTOe1;duc#evice#defira CHALLENGEdLEN 64LICT R ? -1 value:
r1 - auth succticepe
	0 -e oth a,ot = pgC_W (",t	u64 SYNC_)er -1 - auth  oth a,ouin'BIt = pgC_W.
e = drbd_info( satiscooauthmb && __ratelimit(&drbd_ratelimit_se, "Can not satiss, iet *s		b;e_  30;myliNUllenge[CHALLENGEdLEN];   al64 Bytesh { /* b  30;
SETpaile p(peer ("  30;
StzbG_SETpaile p(peer ("  30;
ce->sliN p(peer (" nbitrary wrikey_len ("  30;secI_H[SHARiceSECREAP];
];  al64 byte rd_darbitrary wriSETpjGOIO  "an not Synshrdesc rdesc;"Can not <drbd_err(depi

	an not neG:ctnf *nc  "is au_r, that;
 alFIXME: PutpdevicNUllenge/SETpaile ine sdevipr
a(deviceecs, iet buffd_.  /* g	ccu_ TadiSTART in nc		}ccu_ C_efd_alc 
64)1);
	if (neG:ctnf in key_len =irtrlen(ncvet 30ep_);cI_H)odememcpy();cI_H, ncvet 30ep_);cI_H,ikey_lenevicIcu_ TadiY_START)

n desc = kma(dev(rs_mark		    "Synshrdesclk st_hhhhhhhcryp deSynshrdescGOIOece, "was S->cram_hmac_tfm),st_hhhhhhh		}
KERNEL_rerlock!desclk{ be"this-1Gwprak;
	toth} e}n desc->tfm		}g4)1);
	if (cram_hmac_tfm;n desc-> hg*2",s0	sfo"thisgryp deSynshr);
keyece, "was S->cram_hmac_tfm, (u8 *));cI_H,ikey_lenevicSTOR"t				pr_SYNC_TARatelimit_sta"gryp deSynshr);
keye)  oth aind_lo%dous.(rc)eas>"this-1Gwprak;
	toth} e}n
	geG:ras om_bytes(myliNUllenge, CHALLENGEdLEN)} for		b =vect-Brain detp

	;e_lock!g4)1t{	eear:2commp_uece, "was S "sasb_lk{ be"this0Gwprak;
	toth} e}n "this!g4)1td;ndocommp_uece, "was S "sasb, P)AUTH_CHALLENGE);0
\n");myliNUllenge, CHALLENGEdLEN)} _lock!rc)wprak;
	toth} fotle_if_app_ Throhif C_e "
			     "&ice} e */

	lik{ be"this0Gwprak;
	toth} e}nfoSTOR {.		/ != P)AUTH_CHALLENGE				pr_SYNC_TARatelimit_sta"S= 0)
c" AuthCNUllenge <drbd_,t protoc": %s (0x%04x)ous.\n") 		/nit_
pi.imd) ",
.		/)odetIthis0Gwprak;
	toth} e}nfoSTOR {.GOIO"> CHALLENGEdLEN * 2				pr_SYNC_TARatelimit_sta"S= 0)
c" AuthCNUllenge <dylof _tooubiger_device"this-1Gwprak;
	toth} e}n
	STOR {.GOIO"< CHALLENGEdLEN)			pr_SYNC_TARatelimit_sta"AuthCNUllenge <dylof _toousevicer_device"this-1Gwprak;
	toth} e}n
	ce->sliN p(kma(dev( {.GOIO
			}
NOIO in lock hg*sliN pp(peer)			pr_SYNC_TARatelimit_sta"kma(dev/_frphg*sliN  oth ar_device"this-1Gwprak;
	toth} e}n
	tle_if_app_ Throa(dhretui "
			     ",hg*sliN ",
.GOIOinfo */

	lik{ be"this0Gwprak;
	toth} e}nfoSTOR!memcmp(myliNUllenge, ,hg*sliN "CHALLENGEdLEN))			pr_SYNC_TARatelimit_sta"Pe->ksnotenvc" _submit_piNUllenge!r_device"this-1Gwprak;
	toth} e}n
	SETpjGOIOhisgryp deSynshrdi-e oGOIOece, "was S->cram_hmac_tfm)easc tpaile p(kma(dev(SETpjGOIO
			}
NOIO in lockc tpaile pp(peer)			pr_SYNC_TARatelimit_sta"kma(dev/_frc tpaile  oth ar_device"this-1Gwprak;
	toth} e}n
	"thisgryp deSynshrdi-e o(desc ",hg*sliN ",
.GOIO,rc tpaileevicSTOR"t				pr_SYNC_TARatelimit_sta"gryp deynshrdi-e o()  oth aind_lo%dous.(rc)eas>"this-1Gwprak;
	toth} e}n
	lock!g4)1t{	eear:2commp_uece, "was S "sasb_lk{ be"this0Gwprak;
	toth} e}n "this!g4)1td;ndocommp_uece, "was S "sasb, P)AUTH_
	oPONSE);0
\n");c tpaile,iSETpjGOIO)} _lock!rc)wprak;
	toth} fotle_if_app_ Throhif C_e "
			     "&ice} e */

	lik{ be"this0Gwprak;
	toth} e}nfoSTOR {.		/ != P)AUTH_
	oPONSE				pr_SYNC_TARatelimit_sta"S= 0)
c" AuthR tpaile <drbd_,t protoc": %s (0x%04x)ous.\n") 		/nit_
pi.imd) ",
.		/)odetIthis0Gwprak;
	toth} e}nfoSTOR {.GOIO"!=iSETpjGOIO)			pr_SYNC_TARatelimit_sta"S= 0)
c" AuthR tpaile <dylof __frwrev->sOIOr_device"this0Gwprak;
	toth} e}nfotle_if_app_ Throa(dhretui "
			     "c tpaile ,iSETpjGOIO)} _lock
	lik{ be"this0Gwprak;
	toth} e}nfoStzbG_SETpaile p(kma(dev(SETpjGOIO
			}
NOIO in lockctzbG_SETpaile pp(peer)			pr_SYNC_TARatelimit_sta"kma(dev/_frctzbG_SETpaile  oth ar_device"this-1Gwprak;
	toth} e}n
	"thisgryp deSynshrdi-e o(desc "myliNUllenge, CHALLENGEdLEN.\n"); ctzbG_SETpaileevicSTOR"t				pr_SYNC_TARatelimit_sta"gryp deynshrdi-e o()  oth aind_lo%dous.(rc)eas>"this-1Gwprak;
	toth} e}n
	"this!memcmp(c tpaile,iStzbG_SETpaile,iSETpjGOIO)} icSTOR"t	eor_SYNCp(peeatelimit_sta"Pe->kauthenviviceecu,TRAn%d bytes HMACafter an at SETpjGOIO)} _evice->"this-1Gw
 toth:dek&& c(,hg*sliN in k&& c(SETpaileevick&& c(StzbG_SETpaileevicSTORdesclk{ beSynshrdesc_zeroRdesclGwprk&& c( dsclGwp}iasc ? -1 thatuc#endif

fo( satis protocrmb && __ratelth Tad *thce, "Can not satisimit(&drbd_ratelimit_shisthcvice, "was S  "is ah} 
f_app_0) {
atelimit_sta" protocr (SE)_stat;dr_deviasdok{ beh		}g4)1tce, "wamce, "was S_reoock);hw	rite			pr"g4)1tdisce, "wamce, "was S_reoo	scaseule_UI_dout_illerruptiblS(HZ_reoo}eoock);hw	ri-1i			pr"_SYNCreturatelimit_sta"Discard {  ",t	u64 ctnfigu  -2	Cer_device-g4)1to one o devicece, "was S "NSe "
	, C 0zync requrenc->md.ne-D)eas>bG"}_conle";hw	rite} icSTORhp> 0				prrlkCtstat_plugi 64)1);
	if (norotocr_plug in  dateemce, "was S_reoorlkCf thvihtlugi 64)1);
	if (norotocr_plug in }d
r64)1tdisce, "wamce, "was S_re
f_app_0) {
atelimit_sta" protocr lerminat;dr_devisc ? -1 : 1FLICT *********sdrbnowh agywsendel ********se = drbd_info( ak;_g4)1tRqSR plymb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not p_o oidevic_o plycr" : "this");
		 wriSEtctdri	}beain deterTzuiSEtctdr)} icSTOR"Etctdri>=aSSSSUCCESS		{fo(s;
	turn 2;

WD
ST_CHG;OKAY,vect-Brain det becomnfobPRIMARY)
	s;
	turn 2;

WD
ST_CHG;FAIL,vect-Brain det becomnfor_SYNC_TARatelimit_sta"R one o) {1000);cNULL;
 oth aiby",hg*: %s (%d)ous.\n")  SYNC);toevicrr",
r(SEtctdr), SEtctdr)} >bG"wakeeup(uct-Brain det) { _wai	idedec ? -1 : 1vic* This is aak;_RqSR plymb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		  (hg p_o oidevic_o plycr" : "this");
		 wriSEtctdri	}beain deterTzuiSEtctdr)} ic_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOer node)
			hg = hg*2;
		drbd_info(dedeinfo(deviturn 2;

WD
ST_CHG;REQ,vect-Brain det becom		{fo(d as it may seem.64)1);
	if (e resync start and < d00info(I_HISTOak;_g4)1tRqSR plym "
			     ",
 in }d
rSTOR"Etctdri>=aSSSSUCCESS		{fo(s;
	turn L
ST_CHG;SUCCESS,
ct: Would becomnfobPRIMARY)
	s;
	turn L
ST_CHG;FAIL,vet: Would becomnfo
			  _TARGET use BReone o) {1000);cNULL;
 oth aiby",hg*: %s (%d)ous.\n") SYNC);toevicrr",
r(SEtctdr), SEtctdr)} >bG"wakeeup(uself", device_wai	idedec ? -1 : 1vic* This is aak;_P { tb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Cc ? -1 _app_);ndo) { _a iece, "was S_;_avic* This is aak;_P { A ieb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "C/io Ttsor
oidle"nI_dout_bterg4)1);
	if (me
	.s, iet dek dek_rcvnI_do		}g4)1);
	if (neG:ctnfet) { _is *HZin  */
	(devi  "_s;
	turnGOT_Pren_ACK,Sect-Brain det becom	\n"wakeeup(uct-Brain det) { _wai	idedec ? -1 : 1vic* This is aak;_IsInSe, mb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		  (hg p_b			brTck r" : "this");
			ewaor_t 	"waorn		be64n deterTzuis"waor)

	is aolkGOIO
		beain deterTzuiblkGOIOidede_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOer node)
			hg = hg*2;
		drbd_info(deded as it may seem.rbd_info(device, "was Syne resync start and >=a89idede 1) {
_ hg*2; o(, "Lost last beain deterTzui; o_num)	} ||drbdgeG:
		if (ch_pe				prstateos_cdiscard finuest, , s"waor)

	 Wweig);_errice, mGET use s"waor" blkGOIOide
		}
os2sit__csums_equqd[Uosice ?   u&&y w uni&s_ p BM_B	devEST:
 ->lde_: Wouldos2sit__csum +=i(olkGOIO
>> BM_B	devESHIFTu	spitsums = true;
		} eD eaecoos2pdnd { true;
		} ,atomici deRolkGOIO
>> 9,
ct: Wouldos2s"wa_inr;_asc ? -1 : 1vic* This is 
h", mvic_o q_cNULL;_o oidevicTED)
		return -2;

	*rule_nr  1;
ide s"waor_t 	"waor.\n") t,ve sD)
		rrb_root_broot, bailed  30;
fun	.\n") t,ve && peer_dio oieY &&onK;
" bool mie,TRA_oke, "Can not satiso one oeao o;
ran not reoi  "_SYNC_tm;_aser_conflict;
	tentativenot find ao oi_asb_rero o_iff tiso one omay seem.root, ide s"waor, mie,TRA_ok, fun	 in  */
;
	ike((
!o o_		{fo(sice);
			break;
		case 1not find ao oi_asb_reorI_HISTOer nodeuco_io oimod(o o,-wK;
" &m)easrice);
			break;
		case 1not find ao oi_asb_re>mSTORm.reoc
o(g4iscard ma o)r,tuinished evemevisc ? -1 : 1FLI* This is aak;_B_asbA ieb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		  (hg p_b			brTck r" : "this");
			ewaor_t 	"waorn		be64n deterTzuis"waor)

	is aolkGOIO
		beain deterTzuiblkGOIOide	&& peer_dio oieY &&onK;
dede_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOer node)
			hg = hg*2;
		drbd_info(dede 1) {
_ hg*2; o(, "Lost last beain deterTzui; o_num)	} ||drbdzuibl		bre/ =riID;on sERu			prstate);_errice, mGET use s"waor" blkGOIOide
	aecoos2pdnd { true;
		} ,y rolesOy *]}desnd_chal {
			/) ap,eer !P_RS_WRITE_ACK:r anK;
	= WRITE_ACKicecY_PEER_AND;oISreoor Takodeeer !P_WRITE_ACK:r anK;
	= WRITE_ACKicecY_PEERreoor Takodeeer !P_RECV_ACK:r anK;
	= RECV_ACKicecY_PEERreoor Takodeeer !P_SUPERSEDED:r anK;
	=  2;FLICT_
	oOLVEDreoor Takodeeer !P_RETRY_WRITE:r anK;
	= POSTPONE_WRITEreoor Takodedefault:derBUG(lGwp}iasc ? -1 h", mvic_o q_cNULL;_o oidevicTay seem.ruibl		bre/,Ss"waor" flag set,v
		case 1self:2o one o]e __fun	__, flag set,vwK;
" faIMA) 1FLI* This is aak;_Ne A ieb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		  (hg p_b			brTck r" : "this");
			ewaor_t 	"waorn		be64n deterTzuis"waor)

	is aGOIO
		beain deterTzuiblkGOIOide	is au_reafo_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOer node)
			hg = hg*2;
		drbd_info(dede 1) {
_ hg*2; o(, "Lost last beain deterTzui; o_num)	} ||drbdzuibl		bre/ =riID;on sERu			prsecoos2pdnd { true;
		} ,ystateos_ oth a finuest, , s"waor,=rs_m)eas>I_HISTO0} e}nfotle_ifh", mvic_o q_cNULL;_o oidevicTay seem.ruibl		bre/,Ss"waor" flag set,
		case 1self:2o one o]e __fun	__, flag set,u_d_ACKED,ot uO)} _lock
	lik{ be	}
Pro ?col_A haqunopP_WRITE_ACKs,iap!\ces Pcu_d_ACKshte setTee ma o)r reobmtzbG= l Tadyat lg4iscardd, et pEfor
on ete seto one oeequnopdev-er_inr oflg4lli and ynsh{ /* be	}
In
Pro ?col_Bdn =mtzbG= l Tadyace->pak; a!P_RECV_ACKte setbutpdevnigetpa Pcu_d_ACKbd_infwards{ /* bestate);_eout_ofice, mGET use s"waor,=rs_m)eas}isc ? -1 : 1FLI* This is aak;_Ne DR plymb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		  (hg p_b			brTck r" : "this");
			ewaor_t 	"waorn		be64n deterTzuis"waor)

fo_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOer node)
			hg = hg*2;
		drbd_info(dede 1) {
_ hg*2; o(, "Lost last beain deterTzui; o_num)	} ||			  _TARGET use BGk; Ne DR ply; S"waorn%llusailen*%u.after  at kdevice->ldev-ldev-)s"waor" beain deterTzuiblkGOIOi	} ||c ? -1 h", mvic_o q_cNULL;_o oidevicTay seem.ruibl		bre/,Ss"waor" flag set,v
		case 1 Tadio one o]e __fun	__, flag set,vu_d_ACKED,ofaIMA) 1FLI* This is aak;_Ne RSDR plymb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		ewaor_t 	"waor  "is anow;

		  (hg p_b			brTck r" : "this");
	a
_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOer node)
			hg = hg*2;
		drbd_info(dedes"waorn		be64n deterTzuis"waor)

		OIO
		beain deterTzuiblkGOIOidede 1) {
_ hg*2; o(, "Lost last beain deterTzui; o_num)	} ||	ecoos2pdnd { true;
		} ||drbdgeG:
		i_ifidevicTay seem.D_FAILwbe				prstateos_cdiscard finuest, , s"waor)

	 snd_chal {
			/) ap,eeer !P_u_d_RS_ 
	sLY:\n") SYNCos_ oth a finuest, , s"waor,=rs_m)eas>eer !P_RS_CA sEL:detrb Takodeedefault:derrBUG(lGwpo}eootsums = true;
		} eD ||c ? -1 : 1FLI* This is aak;_BarrierA ieb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not p_barrierrTck r" : "this");
			n not satisfy peer's read request, "
	is avnreafotl_r leer i "
			     ",uibarrier[ beain deterTzui; tjGOIOi	} ||ccu_ TadiSTART in  dr_/* _eaca_	*rryT 64)1);
	if (d request, sbd_asb_st last ich 			pran not satiscal data.\n");g = hg*2;
		drbd_info(deeoock);self", devicee"pTO",=oicAHEAD &&r an atatomici Tadm
		case 1ap_ t_fltzbG_ p p0 &&r an at	(devi  "_s;
	turnAHEAD_TO on success,,vet: Would becomi			pr"_elf", deviat_ Ttove_UI_d_.expiSET = jiff->p + HZin 		 deltI_d_(uself", deviat_ Ttove_UI_d_)eas>bG"}icIcu_ TadiY_START)

n c ? -1 : 1FLI* This is aak;_OVResultmb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Can not satisfy peer's read request, "
			    "no local data.\n");

		  (hg p_b			brTck r" : "this");
				    "no local da_	u6kta.w

		ewaor_t 	"waor  "is anow;

a
_nr = 34;
			}g4)1t{
				drbd_i "
			     ",
which in  */
			devbGgA		/
[UI_HISTOer node)
			hg = hg*2;
		drbd_info(dedes"waorn		be64n deterTzuis"waor)

		OIO
		beain deterTzuiblkGOIOidede 1) {
_ hg*2; o(, "Lost last beain deterTzui; o_num)	} ||STORre64n deterTzuibl		bre/) =riID;OUT_O);on s	eor_SYNCoveout_ofice, _/*undnuest, , s"waor,=rs_m)easevice->oveout_ofice, _pris true;
		} ||drbd!geG:
		if (ch_pe	as>I_HISTO0} 
rstateos_cdiscard finuest, , s"waor)

		ecoos2pdnd { true;
		} ||--;
		drbdoveleftat;
 alcar's advalc eerognote  o)p mar&soedeviUevieverybset prmegabyte rd_ddrbd(;
		drbdoveleft & 0x200		p p0x200	eor_SYNCadvalc oos2mar&snuest, , ;
		drbdoveleft	} ||drbd;
		drbdoveleft 	rite			prdw = kma(dev(rs_marka.w)
			}
NOIO in   */
.w)			pr"_w->w.cb_ifwCovef thvied;	pr"_w->)
			hg =.\n");

	or_SYNConeue_	u6k
 rbd_info(device, "was Syn);ndel_	u6k,vetw->w)eas>bPRIMARY)
		ttach_TARGET use Bkma(dev(.w)	 oth a.device-oveout_ofice, _pris true;
		} n") SYNCoTtove_f thviedtrue;
		} n"bG"}ictsums = true;
		} ec ? -1 : 1FLI* This is aak;_dkicmb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "Cc ? -1 : 1FLI* ))
		me
	ss, i			/ { "	OIO_t pktjGOIO  "is a(*fn)mb && __ratelimit(&drbd_ratelimit_state))
			drbd_err(dev);pv;ic* This rbd_as;
	rcvnI_domb && __ratelimit(&drbd_ratelimit_stabool ) { _nI_doute, "Cdev-lt
				    "neG:ctnf *nc  g	ccu_ TadiSTART in nc		}ccu_ C_efd_alc 
64)1);
	if (neG:ctnf in t : "t{ _nI_dout ? ncve"t{ _nI_do : ncve"t{ _is ;icIcu_ TadiY_START)

n t *= HZin  */
) { _nI_doute,	 t /, d0od
r64)1);
	if (me
	.s, iet dek dek_rcvnI_do		}tea}ic* This rbd_as;
	) { _nI_doutmb && __ratelimit(&drbd_ratelimit_se, "Ca;
	rcvnI_domatelimit_sta1)ea}ic* This rbd_as;
	idle_nI_doutmb && __ratelimit(&drbd_ratelimit_se, "Ca;
	rcvnI_domatelimit_sta0)ea}ic* This * ))
		me
	ss, i			/ drb_ protocr_tbl[de		ap,[PnPren]	hhhhis{ 0,=ak;_P { _},p,[Pnvren_ACK]	hhhhis{ 0,=ak;_P { A ic},p,[PnRECV_ACK]	hhhhis{ rs_mark		    "p_b			brdrb),aak;_B_asbA ic},p,[Pn-RITE_ACK]	hhhhis{ rs_mark		    "p_b			brdrb),aak;_B_asbA ic},p,[PnRS_WRITE_ACK]hhhhis{ rs_mark		    "p_b			brdrb),aak;_B_asbA ic},p,[PnSUPERSEDED]hhhis{ rs_mark		    "p_b			brdrb),aak;_B_asbA ic},p,[Pnu_d_ACK]	hhhhis{ rs_mark		    "p_b			brdrb),aak;_Ne A ic},p,[Pnu_d_D
	sLY]	hhhhis{ rs_mark		    "p_b			brdrb),aak;_Ne DR plyc},p,[Pnu_d_RS_ 
	sLY]hhhis{ rs_mark		    "p_b			brdrb),aak;_Ne RSDR plye},p,[PnOV;
	SULT]	hhhhis{ rs_mark		    "p_b			brdrb),aak;_OVResulte},p,[PnBARRIER_ACK]	hhhhis{ rs_mark		    "p_barrierrTck),aak;_BarrierA i },p,[PnoTATE_CHG;
	sLY]his{ rs_mark		    "p_o oidevic_o ply),aak;_RqSR plyc},p,[PnRS_IS_IN;on s]hhhis{ rs_mark		    "p_b			brdrb),aak;_IsInSe, e},p,[PnDELAYnvicBE]hhhhhis{ rs_mark		    "p_delayocrobe93)
	ak;_dkicc},p,[PnRS_CA sEL]hhhhhhhis{ rs_mark		    "p_b			brdrb),aak;_Ne RSDR plye},p,[PnC2;

ST_CHG;
	sLY]={ rs_mark		    "p_o oidevic_o ply),aak;_g4)1tRqSR plyc},p,[PnRETRY_WRITE]	hhhhis{ rs_mark		    "p_b			brdrb),aak;_B_asbA ic},pv;icfo( satisdrb_ protocrmb && __ratelth Tad *thce, "Can not satisimit(&drbd_ratelimit_shisthcvice, "was S  "* ))
		me
	ss, i			/ *b	/ =vpeer ("an not <drbd_err(depi

	device->ldev-lpr
_ Tcv_jif  "is athat	rbd_a*bufhhhhis64)1);
	if (me
	.rbuf  "is atprotoc"his0Gwparbitrary wrihif C_jGOIO xf_app_hif C_jGOIOece, "was S_de	is au= 0)
 hhishif C_jGOIOde	bool ) { _nI_dout_acttoc = faIMA} "an not Scase_param_param_is{ .Scase_priorheyE= 2 }	sfo"thisScase_a;
scaseuler( get th, SCHED_RR "&iaramevicSTOR"t_<ite sy_SYNC_TARatelimit_sta"satisdrb_ protocr: ERRORas;
 priorhey, SEt=%dous.(rc)eafoconle"dgeG:o devicethce_p peUNNrenc			prratelth Tadoc}

	*r deG:cerTthceod
rrb4)1to clC_m_netht reqo osTce, "was S_re
feinfo(devi  "_gctor,turnSENDnvren,vect-Brain det becom		{fo(  */
_app_);ndo) { Tce, "was S_c			pr"
			  _TARatelimit_sta"satis);ndo) { \ces  oth ar_deviceprak;
	reatelimivicep}o rls;
	) { _nI_doutmce, "was S_reoo	) { _nI_dout_acttoc = t uO} n"bGeootr
_ Tcv_jif 		jiff->p;e_o"this_app_ Throshort(64)1);
	if (me
	.s, iet,iapf
	t= 0)
-tprotoc",ite} ic		}
Note:r an*Oer NTR	 (_shme
	)dn =ak; a!bitralr an*OerAGAIN	 (_shme
	)drcvnI_do	expiSEdr an*OerC2;

	SET	bset prside_clos;dr oflg4)1);
	ifr an*Oer
	STARTSYS  (_shs
		)dn =ak; a!bitralr an*O"t_<i 0	bset prtheviebove:iun== 0)
c" SYNC_!r an*O"t_==_== 0)
c": fullehif C_=evi ommp_ur an*O"t_<i == 0)
c": "woken"iby"bitral dur { \tprotocr an*O"t_==_0	b: "atelimit_shshuri;evi by",hg*"r an*/
feinfo	ike((
"t_> 0	c			pr"tprotoc"h+=athat			apf	h+=athat		Dutomatical"t_==_0		{fo(  */
(deviturn0zync requ_SENT,vect-Brain det becom		{fo( Cdev-lt
			oEccu_ TadiSTART in 		 t : ccu_ C_efd_alc 
64)1);
	if (neG:ctnf ve"t{ _nI_do * HZ/10
			oEccu_ TadiY_START)

n 		 t : rei	_eY &&_nI_doutmce, "was Set) { _wai	, flag _hhhhhhhce, "was S->c1000);<!C e);
	sORTut drbS, flag _hhhhhhhv_rer		  */
(ist_ddrb Takodeep}o rl			  _TARatelimit_sta"me
	 atelimit_shshuri;evi by",hg*er_device-ak;
	reatelimiviceDutomatical"t_==_erAGAIN		{fo( 	}
Ifo_subp

	cs, iet tprotoc"hSS_IB_TRAhmeanconle.\n") * tK;
	is=akod4enough:	ce->kequqiin_ialtoc{ /* be  */
(I_d_d_infmce, "was Setlais_rprotoc",itr
_ Tcv_jifeist_drb4)tinuO} n"  */
) { _nI_dout_acttocc			pr"
			  _TARatelimit_sta"P { A ic;im I_BIarrivey w nI_d.r_deviceprak;
	reatelimivicep}o rls;
	turnSENDnvren,vect-Brain det becomvicepb4)tinuO} n"Dutomatical"t_==_er NTR		{fo( 	}
maybe_ratelth TadideopT): deviwhnle"b4)diit_shwin_iI_Bict_er") * maybe_wokeniUevi);ndo) { :owe'n_i);nd a!) { \ebove.\n") * p_uucNULL;
_subScvnI_do	/* be flush_bitrals( get thmvicepb4)tinuO} n"Dutomat{o rl			  _TARatelimit_sta"s		bro cvmsgtc ? -1kry%dous.(rc)eas>-ak;
	reatelimiviceDe
feinfotprotoc"hi=_== 0)
* A 6	/ =ripeer)			pr|drbd;
ctdrohif C_e "
			     "64)1);
	if (me
	.rbuf "&iceist_drak;
	reatelimivicepb	/ =vedrb_ protocr_tbl[pi.imd]in  oSTOR {.		/ stoARRAYnoIZE(drb_ protocr_tbl) %s !imd-> S_			pr"
			  _TARatelimit_sta"Un== 0)
c" me
	 om %s  %s (0x%04x)ous.\n");g 		/nit_
pi.imd) ",
.		/)odetrrak;
	disce, "wavicep}o rlt= 0)
 =ihif C_jGOIO +}gmd->pktjGOIO  "l drbd {.GOIO"!= e= 0)
 -ihif C_jGOIO_			pr"
			  _TARatelimit_sta"Wrev->om %s  now; _shme
	 (c:=;d
	ln id)ous.\n");g,
.		/ ",
.GOIOinfo(drak;
	reatelimivicep}o r}
feinfotprotoc"hi=_== 0)
_			pr"bool u_reafos>tle_ifimd-> Si "
			     "&ice} eIISTOR
	li			pr""_SYNC_TARatelimit_sta"%pf  oth ar_d "imd-> S_eas>drak;
	reatelimivicep