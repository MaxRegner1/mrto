// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2010 Kent Overstreet <kent.overstreet@gmail.com>
 *
 * Uses a block device as cache for other block devices; optimized for SSDs.
 * All allocation is done in buckets, which should match the erase block size
 * of the device.
 *
 * Buckets containing cached data are kept on a heap sorted by priority;
 * bucket priority is increased on cache hit, and periodically all the buckets
 * on the heap have their priority scaled down. This currently is just used as
 * an LRU but in the future should allow for more intelligent heuristics.
 *
 * Buckets have an 8 bit counter; freeing is accomplished by incrementing the
 * counter. Garbage collection is used to remove stale pointers.
 *
 * Indexing is done via a btree; nodes are not necessarily fully sorted, rather
 * as keys are inserted we only sort the pages that have not yet been written.
 * When garbage collection is run, we resort the entire node.
 *
 * All configuration is done via sysfs; see Documentation/bcache.txt.
 */

#include "bcache.h"
#include "btree.h"
#include "debug.h"
#include "extents.h"

#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/hash.h>
#include <linux/kthread.h>
#include <linux/prefetch.h>
#include <linux/random.h>
#include <linux/rcupdate.h>
#include <linux/sched/clock.h>
#include <linux/rculist.h>

#include <trace/events/bcache.h>

/*
 * Todo:
 * register_bcache: Return errors out to userspace correctly
 *
 * Writeback: don't undirty key until after a cache flush
 *
 * Create an iterator for key pointers
 *
 * On btree write error, mark bucket such that it won't be freed from the cache
 *
 * Journalling:
 *   Check for bad keys in replay
 *   Propagate barriers
 *   Refcount journal entries in journal_replay
 *
 * Garbage collection:
 *   Finish incremental gc
 *   Gc should free old UUIDs, data for invalid UUIDs
 *
 * Provide a way to list backing device UUIDs we have data cached for, and
 * probably how long it's been since we've seen them, and a way to invalidate
 * dirty data for devices that will never be attached again
 *
 * Keep 1 min/5 min/15 min statistics of how busy a block device has been, so
 * that based on that and how much dirty data we have we can keep writeback
 * from being starved
 *
 * Add a tracepoint or somesuch to watch for writeback starvation
 *
 * When btree depth > 1 and splitting an interior node, we have to make sure
 * alloc_bucket() cannot fail. This should be true but is not completely
 * obvious.
 *
 * Plugging?
 *
 * If data write is less than hard sector size of ssd, round up offset in open
 * bucket to the next whole sector
 *
 * Superblock needs to be fleshed out for multiple cache devices
 *
 * Add a sysfs tunable for the number of writeback IOs in flight
 *
 * Add a sysfs tunable for the number of open data buckets
 *
 * IO tracking: Can we track when one process is doing io on behalf of another?
 * IO tracking: Don't use just an average, weigh more recent stuff higher
 *
 * Test module load/unload
 */

#define MAX_NEED_GC		64
#define MAX_SAVE_PRIO		72

#define PTR_DIRTY_BIT		(((uint64_t) 1 << 36))

#define PTR_HASH(c, k)							\
	(((k)->ptr[0] >> c->bucket_bits) | PTR_GEN(k, 0))

#define insert_lock(s, b)	((b)->level <= (s)->lock)

/*
 * These macros are for recursing down the btree - they handle the details of
 * locking and looking up nodes in the cache for you. They're best treated as
 * mere syntax when reading code that uses them.
 *
 * op->lock determines whether we take a read or a write lock at a given depth.
 * If you've got a read lock and find that you need a write lock (i.e. you're
 * going to have to split), set op->lock and return -EINTR; btree_root() will
 * call you again and you'll have the correct lock.
 */

/**
 * btree - recurse down the btree on a specified key
 * @fn:		function to call, which will be passed the child node
 * @key:	key to recurse on
 * @b:		parent btree node
 * @op:		pointer to struct btree_op
 */
#define btree(fn, key, b, op, ...)					\
({									\
	int _r, l = (b)->level - 1;					\
	bool _w = l <= (op)->lock;					\
	struct btree *_child = bch_btree_node_get((b)->c, op, key, l,	\
						  _w, b);		\
	if (!IS_ERR(_child)) {						\
		_r = bch_btree_ ## fn(_child, op, ##__VA_ARGS__);	\
		rw_unlock(_w, _child);					\
	} else								\
		_r = PTR_ERR(_child);					\
	_r;								\
})

/**
 * btree_root - call a function on the root of the btree
 * @fn:		function to call, which will be passed the child node
 * @c:		cache set
 * @op:		pointer to struct btree_op
 */
#define btree_root(fn, c, op, ...)					\
({									\
	int _r = -EINTR;						\
	do {								\
		struct btree *_b = (c)->root;				\
		bool _w = insert_lock(op, _b);				\
		rw_lock(_w, _b, _b->level);				\
		if (_b == (c)->root &&					\
		    _w == insert_lock(op, _b)) {			\
			_r = bch_btree_ ## fn(_b, op, ##__VA_ARGS__);	\
		}							\
		rw_unlock(_w, _b);					\
		bch_cannibalize_unlock(c);				\
		if (_r == -EINTR)					\
			schedule();					\
	} while (_r == -EINTR);						\
									\
	finish_wait(&(c)->btree_cache_wait, &(op)->wait);		\
	_r;								\
})

static inline struct bset *write_block(struct btree *b)
{
	return ((void *) btree_bset_first(b)) + b->written * block_bytes(b->c);
}

static void bch_btree_init_next(struct btree *b)
{
	/* If not a leaf node, always sort */
	if (b->level && b->keys.nsets)
		bch_btree_sort(&b->keys, &b->c->sort);
	else
		bch_btree_sort_lazy(&b->keys, &b->c->sort);

	if (b->written < btree_blocks(b))
		bch_bset_init_next(&b->keys, write_block(b),
				   bset_magic(&b->c->sb));

}

/* Btree key manipulation */

void bkey_put(struct cache_set *c, struct bkey *k)
{
	unsigned i;

	for (i = 0; i < KEY_PTRS(k); i++)
		if (ptr_available(c, k, i))
			atomic_dec_bug(&PTR_BUCKET(c, k, i)->pin);
}

/* Btree IO */

static uint64_t btree_csum_set(struct btree *b, struct bset *i)
{
	uint64_t crc = b->key.ptr[0];
	void *data = (void *) i + 8, *end = bset_bkey_last(i);

	crc = bch_crc64_update(crc, data, end - data);
	return crc ^ 0xffffffffffffffffULL;
}

void bch_btree_node_read_done(struct btree *b)
{
	const char *err = "bad btree header";
	struct bset *i = btree_bset_first(b);
	struct btree_iter *iter;

	iter = mempool_alloc(b->c->fill_iter, GFP_NOIO);
	iter->size = b->c->sb.bucket_size / b->c->sb.block_size;
	iter->used = 0;

#ifdef CONFIG_BCACHE_DEBUG
	iter->b = &b->keys;
#endif

	if (!i->seq)
		goto err;

	for (;
	     b->written < btree_blocks(b) && i->seq == b->keys.set[0].data->seq;
	     i = write_block(b)) {
		err = "unsupported bset version";
		if (i->version > BCACHE_BSET_VERSION)
			goto err;

		err = "bad btree header";
		if (b->written + set_blocks(i, block_bytes(b->c)) >
		    btree_blocks(b))
			goto err;

		err = "bad magic";
		if (i->magic != bset_magic(&b->c->sb))
			goto err;

		err = "bad checksum";
		switch (i->version) {
		case 0:
			if (i->csum != csum_set(i))
				goto err;
			break;
		case BCACHE_BSET_VERSION:
			if (i->csum != btree_csum_set(b, i))
				goto err;
			break;
		}

		err = "empty set";
		if (i != b->keys.set[0].data && !i->keys)
			goto err;

		bch_btree_iter_push(iter, i->start, bset_bkey_last(i));

		b->written += set_blocks(i, block_bytes(b->c));
	}

	err = "corrupted btree";
	for (i = write_block(b);
	     bset_sector_offset(&b->keys, i) < KEY_SIZE(&b->key);
	     i = ((void *) i) + block_bytes(b->c))
		if (i->seq == b->keys.set[0].data->seq)
			goto err;

	bch_btree_sort_and_fix_extents(&b->keys, iter, &b->c->sort);

	i = b->keys.set[0].data;
	err = "short btree key";
	if (b->keys.set[0].size &&
	    bkey_cmp(&b->key, &b->keys.set[0].end) < 0)
		goto err;

	if (b->written < btree_blocks(b))
		bch_bset_init_next(&b->keys, write_block(b),
				   bset_magic(&b->c->sb));
out:
	mempool_free(iter, b->c->fill_iter);
	return;
err:
	set_btree_node_io_error(b);
	bch_cache_set_error(b->c, "%s at bucket %zu, block %u, %u keys",
			    err, PTR_BUCKET_NR(b->c, &b->key, 0),
			    bset_block_offset(b, i), i->keys);
	goto out;
}

static void btree_node_read_endio(struct bio *bio)
{
	struct closure *cl = bio->bi_private;
	closure_put(cl);
}

static void bch_btree_node_read(struct btree *b)
{
	uint64_t start_time = local_clock();
	struct closure cl;
	struct bio *bio;

	trace_bcache_btree_read(b);

	closure_init_stack(&cl);

	bio = bch_bbio_alloc(b->c);
	bio->bi_iter.bi_size = KEY_SIZE(&b->key) << 9;
	bio->bi_end_io	= btree_node_read_endio;
	bio->bi_private	= &cl;
	bio->bi_opf = REQ_OP_READ | REQ_META;

	bch_bio_map(bio, b->keys.set[0].data);

	bch_submit_bbio(bio, b->c, &b->key, 0);
	closure_sync(&cl);

	if (bio->bi_status)
		set_btree_node_io_error(b);

	bch_bbio_free(bio, b->c);

	if (btree_node_io_error(b))
		goto err;

	bch_btree_node_read_done(b);
	bch_time_stats_update(&b->c->btree_read_time, start_time);

	return;
err:
	bch_cache_set_error(b->c, "io error reading bucket %zu",
			    PTR_BUCKET_NR(b->c, &b->key, 0));
}

static void btree_complete_write(struct btree *b, struct btree_write *w)
{
	if (w->prio_blocked &&
	    !atomic_sub_return(w->prio_blocked, &b->c->prio_blocked))
		wake_up_allocators(b->c);

	if (w->journal) {
		atomic_dec_bug(w->journal);
		__closure_wake_up(&b->c->journal.wait);
	}

	w->prio_blocked	= 0;
	w->journal	= NULL;
}

static void btree_node_write_unlock(struct closure *cl)
{
	struct btree *b = container_of(cl, struct btree, io);

	up(&b->io_mutex);
}

static void __btree_node_write_done(struct closure *cl)
{
	struct btree *b = container_of(cl, struct btree, io);
	struct btree_write *w = btree_prev_write(b);

	bch_bbio_free(b->bio, b->c);
	b->bio = NULL;
	btree_complete_write(b, w);

	if (btree_node_dirty(b))
		schedule_delayed_work(&b->work, 30 * HZ);

	closure_return_with_destructor(cl, btree_node_write_unlock);
}

static void btree_node_write_done(struct closure *cl)
{
	struct btree *b = container_of(cl, struct btree, io);

	bio_free_pages(b->bio);
	__btree_node_write_done(cl);
}

static void btree_node_write_endio(struct bio *bio)
{
	struct closure *cl = bio->bi_private;
	struct btree *b = container_of(cl, struct btree, io);

	if (bio->bi_status)
		set_btree_node_io_error(b);

	bch_bbio_count_io_errors(b->c, bio, bio->bi_status, "writing btree");
	closure_put(cl);
}

static void do_btree_node_write(struct btree *b)
{
	struct closure *cl = &b->io;
	struct bset *i = btree_bset_last(b);
	BKEY_PADDED(key) k;

	i->version	= BCACHE_BSET_VERSION;
	i->csum		= btree_csum_set(b, i);

	BUG_ON(b->bio);
	b->bio = bch_bbio_alloc(b->c);

	b->bio->bi_end_io	= btree_node_write_endio;
	b->bio->bi_private	= cl;
	b->bio->bi_iter.bi_size	= roundup(set_bytes(i), block_bytes(b->c));
	b->bio->bi_opf		= REQ_OP_WRITE | REQ_META | REQ_FUA;
	bch_bio_map(b->bio, i);

	/*
	 * If we're appending to a leaf node, we don't technically need FUA -
	 * this write just needs to be persisted before the next journal write,
	 * which will be marked FLUSH|FUA.
	 *
	 * Similarly if we're writing a new btree root - the pointer is going to
	 * be in the next journal entry.
	 *
	 * But if we're writing a new btree node (that isn't a root) or
	 * appending to a non leaf btree node, we need either FUA or a flush
	 * when we write the parent with the new pointer. FUA is cheaper than a
	 * flush, and writes appending to leaf nodes aren't blocking anything so
	 * just make all btree node writes FUA to keep things sane.
	 */

	bkey_copy(&k.key, &b->key);
	SET_PTR_OFFSET(&k.key, 0, PTR_OFFSET(&k.key, 0) +
		       bset_sector_offset(&b->keys, i));

	if (!bio_alloc_pages(b->bio, __GFP_NOWARN|GFP_NOWAIT)) {
		int j;
		struct bio_vec *bv;
		void *base = (void *) ((unsigned long) i & ~(PAGE_SIZE - 1));

		bio_for_each_segment_all(bv, b->bio, j)
			memcpy(page_address(bv->bv_page),
			       base + j * PAGE_SIZE, PAGE_SIZE);

		bch_submit_bbio(b->bio, b->c, &k.key, 0);

		continue_at(cl, btree_node_write_done, NULL);
	} else {
		b->bio->bi_vcnt = 0;
		bch_bio_map(b->bio, i);

		bch_submit_bbio(b->bio, b->c, &k.key, 0);

		closure_sync(cl);
		continue_at_nobarrier(cl, __btree_node_write_done, NULL);
	}
}

void __bch_btree_node_write(struct btree *b, struct closure *parent)
{
	struct bset *i = btree_bset_last(b);

	lockdep_assert_held(&b->write_lock);

	trace_bcache_btree_write(b);

	BUG_ON(current->bio_list);
	BUG_ON(b->written >= btree_blocks(b));
	BUG_ON(b->written && !i->keys);
	BUG_ON(btree_bset_first(b)->seq != i->seq);
	bch_check_keys(&b->keys, "writing");

	cancel_delayed_work(&b->work);

	/* If caller isn't waiting for write, parent refcount is cache set */
	down(&b->io_mutex);
	closure_init(&b->io, parent ?: &b->c->cl);

	clear_bit(BTREE_NODE_dirty,	 &b->flags);
	change_bit(BTREE_NODE_write_idx, &b->flags);

	do_btree_node_write(b);

	atomic_long_add(set_blocks(i, block_bytes(b->c)) * b->c->sb.block_size,
			&PTR_CACHE(b->c, &b->key, 0)->btree_sectors_written);

	b->written += set_blocks(i, block_bytes(b->c));
}

void bch_btree_node_write(struct btree *b, struct closure *parent)
{
	unsigned nsets = b->keys.nsets;

	lockdep_assert_held(&b->lock);

	__bch_btree_node_write(b, parent);

	/*
	 * do verify if there was more than one set initially (i.e. we did a
	 * sort) and we sorted down to a single set:
	 */
	if (nsets && !b->keys.nsets)
		bch_btree_verify(b);

	bch_btree_init_next(b);
}

static void bch_btree_node_write_sync(struct btree *b)
{
	struct closure cl;

	closure_init_stack(&cl);

	mutex_lock(&b->write_lock);
	bch_btree_node_write(b, &cl);
	mutex_unlock(&b->write_lock);

	closure_sync(&cl);
}

static void btree_node_write_work(struct work_struct *w)
{
	struct btree *b = container_of(to_delayed_work(w), struct btree, work);

	mutex_lock(&b->write_lock);
	if (btree_node_dirty(b))
		__bch_btree_node_write(b, NULL);
	mutex_unlock(&b->write_lock);
}

static void bch_btree_leaf_dirty(struct btree *b, atomic_t *journal_ref)
{
	struct bset *i = btree_bset_last(b);
	struct btree_write *w = btree_current_write(b);

	lockdep_assert_held(&b->write_lock);

	BUG_ON(!b->written);
	BUG_ON(!i->keys);

	if (!btree_node_dirty(b))
		schedule_delayed_work(&b->work, 30 * HZ);

	set_btree_node_dirty(b);

	if (journal_ref) {
		if (w->journal &&
		    journal_pin_cmp(b->c, w->journal, journal_ref)) {
			atomic_dec_bug(w->journal);
			w->journal = NULL;
		}

		if (!w->journal) {
			w->journal = journal_ref;
			atomic_inc(w->journal);
		}
	}

	/* Force write if set is too big */
	if (set_bytes(i) > PAGE_SIZE - 48 &&
	    !current->bio_list)
		bch_btree_node_write(b, NULL);
}

/*
 * Btree in memory cache - allocation/freeing
 * mca -> memory cache
 */

#define mca_reserve(c)	(((c->root && c->root->level)		\
			  ? c->root->level : 1) * 8 + 16)
#define mca_can_free(c)						\
	max_t(int, 0, c->btree_cache_used - mca_reserve(c))

static void mca_data_free(struct btree *b)
{
	BUG_ON(b->io_mutex.count != 1);

	bch_btree_keys_free(&b->keys);

	b->c->btree_cache_used--;
	list_move(&b->list, &b->c->btree_cache_freed);
}

static void mca_bucket_free(struct btree *b)
{
	BUG_ON(btree_node_dirty(b));

	b->key.ptr[0] = 0;
	hlist_del_init_rcu(&b->hash);
	list_move(&b->list, &b->c->btree_cache_freeable);
}

static unsigned btree_order(struct bkey *k)
{
	return ilog2(KEY_SIZE(k) / PAGE_SECTORS ?: 1);
}

static void mca_data_alloc(struct btree *b, struct bkey *k, gfp_t gfp)
{
	if (!bch_btree_keys_alloc(&b->keys,
				  max_t(unsigned,
					ilog2(b->c->btree_pages),
					btree_order(k)),
				  gfp)) {
		b->c->btree_cache_used++;
		list_move(&b->list, &b->c->btree_cache);
	} else {
		list_move(&b->list, &b->c->btree_cache_freed);
	}
}

static struct btree *mca_bucket_alloc(struct cache_set *c,
				      struct bkey *k, gfp_t gfp)
{
	struct btree *b = kzalloc(sizeof(struct btree), gfp);
	if (!b)
		return NULL;

	init_rwsem(&b->lock);
	lockdep_set_novalidate_class(&b->lock);
	mutex_init(&b->write_lock);
	lockdep_set_novalidate_class(&b->write_lock);
	INIT_LIST_HEAD(&b->list);
	INIT_DELAYED_WORK(&b->work, btree_node_write_work);
	b->c = c;
	sema_init(&b->io_mutex, 1);

	mca_data_alloc(b, k, gfp);
	return b;
}

static int mca_reap(struct btree *b, unsigned min_order, bool flush)
{
	struct closure cl;

	closure_init_stack(&cl);
	lockdep_assert_held(&b->c->bucket_lock);

	if (!down_write_trylock(&b->lock))
		return -ENOMEM;

	BUG_ON(btree_node_dirty(b) && !b->keys.set[0].data);

	if (b->keys.page_order < min_order)
		goto out_unlock;

	if (!flush) {
		if (btree_node_dirty(b))
			goto out_unlock;

		if (down_trylock(&b->io_mutex))
			goto out_unlock;
		up(&b->io_mutex);
	}

	mutex_lock(&b->write_lock);
	if (btree_node_dirty(b))
		__bch_btree_node_write(b, &cl);
	mutex_unlock(&b->write_lock);

	closure_sync(&cl);

	/* wait for any in flight btree write */
	down(&b->io_mutex);
	up(&b->io_mutex);

	return 0;
out_unlock:
	rw_unlock(true, b);
	return -ENOMEM;
}

static unsigned long bch_mca_scan(struct shrinker *shrink,
				  struct shrink_control *sc)
{
	struct cache_set *c = container_of(shrink, struct cache_set, shrink);
	struct btree *b, *t;
	unsigned long i, nr = sc->nr_to_scan;
	unsigned long freed = 0;

	if (c->shrinker_disabled)
		return SHRINK_STOP;

	if (c->btree_cache_alloc_lock)
		return SHRINK_STOP;

	/* Return -1 if we can't do anything right now */
	if (sc->gfp_mask & __GFP_IO)
		mutex_lock(&c->bucket_lock);
	else if (!mutex_trylock(&c->bucket_lock))
		return -1;

	/*
	 * It's _really_ critical that we don't free too many btree nodes - we
	 * have to always leave ourselves a reserve. The reserve is how we
	 * guarantee that allocating memory for a new btree node can always
	 * succeed, so that inserting keys into the btree can always succeed and
	 * IO can always make forward progress:
	 */
	nr /= c->btree_pages;
	if (nr == 0)
		nr = 1;
	nr = min_t(unsigned long, nr, mca_can_free(c));

	i = 0;
	list_for_each_entry_safe(b, t, &c->btree_cache_freeable, list) {
		if (freed >= nr)
			break;

		if (++i > 3 &&
		    !mca_reap(b, 0, false)) {
			mca_data_free(b);
			rw_unlock(true, b);
			freed++;
		}
	}

	for (i = 0; (nr--) && i < c->btree_cache_used; i++) {
		if (list_empty(&c->btree_cache))
			goto out;

		b = list_first_entry(&c->btree_cache, struct btree, list);
		list_rotate_left(&c->btree_cache);

		if (!b->accessed &&
		    !mca_reap(b, 0, false)) {
			mca_bucket_free(b);
			mca_data_free(b);
			rw_unlock(true, b);
			freed++;
		} else
			b->accessed = 0;
	}
out:
	mutex_unlock(&c->bucket_lock);
	return freed;
}

static unsigned long bch_mca_count(struct shrinker *shrink,
				   struct shrink_control *sc)
{
	struct cache_set *c = container_of(shrink, struct cache_set, shrink);

	if (c->shrinker_disabled)
		return 0;

	if (c->btree_cache_alloc_lock)
		return 0;

	return mca_can_free(c) * c->btree_pages;
}

void bch_btree_cache_free(struct cache_set *c)
{
	struct btree *b;
	struct closure cl;
	closure_init_stack(&cl);

	if (c->shrink.list.next)
		unregister_shrinker(&c->shrink);

	mutex_lock(&c->bucket_lock);

#ifdef CONFIG_BCACHE_DEBUG
	if (c->verify_data)
		list_move(&c->verify_data->list, &c->btree_cache);

	free_pages((unsigned long) c->verify_ondisk, ilog2(bucket_pages(c)));
#endif

	list_splice(&c->btree_cache_freeable,
		    &c->btree_cache);

	while (!list_empty(&c->btree_cache)) {
		b = list_first_entry(&c->btree_cache, struct btree, list);

		if (btree_node_dirty(b))
			btree_complete_write(b, btree_current_write(b));
		clear_bit(BTREE_NODE_dirty, &b->flags);

		mca_data_free(b);
	}

	while (!list_empty(&c->btree_cache_freed)) {
		b = list_first_entry(&c->btree_cache_freed,
				     struct btree, list);
		list_del(&b->list);
		cancel_delayed_work_sync(&b->work);
		kfree(b);
	}

	mutex_unlock(&c->bucket_lock);
}

int bch_btree_cache_alloc(struct cache_set *c)
{
	unsigned i;

	for (i = 0; i < mca_reserve(c); i++)
		if (!mca_bucket_alloc(c, &ZERO_KEY, GFP_KERNEL))
			return -ENOMEM;

	list_splice_init(&c->btree_cache,
			 &c->btree_cache_freeable);

#ifdef CONFIG_BCACHE_DEBUG
	mutex_init(&c->verify_lock);

	c->verify_ondisk = (void *)
		__get_free_pages(GFP_KERNEL, ilog2(bucket_pages(c)));

	c->verify_data = mca_bucket_alloc(c, &ZERO_KEY, GFP_KERNEL);

	if (c->verify_data &&
	    c->verify_data->keys.set->data)
		list_del_init(&c->verify_data->list);
	else
		c->verify_data = NULL;
#endif

	c->shrink.count_objects = bch_mca_count;
	c->shrink.scan_objects = bch_mca_scan;
	c->shrink.seeks = 4;
	c->shrink.batch = c->btree_pages * 2;

	if (register_shrinker(&c->shrink))
		pr_warn("bcache: %s: could not register shrinker",
				__func__);

	return 0;
}

/* Btree in memory cache - hash table */

static struct hlist_head *mca_hash(struct cache_set *c, struct bkey *k)
{
	return &c->bucket_hash[hash_32(PTR_HASH(c, k), BUCKET_HASH_BITS)];
}

static struct btree *mca_find(struct cache_set *c, struct bkey *k)
{
	struct btree *b;

	rcu_read_lock();
	hlist_for_each_entry_rcu(b, mca_hash(c, k), hash)
		if (PTR_HASH(c, &b->key) == PTR_HASH(c, k))
			goto out;
	b = NULL;
out:
	rcu_read_unlock();
	return b;
}

static int mca_cannibalize_lock(struct cache_set *c, struct btree_op *op)
{
	struct task_struct *old;

	old = cmpxchg(&c->btree_cache_alloc_lock, NULL, current);
	if (old && old != current) {
		if (op)
			prepare_to_wait(&c->btree_cache_wait, &op->wait,
					TASK_UNINTERRUPTIBLE);
		return -EINTR;
	}

	return 0;
}

static struct btree *mca_cannibalize(struct cache_set *c, struct btree_op *op,
				     struct bkey *k)
{
	struct btree *b;

	trace_bcache_btree_cache_cannibalize(c);

	if (mca_cannibalize_lock(c, op))
		return ERR_PTR(-EINTR);

	list_for_each_entry_reverse(b, &c->btree_cache, list)
		if (!mca_reap(b, btree_order(k), false))
			return b;

	list_for_each_entry_reverse(b, &c->btree_cache, list)
		if (!mca_reap(b, btree_order(k), true))
			return b;

	WARN(1, "btree cache cannibalize failed\n");
	return ERR_PTR(-ENOMEM);
}

/*
 * We can only have one thread cannibalizing other cached btree nodes at a time,
 * or we'll deadlock. We use an open coded mutex to ensure that, which a
 * cannibalize_bucket() will take. This means every time we unlock the root of
 * the btree, we need to release this lock if we have it held.
 */
static void bch_cannibalize_unlock(struct cache_set *c)
{
	if (c->btree_cache_alloc_lock == current) {
		c->btree_cache_alloc_lock = NULL;
		wake_up(&c->btree_cache_wait);
	}
}

static struct btree *mca_alloc(struct cache_set *c, struct btree_op *op,
			       struct bkey *k, int level)
{
	struct btree *b;

	BUG_ON(current->bio_list);

	lockdep_assert_held(&c->bucket_lock);

	if (mca_find(c, k))
		return NULL;

	/* btree_free() doesn't free memory; it sticks the node on the end of
	 * the list. Check if there's any freed nodes there:
	 */
	list_for_each_entry(b, &c->btree_cache_freeable, list)
		if (!mca_reap(b, btree_order(k), false))
			goto out;

	/* We never free struct btree itself, just the memory that holds the on
	 * disk node. Check the freed list before allocating a new one:
	 */
	list_for_each_entry(b, &c->btree_cache_freed, list)
		if (!mca_reap(b, 0, false)) {
			mca_data_alloc(b, k, __GFP_NOWARN|GFP_NOIO);
			if (!b->keys.set[0].data)
				goto err;
			else
				goto out;
		}

	b = mca_bucket_alloc(c, k, __GFP_NOWARN|GFP_NOIO);
	if (!b)
		goto err;

	BUG_ON(!down_write_trylock(&b->lock));
	if (!b->keys.set->data)
		goto err;
out:
	BUG_ON(b->io_mutex.count != 1);

	bkey_copy(&b->key, k);
	list_move(&b->list, &c->btree_cache);
	hlist_del_init_rcu(&b->hash);
	hlist_add_head_rcu(&b->hash, mca_hash(c, k));

	lock_set_subclass(&b->lock.dep_map, level + 1, _THIS_IP_);
	b->parent	= (void *) ~0UL;
	b->flags	= 0;
	b->written	= 0;
	b->level	= level;

	if (!b->level)
		bch_btree_keys_init(&b->keys, &bch_extent_keys_ops,
				    &b->c->expensive_debug_checks);
	else
		bch_btree_keys_init(&b->keys, &bch_btree_keys_ops,
				    &b->c->expensive_debug_checks);

	return b;
err:
	if (b)
		rw_unlock(true, b);

	b = mca_cannibalize(c, op, k);
	if (!IS_ERR(b))
		goto out;

	return b;
}

/**
 * bch_btree_node_get - find a btree node in the cache and lock it, reading it
 * in from disk if necessary.
 *
 * If IO is necessary and running under generic_make_request, returns -EAGAIN.
 *
 * The btree node will have either a read or a write lock held, depending on
 * level and op->lock.
 */
struct btree *bch_btree_node_get(struct cache_set *c, struct btree_op *op,
				 struct bkey *k, int level, bool write,
				 struct btree *parent)
{
	int i = 0;
	struct btree *b;

	BUG_ON(level < 0);
retry:
	b = mca_find(c, k);

	if (!b) {
		if (current->bio_list)
			return ERR_PTR(-EAGAIN);

		mutex_lock(&c->bucket_lock);
		b = mca_alloc(c, op, k, level);
		mutex_unlock(&c->bucket_lock);

		if (!b)
			goto retry;
		if (IS_ERR(b))
			return b;

		bch_btree_node_read(b);

		if (!write)
			downgrade_write(&b->lock);
	} else {
		rw_lock(write, b, level);
		if (PTR_HASH(c, &b->key) != PTR_HASH(c, k)) {
			rw_unlock(write, b);
			goto retry;
		}
		BUG_ON(b->level != level);
	}

	b->parent = parent;
	b->accessed = 1;

	for (; i <= b->keys.nsets && b->keys.set[i].size; i++) {
		prefetch(b->keys.set[i].tree);
		prefetch(b->keys.set[i].data);
	}

	for (; i <= b->keys.nsets; i++)
		prefetch(b->keys.set[i].data);

	if (btree_node_io_error(b)) {
		rw_unlock(write, b);
		return ERR_PTR(-EIO);
	}

	BUG_ON(!b->written);

	return b;
}

static void btree_node_prefetch(struct btree *parent, struct bkey *k)
{
	struct btree *b;

	mutex_lock(&parent->c->bucket_lock);
	b = mca_alloc(parent->c, NULL, k, parent->level - 1);
	mutex_unlock(&parent->c->bucket_lock);

	if (!IS_ERR_OR_NULL(b)) {
		b->parent = parent;
		bch_btree_node_read(b);
		rw_unlock(true, b);
	}
}

/* Btree alloc */

static void btree_node_free(struct btree *b)
{
	trace_bcache_btree_node_free(b);

	BUG_ON(b == b->c->root);

	mutex_lock(&b->write_lock);

	if (btree_node_dirty(b))
		btree_complete_write(b, btree_current_write(b));
	clear_bit(BTREE_NODE_dirty, &b->flags);

	mutex_unlock(&b->write_lock);

	cancel_delayed_work(&b->work);

	mutex_lock(&b->c->bucket_lock);
	bch_bucket_free(b->c, &b->key);
	mca_bucket_free(b);
	mutex_unlock(&b->c->bucket_lock);
}

struct btree *__bch_btree_node_alloc(struct cache_set *c, struct btree_op *op,
				     int level, bool wait,
				     struct btree *parent)
{
	BKEY_PADDED(key) k;
	struct btree *b = ERR_PTR(-EAGAIN);

	mutex_lock(&c->bucket_lock);
retry:
	if (__bch_bucket_alloc_set(c, RESERVE_BTREE, &k.key, 1, wait))
		goto err;

	bkey_put(c, &k.key);
	SET_KEY_SIZE(&k.key, c->btree_pages * PAGE_SECTORS);

	b = mca_alloc(c, op, &k.key, level);
	if (IS_ERR(b))
		goto err_free;

	if (!b) {
		cache_bug(c,
			"Tried to allocate bucket that was in btree cache");
		goto retry;
	}

	b->accessed = 1;
	b->parent = parent;
	bch_bset_init_next(&b->keys, b->keys.set->data, bset_magic(&b->c->sb));

	mutex_unlock(&c->bucket_lock);

	trace_bcache_btree_node_alloc(b);
	return b;
err_free:
	bch_bucket_free(c, &k.key);
err:
	mutex_unlock(&c->bucket_lock);

	trace_bcache_btree_node_alloc_fail(c);
	return b;
}

static struct btree *bch_btree_node_alloc(struct cache_set *c,
					  struct btree_op *op, int level,
					  struct btree *parent)
{
	return __bch_btree_node_alloc(c, op, level, op != NULL, parent);
}

static struct btree *btree_node_alloc_replacement(struct btree *b,
						  struct btree_op *op)
{
	struct btree *n = bch_btree_node_alloc(b->c, op, b->level, b->parent);
	if (!IS_ERR_OR_NULL(n)) {
		mutex_lock(&n->write_lock);
		bch_btree_sort_into(&b->keys, &n->keys, &b->c->sort);
		bkey_copy_key(&n->key, &b->key);
		mutex_unlock(&n->write_lock);
	}

	return n;
}

static void make_btree_freeing_key(struct btree *b, struct bkey *k)
{
	unsigned i;

	mutex_lock(&b->c->bucket_lock);

	atomic_inc(&b->c->prio_blocked);

	bkey_copy(k, &b->key);
	bkey_copy_key(k, &ZERO_KEY);

	for (i = 0; i < KEY_PTRS(k); i++)
		SET_PTR_GEN(k, i,
			    bch_inc_gen(PTR_CACHE(b->c, &b->key, i),
					PTR_BUCKET(b->c, &b->key, i)));

	mutex_unlock(&b->c->bucket_lock);
}

static int btree_check_reserve(struct btree *b, struct btree_op *op)
{
	struct cache_set *c = b->c;
	struct cache *ca;
	unsigned i, reserve = (c->root->level - b->level) * 2 + 1;

	mutex_lock(&c->bucket_lock);

	for_each_cache(ca, c, i)
		if (fifo_used(&ca->free[RESERVE_BTREE]) < reserve) {
			if (op)
				prepare_to_wait(&c->btree_cache_wait, &op->wait,
						TASK_UNINTERRUPTIBLE);
			mutex_unlock(&c->bucket_lock);
			return -EINTR;
		}

	mutex_unlock(&c->bucket_lock);

	return mca_cannibalize_lock(b->c, op);
}

/* Garbage collection */

static uint8_t __bch_btree_mark_key(struct cache_set *c, int level,
				    struct bkey *k)
{
	uint8_t stale = 0;
	unsigned i;
	struct bucket *g;

	/*
	 * ptr_invalid() can't return true for the keys that mark btree nodes as
	 * freed, but since ptr_bad() returns true we'll never actually use them
	 * for anything and thus we don't want mark their pointers here
	 */
	if (!bkey_cmp(k, &ZERO_KEY))
		return stale;

	for (i = 0; i < KEY_PTRS(k); i++) {
		if (!ptr_available(c, k, i))
			continue;

		g = PTR_BUCKET(c, k, i);

		if (gen_after(g->last_gc, PTR_GEN(k, i)))
			g->last_gc = PTR_GEN(k, i);

		if (ptr_stale(c, k, i)) {
			stale = max(stale, ptr_stale(c, k, i));
			continue;
		}

		cache_bug_on(GC_MARK(g) &&
			     (GC_MARK(g) == GC_MARK_METADATA) != (level != 0),
			     c, "inconsistent ptrs: mark = %llu, level = %i",
			     GC_MARK(g), level);

		if (level)
			SET_GC_MARK(g, GC_MARK_METADATA);
		else if (KEY_DIRTY(k))
			SET_GC_MARK(g, GC_MARK_DIRTY);
		else if (!GC_MARK(g))
			SET_GC_MARK(g, GC_MARK_RECLAIMABLE);

		/* guard against overflow */
		SET_GC_SECTORS_USED(g, min_t(unsigned,
					     GC_SECTORS_USED(g) + KEY_SIZE(k),
					     MAX_GC_SECTORS_USED));

		BUG_ON(!GC_SECTORS_USED(g));
	}

	return stale;
}

#define btree_mark_key(b, k)	__bch_btree_mark_key(b->c, b->level, k)

void bch_initial_mark_key(struct cache_set *c, int level, struct bkey *k)
{
	unsigned i;

	for (i = 0; i < KEY_PTRS(k); i++)
		if (ptr_available(c, k, i) &&
		    !ptr_stale(c, k, i)) {
			struct bucket *b = PTR_BUCKET(c, k, i);

			b->gen = PTR_GEN(k, i);

			if (level && bkey_cmp(k, &ZERO_KEY))
				b->prio = BTREE_PRIO;
			else if (!level && b->prio == BTREE_PRIO)
				b->prio = INITIAL_PRIO;
		}

	__bch_btree_mark_key(c, level, k);
}

static bool btree_gc_mark_node(struct btree *b, struct gc_stat *gc)
{
	uint8_t stale = 0;
	unsigned keys = 0, good_keys = 0;
	struct bkey *k;
	struct btree_iter iter;
	struct bset_tree *t;

	gc->nodes++;

	for_each_key_filter(&b->keys, k, &iter, bch_ptr_invalid) {
		stale = max(stale, btree_mark_key(b, k));
		keys++;

		if (bch_ptr_bad(&b->keys, k))
			continue;

		gc->key_bytes += bkey_u64s(k);
		gc->nkeys++;
		good_keys++;

		gc->data += KEY_SIZE(k);
	}

	for (t = b->keys.set; t <= &b->keys.set[b->keys.nsets]; t++)
		btree_bug_on(t->size &&
			     bset_written(&b->keys, t) &&
			     bkey_cmp(&b->key, &t->end) < 0,
			     b, "found short btree key in gc");

	if (b->c->gc_always_rewrite)
		return true;

	if (stale > 10)
		return true;

	if ((keys - good_keys) * 2 > keys)
		return true;

	return false;
}

#define GC_MERGE_NODES	4U

struct gc_merge_info {
	struct btree	*b;
	unsigned	keys;
};

static int bch_btree_insert_node(struct btree *, struct btree_op *,
				 struct keylist *, atomic_t *, struct bkey *);

static int btree_gc_coalesce(struct btree *b, struct btree_op *op,
			     struct gc_stat *gc, struct gc_merge_info *r)
{
	unsigned i, nodes = 0, keys = 0, blocks;
	struct btree *new_nodes[GC_MERGE_NODES];
	struct keylist keylist;
	struct closure cl;
	struct bkey *k;

	bch_keylist_init(&keylist);

	if (btree_check_reserve(b, NULL))
		return 0;

	memset(new_nodes, 0, sizeof(new_nodes));
	closure_init_stack(&cl);

	while (nodes < GC_MERGE_NODES && !IS_ERR_OR_NULL(r[nodes].b))
		keys += r[nodes++].keys;

	blocks = btree_default_blocks(b->c) * 2 / 3;

	if (nodes < 2 ||
	    __set_blocks(b->keys.set[0].data, keys,
			 block_bytes(b->c)) > blocks * (nodes - 1))
		return 0;

	for (i = 0; i < nodes; i++) {
		new_nodes[i] = btree_node_alloc_replacement(r[i].b, NULL);
		if (IS_ERR_OR_NULL(new_nodes[i]))
			goto out_nocoalesce;
	}

	/*
	 * We have to check the reserve here, after we've allocated our new
	 * nodes, to make sure the insert below will succeed - we also check
	 * before as an optimization to potentially avoid a bunch of expensive
	 * allocs/sorts
	 */
	if (btree_check_reserve(b, NULL))
		goto out_nocoalesce;

	for (i = 0; i < nodes; i++)
		mutex_lock(&new_nodes[i]->write_lock);

	for (i = nodes - 1; i > 0; --i) {
		struct bset *n1 = btree_bset_first(new_nodes[i]);
		struct bset *n2 = btree_bset_first(new_nodes[i - 1]);
		struct bkey *k, *last = NULL;

		keys = 0;

		if (i > 1) {
			for (k = n2->start;
			     k < bset_bkey_last(n2);
			     k = bkey_next(k)) {
				if (__set_blocks(n1, n1->keys + keys +
						 bkey_u64s(k),
						 block_bytes(b->c)) > blocks)
					break;

				last = k;
				keys += bkey_u64s(k);
			}
		} else {
			/*
			 * Last node we're not getting rid of - we're getting
			 * rid of the node at r[0]. Have to try and fit all of
			 * the rem_MA_mark_key(b, k));
		keys++;

ntart;
	_key(<h	reevk0o)P<hWe ha				\
})

/**
 * btree_root - ~ just used as
 <y:	key to recurse on
 * @b:		pa			 bloo recurse on
 * yh will be passed the child nod~* @b:		pa			 bl<hIoy<v, stnod~* @b:		pa			yct btree_op
 */
#define btree_~* have to alwayqL#define bt<Sprhf<Sprhf<Sprhf<Splow */
		SET_GC_SECTORS_USED(5>plow */
<ytree *_b = (c)->root;				\
		be
/**
 * btree_root - ~re *cl)
{
	stru|P_nodes));
-~of the node at @c:		cache set
 * @op:		pointei
	bch_bbio_free(bio, b->c);

	rk_key(b,t<Sprhf<Sprhs, 0, sizeof(new_nodes));
	.Lh,.Lk&n->keyssed the chiP<wed i,nfch_btree_node_get((b)->c, op, E_PRS;
}

s	pa			 bloo |Le_root - ~re *
s	pa			 bloo |Lygic(&b->c->sb))
			goto err;

ey(s 
ey(s 
ey(s 
ey *gc)
{
	uint8_t stale = 0;
	
#defiroot;				\
	fch_btrqCSprhs,ree_node_get(b;
	unn:		cache set
.Ly bset *write_block(struct btre~_node_get(b;
	u<hck(struc<mE&newode_get(b;
	u<hchinc_gen)
{
	stru|P_|Cu.Lmb->key stale = 0;
ed	y<T/**
 * btree_~ub->key stal no, E#define btree_~T/**
 * btree_~ub->s	 bloo |Le_root - ~re  -	e *cl)stry anfouni0rse on
 * @b:		parent btre81_"s(c, t) &&
			    efine btree_~T/**
 ;(list_empty(&c->bt = nodes - 1; i > 0; --ifreeing_ktruct bset *n1 = btree_bset_wn(&b->io_mutex);
	up(&b	 *cachihe numbtree_rootde ateys, "writing");

	cancektruct bse0])keys(&b->krite_lock);

	ifktruct bse0]);h_mca_scan(struct ktruct bse0]);h_ktruct bse0]    k < bseeplacement(r[i].b, NULL);
		if (y stale = 0;
ed	y<T/**
 * btree_~ubtal no, E#decoalesT/**
 * btree_~ub->s	Le_root - ~re  ed i;

	mutex_lock(&b->coalescetree_~u.t_key(t btre81_"s(c,cks(iet(new_node->keysct keylist *, atomic_b{
		cachree_~ubtnt(str;
	}

	/ch(structtre81_"s(c,cache_ft(new_no bch_inc_gen(PTR_CACHb, NULL);
		if (rite_lock);

	ifcoalesee_node_free(struct bcoalesee_t;				\
l &&ktruct bset de->keynode_getr bcwritte_MERGE_;
		)_nodes[i] = btr;h_m[es[i] = b\
l &&, list)
		if (!mca_t btree *bch_btree_gc_stat *gc,S && !;invalid) {
d mc
		return 0;

&b->c-t(new_nodes,t */nDELAYED_ we al));
__clde atcannibalize_loc
->s	 bloo |Le_root - :t_empty(&c->bt = nodes - 1; i > 0; --ifreeing_ktruct bset *n1 = btree_bset->s	Le_root - :t_wn(&b->io_mutex);
			return 0;

&b->c-t(new_nodes,eys += next(btre81_"s(c,cop_ft(new_no OWARN|GFPvel && b->prio == BTREE_PRIjournal) { &ZERO_KEY);

	for (i = 0empty(&c->bt = nodes - 1; i > ee_sort_into(&b->keys,.Lk&n->keyss
			 *rite_lock);

	ifktruct bseruct bk_mca_scan(struct ktruct bseruct bk}set *c, struct btree_o   struct gc_eys - g	unsigned keys = 0, good_keys ruct btree *parent)
{
	int  = 0, 	goto oreserve = (cst_init(&keist;
	struct closurdes, 0, sizeof(new_nodes));
	closure_init_stack(&cl)nLL(new_nodes[i]))
			goto out_nocgoto otr;
	}

	s,t *reto makre the ibelow e_freed, li	goto out_n_rootde at 0, sizeof(new_nodes));
	closure_	if (rite_lock);

	ifnee_node_free(struct bn}

static vo0de->keysct keylisinit_stack(&cl);_btree_return 0;

	memset(ns;
			return 0;

 t) &&
	utex_unlocreservd i;

	mutex_lock(&b->cgoto otr- 1))t_key(tbtre81_"s(c,cks(iet(nsbtree_retkeylist *, atomic_b{
		cachresbtnt(str;
	}

	/ch(structtre81_"s(c,cache_ft(nnc_gen)rite_lock);

	ifcgoto or;h_mca_scan(struct kodes,t */nDELAYED_ we al));
__clde atcannibalize_locY_SIZE(k) / PAGE_SECTORS gc_stubtree_k;

	mutex_lock(&b->write_loces++;

	for_each_key_filter(&b->key/ PAGE_SEcany_last(ne = max(stale, btree_mark_key(b, k));
		keys++;
baurn mca_ getting rid of - atcannibacanruct btree_o   struct gc_eyee_op, struct gc_merge_info *r)
{
	unsigned i, nodedep_assert_held(;
	SETood_keys = 0;
	struct bkei->io, &k.key,int8_he chi_eys - g

	gc->nodes++;

	for_each_key_filter(&b->keys, k, &ruct btree *nertruct bkey *k;

	bch_keyliruct btree *newit;
			    cwriARRAsets]; r)s[i])ree_retkeylister sg_checks);

	rete key";
	if (ructruci = 0empty(&c-rbt = ncwriARRAsets]; r)- 1; i > eb->wri, list)
		if (!mca_eys +=    k = ext(btrekeylister s_u64e, btree_e key";
	ik_key(beys++;
baur
	/*
	 *k!= levelb->wri struct bkey *k, inx_lock(&n-kt level, ss[i]e_alloc(b-_unlock(&c->bied to allolb->k),
					o, &k.io =allolb->k we're& !i->key:		poielb-	fch_btCTORS gc_stubtree_k;lb->k w
			o, &k.btree_gc_stat *gc,b{
		cablocr(&c->bied o, )we're& !i->keyc(w->journ			 b->k
	_data_free(b);
	- find a			 b->kEN(k, i)e chi_eys - g&k.btree_gc_ 0;
	unsig			 b->cabl(&c->bied )e chi_eys - g),
					o, &k.truct gc_eys - g	unsigb{
		ca			 b->k;						 blo, )we'ree& !i->key:		poiee chiP<w set
a, c, i
					o, &k.truct gc_eyee_op,			 b->ca		ca;
	SEToobl(&c->b		 blo, )we'ree& !i->key:		poienfch_btree_node
	if (ructrucy";iP<w set
locreser * rid of tMist wn_trode writes Fve
	 * gc/
	lTood anytcgoto oid of top;
__ion FUA to kytes(i)led		pa			yct 0; --i) {
		iP<w set
t);
		bkey_copy_write(b));
	clear_bit			 b->kEwe're&	e *cl)stry anfouni0			 b->ca;
	SETx_unlock(&c->bucket_iP<w set
t);
		bkey_copy_mca_scan(struct 			 b->k;			}ree_node_getcwritterte_MERGE_;
		)_nod+].keys;

	bloc= btr;h_elb->wri k < bset bloceei_eyb);

( != level, &k._alloc_;
	re& !i->keyc(list_empty(&c-rbt = ncwriARRAsets]; r)- 1; i > ee_sort_into(&b->keyib->kEN(k, i0; --i) {
		i set
t);
		bkey_copy_write(b));
	clear_bitib->kEwe're&	e *cl)stry anfouni0ib->ca;
	SETx_unlock(&c->bucket_i set
t);
		bkey_copy_mca_scan(struct ib->k;			}reecannibacanruct btree_o   st	e *cl)stgc_eoo];
	void *data = (void *) i
{
	unsigned i, nodes = 0, kert_held(;
	SETood_keys = 0;
	struct bkeent);
	if (!IS_ERR*mca_ali->io, &k.key,int8_he chi_eys - g


i)e chi_eys - g&k.btree_gc_ 0;
	unsig>cabl(&c-ied )e chi_eys - g),
			nLL(new_nodes[i]))
			goto out_nobtr;
	}

	s,ree_sort_into(&b->keys, &n->kysct keylisinit_stack(&cl);_btreepy_key(&n->keet_eoo];n}

st)rite_lock);

	if0;
	}
out:
	mutex_unlocnk w
			o, nibalize_lock(b-ark_node(struct btree *b, , int level, swritte&et
locresereturn ta, c, i
			o, &k.truct gc_eyee_op,>ca		ca;
	SEToobl(&c->	 blo, )we'rcannibacanru->keysfch_btree_node
	if (ructrucy";et
locresercannibacanruct btree_ofree(b);

	= 0;
	rint level, bool writereserve = (c->roo

	mute);

			if (level;able(c, k, i) &&ee_sof (ructree 	if (b
static v(fifo_used(&ca->free[RESERVE_BTREE]f (ructree 	if (&k.key,f (ructruc&k.o == BTRREE]) < reserve) {
			if (op)
	) < reser* the b>caca &n->kys= max(stale,				b-copy_writup_allocid bt;et
pis, &n->kyw */
		SET_GCbio_erro	D(g) + KEY_SIZE(k),
		bio_erro	D}
k(b->c, op);
}

/* Garbage collection *b,
						_bch st	e *cl)stgc_fg_c[hash_32(PTR_HASH(c, kreserv_bch set *b = P *t;

	gc->nodef (level;abel) * 2 + 1;

	mutex_lock(&c-(fifo_used(&ca->free[RESERVE_BTREE]eet_= 0;;
}

vct cacf (ructree 	if (&k.1cacf (ceei_gc	=lloc_replacement(r[i].e(c, k, i>freuuidr* the ->key, i),
					SET_GC bkey_cmp(k, &>freuuidr* the bucket_loETADATA);
	RTY);
		elses,t *KEY))
eyelaimdef (lee marg a new
	SEbamak	fch_
	forde atc)
		if (PTR_HASH(eplacement(r[i].ed)
		uuidULL);
		if (gc->nodeTR_HASdevinyt*dwarn("devinyset de-bel) * 2 + 1;dSdevt*dcde-bel) * 2	fcbuf_s++;
w,surde	ex_lock(&cj
	s,ree_sod || UUID_FL strONLYi>freuuidRS_USED(5>= GC_MARK_MEdnker_disabled)
		d 0;

	if (c->bdSdev,nerick w
		surna(&ca->dct
t);
	bamaree_k.bkey_copyr);
	if ostself, (freed >= nr)
			brew,sne_alloc>dct
t);
	bamaree_k.k_key(ock)ED(5>eplacjment(rji].e(c, k, i>wt
locre jy, i),),
					SET_GC bkey_cmp(k, &>w op, E_j);
	}

	retu_RECLAIMABLE);

		surna
}

/* Gdct
t);
	bamaree_k.bkey_cop}k(struct cache_set *cE]) < reserve) {
			if (op	if (bio *bio;*-(fif	ait(inDELAYED__ll bestale,ast(n2eplacemenait(sb.d(r[i].eit(sb.dwrieit(sb.&keiskey, i),,
					SET_GCeit(ef (lee +ewit;ATA);
	RTY);
		elses,2eplacemenait(EY);

f (lee

		_PADDEi].eit(EY);

f (lee +eEY);

f (lee(ca &coulkey, i),,
					SET_GCeit(ef (lee +ewit;ATA);
	RTY);
		elses,2epl reser* the b>caca &n->kyf (ceei_gc	=l->kef (ceei_gc,def (le_= 0)));lloc(b),ree_sp_allocid bt;et
pis, 
	}

_keys++;

		g_mark_key(b,ET_GCbache) KEY_SIZE(k),
		bloc(b),ree_sy(b,ET_GCbac|| (b,ET_GCbacrs: mark = ED(g, min_t(
	}

et *b = P{
		if (list_long bch_mca_count(struct shrinker *shret *b = Pjournal_ref)
{
	struct bsetgve(c); i++)
		if (!mca_buci->io, ;HRINK_STOP;

	ifet *b = Pjo	d_keys = 0;
	st	bch_;);

	if (btree_ch;
	SET	for_each_key_finsiop;
(bio *bio;reading bu lisocal	w->set *cE]t btree *bch_= 0;
	rin
	list;

	whi&	bch_ < GC_MERGE_d_keys = 0;
	s!IS_ERR_OR_NULL(r[nodes].;
	SETx_untruct bsetopsg_chec		caSHRTarkX_gen)rite_l= 0;
	rin
	listdoi
			o, &k.truct eoo];gc_eoo]	if (c		ca&;
	SEToo&	bch_;

		wn(&b->io_mute;
	SETx_unl_kei_eyb);

( 
	s,ree_so, &he)o, &!k._alloc_(
	}


	returntalne thr!_bset} eys += o, )ey_cot *b = P *tt	e *cl)stgc_fg_c[hat cac>journal);
		__closcbtree_retg bu0;
	ssrnaYED_	mutex_unlogc_that,;reading buTREE]e
	ss.&ke += KEY*=C_MERGE_bio *bio);E]e
	ss.rn 0	<<= 9;E]e
	ss.rna
sREQ_ef (cef (lee -fet *b = P &co100 / f (cef (lee;
	pa			 b&f (ruc	bch_ <&	bch_ <_MERGE_d_keys = 0;
	s!IS_E]t btree *bch_= 0e-EAGbtree_rete_gock(gvectat *gc)
{
	uint8_= 0;e chi_eunnt level, bool writereserve = (c->roo

	mutele(c, k, i) &&
		 reserve) {
			if (op)
				pait(inDELAYED__ll besta)we'rcanniban false;
}

p_allocid bt;rite;
}

votosta)_list
ct gc_merge_info {
	struct btree	*btree_o   st	e gc_tll destruc<margreserve = (c->root->level -argmca_eys +=    k = k);
_, cn &b->errupti= PTR (ruck);
			re
			tll de0;e chi_stsp(ac|| = 0;e chi_eunncssed the ch	tll de0;e chi_stsp(ak
	_data_free(beet_= 0;;
}

vct cac	truct bsetgvecode->keyt *c, struct    st	e gc_tll de0;
	rint level, bool writereserR (ruc'll dea=		tll de0eunnt	e gc_tll de	if ("e *bch_= "(&c-ied  find aR (ruc'll dee_init_stackio =alloR (ruc'll dee;keyt *c, struct t */n	unsiructunsirge_btree_node_or (i = 0; i < new_nodee_op, struct gc_merge_info *r)
{
	unsignet bkei->io, &k.key, (k = n2->start;
pERR*mca_alr_each_key_filter(&b->keale = max(stale, btree_mark_key(b, k));
		keys++;

		if (bac	truc{
	unsigned i;

	, int level, structree_ret{
	unsigned i;

	, int level, swritte&et
locresereturn ta, c, i
			_retkeylister sg_checks);

	rete key";
	}

	s,rdoi
			 ext(btrekeylister s_u64e, btree_e key";
	ik_key_alloc(b- , &b->ke++;
baur
	/**
	 *k!we're&ite_lock);rw_unlock(tructreetruct b)					o, &k.truct( new_nodee_op, pw_unl_key(
	}

node we'} eys += pcks =o, )ey	}reecannibacanruct or (i = 0; i < new_osure_init_stack(&cl);

	if (c->shrininsiop;
untruct bsetopsg_chec		caSHRTarkX_gen)he_set *ruct eoo]; new_nodee_op, f (c		)ee *b;
	struct{
	unsiggc_fg_c[hash_32(PTR_HASH(c, kreserve = (c->roo

	mute);

			if (level;able(c, k, i) &&t	e *cl)stgc_fg_c[hat caifo_used(&ca->free[RESERVE_BTREE]ur new
	 *_cannibap foromf we)
{
	ef (lee diodetlyre:
	 */stati

	iinit(i:
	 */_node_ibaget befo);
		__cle'll dea;
	rivoid it*_cansy(b, &cef (lee i:
	 */_node_ibaeys - g&	 */statsrecur)))s	 * juit*_cansyibaeys - g&statsout_nocur)))sa			_node_ibaisk nef (leeTR_OFout_n to ro rtime,		br/
	ifef (lee matings++;no li++;[b->k.
 *
 me we un_for_eac_ch;e chiurse on
beoromf ofTR_OFFS]) < reserve) {
			if (op	if (epl reser* the b>caca &n->ky			preparfull_wait(&c->btree_cacee_g]ort btree kereparfull_wait(&c->btree_cache_wait)we're& !i->kopy_writealloc__inDELAYED__* the b			ibort btree key(b,ET_GCba &n->kywode(stinDELAYED__on__* the b			ibo&c->b		 bl!reparcks(ieait(&c->btree_cacee_g]uct buckebid eit(ef (leet)we're	reparcks(ieait(&c->btree_cache_waiy_alloc(b-bid eit(ef (leetrro	D}
k(b-(b->c, op);
}

/* Garbage collection *bd *mca_hashays mnt level,
				int8_t stalt *, atkb->c->bucket_lock);

	atomic_inc(&d i, nodes = 0, kc_inc(	goto otkb-nodes[GC_MERGEl,
	umb->keys, "wrel && b->priy;
		}
	>e_writel,
	umxt(btrekeylist *, atkb->_mark_key(b, 	goto otkb-n&c-ied ),
	umx!key(c, lINee_T_STATUS_NOlINee_T i
			_ret

	/* If caller isn't %u/
	if%s",El,
	umON(current->	goto otkb- ? "	goto o" :, le*, a0)
		r_t btree *bch_btree_t *, atkb->by(b, 	goto otkb-cement(st
lloc(b- , &l,
	um}

static voge_inf);
	return{
	struct btree	*btree_ov_bch st *, atrid _ha				\
};

	mutex_lock(&b->wr

	ifo, &k.trn b;
err:
	ifrid _ha				\
};->btree_cacheur new
M_lockl* jui:
	 */middlf oflocsexnit, libch_btnocur nodes, ef Ctnninfor (i = 0;s - 1))opst(is	bch_bttruct gcP_|Ce(c,n stU64Sgen)he_set ->kere bu0Ltat *gc)
{
	uint8_btrekeylist *, atkb-k;

	mutex_lock(&ruct btree *parent)
{
	BKEY_Pc_coalesce(struct *, atkb-k
	BKEY_Pc_coalec_inc(	goto otkb-nodesint8_o, &k.ct btreei->i a nkey,k.trn stubtr[b->;->btree_cachery(&c->bttre81_"s(c,cache_t *, atkb-k)		if (gc->node_inc(&,k.t *, atkb-kCSprhs,ree_ = 0;ing rid of 	>et *, atrid _ha				\
};bak
	_data_free(b = 0;ing & b->priy;
		}
	<	   &n->ky			p				    &b->c	_SECTORS);, int uctreetro, &|=_t stalt *, atkb->by(b, 	goto otkb-}

st)rtre81_"s(c,cop_front_t *, atkb-k) we'} RIO;
		}
 in gc");
STARc(c,  justiy;
		}
	<   &n->kylock(&c->bucket_ltcac

st)rfch_btre(&tcace_bug(t *, atkb-kCSprhsbtreepy_keycut_babtree *k (b->ccace_bu}

st)rtrecut_front_btree_chec *, atkb-kCSprhsbtreepyo, &|=_t stalt *, atkb->by(&tcace_bug(	goto otkb-}

st)r !i->keyctting
			 *& !i->keyc(list_			p	o, )we'_loct *, atet *ismnt =rge_info ch(structtre81_"s(c,cache_t *, atkb-k)h_btree if (KEY_Deys, "wrrn stubtr[b->;->btree_c	<  a nkeyrinker *shrcanruct btree_o   struct ef Ct;

	mutex_lock(&ruct btree *parent)
{
	BKrrent->bio_lissce(struct *, atkb-k
	BKrrent->bio_list);

		goto otkb-nodesint8_ef Ct;keent);
	if (!IS_1,t = NULnt(str*n3ERR*mca_albio *bio;reading bu lisocal	w->set *c;

	if (btree_check_reserve(sce(stru
		bch_prhs,ree;

	if (!down_write_tryloc_return 0;

	memse
		bch_prhsresereturnizeof(new_nodes));
	cloree_cach			p				    &b->c	o, nibalize_lock(GFP_NOIOMEM);
}

t *ufficie->io, the i
	ifef Ctead canist_et_first(nedes[i]))
			goto out_nobtronovalidat find an1ree_pages * PAGE_ef Ctn= ocks * (nodeiting");

	cancek1ket_loETAb,t<Sprhf<SprqCS;
			}deiting"* (node &co4e *b5 keys)
		f Ctp	if (bter iter;
	struc
		r_t btree *bch_btree_des[ief Ct;s);

	mut");

	cancek1kCSprhsbtreep= NULLOR_NULL(n)) {
		mutex_lock(&n->write_lock);
		bch_btrlidat find an2* btree_~ubket_lock1
	s,ree_sok);
		bch_
			 *n3ERRLOR_NULL(n)) {
		mutex_lock(&n->write_lwritte;
	}

	/*lidat find an3SED(5>plow *ket_lock2;			}ree_n; --i) {
		s1t
t);
		bkey_copyn; --i) {
		s2*n1 = btree_bset_e_retkeylist *, atkb-k;_1,t_node_*, atkb-k
(	goto otkb-}


 * rid &coHasyibabeoa lineaifeeaichabecaers RO_KEY))
 node(nr uxiliaryid &coeeaichaf (!Iyet
pa			y = ky(&c->;
	st< deiting");

	cancek1kb-	fch_* 3e *b5ED(5>knot getting rid onewode_get(b;
eiting");

	cancek1ket_lo	(5>knotssed thnfch_btree_node_qCSprhON(current-newode_get(b;
eiting");

	cancek1kehchinc_ge5>knot getting rid onewode_get(b;
eiting");

	cancek1kehchinc_gen)
eiting");

	cancek2)b-	fch_btCTORS ");

	cancek1kb-	fch_-efiroot;	CTORS ");

	cancek1kb-	fch_|Cu.Lmb->kepa			 bliting");

	cancek2)b- |Lygic(&b->c->stre~_node_get(bCTORS ");

	cancek1ke *
s	pa			 eiting");

	cancek2)b-	fch_*C_MERGE_bio *bio)sed thnfch_btree_node_2

static void mak
t btre81_"s(c, t) &
		bch_prhstex_2void make_-	e *cl)stry anfouni0r2:		parent 0; --ifreeing_k2*n1 = btree_bse
out:
	mutex_unlocn2); != PTR_HASH(t btree *bch_btree_des[idirtact;s);

	mut");

	cancek1kCSprhsbtreepn; --i) {
		s1t
t);
		bkey_copy_retkeylist *, atkb-k;_1,t_node_*, atkb-k
(	goto otkb-}

->keysct 81_"s(c, t) &
		bch_prhstex_1k); i++)
		e *cl)stry anfouni0r1wait for any in flight s1t
t);
		bkey_coeys)
	n3SHASH(d *Depthde_cre_unstee as st)
		held.		yctn; --i) {
		s3t
t);
		bkey_copy_fch_btree_node_3

staticn st    bcpy_retkeylist *, atkb-k;_3{
		cac
		bch_prhste;
	}

	/*		e *cl)stry anfouni0r3:		parent 0; --ifreeing_k3t
t);
		bkey_co
		wn(&b->io_muteparent btre(&n->keet_eoo];n3bse
out:
	mutex_unlocn3); != PTR_Hee_sok);
		bch_
			 sk &eld.fi)led up * fodiild n_cannibabeorf Ctn		yctwn(&b->io_muteparent btre(&n->keet_eoo];n1); != PTR_HASH(sk Sf Ctnst)nt held.rootde attwn(&b->io_muteparent vd i;

	mutex_lock(&b->b,u
		bch_prhs.t_key(t btre81_"s(c,cks(ie
		bch_prhsresere_retkeylist *, atomic_b);
		bch{
		cac
		bch_prhste;
	}te;
	}

	/*ch(structtre81_"s(c,cache_f
		bch_prhsr}

->keysite_lock);

	if0;
	}ut:
	mutex_unlocn1btree_retg bu0;
	ssrnaYED_	me *b, sruct ef Ct_that,;reading buTREE]t *c, struket_lock2:
_SECTORS);, int x_2void makerite_lock);

	ifn2;
	}ut:
	mutex_unlocn2;
	ket_lock1:
_SECTORS);, int x_1void makerite_lock);

	ifn1;
	}ut:
	mutex_unlocn1btrloc_faMEM);
}

/0;
}

/if (!Irf Ctnne thrlse if (%u)",tree if (KEY_Ds)
	n3crs:h_bucket_alloc_sb->c)) > = NUs:h_bucket_alloc_sb->c)) > =1NUs:h_bucket_alloc_s_init_stack_alloc_;
inker *shrink,
				  struct  struct keylist *, atomic_t *, struct bkgood_keys ruct btree *parent)
{
	intsce(struct *, atkb-k
	BKEY_btree *b, ytes(i)		gfparent)
{
	int i = 0	goto otkb-nodes

	if (btree_check_
 = 1;

	for (; i he)o,oto otkb-}


 RR_OR_NULL(r[nodes].b))
		kmplete_write(b, btree_current_writ btreeb_btreevel =

	mut");

get(bC)fy_data->;s - 1))max(s);

untree *paere_retkeylist tex_unlo0;
 sk list wrots stH(c, /sereturntre81_"s(c, b->k_t *, atkb-k)h>et *, atrid _ha				\
};bak&n->keys, &lock(&b->c->bucket_lock)>plow *ef Ct;keefetch(stru btreeb_btreevel =

	mut");

get(bC)resereturnretkeylist *, atkb-k;b,t_node_*, atkb-k
(	goto otkb-}e_cach			p				    &b->c	nretkeylisde wear_bit(, ytes(i)		gf)ock(GFP_NOIO		e *cl)stry anfouni0b:		parentb->c, op);
}

/* G(b, btree_current_sk k);
/
	ifeuct bru|P_n - g&make_requestt below we havee atwn(&b->io_mutex);
	upt *c, struef Ct
		gotobucket_lock);
		b _cachc, struc ty(b))
		btrwrite_lwrit;init_stack_alloc_;
!= PTR_Hee_sc, struc <ty(b))
		btrwrite_l _cachc, struc ty(b))
		btrwrite_lwrit;init_stack_act cache PTR_HASH(sk /nDELAYED_ w<h	r));
__clsde atti->io, &k.truct ef Ct;b,t_node_*, atkb-k
(	goto otkb-}ree(b = 0;tre81_"s(c,cache_t *, atkb-k)	->c	o, niba0ock(GFP_ 			p	o, )we'	o, nibalize_lock(cannibacanru->kct or (i = 0; i <t *, ate
	/* If ;
	void *data = (void *) i
{
	unsigned i, nodes->bio_list);

	e
	/* If t bkei->io, &k.lize_lockbio *bio;);
	if trg_on(t->s. tr[0];HRINK_STOP;

	ifseqg_on(tseqk_reserve(sce(strut *, a;esint8_up (PTRg_oc, struc tk.l])ree_return 0;

	memset *, aresereturup (PTR->written);

	rect btb;
}

sta, k)) {_unlock,tree if (KEY_Di = 0;s - 1. tr[0]el =

	mut trg->c                   n(tseqel =seqe+    k                        c, struc ty(b)nt_keys	ee node in t               c(list_loc(c, o k, ie
	/* If ,L(b)) ata randomprhf<Sp&e
	/* If );
tr[0],C_MERGE_bio *bio)sed t
					PTDEVie
	/* If ,L0(c, k,CHECKTDEV))ree_return 0;

 t) &t *, a, e
	/* If t
	upt *xt(btrekeylist *, atomic_b{
		cact *, a, ;
	}te;
	}

	o ch(struco, &he)cttre81_"s(c,cache_ft *, ar)	list_moeturup (PTR-
;
		if (PTR_HASH(c, &b->key) !er *shrcanruct bteach_key_fil *, atnsi

	if (c->shrinins	op;
(eserve(sce(str	*firoot;btree *b	 ytes(i)		gfey, (k = n2->s	0	goto otkb-ee_op *,
				 struey_fil *, atfn(oid *) i
{
	unsigb_		ca

	mutex_lock(&b->write_loceey_fil *, atnsigneker_disabled)
		b_		ct_lo	(ite_loceey_fil *, atnsnl_key(
	i->io, &k.ttrekeylist *, atomic_b{
t_locnsnl_k	ik_key_alloc_k	iytes(i)		gfpl_k	i	goto otkb-n&c-ied o, &he)cttre81_"s(c,cache__k	ik_kes_init_stackcanru-	return{
	struMAP_DONEruct or (i = 0; i <t *, aol wait,
				     struct btresce(struckb-k
	BKrrentbtree *b, ytes(i)		gfp)
{
	int i = 0	goto otkb-nodes

	if (eey_fil *, atnsiop;
(i->io, &k.keyk);

	if (mca_find(c, k))
		rDeys, "wrrn 81_"s(c,cache_chinc_gen)truct bsetopsg_chec		.nsnl_errooce_bus		defiroot;oceytes(i)		gf	deytes(i)		gfey,oce	goto otkb-	de	goto otkb-eehery(&c->bo, &he)cttre81_"s(c,cache_kb-k)		if (		.ns.truc ty0ock(can&k.ttrekeylismapsde weites ec		.nsnlct
lloc(b- , & 
STARc(c,  jb-kCSprhsbt
lloc(b- , & uey_fil *, atfnrentb->c	 blo, )	if (gc->node_inc(&EY_DiprBUG_("UG_ON GC_M o, )ey_c,eys += next(btre81_"s(c,cop_kb-k)	ed i,nfch_RS);

	ey) != PTR_Hee_sc,.ns.t *, atet *ismnt)
vel, &k._aSRCHesercannibacanruct 
{
	struct bseteet_eoo];

	mutex_lock(&b->wrn true for the keys btree_check_
 RR_OR_NULL(r[nodes].b))
		kt btree *bch_btree_eet_eoo]; (btree_node_ct btree *parenteplacement(r[i].e(c, k, i> void makey, i),e_node_);
}

static int btree_check

stati!key(c, level,
		kmplete_write(b,ount(struct shrinksh(c, k));

	t_add_heafor any in flight y(k, &b->key);
	bkey_c(b))
		btrxt(bgen)trucytes(i)	metaic int bparentwn(&b->io_mutex);
	 *bd *Map acrossrites Fpla	fch_*tree_node_or (i = 0; i <mapsites _eyee_op, struct gc_merge_info *r)
{
	unsigned i,  nodes->bio_list);

	r ged i,  nodes->0; i <mapsites _fn
	rn_PTRS(>work) bkei->io, &k.MAP_CONTINUEesereturn ta, c, i
			gc->nodes++;

	folr_each_key_filter(&b->keal	_retkeylister sg_checks);

	rete key"r ge)ey_c,eys += next(btrekeylister s_u64e, btree_e key";
	ik_key_alloc(b- , &b->ke++;
baur != level, &k.truct(mapsites _eyee_opn-kt ld i,  nodensnlr ged rn_P>work);
re	r genri k < bset-ied o, &!k.MAP_CONTINUE)					o, tackcanru-yc(list_			p	for (; i ||P>work tk.MAP_ALL

	blo)
vel, &k.fn((&n->)esercannibacanruct i->iode(struct btrpsites (oid *) i
{
	unsig		ca

	mutet btree *parent)
->bio_list);

	r ged>0; i <mapsites _fn
	rn_PTRS(>work) bkehe_set *ruct eoo];mapsites _eyee_opn-ock(&n-r ged rn_P>work);
	  struct  struct keylismapsk_ke_eyee_op, struct gc_merge_info *r)
{
	unsigned i,  nodes-bio_list);

	r ged>0; i <mapsk_ke_fn
	rn_ i,  nodes-TRS(>work) bkei->io, &k.MAP_CONTINUEes	gc->nodes++;

	for_each_key_filter(&b->keee_retkeylister sg_checks);

	rete key"r ge)ey_ceys += next(btrekeylister s_u64e, btree_e key";
	ik_keyb->ke++;
baur != levo, &k.	for (; i i, ?.fn((&n->tructi, :.truct(mapsk_ke_eyee_opn-kt ldensnlr ged rn_P>work);
rer genri k < bsetied o, &!k.MAP_CONTINUE)				cannibacanru->key			p	for (; i he)(>work &.MAP_END (!ptr_avail&k.fn((&n->y";c,  e(c,I
	bli> void m_ i,  nodese(c,OFF
		i> void m_l_e)esercannibacanruct i->iuct keylismapsk_ke(oid *) i
{
	unsig		ca

	mutet btree *parent)nodes->bio_list);

	r ged>0; i <mapsk_ke_fn
	rn_PTRS(>work) bkehe_set *ruct eoo];mapsk_ke_eyee_opn-ock(&n-r ged rn_P>work);
	  d *Kfcbufr_dotde a struct  slinePTRS(kfcbuf_c");el) * 2	fcbuf_s++;
lruct btrescebuf_s++;
_nodesd *Overlapp\
})

/**dirta_chequalor (i = 0;kn gc");
l

staticSTARc(c,  &rT/**
 *	<	   init_stack_1;(i = 0;kn gc");
STARc(c,  &lvoid m_l&rT/**
  >	   init_stack1;set *c, struct btree_o  linePTRS(kfcbuf_nonoverlapp\
}_c");el) * 2	fcbuf_s++;
lr
lloc(b- ,ct btrescebuf_s++;
_nodest *c, sclamp_t_t  *bio, ;kn gc");
l

staticrvoid m_l-1,L(b))ct bteach_	gfrecu

	if (c->shrinins	op;
(*, structn, (fund;
(eserve(scebuf	*bufey, (k = n2->s	0end;
(scebuf_pred_fn	*predee_op *,
				 str	gfrec_scebuf_fn(oid *) i
{
	unsig		ca

	mutex_lock(&d i, nodedep_ass);
	b = mca_alloc(p	gfrecu*	gfrecuer_disabled)
				ca

	mute	gfrectronovaleserve(scebuf *buf&c-rgfrec &b-f;kei->io, &k.MAP_CONTINUEesereturnkng & b->prrgfrec &gc_al>   &n->ko, &k.MAP_DONEruO);
	if (!b)
>key			p	.nsets]; t+) sk gc_e *cle att node in theied o,frec &pred(b-fvel != level) * 2	fcbuf_s++;
w w
		surna(&ca->buf&b->key) _c,el -array
		mute>buf&b

	iinit_btrlidat!wEN(k, i)urna
}

/* Gbuf&b->key) 	rn{
	struMAP_DONEru		}ree_w

stava g&k.*mca_al)rfch_btre(&w op, E_h_btree_nodRBlINee_T Gbuf&bk_keybwy(ock),(kfcbuf_c")EE_PRIjrray
&b->c-buf&b

	iinitybw)ock(GFP_NOIOo,frec &n, (funds += bkey_ujrray
&b->"s(c,cache_fbuf&b

	iinit_)				can&k.MAP_DONEru
 i)urna
}

/* Gbuf&b->key) 	}list_mobuf&b-ax(s)int8s.set

	focannibacanruct 
{
	struc	gfrec_scebufol wait,
				     struct btrescebuf *bufent)nodes->bio_list);

	gc_,(kfcbuf_pred_fn *predb->write_loces++;readixt(buf&b-ax(s)int8s.;a_alloc(p	gfrecu	gfreck_
 Rkei_eyb);

( 
	s,truct bsetopsg_chec	gfrec.nsnl-1;
	}ugfrec.n, (fund	=lloc}ugfrec.buf	t(bufoc}ugfrec.end	=lend;
(ugfrec.pred	=lpredees,truct bsetmapsk_ke(c	gfrec.nsnlnt btuf&b-ax(s)int8s.d i, nod	gfrec_scebuf_fn,.MAP_END (!pt
		kt btree *bch_k_keint(ugfrec.n, (fundd i, nodese(c,I
	bli>int8_t,se(c,OFF
		i>int8_t, i, nodese(c,I
	bli>tuf&b-ax(s)int8s.t, i, nodese(c,OFF
		i> uf&b-ax(s)int8s.twritelurna(&ca->buf&b->key) _cidat!RBlEMPTY_ROOT Gbuf&bk_ke != level) * 2	fcbuf_s++;
w wc,el -RBlFIRST Gbuf&bk_keybel) * 2	fcbuf_s++y(ock)E_al)ruf&bint8_	=lSTARc(c,  &wvoid mak
t el -RBlLAST Gbuf&bk_keybel) * 2	fcbuf_s++y(ock)E_al)ruf&bend	=lwt
 * @ope PTR_HASH(ruf&bint8_	=ln st   _al)ruf&bend	=ln st   _al}itelurna
}

/* Gbuf&b->key) urnal_ref)
{
	sale = 0;
buf_ k)(ct btrescebuf *bufe,ct btrescebuf_s++;
wnodestb_eraop,&wvoock),(Gbuf&bk_ke ot;brray
&b->c-buf&b

	iinitybw)occt 
{
	struc0;
buf_ k)(ct btrescebuf *bufe,ct btrescebuf_s++;
wnodeslurna(&ca->buf&b->key) 	ale = 0;
buf_ k)(bufe,w)ocklurna
}

/* Gbuf&b->key) urnint8_btrekfcbuf_c
	/* overlapp\
}(ct btrescebuf *bufe,ct btret);

		unn:		cac
->bio_list);

	gc_aodesint8_o, &k.ct btreect btrescebuf_s++;
snl
w,sT	fore_buset
{
				iereturnkng & b-gc_,(&ruf&bint8_*	<	  b->c)) > nkng & b-	unn:	(&ruf&bgc_al>	   init_stackct btreeslurna(&ca->buf&b->key) 	el -RBlGREATER Gbuf&bk_keybey(ock),(kfcbuf_nonoverlapp\
}_c"))ey_ceys += w(!level && b-&STARc(c,  &wvoid m, gc_alway!= levekerw wc,el -RBlNEXT(wy(ock)E_a bkey_up

stava g)				can&k.ge_inf)(GFP_NOIOale = 0;
buf_ k)(bufe,p)_al}itelurna
}

/* Gbuf&b->key) !er *shrcanruct bteach_scebuf_s++;
e = 0;
buf__unloct btrescebuf *bufreserve = (cst_buf_s++;
w wclurna(&ca->buf&b->key) _cel -RBlFIRST Gbuf&bk_keybel) * 2	fcbuf_s++y(ock)E_a_ceys += w(!lew

stava g)			el -RBlNEXT(wy(ock)E_a bwrit )ee_w

stava g&k., list)
		if (!mca_lurna
}

/* Gbuf&b->key) !er *shrwruct bteach_scebuf_s++;
e = 0;
buf__unl_eyb)annt level, bool write, op, level, op scebuf *bufent)ac
->bio_list);

	gc_ent)ac
->kfcbuf_pred_fn *predb->write_locscebuf_s++;
_etmca_eys +=    k = can&k.ttre0;
buf__unlobufrbtrlidato, )we'	ata_free(b = 0;ing & b-btuf&b-ax(s)int8s.d gc_al>	   N(k, ipr_ kgoto")int fg_c[hed_bset_e& !i->keyc(w->truc	gfrec_scebufoc,deffd gc_,lpred)_al}itecannibacanruct 
{
	struc0;
buf_g_chect btrescebuf *bufresertuf&b-ax(s)int8s.	=ln st   _albuf&bk_ke		deRBlROOTreeslurn