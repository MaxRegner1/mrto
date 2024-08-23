// SPDX-License-Identifier: GPL-2.0
/*
 * This file contains common generic and tag-based KASAN code.
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 * Author: Andrey Ryabinin <ryabinin.a.a@gmail.com>
 *
 * Some code borrowed from https://github.com/xairy/kasan-prototype by
 *        Andrey Konovalov <andreyknvl@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define __KASAN_INTERNAL

#include <linux/export.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/kasan.h>
#include <linux/kernel.h>
#include <linux/kmemleak.h>
#include <linux/linkage.h>
#include <linux/memblock.h>
#include <linux/memory.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/sched/task_stack.h>
#include <linux/slab.h>
#include <linux/stacktrace.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/bug.h>
#include <linux/uaccess.h>

#include "kasan.h"
#include "../slab.h"

static inline int in_irqentry_text(unsigned long ptr)
{
	return (ptr >= (unsigned long)&__irqentry_text_start &&
		ptr < (unsigned long)&__irqentry_text_end) ||
		(ptr >= (unsigned long)&__softirqentry_text_start &&
		 ptr < (unsigned long)&__softirqentry_text_end);
}

static inline void filter_irq_stacks(struct stack_trace *trace)
{
	int i;

	if (!trace->nr_entries)
		return;
	for (i = 0; i < trace->nr_entries; i++)
		if (in_irqentry_text(trace->entries[i])) {
			/* Include the irqentry function into the stack. */
			trace->nr_entries = i + 1;
			break;
		}
}

static inline depot_stack_handle_t save_stack(gfp_t flags)
{
	unsigned long entries[KASAN_STACK_DEPTH];
	struct stack_trace trace = {
		.nr_entries = 0,
		.entries = entries,
		.max_entries = KASAN_STACK_DEPTH,
		.skip = 0
	};

	save_stack_trace(&trace);
	filter_irq_stacks(&trace);

	return depot_save_stack(&trace, flags);
}

static inline void set_track(struct kasan_track *track, gfp_t flags)
{
	track->pid = current->pid;
	track->stack = save_stack(flags);
}

void kasan_enable_current(void)
{
	current->kasan_depth++;
}

void kasan_disable_current(void)
{
	current->kasan_depth--;
}

void kasan_check_read(const volatile void *p, unsigned int size)
{
	check_memory_region((unsigned long)p, size, false, _RET_IP_);
}
EXPORT_SYMBOL(kasan_check_read);

void kasan_check_write(const volatile void *p, unsigned int size)
{
	check_memory_region((unsigned long)p, size, true, _RET_IP_);
}
EXPORT_SYMBOL(kasan_check_write);

#undef memset
void *memset(void *addr, int c, size_t len)
{
	check_memory_region((unsigned long)addr, len, true, _RET_IP_);

	return __memset(addr, c, len);
}

#undef memmove
void *memmove(void *dest, const void *src, size_t len)
{
	check_memory_region((unsigned long)src, len, false, _RET_IP_);
	check_memory_region((unsigned long)dest, len, true, _RET_IP_);

	return __memmove(dest, src, len);
}

#undef memcpy
void *memcpy(void *dest, const void *src, size_t len)
{
	check_memory_region((unsigned long)src, len, false, _RET_IP_);
	check_memory_region((unsigned long)dest, len, true, _RET_IP_);

	return __memcpy(dest, src, len);
}

/*
 * Poisons the shadow memory for 'size' bytes starting from 'addr'.
 * Memory addresses should be aligned to KASAN_SHADOW_SCALE_SIZE.
 */
void kasan_poison_shadow(const void *address, size_t size, u8 value)
{
	void *shadow_start, *shadow_end;

	/*
	 * Perform shadow offset calculation based on untagged address, as
	 * some of the callers (e.g. kasan_poison_object_data) pass tagged
	 * addresses to this function.
	 */
	address = reset_tag(address);

	shadow_start = kasan_mem_to_shadow(address);
	shadow_end = kasan_mem_to_shadow(address + size);

	__memset(shadow_start, value, shadow_end - shadow_start);
}

void kasan_unpoison_shadow(const void *address, size_t size)
{
	u8 tag = get_tag(address);

	/*
	 * Perform shadow offset calculation based on untagged address, as
	 * some of the callers (e.g. kasan_unpoison_object_data) pass tagged
	 * addresses to this function.
	 */
	address = reset_tag(address);

	kasan_poison_shadow(address, size, tag);

	if (size & KASAN_SHADOW_MASK) {
		u8 *shadow = (u8 *)kasan_mem_to_shadow(address + size);

		if (IS_ENABLED(CONFIG_KASAN_SW_TAGS))
			*shadow = tag;
		else
			*shadow = size & KASAN_SHADOW_MASK;
	}
}

static void __kasan_unpoison_stack(struct task_struct *task, const void *sp)
{
	void *base = task_stack_page(task);
	size_t size = sp - base;

	kasan_unpoison_shadow(base, size);
}

/* Unpoison the entire stack for a task. */
void kasan_unpoison_task_stack(struct task_struct *task)
{
	__kasan_unpoison_stack(task, task_stack_page(task) + THREAD_SIZE);
}

/* Unpoison the stack for the current task beyond a watermark sp value. */
asmlinkage void kasan_unpoison_task_stack_below(const void *watermark)
{
	/*
	 * Calculate the task stack base address.  Avoid using 'current'
	 * because this function is called by early resume code which hasn't
	 * yet set up the percpu register (%gs).
	 */
	void *base = (void *)((unsigned long)watermark & ~(THREAD_SIZE - 1));

	kasan_unpoison_shadow(base, watermark - base);
}

/*
 * Clear all poison for the region between the current SP and a provided
 * watermark value, as is sometimes required prior to hand-crafted asm function
 * returns in the middle of functions.
 */
void kasan_unpoison_stack_above_sp_to(const void *watermark)
{
	const void *sp = __builtin_frame_address(0);
	size_t size = watermark - sp;

	if (WARN_ON(sp > watermark))
		return;
	kasan_unpoison_shadow(sp, size);
}

void kasan_alloc_pages(struct page *page, unsigned int order)
{
	u8 tag;
	unsigned long i;

	if (unlikely(PageHighMem(page)))
		return;

	tag = random_tag();
	for (i = 0; i < (1 << order); i++)
		page_kasan_tag_set(page + i, tag);
	kasan_unpoison_shadow(page_address(page), PAGE_SIZE << order);
}

void kasan_free_pages(struct page *page, unsigned int order)
{
	if (likely(!PageHighMem(page)))
		kasan_poison_shadow(page_address(page),
				PAGE_SIZE << order,
				KASAN_FREE_PAGE);
}

/*
 * Adaptive redzone policy taken from the userspace AddressSanitizer runtime.
 * For larger allocations larger redzones are used.
 */
static inline unsigned int optimal_redzone(unsigned int object_size)
{
	if (IS_ENABLED(CONFIG_KASAN_SW_TAGS))
		return 0;

	return
		object_size <= 64        - 16   ? 16 :
		object_size <= 128       - 32   ? 32 :
		object_size <= 512       - 64   ? 64 :
		object_size <= 4096      - 128  ? 128 :
		object_size <= (1 << 14) - 256  ? 256 :
		object_size <= (1 << 15) - 512  ? 512 :
		object_size <= (1 << 16) - 1024 ? 1024 : 2048;
}

void kasan_cache_create(struct kmem_cache *cache, unsigned int *size,
			unsigned long *flags)
{
	unsigned int orig_size = *size;
	unsigned int redzone_size;
	int redzone_adjust;

	/* Add alloc meta. */
	cache->kasan_info.alloc_meta_offset = *size;
	*size += sizeof(struct kasan_alloc_meta);

	/* Add free meta. */
	if (IS_ENABLED(CONFIG_KASAN_GENERIC) &&
	    (cache->flags & SLAB_TYPESAFE_BY_RCU || cache->ctor ||
	     cache->object_size < sizeof(struct kasan_free_meta))) {
		cache->kasan_info.free_meta_offset = *size;
		*size += sizeof(struct kasan_free_meta);
	}

	redzone_size = optimal_redzone(cache->object_size);
	redzone_adjust = redzone_size -	(*size - cache->object_size);
	if (redzone_adjust > 0)
		*size += redzone_adjust;

	*size = min_t(unsigned int, KMALLOC_MAX_SIZE,
			max(*size, cache->object_size + redzone_size));

	/*
	 * If the metadata doesn't fit, don't enable KASAN at all.
	 */
	if (*size <= cache->kasan_info.alloc_meta_offset ||
			*size <= cache->kasan_info.free_meta_offset) {
		cache->kasan_info.alloc_meta_offset = 0;
		cache->kasan_info.free_meta_offset = 0;
		*size = orig_size;
		return;
	}

	*flags |= SLAB_KASAN;
}

size_t kasan_metadata_size(struct kmem_cache *cache)
{
	return (cache->kasan_info.alloc_meta_offset ?
		sizeof(struct kasan_alloc_meta) : 0) +
		(cache->kasan_info.free_meta_offset ?
		sizeof(struct kasan_free_meta) : 0);
}

struct kasan_alloc_meta *get_alloc_info(struct kmem_cache *cache,
					const void *object)
{
	return (void *)object + cache->kasan_info.alloc_meta_offset;
}

struct kasan_free_meta *get_free_info(struct kmem_cache *cache,
				      const void *object)
{
	BUILD_BUG_ON(sizeof(struct kasan_free_meta) > 32);
	return (void *)object + cache->kasan_info.free_meta_offset;
}


static void kasan_set_free_info(struct kmem_cache *cache,
		void *object, u8 tag)
{
	struct kasan_alloc_meta *alloc_meta;
	u8 idx = 0;

	alloc_meta = get_alloc_info(cache, object);

#ifdef CONFIG_KASAN_SW_TAGS_IDENTIFY
	idx = alloc_meta->free_track_idx;
	alloc_meta->free_pointer_tag[idx] = tag;
	alloc_meta->free_track_idx = (idx + 1) % KASAN_NR_FREE_STACKS;
#endif

	set_track(&alloc_meta->free_track[idx], GFP_NOWAIT);
}

void kasan_poison_slab(struct page *page)
{
	unsigned long i;

	for (i = 0; i < (1 << compound_order(page)); i++)
		page_kasan_tag_reset(page + i);
	kasan_poison_shadow(page_address(page),
			PAGE_SIZE << compound_order(page),
			KASAN_KMALLOC_REDZONE);
}

void kasan_unpoison_object_data(struct kmem_cache *cache, void *object)
{
	kasan_unpoison_shadow(object, cache->object_size);
}

void kasan_poison_object_data(struct kmem_cache *cache, void *object)
{
	kasan_poison_shadow(object,
			round_up(cache->object_size, KASAN_SHADOW_SCALE_SIZE),
			KASAN_KMALLOC_REDZONE);
}

/*
 * This function assigns a tag to an object considering the following:
 * 1. A cache might have a constructor, which might save a pointer to a slab
 *    object somewhere (e.g. in the object itself). We preassign a tag for
 *    each object in caches with constructors during slab creation and reuse
 *    the same tag each time a particular object is allocated.
 * 2. A cache might be SLAB_TYPESAFE_BY_RCU, which means objects can be
 *    accessed after being freed. We preassign tags for objects in these
 *    caches as well.
 * 3. For SLAB allocator we can't preassign tags randomly since the freelist
 *    is stored as an array of indexes instead of a linked list. Assign tags
 *    based on objects indexes, so that objects that are next to each other
 *    get different tags.
 */
static u8 assign_tag(struct kmem_cache *cache, const void *object,
			bool init, bool keep_tag)
{
	/*
	 * 1. When an object is kmalloc()'ed, two hooks are called:
	 *    kasan_slab_alloc() and kasan_kmalloc(). We assign the
	 *    tag only in the first one.
	 * 2. We reuse the same tag for krealloc'ed objects.
	 */
	if (keep_tag)
		return get_tag(object);

	/*
	 * If the cache neither has a constructor nor has SLAB_TYPESAFE_BY_RCU
	 * set, assign a tag when the object is being allocated (init == false).
	 */
	if (!cache->ctor && !(cache->flags & SLAB_TYPESAFE_BY_RCU))
		return init ? KASAN_TAG_KERNEL : random_tag();

	/* For caches that either have a constructor or SLAB_TYPESAFE_BY_RCU: */
#ifdef CONFIG_SLAB
	/* For SLAB assign tags based on the object index in the freelist. */
	return (u8)obj_to_index(cache, virt_to_page(object), (void *)object);
#else
	/*
	 * For SLUB assign a random tag during slab creation, otherwise reuse
	 * the already assigned tag.
	 */
	return init ? random_tag() : get_tag(object);
#endif
}

void * __must_check kasan_init_slab_obj(struct kmem_cache *cache,
						const void *object)
{
	struct kasan_alloc_meta *alloc_info;

	if (!(cache->flags & SLAB_KASAN))
		return (void *)object;

	alloc_info = get_alloc_info(cache, object);
	__memset(alloc_info, 0, sizeof(*alloc_info));

	if (IS_ENABLED(CONFIG_KASAN_SW_TAGS))
		object = set_tag(object,
				assign_tag(cache, object, true, false));

	return (void *)object;
}

static inline bool shadow_invalid(u8 tag, s8 shadow_byte)
{
	if (IS_ENABLED(CONFIG_KASAN_GENERIC))
		return shadow_byte < 0 ||
			shadow_byte >= KASAN_SHADOW_SCALE_SIZE;

	/* else CONFIG_KASAN_SW_TAGS: */
	if ((u8)shadow_byte == KASAN_TAG_INVALID)
		return true;
	if ((tag != KASAN_TAG_KERNEL) && (tag != (u8)shadow_byte))
		return true;

	return false;
}

static bool __kasan_slab_free(struct kmem_cache *cache, void *object,
			      unsigned long ip, bool quarantine)
{
	s8 shadow_byte;
	u8 tag;
	void *tagged_object;
	unsigned long rounded_up_size;

	tag = get_tag(object);
	tagged_object = object;
	object = reset_tag(object);

	if (unlikely(nearest_obj(cache, virt_to_head_page(object), object) !=
	    object)) {
		kasan_report_invalid_free(tagged_object, ip);
		return true;
	}

	/* RCU slabs could be legally used after free within the RCU period */
	if (unlikely(cache->flags & SLAB_TYPESAFE_BY_RCU))
		return false;

	shadow_byte = READ_ONCEA(;l?R(;?xb=CnIbll?ROC
	if (!cache->.Nsaui@i