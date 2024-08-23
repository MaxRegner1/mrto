/*
 *  Copyright (C) 2009  Red Hat, Inc.
 *
 *  This work is licensed under the terms of the GNU GPL, version 2. See
 *  the COPYING file in the top-level directory.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/sched/coredump.h>
#include <linux/sched/numa_balancing.h>
#include <linux/highmem.h>
#include <linux/hugetlb.h>
#include <linux/mmu_notifier.h>
#include <linux/rmap.h>
#include <linux/swap.h>
#include <linux/shrinker.h>
#include <linux/mm_inline.h>
#include <linux/swapops.h>
#include <linux/dax.h>
#include <linux/khugepaged.h>
#include <linux/freezer.h>
#include <linux/pfn_t.h>
#include <linux/mman.h>
#include <linux/memremap.h>
#include <linux/pagemap.h>
#include <linux/debugfs.h>
#include <linux/migrate.h>
#include <linux/hashtable.h>
#include <linux/userfaultfd_k.h>
#include <linux/page_idle.h>
#include <linux/shmem_fs.h>
#include <linux/oom.h>
#include <linux/page_owner.h>

#include <asm/tlb.h>
#include <asm/pgalloc.h>
#include "internal.h"

/*
 * By default transparent hugepage support is disabled in order that avoid
 * to risk increase the memory footprint of applications without a guaranteed
 * benefit. When transparent hugepage support is enabled, is for all mappings,
 * and khugepaged scans all mappings.
 * Defrag is invoked by khugepaged hugepage allocations and by page faults
 * for all hugepage allocations.
 */
unsigned long transparent_hugepage_flags __read_mostly =
#ifdef CONFIG_TRANSPARENT_HUGEPAGE_ALWAYS
	(1<<TRANSPARENT_HUGEPAGE_FLAG)|
#endif
#ifdef CONFIG_TRANSPARENT_HUGEPAGE_MADVISE
	(1<<TRANSPARENT_HUGEPAGE_REQ_MADV_FLAG)|
#endif
	(1<<TRANSPARENT_HUGEPAGE_DEFRAG_REQ_MADV_FLAG)|
	(1<<TRANSPARENT_HUGEPAGE_DEFRAG_KHUGEPAGED_FLAG)|
	(1<<TRANSPARENT_HUGEPAGE_USE_ZERO_PAGE_FLAG);

static struct shrinker deferred_split_shrinker;

static atomic_t huge_zero_refcount;
struct page *huge_zero_page __read_mostly;

static struct page *get_huge_zero_page(void)
{
	struct page *zero_page;
retry:
	if (likely(atomic_inc_not_zero(&huge_zero_refcount)))
		return READ_ONCE(huge_zero_page);

	zero_page = alloc_pages((GFP_TRANSHUGE | __GFP_ZERO) & ~__GFP_MOVABLE,
			HPAGE_PMD_ORDER);
	if (!zero_page) {
		count_vm_event(THP_ZERO_PAGE_ALLOC_FAILED);
		return NULL;
	}
	count_vm_event(THP_ZERO_PAGE_ALLOC);
	preempt_disable();
	if (cmpxchg(&huge_zero_page, NULL, zero_page)) {
		preempt_enable();
		__free_pages(zero_page, compound_order(zero_page));
		goto retry;
	}

	/* We take additional reference here. It will be put back by shrinker */
	atomic_set(&huge_zero_refcount, 2);
	preempt_enable();
	return READ_ONCE(huge_zero_page);
}

static void put_huge_zero_page(void)
{
	/*
	 * Counter should never go to zero here. Only shrinker can put
	 * last reference.
	 */
	BUG_ON(atomic_dec_and_test(&huge_zero_refcount));
}

struct page *mm_get_huge_zero_page(struct mm_struct *mm)
{
	if (test_bit(MMF_HUGE_ZERO_PAGE, &mm->flags))
		return READ_ONCE(huge_zero_page);

	if (!get_huge_zero_page())
		return NULL;

	if (test_and_set_bit(MMF_HUGE_ZERO_PAGE, &mm->flags))
		put_huge_zero_page();

	return READ_ONCE(huge_zero_page);
}

void mm_put_huge_zero_page(struct mm_struct *mm)
{
	if (test_bit(MMF_HUGE_ZERO_PAGE, &mm->flags))
		put_huge_zero_page();
}

static unsigned long shrink_huge_zero_page_count(struct shrinker *shrink,
					struct shrink_control *sc)
{
	/* we can free zero page only if last reference remains */
	return atomic_read(&huge_zero_refcount) == 1 ? HPAGE_PMD_NR : 0;
}

static unsigned long shrink_huge_zero_page_scan(struct shrinker *shrink,
				       struct shrink_control *sc)
{
	if (atomic_cmpxchg(&huge_zero_refcount, 1, 0) == 1) {
		struct page *zero_page = xchg(&huge_zero_page, NULL);
		BUG_ON(zero_page == NULL);
		__free_pages(zero_page, compound_order(zero_page));
		return HPAGE_PMD_NR;
	}

	return 0;
}

static struct shrinker huge_zero_page_shrinker = {
	.count_objects = shrink_huge_zero_page_count,
	.scan_objects = shrink_huge_zero_page_scan,
	.seeks = DEFAULT_SEEKS,
};

#ifdef CONFIG_SYSFS
static ssize_t enabled_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	if (test_bit(TRANSPARENT_HUGEPAGE_FLAG, &transparent_hugepage_flags))
		return sprintf(buf, "[always] madvise never\n");
	else if (test_bit(TRANSPARENT_HUGEPAGE_REQ_MADV_FLAG, &transparent_hugepage_flags))
		return sprintf(buf, "always [madvise] never\n");
	else
		return sprintf(buf, "always madvise [never]\n");
}

static ssize_t enabled_store(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	ssize_t ret = count;

	if (sysfs_streq(buf, "always")) {
		clear_bit(TRANSPARENT_HUGEPAGE_REQ_MADV_FLAG, &transparent_hugepage_flags);
		set_bit(TRANSPARENT_HUGEPAGE_FLAG, &transparent_hugepage_flags);
	} else if (sysfs_streq(buf, "madvise")) {
		clear_bit(TRANSPARENT_HUGEPAGE_FLAG, &transparent_hugepage_flags);
		set_bit(TRANSPARENT_HUGEPAGE_REQ_MADV_FLAG, &transparent_hugepage_flags);
	} else if (sysfs_streq(buf, "never")) {
		clear_bit(TRANSPARENT_HUGEPAGE_FLAG, &transparent_hugepage_flags);
		clear_bit(TRANSPARENT_HUGEPAGE_REQ_MADV_FLAG, &transparent_hugepage_flags);
	} else
		ret = -EINVAL;

	if (ret > 0) {
		int err = start_stop_khugepaged();
		if (err)
			ret = err;
	}
	return ret;
}
static struct kobj_attribute enabled_attr =
	__ATTR(enabled, 0644, enabled_show, enabled_store);

ssize_t single_hugepage_flag_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf,
				enum transparent_hugepage_flag flag)
{
	return sprintf(buf, "%d\n",
		       !!test_bit(flag, &transparent_hugepage_flags));
}

ssize_t single_hugepage_flag_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count,
				 enum transparent_hugepage_flag flag)
{
	unsigned long value;
	int ret;

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0)
		return ret;
	if (value > 1)
		return -EINVAL;

	if (value)
		set_bit(flag, &transparent_hugepage_flags);
	else
		clear_bit(flag, &transparent_hugepage_flags);

	return count;
}

static ssize_t defrag_show(struct kobject *kobj,
			   struct kobj_attribute *attr, char *buf)
{
	if (test_bit(TRANSPARENT_HUGEPAGE_DEFRAG_DIRECT_FLAG, &transparent_hugepage_flags))
		return sprintf(buf, "[always] defer defer+madvise madvise never\n");
	if (test_bit(TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_FLAG, &transparent_hugepage_flags))
		return sprintf(buf, "always [defer] defer+madvise madvise never\n");
	if (test_bit(TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_OR_MADV_FLAG, &transparent_hugepage_flags))
		return sprintf(buf, "always defer [defer+madvise] madvise never\n");
	if (test_bit(TRANSPARENT_HUGEPAGE_DEFRAG_REQ_MADV_FLAG, &transparent_hugepage_flags))
		return sprintf(buf, "always defer defer+madvise [madvise] never\n");
	return sprintf(buf, "always defer defer+madvise madvise [never]\n");
}

static ssize_t defrag_store(struct kobject *kobj,
			    struct kobj_attribute *attr,
			    const char *buf, size_t count)
{
	if (sysfs_streq(buf, "always")) {
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_FLAG, &transparent_hugepage_flags);
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_OR_MADV_FLAG, &transparent_hugepage_flags);
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_REQ_MADV_FLAG, &transparent_hugepage_flags);
		set_bit(TRANSPARENT_HUGEPAGE_DEFRAG_DIRECT_FLAG, &transparent_hugepage_flags);
	} else if (sysfs_streq(buf, "defer+madvise")) {
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_DIRECT_FLAG, &transparent_hugepage_flags);
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_FLAG, &transparent_hugepage_flags);
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_REQ_MADV_FLAG, &transparent_hugepage_flags);
		set_bit(TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_OR_MADV_FLAG, &transparent_hugepage_flags);
	} else if (sysfs_streq(buf, "defer")) {
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_DIRECT_FLAG, &transparent_hugepage_flags);
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_OR_MADV_FLAG, &transparent_hugepage_flags);
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_REQ_MADV_FLAG, &transparent_hugepage_flags);
		set_bit(TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_FLAG, &transparent_hugepage_flags);
	} else if (sysfs_streq(buf, "madvise")) {
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_DIRECT_FLAG, &transparent_hugepage_flags);
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_FLAG, &transparent_hugepage_flags);
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_OR_MADV_FLAG, &transparent_hugepage_flags);
		set_bit(TRANSPARENT_HUGEPAGE_DEFRAG_REQ_MADV_FLAG, &transparent_hugepage_flags);
	} else if (sysfs_streq(buf, "never")) {
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_DIRECT_FLAG, &transparent_hugepage_flags);
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_FLAG, &transparent_hugepage_flags);
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_OR_MADV_FLAG, &transparent_hugepage_flags);
		clear_bit(TRANSPARENT_HUGEPAGE_DEFRAG_REQ_MADV_FLAG, &transparent_hugepage_flags);
	} else
		return -EINVAL;

	return count;
}
static struct kobj_attribute defrag_attr =
	__ATTR(defrag, 0644, defrag_show, defrag_store);

static ssize_t use_zero_page_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return single_hugepage_flag_show(kobj, attr, buf,
				TRANSPARENT_HUGEPAGE_USE_ZERO_PAGE_FLAG);
}
static ssize_t use_zero_page_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	return single_hugepage_flag_store(kobj, attr, buf, count,
				 TRANSPARENT_HUGEPAGE_USE_ZERO_PAGE_FLAG);
}
static struct kobj_attribute use_zero_page_attr =
	__ATTR(use_zero_page, 0644, use_zero_page_show, use_zero_page_store);

static ssize_t hpage_pmd_size_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", HPAGE_PMD_SIZE);
}
static struct kobj_attribute hpage_pmd_size_attr =
	__ATTR_RO(hpage_pmd_size);

#ifdef CONFIG_DEBUG_VM
static ssize_t debug_cow_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return single_hugepage_flag_show(kobj, attr, buf,
				TRANSPARENT_HUGEPAGE_DEBUG_COW_FLAG);
}
static ssize_t debug_cow_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t count)
{
	return single_hugepage_flag_store(kobj, attr, buf, count,
				 TRANSPARENT_HUGEPAGE_DEBUG_COW_FLAG);
}
static struct kobj_attribute debug_cow_attr =
	__ATTR(debug_cow, 0644, debug_cow_show, debug_cow_store);
#endif /* CONFIG_DEBUG_VM */

static struct attribute *hugepage_attr[] = {
	&enabled_attr.attr,
	&defrag_attr.attr,
	&use_zero_page_attr.attr,
	&hpage_pmd_size_attr.attr,
#if defined(CONFIG_SHMEM) && defined(CONFIG_TRANSPARENT_HUGE_PAGECACHE)
	&shmem_enabled_attr.attr,
#endif
#ifdef CONFIG_DEBUG_VM
	&debug_cow_attr.attr,
#endif
	NULL,
};

static const struct attribute_group hugepage_attr_group = {
	.attrs = hugepage_attr,
};

static int __init hugepage_init_sysfs(struct kobject **hugepage_kobj)
{
	int err;

	*hugepage_kobj = kobject_create_and_add("transparent_hugepage", mm_kobj);
	if (unlikely(!*hugepage_kobj)) {
		pr_err("failed to create transparent hugepage kobject\n");
		return -ENOMEM;
	}

	err = sysfs_create_group(*hugepage_kobj, &hugepage_attr_group);
	if (err) {
		pr_err("failed to register transparent hugepage group\n");
		goto delete_obj;
	}

	err = sysfs_create_group(*hugepage_kobj, &khugepaged_attr_group);
	if (err) {
		pr_err("failed to register transparent hugepage group\n");
		goto remove_hp_group;
	}

	return 0;

remove_hp_group:
	sysfs_remove_group(*hugepage_kobj, &hugepage_attr_group);
delete_obj:
	kobject_put(*hugepage_kobj);
	return err;
}

static void __init hugepage_exit_sysfs(struct kobject *hugepage_kobj)
{
	sysfs_remove_group(hugepage_kobj, &khugepaged_attr_group);
	sysfs_remove_group(hugepage_kobj, &hugepage_attr_group);
	kobject_put(hugepage_kobj);
}
#else
static inline int hugepage_init_sysfs(struct kobject **hugepage_kobj)
{
	return 0;
}

static inline void hugepage_exit_sysfs(struct kobject *hugepage_kobj)
{
}
#endif /* CONFIG_SYSFS */

static int __init hugepage_init(void)
{
	int err;
	struct kobject *hugepage_kobj;

	if (!has_transparent_hugepage()) {
		transparent_hugepage_flags = 0;
		return -EINVAL;
	}

	/*
	 * hugepages can't be allocated by the buddy allocator
	 */
	MAYBE_BUILD_BUG_ON(HPAGE_PMD_ORDER >= MAX_ORDER);
	/*
	 * we use page->mapping and page->index in second tail page
	 * as list_head: assuming THP order >= 2
	 */
	MAYBE_BUILD_BUG_ON(HPAGE_PMD_ORDER < 2);

	err = hugepage_init_sysfs(&hugepage_kobj);
	if (err)
		goto err_sysfs;

	err = khugepaged_init();
	if (err)
		goto err_slab;

	err = register_shrinker(&huge_zero_page_shrinker);
	if (err)
		goto err_hzp_shrinker;
	err = register_shrinker(&deferred_split_shrinker);
	if (err)
		goto err_split_shrinker;

	/*
	 * By default disable transparent hugepages on smaller systems,
	 * where the extra memory used could hurt more than TLB overhead
	 * is likely to save.  The admin can still enable it through /sys.
	 */
	if (totalram_pages < (512 << (20 - PAGE_SHIFT))) {
		transparent_hugepage_flags = 0;
		return 0;
	}

	err = start_stop_khugepaged();
	if (err)
		goto err_khugepaged;

	return 0;
err_khugepaged:
	unregister_shrinker(&deferred_split_shrinker);
err_split_shrinker:
	unregister_shrinker(&huge_zero_page_shrinker);
err_hzp_shrinker:
	khugepaged_destroy();
err_slab:
	hugepage_exit_sysfs(hugepage_kobj);
err_sysfs:
	return err;
}
subsys_initcall(hugepage_init);

static int __init setup_transparent_hugepage(char *str)
{
	int ret = 0;
	if (!str)
		goto out;
	if (!strcmp(str, "always")) {
		set_bit(TRANSPARENT_HUGEPAGE_FLAG,
			&transparent_hugepage_flags);
		clear_bit(TRANSPARENT_HUGEPAGE_REQ_MADV_FLAG,
			  &transparent_hugepage_flags);
		ret = 1;
	} else if (!strcmp(str, "madvise")) {
		clear_bit(TRANSPARENT_HUGEPAGE_FLAG,
			  &transparent_hugepage_flags);
		set_bit(TRANSPARENT_HUGEPAGE_REQ_MADV_FLAG,
			&transparent_hugepage_flags);
		ret = 1;
	} else if (!strcmp(str, "never")) {
		clear_bit(TRANSPARENT_HUGEPAGE_FLAG,
			  &transparent_hugepage_flags);
		clear_bit(TRANSPARENT_HUGEPAGE_REQ_MADV_FLAG,
			  &transparent_hugepage_flags);
		ret = 1;
	}
out:
	if (!ret)
		pr_warn("transparent_hugepage= cannot parse, ignored\n");
	return ret;
}
__setup("transparent_hugepage=", setup_transparent_hugepage);

pmd_t maybe_pmd_mkwrite(pmd_t pmd, struct vm_area_struct *vma)
{
	if (likely(vma->vm_flags & VM_WRITE))
		pmd = pmd_mkwrite(pmd);
	return pmd;
}

static inline struct list_head *page_deferred_list(struct page *page)
{
	/*
	 * ->lru in the tail pages is occupied by compound_head.
	 * Let's use ->mapping + ->index in the second tail page as list_head.
	 */
	return (struct list_head *)&page[2].mapping;
}

void prep_transhuge_page(struct page *page)
{
	/*
	 * we use page->mapping and page->indexlru in second tail page
	 * as list_head: assuming THP order >= 2
	 */

	INIT_LIST_HEAD(page_deferred_list(page));
	set_compound_page_dtor(page, TRANSHUGE_PAGE_DTOR);
}

static unsigned long __thp_get_unmapped_area(struct file *filp,
		unsigned long addr, unsigned long len,
		loff_t off, unsigned long flags, unsigned long size)
{
	loff_t off_end = off + len;
	loff_t off_align = round_up(off, size);
	unsigned long len_pad, ret;

	if (off_end <= off_align || (off_end - off_align) < size)
		return 0;

	len_pad = len + size;
	if (len_pad < len || (off + len_pad) < off)
		return 0;

	ret = current->mm->get_unmapped_area(filp, addr, len_pad,
					      off >> PAGE_SHIFT, flags);

	/*
	 * The failure might be due to length padding. The caller will retry
	 * without the padding.
	 */
	if (IS_ERR_VALUE(ret))
		return 0;

	/*
	 * Do not try to align to THP boundary if allocation at the address
	 * hint succeeds.
	 */
	if (ret == addr)
		return addr;

	ret += (off - ret) & (size - 1);
	return ret;
}

unsigned long thp_get_unmapped_area(struct file *filp, unsigned long addr,
		unsigned long len, unsigned long pgoff, unsigned long flags)
{
	unsigned long ret;
	loff_t off = (loff_t)pgoff << PAGE_SHIFT;

	if (!IS_DAX(filp->f_mapping->host) || !IS_ENABLED(CONFIG_FS_DAX_PMD))
		goto out;

	ret = __thp_get_unmapped_area(filp, addr, len, off, flags, PMD_SIZE);
	if (ret)
		return ret;
out:
	return current->mm->get_unmapped_area(filp, addr, len, pgoff, flags);
}
EXPORT_SYMBOL_GPL(thp_get_unmapped_area);

static int __do_huge_pmd_anonymous_page(struct vm_fault *vmf, struct page *page,
		gfp_t gfp)
{
	struct vm_area_struct *vma = vmf->vma;
	struct mem_cgroup *memcg;
	pgtable_t pgtable;
	unsigned long haddr = vmf->address & HPAGE_PMD_MASK;
	int ret = 0;

	VM_BUG_ON_PAGE(!PageCompound(page), page);

	if (mem_cgroup_try_charge(page, vma->vm_mm, gfp | __GFP_NORETRY, &memcg,
				  true)) {
		put_page(page);
		count_vm_event(THP_FAULT_FALLBACK);
		return VM_FAULT_FALLBACK;
	}

	pgtable = pte_alloc_one(vma->vm_mm, haddr);
	if (unlikely(!pgtable)) {
		ret = VM_FAULT_OOM;
		goto release;
	}

	clear_huge_page(page, vmf->address, HPAGE_PMD_NR);
	/*
	 * The memory barrier inside __SetPageUptodate makes sure that
	 * clear_huge_page writes become visible before the set_pmd_at()
	 * write.
	 */
	__SetPageUptodate(page);

	vmf->ptl = pmd_lock(vma->vm_mm, vmf->pmd);
	if (unlikely(!pmd_none(*vmf->pmd))) {
		goto unlock_release;
	} else {
		pmd_t entry;

		ret = check_stable_address_space(vma->vm_mm);
		if (ret)
			goto unlock_release;

		/* Deliver the page fault to userland */
		if (userfaultfd_missing(vma)) {
			int ret;

			spin_unlock(vmf->ptl);
			mem_cgroup_cancel_charge(page, memcg, true);
			put_page(page);
			pte_free(vma->vm_mm, pgtable);
			ret = handle_userfault(vmf, VM_UFFD_MISSING);
			VM_BUG_ON(ret & VM_FAULT_FALLBACK);
			return ret;
		}

		entry = mk_huge_pmd(page, vma->vm_page_prot);
		entry = maybe_pmd_mkwrite(pmd_mkdirty(entry), vma);
		page_add_new_anon_rmap(page, vma, haddr, true);
		mem_cgroup_commit_charge(page, memcg, false, true);
		lru_cache_add_active_or_unevictable(page, vma);
		pgtable_trans_huge_deposit(vma->vm_mm, vmf->pmd, pgtable);
		set_pmd_at(vma->vm_mm, haddr, vmf->pmd, entry);
		add_mm_counter(vma->vm_mm, MM_ANONPAGES, HPAGE_PMD_NR);
		atomic_long_inc(&vma->vm_mm->nr_ptes);
		spin_unlock(vmf->ptl);
		count_vm_event(THP_FAULT_ALLOC);
	}

	return 0;
unlock_release:
	spin_unlock(vmf->ptl);
release:
	if (pgtable)
		pte_free(vma->vm_mm, pgtable);
	mem_cgroup_cancel_charge(page, memcg, true);
	put_page(page);
	return ret;

}

/*
 * always: directly stall for all thp allocations
 * defer: wake kswapd and fail if not immediately available
 * defer+madvise: wake kswapd and directly stall for MADV_HUGEPAGE, otherwise
 *		  fail if not immediately available
 * madvise: directly stall for MADV_HUGEPAGE, otherwise fail if not immediately
 *	    available
 * never: never stall for any thp allocation
 */
static inline gfp_t alloc_hugepage_direct_gfpmask(struct vm_area_struct *vma)
{
	const bool vma_madvised = !!(vma->vm_flags & VM_HUGEPAGE);

	if (test_bit(TRANSPARENT_HUGEPAGE_DEFRAG_DIRECT_FLAG, &transparent_hugepage_flags))
		return GFP_TRANSHUGE | (vma_madvised ? 0 : __GFP_NORETRY);
	if (test_bit(TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_FLAG, &transparent_hugepage_flags))
		return GFP_TRANSHUGE_LIGHT | __GFP_KSWAPD_RECLAIM;
	if (test_bit(TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_OR_MADV_FLAG, &transparent_hugepage_flags))
		return GFP_TRANSHUGE_LIGHT | (vma_madvised ? __GFP_DIRECT_RECLAIM :
							     __GFP_KSWAPD_RECLAIM);
	if (test_bit(TRANSPARENT_HUGEPAGE_DEFRAG_REQ_MADV_FLAG, &transparent_hugepage_flags))
		return GFP_TRANSHUGE_LIGHT | (vma_madvised ? __GFP_DIRECT_RECLAIM :
							     0);
	return GFP_TRANSHUGE_LIGHT;
}

/* Caller must hold page table lock. */
static bool set_huge_zero_page(pgtable_t pgtable, struct mm_struct *mm,
		struct vm_area_struct *vma, unsigned long haddr, pmd_t *pmd,
		struct page *zero_page)
{
	pmd_t entry;
	if (!pmd_none(*pmd))
		return false;
	entry = mk_pmd(zero_page, vma->vm_page_prot);
	entry = pmd_mkhuge(entry);
	if (pgtable)
		pgtable_trans_huge_deposit(mm, pmd, pgtable);
	set_pmd_at(mm, haddr, pmd, entry);
	atomic_long_inc(&mm->nr_ptes);
	return true;
}

int do_huge_pmd_anonymous_page(struct vm_fault *vmf)
{
	struct vm_area_struct *vma = vmf->vma;
	gfp_t gfp;
	struct page *page;
	unsigned long haddr = vmf->address & HPAGE_PMD_MASK;

	if (haddr < vma->vm_start || haddr + HPAGE_PMD_SIZE > vma->vm_end)
		return VM_FAULT_FALLBACK;
	if (unlikely(anon_vma_prepare(vma)))
		return VM_FAULT_OOM;
	if (unlikely(khugepaged_enter(vma, vma->vm_flags)))
		return VM_FAULT_OOM;
	if (!(vmf->flags & FAULT_FLAG_WRITE) &&
			!mm_forbids_zeropage(vma->vm_mm) &&
			transparent_hugepage_use_zero_page()) {
		pgtable_t pgtable;
		struct page *zero_page;
		bool set;
		int ret;
		pgtable = pte_alloc_one(vma->vm_mm, haddr);
		if (unlikely(!pgtable))
			return VM_FAULT_OOM;
		zero_page = mm_get_huge_zero_page(vma->vm_mm);
		if (unlikely(!zero_page)) {
			pte_free(vma->vm_mm, pgtable);
			count_vm_event(THP_FAULT_FALLBACK);
			return VM_FAULT_FALLBACK;
		}
		vmf->ptl = pmd_lock(vma->vm_mm, vmf->pmd);
		ret = 0;
		set = false;
		if (pmd_none(*vmf->pmd)) {
			ret = check_stable_address_space(vma->vm_mm);
			if (ret) {
				spin_unlock(vmf->ptl);
			} else if (userfaultfd_missing(vma)) {
				spin_unlock(vmf->ptl);
				ret = handle_userfault(vmf, VM_UFFD_MISSING);
				VM_BUG_ON(ret & VM_FAULT_FALLBACK);
			} else {
				set_huge_zero_page(pgtable, vma->vm_mm, vma,
						   haddr, vmf->pmd, zero_page);
				spin_unlock(vmf->ptl);
				set = true;
			}
		} else
			spin_unlock(vmf->ptl);
		if (!set)
			pte_free(vma->vm_mm, pgtable);
		return ret;
	}
	gfp = alloc_hugepage_direct_gfpmask(vma);
	page = alloc_hugepage_vma(gfp, vma, haddr, HPAGE_PMD_ORDER);
	if (unlikely(!page)) {
		count_vm_event(THP_FAULT_FALLBACK);
		return VM_FAULT_FALLBACK;
	}
	prep_transhuge_page(page);
	return __do_huge_pmd_anonymous_page(vmf, page, gfp);
}

static void insert_pfn_pmd(struct vm_area_struct *vma, unsigned long addr,
		pmd_t *pmd, pfn_t pfn, pgprot_t prot, bool write,
		pgtable_t pgtable)
{
	struct mm_struct *mm = vma->vm_mm;
	pmd_t entry;
	spinlock_t *ptl;

	ptl = pmd_lock(mm, pmd);
	entry = pmd_mkhuge(pfn_t_pmd(pfn, prot));
	if (pfn_t_devmap(pfn))
		entry = pmd_mkdevmap(entry);
	if (write) {
		entry = pmd_mkyoung(pmd_mkdirty(entry));
		entry = maybe_pmd_mkwrite(entry, vma);
	}

	if (pgtable) {
		pgtable_trans_huge_deposit(mm, pmd, pgtable);
		atomic_long_inc(&mm->nr_ptes);
	}

	set_pmd_at(mm, addr, pmd, entry);
	update_mmu_cache_pmd(vma, addr, pmd);
	spin_unlock(ptl);
}

int vmf_insert_pfn_pmd(struct vm_area_struct *vma, unsigned long addr,
			pmd_t *pmd, pfn_t pfn, bool write)
{
	pgprot_t pgprot = vma->vm_page_prot;
	pgtable_t pgtable = NULL;
	/*
	 * If we had pmd_special, we could avoid all these restrictions,
	 * but we need to be consistent with PTEs and architectures that
	 * can't support a 'special' bit.
	 */
	BUG_ON(!(vma->vm_flags & (VM_PFNMAP|VM_MIXEDMAP)));
	BUG_ON((vma->vm_flags & (VM_PFNMAP|VM_MIXEDMAP)) ==
						(VM_PFNMAP|VM_MIXEDMAP));
	BUG_ON((vma->vm_flags & VM_PFNMAP) && is_cow_mapping(vma->vm_flags));
	BUG_ON(!pfn_t_devmap(pfn));

	if (addr < vma->vm_start || addr >= vma->vm_end)
		return VM_FAULT_SIGBUS;

	if (arch_needs_pgtable_deposit()) {
		pgtable = pte_alloc_one(vma->vm_mm, addr);
		if (!pgtable)
			return VM_FAULT_OOM;
	}

	track_pfn_insert(vma, &pgprot, pfn);

	insert_pfn_pmd(vma, addr, pmd, pfn, pgprot, write, pgtable);
	return VM_FAULT_NOPAGE;
}
EXPORT_SYMBOL_GPL(vmf_insert_pfn_pmd);

#ifdef CONFIG_HAVE_ARCH_TRANSPARENT_HUGEPAGE_PUD
static pud_t maybe_pud_mkwrite(pud_t pud, struct vm_area_struct *vma)
{
	if (likely(vma->vm_flags & VM_WRITE))
		pud = pud_mkwrite(pud);
	return pud;
}

static void insert_pfn_pud(struct vm_area_struct *vma, unsigned long addr,
		pud_t *pud, pfn_t pfn, pgprot_t prot, bool write)
{
	struct mm_struct *mm = vma->vm_mm;
	pud_t entry;
	spinlock_t *ptl;

	ptl = pud_lock(mm, pud);
	entry = pud_mkhuge(pfn_t_pud(pfn, prot));
	if (pfn_t_devmap(pfn))
		entry = pud_mkdevmap(entry);
	if (write) {
		entry = pud_mkyoung(pud_mkdirty(entry));
		entry = maybe_pud_mkwrite(entry, vma);
	}
	set_pud_at(mm, addr, pud, entry);
	update_mmu_cache_pud(vma, addr, pud);
	spin_unlock(ptl);
}

int vmf_insert_pfn_pud(struct vm_area_struct *vma, unsigned long addr,
			pud_t *pud, pfn_t pfn, bool write)
{
	pgprot_t pgprot = vma->vm_page_prot;
	/*
	 * If we had pud_special, we could avoid all these restrictions,
	 * but we need to be consistent with PTEs and architectures that
	 * can't support a 'special' bit.
	 */
	BUG_ON(!(vma->vm_flags & (VM_PFNMAP|VM_MIXEDMAP)));
	BUG_ON((vma->vm_flags & (VM_PFNMAP|VM_MIXEDMAP)) ==
						(VM_PFNMAP|VM_MIXEDMAP));
	BUG_ON((vma->vm_flags & VM_PFNMAP) && is_cow_mapping(vma->vm_flags));
	BUG_ON(!pfn_t_devmap(pfn));

	if (addr < vma->vm_start || addr >= vma->vm_end)
		return VM_FAULT_SIGBUS;

	track_pfn_insert(vma, &pgprot, pfn);

	insert_pfn_pud(vma, addr, pud, pfn, pgprot, write);
	return VM_FAULT_NOPAGE;
}
EXPORT_SYMBOL_GPL(vmf_insert_pfn_pud);
#endif /* CONFIG_HAVE_ARCH_TRANSPARENT_HUGEPAGE_PUD */

static void touch_pmd(struct vm_area_struct *vma, unsigned long addr,
		pmd_t *pmd, int flags)
{
	pmd_t _pmd;

	_pmd = pmd_mkyoung(*pmd);
	if (flags & FOLL_WRITE)
		_pmd = pmd_mkdirty(_pmd);
	if (pmdp_set_access_flags(vma, addr & HPAGE_PMD_MASK,
				pmd, _pmd, flags & FOLL_WRITE))
		update_mmu_cache_pmd(vma, addr, pmd);
}

struct page *follow_devmap_pmd(struct vm_area_struct *vma, unsigned long addr,
		pmd_t *pmd, int flags)
{
	unsigned long pfn = pmd_pfn(*pmd);
	struct mm_struct *mm = vma->vm_mm;
	struct dev_pagemap *pgmap;
	struct page *page;

	assert_spin_locked(pmd_lockptr(mm, pmd));

	/*
	 * When we COW a devmap PMD entry, we split it into PTEs, so we should
	 * not be in this function with `flags & FOLL_COW` set.
	 */
	WARN_ONCE(flags & FOLL_COW, "mm: In follow_devmap_pmd with FOLL_COW set");

	if (flags & FOLL_WRITE && !pmd_write(*pmd))
		return NULL;

	if (pmd_present(*pmd) && pmd_devmap(*pmd))
		/* pass */;
	else
		return NULL;

	if (flags & FOLL_TOUCH)
		touch_pmd(vma, addr, pmd, flags);

	/*
	 * device mapped pages can only be returned if the
	 * caller will manage the page reference count.
	 */
	if (!(flags & FOLL_GET))
		return ERR_PTR(-EEXIST);

	pfn += (addr & ~PMD_MASK) >> PAGE_SHIFT;
	pgmap = get_dev_pagemap(pfn, NULL);
	if (!pgmap)
		return ERR_PTR(-EFAULT);
	page = pfn_to_page(pfn);
	get_page(page);
	put_dev_pagemap(pgmap);

	return page;
}

int copy_huge_pmd(struct mm_struct *dst_mm, struct mm_struct *src_mm,
		  pmd_t *dst_pmd, pmd_t *src_pmd, unsigned long addr,
		  struct vm_area_struct *vma)
{
	spinlock_t *dst_ptl, *src_ptl;
	struct page *src_page;
	pmd_t pmd;
	pgtable_t pgtable = NULL;
	int ret = -ENOMEM;

	/* Skip if can be re-fill on fault */
	if (!vma_is_anonymous(vma))
		return 0;

	pgtable = pte_alloc_one(dst_mm, addr);
	if (unlikely(!pgtable))
		goto out;

	dst_ptl = pmd_lock(dst_mm, dst_pmd);
	src_ptl = pmd_lockptr(src_mm, src_pmd);
	spin_lock_nested(src_ptl, SINGLE_DEPTH_NESTING);

	ret = -EAGAIN;
	pmd = *src_pmd;

#ifdef CONFIG_ARCH_ENABLE_THP_MIGRATION
	if (unlikely(is_swap_pmd(pmd))) {
		swp_entry_t entry = pmd_to_swp_entry(pmd);

		VM_BUG_ON(!is_pmd_migration_entry(pmd));
		if (is_write_migration_entry(entry)) {
			make_migration_entry_read(&entry);
			pmd = swp_entry_to_pmd(entry);
			if (pmd_swp_soft_dirty(*src_pmd))
				pmd = pmd_swp_mksoft_dirty(pmd);
			set_pmd_at(src_mm, addr, src_pmd, pmd);
		}
		add_mm_counter(dst_mm, MM_ANONPAGES, HPAGE_PMD_NR);
		atomic_long_inc(&dst_mm->nr_ptes);
		pgtable_trans_huge_deposit(dst_mm, dst_pmd, pgtable);
		set_pmd_at(dst_mm, addr, dst_pmd, pmd);
		ret = 0;
		goto out_unlock;
	}
#endif

	if (unlikely(!pmd_trans_huge(pmd))) {
		pte_free(dst_mm, pgtable);
		goto out_unlock;
	}
	/*
	 * When page table lock is held, the huge zero pmd should not be
	 * under splitting since we don't split the page itself, only pmd to
	 * a page table.
	 */
	if (is_huge_zero_pmd(pmd)) {
		struct page *zero_page;
		/*
		 * get_huge_zero_page() will never allocate a new page here,
		 * since we already have a zero page to copy. It just takes a
		 * reference.
		 */
		zero_page = mm_get_huge_zero_page(dst_mm);
		set_huge_zero_page(pgtable, dst_mm, vma, addr, dst_pmd,
				zero_page);
		ret = 0;
		goto out_unlock;
	}

	src_page = pmd_page(pmd);
	VM_BUG_ON_PAGE(!PageHead(src_page), src_page);
	get_page(src_page);
	page_dup_rmap(src_page, true);
	add_mm_counter(dst_mm, MM_ANONPAGES, HPAGE_PMD_NR);
	atomic_long_inc(&dst_mm->nr_ptes);
	pgtable_trans_huge_deposit(dst_mm, dst_pmd, pgtable);

	pmdp_set_wrprotect(src_mm, addr, src_pmd);
	pmd = pmd_mkold(pmd_wrprotect(pmd));
	set_pmd_at(dst_mm, addr, dst_pmd, pmd);

	ret = 0;
out_unlock:
	spin_unlock(src_ptl);
	spin_unlock(dst_ptl);
out:
	return ret;
}

#ifdef CONFIG_HAVE_ARCH_TRANSPARENT_HUGEPAGE_PUD
static void touch_pud(struct vm_area_struct *vma, unsigned long addr,
		pud_t *pud, int flags)
{
	pud_t _pud;

	_pud = pud_mkyoung(*pud);
	if (flags & FOLL_WRITE)
		_pud = pud_mkdirty(_pud);
	if (pudp_set_access_flags(vma, addr & HPAGE_PUD_MASK,
				pud, _pud, flags & FOLL_WRITE))
		update_mmu_cache_pud(vma, addr, pud);
}

struct page *follow_devmap_pud(struct vm_area_struct *vma, unsigned long addr,
		pud_t *pud, int flags)
{
	unsigned long pfn = pud_pfn(*pud);
	struct mm_struct *mm = vma->vm_mm;
	struct dev_pagemap *pgmap;
	struct page *page;

	assert_spin_locked(pud_lockptr(mm, pud));

	if (flags & FOLL_WRITE && !pud_write(*pud))
		return NULL;

	if (pud_present(*pud) && pud_devmap(*pud))
		/* pass */;
	else
		return NULL;

	if (flags & FOLL_TOUCH)
		touch_pud(vma, addr, pud, flags);

	/*
	 * device mapped pages can only be returned if the
	 * caller will manage the page reference count.
	 */
	if (!(flags & FOLL_GET))
		return ERR_PTR(-EEXIST);

	pfn += (addr & ~PUD_MASK) >> PAGE_SHIFT;
	pgmap = get_dev_pagemap(pfn, NULL);
	if (!pgmap)
		return ERR_PTR(-EFAULT);
	page = pfn_to_page(pfn);
	get_page(page);
	put_dev_pagemap(pgmap);

	return page;
}

int copy_huge_pud(struct mm_struct *dst_mm, struct mm_struct *src_mm,
		  pud_t *dst_pud, pud_t *src_pud, unsigned long addr,
		  struct vm_area_struct *vma)
{
	spinlock_t *dst_ptl, *src_ptl;
	pud_t pud;
	int ret;

	dst_ptl = pud_lock(dst_mm, dst_pud);
	src_ptl = pud_lockptr(src_mm, src_pud);
	spin_lock_nested(src_ptl, SINGLE_DEPTH_NESTING);

	ret = -EAGAIN;
	pud = *src_pud;
	if (unlikely(!pud_trans_huge(pud) && !pud_devmap(pud)))
		goto out_unlock;

	/*
	 * When page table lock is held, the huge zero pud should not be
	 * under splitting since we don't split the page itself, only pud to
	 * a page table.
	 */
	if (is_huge_zero_pud(pud)) {
		/* No huge zero pud yet */
	}

	pudp_set_wrprotect(src_mm, addr, src_pud);
	pud = pud_mkold(pud_wrprotect(pud));
	set_pud_at(dst_mm, addr, dst_pud, pud);

	ret = 0;
out_unlock:
	spin_unlock(src_ptl);
	spin_unlock(dst_ptl);
	return ret;
}

void huge_pud_set_accessed(struct vm_fault *vmf, pud_t orig_pud)
{
	pud_t entry;
	unsigned long haddr;
	bool write = vmf->flags & FAULT_FLAG_WRITE;

	vmf->ptl = pud_lock(vmf->vma->vm_mm, vmf->pud);
	if (unlikely(!pud_same(*vmf->pud, orig_pud)))
		goto unlock;

	entry = pud_mkyoung(orig_pud);
	if (write)
		entry = pud_mkdirty(entry);
	haddr = vmf->address & HPAGE_PUD_MASK;
	if (pudp_set_access_flags(vmf->vma, haddr, vmf->pud, entry, write))
		update_mmu_cache_pud(vmf->vma, vmf->address, vmf->pud);

unlock:
	spin_unlock(vmf->ptl);
}
#endif /* CONFIG_HAVE_ARCH_TRANSPARENT_HUGEPAGE_PUD */

void huge_pmd_set_accessed(struct vm_fault *vmf, pmd_t orig_pmd)
{
	pmd_t entry;
	unsigned long haddr;
	bool write = vmf->flags & FAULT_FLAG_WRITE;

	vmf->ptl = pmd_lock(vmf->vma->vm_mm, vmf->pmd);
	if (unlikely(!pmd_same(*vmf->pmd, orig_pmd)))
		goto unlock;

	entry = pmd_mkyoung(orig_pmd);
	if (write)
		entry = pmd_mkdirty(entry);
	haddr = vmf->address & HPAGE_PMD_MASK;
	if (pmdp_set_access_flags(vmf->vma, haddr, vmf->pmd, entry, write))
		update_mmu_cache_pmd(vmf->vma, vmf->address, vmf->pmd);

unlock:
	spin_unlock(vmf->ptl);
}

static int do_huge_pmd_wp_page_fallback(struct vm_fault *vmf, pmd_t orig_pmd,
		struct page *page)
{
	struct vm_area_struct *vma = vmf->vma;
	unsigned long haddr = vmf->address & HPAGE_PMD_MASK;
	struct mem_cgroup *memcg;
	pgtable_t pgtable;
	pmd_t _pmd;
	int ret = 0, i;
	struct page **pages;
	unsigned long mmun_start;	/* For mmu_notifiers */
	unsigned long mmun_end;		/* For mmu_notifiers */

	pages = kmalloc(sizeof(struct page *) * HPAGE_PMD_NR,
			GFP_KERNEL);
	if (unlikely(!pages)) {
		ret |= VM_FAULT_OOM;
		goto out;
	}

	for (i = 0; i < HPAGE_PMD_NR; i++) {
		pages[i] = alloc_page_vma_node(GFP_HIGHUSER_MOVABLE, vma,
					       vmf->address, page_to_nid(page));
		if (unlikely(!pages[i] ||
			     mem_cgroup_try_charge(pages[i], vma->vm_mm,
				     GFP_KERNEL, &memcg, false))) {
			if (pages[i])
				put_page(pages[i]);
			while (--i >= 0) {
				memcg = (void *)page_private(pages[i]);
				set_page_private(pages[i], 0);
				mem_cgroup_cancel_charge(pages[i], memcg,
						false);
				put_page(pages[i]);
			}
			kfree(pages);
			ret |= VM_FAULT_OOM;
			goto out;
		}
		set_page_private(pages[i], (unsigned long)memcg);
	}

	for (i = 0; i < HPAGE_PMD_NR; i++) {
		copy_user_highpage(pages[i], page + i,
				   haddr + PAGE_SIZE * i, vma);
		__SetPageUptodate(pages[i]);
		cond_resched();
	}

	mmun_start = haddr;
	mmun_end   = haddr + HPAGE_PMD_SIZE;
	mmu_notifier_invalidate_range_start(vma->vm_mm, mmun_start, mmun_end);

	vmf->ptl = pmd_lock(vma->vm_mm, vmf->pmd);
	if (unlikely(!pmd_same(*vmf->pmd, orig_pmd)))
		goto out_free_pages;
	VM_BUG_ON_PAGE(!PageHead(page), page);

	pmdp_huge_clear_flush_notify(vma, haddr, vmf->pmd);
	/* leave pmd empty until pte is filled */

	pgtable = pgtable_trans_huge_withdraw(vma->vm_mm, vmf->pmd);
	pmd_populate(vma->vm_mm, &_pmd, pgtable);

	for (i = 0; i < HPAGE_PMD_NR; i++, haddr += PAGE_SIZE) {
		pte_t entry;
		entry = mk_pte(pages[i], vmf->vma_page_prot);
		entry = maybe_mkwrite(pte_mkdirty(entry), vmf->vma_flags);
		memcg = (void *)page_private(pages[i]);
		set_page_private(pages[i], 0);
		page_add_new_anon_rmap(pages[i], vmf->vma, haddr, false);
		mem_cgroup_commit_charge(pages[i], memcg, false, false);
		lru_cache_add_active_or_unevictable(pages[i], vma);
		vmf->pte = pte_offset_map(&_pmd, haddr);
		VM_BUG_ON(!pte_none(*vmf->pte));
		set_pte_at(vma->vm_mm, haddr, vmf->pte, entry);
		pte_unmap(vmf->pte);
	}
	kfree(pages);

	smp_wmb(); /* make pte visible before pmd */
	pmd_populate(vma->vm_mm, vmf->pmd, pgtable);
	page_remove_rmap(page, true);
	spin_unlock(vmf->ptl);

	mmu_notifier_invalidate_range_end(vma->vm_mm, mmun_start, mmun_end);

	ret |= VM_FAULT_WRITE;
	put_page(page);

out:
	return ret;

out_free_pages:
	spin_unlock(vmf->ptl);
	mmu_notifier_invalidate_range_end(vma->vm_mm, mmun_start, mmun_end);
	for (i = 0; i < HPAGE_PMD_NR; i++) {
		memcg = (void *)page_private(pages[i]);
		set_page_private(pages[i], 0);
		mem_cgroup_cancel_charge(pages[i], memcg, false);
		put_page(pages[i]);
	}
	kfree(pages);
	goto out;
}

int do_huge_pmd_wp_page(struct vm_fault *vmf, pmd_t orig_pmd)
{
	struct vm_area_struct *vma = vmf->vma;
	struct page *page = NULL, *new_page;
	struct mem_cgroup *memcg;
	unsigned long haddr = vmf->address & HPAGE_PMD_MASK;
	unsigned long mmun_start;	/* For mmu_notifiers */
	unsigned long mmun_end;		/* For mmu_notifiers */
	gfp_t huge_gfp;			/* for allocation and charge */
	int ret = 0;

	vmf->ptl = pmd_lockptr(vma->vm_mm, vmf->pmd);
	VM_BUG_ON_VMA(!vma->anon_vma, vma);
	if (is_huge_zero_pmd(orig_pmd))
		goto alloc;
	spin_lock(vmf->ptl);
	if (unlikely(!pmd_same(*vmf->pmd, orig_pmd)))
		goto out_unlock;

	page = pmd_page(orig_pmd);
	VM_BUG_ON_PAGE(!PageCompound(page) || !PageHead(page), page);
	/*
	 * We can only reuse the page if nobody else maps the huge page or it's
	 * part.
	 */
	if (!trylock_page(page)) {
		get_page(page);
		spin_unlock(vmf->ptl);
		lock_page(page);
		spin_lock(vmf->ptl);
		if (unlikely(!pmd_same(*vmf->pmd, orig_pmd))) {
			unlock_page(page);
			put_page(page);
			goto out_unlock;
		}
		put_page(page);
	}
	if (reuse_swap_page(page, NULL)) {
		pmd_t entry;
		entry = pmd_mkyoung(orig_pmd);
		entry = maybe_pmd_mkwrite(pmd_mkdirty(entry), vma);
		if (pmdp_set_access_flags(vma, haddr, vmf->pmd, entry,  1))
			update_mmu_cache_pmd(vma, vmf->address, vmf->pmd);
		ret |= VM_FAULT_WRITE;
		unlock_page(page);
		goto out_unlock;
	}
	unlock_page(page);
	get_page(page);
	spin_unlock(vmf->ptl);
alloc:
	if (transparent_hugepage_enabled(vma) &&
	    !transparent_hugepage_debug_cow()) {
		huge_gfp = alloc_hugepage_direct_gfpmask(vma);
		new_page = alloc_hugepage_vma(huge_gfp, vma, haddr, HPAGE_PMD_ORDER);
	} else
		new_page = NULL;

	if (likely(new_page)) {
		prep_transhuge_page(new_page);
	} else {
		if (!page) {
			split_huge_pmd(vma, vmf->pmd, vmf->address);
			ret |= VM_FAULT_FALLBACK;
		} else {
			ret = do_huge_pmd_wp_page_fallback(vmf, orig_pmd, page);
			if (ret & VM_FAULT_OOM) {
				split_huge_pmd(vma, vmf->pmd, vmf->address);
				ret |= VM_FAULT_FALLBACK;
			}
			put_page(page);
		}
		count_vm_event(THP_FAULT_FALLBACK);
		goto out;
	}

	if (unlikely(mem_cgroup_try_charge(new_page, vma->vm_mm,
				huge_gfp | __GFP_NORETRY, &memcg, true))) {
		put_page(new_page);
		split_huge_pmd(vma, vmf->pmd, vmf->address);
		if (page)
			put_page(page);
		ret |= VM_FAULT_FALLBACK;
		count_vm_event(THP_FAULT_FALLBACK);
		goto out;
	}

	count_vm_event(THP_FAULT_ALLOC);

	if (!page)
		clear_huge_page(new_page, vmf->address, HPAGE_PMD_NR);
	else
		copy_user_huge_page(new_page, page, haddr, vma, HPAGE_PMD_NR);
	__SetPageUptodate(new_page);

	mmun_start = haddr;
	mmun_end   = haddr + HPAGE_PMD_SIZE;
	mmu_notifier_invalidate_range_start(vma->vm_mm, mmun_start, mmun_end);

	spin_lock(vmf->ptl);
	if (page)
		put_page(page);
	if (unlikely(!pmd_same(*vmf->pmd, orig_pmd))) {
		spin_unlock(vmf->ptl);
		mem_cgroup_cancel_charge(new_page, memcg, true);
		put_page(new_page);
		goto out_mn;
	} else {
		pmd_t entry;
		entry = mk_huge_pmd(new_page, vma->vm_page_prot);
		entry = maybe_pmd_mkwrite(pmd_mkdirty(entry), vma);
		pmdp_huge_clear_flush_notify(vma, haddr, vmf->pmd);
		page_add_new_anon_rmap(new_page, vma, haddr, true);
		mem_cgroup_commit_charge(new_page, memcg, false, true);
		lru_cache_add_active_or_unevictable(new_page, vma);
		set_pmd_at(vma->vm_mm, haddr, vmf->pmd, entry);
		update_mmu_cache_pmd(vma, vmf->address, vmf->pmd);
		if (!page) {
			add_mm_counter(vma->vm_mm, MM_ANONPAGES, HPAGE_PMD_NR);
		} else {
			VM_BUG_ON_PAGE(!PageHead(page), page);
			page_remove_rmap(page, true);
			put_page(page);
		}
		ret |= VM_FAULT_WRITE;
	}
	spin_unlock(vmf->ptl);
out_mn:
	mmu_notifier_invalidate_range_end(vma->vm_mm, mmun_start, mmun_end);
out:
	return ret;
out_unlock:
	spin_unlock(vmf->ptl);
	return ret;
}

/*
 * FOLL_FORCE can write to even unwritable pmd's, but only
 * after we've gone through a COW cycle and they are dirty.
 */
static inline bool can_follow_write_pmd(pmd_t pmd, unsigned int flags)
{
	return pmd_write(pmd) ||
	       ((flags & FOLL_FORCE) && (flags & FOLL_COW) && pmd_dirty(pmd));
}

struct page *follow_trans_huge_pmd(struct vm_area_struct *vma,
				   unsigned long addr,
				   pmd_t *pmd,
				   unsigned int flags)
{
	struct mm_struct *mm = vma->vm_mm;
	struct page *page = NULL;

	assert_spin_locked(pmd_lockptr(mm, pmd));

	if (flags & FOLL_WRITE && !can_follow_write_pmd(*pmd, flags))
		goto out;

	/* Avoid dumping huge zero page */
	if ((flags & FOLL_DUMP) && is_huge_zero_pmd(*pmd))
		return ERR_PTR(-EFAULT);

	/* Full NUMA hinting faults to serialise migration in fault paths */
	if ((flags & FOLL_NUMA) && pmd_protnone(*pmd))
		goto out;

	page = pmd_page(*pmd);
	VM_BUG_ON_PAGE(!PageHead(page) && !is_zone_device_page(page), page);
	if (flags & FOLL_TOUCH)
		touch_pmd(vma, addr, pmd, flags);
	if ((flags & FOLL_MLOCK) && (vma->vm_flags & VM_LOCKED)) {
		/*
		 * We don't mlock() pte-mapped THPs. This way we can avoid
		 * leaking mlocked pages into non-VM_LOCKED VMAs.
		 *
		 * For anon THP:
		 *
		 * In most cases the pmd is the only mapping of the page as we
		 * break COW for the mlock() -- see gup_flags |= FOLL_WRITE for
		 * writable private mappings in populate_vma_page_range().
		 *
		 * The only scenario when we have the page shared here is if we
		 * mlocking read-only mapping shared over fork(). We skip
		 * mlocking such pages.
		 *
		 * For file THP:
		 *
		 * We can expect PageDoubleMap() to be stable under page lock:
		 * for file pages we set it in page_add_file_rmap(), which
		 * requires page to be locked.
		 */

		if (PageAnon(page) && compound_mapcount(page) != 1)
			goto skip_mlock;
		if (PageDoubleMap(page) || !page->mapping)
			goto skip_mlock;
		if (!trylock_page(page))
			goto skip_mlock;
		lru_add_drain();
		if (page->mapping && !PageDoubleMap(page))
			mlock_vma_page(page);
		unlock_page(page);
	}
skip_mlock:
	page += (addr & ~HPAGE_PMD_MASK) >> PAGE_SHIFT;
	VM_BUG_ON_PAGE(!PageCompound(page) && !is_zone_device_page(page), page);
	if (flags & FOLL_GET)
		get_page(page);

out:
	return page;
}

/* NUMA hinting page fault entry point for trans huge pmds */
int do_huge_pmd_numa_page(struct vm_fault *vmf, pmd_t pmd)
{
	struct vm_area_struct *vma = vmf->vma;
	struct anon_vma *anon_vma = NULL;
	struct page *page;
	unsigned long haddr = vmf->address & HPAGE_PMD_MASK;
	int page_nid = -1, this_nid = numa_node_id();
	int target_nid, last_cpupid = -1;
	bool page_locked;
	bool migrated = false;
	bool was_writable;
	int flags = 0;

	vmf->ptl = pmd_lock(vma->vm_mm, vmf->pmd);
	if (unlikely(!pmd_same(pmd, *vmf->pmd)))
		goto out_unlock;

	/*
	 * If there are potential migrations, wait for completion and retry
	 * without disrupting NUMA hinting information. Do not relock and
	 * check_same as the page may no longer be mapped.
	 */
	if (unlikely(pmd_trans_migrating(*vmf->pmd))) {
		page = pmd_page(*vmf->pmd);
		if (!get_page_unless_zero(page))
			goto out_unlock;
		spin_unlock(vmf->ptl);
		wait_on_page_locked(page);
		put_page(page);
		goto out;
	}

	page = pmd_page(pmd);
	BUG_ON(is_huge_zero_page(page));
	page_nid = page_to_nid(page);
	last_cpupid = page_cpupid_last(page);
	count_vm_numa_event(NUMA_HINT_FAULTS);
	if (page_nid == this_nid) {
		count_vm_numa_event(NUMA_HINT_FAULTS_LOCAL);
		flags |= TNF_FAULT_LOCAL;
	}

	/* See similar comment in do_numa_page for explanation */
	if (!pmd_savedwrite(pmd))
		flags |= TNF_NO_GROUP;

	/*
	 * Acquire the page lock to serialise THP migrations but avoid dropping
	 * page_table_lock if at all possible
	 */
	page_locked = trylock_page(page);
	target_nid = mpol_misplaced(page, vma, haddr);
	if (target_nid == -1) {
		/* If the page was locked, there are no parallel migrations */
		if (page_locked)
			goto clear_pmdnuma;
	}

	/* Migration could have started since the pmd_trans_migrating check */
	if (!page_locked) {
		page_nid = -1;
		if (!get_page_unless_zero(page))
			goto out_unlock;
		spin_unlock(vmf->ptl);
		wait_on_page_locked(page);
		put_page(page);
		goto out;
	}

	/*
	 * Page is misplaced. Page lock serialises migrations. Acquire anon_vma
	 * to serialises splits
	 */
	get_page(page);
	spin_unlock(vmf->ptl);
	anon_vma = page_lock_anon_vma_read(page);

	/* Confirm the PMD did not change while page_table_lock was released */
	spin_lock(vmf->ptl);
	if (unlikely(!pmd_same(pmd, *vmf->pmd))) {
		unlock_page(page);
		put_page(page);
		page_nid = -1;
		goto out_unlock;
	}

	/* Bail if we fail to protect against THP splits for any reason */
	if (unlikely(!anon_vma)) {
		put_page(page);
		page_nid = -1;
		goto clear_pmdnuma;
	}

	/*
	 * Since we took the NUMA fault, we must have observed the !accessible
	 * bit. Make sure all other CPUs agree with that, to avoid them
	 * modifying the page we're about to migrate.
	 *
	 * Must be done under PTL such that we'll observe the relevant
	 * inc_tlb_flush_pending().
	 *
	 * We are not sure a pending tlb flush here is for a huge page
	 * mapping or not. Hence use the tlb range variant
	 */
	if (mm_tlb_flush_pending(vma->vm_mm))
		flush_tlb_range(vma, haddr, haddr + HPAGE_PMD_SIZE);

	/*
	 * Migrate the THP to the requested node, returns with page unlocked
	 * and access rights restored.
	 */
	spin_unlock(vmf->ptl);

	migrated = migrate_misplaced_transhuge_page(vma->vm_mm, vma,
				vmf->pmd, pmd, vmf->address, page, target_nid);
	if (migrated) {
		flags |= TNF_MIGRATED;
		page_nid = target_nid;
	} else
		flags |= TNF_MIGRATE_FAIL;

	goto out;
clear_pmdnuma:
	BUG_ON(!PageLocked(page));
	was_writable = pmd_savedwrite(pmd);
	pmd = pmd_modify(pmd, vma->vm_page_prot);
	pmd = pmd_mkyoung(pmd);
	if (was_writable)
		pmd = pmd_mkwrite(pmd);
	set_pmd_at(vma->vm_mm, haddr, vmf->pmd, pmd);
	update_mmu_cache_pmd(vma, vmf->address, vmf->pmd);
	unlock_page(page);
out_unlock:
	spin_unlock(vmf->ptl);

out:
	if (anon_vma)
		page_unlock_anon_vma_read(anon_vma);

	if (page_nid != -1)
		task_numa_fault(last_cpupid, page_nid, HPAGE_PMD_NR,
				flags);

	return 0;
}

/*
 * Return true if we do MADV_FREE successfully on entire pmd page.
 * Otherwise, return false.
 */
bool madvise_free_huge_pmd(struct mmu_gather *tlb, struct vm_area_struct *vma,
		pmd_t *pmd, unsigned long addr, unsigned long next)
{
	spinlock_t *ptl;
	pmd_t orig_pmd;
	struct page *page;
	struct mm_struct *mm = tlb->mm;
	bool ret = false;

	tlb_remove_check_page_size_change(tlb, HPAGE_PMD_SIZE);

	ptl = pmd_trans_huge_lock(pmd, vma);
	if (!ptl)
		goto out_unlocked;

	orig_pmd = *pmd;
	if (is_huge_zero_pmd(orig_pmd))
		goto out;

	if (unlikely(!pmd_present(orig_pmd))) {
		VM_BUG_ON(thp_migration_supported() &&
				  !is_pmd_migration_entry(orig_pmd));
		goto out;
	}

	page = pmd_page(orig_pmd);
	/*
	 * If other processes are mapping this page, we couldn't discard
	 * the page unless they all do MADV_FREE so let's skip the page.
	 */
	if (page_mapcount(page) != 1)
		goto out;

	if (!trylock_page(page))
		goto out;

	/*
	 * If user want to discard part-pages of THP, split it so MADV_FREE
	 * will deactivate only them.
	 */
	if (next - addr != HPAGE_PMD_SIZE) {
		get_page(page);
		spin_unlock(ptl);
		split_huge_page(page);
		unlock_page(page);
		put_page(page);
		goto out_unlocked;
	}

	if (PageDirty(page))
		ClearPageDirty(page);
	unlock_page(page);

	if (pmd_young(orig_pmd) || pmd_dirty(orig_pmd)) {
		pmdp_invalidate(vma, addr, pmd);
		orig_pmd = pmd_mkold(orig_pmd);
		orig_pmd = pmd_mkclean(orig_pmd);

		set_pmd_at(mm, addr, pmd, orig_pmd);
		tlb_remove_pmd_tlb_entry(tlb, pmd, addr);
	}

	mark_page_lazyfree(page);
	ret = true;
out:
	spin_unlock(ptl);
out_unlocked:
	return ret;
}

static inline void zap_deposited_table(struct mm_struct *mm, pmd_t *pmd)
{
	pgtable_t pgtable;

	pgtable = pgtable_trans_huge_withdraw(mm, pmd);
	pte_free(mm, pgtable);
	atomic_long_dec(&mm->nr_ptes);
}

int zap_huge_pmd(struct mmu_gather *tlb, struct vm_area_struct *vma,
		 pmd_t *pmd, unsigned long addr)
{
	pmd_t orig_pmd;
	spinlock_t *ptl;

	tlb_remove_check_page_size_change(tlb, HPAGE_PMD_SIZE);

	ptl = __pmd_trans_huge_lock(pmd, vma);
	if (!ptl)
		return 0;
	/*
	 * For architectures like ppc64 we look at deposited pgtable
	 * when calling pmdp_huge_get_and_clear. So do the
	 * pgtable_trans_huge_withdraw after finishing pmdp related
	 * operations.
	 */
	orig_pmd = pmdp_huge_get_and_clear_full(tlb->mm, addr, pmd,
			tlb->fullmm);
	tlb_remove_pmd_tlb_entry(tlb, pmd, addr);
	if (vma_is_dax(vma)) {
		if (arch_needs_pgtable_deposit())
			zap_deposited_table(tlb->mm, pmd);
		spin_unlock(ptl);
		if (is_huge_zero_pmd(orig_pmd))
			tlb_remove_page_size(tlb, pmd_page(orig_pmd), HPAGE_PMD_SIZE);
	} else if (is_huge_zero_pmd(orig_pmd)) {
		zap_deposited_table(tlb->mm, pmd);
		spin_unlock(ptl);
		tlb_remove_page_size(tlb, pmd_page(orig_pmd), HPAGE_PMD_SIZE);
	} else {
		struct page *page = NULL;
		int flush_needed = 1;

		if (pmd_present(orig_pmd)) {
			page = pmd_page(orig_pmd);
			page_remove_rmap(page, true);
			VM_BUG_ON_PAGE(page_mapcount(page) < 0, page);
			VM_BUG_ON_PAGE(!PageHead(page), page);
		} else if (thp_migration_supported()) {
			swp_entry_t entry;

			VM_BUG_ON(!is_pmd_migration_entry(orig_pmd));
			entry = pmd_to_swp_entry(orig_pmd);
			page = pfn_to_page(swp_offset(entry));
			flush_needed = 0;
		} else
			WARN_ONCE(1, "Non present huge pmd without pmd migration enabled!");

		if (PageAnon(page)) {
			zap_deposited_table(tlb->mm, pmd);
			add_mm_counter(tlb->mm, MM_ANONPAGES, -HPAGE_PMD_NR);
		} else {
			if (arch_needs_pgtable_deposit())
				zap_deposited_table(tlb->mm, pmd);
			add_mm_counter(tlb->mm, MM_FILEPAGES, -HPAGE_PMD_NR);
		}

		spin_unlock(ptl);
		if (flush_needed)
			tlb_remove_page_size(tlb, page, HPAGE_PMD_SIZE);
	}
	return 1;
}

#ifndef pmd_move_must_withdraw
static inline int pmd_move_must_withdraw(spinlock_t *new_pmd_ptl,
					 spinlock_t *old_pmd_ptl,
					 struct vm_area_struct *vma)
{
	/*
	 * With split pmd lock we also need to move preallocated
	 * PTE page table if new_pmd is on different PMD page table.
	 *
	 * We also don't deposit and withdraw tables for file pages.
	 */
	return (new_pmd_ptl != old_pmd_ptl) && vma_is_anonymous(vma);
}
#endif

static pmd_t move_soft_dirty_pmd(pmd_t pmd)
{
#ifdef CONFIG_MEM_SOFT_DIRTY
	if (unlikely(is_pmd_migration_entry(pmd)))
		pmd = pmd_swp_mksoft_dirty(pmd);
	else if (pmd_present(pmd))
		pmd = pmd_mksoft_dirty(pmd);
#endif
	return pmd;
}

bool move_huge_pmd(struct vm_area_struct *vma, unsigned long old_addr,
		  unsigned long new_addr, unsigned long old_end,
		  pmd_t *old_pmd, pmd_t *new_pmd)
{
	spinlock_t *old_ptl, *new_ptl;
	pmd_t pmd;
	struct mm_struct *mm = vma->vm_mm;
	bool force_flush = false;

	if ((old_addr & ~HPAGE_PMD_MASK) ||
	    (new_addr & ~HPAGE_PMD_MASK) ||
	    old_end - old_addr < HPAGE_PMD_SIZE)
		return false;

	/*
	 * The destination pmd shouldn't be established, free_pgtables()
	 * should have release it.
	 */
	if (WARN_ON(!pmd_none(*new_pmd))) {
		VM_BUG_ON(pmd_trans_huge(*new_pmd));
		return false;
	}

	/*
	 * We don't have to worry about the ordering of src and dst
	 * ptlocks because exclusive mmap_sem prevents deadlock.
	 */
	old_ptl = __pmd_trans_huge_lock(old_pmd, vma);
	if (old_ptl) {
		new_ptl = pmd_lockptr(mm, new_pmd);
		if (new_ptl != old_ptl)
			spin_lock_nested(new_ptl, SINGLE_DEPTH_NESTING);
		pmd = pmdp_huge_get_and_clear(mm, old_addr, old_pmd);
		if (pmd_present(pmd))
			force_flush = true;
		VM_BUG_ON(!pmd_none(*new_pmd));

		if (pmd_move_must_withdraw(new_ptl, old_ptl, vma)) {
			pgtable_t pgtable;
			pgtable = pgtable_trans_huge_withdraw(mm, old_pmd);
			pgtable_trans_huge_deposit(mm, new_pmd, pgtable);
		}
		pmd = move_soft_dirty_pmd(pmd);
		set_pmd_at(mm, new_addr, new_pmd, pmd);
		if (force_flush)
			flush_tlb_range(vma, old_addr, old_addr + PMD_SIZE);
		if (new_ptl != old_ptl)
			spin_unlock(new_ptl);
		spin_unlock(old_ptl);
		return true;
	}
	return false;
}

/*
 * Returns
 * wbn_vma_@G
Ca_@G
Ca_@G
Ca_@G
Ca_@G
Ca_@G
Ca_@G
Ca_@G
Ca_@G
C	4+hge_l);
		}
		pmda}
		pmda}
		pmda}
			iDy(khkorig_pA_remove_check_page_size_change(tlb, HPAGE_PMD_SIZE);

	ptl = __pmd_trans_huge_lock(pmd, vma);
	if (!ptl)
		return 0;
	/*
	 * For architectures like ppc64 we look at deposited pgtable
	 * when cGE);

	if (test_bit(TRe_get_and_clear_fullDCfullDCfullDCfullDCfullDCfullDCfullDCfullDCfullDC	Fry(pmd)))
		pmwbire aa_@G
Ca_@G
C|G/* Skip if can be re-fill .pge = mm_get_hugfullDCfull(vmf->ve_check_pag|>n_SIZE);

	ptl = __pmd_trans_huge_lock(pmd, vma);
	tl = __tpag|>n_SIZE);

	ptl = __pm.-trans_huge_lock(.-ln64 we look at deposited pgtable
	 * when cGE);

	i:Parge(new_page, memcg, true);
		put_page(new_page);
		goto out_mn;
	} k at depout_unlock;
	}

	/* Bail if we fail to protect against THP splits for any reason */
	if (unlikely(!anon_vma)) {
		put_page(page);
		page_nid = -1;
		goto clear_pmdnuma;
	}

	/*
	 * Since we too
		page_nid = -1;
		goto c;

	pt.-trans_huge|<*
	 * Since we too
		page_.D->ptl);
			} else if (* when cGParge(new_pa|>equesage, mma)) {
		put(pmd_trans_huge(*new_wyS
C_wyS
C_wyS
C_wyS
C_wyS
C_wyS
C_wyS
C_wyS
C_wyS
C	4+gn:vmf->ptl);
		if (!wS
C_wyS
C_wyS
C_wyS
C_wyS
.DFAULT_NOPAGE;
}
EXPORT- see gup_flagiSyS
C_wyS
C_wyS
.DFAULT_NOPAGE;
}
EXPORT-ge zero +zero +zero +zero +zero +zero +zero +zero +zero +	
writentoo
		page_.D->ptl);
			} else if (* when cGParge(G+wr + Hw_pa|>equesalush_nCush_nCush_nCush_nCush_nCush_nCush_nCush_nCush_nC	4+_nC	4+_nC	4+_nC	4+_nC	4+_nC	4+_nC	4+_nC	4+_nFp=g(oriwg(oriwg(oriwg(oriwg(oriwg(oriwg(oriwg(oriwg(oriw	F, pmd);
wust_withdraw(new_ptl, old_ptl, vma)) {
			pgtable_wable_wable_wable_wable_wable_wable_wable_wable_w	m4+_nC	4+_nFp=g(orRATED;
		paable_wable_wable_wable_wab6ge_able_wable_wable_wab6ge_able_wable_wable_wab6ge_abe_pmd_tlocking/
	spin_D_SIappings i.s_hug;

out:
	ud;
	iatanC	4rn falnsigned;

oupmd =dMD_MD_SIE_PMl_MIGropping
	E_PMl/huge_NOhiing reD_SIappings ie)
		pmd id);
_withgut_unlocked:
E;
}
EXPORT-*
		 * The only scenario w,
			GFP_KERNELked:
E;
}
EXPORT-ps.
		 *
		 * For file THP:ge for explanationnf (paAGE;
}
EXPtlb, )
		;
	pmddown;
wust;
		spin)touc'ge_riwiPMllevant
	b ran_SIZappingd);
mitsavealnsig
clearracAIL;

	e_witDONTNEED= pmd_pichnC	4GE_PM;
	pmddown;
wust;
		spin):d_addr <	MIG0:ble_MIG1:d_adble_
	 * Since we toGE;
}
EXP=1)d_adble_
		} else if (thp_migratinly
 * )r, vmf-f (paga, t((ol )r, vmremovps.
geCompor, vmrpage(page);
		pe = NUL==t d(aw tables_wyS
 vmrp//unlock(ptl)mdd_adble_
aable_wableble_adble_
//uPAGE_PMre-gtable_trand_addr < _hug;aunlikructe_witDONTNEED	 *
gepage_debugm -1;
	a, oldn_SIZait= pmd_pichnt ale(page
{
	p);
}ld_addr < p	return 0;
	/*
)E_PMreobserld_addlse
		flavmf->vma, *
gud;
	iFp=g/__pmdt_vm_nustrub_HUGrdw)
	ut_unlocro +	
wr		Clearp	return 0;
	/*
	 * For architectxplanatioRec/* NUiFp=g/__pmdt_vm_n. ouche);iePMD_Mp	return 0;
	/*nt
	b ring
	 d_aupt pgtable);
	atomipk(pmd, vm For file_OOM;
		goto out;
	}

	for (i
	ptl = __pmd_ For file_OOM;
		goto o__pmd_

	for (EL);
	if (unlik>mm;
	s);
			e_.D->pr (i
	ptlwyS
.DFAULT_Nfile_OOM;
		goto o;
	struct mm

	for (iC_wyS
tl);
	if (unlreturn VM_FAULT_SIGBUS;

	if (arch_neelse if	 * should have releantry =wyS
.DFAULT_NORT- see gup_f_pmd is onlikely(!pmd_present( ERR_PTR(-EFAULT);

	/* Fullut_mn;
_mkdirty(entry);
mf->p NUlock gierias forhugea pgpM_FAULorig_pso lpageFullNo_vma_atUlocitid != -1)mkdirty(entry);
mf->p Nge_to_ rablold_d != -1)
		tabl
f (puentry pomkdirty(entry);(page)))
		s|= TNFpuentr
		pm out; {
		put(pm_wyS
C_wyS
C	4+gn:vmf->pts_huge_ze);
	return ret;
}

void huge_pud_set_accessd_trans_HINT_FAULTS_LOCAL);
		flape_change(t
	 * under splittin = NUL_lock(ppage);
		pe = NUL	vmf->pk(pmret = -ENOMEyS
C_wyS
C_swp_mksmd_present( ERR_PTR(-EFAUlb->mm
	/* Fullut_mn;
_e, we cok gieriasuforhugea pgpM__pageorig_pso lpageFullNo_vma_atUlocitid != -1)e, wge_to_ rablold_d != -1)
		tabl(puentry pomkdiFullty(entry);(page)))
		s|= TNFpuentr
		pm out; {
		put(pm_wyu
C_wyS
C	4+gn:vmf->);

	return);
	return ret;
}

void huge_pud_set_accessd_traIXEDMAP)) ==
				OCAL);
		flapu_change(t
	 * un
		goto unlock;_pud);_locc_ptl = pud_lockyS
C_wyS
C_swp_mksmd_present( ERR_PTR(-EFAUlb->mm
	/ap(*pud))
		/* pass */;
	else
		return NULL;

	iflb_remove_pageusize(tlb, pmd_page(orig_pmd), HPAGE_PMD_SIZE);
	} else if );

	returnro_pmd(orig_pmd)) {
		za
	if (unlikelable(tlb->mm, pmd);
		_wyS
C_wyu
C_wyS
C	4+gn:vmf->);e {
		struct page *page = NULL;
		int flush_needed = 1;

		if (pmd_present(orig_pmd)) {
			page = pmd_page(orig_pmdu;
			page_remove_rmap(page, true);
			VM_BUG_ON_PAGE(page_mapcount(page) < 0, pageu;
			VM_BUG_ON_PAGE(!PageHead(page), paGEPAGE_P else if (thp_migration_supported()) {
			uwp_entry_t entry;

			VM_BUG_ON(!us_pmd_migration_enury(orig_pmd));
			entry = pmd_to_swpdirty(pmd);
#endif
	

unloappings ierty(_pmdck(vmf->pCONFIG_MEM

unlorty(_pmdf_mlock;have  
	stgtableck(vmf->eels(ned long old_addr,
	 (flags & FO__uct vm_area_ ==
			etoo
		page_nid = -1;
		goto c;
);

	returs_huge|<*
	 * Sinhd)) {
		z	} else ifh_move_must_with don't s;a);
		if (pmdp_sOCAL);
			upda>ew_page, vms;a);
		if (pmdp_sOCAL);
	, vm<

out:
	if (anonU_huge_e, vms;a);
		if (p))
		goto unlock;_pud);
	s!cc_ptl = pud_lock;
		e_add_new_anon_rmapSPLITonU_e(origuritable pmd's, but only
 * after we've 	return E & FO__uct vm_area_ =oo
		page_nid = -1;
		goto c;
);

	returs_huge|<*
	 * Sinage_prote_pud_set_accessd_tran when cGParge(new_pa|>equesage, mma)	page = pmd_page(pmd);pmd);

unlock:
	spin_unloc
		if (!page) {
			add_mm_counter(vma-	if (!ptl)

out:
	if (anonU_huge_);IXEDMAP)) ==
					flapu_change(trig_pud)))
		goto unlock;_pud);
	s!cc_ptl = pud_lockmm_struct *mm,	__uct vm_area_ ==
			etoto c;
);age(page);uge_get_and_clear_full(tl flags))
		goto out;

	/* Avoid dum	if (!ptl)

out:
	if (anonU_huge_);Isigned long haddr = vmf->address & HPAGE_PMD_MASK;
	struct (flags & FO__uct vm_area_migratinVM_LOCKED)) {
		/*
		 * We don't ml	page = pmd_page(pm_dax(vma)) {
		if(VM_PFNMAP|VM_MIXEDMAP)));
	BUG_ON((ve(pages[i], memcg,
						false);
			ing or nes);

	smp_wmb(); /* make pte visible bunwritable pmd's, but only
 * after we've hitectxpdeposit())
			zap_deposited_table(tlb->mm, pmd);
	long haddr =ue);
	spin_unlock(vmf->ptl);

	mmu_notifier_invalidate_range_end(vma->vm_mm, mmun_start,*rea_struct;ile_OOM;
		g notr =uya_migrafnfh_mov):
	spin_unlock(vmf->ptl);
	returree_pa	if (pm		pgtable_w	}
	kfree(pages);
	goto out;
}

int do_huge_pmd_wp_page(strm_fault *vmf, pmd_t 	if (!ptl)
rea_struct *vma = vmf->vmtruct pagenew_page;
	struct mem_cgroup *memcg;
	unsigned long haddr =>mm, pmn_unlock(vmf,
	 (flags & FO__uct vm_area_m==
			etoo
		page_nid = -1;
		goto c;

	pt.-trans_huge|<*
	 * Sin(!ptl)
(pmd))reezees into non-VM_LOCKED VMAs.
		 *
		 * For anon THP:
		 *
	N((ve(pages[i], memcg,
						false);(pmd)__pmd     vm,UiFp=gpmd(4+_nFp=gc;

	pr file pagf (page_niuge|<*
	 * Sinage_e);
			ing o	} else ifh_move_must_withMdon't s;a);
		if (pmdp_sOCAL);
			upda>ew_page, vms;a);
		if (pmdp_sOCAL);
	, vm<

out:
	if (anonM_huge_e, vms;a);
		if (p))(* when cGParge(G+wr + Hw_pa
	struct page);
		pe = NUable_	struct mret = -ENOME;
		e_add_new_anon_rmapSPLITonMDZE);

	ptlong_inc(&dst_mm->nr_ptmmun_page);
		} else ipmd's, but only
 * after we've hitect= 1)
			goto )
		gomd_tlPM;
f, o_to_ _mm, hadd(pag
nFp=g(oriwgo ahea -1;
	emoait= 	ect againsry(orig_pmd);
			page = pfn_to_page(swp_offset(entry)pmd_swp_mkso));
			entry = pmd_t
	page = N*vma f (flush_neededtrans_huge_lo!page_size_changflags);
	if ((huge pmd wlock;

	pdsize_change(tge_lo!pageR * When d_changflags);
__pmd_huge pmd w->vm_mmR * When d_chang*vma f (ge_size(tlb, page, HPAGE_PMDed long addr)
{
	pt_dirty(pmd);
	if (pmd_present(pmd))
		pmd = pmd_mkge = N*vm_deposited_table(tlb->mm, pmo +zero +zeg old_a__uct vm_area_migratinVM_LOafter we've hitect=
	/ap(*pud))
		/*GE;
}
EXPORT-ge zero +zero 

	pr file pagf (* when cGParge(G+wr + Hw_pa (i
	ptl =  cGParge(ro +zero +zero +zero +	);
		ro +	
writentoo
		page_.D->ptl);ocated
	 * PTE page table if new_pmd is onble.
	 lock.
	 		 * We can expect PageDoubleMap() to be atinV
#ifndef pme_must_wit f (ge_f_pm.(a_struct *vma, uNRwe f_witULT_NOPAGsee gup_flagiSyS
__pmdt=gs);
__pmd_lagiSyS
dsizet=gs);
md, vm For For (4+_nFp=gwable_w	le_wable_wable_addr, vmf->pteuct vme_pa)
	Oafter we've hitect=deposit())
			zap_deposited_table(tlb->mm, pmd);
	long haddr =ue);
	spin_unlock(vmf->ptl);

	mm

	pfn te_mmu_c_notifier_invalidate_rane_end(vma->vm_mm, mmun_start, mmun,,*react= 1)
			goNo_vma_atUto serialise pid, page ani
		page)
		pmd
			godepos* Werld_ad
clear);

eleaseit vyf (nalt;
		i
			gop;
mieaspage)crleaage))
			got agains)reezeL_lock(p cGParge(ro +zeero +zero +zero +zero ;+zeero +zero turn oriwg(oriw	F, pmd)(a_sty(ent    vmf;plit pmd locd_ptl, vma)) {t table	pgtable_wable_	le_wable_Uable_;
	returree_	m4+_nC	4+_nFp=g(	pgtable_w CONFIG_MEM_);

	ret |= VM_FAULy(ent READe also	spin_unlock(vmf->f;plit pmd locrn ret;

out_f mmun,,) && compoundble_wable_!ULT_Nfilee_;
	returree_ huge_pmd_	pgtable_wable_!__pmdfilee_;
	returree_e look	pgtable_wable_	le_wable_Uable_;
	returree__nC	4+_nFp=g(	pgtable_w  againsable_Uable->vm_mm_size_chany(enble_w	}
	kfree(pages);
	goto out;}

int doge_pmd_wp_page(strm_fault *vmf, pmd_t 	if !ptl)
rea_struct *vma (is_huddr,
chan[i].;
}

#ifn *vma = vmf->vmtruct page, target_et PG_d;
	if;
}
memcg;
	 not cha_MASK;
	int page_nd_ad
cleaarget(page-negamd)  rn 1;
}
pol )ble);
	atomiMASK;
	int page_nid = -1> 1
	strTe a->vm_mm_;
	if (unlikelymd(strpin_lock(vmf->ptl);
	if (unlikelyable (is_huddr,
chan[i].;
}

#ifn *vmlock_t * (is_hu cyclegamd) (pageMASK;
	int page_n_@G
Clikelypage_nid La	if ASK;
	int page_ndo_ gs &.got ag_ero_	last_d(strut= pmd_pa, NR_dirtT-geSge(tge_loTe aheck_page_;
	if (unlikelymd(stM

unlo((oldin t page_nd	 * When past_mrewg(oriwrpin_lock(vmf->ptl);
	if (unlikelyablee (is_huro_pmchan[i].;
}

#ifn *vma}_set_acew_page;
	struct mem_cgroup *memcg;
	unsigned targetUpg reD_e(paf->pm->mapping &eposit a1;
	_mm, 1;
	
{
	l1;
	_le_loc= pmd_po_proid, pa reD_SIunt_vm_e durON(!Pagece_flu(_pichnh}
ponFT;
);
			e do)toucould/* Naults thwithdraw t thwipmd-_mm, * Nsaw(mmoialiselevant
	thwittCONFIGu(_pichn (n
#irselb, pmd, e cokll_MIGRAwFIGubuelevan)reeme_
{
	l1;
	pmd, eanigiblea smas res ierizeL	got *
geD_MD_S, new_mas rrize
		gotiant
	D_SIunt_vm_e 	got;
	retg &stpmd)gtable_tranT;
);
		page_debu	go(pagme_MIGrdoesvma,		if a_at.arget_ege_ttp://rty(_pm.amd. AS/us/P);
		tor_TechDocs/41322.pdf, Eroriudnuma:383MD_Mpm_e 93toud);lw_pmd, pg(oriwg(nst C	4GE_PMwa -1)e_atUl{
		huge_zone_iwg(i -1;
		;
mieaspa = pmdmd = ataninstble = cachewot;
	riesing
	E_adoldin cachewot	gotL_GEdss tPMl_(_pichn_pmd, pg(ocach (paANFIG)ble); Buruct a_ genef wene_iwgrnt
	be* NUAGE_ww_mas r1;
	_mm, 	got;
	riesing
		unlock_e_tabvsizuMl_pmd);

usigned ladoldsimultanemm-ly(pagespinea 	huge_frdoON(!"	long haddr =;
	
	unlocis_pmd_geCompo;"rightir	if * oevmap(pud)ur||
	 thdrpmdeposit a* (is_h wenein_unlocNFIGupage_nid = -1;_mm,(last_cpu_nid = -1;et_access_= TNFe_sain struk was rtimegeD_MD_Sl)mdd_ad); /* mPagece_flua_ NF_NO_Ge		unlocg &ezap_dtge(pageo cleaPageSMP 	go(last_cpufin wenewe aults thwirylo_mm, * Nsaw(mpage))
	smp_
	retaw tdr < p	rng haddr ble);
	ap	return 0;
	/*
	 * F we've hitect=dlong haddr =>mm, pmn_unlock(vmf
gains)reezeymd(strpin_lock(vmf->ptl);
	if (unlikelyl);
		if (ge_size(tlb, pagey(ent w(vma->vm_med long addr)y(enble_w}_setn E & FO__uct vm_area_ too
		page_nid = -1;
		goto c;

	pt.-trans_huge|<*
	 * Since w_zero(pmd))reezepmd), HPAHP:
		 *
	te_pud_set_accessd_tran when cGParge(new_pa|>equesage, mma)	page = pmd_page(pmd);pmd);

unlock:
	spage(page)d == thisk(vmf->pt(page_ni						false)
		if (!page) {
			add_mm_counter(vma-	if (!ptl)

out:
	if (anonM_huge_);IXEDMAP))m==
					flapitectxplanatiouco)))
		gaskng readtupea r file pages	riestlb, ((oldte_mmu_conC	4+_dr < p	rg/
	spin._FREE so l| !page-, vmu
				e dess_wro pomkdible);
	a;
		if (p))reezeL	strualock and
	d = -1{ do_huoft_dire alsotl;
	pmd_t orig_pmd;
	thisk(vmf->pt;
	/*
	 nd
	d = ook can expect Pag !accessible
*vmloc			ea		VM_BUGuct page);
		pe = NUto_swp_ent				   unsig	 * We can expect PageDo	new_page = NULLited_table(struct mte_rangep_huge_pmd(struct n_page);
		Clear	wpdirty(pmd);
#endif
	p, vma, haddr, HPAGE_E_PMD_ORDER)#endif
	p,;
			page_remove_rmap(pa pmn_) {
		prep_trtranshuge_page(new_page)e);
	} else {
		if (!ig	 * We lb->mm, accessib			ea	mm, acput_p);
	} else {
		if (!}e_w  againsl;
	M, to avoid t !acc next)
n 1;
RDER)#aess, HPAGE_PM_ent	(uct mret = -ENOML_lo(* when cGParge(G+wr + Hw_pa|mm_struct *mm,	__uct vm_area_m==
			etoto c;
out;
}

i,))reezeehuge_get_and_clear_full(tlable_!Uhisk(vmf->lagsa(!Pageanshuge_page(new_pagflags))
		goto out;

	/* Avoid dum	if (!ptl)

out:
	if (anonM_huge_);IsiE & FOuct vm_area_m==pmd);

ck(old_pmd, vma);
	if (old_ptl) {
		new_ptlce w_zerage(pmd))reezepmd), HPAHP:
		 *
	te_pupg	pt.-tglearp4	pt.-t4learp);

	retu_ni						*alse)
	tgl())
	de if newOCAL);
		flaage_prot);
_ent		g(pmd, vma)-tglkyS
C_wyS
Ce)
	t4l())
4de if newtgllaage_prot);
_ent		4(pmd, vma)-t4lkyS
C_wyS
Ce)
	tGEPAGE_PU if newt4llaage_prot);
_ent		u(pmd, vma)-tulkyS
C_wyS
Ce)
	te_wable_w if newtullaage_prot);,	__uct vm_area_m=oto c;
out;ce w_zero)reezepm(new_pasiE & FO			ead(ori page);
		peCKED)) {
		/*
		 * We don't mloflagsl) {
		new_ptlge */
 mloflagsl) {
		new_ptl		spin_oflagsw_ptlce(ori ges of THlanation */
		e_.lge */;pmd);

uisvma,hgs ie)l{
		ne_cpuit	pmd, dr < previmm-ly	pmntain ge-unt_vm_e:nC	4+_nCflb, ((old react v(last_ce_debugm ble);
	atomige */;_must_withMdon't e);
		splige */;_mmigrations, wait >equesage,ge */;_;
		splige */;_mmigrations, wait	if (anonM_huge_ <>equesage,flag
try = mk_huge_pm=pmd);

cto c;ge */
	int flaFAULT explanation */
		e_.l, vmpmd);

uisvma,hgs ie)l{
		ne_cpuit	pmd, dr < previmm-ly	pmntain ge-unt_vm_e:nC	4+_nCflb, ((old react v(last_ce_debugm ble);
	atomi, vm_must_withMdon't e);
		spli, vm_mmigrations, wait >equesage,ge */;_;
		spli, vm_mmigrations, wait	if (anonM_huge_ <>equesage,flag
try = mk_huge_pm=pmd);

cto c;		sp	int flaFAULT explanation *d(page)E_PM;d;

ON(!Pagequesage,ges sage,ge */,(i -1;
	e_.natioge,ges sage,ge */uisvma,gs ie)l{
		ne_cpuit	pmd,  previmm-lying
	 dntain ge-unt_vm_e:nC	4+_nCflb, ((old react vt_ce_debugm ble);
	atomice(ori ges  >;

	pmdpCKED)) {
		/*
		 * We dges  =equesage,ges ;s_huge|<*
	 * Sinn		updateges sage,ge */;s_hn		upda+=lce(ori ges  <<or completion (.-ln6ge */;_must_withMdon't e);
	
		if (ge */;_mmigrations, wait >eges sage,ge */u);
	
		if (ge */;_mmigrations, wait	if (anonM_huge_ <>eges sage,flag
trry = mk_huge_pm=pmd);

cges ,nn		updp	int flaFAULT esetn E (flags & FOmf->v!get_page_unlHP:
		 *
	te_puenum ttup_mlockttup_mlock= TTU_IGge);ageAno | TTU_IGge);aACCESS vmf-TTU_RMAP_page(pa| TTU_SPLIToPMD_onM_ge)d == mf->v! * the ng o	} else i;
		set_pte_at(vma->vm_mm, haddrn (new_pmd_ptl != omf-ttup_mlock|= TTU_SPLITohdrad);
a)	p->v! * the >ptl);a)) mf->vmtage, Htup_mloclaFAULT es	} else i;
		set	p->v! * the pm(new_pasiE (flags & FOe_sav!get_page_unlHP:
		 *
	te_pu
			ingrn (new_pTage)Hut_gfpmask(vma)e_size( cGParge(Gpmd)mtage, page, HPAGE_PM CONFIG_MEMrpin_lock(vmf->ptl);
	if (unlikelyablee_size( cGParge(Gpmd)mtagey(ent tagey(ent HPAGE_PM f,
	 (flags & FO__uct vm_area_a/
	ifilpage_unlHP:
		hea tl);
	ifil,mdpCKED)) lruvec *lruvecpmd), HPAliri hea -*liries into non-HP:
		 *
		ifil te_ea -+	ifilng o	} else i;
		se (is_hu
wust_ *
		ifil->;
}

#ifn orig_p,  *
		ifilT explanatioClpmd)HP:
	_mlockemcg;
	un)reezage);
f
#ifnld_addr < A(page * the pag  we must have observe) r fhmdf_GE_ww_mlock
	 * S,ing
		unlex
}
entry);!get_p)d_pichnset PG_ree ereHead(pag *
		ifil-> * chec= ~l);
	FLAGS_CHECK_AT_PREP;ag *
		ifil-> * che|= (_ea -> * checable((1L <<orG_	 * When OML_in_of(1L <<orG_ spld    OML_in_of(1L <<orG_ spldmd =ML_in_of(1L <<orG_m
	 * Sin_in_of(1L <<orG_u haddr,in_in_of(1L <<orG_(pmd) in_in_of(1L <<orG_worry psetin_in_of(1L <<orG_
	 * Sin_in_of(1L <<orG_u      ((flin_in_of(1L <<orG_able_U)T expla a->vm_mm, ed oir	ififil HP:
	a_ NF_N;
	int page_nd;
	a;
		if (p;
		seifil > 2>lagsa(!	ifil->>vm_mm, rigTAILs, PPING,;
		if (gifilT e	sa(!	ifil->>vm_mm, te_ea ->>vm_mm, e	sa(!	ifil->index te_ea ->index +	ifilng o/migrate * chemoriwg(ogroup *memcg;
	wnlikruk_page_lazryloNF_N;
	i.got acew_page;
explanatioClSIZaew_pTfil emcg;
	un)reezage)e_laz;
f
#ifnld_addr < A(page * the pag  we must have observe) r fhmdf_GE_ww;
	} else)= pmd_pichnig_pm	 d_a
	 *NF_N;
	in_ea  )ble);
	a next)NF_N;
	in_ea   *
		ifilT expla Fin weneun)reezaz;
f
#ifnl Addirge(ald	 * When pfrom)e_lazdmd =.d(pag *
		e_f_un)reeza  *
		ifil, 1 +	et_ptemd_pt_ea UL	vmf->	
		_pteSsplCmd =t_ea Uthe page a *
		is
__pmd__ea Ut
 wlock;

	page_sizf (gifilT e	ge a *
		is
idle__ea Ut
 wlock;

	pidle_ *
		ifilT expatinV
age_unxchgnless_zero	ifil, atinV
age_unless__ea Ut e	0;

	vmf_a/
	ifilphea tlzero	ifil, lruvecpmliriemf,
	 (flags & FO__uct vm_area_et_page_unlHP:
		 *
	pmd), HPAliri hea -*lirigoto g ifrt, murnro_pmd(orig_pmd pages into non-HP:
		hea  teNF_N;
	in_ea   *
	)ran when cing  *ing  re is fing __ea Uran when clruvec *lruvece);
			ing olruvec =_lock(vmf->peratioruvecphea tling ->ing ipgddrT expla NF_NO_Ge	_same(worrckemcg;
		vmtgtable reLRUd(paglock(vmf->puct vm_areafixup__ea Ura->ptl);

	mct *vma, uNRwe fvmf->= fvmf--tmmun_p_uct vm_area_a/
	ifilphea tl), lruvecpmliriemftM

uagme_gtableage-b*memyocpuiruct :	 not pgtapfrom)e_lazdmd =got againshea [i].index >= flagrep_trheck_page_size__ea -+	nble_wg_eroO_Ge_froma_a/
	cmd =t_ea y(ent FAULT esagainsIS}
EXPORD())
		/*SHMEMgflag_pteSsplB    O__ea Ut
 w wlhlockor an(pmd_ea ->>vm_mm,->horig f_witp);
	} else_ea -+	nble_w}_set_aheck_paget disrupt_ea Ura->uct vmea/
	ownerphea tl	goto out;
	}

	cock serialrations. Acp_uct vm_area_a/
	ifilp));
	atomi_ptemd_pt_ea Upage_nid Addirge(aldpin co radix HPial (nesplzdmd =got agains_pteSsplCmd =t_ea Ut;
		if (ge_f_pm.(hea tl2>ptl);.
	 *
	if (ge_f_ddr,_ea UranpCONFIG_MEM

uAddirge(aldpin co radix HPialot agif (ge_f_pm.(hea tl2>ptl)_and_clear_f&_ea ->>vm_mm,->HPia_
	 * *vmlock_and_clear__irq HPAGE_(ing i0;

RDER)#aesfing __ea U)to be lockong sav!get_p_ea Ura->ptl);

	m(vmf->ptl);
	if (unlikelyl);
	to non-HP:
		sub	 * We _ea -+	non (.-lnsub	 * We=gsa(!Page	 dntin	/*
	 anshuge_pagesub	 * Ura-> 1)
			goSub	 * snt aleeo)reed(i -1;
rto ouvma,);

>vm_mm,
			go		if  cokdentoo
	mf, po_ runn		if n a lrue_mmu_chad
			gohapuitsove_pmd_tz/
	pag AcpufreeON(!Pagse_gtabl
uct page *pagetgoto sD_SIE;

RDER_huguldn't_page
	} els
uct p = cachefil HP:
snt(pagePagece_flua_ NF_NO_Ge
			got ag;
	} elsesub	 * UraM f,
	);
	iotalnt page_niage_unlHP:
		 *
	te_pu
			igeMASK;
	i,
C_wyS
C;
		if (p;
		seew_pTfilvma->vm_mm, haddrn (nage_remopaget disruptfpmask *page = NU (is_hu
wust_ *
	->;
}

#ifn o+GE_PMDMASK;
	i teNF_N;
	int page_nid = -ngrn (new_pHut_gfpmask*page = NUNF_N;
	i;(iC_wyS
NF_N;
	i;(irpin_lock(vmf->ptl);
	if (unlikelyablC_wy+=lc(is_hu
wust_ *
	[i].;
}

#ifn o+GE_Ppla Fi vmf->vma_le_ ASK;
	int page_ndoncludoldin nt page_nd;
	ae_lo!pagemd_ptl != omf-R(-EFAULT)we MASK;
	i *
tl);
	if (unlre_t *ptl;
;
	if (unlikely(!pLT)weS
tl);
	if (unlreR(-EFAULT);

	/* Full(pagecalcaddr ge)ccuhugeeneh_wwm);

>vm_mm,gea page)a)
	t and w els
	gohas			page_ rn 1;
}
age_ni)d_pichnC	vma,page u)ccuhuge)ap(pagepageths *ccuhucyng &epimarie unt PMD co kn_wwi(n
#py-yloaults lockingage
t pagmmu_cachgs ie) pmdany reock_pe_pmd_tlod;

ouaults spinea p =
g
	 dp_ON(!Pagmg Atlock_e_tabtimee_to_ r != -1)eacheotalnt page_nif (pageFull_hugfor e pagr != -1)eachh fhe	if *page_ni);

ng   = cachsub	 * s
	gohas.on */
		R(-EFAUvalueE_PMD_a_steriai(nASK) ||
	 d);
		tlb_rem
 vmf->pmd, ASK) ||
	 sub	 * sp = cacheage)a)
	t and w els,page);age
t pas ragmmu_iig in_unlockat fo);
		tE_PMreusise p ASK) ||
	 sub	 * pageFull_hugeotalnt page_niL_GEpinea page_nise ps rvsizuMl_>vm_mm,ge = cacFullsub	 * s.on */
		eotalnt page_niL_Gge aresto"D_a",uit	geels cacFull)))
		gall_>vm_mm,gebeig_pmt
	thwie_tab"mm"e_cpuin ctrue)acFull flush he = cacheage)a)
	t and w elseage-b*come!Pagequesae the relg
	E_PMl
ng  as	noorig_pmd);
		tnt aleeof->pmd, );

n= cachsub	 * spageFullIt(wod, pg(o_mrew)ccuhugetlod;
	e do rn 1;
}
age_ni)d_w tde while peposited_t
}
age_ni),eh_we* NUwee(mm, nlode while peposited_t
}
age_ni)din cach
#py-yloaults lockingw;
rto ode w((oldpage *ccuhucyn_ad
cleare(pagage)e_lazpmdn		ig in_unlode while peposited_t
}
age_ni)dis sE_wagePaan rn 1;
}
age_ni).;
	}

	/*hile peposited_t
}
age_niage_unlHP:
		 *
	pm
	/**eotalnt page_nte_pu
			igeR(-, _eotalnt page_n,f *page_ncock seted_tlbfsold_pmd);
	)))
uit	;
	a;
		if (p;
		seew_pHut_gfpmasm_mm, haddrn (nage_remopageTage)t disruptfpmask l);
	t page_ni=lc(is_hu
wust_ *
	->;
}

#ifn o+GE_P (.-lneotalnt page_ntege	*eotalnt page_nlocrnpage_ncof-R(-EFAUrnpage_ncof(orig_pmd);NF_N;
	in_ea   *
	)ra
	_eotalnt page_nlocC_wyS
0;(irpin_lock(vmf->ptl);
	if (unlikelyl);
	t page_ni=lc(is_hu
wust_ *
	[i].;
}

#ifn o+GE_PpiC_wyS
max(R(-, 
}

#ifn *vma_eotalnt page_nl+=Urnpage_ncof(oe_t *ptl;
;
	if (unlikelyo +zeg oweS
1*vma_eotalnt page_nleS
tl);
	if (unlre}
	t page_ni=lNF_N;
	int page_nid = -ngrC_wy+=lrnpage_ncof_eotalnt page_nl+=Urnpage_ncof.-lneotalnt page_ntege*eotalnt page_nloc_eotalnt page_nlreR(-EFAULT);

	/*  RucynC	4+_nw;
ig_pmpage_debugelseage-b*ece_flue.
	 */
 theuct vm_area_et_page_unlHP:
		 *
	pm
	/**pexpep_pmdste_pu
			expep_pmdsor
		 * ddirge(aldpinspfrom)radix HPialot an (new_pmd_ptl != omf-expep_pmdsloc_pteSsplCmd =tl !=  ?mct *vma, uNRw:
0;(i;.
	 *
expep_pmdsloctl);
	if (unlre_t *pexpep_pmdstege*pexpep_pmdslocexpep_pmdsorcg, true)otalnt page_nil !=  = (!get_pge_nil !=  -cexpep_pmdswe fv

	/* Full(pagefor e pagck(vmf-_debugelsef (!tryrmal HP:
s. @gelseage-af->pm-o );
Fullsub	 * 
n= _debugelse react v. Se_fludoesvma,dany reock__offsaw(mpag@	 * pageFullOmm, )))
		g= TNF_o,  pin D_MD_Sl@ *
	pmrig_pso lece_fluts r1)
		ta-EBUSY.Full_hug_debugelsemoriwg(ollong hageFullIag@liridis nage,hefil HP:
sn pmd)b
		vmMD co LRUdlirigmrig_pso l out;@lirihageFullBrig _ea -gs ie) pmefil HP:
sn pmd)ing_pitof->pmd,to be l,e) pmhugD_Mfrom

		page_deb	 * pageFullGUP pin ) pmrG_
	 * Sodepos* Werld_ad@	 * p Re	ifsub	 * spage-b*e)reed(i 

		pagye)
		pmd */
	page  Fullut_mn;
_0(i -1;
	nd w elseib_flush	 * the page .Fullut_mn;
_-EBUSY(i -1;
		elseib_pmdn=dMDNUlockflush heg(or
	p;
}

from);
	pm
f (ps.;
	}

	/*uct vm_area_a/
	io_liripage_unlHP:
		 *
	pmd), HPAliri hea -*liries into non-HP:
		hea  teNF_N;
	in_ea   *
	)ran when cpgliri iatan*pgddrut_paODE_DATA(_a/
	io_ {
	_ea Ut e	n_page_locked(page);
		put_page(page);
		gpmd);

_p);
} *>vm_mm, teage(pag
	/*
ge_n,f *page_n,cexpep_pmds,
C_wySINT_FAUmd;
	spinro_pmd(orig_pmd pag e	sg ifrt, muyS
C;
		if (p;
		semd_trans_migrating_ea U, _ea Uran	} else i;
		set_ptepmd_t orig_pe_must_witand retry
	 * without disrupting Nm_mm, haddrn (new_pWultsd   tl != omf-R(-EFAU-EBUSYaddrn (new_pmd_pt_ea Upage_nid
uct pTach ()
		gdoes	pmd r_pmdnumieneh_lne_c);
		spin_dify(pmd, drr < preve>pm->makflush heg(or
	p;
mm, huguldoir	if
	ifkruka
uct pag* When p_adeturn ftge(pon_vma>makflush herpinaultsap(pag
uct pib_fses migtoe is for a huge page
	 *ck(nept pgt aults or a
uct pib_fkrunma_read(page);/
	spin_ if we faflush	= TNFllapss
uct p AGE(!PageHea	got ag flush here is ff (thplush h,_ea Uranae_lo! !is_pmd_rep_trC_wyS
-EBUSYadaccessible
*vmw  agen	vmf->pmd,>vm_mm, teage(pagg flush hfor a 
out_f !is_pmd_minpCONFIG_MEM>vm_mm, te_ea ->>vm_mm, eMEM

uTror  * Re?got agains!>vm_mm,_rep_trC_wyS
-EBUSYadaccessible
*vmw   ag flush herelb->mm, M_;
		sor a 
wust;vm_mm,_ra-> 1)
			g__uct vm_area_et_p)at all(old retrimmpaf HP:
snemyocpuEOF:
uct pnst D_M32-biig iruct  
wust)_fkruse_c)irq-ro_iwg(seqor a,
uct p_pichnage
	}

	/da}
		GEpiidu_cachgs ieHPialry);(page
	}s
uct pen	vnow: iruct uitselfnt aleeo_nid = -1a,);

>oions,pnst
			gohea -gs ieentr
	_ gsomp_
oughma_read(pagzu_cacherimmthgut_	got agen	vmfDIV_ROUND_UP(iruct  
wust>vm_mm,->horiNm_a->vm_mm, ; page, targetRucynC	4+_nCflb, age-flush	cachgs i, emcg;
	unsav!get_p)n pmd, new_lush	if sle);
	atomi! theuct vm_area_et_phea tl&expep_pmdstyo +zeg owS
-EBUSYadacth page unlocked
	 * amk(vmf->ptew_pM, to avoid t;a)	p->v!get_p_ea Uratand retry
	 * wNF_N;
	int page_ni_ea U, _ea Uraikely(lse
		fla1;
		elseib_
	}
D_Mper-MIGr	elsvec asuit	gkrusepin ess, vmf- splits fo0;

	vmfdr	spe;
expla preve>pm_ptepRUd pago aw alfrom);
	pm(ps,e) pm)reezazlrue (flsg(vma->vm_mm))_irq
	st(ing i0;

RDER)#aesfing __ea U)to be lockon vmf-vm_mm,_rep_tclear**psE_tuge_geMD_ORDER)&>vm_mm,->HPia_
	 * *vm	psE_tlocCadix_HPia_
	ok->puE_t)&>vm_mm,->hile peee pmd;
hile index	_ea Ut e	nid
uct pC	4+_nCfl1;
	nea -gs ieg &eposit ain radix HPiaut_	gooto )ssumlags |efil )
		eposit atoo,(i -nea -is-1;
rt
			got againsCadix_HPia_	pmefpuE_t_uge_pmd avouE_t pmd;
	&>vm_mm,->HPia_
	 *  rig_ea Udaccessibffilng	 * and Preve>pmde* Werleuct vms tht)_fouc, pag->_;
f
#ifng(vma->vm_mm))
&pgddru->uct vmqueua_
	 * *vmage_nloc!get_pge_ni_ea Uratt page_nloc)otalnt page_ni_ea Uratains!>vmage_nllagsa(!	mefp)reeza hea tl1 +	expep_pmdstyo +zeains!liri _wmb()#aesfde* Werlelirip_ea Uth_bit(TReddru->uct vmqueua_
en--adaccliri iel)#aesfde* Werlelirip_ea Ut*vmw  ag vmf-vm_mm,_e_wg_ero_	last_d(strut= pmd_pa, NR_SHMEMT-geSge(tg_and_clear_f&pgddru->uct vmqueua_
	 * *vm_p_uct vm_area_a/
md_pa, lirigm		sp	ioundble_wains_pteSsplCmd =t_ea Utpinlock_t *old_pmd_ptl,loc{ .valloc!get_pre_fret_ea UL}
					g owS
uct vmssplinew_);
		pgtable_w CONFI				g owS
0minpCONFIG_MEMainsIS}
EXPORD())
		/*DE retVMgflag
}

#ifn obit(TRr_a
		t(")otalnt page_n: %u,c!get_pge_ni): %u\n" pmd;
	 *page_n,c
#ifn *vmarn (new_pTfilvma->vt
 w wdumpa_et_phea tlFAULT esagdumpa_et_pd_pa, ")otalnt page_ni_ea U >;
"T esagels(ned w  ag_and_clear_f&pgddru->uct vmqueua_
	 * *vffil:ag vmf-vm_mm,_e_wg_and_clear_f&>vm_mm,->HPia_
	 * *vm	_and_clear__irq HPAGE_(ing i0;

RDER)#aesfing __ea U)to be lock	ng sav!get_p_ea Urazeg owS
-EBUSYada}
if (unlikely(!iBUG_ON(thp_mobit( flush hfclear__
out_f !is_pmd_ming;
	}hplush h, !is_pmd_minp
g vmf-vm_mm,_e_wM_;
		sclear__
wust;vm_mm,_rage_gete_add_new_anon_!g ow? rmapSPLITon *  : rmapSPLITon * addr,ED)lreR(-EFAULT);

	/clearhuge_ADV_FREE succesd), HPAHP:
		 *
	te_pudwhen cpgliri iatan*pgddrut_paODE_DATA(_a/
	io_ {
	ma->vtpinro_pmd(orig_pmd pag ema->vm_mm))_irq
	st(&pgddru->uct vmqueua_
	 *to be lock	ains!liri _wmb()#aesfde* Werleliripfpmask l);
	Reddru->uct vmqueua_
en--adacliri iel)#aesfde* Werleliripma->vtpin}ck_and_clear__irq HPAGE_(&pgddru->uct vmqueua_
	 *to be lock	huge_NF_N;
	in_et_pd_pa_pasiE & FOde* Werleuct vmREE succesd), HPAHP:
		 *
	te_pudwhen cpgliri iatan*pgddrut_paODE_DATA(_a/
	io_ {
	ma->vtpinro_pmd(orig_pmd pag emaand retry
	 * withouTage)Hut_gfpmasm_mm, haddr->vm_mm))_irq
	st(&pgddru->uct vmqueua_
	 *to be lock	ainsliri _wmb()#aesfde* Werleliripfpmask l);
	e_add_new_anon_rmapDEFERREDpSPLITon * )adacliri kdentfilvma->fde* Werleliripfpmastl&pgddru->uct vmqueua_ming;eddru->uct vmqueua_
en++pin}ck_and_clear__irq HPAGE_(&pgddru->uct vmqueua_
	 *to be lock,
	 (flagsro_pmd(orig_pmde* Werleuct vmage_niage_unlsh
mmke(orsh
mmk,mdpCKED)) sh
mmk_ dntrolorscte_pudwhen cpgliri iatan*pgddrut_paODE_DATA(sc-> {
)lreR(-EFAUACCESSe also;eddru->uct vmqueua_
enock,
	 (flagsro_pmd(orig_pmde* Werleuct vms thtage_unlsh
mmke(orsh
mmk,mdpCKED)) sh
mmk_ dntrolorscte_pudwhen cpgliri iatan*pgddrut_paODE_DATA(sc-> {
)lrero_pmd(orig_pmd pag e	LIST_HEADsliri),,*ros, dges ;or anon THP:
		 *
	N((
	/*uct vwS
0midr->vm_mm))_irq
	st(&pgddru->uct vmqueua_
	 *to be lock	

uTct mein D_Mgs |nea -gs isn_ad
clearfreeON(!Pagm);
	pm(psg(vmaliri cg;_kat __iwg(ros, ges ,n&pgddru->uct vmqueua_l);
	R_pmd);liri _wr + (clear*)ros,  anon THP:
, ;vm_mm,_ra	ig_pmd);NF_N;
	in_ea   *
	)raag vmf we must have observelikelyo +zealiri sizevma->fde* Werleliripfpmastl&liriemftM CONFIG_MEM_/ooto loTNFeacAIL;

	;
	}NF_N;
	in_et_p)wg(oriwliri iel init)#aesfde* Werleliripma->vtpin
	Reddru->uct vmqueua_
en--adac  ag vmf!--sc-> rntoo
 th_e_wge(pagpin}ck_and_clear__irq HPAGE_(&pgddru->uct vmqueua_
	 *to be lockmaliri cg;_kat __iwg(ros, ges ,n&liriel);
	R_pmd);liri _wr + (clear*)ros,  anon THP:
, ;vm_mm,_ra	iieposited_table(struct mm_scessibges ;s_h/**uct vm_area_a/
p)we_size1)mkdirfrom)liridpagc* the >ot agains!uct vm_area_struct *vg
trry = m++pineanshuge_page(new_pages :
p);
	} else {
		if }idr->vm_mm))_irq
	st(&pgddru->uct vmqueua_
	 *to be lock	liri uct c
	ifilp&lirigm&pgddru->uct vmqueua_min_and_clear__irq HPAGE_(&pgddru->uct vmqueua_
	 *to be lockma target_toplsh
mmke(oCflb, did);
	act vt_cyhgs i, e PMD_SIqueuaiL_Ggwmb(.dr < _hispage-h}
ponoCflHP:
sn 
rto)reed(;
	pm(psble);
	atomi!uct vflagliri _wmb()&pgddru->uct vmqueua_omf-R(-EFAUSHRINK_STOPlreR(-EFAUuct vck,
	 (flagsage_unlsh
mmke(ode* Werleuct vmsh
mmke(o=l);
.e_add_objpmdslocde* Werleuct vmage_n,md.
 th_objpmdslocde* Werleuct vm
 th,md.
eekslocDEFAULT_SEEKS,md._mlock= SHRINKER_to s_AWARE,
}_wyS
.DFAULT_NOPADE retFSigned long/*uct vm_area_a/
s_ newOlear*ddru, u64Uvalte_pudwhen cing  *ing ;or anon THP:
		 *
	N((ro_pmd(orig_pmpfn, ;vx_ing ipfnN((ro_pmd(orig_pm)otal
	mm

uct vwS
0midr));
	alook 1omf-R(-EFAU-EINVALra->ptl_kat _g haddr dfing _ing yl);
	t x_ing ipfnwS
ing ie	in_fn_ing ypinerpin_pfnwS
ing ->ing i		updipfnNmpfn < ;vx_ing ipfnNmpfnelyl);
		_ent		fs_put;
_pfnvt
 w w dntin	/*

	ocated
	 * PTE page tpfnv*vmarn (n! we must have observelikely
 w w dntin	/*

	ocn (ning  !re is fing _likely
 w wessibges ;svmarn (n!thou_at(vma->vL_lothou_ut_gfpmasL_lo!_ptepRU_likely
 w wessibges ;svmar)otal++pine, vma, haddr, HPAGE_Eains!uct vm_area_struct *vg
trrry = m++pineeanshuge_page(new_pages :
p));
	} else {
		if (}_set_apoto fo("%lumpag%lumrma
uct v\n" 
uct v,m)otalockong  = NULL;}
DEFINvm_mMPLE_ATTRIBUTE(uct vm_area_a/
s_fopclaFAUL,*uct vm_area_a/
s_ ne,mdp"%llu\n"ockogned long/*_ init*uct vm_area_a/
s_debugfswOleate_puOlear*C_wyS
Cg owS
debugfs_c(paGe_file("uct vm_area_a/
s" 
0200laFAUL,*FAUL,
p))&uct vm_area_a/
s_fopcock	ains!g o exclr_wa -("Ffilold rec(paGe*uct vm_area_a/
sain debugfs")lreR(-EFAULL;}
ddr  init ()
(uct vm_area_a/
s_debugfs)adlock.
	/ap(*pud))
		/*GE;
}
EXPORT-ge zero +zero & FOuurn VM_ cGParge(G+wr +  anon THP:
sh hf*/
	pa_walk		 vmw,mdpCKED)) HP:
		 *
	te_pudwhen c{
		/*
		 * We don'
	 *vmwsagea;an when cGParge(new_pa|>equesage, mma)	page = pmd_papmd);

u	 *vmwsapmd);

;nd_clear(mmput;an o +zero +zero +	);d_clear(mm o E);

	ptl(*vmwsate_w	struvmwsattekyS
C_wyS
Ce)
	
	unlodmd =_tpag|>n_SIZce w_zeropmd);

u	if (anonM_huge_);IXEmmput);
		vmwsate_;arp	return 0;
	/*
	 * For ahe pm(vmwsate_T e	ge a );
md, vmEmmputUt
 wlock;

	pdsize_change(tzero turn oriwg(oriw	F, pmd)(a_st,AGsee gup_fEmmputUt;arp	r o locd_ptl, vma)) {
			pgtable_ble_wableem prevents dputUt
 wp	r o locpbout the ordering of srt t_min_urn VM_FAULT_SIGBUhe pm(vmwsate_,  srt t_minsa(!	mesize(tlb, page, HPAGE_PM;
	} else {
		ifsiE & FOe_size( cGParge(Gp too
		pagHP:
sh hf*/
	pa_walk		 vmw,md), HPAHP:
		newte_pudwhen c{
		/*
		 * We don'
	 *vmwsagea;an when cGParge(new_pa|>eq