/*
 * Kernel-based Virtual Machine driver for Linux
 *
 * This module enables machines with Intel VT-x extensions to run virtual
 * machines without emulation or binary translation.
 *
 * Copyright (C) 2006 Qumranet, Inc.
 * Copyright 2010 Red Hat, Inc. and/or its affiliates.
 *
 * Authors:
 *   Avi Kivity   <avi@qumranet.com>
 *   Yaniv Kamay  <yaniv@qumranet.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 */

#include <kvm/iodev.h>

#include <linux/kvm_host.h>
#include <linux/kvm.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/percpu.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <linux/reboot.h>
#include <linux/debugfs.h>
#include <linux/highmem.h>
#include <linux/file.h>
#include <linux/syscore_ops.h>
#include <linux/cpu.h>
#include <linux/sched/signal.h>
#include <linux/sched/mm.h>
#include <linux/sched/stat.h>
#include <linux/cpumask.h>
#include <linux/smp.h>
#include <linux/anon_inodes.h>
#include <linux/profile.h>
#include <linux/kvm_para.h>
#include <linux/pagemap.h>
#include <linux/mman.h>
#include <linux/swap.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/compat.h>
#include <linux/srcu.h>
#include <linux/hugetlb.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/bsearch.h>
#include <linux/kthread.h>

#include <asm/processor.h>
#include <asm/io.h>
#include <asm/ioctl.h>
#include <linux/uaccess.h>
#include <asm/pgtable.h>

#include "coalesced_mmio.h"
#include "async_pf.h"
#include "vfio.h"

#define CREATE_TRACE_POINTS
#include <trace/events/kvm.h>

/* Worst case buffer size needed for holding an integer. */
#define ITOA_MAX_LEN 12

MODULE_AUTHOR("Qumranet");
MODULE_LICENSE("GPL");

/* Architectures should define their poll value according to the halt latency */
unsigned int halt_poll_ns = KVM_HALT_POLL_NS_DEFAULT;
module_param(halt_poll_ns, uint, 0644);
EXPORT_SYMBOL_GPL(halt_poll_ns);

/* Default doubles per-vcpu halt_poll_ns. */
unsigned int halt_poll_ns_grow = 2;
module_param(halt_poll_ns_grow, uint, 0644);
EXPORT_SYMBOL_GPL(halt_poll_ns_grow);

/* Default resets per-vcpu halt_poll_ns . */
unsigned int halt_poll_ns_shrink;
module_param(halt_poll_ns_shrink, uint, 0644);
EXPORT_SYMBOL_GPL(halt_poll_ns_shrink);

/*
 * Ordering of locks:
 *
 *	kvm->lock --> kvm->slots_lock --> kvm->irq_lock
 */

DEFINE_MUTEX(kvm_lock);
static DEFINE_RAW_SPINLOCK(kvm_count_lock);
LIST_HEAD(vm_list);

static cpumask_var_t cpus_hardware_enabled;
static int kvm_usage_count;
static atomic_t hardware_enable_failed;

struct kmem_cache *kvm_vcpu_cache;
EXPORT_SYMBOL_GPL(kvm_vcpu_cache);

static __read_mostly struct preempt_ops kvm_preempt_ops;

struct dentry *kvm_debugfs_dir;
EXPORT_SYMBOL_GPL(kvm_debugfs_dir);

static int kvm_debugfs_num_entries;
static const struct file_operations *stat_fops_per_vm[];

static long kvm_vcpu_ioctl(struct file *file, unsigned int ioctl,
			   unsigned long arg);
#ifdef CONFIG_KVM_COMPAT
static long kvm_vcpu_compat_ioctl(struct file *file, unsigned int ioctl,
				  unsigned long arg);
#endif
static int hardware_enable_all(void);
static void hardware_disable_all(void);

static void kvm_io_bus_destroy(struct kvm_io_bus *bus);

static void kvm_release_pfn_dirty(kvm_pfn_t pfn);
static void mark_page_dirty_in_slot(struct kvm_memory_slot *memslot, gfn_t gfn);

__visible bool kvm_rebooting;
EXPORT_SYMBOL_GPL(kvm_rebooting);

static bool largepages_enabled = true;

#define KVM_EVENT_CREATE_VM 0
#define KVM_EVENT_DESTROY_VM 1
static void kvm_uevent_notify_change(unsigned int type, struct kvm *kvm);
static unsigned long long kvm_createvm_count;
static unsigned long long kvm_active_vms;

__weak void kvm_arch_mmu_notifier_invalidate_range(struct kvm *kvm,
		unsigned long start, unsigned long end)
{
}

bool kvm_is_zone_device_pfn(kvm_pfn_t pfn)
{
	/*
	 * The metadata used by is_zone_device_page() to determine whether or
	 * not a page is ZONE_DEVICE is guaranteed to be valid if and only if
	 * the device has been pinned, e.g. by get_user_pages().  WARN if the
	 * page_count() is zero to help detect bad usage of this helper.
	 */
	if (!pfn_valid(pfn) || WARN_ON_ONCE(!page_count(pfn_to_page(pfn))))
		return false;

	return is_zone_device_page(pfn_to_page(pfn));
}

bool kvm_is_reserved_pfn(kvm_pfn_t pfn)
{
	/*
	 * ZONE_DEVICE pages currently set PG_reserved, but from a refcounting
	 * perspective they are "normal" pages, albeit with slightly different
	 * usage rules.
	 */
	if (pfn_valid(pfn))
		return PageReserved(pfn_to_page(pfn)) &&
		       !kvm_is_zone_device_pfn(pfn);

	return true;
}

/*
 * Switches to specified vcpu, until a matching vcpu_put()
 */
int vcpu_load(struct kvm_vcpu *vcpu)
{
	int cpu;

	if (mutex_lock_killable(&vcpu->mutex))
		return -EINTR;
	cpu = get_cpu();
	preempt_notifier_register(&vcpu->preempt_notifier);
	kvm_arch_vcpu_load(vcpu, cpu);
	put_cpu();
	return 0;
}
EXPORT_SYMBOL_GPL(vcpu_load);

void vcpu_put(struct kvm_vcpu *vcpu)
{
	preempt_disable();
	kvm_arch_vcpu_put(vcpu);
	preempt_notifier_unregister(&vcpu->preempt_notifier);
	preempt_enable();
	mutex_unlock(&vcpu->mutex);
}
EXPORT_SYMBOL_GPL(vcpu_put);

/* TODO: merge with kvm_arch_vcpu_should_kick */
static bool kvm_request_needs_ipi(struct kvm_vcpu *vcpu, unsigned req)
{
	int mode = kvm_vcpu_exiting_guest_mode(vcpu);

	/*
	 * We need to wait for the VCPU to reenable interrupts and get out of
	 * READING_SHADOW_PAGE_TABLES mode.
	 */
	if (req & KVM_REQUEST_WAIT)
		return mode != OUTSIDE_GUEST_MODE;

	/*
	 * Need to kick a running VCPU, but otherwise there is nothing to do.
	 */
	return mode == IN_GUEST_MODE;
}

static void ack_flush(void *_completed)
{
}

static inline bool kvm_kick_many_cpus(const struct cpumask *cpus, bool wait)
{
	if (unlikely(!cpus))
		cpus = cpu_online_mask;

	if (cpumask_empty(cpus))
		return false;

	smp_call_function_many(cpus, ack_flush, NULL, wait);
	return true;
}

bool kvm_make_all_cpus_request(struct kvm *kvm, unsigned int req)
{
	int i, cpu, me;
	cpumask_var_t cpus;
	bool called;
	struct kvm_vcpu *vcpu;

	zalloc_cpumask_var(&cpus, GFP_ATOMIC);

	me = get_cpu();
	kvm_for_each_vcpu(i, vcpu, kvm) {
		kvm_make_request(req, vcpu);
		cpu = vcpu->cpu;

		if (!(req & KVM_REQUEST_NO_WAKEUP) && kvm_vcpu_wake_up(vcpu))
			continue;

		if (cpus != NULL && cpu != -1 && cpu != me &&
		    kvm_request_needs_ipi(vcpu, req))
			__cpumask_set_cpu(cpu, cpus);
	}
	called = kvm_kick_many_cpus(cpus, !!(req & KVM_REQUEST_WAIT));
	put_cpu();
	free_cpumask_var(cpus);
	return called;
}

#ifndef CONFIG_HAVE_KVM_ARCH_TLB_FLUSH_ALL
void kvm_flush_remote_tlbs(struct kvm *kvm)
{
	/*
	 * Read tlbs_dirty before setting KVM_REQ_TLB_FLUSH in
	 * kvm_make_all_cpus_request.
	 */
	long dirty_count = smp_load_acquire(&kvm->tlbs_dirty);

	/*
	 * We want to publish modifications to the page tables before reading
	 * mode. Pairs with a memory barrier in arch-specific code.
	 * - x86: smp_mb__after_srcu_read_unlock in vcpu_enter_guest
	 * and smp_mb in walk_shadow_page_lockless_begin/end.
	 * - powerpc: smp_mb in kvmppc_prepare_to_enter.
	 *
	 * There is already an smp_mb__after_atomic() before
	 * kvm_make_all_cpus_request() reads vcpu->mode. We reuse that
	 * barrier here.
	 */
	if (kvm_make_all_cpus_request(kvm, KVM_REQ_TLB_FLUSH))
		++kvm->stat.remote_tlb_flush;
	cmpxchg(&kvm->tlbs_dirty, dirty_count, 0);
}
EXPORT_SYMBOL_GPL(kvm_flush_remote_tlbs);
#endif

void kvm_reload_remote_mmus(struct kvm *kvm)
{
	kvm_make_all_cpus_request(kvm, KVM_REQ_MMU_RELOAD);
}

int kvm_vcpu_init(struct kvm_vcpu *vcpu, struct kvm *kvm, unsigned id)
{
	struct page *page;
	int r;

	mutex_init(&vcpu->mutex);
	vcpu->cpu = -1;
	vcpu->kvm = kvm;
	vcpu->vcpu_id = id;
	vcpu->pid = NULL;
	init_swait_queue_head(&vcpu->wq);
	kvm_async_pf_vcpu_init(vcpu);

	vcpu->pre_pcpu = -1;
	INIT_LIST_HEAD(&vcpu->blocked_vcpu_list);

	page = alloc_page(GFP_KERNEL | __GFP_ZERO);
	if (!page) {
		r = -ENOMEM;
		goto fail;
	}
	vcpu->run = page_address(page);

	kvm_vcpu_set_in_spin_loop(vcpu, false);
	kvm_vcpu_set_dy_eligible(vcpu, false);
	vcpu->preempted = false;

	r = kvm_arch_vcpu_init(vcpu);
	if (r < 0)
		goto fail_free_run;
	return 0;

fail_free_run:
	free_page((unsigned long)vcpu->run);
fail:
	return r;
}
EXPORT_SYMBOL_GPL(kvm_vcpu_init);

void kvm_vcpu_uninit(struct kvm_vcpu *vcpu)
{
	/*
	 * no need for rcu_read_lock as VCPU_RUN is the only place that
	 * will change the vcpu->pid pointer and on uninit all file
	 * descriptors are already gone.
	 */
	put_pid(rcu_dereference_protected(vcpu->pid, 1));
	kvm_arch_vcpu_uninit(vcpu);
	free_page((unsigned long)vcpu->run);
}
EXPORT_SYMBOL_GPL(kvm_vcpu_uninit);

#if defined(CONFIG_MMU_NOTIFIER) && defined(KVM_ARCH_WANT_MMU_NOTIFIER)
static inline struct kvm *mmu_notifier_to_kvm(struct mmu_notifier *mn)
{
	return container_of(mn, struct kvm, mmu_notifier);
}

static void kvm_mmu_notifier_change_pte(struct mmu_notifier *mn,
					struct mm_struct *mm,
					unsigned long address,
					pte_t pte)
{
	struct kvm *kvm = mmu_notifier_to_kvm(mn);
	int idx;

	idx = srcu_read_lock(&kvm->srcu);
	spin_lock(&kvm->mmu_lock);
	kvm->mmu_notifier_seq++;
	kvm_set_spte_hva(kvm, address, pte);
	spin_unlock(&kvm->mmu_lock);
	srcu_read_unlock(&kvm->srcu, idx);
}

static void kvm_mmu_notifier_invalidate_range_start(struct mmu_notifier *mn,
						    struct mm_struct *mm,
						    unsigned long start,
						    unsigned long end)
{
	struct kvm *kvm = mmu_notifier_to_kvm(mn);
	int need_tlb_flush = 0, idx;

	idx = srcu_read_lock(&kvm->srcu);
	spin_lock(&kvm->mmu_lock);
	/*
	 * The count increase must become visible at unlock time as no
	 * spte can be established without taking the mmu_lock and
	 * count is also read inside the mmu_lock critical section.
	 */
	kvm->mmu_notifier_count++;
	need_tlb_flush = kvm_unmap_hva_range(kvm, start, end);
	need_tlb_flush |= kvm->tlbs_dirty;
	/* we've to flush the tlb before the pages can be freed */
	if (need_tlb_flush)
		kvm_flush_remote_tlbs(kvm);

	spin_unlock(&kvm->mmu_lock);

	kvm_arch_mmu_notifier_invalidate_range(kvm, start, end);

	srcu_read_unlock(&kvm->srcu, idx);
}

static void kvm_mmu_notifier_invalidate_range_end(struct mmu_notifier *mn,
						  struct mm_struct *mm,
						  unsigned long start,
						  unsigned long end)
{
	struct kvm *kvm = mmu_notifier_to_kvm(mn);

	spin_lock(&kvm->mmu_lock);
	/*
	 * This sequence increase will notify the kvm page fault that
	 * the page that is going to be mapped in the spte could have
	 * been freed.
	 */
	kvm->mmu_notifier_seq++;
	smp_wmb();
	/*
	 * The above sequence increase must be visible before the
	 * below count decrease, which is ensured by the smp_wmb above
	 * in conjunction with the smp_rmb in mmu_notifier_retry().
	 */
	kvm->mmu_notifier_count--;
	spin_unlock(&kvm->mmu_lock);

	BUG_ON(kvm->mmu_notifier_count < 0);
}

static int kvm_mmu_notifier_clear_flush_young(struct mmu_notifier *mn,
					      struct mm_struct *mm,
					      unsigned long start,
					      unsigned long end)
{
	struct kvm *kvm = mmu_notifier_to_kvm(mn);
	int young, idx;

	idx = srcu_read_lock(&kvm->srcu);
	spin_lock(&kvm->mmu_lock);

	young = kvm_age_hva(kvm, start, end);
	if (young)
		kvm_flush_remote_tlbs(kvm);

	spin_unlock(&kvm->mmu_lock);
	srcu_read_unlock(&kvm->srcu, idx);

	return young;
}

static int kvm_mmu_notifier_clear_young(struct mmu_notifier *mn,
					struct mm_struct *mm,
					unsigned long start,
					unsigned long end)
{
	struct kvm *kvm = mmu_notifier_to_kvm(mn);
	int young, idx;

	idx = srcu_read_lock(&kvm->srcu);
	spin_lock(&kvm->mmu_lock);
	/*
	 * Even though we do not flush TLB, this will still adversely
	 * affect performance on pre-Haswell Intel EPT, where there is
	 * no EPT Access Bit to clear so that we have to tear down EPT
	 * tables instead. If we find this unacceptable, we can always
	 * add a parameter to kvm_age_hva so that it effectively doesn't
	 * do anything on clear_young.
	 *
	 * Also note that currently we never issue secondary TLB flushes
	 * from clear_young, leaving this job up to the regular system
	 * cadence. If we find this inaccurate, we might come up with a
	 * more sophisticated heuristic later.
	 */
	young = kvm_age_hva(kvm, start, end);
	spin_unlock(&kvm->mmu_lock);
	srcu_read_unlock(&kvm->srcu, idx);

	return young;
}

static int kvm_mmu_notifier_test_young(struct mmu_notifier *mn,
				       struct mm_struct *mm,
				       unsigned long address)
{
	struct kvm *kvm = mmu_notifier_to_kvm(mn);
	int young, idx;

	idx = srcu_read_lock(&kvm->srcu);
	spin_lock(&kvm->mmu_lock);
	young = kvm_test_age_hva(kvm, address);
	spin_unlock(&kvm->mmu_lock);
	srcu_read_unlock(&kvm->srcu, idx);

	return young;
}

static void kvm_mmu_notifier_release(struct mmu_notifier *mn,
				     struct mm_struct *mm)
{
	struct kvm *kvm = mmu_notifier_to_kvm(mn);
	int idx;

	idx = srcu_read_lock(&kvm->srcu);
	kvm_arch_flush_shadow_all(kvm);
	srcu_read_unlock(&kvm->srcu, idx);
}

static const struct mmu_notifier_ops kvm_mmu_notifier_ops = {
	.invalidate_range_start	= kvm_mmu_notifier_invalidate_range_start,
	.invalidate_range_end	= kvm_mmu_notifier_invalidate_range_end,
	.clear_flush_young	= kvm_mmu_notifier_clear_flush_young,
	.clear_young		= kvm_mmu_notifier_clear_young,
	.test_young		= kvm_mmu_notifier_test_young,
	.change_pte		= kvm_mmu_notifier_change_pte,
	.release		= kvm_mmu_notifier_release,
};

static int kvm_init_mmu_notifier(struct kvm *kvm)
{
	kvm->mmu_notifier.ops = &kvm_mmu_notifier_ops;
	return mmu_notifier_register(&kvm->mmu_notifier, current->mm);
}

#else  /* !(CONFIG_MMU_NOTIFIER && KVM_ARCH_WANT_MMU_NOTIFIER) */

static int kvm_init_mmu_notifier(struct kvm *kvm)
{
	return 0;
}

#endif /* CONFIG_MMU_NOTIFIER && KVM_ARCH_WANT_MMU_NOTIFIER */

static struct kvm_memslots *kvm_alloc_memslots(void)
{
	int i;
	struct kvm_memslots *slots;

	slots = kvzalloc(sizeof(struct kvm_memslots), GFP_KERNEL);
	if (!slots)
		return NULL;

	for (i = 0; i < KVM_MEM_SLOTS_NUM; i++)
		slots->id_to_index[i] = slots->memslots[i].id = i;

	return slots;
}

static void kvm_destroy_dirty_bitmap(struct kvm_memory_slot *memslot)
{
	if (!memslot->dirty_bitmap)
		return;

	kvfree(memslot->dirty_bitmap);
	memslot->dirty_bitmap = NULL;
}

/*
 * Free any memory in @free but not in @dont.
 */
static void kvm_free_memslot(struct kvm *kvm, struct kvm_memory_slot *free,
			      struct kvm_memory_slot *dont)
{
	if (!dont || free->dirty_bitmap != dont->dirty_bitmap)
		kvm_destroy_dirty_bitmap(free);

	kvm_arch_free_memslot(kvm, free, dont);

	free->npages = 0;
}

static void kvm_free_memslots(struct kvm *kvm, struct kvm_memslots *slots)
{
	struct kvm_memory_slot *memslot;

	if (!slots)
		return;

	kvm_for_each_memslot(memslot, slots)
		kvm_free_memslot(kvm, memslot, NULL);

	kvfree(slots);
}

static void kvm_destroy_vm_debugfs(struct kvm *kvm)
{
	int i;

	if (!kvm->debugfs_dentry)
		return;

	debugfs_remove_recursive(kvm->debugfs_dentry);

	if (kvm->debugfs_stat_data) {
		for (i = 0; i < kvm_debugfs_num_entries; i++)
			kfree(kvm->debugfs_stat_data[i]);
		kfree(kvm->debugfs_stat_data);
	}
}

static int kvm_create_vm_debugfs(struct kvm *kvm, int fd)
{
	char dir_name[ITOA_MAX_LEN * 2];
	struct kvm_stat_data *stat_data;
	struct kvm_stats_debugfs_item *p;

	if (!debugfs_initialized())
		return 0;

	snprintf(dir_name, sizeof(dir_name), "%d-%d", task_pid_nr(current), fd);
	kvm->debugfs_dentry = debugfs_create_dir(dir_name,
						 kvm_debugfs_dir);
	if (!kvm->debugfs_dentry)
		return -ENOMEM;

	kvm->debugfs_stat_data = kcalloc(kvm_debugfs_num_entries,
					 sizeof(*kvm->debugfs_stat_data),
					 GFP_KERNEL);
	if (!kvm->debugfs_stat_data)
		return -ENOMEM;

	for (p = debugfs_entries; p->name; p++) {
		stat_data = kzalloc(sizeof(*stat_data), GFP_KERNEL);
		if (!stat_data)
			return -ENOMEM;

		stat_data->kvm = kvm;
		stat_data->offset = p->offset;
		stat_data->mode = p->mode ? p->mode : 0644;
		kvm->debugfs_stat_data[p - debugfs_entries] = stat_data;
		if (!debugfs_create_file(p->name, stat_data->mode,
					 kvm->debugfs_dentry,
					 stat_data,
					 stat_fops_per_vm[p->kind]))
			return -ENOMEM;
	}
	return 0;
}

/*
 * Called after the VM is otherwise initialized, but just before adding it to
 * the vm_list.
 */
int __weak kvm_arch_post_init_vm(struct kvm *kvm)
{
	return 0;
}

/*
 * Called just after removing the VM from the vm_list, but before doing any
 * other destruction.
 */
void __weak kvm_arch_pre_destroy_vm(struct kvm *kvm)
{
}

static struct kvm *kvm_create_vm(unsigned long type)
{
	int r, i;
	struct kvm *kvm = kvm_arch_alloc_vm();

	if (!kvm)
		return ERR_PTR(-ENOMEM);

	spin_lock_init(&kvm->mmu_lock);
	mmgrab(current->mm);
	kvm->mm = current->mm;
	kvm_eventfd_init(kvm);
	mutex_init(&kvm->lock);
	mutex_init(&kvm->irq_lock);
	mutex_init(&kvm->slots_lock);
	refcount_set(&kvm->users_count, 1);
	INIT_LIST_HEAD(&kvm->devices);

	r = kvm_arch_init_vm(kvm, type);
	if (r)
		goto out_err_no_disable;

	r = hardware_enable_all();
	if (r)
		goto out_err_no_disable;

#ifdef CONFIG_HAVE_KVM_IRQFD
	INIT_HLIST_HEAD(&kvm->irq_ack_notifier_list);
#endif

	BUILD_BUG_ON(KVM_MEM_SLOTS_NUM > SHRT_MAX);

	r = -ENOMEM;
	for (i = 0; i < KVM_ADDRESS_SPACE_NUM; i++) {
		struct kvm_memslots *slots = kvm_alloc_memslots();
		if (!slots)
			goto out_err_no_srcu;
		/*
		 * Generations must be different for each address space.
		 * Init kvm generation close to the maximum to easily test the
		 * code of handling generation number wrap-around.
		 */
		slots->generation = i * 2 - 150;
		rcu_assign_pointer(kvm->memslots[i], slots);
	}

	if (init_srcu_struct(&kvm->srcu))
		goto out_err_no_srcu;
	if (init_srcu_struct(&kvm->irq_srcu))
		goto out_err_no_irq_srcu;
	for (i = 0; i < KVM_NR_BUSES; i++) {
		rcu_assign_pointer(kvm->buses[i],
			kzalloc(sizeof(struct kvm_io_bus), GFP_KERNEL));
		if (!kvm->buses[i])
			goto out_err_no_mmu_notifier;
	}

	r = kvm_init_mmu_notifier(kvm);
	if (r)
		goto out_err_no_mmu_notifier;

	r = kvm_arch_post_init_vm(kvm);
	if (r)
		goto out_err;

	mutex_lock(&kvm_lock);
	list_add(&kvm->vm_list, &vm_list);
	mutex_unlock(&kvm_lock);

	preempt_notifier_inc();

	return kvm;

out_err:
#if defined(CONFIG_MMU_NOTIFIER) && defined(KVM_ARCH_WANT_MMU_NOTIFIER)
	if (kvm->mmu_notifier.ops)
		mmu_notifier_unregister(&kvm->mmu_notifier, current->mm);
#endif
out_err_no_mmu_notifier:
	cleanup_srcu_struct(&kvm->irq_srcu);
out_err_no_irq_srcu:
	cleanup_srcu_struct(&kvm->srcu);
out_err_no_srcu:
	hardware_disable_all();
out_err_no_disable:
	refcount_set(&kvm->users_count, 0);
	for (i = 0; i < KVM_NR_BUSES; i++)
		kfree(kvm_get_bus(kvm, i));
	for (i = 0; i < KVM_ADDRESS_SPACE_NUM; i++)
		kvm_free_memslots(kvm, __kvm_memslots(kvm, i));
	kvm_arch_free_vm(kvm);
	mmdrop(current->mm);
	return ERR_PTR(r);
}

static void kvm_destroy_devices(struct kvm *kvm)
{
	struct kvm_device *dev, *tmp;

	/*
	 * We do not need to take the kvm->lock here, because nobody else
	 * has a reference to the struct kvm at this point and therefore
	 * cannot access the devices list anyhow.
	 */
	list_for_each_entry_safe(dev, tmp, &kvm->devices, vm_node) {
		list_del(&dev->vm_node);
		dev->ops->destroy(dev);
	}
}

static void kvm_destroy_vm(struct kvm *kvm)
{
	int i;
	struct mm_struct *mm = kvm->mm;

	kvm_uevent_notify_change(KVM_EVENT_DESTROY_VM, kvm);
	kvm_destroy_vm_debugfs(kvm);
	kvm_arch_sync_events(kvm);
	mutex_lock(&kvm_lock);
	list_del(&kvm->vm_list);
	mutex_unlock(&kvm_lock);
	kvm_arch_pre_destroy_vm(kvm);

	kvm_free_irq_routing(kvm);
	for (i = 0; i < KVM_NR_BUSES; i++) {
		struct kvm_io_bus *bus = kvm_get_bus(kvm, i);

		if (bus)
			kvm_io_bus_destroy(bus);
		kvm->buses[i] = NULL;
	}
	kvm_coalesced_mmio_free(kvm);
#if defined(CONFIG_MMU_NOTIFIER) && defined(KVM_ARCH_WANT_MMU_NOTIFIER)
	mmu_notifier_unregister(&kvm->mmu_notifier, kvm->mm);
#else
	kvm_arch_flush_shadow_all(kvm);
#endif
	kvm_arch_destroy_vm(kvm);
	kvm_destroy_devices(kvm);
	for (i = 0; i < KVM_ADDRESS_SPACE_NUM; i++)
		kvm_free_memslots(kvm, __kvm_memslots(kvm, i));
	cleanup_srcu_struct(&kvm->irq_srcu);
	cleanup_srcu_struct(&kvm->srcu);
	kvm_arch_free_vm(kvm);
	preempt_notifier_dec();
	hardware_disable_all();
	mmdrop(mm);
}

void kvm_get_kvm(struct kvm *kvm)
{
	refcount_inc(&kvm->users_count);
}
EXPORT_SYMBOL_GPL(kvm_get_kvm);

void kvm_put_kvm(struct kvm *kvm)
{
	if (refcount_dec_and_test(&kvm->users_count))
		kvm_destroy_vm(kvm);
}
EXPORT_SYMBOL_GPL(kvm_put_kvm);


static int kvm_vm_release(struct inode *inode, struct file *filp)
{
	struct kvm *kvm = filp->private_data;

	kvm_irqfd_release(kvm);

	kvm_put_kvm(kvm);
	return 0;
}

/*
 * Allocation size is twice as large as the actual dirty bitmap size.
 * See x86's kvm_vm_ioctl_get_dirty_log() why this is needed.
 */
static int kvm_create_dirty_bitmap(struct kvm_memory_slot *memslot)
{
	unsigned long dirty_bytes = 2 * kvm_dirty_bitmap_bytes(memslot);

	memslot->dirty_bitmap = kvzalloc(dirty_bytes, GFP_KERNEL);
	if (!memslot->dirty_bitmap)
		return -ENOMEM;

	return 0;
}

/*
 * Insert memslot and re-sort memslots based on their GFN,
 * so binary search could be used to lookup GFN.
 * Sorting algorithm takes advantage of having initially
 * sorted array and known changed memslot position.
 */
static void update_memslots(struct kvm_memslots *slots,
			    struct kvm_memory_slot *new)
{
	int id = new->id;
	int i = slots->id_to_index[id];
	struct kvm_memory_slot *mslots = slots->memslots;

	WARN_ON(mslots[i].id != id);
	if (!new->npages) {
		WARN_ON(!mslots[i].npages);
		if (mslots[i].npages)
			slots->used_slots--;
	} else {
		if (!mslots[i].npages)
			slots->used_slots++;
	}

	while (i < KVM_MEM_SLOTS_NUM - 1 &&
	       new->base_gfn <= mslots[i + 1].base_gfn) {
		if (!mslots[i + 1].npages)
			break;
		mslots[i] = mslots[i + 1];
		slots->id_to_index[mslots[i].id] = i;
		i++;
	}

	/*
	 * The ">=" is needed when creating a slot with base_gfn == 0,
	 * so that it moves before all those with base_gfn == npages == 0.
	 *
	 * On the other hand, if new->npages is zero, the above loop has
	 * already left i pointing to the beginning of the empty part of
	 * mslots, and the ">=" would move the hole backwards in this
	 * case---which is wrong.  So skip the loop when deleting a slot.
	 */
	if (new->npages) {
		while (i > 0 &&
		       new->base_gfn >= mslots[i - 1].base_gfn) {
			mslots[i] = mslots[i - 1];
			slots->id_to_index[mslots[i].id] = i;
			i--;
		}
	} else
		WARN_ON_ONCE(i != slots->used_slots);

	mslots[i] = *new;
	slots->id_to_index[mslots[i].id] = i;
}

static int check_memory_region_flags(const struct kvm_userspace_memory_region *mem)
{
	u32 valid_flags = KVM_MEM_LOG_DIRTY_PAGES;

#ifdef __KVM_HAVE_READONLY_MEM
	valid_flags |= KVM_MEM_READONLY;
#endif

	if (mem->flags & ~valid_flags)
		return -EINVAL;

	return 0;
}

static struct kvm_memslots *install_new_memslots(struct kvm *kvm,
		int as_id, struct kvm_memslots *slots)
{
	struct kvm_memslots *old_memslots = __kvm_memslots(kvm, as_id);
	u64 gen;

	/*
	 * Set the low bit in the generation, which disables SPTE caching
	 * until the end of synchronize_srcu_expedited.
	 */
	WARN_ON(old_memslots->generation & 1);
	slots->generation = old_memslots->generation + 1;

	rcu_assign_pointer(kvm->memslots[as_id], slots);
	synchronize_srcu_expedited(&kvm->srcu);

	/*
	 * Increment the new memslot generation a second time. This prevents
	 * vm exits that race with memslot updates from caching a memslot
	 * generation that will (potentially) be valid forever.
	 *
	 * Generations must be unique even across address spaces.  We do not need
	 * a global counter for that, instead the generation space is evenly split
	 * across address spaces.  For example, with two address spaces, address
	 * space 0 will use generations 0, 4, 8, ... while * address space 1 will
	 * use generations 2, 6, 10, 14, ...
	 */
	gen = slots->generation + KVM_ADDRESS_SPACE_NUM * 2 - 1;

	kvm_arch_memslots_updated(kvm, gen);

	slots->generation = gen;

	return old_memslots;
}

/*
 * Allocate some memory and give it an address in the guest physical address
 * space.
 *
 * Discontiguous memory is allowed, mostly for framebuffers.
 *
 * Must be called holding kvm->slots_lock for write.
 */
int __kvm_set_memory_region(struct kvm *kvm,
			    const struct kvm_userspace_memory_region *mem)
{
	int r;
	gfn_t base_gfn;
	unsigned long npages;
	struct kvm_memory_slot *slot;
	struct kvm_memory_slot old, new;
	struct kvm_memslots *slots = NULL, *old_memslots;
	int as_id, id;
	enum kvm_mr_change change;

	r = check_memory_region_flags(mem);
	if (r)
		goto out;

	r = -EINVAL;
	as_id = mem->slot >> 16;
	id = (u16)mem->slot;

	/* General sanity checks */
	if (mem->memory_size & (PAGE_SIZE - 1))
		goto out;
	if (mem->guest_phys_addr & (PAGE_SIZE - 1))
		goto out;
	/* We can read the guest memory with __xxx_user() later on. */
	if ((id < KVM_USER_MEM_SLOTS) &&
	    ((mem->userspace_addr & (PAGE_SIZE - 1)) ||
	     !access_ok(VERIFY_WRITE,
			(void __user *)(unsigned long)mem->userspace_addr,
			mem->memory_size)))
		goto out;
	if (as_id >= KVM_ADDRESS_SPACE_NUM || id >= KVM_MEM_SLOTS_NUM)
		goto out;
	if (mem->guest_phys_addr + mem->memory_size < mem->guest_phys_addr)
		goto out;

	slot = id_to_memslot(__kvm_memslots(kvm, as_id), id);
	base_gfn = mem->guest_phys_addr >> PAGE_SHIFT;
	npages = mem->memory_size >> PAGE_SHIFT;

	if (npages > KVM_MEM_MAX_NR_PAGES)
		goto out;

	new = old = *slot;

	new.id = id;
	new.base_gfn = base_gfn;
	new.npages = npages;
	new.flags = mem->flags;

	if (npages) {
		if (!old.npages)
			change = KVM_MR_CREATE;
		else { /* Modify an existing slot. */
			if ((mem->userspace_addr != old.userspace_addr) ||
			    (npages != old.npages) ||
			    ((new.flags ^ old.flags) & KVM_MEM_READONLY))
				goto out;

			if (base_gfn != old.base_gfn)
				change = KVM_MR_MOVE;
			else if (new.flags != old.flags)
				change = KVM_MR_FLAGS_ONLY;
			else { /* Nothing to change. */
				r = 0;
				goto out;
			}
		}
	} else {
		if (!old.npages)
			goto out;

		change = KVM_MR_DELETE;
		new.base_gfn = 0;
		new.flags = 0;
	}

	if ((change == KVM_MR_CREATE) || (change == KVM_MR_MOVE)) {
		/* Check for overlaps */
		r = -EEXIST;
		kvm_for_each_memslot(slot, __kvm_memslots(kvm, as_id)) {
			if (slot->id == id)
				continue;
			if (!((base_gfn + npages <= slot->base_gfn) ||
			      (base_gfn >= slot->base_gfn + slot->npages)))
				goto out;
		}
	}

	/* Free page dirty bitmap if unneeded */
	if (!(new.flags & KVM_MEM_LOG_DIRTY_PAGES))
		new.dirty_bitmap = NULL;

	r = -ENOMEM;
	if (change == KVM_MR_CREATE) {
		new.userspace_addr = mem->userspace_addr;

		if (kvm_arch_create_memslot(kvm, &new, npages))
			goto out_free;
	}

	/* Allocate page dirty bitmap if needed */
	if ((new.flags & KVM_MEM_LOG_DIRTY_PAGES) && !new.dirty_bitmap) {
		if (kvm_create_dirty_bitmap(&new) < 0)
			goto out_free;
	}

	slots = kvzalloc(sizeof(struct kvm_memslots), GFP_KERNEL);
	if (!slots)
		goto out_free;
	memcpy(slots, __kvm_memslots(kvm, as_id), sizeof(struct kvm_memslots));

	if ((change == KVM_MR_DELETE) || (change == KVM_MR_MOVE)) {
		slot = id_to_memslot(slots, id);
		slot->flags |= KVM_MEMSLOT_INVALID;

		old_memslots = install_new_memslots(kvm, as_id, slots);

		/* From this point no new shadow pages pointing to a deleted,
		 * or moved, memslot will be created.
		 *
		 * validation of sp->gfn happens in:
		 *	- gfn_to_hva (kvm_read_guest, gfn_to_pfn)
		 *	- kvm_is_visible_gfn (mmu_check_roots)
		 */
		kvm_arch_flush_shadow_memslot(kvm, slot);

		/*
		 * We can re-use the old_memslots from above, the only difference
		 * from the currently installed memslots is the invalid flag.  This
		 * will get overwritten by update_memslots anyway.
		 */
		slots = old_memslots;
	}

	r = kvm_arch_prepare_memory_region(kvm, &new, mem, change);
	if (r)
		goto out_slots;

	/* actual memory is freed via old in kvm_free_memslot below */
	if (change == KVM_MR_DELETE) {
		new.dirty_bitmap = NULL;
		memset(&new.arch, 0, sizeof(new.arch));
	}

	update_memslots(slots, &new);
	old_memslots = install_new_memslots(kvm, as_id, slots);

	kvm_arch_commit_memory_region(kvm, mem, &old, &new, change);

	kvm_free_memslot(kvm, &old, &new);
	kvfree(old_memslots);
	return 0;

out_slots:
	kvfree(slots);
out_free:
	kvm_free_memslot(kvm, &new, &old);
out:
	return r;
}
EXPORT_SYMBOL_GPL(__kvm_set_memory_region);

int kvm_set_memory_region(struct kvm *kvm,
			  const struct kvm_userspace_memory_region *mem)
{
	int r;

	mutex_lock(&kvm->slots_lock);
	r = __kvm_set_memory_region(kvm, mem);
	mutex_unlock(&kvm->slots_lock);
	return r;
}
EXPORT_SYMBOL_GPL(kvm_set_memory_region);

static int kvm_vm_ioctl_set_memory_region(struct kvm *kvm,
					  struct kvm_userspace_memory_region *mem)
{
	if ((u16)mem->slot >= KVM_USER_MEM_SLOTS)
		return -EINVAL;

	return kvm_set_memory_region(kvm, mem);
}

int kvm_get_dirty_log(struct kvm *kvm,
			struct kvm_dirty_log *log, int *is_dirty)
{
	struct kvm_memslots *slots;
	struct kvm_memory_slot *memslot;
	int i, as_id, id;
	unsigned long n;
	unsigned long any = 0;

	as_id = log->slot >> 16;
	id = (u16)log->slot;
	if (as_id >= KVM_ADDRESS_SPACE_NUM || id >= KVM_USER_MEM_SLOTS)
		return -EINVAL;

	slots = __kvm_memslots(kvm, as_id);
	memslot = id_to_memslot(slots, id);
	if (!memslot->dirty_bitmap)
		return -ENOENT;

	n = kvm_dirty_bitmap_bytes(memslot);

	for (i = 0; !any && i < n/sizeof(long); ++i)
		any = memslot->dirty_bitmap[i];

	if (copy_to_user(log->dirty_bitmap, memslot->dirty_bitmap, n))
		return -EFAULT;

	if (any)
		*is_dirty = 1;
	return 0;
}
EXPORT_SYMBOL_GPL(kvm_get_dirty_log);

#ifdef CONFIG_KVM_GENERIC_DIRTYLOG_READ_PROTECT
/**
 * kvm_get_dirty_log_protect - get a snapshot of dirty pages, and if any pages
 *	are dirty write protect them for next write.
 * @kvm:	pointer to kvm instance
 * @log:	slot id and address to which we copy the log
 * @is_dirty:	flag set if any page is dirty
 *
 * We need to keep it in mind that VCPU threads can write to the bitmap
 * concurrently. So, to avoid losing track of dirty pages we keep the
 * following order:
 *
 *    1. Take a snapshot of the bit and clear it if needed.
 *    2. Write protect the corresponding page.
 *    3. Copy the snapshot to the userspace.
 *    4. Upon return caller flushes TLB's if needed.
 *
 * Between 2 and 4, the guest may write to the page using the remaining TLB
 * entry.  This is not a problem because the page is reported dirty using
 * the snapshot taken before and step 4 ensures that writes done after
 * exiting to userspace will be logged for the next call.
 *
 */
int kvm_get_dirty_log_protect(struct kvm *kvm,
			struct kvm_dirty_log *log, bool *is_dirty)
{
	struct kvm_memslots *slots;
	struct kvm_memory_slot *memslot;
	int i, as_id, id;
	unsigned long n;
	unsigned long *dirty_bitmap;
	unsigned long *dirty_bitmap_buffer;

	as_id = log->slot >> 16;
	id = (u16)log->slot;
	if (as_id >= KVM_ADDRESS_SPACE_NUM || id >= KVM_USER_MEM_SLOTS)
		return -EINVAL;

	slots = __kvm_memslots(kvm, as_id);
	memslot = id_to_memslot(slots, id);

	dirty_bitmap = memslot->dirty_bitmap;
	if (!dirty_bitmap)
		return -ENOENT;

	n = kvm_dirty_bitmap_bytes(memslot);

	dirty_bitmap_buffer = dirty_bitmap + n / sizeof(long);
	memset(dirty_bitmap_buffer, 0, n);

	spin_lock(&kvm->mmu_lock);
	*is_dirty = false;
	for (i = 0; i < n / sizeof(long); i++) {
		unsigned long mask;
		gfn_t offset;

		if (!dirty_bitmap[i])
			continue;

		*is_dirty = true;

		mask = xchg(&dirty_bitmap[i], 0);
		dirty_bitmap_buffer[i] = mask;

		if (mask) {
			offset = i * BITS_PER_LONG;
			kvm_arch_mmu_enable_log_dirty_pt_masked(kvm, memslot,
								offset, mask);
		}
	}

	spin_unlock(&kvm->mmu_lock);
	if (copy_to_user(log->dirty_bitmap, dirty_bitmap_buffer, n))
		return -EFAULT;
	return 0;
}
EXPORT_SYMBOL_GPL(kvm_get_dirty_log_protect);
#endif

bool kvm_largepages_enabled(void)
{
	return largepages_enabled;
}

void kvm_disable_largepages(void)
{
	largepages_enabled = false;
}
EXPORT_SYMBOL_GPL(kvm_disable_largepages);

struct kvm_memory_slot *gfn_to_memslot(struct kvm *kvm, gfn_t gfn)
{
	return __gfn_to_memslot(kvm_memslots(kvm), gfn);
}
EXPORT_SYMBOL_GPL(gfn_to_memslot);

struct kvm_memory_slot *kvm_vcpu_gfn_to_memslot(struct kvm_vcpu *vcpu, gfn_t gfn)
{
	return __gfn_to_memslot(kvm_vcpu_memslots(vcpu), gfn);
}

bool kvm_is_visible_gfn(struct kvm *kvm, gfn_t gfn)
{
	struct kvm_memory_slot *memslot = gfn_to_memslot(kvm, gfn);

	if (!memslot || memslot->id >= KVM_USER_MEM_SLOTS ||
	      memslot->flags & KVM_MEMSLOT_INVALID)
		return false;

	return true;
}
EXPORT_SYMBOL_GPL(kvm_is_visible_gfn);

unsigned long kvm_host_page_size(struct kvm_vcpu *vcpu, gfn_t gfn)
{
	struct vm_area_struct *vma;
	unsigned long addr, size;

	size = PAGE_SIZE;

	addr = kvm_vcpu_gfn_to_hva_prot(vcpu, gfn, NULL);
	if (kvm_is_error_hva(addr))
		return PAGE_SIZE;

	down_read(&current->mm->mmap_sem);
	vma = find_vma(current->mm, addr);
	if (!vma)
		goto out;

	size = vma_kernel_pagesize(vma);

out:
	up_read(&current->mm->mmap_sem);

	return size;
}

static bool memslot_is_readonly(struct kvm_memory_slot *slot)
{
	return slot->flags & KVM_MEM_READONLY;
}

static unsigned long __gfn_to_hva_many(struct kvm_memory_slot *slot, gfn_t gfn,
				       gfn_t *nr_pages, bool write)
{
	if (!slot || slot->flags & KVM_MEMSLOT_INVALID)
		return KVM_HVA_ERR_BAD;

	if (memslot_is_readonly(slot) && write)
		return KVM_HVA_ERR_RO_BAD;

	if (nr_pages)
		*nr_pages = slot->npages - (gfn - slot->base_gfn);

	return __gfn_to_hva_memslot(slot, gfn);
}

static unsigned long gfn_to_hva_many(struct kvm_memory_slot *slot, gfn_t gfn,
				     gfn_t *nr_pages)
{
	return __gfn_to_hva_many(slot, gfn, nr_pages, true);
}

unsigned long gfn_to_hva_memslot(struct kvm_memory_slot *slot,
					gfn_t gfn)
{
	return gfn_to_hva_many(slot, gfn, NULL);
}
EXPORT_SYMBOL_GPL(gfn_to_hva_memslot);

unsigned long gfn_to_hva(struct kvm *kvm, gfn_t gfn)
{
	return gfn_to_hva_many(gfn_to_memslot(kvm, gfn), gfn, NULL);
}
EXPORT_SYMBOL_GPL(gfn_to_hva);

unsigned long kvm_vcpu_gfn_to_hva(struct kvm_vcpu *vcpu, gfn_t gfn)
{
	return gfn_to_hva_many(kvm_vcpu_gfn_to_memslot(vcpu, gfn), gfn, NULL);
}
EXPORT_SYMBOL_GPL(kvm_vcpu_gfn_to_hva);

/*
 * If writable is set to false, the hva returned by this function is only
 * allowed to be read.
 */
unsigned long gfn_to_hva_memslot_prot(struct kvm_memory_slot *slot,
				      gfn_t gfn, bool *writable)
{
	unsigned long hva = __gfn_to_hva_many(slot, gfn, NULL, false);

	if (!kvm_is_error_hva(hva) && writable)
		*writable = !memslot_is_readonly(slot);

	return hva;
}

unsigned long gfn_to_hva_prot(struct kvm *kvm, gfn_t gfn, bool *writable)
{
	struct kvm_memory_slot *slot = gfn_to_memslot(kvm, gfn);

	return gfn_to_hva_memslot_prot(slot, gfn, writable);
}

unsigned long kvm_vcpu_gfn_to_hva_prot(struct kvm_vcpu *vcpu, gfn_t gfn, bool *writable)
{
	struct kvm_memory_slot *slot = kvm_vcpu_gfn_to_memslot(vcpu, gfn);

	return gfn_to_hva_memslot_prot(slot, gfn, writable);
}

static int get_user_page_nowait(unsigned long start, int write,
		struct page **page)
{
	int flags = FOLL_NOWAIT | FOLL_HWPOISON;

	if (write)
		flags |= FOLL_WRITE;

	return get_user_pages(start, 1, flags, page, NULL);
}

static inline int check_user_page_hwpoison(unsigned long addr)
{
	int rc, flags = FOLL_HWPOISON | FOLL_WRITE;

	rc = get_user_pages(addr, 1, flags, NULL, NULL);
	return rc == -EHWPOISON;
}

/*
 * The atomic path to get the writable pfn which will be stored in @pfn,
 * true indicates success, otherwise false is returned.
 */
static bool hva_to_pfn_fast(unsigned long addr, bool atomic, bool *async,
			    bool write_fault, bool *writable, kvm_pfn_t *pfn)
{
	struct page *page[1];
	int npages;

	if (!(async || atomic))
		return false;

	/*
	 * Fast pin a writable pfn only if it is a write fault request
	 * or the caller allows to map a writable pfn for a read fault
	 * request.
	 */
	if (!(write_fault || writable))
		return false;

	npages = __get_user_pages_fast(addr, 1, 1, page);
	if (npages == 1) {
		*pfn = page_to_pfn(page[0]);

		if (writable)
			*writable = true;
		return true;
	}

	return false;
}

/*
 * The slow path to get the pfn of the specified host virtual address,
 * 1 indicates success, -errno is returned if error is detected.
 */
static int hva_to_pfn_slow(unsigned long addr, bool *async, bool write_fault,
			   bool *writable, kvm_pfn_t *pfn)
{
	struct page *page[1];
	int npages = 0;

	might_sleep();

	if (writable)
		*writable = write_fault;

	if (async) {
		down_read(&current->mm->mmap_sem);
		npages = get_user_page_nowait(addr, write_fault, page);
		up_read(&current->mm->mmap_sem);
	} else {
		unsigned int flags = FOLL_HWPOISON;

		if (write_fault)
			flags |= FOLL_WRITE;

		npages = get_user_pages_unlocked(addr, 1, page, flags);
	}
	if (npages != 1)
		return npages;

	/* map read fault as writable if possible */
	if (unlikely(!write_fault) && writable) {
		struct page *wpage[1];

		npages = __get_user_pages_fast(addr, 1, 1, wpage);
		if (npages == 1) {
			*writable = true;
			put_page(page[0]);
			page[0] = wpage[0];
		}

		npages = 1;
	}
	*pfn = page_to_pfn(page[0]);
	return npages;
}

static bool vma_is_valid(struct vm_area_struct *vma, bool write_fault)
{
	if (unlikely(!(vma->vm_flags & VM_READ)))
		return false;

	if (write_fault && (unlikely(!(vma->vm_flags & VM_WRITE))))
		return false;

	return true;
}

static int hva_to_pfn_remapped(struct vm_area_struct *vma,
			       unsigned long addr, bool *async,
			       bool write_fault, bool *writable,
			       kvm_pfn_t *p_pfn)
{
	unsigned long pfn;
	int r;

	r = follow_pfn(vma, addr, &pfn);
	if (r) {
		/*
		 * get_user_pages fails for VM_IO and VM_PFNMAP vmas and does
		 * not call the fault handler, so do it here.
		 */
		bool unlocked = false;
		r = fixup_user_fault(current, current->mm, addr,
				     (write_fault ? FAULT_FLAG_WRITE : 0),
				     &unlocked);
		if (unlocked)
			return -EAGAIN;
		if (r)
			return r;

		r = follow_pfn(vma, addr, &pfn);
		if (r)
			return r;

	}

	if (writable)
		*writable = true;

	/*
	 * Get a reference here because callers of *hva_to_pfn* and
	 * *gfn_to_pfn* ultimately call kvm_release_pfn_clean on the
	 * returned pfn.  This is only needed if the VMA has VM_MIXEDMAP
	 * set, but the kvm_get_pfn/kvm_release_pfn_clean pair will
	 * simply do nothing for reserved pfns.
	 *
	 * Whoever called remap_pfn_range is also going to call e.g.
	 * unmap_mapping_range before the underlying pages are freed,
	 * causing a call to our MMU notifier.
	 */ 
	kvm_get_pfn(pfn);

	*p_pfn = pfn;
	return 0;
}

/*
 * Pin guest page in memory and return its pfn.
 * @addr: host virtual address which maps memory to the guest
 * @atomic: whether this function can sleep
 * @async: whether this function need to wait IO complete if the
 *         host page is not in the memory
 * @write_fault: whether we should get a writable host page
 * @writable: whether it allows to map a writable host page for !@write_fault
 *
 * The function will map a writable host page for these two cases:
 * 1): @write_fault = true
 * 2): @write_fault = false && @writable, @writable will tell the caller
 *     whether the mapping is writable.
 */
static kvm_pfn_t hva_to_pfn(unsigned long addr, bool atomic, bool *async,
			bool write_fault, bool *writable)
{
	struct vm_area_struct *vma;
	kvm_pfn_t pfn = 0;
	int npages, r;

	/* we can do it either atomically or asynchronously, not both */
	BUG_ON(atomic && async);

	if (hva_to_pfn_fast(addr, atomic, async, write_fault, writable, &pfn))
		return pfn;

	if (atomic)
		return KVM_PFN_ERR_FAULT;

	npages = hva_to_pfn_slow(addr, async, write_fault, writable, &pfn);
	if (npages == 1)
		return pfn;

	down_read(&current->mm->mmap_sem);
	if (npages == -EHWPOISON ||
	      (!async && check_user_page_hwpoison(addr))) {
		pfn = KVM_PFN_ERR_HWPOISON;
		goto exit;
	}

retry:
	vma = find_vma_intersection(current->mm, addr, addr + 1);

	if (vma == NULL)
		pfn = KVM_PFN_ERR_FAULT;
	else if (vma->vm_flags & (VM_IO | VM_PFNMAP)) {
		r = hva_to_pfn_remapped(vma, addr, async, write_fault, writable, &pfn);
		if (r == -EAGAIN)
			goto retry;
		if (r < 0)
			pfn = KVM_PFN_ERR_FAULT;
	} else {
		if (async && vma_is_valid(vma, write_fault))
			*async = true;
		pfn = KVM_PFN_ERR_FAULT;
	}
exit:
	up_read(&current->mm->mmap_sem);
	return pfn;
}

kvm_pfn_t __gfn_to_pfn_memslot(struct kvm_memory_slot *slot, gfn_t gfn,
			       bool atomic, bool *async, bool write_fault,
			       bool *writable)
{
	unsigned long addr = __gfn_to_hva_many(slot, gfn, NULL, write_fault);

	if (addr == KVM_HVA_ERR_RO_BAD) {
		if (writable)
			*writable = false;
		return KVM_PFN_ERR_RO_FAULT;
	}

	if (kvm_is_error_hva(addr)) {
		if (writable)
			*writable = false;
		return KVM_PFN_NOSLOT;
	}

	/* Do not map writable pfn in the readonly memslot. */
	if (writable && memslot_is_readonly(slot)) {
		*writable = false;
		writable = NULL;
	}

	return hva_to_pfn(addr, atomic, async, write_fault,
			  writable);
}
EXPORT_SYMBOL_GPL(__gfn_to_pfn_memslot);

kvm_pfn_t gfn_to_pfn_prot(struct kvm *kvm, gfn_t gfn, bool write_fault,
		      bool *writable)
{
	return __gfn_to_pfn_memslot(gfn_to_memslot(kvm, gfn), gfn, false, NULL,
				    write_fault, writable);
}
EXPORT_SYMBOL_GPL(gfn_to_pfn_prot);

kvm_pfn_t gfn_to_pfn_memslot(struct kvm_memory_slot *slot, gfn_t gfn)
{
	return __gfn_to_pfn_memslot(slot, gfn, false, NULL, true, NULL);
}
EXPORT_SYMBOL_GPL(gfn_to_pfn_memslot);

kvm_pfn_t gfn_to_pfn_memslot_atomic(struct kvm_memory_slot *slot, gfn_t gfn)
{
	return __gfn_to_pfn_memslot(slot, gfn, true, NULL, true, NULL);
}
EXPORT_SYMBOL_GPL(gfn_to_pfn_memslot_atomic);

kvm_pfn_t gfn_to_pfn_atomic(struct kvm *kvm, gfn_t gfn)
{
	return gfn_to_pfn_memslot_atomic(gfn_to_memslot(kvm, gfn), gfn);
}
EXPORT_SYMBOL_GPL(gfn_to_pfn_atomic);

kvm_pfn_t kvm_vcpu_gfn_to_pfn_atomic(struct kvm_vcpu *vcpu, gfn_t gfn)
{
	return gfn_to_pfn_memslot_atomic(kvm_vcpu_gfn_to_memslot(vcpu, gfn), gfn);
}
EXPORT_SYMBOL_GPL(kvm_vcpu_gfn_to_pfn_atomic);

kvm_pfn_t gfn_to_pfn(struct kvm *kvm, gfn_t gfn)
{
	return gfn_to_pfn_memslot(gfn_to_memslot(kvm, gfn), gfn);
}
EXPORT_SYMBOL_GPL(gfn_to_pfn);

kvm_pfn_t kvm_vcpu_gfn_to_pfn(struct kvm_vcpu *vcpu, gfn_t gfn)
{
	return gfn_to_pfn_memslot(kvm_vcpu_gfn_to_memslot(vcpu, gfn), gfn);
}
EXPORT_SYMBOL_GPL(kvm_vcpu_gfn_to_pfn);

int gfn_to_page_many_atomic(struct kvm_memory_slot *slot, gfn_t gfn,
			    struct page **pages, int nr_pages)
{
	unsigned long addr;
	gfn_t entry = 0;

	addr = gfn_to_hva_many(slot, gfn, &entry);
	if (kvm_is_error_hva(addr))
		return -1;

	if (entry < nr_pages)
		return 0;

	return __get_user_pages_fast(addr, nr_pages, 1, pages);
}
EXPORT_SYMBOL_GPL(gfn_to_page_many_atomic);

static struct page *kvm_pfn_to_page(kvm_pfn_t pfn)
{
	if (is_error_noslot_pfn(pfn))
		return KVM_ERR_PTR_BAD_PAGE;

	if (kvm_is_reserved_pfn(pfn)) {
		WARN_ON(1);
		return KVM_ERR_PTR_BAD_PAGE;
	}

	return pfn_to_page(pfn);
}

struct page *gfn_to_page(struct kvm *kvm, gfn_t gfn)
{
	kvm_pfn_t pfn;

	pfn = gfn_to_pfn(kvm, gfn);

	return kvm_pfn_to_page(pfn);
}
EXPORT_SYMBOL_GPL(gfn_to_page);

struct page *kvm_vcpu_gfn_to_page(struct kvm_vcpu *vcpu, gfn_t gfn)
{
	kvm_pfn_t pfn;

	pfn = kvm_vcpu_gfn_to_pfn(vcpu, gfn);

	return kvm_pfn_to_page(pfn);
}
EXPORT_SYMBOL_GPL(kvm_vcpu_gfn_to_page);

void kvm_release_page_clean(struct page *page)
{
	WARN_ON(is_error_page(page));

	kvm_release_pfn_clean(page_to_pfn(page));
}
EXPORT_SYMBOL_GPL(kvm_release_page_clean);

void kvm_release_pfn_clean(kvm_pfn_t pfn)
{
	if (!is_error_noslot_pfn(pfn) && !kvm_is_reserved_pfn(pfn))
		put_page(pfn_to_page(pfn));
}
EXPORT_SYMBOL_GPL(kvm_release_pfn_clean);

void kvm_release_page_dirty(struct page *page)
{
	WARN_ON(is_error_page(page));

	kvm_release_pfn_dirty(page_to_pfn(page));
}
EXPORT_SYMBOL_GPL(kvm_release_page_dirty);

static void kvm_release_pfn_dirty(kvm_pfn_t pfn)
{
	kvm_set_pfn_dirty(pfn);
	kvm_release_pfn_clean(pfn);
}

void kvm_set_pfn_dirty(kvm_pfn_t pfn)
{
	if (!kvm_is_reserved_pfn(pfn) && !kvm_is_zone_device_pfn(pfn)) {
		struct page *page = pfn_to_page(pfn);

		if (!PageReserved(page))
			SetPageDirty(page);
	}
}
EXPORT_SYMBOL_GPL(kvm_set_pfn_dirty);

void kvm_set_pfn_accessed(kvm_pfn_t pfn)
{
	if (!kvm_is_reserved_pfn(pfn) && !kvm_is_zone_device_pfn(pfn))
		mark_page_accessed(pfn_to_page(pfn));
}
EXPORT_SYMBOL_GPL(kvm_set_pfn_accessed);

void kvm_get_pfn(kvm_pfn_t pfn)
{
	if (!kvm_is_reserved_pfn(pfn))
		get_page(pfn_to_page(pfn));
}
EXPORT_SYMBOL_GPL(kvm_get_pfn);

static int next_segment(unsigned long len, int offset)
{
	if (len > PAGE_SIZE - offset)
		return PAGE_SIZE - offset;
	else
		return len;
}

static int __kvm_read_guest_page(struct kvm_memory_slot *slot, gfn_t gfn,
				 void *data, int offset, int len)
{
	int r;
	unsigned long addr;

	addr = gfn_to_hva_memslot_prot(slot, gfn, NULL);
	if (kvm_is_error_hva(addr))
		return -EFAULT;
	r = __copy_from_user(data, (void __user *)addr + offset, len);
	if (r)
		return -EFAULT;
	return 0;
}

int kvm_read_guest_page(struct kvm *kvm, gfn_t gfn, void *data, int offset,
			int len)
{
	struct kvm_memory_slot *slot = gfn_to_memslot(kvm, gfn);

	return __kvm_read_guest_page(slot, gfn, data, offset, len);
}
EXPORT_SYMBOL_GPL(kvm_read_guest_page);

int kvm_vcpu_read_guest_page(struct kvm_vcpu *vcpu, gfn_t gfn, void *data,
			     int offset, int len)
{
	struct kvm_memory_slot *slot = kvm_vcpu_gfn_to_memslot(vcpu, gfn);

	return __kvm_read_guest_page(slot, gfn, data, offset, len);
}
EXPORT_SYMBOL_GPL(kvm_vcpu_read_guest_page);

int kvm_read_guest(struct kvm *kvm, gpa_t gpa, void *data, unsigned long len)
{
	gfn_t gfn = gpa >> PAGE_SHIFT;
	int seg;
	int offset = offset_in_page(gpa);
	int ret;

	while ((seg = next_segment(len, offset)) != 0) {
		ret = kvm_read_guest_page(kvm, gfn, data, offset, seg);
		if (ret < 0)
			return ret;
		offset = 0;
		len -= seg;
		data += seg;
		++gfn;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(kvm_read_guest);

int kvm_vcpu_read_guest(struct kvm_vcpu *vcpu, gpa_t gpa, void *data, unsigned long len)
{
	gfn_t gfn = gpa >> PAGE_SHIFT;
	int seg;
	int offset = offset_in_page(gpa);
	int ret;

	while ((seg = next_segment(len, offset)) != 0) {
		ret = kvm_vcpu_read_guest_page(vcpu, gfn, data, offset, seg);
		if (ret < 0)
			return ret;
		offset = 0;
		len -= seg;
		data += seg;
		++gfn;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(kvm_vcpu_read_guest);

static int __kvm_read_guest_atomic(struct kvm_memory_slot *slot, gfn_t gfn,
			           void *data, int offset, unsigned long len)
{
	int r;
	unsigned long addr;

	addr = gfn_to_hva_memslot_prot(slot, gfn, NULL);
	if (kvm_is_error_hva(addr))
		return -EFAULT;
	pagefault_disable();
	r = __copy_from_user_inatomic(data, (void __user *)addr + offset, len);
	pagefault_enable();
	if (r)
		return -EFAULT;
	return 0;
}

int kvm_read_guest_atomic(struct kvm *kvm, gpa_t gpa, void *data,
			  unsigned long len)
{
	gfn_t gfn = gpa >> PAGE_SHIFT;
	struct kvm_memory_slot *slot = gfn_to_memslot(kvm, gfn);
	int offset = offset_in_page(gpa);

	return __kvm_read_guest_atomic(slot, gfn, data, offset, len);
}
EXPORT_SYMBOL_GPL(kvm_read_guest_atomic);

int kvm_vcpu_read_guest_atomic(struct kvm_vcpu *vcpu, gpa_t gpa,
			       void *data, unsigned long len)
{
	gfn_t gfn = gpa >> PAGE_SHIFT;
	struct kvm_memory_slot *slot = kvm_vcpu_gfn_to_memslot(vcpu, gfn);
	int offset = offset_in_page(gpa);

	return __kvm_read_guest_atomic(slot, gfn, data, offset, len);
}
EXPORT_SYMBOL_GPL(kvm_vcpu_read_guest_atomic);

static int __kvm_write_guest_page(struct kvm_memory_slot *memslot, gfn_t gfn,
			          const void *data, int offset, int len)
{
	int r;
	unsigned long addr;

	addr = gfn_to_hva_memslot(memslot, gfn);
	if (kvm_is_error_hva(addr))
		return -EFAULT;
	r = __copy_to_user(ZVEGvto_hva_memssWBh
EGyEGyEGb!Svm exits ByiyByEGUfDGyEGyA_user(ZByEGUBOL_ct kyryBU:GyEGyA_useBv_dirty0);
		diGUgned longot *memslirty0);
		diGUgned ByEGhf (!kvm(kvmifiedDGbfn);
}s; i++)
		dirty0);
		diGUgned longotZVEGw			inir(ZByEG(ZByEG(ZByEG(,;yEG(,;yEG(,;y,int offset, unsignedvyEGyEGg n)
{
gned lofDGySot = gfn_to_memZ2s_error_hva(addr))
		retureg;
	int offsea(addr))
		retU

	return __kvm_red lofDGT;
	stAWBh
EGyEGwe{
	strA?addr))
		retUBb VBh
EGv

	ret= 0;

	mighZ>etU

	return __kvm_red lof
	stAWBh
EGyEGwe{
	strA?adZVEGk_red vm(kv|VEGyEGyEGyEGyEGv conEGyEGyEGgin__kvm_red loo_pfn_prot);a.M0*lRt(vcp*writable = !mloo_pfn_prUgned 2 *data,
			Sfl *asynct kv map stall * msuct wo;
	}

	if,>vm_ftaddr,n != ern th kvm_(mslottireap stallisll thos,
 * 1 indicattruct ks_error_hv<gfn_to_men @pfn,etureg;
	int ot(memslo)
		retUBb VBh
EGv

	ret= 0;

	mighZ>etU

	return __k	kvm_red lof
	stAWBh
EGyEGwe{
	strA?adZVEGk_red vm(kv
	MEMSLOT &etureg;
	int o __k	kegment(len, offset)) n__kvm_rer on. ret;
		offset = 0_manerror_hva=retureg;
	int offld, ne/* Uet_dirto_hva_prot	    * msl, &en	  cone[0]memsloslot;
	
		retUBb VBh
Eck);

	pre	}
	return 0;
}
EXPORexits ByiyByEGUfDGyEGyA_user(ZBystruct _dirty0);
		diGUgned longot *memsrty0);
		diGUgned ByEGhf (!kvm(kvmifiedtruct kvm_memslots *slots)
{
vm_memslots s list anyhow.
b!Svm exits ByiyByEGUfDGyEGyAhZ>etUot *GUgnedm, gpa_t gpa, void *data,
			 exits ByiyByEGUfDGyEG
	gfn_t gfn = gpa >> PA_++)
		dyEGUfdyA_user(ZBystruct _dirty0);
		diGUgned longot *mems	
	int offset = offset_ifn_t pfn)
{
	if
			     int offset, int truct kvm_memslots *slots)
{
vm_memslots s list a		return 		diGUgnof
	(addr))
+mloo_pfn_);
		if (	str+mloo_pf > Gwe{
	st host paglofDGT;
	stAWBh
En,
		return __kvm_rr onb!Svm exits ByiyByEGUfDGyEGyAhZ>etUot *GU(addr)), Gwe{
	st host pagnt(len, offset)) n__kvm_rer on
		memset(&new.arch, 0n a writab
		retUBb VBer on
		memsgfn = gpa >> PAGruct kvm	if (ret < 0)
_read_guest_page(vcpu, gfn, data, ofritable = seg);
		if (ret < 0)
			return ret;
		offset = 0;
		len -= seg;
		data +
	strA?adZVEGkinir(ZByEG(ZByEG(AULT;
	returmemory_slot *slot = kvm_vcpu_gfn_to_me_++)
		dyEGUfd
	gfn_t gfn = gpa >> PA_yEGUfdyA_user(ZBystruct _dirty0);
		diGUgned longot *mems;
	int offset = offset_in_page(gpa);

		memsgfn = gpa >> PA_++)
		dyEGUfdyruct kt *Gfset =0ddr = gfn_to_hva_memslot_prot(slot, gfn, NULyEGUfd
	gfn_t gfn BOL_GPL(kvmyEGUfdyA_user(ZBystruct _dirty0);
		diGUgned longot *mems;
	int offset = offset_in_page(gpa);
truct kvm_memslots *slots)
{
vm_memslots s list a		retur);
		if (	str> Gwe{
	st host paglofDGT;
	stAWBh
En,
		return __kvm_rr onb!Svm exits ByiyByEGUfDGyEGyAhZ>etUot *GU(addr)), Gwe{
	st host pagnt(len, offset)) n__kvm_rer on
		memset(&new.arch, 0n a writab
		retUBb VBer on
		memsgfn (!kvm_is_reruct kt ddr)), f (ret < 0)
_read_guest_p
{
	WARN_ON(is_error_page(page)n__kvm_ret < 0)
			return ret;
		offset = (kvm_set_pfn_accessed);

void kvm_get_pfn(kvm_LyEGUfd
	gfn_t gfn ;
	release_page_clean);

void kvm_release_pfnomic(data, (void __user _kvm_read_guuct _, &entr(_kvm_read_gu)pag)) turn gfn_hys(ZERO_pfn_(0))gfn_t gfn)
{
	re_read_guest_atomic(struct kuct _, &ect kvm *kvm, gpa_t gpa, void *data,
			 ;
	release_page_
	gfn_t gfn ;
	release_GE_SHIFT;
	struct kvm_memory_gfn, data, offset, len);
}
EXPORT_SYMBOL_GPL(kvm_vcpu_read_guest_atomic);

static int __kvm_write_guest_page(struct kvm_memory_slot *memslot, gfn_t gfn,
			          const;
	release_page_cc(struct kt r;
	unsigned long addr;

	addr = gfn_to_hva_memslot(memslot, gfn);
	if (a(addr))
		return -EFAULT;
	r = __copy_to_use;
	release_n);
}
EXPORT_SYM;
		len -= seg;
		data +t kvm_vcpu *vcpu, gpa_t gpa, voimemslirtyfn_memslot_atoet_dirty_loitable)
{
	slots;
	}

	r =m instance
 * @log:relVEGw			imemZ2le)
{
	slh_memslotsa_ma in}

_le(relVEGwE) {
		new.dirty_bitmapes)
		re
T_SYM;
		len -= seg;dirty_bitmap;
	unsigned long *dirty_bitmap_buffer;

	as_id = loslot d = log->slot >> 16;
	id = (u16)log0;
		len -= seg;
		data += seg;
		++gfn;AULT;
	r = __copy_to;
		len -= seg;;

	if (kvm_i	int i
		len -= seg;dirty_bitmay_bitmap)
		return -ENOENT;

	n = map_buffer;

	as_id = loslot d = log->ret = kvm_read_guest_page(kvm, gfn, d;
		len -= seg;
		data += seg;
		++gfn;AULT;
	r = __copy_tovm_i	int i
		len -= seg;ast(addr, nr_piga inactivateturn -EFAULT;
	pagefau{
		WARN_Oefau->piga inactivvm_largepagn_read(&cur flagt th a tabllpagem in ory_se_gfn)->t_pl_btable,s) {
		Writeinelse {		npage,= FOLLis func   (w->base_->t_pl_btable,(kvm, lln	  cet_uselse {->t_pl_btable,(don'c   t *is @log:->t_pl_btable,(writawayh a suba iy
 * an)->btable,m_memorypigproce a (SIG_SETMASK, &efau->piga i, &is functit_pl_btable,fn_t gfn,
			   iga indeactivateturn -EFAULT;
	pagefau{
		WARN_Oefau->piga inactivvm_largepagn_repigproce a (SIG_SETMASK, &is functit_pl_btable,v|VEGyEGyEpig

	kv pageis functit_pl_btable,fn_t g}
EXPORT_SYMgrow_hal_paotmapsturn -EFAULT;
	pagefau{
		W offset_ifn_t e_meval,Mgrown_ren + KVval KVvfau->hal_paotmapsurn row KVs);

mslothal_paotmaps_ rowEGyE/* 10usch_mee_fault, val K=his iMgrowm_laval KV10000;ORT_SYMBOval *=Mgrown_relt, val > hal_paotmapsm_laval KVhal_paotmapsur
	vfau->hal_paotmaps KVval;OR_loce!Svm hal_paotmaps_ row(vfau->	int i_meval,M KVM_Mt g}
EXPORT_SYMshrink_hal_paotmapsturn -EFAULT;
	pagefau{
		W offset_ifn_t e_meval,Mshrinkn_ren + KVval KVvfau->hal_paotmapsurnshrink KVs);

mslothal_paotmaps_shrink0)
			reshrink K=

	addval KV0;ORT_SYMBOval /=Mshrinkn_revfau->hal_paotmaps KVval;OR_loce!Svm hal_paotmaps_shrink(vfau->	int i_meval,M KVM_Mt g}
EXPORt(vcpu, gfn);_MIXEDbtablturn -EFAULT;
	pagefau{
		WARN_AULTack oT_SYMBun}
EXPOefau{=m insmap_bake_ynct kv(Ksync Q_UNHALT,Vvfaun_t gfn)
{
	>dirTR;_hwpoison_use;SYMhas_pe;

st_[mslrOefau{=t gfn)
{
	>dirTR;_h		resfsepl_pe;

st * Whoev{=t gfn)
{
	>dirTR;_*writable = true;
				   vvm,
writexecuv maa HLTut_frn -Ensignet	Wrn-)
			cem islotuse tuser_pif (kvm_i	int btablturn -EFAULT;
	pagefau{
		Wk[msl longotZ */
s;_hDECLAR(kvg gfQUEUE(nlikn_t eturn a + 1pfn_t pfn =ts;
btablapsur
	ngotZpfn/
sg->r[msl gpag0)
			revfau->hal_paotmaps=m insm[msl longopg->r[msl addapstr[msl gpag0,Vvfau->hal_paotmaps_error++efau->p
EX.hal_patt

	kn KVotmif (ko @pfn,ata,
&cur flag pas Ksync Q_UNHALT_set_m  pfnsrupta,
&curarrivv)
		 indicatoison_usegfn);_MIXEDbtabltefau{r;

	 @pfn,r++efau->p
EX.hal_pt *slotfulKVotmif (	WARN_Oefaumaps m_wakeupOefau{=t gn,r++efau->p
EX.hal_paotmae == KVif (	Wmory_slot old, ne	/
sg->r[msl gpag0)
		} truct ks
stll l a MBun}
st ed lor[msl  && wr * W,ongop
	if (!(nAULTack oT_SYMbtabl
st vfaun_tm for n;;get_pfn_kvm_med_gs long&efau->wq, & lon, TASK_irTERRUPTIBLE_error_hva_usegfn);_MIXEDbtabltefau{r;

	 ne	bt_pte to  a + 1pfnhether tsGUfduXPORT_S(!(neini			r long&efau->wq, & lonRT_S/
sg->r[msl gpag0)
(nAULTack oT_SYMunbtabl
st vfaun_tirty =btablapsg->r[msl d_gpst/
s)mZ2r[msl d_gpstngotZ host pagOefaumaps m_wakeupOefau{=t gshrink_hal_paotmapstvfaun_t _range behal_paotmaps=m ins_regitablapsg<KVvfau->hal_paotmaps	 ne	; ne/* we
wrd a taog:itabl,Mshrink aotm
		/dicat_range beffau->hal_paotmaps  loitablapsg> hal_paotmapsm_lagshrink_hal_paotmapstvfaun_t e/* we
wrd a short hal_(kvm,er_paotm [msl ((choo sm llndicat_range beffau->hal_paotmaps < hal_paotmaps  l ne	btablapsg< hal_paotmapsm_laggrow_hal_paotmapstvfaun_t rUgnedMBOvfau->hal_paotmaps KVm *kv_loce!Svm r(ZVEGakeupObtablaps,n a + 1,Vvfaumaps m_wakeupOefau{=;(nAULTack oT_SYMbtabl_eini		tvfaun_tAULT;
	r = __copy_tovm_i	int bf needelong n;
	ur(ZVEGake_upOurn -EFAULT;
	pagefau{
		Wurn -EFr lon_ct ue_hFOLL*wqpge(stqp slot(slot, r(ZVEGqtvfaun_t 		reswqMhas_vm_arlrOtqpo_hva_maGake_upOtqpo;ror++efau->p
EX.hal_pwakeup_t gfn)
{
	;

	return hva;
}

unsignedLT;
	r = __copy_to_user(ZVEGake_upedel#ifnll_new_memsS390ue;
			Kick a sm_ar
		/kvm,,_WRIT_page(pkvm,
ut_page(pm is (voio *asyn)
			cem isuser_pif (kvm_i	int kiblturn -EFAULT;
	pagefau{
		WAnonize_guest
	paKVvfau->fauhost pagnt(lr(ZVEGake_upOefau{=t gfn)
{
slot dpage, Nfaug0)
			re
	pa!;

	st pin ffset_)
	pato_pfint i_s  loint o nr_p(fau{=t gARN_AULTack oT_SYMflags  kibltefau{=t gnsm to nd		reGUfduXPOfaun_t , gffaug0)
edLT;
	r = __copy_to_user(ZVEkibl)if needeuct !ew_memsS390er_pot(vcpu, gfn);yielslotturn -EFAULT;
	pagtshottool *writabled_gupKVif writabll a M	r = fixtirty
 ck);

	uest_pa KVm *kvrcYMBOL_Gtabltn_t ,vm_dircYMcetm->mmap_(tshott->p
		slots =poto outirty
 e *gfmslo a (pi_mePIDTYPE_PIDst ancYMBOL_Git and EGyEGyEGyo a =t gfn)
{
	to_hva_pa KVyielslotto a ,n_ran , gfl a M	r = fto a gfn_t gfn)
{to_hvedLT;
	r = __copy_to_user(ZVEyielslotgfn_e;
			Help;
	ifas, trues &pfn);
	apkvm,
uitelig	rc =for dirn gfnVyiels	}

	Masynelig	rc =ckvmd.baTS)
	yiels
	retucidunsignset_memoryheurst pcson(kvm, m(a)pkvm,
 {
		Wwrit adds *slpl-sync_WRI
	parelax  pfnscepv map cion *vm, m(n_k

	kn  tabl *alcet),_memory_snsign@		dad if aop *kvm,SVM_Wm_(mslbeig	n}
stlock);
	re maam_(mslotd an) pfnscepvnsi/PLEost virt.n(kvm, m(b)pkvm,
 {
		Wwrits *slpl-sync/I
	parelax  pfnscepv maFOLLmd.t adde *vm, m->bace l is [msl (masy rettWwritbecosl elig	rc =now s
st *me
wrvkvm_sbuse*vm, myiels	if (utabl*alcet
ut_l is  + _kvm_r.t flags =s *sligneoggl	struct @dy_elig	rc =ener [msl apkvm,
 truet i, aselig	rility.)n(kvm, mYiels ^ old.flp cion *lpl-synced/
	parelax  pfnscepv makvm,
 && wriyiels ^ vm, mld.n_k

	kn  tabl-*alcet
cags &	re_pfni stro		/kvm,g pl
	 *
	lock)vm,vm, mbn)
 ^ . Givstrucriorityto_hva_potottial tabl-*alcet

strmorys tablvm, mm_sg pag.n(kvm, mS
st *algorithmgs =h_med);
	heurst pcs, BAD_PA
stlocgfn),/kvm,gkvm_inet	irtvm, mtabl
stgt th  addharm. IER_MEM	re_pfni stry ^ old.yiels
ld. sasl kvm,,_urn vm, mock);s_dirtygnet	W id;
kvm,gis_diy_snSYMBOL_GPL(gfn_to__user(ZVEelig	rc _o_h_dirn gfnEyielsturn -EFAULT;
	pagefau{
		stall_new_memsHAVEslotsvm,nc LAX_irTERCEPTt eturnelig	rc fn_telig	rc == Oefau->pd if aop 		dad if aopeded lirtyefau->pd if aop dy_elig	rc n_relt, vfau->pd if aop 		dad if aop)insmap_T_SYMf indy_elig	rc ge(kvm,Oefau->pd if aop dy_elig	rc gfn_t gfn)
{elig	rc n_#gnedMBfn)
{
	;

	re neededtrue;
			U a wr AULTack oT_SYMBun}
EXP,ned long addr, siz
	/* mairtsidu	sizeVvfaumload/vfaum, glocke.  Howges;,to_hvmasy ack irn gtrucommit_meack oT_SYMBun}
EXPgt th  addynctireavfaumloadSYMBOLeturn__w_ptit_meack ody_Bun}
EXPOurn -EFAULT;
	pagefau{
		W gfn)
{
	reack oT_SYMBun}
EXPOefau{sync,
			    bool _SYMdy_Bun}
EXPOurn -EFAULT;
	pagefau{
		WARN_AULTack ody_Bun}
EXPOefau{=t gfn)
{
		down_rstall_new_memslotsASYNC_PFyEGyEGylst _

	kv_  t fulg&efau-> or a_pf.s *s{=t gfn)
{
		down_ needed. hva;
}

unsigned if (kvm_i	int o	dad iOurn -EFAULT;
	pagme	returnyielslot])
			com isOENT;

	n = mapp;
	u ;

	->
	u;T;

	n = map_;
	pagefau

	uestl is_etukv m_;
	pa;

	->
	u->l is_etukv m_;
	p

	uestyiels	ift, currentaddr, 3currentpasscurrenti)
(nAULTT_SYMf in		dad if aop(me	r_SYMBOLead(&curWe etukvt kvm rioritytan)a*kvm,
		aed byBun}
EXPgFOLL ad];

		0;
		new.Bun}
st,
		npagesttWgaddn_k

	kn  ignsosl);
	}];

	_rangock);	/* maeGUfduXPni s_oT_SYMBun.  Hop fullth kvm];

	kvm,
uit*alcNVAL;

	tabl  kvm_m_set_mee[0]milln	 emorystt.(&curWe apm_sx;
		u round-_sbi signsgotZ
stlovt kvml is etukv m	kvm,m_memoryfor npassft, ctpassato2
statyiels	ifstauesttpass++=m insmap_o_h_eneroT_SY(i,Vvfaupu, g	 @pfn,GyEGypassalots;
=tl is_etukv m_;
	p	 @pfn,rext l is_etukv m_;
	p

	* @is_dirty:	old, _range bepassalots;>tl is_etukv m_;
	p	
	* @bt_pte fn,GyEGyACC0; !mslotefau->n_k

	kn {=t gn,is_dirty:	oldlt, vfau K=
me=t gn,is_dirty:	oldlt, r lon_activvg&efau->wq

stat _SYMdy_Bun}
EXPOefau{=t gn,is_dirty:	oldlt, yielslot])
			com is
static sack oT_SYM if)
			cOefau{=t gn,is_dirty:	oldlt, !_user(ZVEelig	rc _o_h_dirn gfnEyielstefau{=t gn,is_dirty:	t gnyiels	ift,pu, gfn);yielslottvfaun_t edlt, yiels	if>

	 @pfn,r
	u->l is_etukv m_;
	pft,i;
	* @bt_pte fn,, _range beyiels	if;

	 @pfn,rues--if (	WARN_O;
}
t gn,rbt_pte fn,,
pshot onAULTT_SYMf in		dad if aop(me	r;
	if (!vmct E	struVvfaun false;elig	rc =durs, asid;
ad i aopemorymap_T_SYMf indy_elig	rc gme	r;
	if (!edLT;
	r = __copy_to_user(ZVEo	dad imssWBh
EGyEGyEG_user(ZVEis detcked = fal*p_pfn*vmfOENT;

	n = map_;
	pagefauaKVvmfe thre the ile->n_ivate_kvm_; *writable)
{
	struange is alfe pgoff K=

	add, &entr
	}

	return efau->Bun)if tall_new_memsX86ng_range befofe pgoff K=
_t hvIO_pfn__OFFSET	add, &entr
	}

	return efau->ack .pio_kvm_)if neederstall_new_memslotsMMIOng_range befofe pgoff K=
_t hCOALESCEDsMMIO_pfn__OFFSET	add, &entr
	}

	return efau->
	u->coalesc m_mmio_rs, )if neederRT_SYMBOL_GPL(gic sack oT_SYMis detvfaupuvmfOurn D_PAGE;
	pages)
fofe p &entrytruanwritable = truBh
EGyE_kvm_rcked = falop _kvm_rs_

	n = map_;
	p_falopsft,NT;.*p_pfn)
_user(ZVEis de,
}ssWBh
EGyEGyEG_user(ZVEeed tcked =  ile * ile,rcked = false;
		r = fixup_OENT;thre theopsft,&map_;
	p_falopsanwritable = truBh
EGyEoid *data, unsiemorytcked = in is
*in is,rcked =  ile * ilpOENT;

	n = map_;
	pagefauaKV ilp->n_ivate_kvm_; 
	debugf
		rmove_yn	0;sivvgefau->debugf
	de);
}
EXPmap_, gfmap efau->
	ufn;
	}
	return 0;vm, gfn), gfn, ilelop _kvm_rs
_user(ZVEiopsft,NT;.	 emoryssssssss= *data, unsiemory,T;.ritable,_ioctls= *data, unioctl,rstall_new_memslotsCOMPATt .!(vmat_ioctlsss= *data, un!(vmat_ioctl,f neederR.eed ta, (void _=G_user(ZVEeed ,T;.l_SYek		=alsop_l_SYek,
}ssWe;
			Altaby_slo_m  p is
, as_id,r(ZV_to_hva_memslot_ptrmote_r(ZVEisturn -EFAULT;
	pagefau{
		
	}
	retocgn_ p is gpaist"
	u-efau",,&map_;
	p_iops,VvfaupuO_RDWR |uO_CLOEXECM_Mt g}
EXPORt(vcpu, trmote_r(ZVEdebugf
turn -EFAULT;
	pagefau{
		
char dir_nasl[ITOA_MAX_LEN     kvm_mem_page(slt, !_useack ohas_r(ZVEdebugf
t{=t gfn)
{
	0ge(slt, !debugf
	GyEGializedt{=t gfn)
{
	0ge(ssnn_intf(dir_nasl,@kvm:	podir_nasl), "r(ZV%d",Vvfau->a, uni		sloefau->debugf
	de);
}_=Gdebugf
	trmote_dir(dir_nasl,
	MEMSLBOvfau->
	u->debugf
	de);
}
EXPARN_Oefau->debugf
	de);
}
t gfn)
{
	>dNOMEMfn_t gf slot(slot, trmote_r(ZVEdebugf
tvfaun_t 		readdr;

	t(slotebugf
		rmove_yn	0;sivvgefau->debugf
	de);
}
EXPgfn)
{
	to_hvaurn hva;
}
 = true;
			Crmotesnsosl;
	}
	*pffaus.  Good lubl trmots, am writaem);
e_to_hva_memslot_pAULT;m_ioctl trmote_r(ZVdirty_bitmap;
	unsu32 s not a		return

	n = map_;
	pagefau

t 		reDDRESS_SPAMAX_kvm,_urn 0;
}
EXPO>dirty_bitmmutexf any pages
f needed.
 *
	u->crmotem_;
	ps K=
_t hMAX_kvm,S	t(slomutexfit and clear f needed;
}
EXPO>dirty_bi (!(nAUL->crmotem_;
	ps++;
omutexfit and clear f needeloefau slot(slot, r(ZVEcrmoted = (u
		slots =ISurn Oefau{=m inssg->_gfnrn Oefau{ded;mory_r(ZVEdecrm*membi (!(nn_k

	k__fast(adDGyEGy&efau->n_k

	k__fast(ad,,&map_n_k

	k_ops0)
_read_AULTack oT_SYMf}
Eptvfaun_t 		rea)ed;mory_r(ZVEde

	oy)
_read_AULTtrmote_r(ZVEdebugf
tvfaun_t 		rea)ed;mory_r(ZVEde

	oy)
_rmutexf any pages
f needed.
 *
	uge *gr(ZVEbyni	d = (u
		=m inssg->-EEXISTded;mory_it and_r(ZVEde

	oy)
 (!(n
		if (ages
;
	ps[_read_ther thages
o nr_p_;
	ps)] (!vmct Nowstt's, llnslotupkvm,otuserspacen	  		Wrtemorymap_e *gmap 
	ufn;
	pfn/rmote_r(ZVEistvfaun_t 		rear;

	t(slomap_, gfmap 
	ufn;
;mory_it and_r(ZVEde

	oy)
 (!(nages
;
	ps[_read_ther thages
o nr_p_;
	ps)]aKVvfaun_read(&curPairsgnet	Wsm trmb()ni s
	uge *gr(ZV.  Wis fuages
;
	pslse {		& wriages
o nr_p_;
	p's,
strm*mem	ifvaluem_memorypm twmb())
 _read_t
stthages
o nr_p_;
	ps))
_rmutexfit and clear f neededAULTack oT_SYMpukvcrmotedvfaun_t 
		*writablit and_r(ZVEde

	oy:_rmutexfit and clear f neededtebugf
		rmove_yn	0;sivvgefau->debugf
	de);
}
EXr(ZVEde

	oy:_rAULTack oT_SYMde

	oydvfaun_tr(ZVEdecrm*mem:_rmutexf any pages
f neededAUL->crmotem_;
	ps--if mutexfit and clear f needed
		*writabtruBh
EGyEoid *data, unioctl f insige a (irty_bitmay_bitmap)
		rpiga in;
	piga i{
		WARN_piga i{hva_maigdT_SYte a (iiga i, sige a (SIGKILL)|sige a (SIGSTOP{=;(nOvfau->piga inactivv to_pfnOvfau->piga i to	piga i_t rUgnedMBOvfau->piga inactivv to0n;
	}
	return 0;vm, gfn      gfn_t *nioctltcked =  ile * ilp*mems;
	 offset_ifn_tioctl,write_fault || wrgOENT;

	n = map_;
	pagefauaKV ilp->n_ivate_kvm_; 	ror_page(pagewrgp sl, gfn, data, offruest_atoturn

	n = map_f	pagfauaKVck);

	

	n = map_sregsULL, tsregsUKVck);

relt, vfau->lear mma!;
is functionn 0;
}
EXPO>diO.arch, 0n a writa_IOC_TYPE(ioctlfn,
	_t IO{=t gfn)
{
	>dirty_bitstaGdeeined(ew_memsS390)    deeined(ew_memsPPC)    deeined(ew_memsMIPS)read(&curS gfn*pffite_fVvfaun octls  kvm_ t *is : 0),
			 ry_r(ZVtexecuvm_r1];

	soavfaumload()nwags &bt_ptstt.(&cufault, ioctls==
_t hS390_irTERRUPT    ioctls==
_t hS390_iRQ    ioctls==
_t hirTERRUPT)MBOL_GPL(gic sack oT_SYMioctlt ilp*tioctl,wwrgOn_ needed.;
	pfnvfaumload(vfaun_t 		rea)ed;
		*writab	snet		W(ioctlfn		
corysKsyncUN:hva_many(slotd_guoldpKVif ssg->-Eirty_bi t pagergOE(	Wmory_slot ololdpKV_dircYMBAD_PAMpu pfnstefau->ni   bool *writ writaoldpKV_!;
is functipKVs[PIDTYPE_PID].p
		=m insmct 	   th FOLLBun}
stned lokvm,
 tbasedslot;
	many(slotd_gunewpKV_die *gl a MpKVtected.
 *PIDTYPE_PIDst addr cYMBste_fMpu pfnstefau->ni , newpKVn_t edlt, oldpKV
t gn,s : 0),
vm:_ cYg0)
		m, gfnKVtoldpKV
;
pshotread_AULTack oT_SYMioctl Buntvfaupuvfau->Bun)if	v_loce!Svm userspace_sync efau->Bun->syncthers_r1 r)if	vbt_pte f}	
corysKsynGET_REGS:hva_many(slok_get_gsULL, tt_gst addsg->-ENOMEMfn		k_get_gsUd_Azaticc(kvm:	poany(slok_get_gs), GFP_KERNEL  bool *w!k_get_gs)E(	Wmory_slot olead_AULTack oT_SYMioctl e *gt_gs(vfaupu, get_gs) bool *wr)E(	Wmory_slo_page1if ssg->-Efset = 0_		re
st_page(vcpuwrgppu, get_gs,@kvm:	poany(slok_get_gs)))E(	Wmory_slo_page1if ssg->0_tirt_page1:n		kpage(, get_gs) boobt_pte f}	
corysKsynSET_REGS:hva_many(slok_get_gsULL, tt_gst addsg->-ENOMEMfn		k_get_gsUd_memdf erroruwrgppukvm:	po*k_get_gs))= 0_		reISurn Ok_get_gs))m insmsg->_gfnrn O, get_gs) booWmory_slot olhotread_AULTack oT_SYMioctl s *gt_gs(vfaupu, get_gs) bookpage(, get_gs) boobt_pte f}	
corysKsynGET_SREGS:hva_mL, tsregsUKVAzaticc(kvm:	poany(slok_gest_gs), GFP_KERNEL  boosg->-ENOMEMfn		l *w!k_gest_gs)E(	Wmory_slot olead_AULTack oT_SYMioctl e *gst_gs(vfaupu, gest_gs) bool *wr)E(	Wmory_sloif ssg->-Efset = 0_		re
st_page(vcpuwrgppu, gest_gs,@kvm:	poany(slok_gest_gs)))E(	Wmory_sloif ssg->0_toobt_pte f}	
corysKsynSET_SREGS:hva_mL, tsregsUKVmemdf erroruwrgppukvm:	po*k_gest_gs))= 0_		reISurn Ok_gest_gs))m insmsg->_gfnrn O, gest_gs) boomL, tsregsUKVck);

	oWmory_slot olhotread_AULTack oT_SYMioctl s *gst_gs(vfaupu, gest_gs) boobt_pte f}	
corysKsynGET_MP_STATE:hva_many(slok_gem tot		u m tot		ut addsg->AULTack oT_SYMioctl e *gmpot		u(vfaupu&m tot		u) bool *wr)E(	Wmory_sloif ssg->-Efset = 0_		re
st_page(vcpuwrgppu&m tot		upukvm:	pom tot		u)))E(	Wmory_sloif ssg->0_toobt_pte f}	
corysKsynSET_MP_STATE:hva_many(slok_gem tot		u m tot		ut addsg->-Efset = 0_		re
st_p
{
	WARN_O&m tot		upuwrgppukvm:	pom tot		u)))E(	Wmory_sloif ssg->AULTack oT_SYMioctl s *gmpot		u(vfaupu&m tot		u) boobt_pte f}	
corysKsynTRANSLATE:hva_many(slok_ge_lonsly_se_gtrt addsg->-Efset = 0_		re
st_p
{
	WARN_O&trpuwrgppukvm:	potr)))E(	Wmory_sloif ssg->AULTack oT_SYMioctl _lonsly_u(vfaupu&tr) bool *wr)E(	Wmory_sloif ssg->-Efset = 0_		re
st_page(vcpuwrgppu&trpukvm:	potr)))E(	Wmory_sloif ssg->0_toobt_pte f}	
corysKsynSET_GUEST_DEBUG:hva_many(slok_geease_pdebug dbgt addsg->-Efset = 0_		re
st_p
{
	WARN_O&dbgpuwrgppukvm:	podbg)))E(	Wmory_sloif ssg->AULTack oT_SYMioctl s *gease_pdebug(vfaupu&dbg)_toobt_pte f}	
corysKsynSET_SIGNAL_MASK:hva_many(slok_gesfsepl_e a , data, osige a Tacmemowrgp;a_many(slok_gesfsepl_e a ,k_gesfse a ;a_maiga in;
iiga i, *pge(s	pUKVck);

	o pagergp)m insmsg->offset = 0_m		re
st_p
{
	WARN_O&k_gesfse a puwrgpp
MEMSLOT kvm:	pok_gesfse a )))E(	WWmory_slot oldsg->-Eirty_bi td.
 *
	ugsfse a .t, g!;
kvm:	poaiga i{)E(	WWmory_slot oldsg->-Efset = 0_m		re
st_p
{
	WARN_O&iiga i, sige a Tacm->piga i,
MEMSLOT kvm:	poaiga i{))E(	WWmory_slot oldpUKV&piga i_t lhotread_AULTa, unioctl f insige a (vfaupup) boobt_pte f}	
corysKsynGET_FPU:hva_mfauaKVAzaticc(kvm:	poany(slok_gefau), GFP_KERNEL  boosg->-ENOMEMfn		l *w!f	p	
	* mory_slot olead_AULTack oT_SYMioctl e *gf	p(vfaupufau{ded;l *wr)E(	Wmory_sloif ssg->-Efset = 0_		re
st_page(vcpuwrgppuf
		rpim:	poany(slok_gefau)))E(	Wmory_sloif ssg->0_toobt_pte f}	
corysKsynSET_FPU:hva_mfauaKVmemdf erroruwrgppukvm:	po*fau{=;(n_		reISurn Ofau{=m insmsg->_gfnrn Ofau{ded;	fauaKVck);

	oWmory_slot olhotread_AULTack oT_SYMioctl s *gf	p(vfaupufau{ded;bt_pte f}	
d, len):otread_AULTack oT_SYMioctlt ilp*tioctl,wwrgOn_f}	irty =T_SYMput(vfaun_t kpage(faun_t kpage(, gest_gs) bo
		*writabtrustall_new_memslotsCOMPATtvm, gfn      gfn_t *n!(vmat_ioctltcked =  ile * ilp*mems	
	 offset_ifn_tioctl,write_fault || wrgOENT;

	n = map_;
	pagefauaKV ilp->n_ivate_kvm_; 	ror_page(pagewrgp sl!(vmat_ptruwrg)est_atoturrelt, vfau->lear mma!;
is functionn 0;
}
EXPO>diO.arcsnet		W(ioctlfn		
corysKsynSET_SIGNAL_MASK:hva_many(slok_gesfsepl_e a , data, osige a Tacmemowrgp;a_many(slok_gesfsepl_e a ,k_gesfse a ;a_m!(vmat_aiga in;
cpiga i_t laiga in;
iiga ierror_hvaergp)m insmsg->offset = 0_m		re
st_p
{
	WARN_O&k_gesfse a puwrgpp
MEMSLOT kvm:	pok_gesfse a )))E(	WWmory_slot oldsg->-Eirty_bi td.
 *
	ugsfse a .t, g!;
kvm:	pocaiga i{)E(	WWmory_slot oldsg->-Efset = 0_m		re
st_p
{
	WARN_O&ciiga i, sige a Tacm->piga i,
MEMSLOT kvm:	pocaiga i{))E(	WWmory_slot oldaiga in
{
	W!(vmatO&iiga i, &ciiga i)t oldsg->AULTa, unioctl f insige a (vfaupu&iiga i)t olrUgnedMBOdsg->AULTa, unioctl f insige a (vfaupuVEGyEGyE;bt_pte f}	
d, len):otread_AULTT_SYMioctlt ilp*tioctl,wwrgOn_f}		irty =
		*writabtr needed.Bh
EGyEoid *datge *kvmioctl attruany(slok_gege *kv *ge *mems	
oid (*BAD_PAor)uany(slok_gege *kv *ge *mems	s	
any(slok_gege *kv attrgewttr)*mems	
rite_fault || wrgOENT;

	n = map_ge *kv attrgattrge(slt, !BAD_PAor) 0;
}
EXPO>dPERMge(slt, 
st_p
{
	WARN_O&attrvm_read_guest_pagergpukvm:	powttr)er on
		memset(&new.arc	}
	retoAD_PAor(ge * &attr)rn 0;vm, gfn      gfnge *kvmioctltcked =  ile * ilp*	 offset_ifn_tioctl,
	msrty0)rite_fault || wrgOENT;

	n = map_ge *kv *ge aKV ilp->n_ivate_kvm_; 
	lt, ge ->lear mma!;
is functionn 0;
}
EXPO>diO.arcsnet		W(ioctlfn		
corysKsynSET_DEVICE_ATTR: 0;
}
EXPO*datge *kvmioctl attruge * ge ->ops->pe_pattr,wwrgOn_fcorysKsynGET_DEVICE_ATTR: 0;
}
EXPO*datge *kvmioctl attruge * ge ->ops->ge_pattr,wwrgOn_fcorysKsynHAS_DEVICE_ATTR: 0;
}
EXPO*datge *kvmioctl attruge * ge ->ops->has_attr,wwrgOn_fd, len):otrlt, ge ->ops->ioctlfMBOds}
EXPOge ->ops->ioctluge * ioctl,wwrgOn_ 0;
}
EXPO>dNOTTY;)
		re
Bh
EGyEoid *datge *kvmsiemorytcked = in is
*in is,rcked =  ile * ilpOENT;

	n = map_ge *kv *ge aKV ilp->n_ivate_kvm_; ;

	n = mapp;
	u ;
ge ->lea)
(nAULT, gfmap 
	ufn;
ritable = truBh
EGyE_kvm_rcked =  ilelop _kvm_rs
_usege *kvmiopsft,NT;.ritable,_ioctls= *datge *kvmioctl,rstall_new_memslotsCOMPATt .!(vmat_ioctls= *datge *kvmioctl,rsneederR.	 emorys= *datge *kvmsiemory,
}ssWBh	n = map_ge *kv *_usege *kvmi{
	W ilptcked =  ile * ilp{
		WARN_ ilp->flopa!;
&_usege *kvmiopsn 0;
}
EXPOck);

rehva;
}

ilp->n_ivate_kvm_;  0;vm, gfn), gfn,_usege *kvmopsf*_usege *kvmops_		ret[lotsDEV_TYPEhMAX]ft,NTstall_new_memslotsMPIC
	[lotsDEV_TYPEhFSLsMPIC_20]	;
&_usempicmops,
	[lotsDEV_TYPEhFSLsMPIC_42]	;
&_usempicmops,
 neededt	gfn_t gfn BOgisterege *kvmopsuany(slok_gege *kvmopsf*ops,Vu32 type{
		WARN_typeRESSARRAYage(p(_usege *kvmops_		reter on
		memsetNOSPChost pagnt(lge *kvmops_		ret[type]a!;
ing to c
		memsetEXISTde(nAULTge *kvmops_		ret[type]a= opsanwritable = truif (kvm_iunBOgisterege *kvmopsuu32 type{
		WARN_nt(lge *kvmops_		ret[type]a!;
ing to cAULTge *kvmops_		ret[type]a= ck);

re
Bh
EGyEoid *datioctl trmote_ge *kvdirty_bitmap;
	un
EMSLOT kny(slok_getrmote_ge *kv *cdOENT;

	n = map_ge *kv opsf*opsaKVck);

	

	n = map_ge *kv *ge ;t eturntge(p;
id->flags &
_t hCREATE_DEVICE_TEScpu_readtypeest_atotpage(slt, id->typeRESSARRAYage(p(_usege *kvmops_		reter on
		memsetNODEV *kv_ypeRmowrra;
		dexfn_apec id->type,SARRAYage(p(_usege *kvmops_		reterestopsaKVAULTge *kvmops_		ret[type];(slt, opsaK;
ing to c
		memsetNODEV *kvARN_tse_no c
		mems0; 
	devaKVAzaticc(kvm:	po*ge ), GFP_KERNEL  bolt, !dev
t gfn)
{
	>dNOMEMfn_tge ->opsa= opsanwge ->leaaKVAUL)
_rmutexf any pages
f needed gf slops->crmotedge * type{;t 		readdr;

	t(slomutexfit and clear f needed;kpage(dev
;t gfn)
{
	to_hvaur	lst _add(&ge ->ULTn is,rclear ge *kvs) bomutexfit and clear f needelolt, ops->iyEG
t gops->iyEG(dev
;trymap_e *gmap 
	ufn;
	gf slocgn_ p is gpaistops->nasl,@&_usege *kvmiops* ge puO_RDWR |uO_CLOEXECM_M 		readdr;

	t(sloAULT, gfmap 
	ufn;
rmutexf any pages
f needed	lst _del(&ge ->ULTn isfn;
rmutexfit and clear f needed;ops->de

	oyddev
;t gfn)
{
	to_hvaur_fcd->fV_diro_hva_pa	return 0;vm, gfn      gfn_atioctl tMIXEDexteitegn_rn __struct kvm_vcpu *vcpt || wrgOENT;
net		W(wrgOn		
corysKsynCAP_USER_MEMORY:	
corysKsynCAP_DEScROY_MEMORY_REGION_WORKS:	
corysKsynCAP_JOIN_MEMORY_REGIONS_WORKS:	
corysKsynCAP_irTERNAL_ERROR_DATA:	stall_new_memsHAVEslotsMSI	
corysKsynCAP_SIGNAL_MSI:f neederstall_new_memsHAVEslotsIRQFD	
corysKsynCAP_iRQFD:	
corysKsynCAP_iRQFD_RESAMPLE:f needer
corysKsynCAP_iOEVENTFD_ANY_LENGTH:	
corysKsynCAP_CHECK_EXTENSION_VM: 0;
}
EXPO1if tall_new_memslotsMMIOngcorysKsynCAP_COALESCEDsMMIO: 0;
}
EXPO_t hCOALESCEDsMMIO_pfn__OFFSETif neederstall_new_memsHAVEslotsIRQ_ROUTING	
corysKsynCAP_iRQ_ROUTING: 0;
}
EXPO_t hMAX_iRQ_ROUTESif neederstaO_t hADDR0; !SPACE_NUM > 1	
corysKsynCAP_MnewIhADDR0; !SPACE: 0;
}
EXPO_t hADDR0; !SPACE_NUMif neederRd, len):otrbt_pte f}	

}
EXPO*dat_atioctl tMIXEDexteitegnd = (uwrgOn_ 0;vm, gfn      gfn_atioctltcked =  ile * ilp*mems;
	 offset_ifn_tioctl,write_fault || wrgOENT;

	n = mapp;
	u ;
 ilp->n_ivate_kvm_; 	ror_page(pagewrgp sl, gfn, data, offruest_atotured.
 *
	u->mma!;
is functionn 0;
}
EXPO>diO.acsnet		W(ioctlfn		
corysKsynCREATE_kvm,:otread_AULTTm_ioctl trmote_r(ZVd = (uwrgOn_trbt_pte fcorysKsynSET_USER_MEMORY_REGION:hva_many(slok_geuserspace_buffer;p stallk_geuserspace_buft addsg->-Efset = 0_		re
st_p
{
	WARN_O&k_geuserspace_bufpuwrgpp
MEMSL	kvm:	pok_geuserspace_buf)))E(	Wmory_sloifotread_AULTTm_ioctl s *gmuffer;p stald = (u&k_geuserspace_buf) boobt_pte f}	
corysKsynGET_DIRTY_LOG:hva_many(slok_ge seg;
loglt gt addsg->-Efset = 0_		re
st_p
{
	WARN_O&t gpuwrgppukvm:	pot g)))E(	Wmory_sloif ssg->AULTTm_ioctl e *g seg;
logd = (u&t g) boobt_pte f}	 tall_new_memslotsMMIOngcorysKsynREGISTER_COALESCEDsMMIO:hva_many(slok_gecoalesc m_mmio_ct p ct pt addsg->-Efset = 0_		re
st_p
{
	WARN_O&ct ppuwrgppukvm:	poz *s{=)E(	Wmory_sloif ssg->AULTTm_ioctl BOgisterecoalesc m_mmiod = (u&z *s{ boobt_pte f}	
corysKsynUNREGISTER_COALESCEDsMMIO:hva_many(slok_gecoalesc m_mmio_ct p ct pt addsg->-Efset = 0_		re
st_p
{
	WARN_O&ct ppuwrgppukvm:	poz *s{=)E(	Wmory_sloif ssg->AULTTm_ioctl unBOgisterecoalesc m_mmiod = (u&z *s{ boobt_pte f}	 needer
corysKsyniRQFD:hva_many(slok_geirqfV_kvm_; 
	dsg->-Efset = 0_		re
st_p
{
	WARN_O&d(is_ewrgppukvm:	podvm_)=)E(	Wmory_sloif ssg->AULTirqfVd = (u&kvm_)ifoobt_pte f}	
corysKsyniOEVENTFD:hva_many(slok_geioevuncfV_kvm_; 
	dsg->-Efset = 0_		re
st_p
{
	WARN_O&d(is_ewrgppukvm:	podvm_)=)E(	Wmory_sloif ssg->AULTioevuncfVd = (u&kvm_)ifoobt_pte f}	stall_new_memsHAVEslotsMSI	
corysKsynSIGNAL_MSI:hva_many(slok_gemsi msit addsg->-Efset = 0_		re
st_p
{
	WARN_O&msipuwrgppukvm:	pomsi)=)E(	Wmory_sloif ssg->AULTo nd	userspace_bsid = (u&msi) boobt_pte f}	 neederstall_n__KsynHAVEsiRQ_LINEr
corysKsyniRQ_LINE_STATUS:	
corysKsyniRQ_LINE:hva_many(slok_geirq_levul irq_evunct addsg->-Efset = 0_		re
st_p
{
	WARN_O&irq_evuncpuwrgppukvm:	poirq_evunc)))E(	Wmory_sloifotread_AULTTm_ioctl irq_lr_p( = (u&irq_evuncp
MEMSLioctls==
_t hiRQ_LINE_STATUS{ded;l *wr)E(	Wmory_sloifaddsg->-Efset = 0_		reioctls==
_t hiRQ_LINE_STATUS{ @pfn,GyEG
st_page(vcpuwrgppu&irq_evuncpukvm:	poirq_evunc)))E(	WWmory_slot olho
 ssg->0_toobt_pte f}	 neederstall_new_memsHAVEslotsIRQ_ROUTING	
corysKsynSET_GSI_ROUTING:hva_many(slok_geirq_rslo
stnrslo
st;a_many(slok_geirq_rslo
stn data, ourslo
st;a_many(slok_geirq_rslo
st_e);
}_*e);
iesUKVck);

redsg->-Efset = 0_		re
st_p
{
	WARN_O&rslo
stpuwrgppukvm:	porslo
st)=)E(	Wmory_sloif ssg->-Eirty_bi t pag!_useack ocanMf in	rq_rslo
st 
	uf)E(	Wmory_sloif sl *wrslo
st.nr > _t hMAX_iRQ_ROUTES)E(	Wmory_sloif sl *wrslo
st.flags)E(	Wmory_sloif sl *wrslo
st.nr)m insmsg->ofNOMEMfn			e);
iesUKVvmaticc(rslo
st.nr *ukvm:	po*e);
ies)n_t edlt, !e);
ies)E(	WWmory_slot oldsg->-Efset = 0_murslo
stemowrgp;a_m_		re
st_p
{
	WARN_Oe);
ies,wrrslo
st->e);
ies,
MEMSLOT rslo
st.nr *ukvm:	po*e);
ies)n)E(	WWmory_slo_pagen	rq_rslo
stt olho ssg->AULTo in	rq_rslo
st 
	u, e);
ies,wrslo
st.nr,
MEMSLrslo
st.flags)_tirt_pagen	rq_rslo
st:otrvpage(e);
ies)_toobt_pte f}	 needeuct ew_memsHAVEslotsIRQ_ROUTINGlot;
corysKsynCREATE_DEVICE:hva_many(slok_gecrmote_ge *kv cd

redsg->-Efset = 0_		re
st_p
{
	WARN_O&cdpuwrgppukvm:	pocd)))E(	Wmory_sloifotread_AULTioctl trmote_ge *kvd = (u&cd{ded;l *wr)E(	Wmory_sloifaddsg->-Efset = 0_		re
st_page(vcpuwrgppu&cdpukvm:	pocd)))E(	Wmory_sloifotread_0_toobt_pte f}	
corysKsynCHECK_EXTENSION:otread_AULTTm_ioctl tMIXEDexteitegn_rn __str = (uwrgOn_trbt_pte fd, len):otread_AULTack oTmMioctlt ilp*tioctl,wwrgOn_f}	irty =
		*writabtrustall_new_memslotsCOMPATtvmy(slo!(vmat_k_ge seg;
loglva_ da32 = losl_ da32 padcNVA1if untallva_m!(vmat_uptr_LLmdrty_bitmap;uct  *sliit p;
	e)
{
	t;
	 da64 padcNVA2n_f};
}ssWBh
EGyE      gfn_at!(vmat_ioctltcked =  ile * ilp*mems;
	 offset_ifn_tioctl,write_fault || wrgOENT;

	n = mapp;
	u ;
 ilp->n_ivate_kvm_; 	_atotured.
 *
	u->mma!;
is functionn 0;
}
EXPO>diO.acsnet		W(ioctlfn		
corysKsynGET_DIRTY_LOG:hva_many(slo!(vmat_k_ge seg;
logl!(vmat_t gt _many(slok_ge seg;
loglt gt add		re
st_p
{
	WARN_O&c(vmat_t gvm_read_guest_pagergp
EMSLOT kvm:	poc(vmat_t g)))E(	Wret;
		offset = 0_t g.= lo	 sl!(vmat_t g.= lo= 0_t g.padcNVA1	 sl!(vmat_t g.padcNVA1if _t g.padcNVA2	 sl!(vmat_t g.padcNVA2if _t g.mdrty_bitmap sl!(vmat_ptru!(vmat_t g.dirty_bitmapes)f ssg->AULTTm_ioctl e *g seg;
logd = (u&t g) boobt_pte f}	
d, len):otread_AULTTmMioctlt ilp*tioctl,wwrgOn_f}	=
		*writabtr needed.Bh
EGyE), gfn, ilelop _kvm_rs
_usermEiopsft,NT;.	 emoryssssssss= *datamnsiemory,T;.ritable,_ioctls= *datamnioctl,rstall_new_memslotsCOMPATt .!(vmat_ioctlsss= *datamn!(vmat_ioctl,f neederR.l_SYek		=alsop_l_SYek,
}ssWBh
EGyEoid *datge _ioctl trmote_rm(rite_fault || type{
		WAatoturn

	n = mapp;
	uurn

	n =  ile * ile;trymap d_AULTtrmote_rm(type{;t 		reISurn Ok_ger on
		mems_gfnrn O, g)if tall_new_memslotsMMIOngead_AULTtoalesc m_mmio_iyEG(
	ufn;
		rear;

	t(slomap_, gfmap 
	ufn;
;
		*writab	}	 needer
r_die *gunestd_fd_flags(O_CLOEXECM_M 		rear;

	t(slomap_, gfmap 
	ufn;
;
		*writab	}		 ile slocgn_ p is gpaiilet"
	u-em",,&map_;mmiops*  = (uO_RDWR{;t 		reISurn Oiile{=m ins, gfunestd_fd(r) boomap_, gfmap 
	ufn;
;
		*wri_gfnrn Ofic gfnlho
 ad(&curDon'c   lo__use, gfmaplocym wriovt kis pu pf;  ile->flopaislse {al FOLy a i, net	Wtit_emoryt)lbei    gfn_att_emoryt).  In  offs];

		orys it millnbe);	/* maigneheteinalufautOfic gee[0]millntake];

		owriof doi    gfn, gfmap 
	uf.(&cufault, AULTtrmote_rmEdebugf
t = (ur{r;

	 @pfn, gfunestd_fd(r) boofautOfic g; 0;
}
EXPO>dNOMEMfn	 onAULTuevunc__fasty_ tbase(KsynEVENTnCREATE_kMpu, g	_tm f,_inBh
ll(;,toic g; 0
		*writabtruBh
EGyE      gfnge _ioctltcked =  ile * ilp*mems;
 offset_ifn_tioctl,write_fault || wrgOENT;@log:rg->-Eirty_biacsnet		W(ioctlfn		
corysKsynGET_API_VERSION:otr pagergOE(	Wmory_slot olrg->_t hAPI_VERSIONn_trbt_pte fcorysKsynCREATE_kM:otread_AULTge _ioctl trmote_rm(wrgOn_trbt_pte fcorysKsynCHECK_EXTENSION:otread_AULTTm_ioctl tMIXEDexteitegn_rn __strck);(uwrgOn_trbt_pte fcorysKsynGET_kvm,_MMAP_SIZE:otr pagergOE(	Wmory_slot olrg->GPL(kvIZE;sssssct any(slok_getuncufa tall_new_memsX86nglrg+->GPL(kvIZE;ssssct piogkvm_ie)
{
	t; neederstall_new_memslotsMMIOnglrg+->GPL(kvIZE;ssssct toalesc m mmio rs, ae)
{
	t; needertrbt_pte fcorysKsynTRACE_ENABLE:ffcorysKsynTRACE_PAUSE:ffcorysKsynTRACE_DISABLE:ffdsg->-EOPNOTSUPPn_trbt_pte fd, len):otre_GPL(gic sack oge _ioctlt ilp*tioctl,wwrgOn_f}	irty =
		*writabtruBh
EGyE), gfn, ilelop _kvm_rs
_usecharge _opsft,NT;.ritable,_ioctls= *datge _ioctl,f .!(vmat_ioctlsss= *datge _ioctl,f .l_SYek		=alsop_l_SYek,
}ssWBh
EGyE), gfn,miscge *kv *datge ft,NT;lotsMINOR,f "
	u",f &_usecharge _ops,
}ssWBh
EGyEread_hargwm_meotuse __ftabltead_gujunk{
		WAatofauaKVraw_pm tproc_PAorni	d)est_atoturrelt, faue a Ttvm_Lypu(faupu
	ps_hargwm_meotuse d{=t gfn)
{
slotfaue a Tse_Lypu(faupu
	ps_hargwm_meotuse d{)
_read_AULTack ohargwm_meotuse (edelolt, r)m insfaue a T;
	relypu(faupu
	ps_hargwm_meotuse d{)
	 _read_t
stthhargwm_meotuse _urn  d{)
	 pdDGyfot"
	u:lotuses, a
	}
	*pizy_se_gfn)vm,%d urn  d\n"pu
	pOn_f}	re
Bh
EGyEoid *datsgotZ
stlypu( offset_ifn_tfau{
		
raw_pd if and cleaTtount_f needed.
 *
	ugusn -=tount=t ghargwm_meotuse __ftabltVEGyEGyEraw_pd ifit and cleaTtount_f needed_pa	return 0;vm, gfnread_hargwm_medisuse __ftabltead_gujunk{
		WAatofauaKVraw_pm tproc_PAorni	d)esbolt, !faue a Ttvm_Lypu(faupu
	ps_hargwm_meotuse d{=t gfn)
{
slsfaue a T;
	relypu(faupu
	ps_hargwm_meotuse d{)
	AULTack ohargwm_medisuse ()

re
Bh
EGyEoid *datdy
stlypu( offset_ifn_tfau{
		
raw_pd if and cleaTtount_f needed.
 *
	ugusn -=tount=t ghargwm_medisuse __ftabltVEGyEGyEraw_pd ifit and cleaTtount_f needed_pa	return 0;vm, gfnread_hargwm_medisuse _atmapftabltead_{
		

		if (!
	ugusn -=tount=;trymap_usn -=tount--if  pag!_useusn -=tount=t ggn_eneroypu(hargwm_medisuse __ftablpuVEGy,n_ran 0;vm, gfnread_hargwm_medisuse _atmtead_{
		
raw_pd if and cleaTtount_f neededhargwm_medisuse _atmapftabltEGyEraw_pd ifit and cleaTtount_f needere
Bh
EGyEoid hargwm_meotuse _atmtead_{
		
_atot KVm *kvraw_pd if and cleaTtount_f neederymap_usn -=tount++;
o.
 *
	ugusn -=tounts==
1)m ins_read_t pagehargwm_meotuse _urn  d,

	;t ggn_eneroypu(hargwm_meotuse __ftablpuVEGy,n_ranotr pageread_ther thhargwm_meotuse _urn  d{)m insmhargwm_medisuse _atmapftabltEGyEfdsg->-EBUSYt olho }
yEraw_pd ifit and cleaTtount_f needeed
		*writabtruBh
EGyEoid *datreboa +t kvm_v_fast(adDbf ne *_fast(ad,,rite_fault || val,
msrty0);ead_guv{
		
ad(&curSosl;(well,wwid _ is min geBIOSys tbasgfn)reboa  ielse {in vmx roa  m isusse lse {A[0]Intel TXTdynctired VMXkt rto_hvalpffau &pfn system shutdown.(&cufaupdDGyfot"
	u:loxio
stehargwm_ma
	}
	*pizy_se_\n"{)
	AULTreboa 
stemo;

	retgn_eneroypu(hargwm_medisuse __ftablpuVEGy,n_ran;
}
EXPOcOTIFY_OKabtruBh
EGyE), gfn,_fast(adDbf ne AULTreboa __fast(adft,NT;._fast(adD  lo_= AULTreboa ,f . rioritytKVm,
}ssWBh
EGyEread_AULTio_busMde

	oydany(slok_geio_bus *bus{
		
_atoi_tm for ni KVm ts;
 busr ge Ttount ts++=m insany(slok_geio_ge *kv *posft,busr rbase[i].ge ;tboomap_ioge *kvmde

	(slor(posOn_f}	=kpage(bps))
truBh
EGyEoilr_pEoid *datio_busMcmp(_kvm_rcked = *datio_rbase *r1p
EMSLO_kvm_rcked = *datio_rbase *r2, len)vm_meaddr1aKVr1->addr;en)vm_meaddr2aKVr2->addr;e
o.
 *addr1a<eaddr2n 0;
}
EXPO>1(!vmct IfVr2->t, gK=

, mat		W(msloxacmeaddrpag.  IfVr2->t, g!KVm,
se {accepvlocy overlapr
		/_read.  Acy orcet

s{accepvarc =fory
 * averlapr
		/rbases,
		npages*datio_busMe *gfirt _devlotstruco
 * wkvm_sc_PAvalpfof (msm.(&cufault, r2->t, )m ins_ddr1a+KVr1->t, ;ins_ddr2a+KVr2->t, ;in}e
o.
 *addr1a>eaddr2n 0;
}
EXPO1;_*writable = truBh
EGyEoid *datio_busMAortMcmp(_kvm_read_gup1, _kvm_read_gup2{
		W gfn)
{
	reio_busMcmp(p1, p2{= truBh
EGyEoid *datio_busMinser _devdany(slok_geio_bus *bus, any(slok_geio_ge *kv *ge *mems  )vm_meaddr (void __user busr rbase[busr ge Ttount++]a= (cked = *datio_rbase)m ins.addr sloddr ins.t, gKd __ ins.ge ft,ge *me}biacsort(busr rbase, busr ge Ttount	rpim:	poany(slok_geio_rbase),boomap_io_busMAortMcmppuVEGyEGy*writable = truBh
EGyEoid *datio_busMe *gfirt _devdany(slok_geio_bus *bus,
	msrty0))vm_meaddr (void __user cked = *datio_rbase *rbase, keyest_atot rderymeya= (cked = *datio_rbase)m ins.addr sloddr ins.t, gKd __ in} *kvran&entrbseack  cley, busr rbase, busr ge Ttount	
SL	kvm:	poany(slok_geio_rbase), *datio_busMAortMcmpM_M 		reaan&ent;
ing to c
		memsetNOENTde(noff K/rbase - busr rbasege(struct koff >his iM
	reio_busMcmp(cley, &busr rbase[off-1]) K=

	addoff--Gy*writablet rdetruBh
EGyEoid __
	reio_busM_read(irty_bitmay_bitmap)
		rpny(slok_geio_bus *bus,
	msrty0)rcked = *datio_rbase *rbase, _kvm_read_guval{
		
_atoidx;e
o.dxad_AULTio_busMe *gfirt _devdbus, rbase->addr, rbase-> __u_M 		re.dxa;

	 ne
		memsetOPNOTSUPPn_(struct k.dxa;
busr ge Ttount  l ne
	reio_busMcmp(rbase, &busr rbase[.dx]) K=

	m ins_reg!map_ioge *kvm_read(p)
		rbusr rbase[.dx].ge purbase->addr,
MEMSLrbase-> __meval))E(	Wret;
		idx;e	o.dx++;
ourn hva;
}
etOPNOTSUPPn_true; 
	reio_busM_read - ;	/* mauncet

	u->= los_f ne */
oid *datio_busM_read(irty_bitmay_bitmap)
		renumitmaybus busMidx,))vm_meaddr 
msrty0)void __, _kvm_read_guval{
		
pny(slok_geio_bus *bus;r cked = *datio_rbase rbaseget_atoturreran&entr(cked = *datio_rbase)m ins.addr sloddr ins.t, gKd __ in} *kvbus = srcYMcetm->mmap_(vfau->lear buses[busMidx], &efau->
	u->=rcY  bolt, !bus{
0;
}
EXPO>dNOMEMfn	r sl__
	reio_busM_read(p)
		rbus, &rbase, val{; 0
		*writa;

 ?ita:
 = true; *datio_busM_readTtookid - ;	/* mauncet

	u->= los_f ne */
oid *datio_busM_readTtookid(irty_bitmay_bitmap)
		renumitmaybus busMidx,
	msrty0)vm_meaddr (void __, _kvm_read_guvalcpt || tookid{
		
pny(slok_geio_bus *bus;r cked = *datio_rbase rbasegereran&entr(cked = *datio_rbase)m ins.addr sloddr ins.t, gKd __ in} *kvbus = srcYMcetm->mmap_(vfau->lear buses[busMidx], &efau->
	u->=rcY  bolt, !bus{
0;
}
EXPO>dNOMEMfnvmct Firt taddr(mslge *kv tm->mmap_maigntookid.cufault, (tookid >=

	mt pitookid ;
busr ge Ttount)  l nrty0(
	reio_busMcmp(crbase, &busr rbase[tookid]) K=

	)ins_reg!map_ioge *kvm_read(p)
		rbusr rbase[tookid].ge puaddr ( __ ins	BOval))E(	Wret;
		tookidn_read(&curtookid is_dained garbtruar;
	l babl  o seack ee[0]fn)
{
		he];

		orrn grtookid valuem_memoryfn)
{
	__
	reio_busM_read(p)
		rbus, &rbase, val{; truBh
EGyEoid __
	reio_busMher tirty_bitmay_bitmap)
		rpny(slok_geio_bus *bus,
	msrty0)cked = *datio_rbase *rbase, ead_guval{
		
_atoidx;e
o.dxad_AULTio_busMe *gfirt _devdbus, rbase->addr, rbase-> __u_M 		re.dxa;

	 ne
		memsetOPNOTSUPPn_(struct k.dxa;
busr ge Ttount  l ne
	reio_busMcmp(rbase, &busr rbase[.dx]) K=

	m ins_reg!map_ioge *kvmher tp)
		rbusr rbase[.dx].ge purbase->addr,
MEMSsssssssrbase-> __meval))E(	Wret;
		idx;e	o.dx++;
ourn hva;
}
etOPNOTSUPPn_trLT;
	r = __copy_to_useio_busM_read);rue; *datio_busM FOLL- ;	/* mauncet

	u->= los_f ne */
oid *datio_busMher tirty_bitmay_bitmap)
		renumitmaybus busMidx,))vm_meaddr 
msrty0void __, ead_guval{
		
pny(slok_geio_bus *bus;r cked = *datio_rbase rbaseget_atoturreran&entr(cked = *datio_rbase)m ins.addr sloddr ins.t, gKd __ in} *kvbus = srcYMcetm->mmap_(vfau->lear buses[busMidx], &efau->
	u->=rcY  bolt, !bus{
0;
}
EXPO>dNOMEMfn	r sl__
	reio_busmher tp)
		rbus, &rbase, val{; 0
		*writa;

 ?ita:
 = truue; C	/* r mut t*alc = los_f ne. */
oid *datio_busMhegisterege ruct kvm_vcpu *vcpenumitmaybus busMidx,))vm_meaddr 
mssrty0void __, any(slok_geio_ge *kv *ge {
		
pny(slok_geio_bus *new_bus, *bus;rkvbus = k_gee *gbust = (ubusMidx  bolt, !bus{
0;
}
EXPO>dNOMEMfnvmct exclude ioevuncfV_ {
		Wis limikn  ignmaximumifV_ufault, busr ge Ttount - busr ioevuncfVTtount > NR_IOBUS_DEVS - 1r on
		memsetNOSPChostnew_bus = kmaticc(kvm:	po*bus{ +, (busr ge Ttount +
1)m*
mssrtkvm:	poany(slok_geio_rbase)), GFP_KERNEL  bolt, !new_bus{
0;
}
EXPO>dNOMEMfn	memcpy(new_bus, bus, avm:	po*bus{ +, busr ge Ttount *
Sssssssskvm:	poany(slok_geio_rbase)){)
	AULTio_busMinser _devdnew_bus, ge puaddr ( __{; 0
cYMBste_fMpu pfnstlear buses[busMidx], new_bus{;r c : 0),
vm:_srcYMexpedikn  clear =rcY  bokpage(bps))
n hva;
}
 = true; C	/* r mut t*alc = los_f ne. */
read_AULTio_busMunBOgisterege ruct kvm_vcpu *vcpenumitmaybus busMidx,
	msrty0)r any(slok_geio_ge *kv *ge {
		
_atoi_t
pny(slok_geio_bus *new_bus, *bus;rkvbus = k_gee *gbust = (ubusMidx  bolt, !bus{
0;
}
EXP_tm for ni KVm ts;
 busr ge Ttount ts++=ins_regbusr rbase[i].ge  K=
ge {m insmbt_pte fn}e
o.
 *i K=
busr ge Ttount)
0;
}
EXP_tm new_bus = kmaticc(kvm:	po*bus{ +, (busr ge Ttount -
1)m*
mssrtkvm:	poany(slok_geio_rbase)), GFP_KERNEL  bolt, !new_bus{  @pfn,r_errt"
	u:lurn  d  o shrink bus, 	rmov
		/ilo!(vmletely\n"{)
	Wmory_brok, ;in}e
omemcpy(new_bus, bus, avm:	po*bus{ +,i *ukvm:	poany(slok_geio_rbase));m new_busr ge Ttount--if memcpy(new_busr rbase +,i	rbusr rbase +,i +
1,
Ssssssss(new_busr ge Ttount -
i) *ukvm:	poany(slok_geio_rbase));m
brok, : 0
cYMBste_fMpu pfnstlear buses[busMidx], new_bus{;r c : 0),
vm:_srcYMexpedikn  clear =rcY  bokpage(bps))
;
}
EXP_t}sWBh	n = map_io_ge *kv *AULTio_busMe *gge ruct kvm_vcpu *vcpenumitmaybus busMidx,
	msssr)vm_meaddr{
		
pny(slok_geio_bus *bus;r _atoge _idx,)srcYMidx;e	Bh	n = map_io_ge *kv *ioge UKVck);

resrcYMidx = srcYMBOL_Gtabltclear =rcY  bkvbus = srcYMcetm->mmap_(lear buses[busMidx], &
	u->=rcY  bolt, !bus{
0;mory_slo_it andfn_tge _.dxad_AULTio_busMe *gfirt _devdbus, addr (1  bolt, ge _.dxa;

	 nemory_slo_it andfn_tioge UKVbusr rbase[ge _.dx].ge ;tbslo_it and:resrcYMBOL_Git and &
	u->=rcY,)srcYMidx))
n hva;
}
ioge n_trLT;
	r = __copy_to_useio_busMe *gge )ssWBh
EGyEoid *datgebugf
	opeiOurn -EFin is
*in is,rcked =  ile * ile,
	msrtyoid (*e *)tead_gu, a64 *),yoid (*s *)tead_gu, a64),
	msrty_kvm_rchar *fmt{
		
pny(slok_geBh
E_kvm_ *Bh
E_kvm_ tr(cked = *datBh
E_kvm_ *)
	msssrFin isr i_n_ivatefnvmct Tmslgebugf
  iles_ t *i tm->mmap_  o (msl_vcpcked =  {
		lse {ispckillnvalid &pfn *datge

	oy_vm siz
	/* m.(&curTo aead_g(mslloce betwefn opeiee[0](msllrmovapfof (mslgebugf
(&curdirn goddrwentge(pagainBh (mslusers tount.(&cufault, !tm-tount_inc__fa_zero(&Bh
E_kvm_->
	u->usersTtount)to c
		memsetNOENTde(nARN_pivmle_attr	opeiOin is,r ile,rg i,
MEMy0)r an
E_kvm_->m is
s S_IWUGO ?ia i :uVEGy,
MEMy0)r fmt{	t(slomap_, gfmap Bh
E_kvm_->
	ug; 0;
}
EXPO>dNOMEMfn	 o*writable = truBh
EGyEoid *dattebugf
		remorytcked = in is
*in is,rcked =  ile * ild{
		
pny(slok_geBh
E_kvm_ *Bh
E_kvm_ tr(cked = *datBh
E_kvm_ *)
	msssrFin isr i_n_ivatefnvmpivmle_attr		remorytin is,r ile{)
	AULT, gfmap Bh
E_kvm_->
	ug; *writable = truBh
EGyEoid datBh
E_ D_PAereap ead_gud(is_ea64 *val{
		
pny(slok_geBh
E_kvm_ *Bh
E_kvm_ tr(cked = *datBh
E_kvm_ *)kvm_; 
	*val tr*(ut || *)( ead_gu)Bh
E_kvm_->
	u +
Bh
E_kvm_->offa i)t *writable = truBh
EGyEoid datBh
E_;
	relAereap ead_gud(is_ea64 val{
		
pny(slok_geBh
E_kvm_ *Bh
E_kvm_ tr(cked = *datBh
E_kvm_ *)kvm_; 
	lt, valn 0;
}
EXPO>dirty_biac*(ut || *)( ead_gu)Bh
E_kvm_->
	u +
Bh
E_kvm_->offa i) KVm *kvritable = truBh
EGyEoid datBh
E_ D_PAereap	opeiOurn -EFin is
*in is,rcked =  ile * ile{
		
__pivmle_attr	tMIXEDformat("%llu\n"pu0ull{; 0
		*wri*datgebugf
	opeiOin is,r ile,rdatBh
E_ D_PAereap,
	mssdatBh
E_;
	relAereap, "%llu\n"{; truBh
EGyE_kvm_rcked =  ilelop _kvm_rs
datBh
E_ D_PAereap	iopsft,NT;.own r ss= THIS_MODULE,f .opeie ss= datBh
E_ D_PAereap	opei,rR.	 emorys= *datgebugf
		remory,rR.	 ade ss= pivmle_attr		rad,rR._read ss= pivmle_attr	_read,f .l_SYekss= no_l_SYek,
}ssWBh
EGyEoid T_SYMfh
E_ D_PAereap ead_gud(is_ea64 *val{
		
_atoi_t
pny(slok_geBh
E_kvm_ *Bh
E_kvm_ tr(cked = *datBh
E_kvm_ *)kvm_; n

	n = map_;
	pagefau

t *val tr0derymap_o_h_eneroT_SY(i,VvfaupuBh
E_kvm_->
	ug
	m*val +tr*(u64 *)( ead_gu);
	pa+
Bh
E_kvm_->offa i)t *writable = truBh
EGyEoid d_SYMfh
E_;
	relAereap ead_gud(is_ea64 val{
		
_atoi_t
pny(slok_geBh
E_kvm_ *Bh
E_kvm_ tr(cked = *datBh
E_kvm_ *)kvm_; n

	n = map_;
	pagefau

t lt, valn 0;
}
EXPO>dirty_biacmap_o_h_eneroT_SY(i,VvfaupuBh
E_kvm_->
	ug
	m*(u64 *)( ead_gu);
	pa+
Bh
E_kvm_->offa i) KVm *kvritable = truBh
EGyEoid d_SYMfh
E_ D_PAereap	opeiOurn -EFin is
*in is,rcked =  ile * ile{
		
__pivmle_attr	tMIXEDformat("%llu\n"pu0ull{; 0
		*wri*datgebugf
	opeiOin is,r ile,rd_SYMfh
E_ D_PAereap,
MEMSsd_SYMfh
E_;
	relAereap, "%llu\n"{; truBh
EGyE_kvm_rcked =  ilelop _kvm_rs
d_SYMfh
E_ D_PAereap	iopsft,NT;.own r ss= THIS_MODULE,f .opeie ss= d_SYMfh
E_ D_PAereap	opei,rR.	 emorys= *datgebugf
		remory,rR.	 ade ss= pivmle_attr		rad,rR._read ss= pivmle_attr	_read,f .l_SYekss= no_l_SYek,
}ssWBh
EGyE_kvm_rcked =  ilelop _kvm_rs
*Bh
E_iopsPAereap[]ft,NT	[lotsSTAT_kvm,]ft,&d_SYMfh
E_ D_PAereap	iops,T	[lotsSTAT_kM] ss= &datBh
E_ D_PAereap	iops,
}ssWBh
EGyEoid TatBh
E_ D_ ead_gu_offa i_ea64 *val{
		
rite_fauloffa i tr(t ||)_offa i;rn

	n = mapp;
	uurn

	n = *datBh
E_kvm_ Bh
E_tmp sl{.offa i troffa i}ss	a64 tmp_val

t *val tr0dermutexf any page_f neededlst _o_h_eneroe);
}d = (u&ge_fst ,rdatfst =m insan
E_tmp.keaaKVAUL)
ssdatBh
E_ D_PAereap  ead_gu)&an
E_tmppu&tmp_valg; 0;*val +trtmp_val

f}	=mutexfit and clea_f needed_pa	return 0;vm, gfnoid datBh
E_;
	re ead_gu_offa i_ea64 val{
		
rite_fauloffa i tr(t ||)_offa i;rn

	n = mapp;
	uurn

	n = *datBh
E_kvm_ Bh
E_tmp sl{.offa i troffa i}sst lt, valn 0;
}
EXPO>dirty_biacmutexf any page_f neededlst _o_h_eneroe);
}d = (u&ge_fst ,rdatfst =m insan
E_tmp.keaaKVAUL)
ssdatBh
E_;
	relAereap  ead_gu)&an
E_tmppu0)

f}	=mutexfit and clea_f needeed_pa	return 0;DEFINE_SIMPLE_ATTRIBUTE(datBh
E_iops,VvatBh
E_ D_, datBh
E_;
	re, "%llu\n"{; uBh
EGyEoid d_SYMfh
E_ D_ ead_gu_offa i_ea64 *val{
		
rite_fauloffa i tr(t ||)_offa i;rn

	n = mapp;
	uurn

	n = *datBh
E_kvm_ Bh
E_tmp sl{.offa i troffa i}ss	a64 tmp_val

t *val tr0dermutexf any page_f neededlst _o_h_eneroe);
}d = (u&ge_fst ,rdatfst =m insan
E_tmp.keaaKVAUL)
ssd_SYMfh
E_ D_PAereap  ead_gu)&an
E_tmppu&tmp_valg; 0;*val +trtmp_val

f}	=mutexfit and clea_f needed_pa	return 0;vm, gfnoid d_SYMfh
E_;
	re ead_gu_offa i_ea64 val{
		
rite_fauloffa i tr(t ||)_offa i;rn

	n = mapp;
	uurn

	n = *datBh
E_kvm_ Bh
E_tmp sl{.offa i troffa i}sst lt, valn 0;
}
EXPO>dirty_biacmutexf any page_f neededlst _o_h_eneroe);
}d = (u&ge_fst ,rdatfst =m insan
E_tmp.keaaKVAUL)
ssd_SYMfh
E_;
	relAereap  ead_gu)&an
E_tmppu0)

f}	=mutexfit and clea_f needeed_pa	return 0;DEFINE_SIMPLE_ATTRIBUTE(d_SYMfh
E_iops,VvfautBh
E_ D_, d_SYMfh
E_;
	re,
MEM"%llu\n"{; uBh
EGyE_kvm_rcked =  ilelop _kvm_rs
*Bh
E_iops[]ft,NT	[lotsSTAT_kvm,]ft,&d_SYMfh
E_iops,T	[lotsSTAT_kM] ss= &datBh
E_iops,
}ssWBh
EGyEif (kvm_iuevunc__fasty_ tbase( offset_ifn_ttype,S

	n = mapp;
	u{
		
pny(slokobjiuevunc_env_*e) ;t rite_fault || t || trmotem, activvesbolt, !*datgev. kis_ge *kv || !
	ug
	m
}
EXP_tm mutexf any page_f neededARN_typeR==
_t hEVENTnCREATE_kM	t(slomap_trmotevm=tount++;
o	AULTactivveaps++;
orUgned ARN_typeR==
_t hEVENTnDEScROY_kM	t(slomap_activveaps--if }	
crmotem d_AULTtrmotevm=tountif activv tomap_activveaps;	=mutexfit and clea_f needeedenv_KVAzaticc(kvm:	po*env), GFP_KERNEL  bolt, !env)
	m
}
EXP_tm addiuevunc_va_Oe)v, "CREATED=%llu"pu
rmotem  boaddiuevunc_va_Oe)v, "COUNT=%llu"puactivv)esbolt, typeR==
_t hEVENTnCREATE_kM	t(sloaddiuevunc_va_Oe)v, "EVENT=
rmote") boomap->userspace_pKV_dil a MpKV_nrtected.
);
orUgned ARN_typeR==
_t hEVENTnDEScROY_kM	t(sloaddiuevunc_va_Oe)v, "EVENT=ge

	oy")

f}	=addiuevunc_va_Oe)v, "PID=%d", map->userspace_pKV)esbolt, !ISurn _OR_VEGy(lear debugf
	de);
}
)m insfhar *tmppu*p = kmaticc(PATHhMAX, GFP_KERNEL  bins_regp)m insmtmp slde);
}_path_raw(lear debugf
	de);
}, ppuPATHhMAXn_t edlt, !ISurn Otmpn)E(	WWaddiuevunc_va_Oe)v, "STATS_PATH=%s", tmpM_M d;kpage(pM_M dho }
mct no neem for tMIXEs, avap_ we_ t *idcNVAwwidmom_ronly 5 keyscufaue)v->e)vp[e)v->e)vp_.dx++]a= ck);

	kobjectiuevunc_env(&*datgev. kis_ge *kv->
obj, KOBJ_CHANGEcpenv->e)vp  bokpage(env)= truBh
EGyEoid *datini_pdebug(vad_{
		
_atot KV-EEXISTded

	n = *datBh
Estgebugf
	item *pge(s*datgebugf
	dir sldebugf
	trmote_girt"
	u"puVEGyEGyEARN_nt(lgebugf
	dir s;
ing to cmory_sloifotnt(lgebugf
	num_e);
iesUKV0derfor np sldebugf
	e);
ies; p->nasl; ++ppu, gegebugf
	num_e);
ies++=m ins_atom is
= p->m is
? p->m is
: 0644;ins_reg!debugf
	trmote_iiletp->nasl,@m is,rnt(lgebugf
	dir,
	msssr ead_gu)(t ||)p->offa i,
	msssrBh
E_iops[p->kind]))E(	Wmory_slo_dirfn	 o*writable = 
slo_dir:edtebugf
		rmove_yn	0;sivvgnt(lgebugf
	dir);	irty =
		*writabtruBh
EGyEoid *datsuspeid(vad_{
		
_
 *
	ugusn -=tount=t ghargwm_medisuse __ftabltVEGyEGyErpa	return 0;vm, gfnread_
	ugresume(vad_{
		
_
 *
	ugusn -=tount=m insWARNif (raw_pd ifis_f nen  cleaTtount_f neeM_M dhargwm_meotuse __ftabltVEGyEGyE}btruBh
EGyE), gfn,syscorv opsf*datsyscorv opsft,NT;.suspeid tomap_suspeid,rR.	 sume_= AULTresume,
}ssWBh
EGyEoilr_p


	n = map_;
	pagpagemp __fast(adpageT_SY(

	n = pagemp __fast(adagpn{
		W gfn)
{is_dainer_of(p_, any(slok_gevfaupupagemp __fast(adran 0;vm, gfnread_map_stMI,_in(

	n = pagemp __fast(adagpn,ifn_tfau{
		


	n = map_;
	pagefau
= pagemp __fast(adpageT_SY(pn)esbolt, efau->nagemp eV
t gefau->nagemp eV
= fanedifotnt(lack ostMI,_in(vfaupu
	p)ifotnt(lack ovfaumload(vfaupu
	p)if 0;vm, gfnread_map_stMI,_irt(

	n = pagemp __fast(adagpn,
mssrtk
	n = l a Mk
	n = *next{
		
pny(slok_ge;
	pagefau
= pagemp __fast(adpageT_SY(pn)esbolt, is functivm, eR==
TASKncUNNING
t gefau->nagemp eV
= ;

	retnt(lack ovfaumput(vfaun_t 0;oid *datini_ ead_guopaque,,rite_fauld_SYMfvm:,,rite_fauld_SYMale_f 
msrt), gfn,module *module{
		WAatoturnfn_tfau)
_read_AULTack oini_ opaqueM_M 		rea	 nemory_slo_urn n_read(&curAULTack oini_ makesUstru (msre'swwidmom_rone);	/* ry
 * o_hvack itecttruc  kvm_supporidmultiple ivmlemunckvm_rs,
se {t wrifn_elee[0]amdgfn)x86.(&curAULTack oini_ mut tbe);	/* maief wri*datirqfVoini_ to aead_gtrmotNVA];

		onflicts{in coryskvm sizal FOLy a iup o_hva_fahet

vmlemunckvm_rm_memoryfad_AULTirqfVoini_(M_M 		rea	 nemory_slo_irqfVesbolt, !zaticc_faue a Tva_O&
	ps_hargwm_meotuse d, GFP_KERNEL =m inssg->ofNOMEMfn		mory_slo_pagen0fn	 o*wrad_AULTack ohargwm_mea iup(M_M 		reaa;

	 nemory_slo_pagen0a_tm for_enerooilr_plypu(fau=m insampD  lo_funcvm_rMfvngletfaup
	mssAULTack otMIXEDproc_PAorn!(vmatp
	mss&r (1  bo 		reaa;

	 neemory_slo_pagen1fn	 o*wrad_fauhpea iuptot		u__f  los(vm,HP_AP_lotsSTARTING, "AUL/fau:sgotZ
st",
MEMSssssss*datBh
tZ
stlypu,rnt(lgy
stlypuM_M 		rea	 nemory_slo_uagen2GyErpgisterereboa __fast(ad cleaTreboa __fast(ad)fnvmct A kmem cnere lets{us meeh (mslale_fmuncdynctiremuncsfof fx_savd.cufault, !d_SYMale_f
t gefauMale_f sl__ale_fof__tirty_bitmay_bit{)
	AULT_t *n!nere = kmemn!nere	trmotet"
	uT_t *",ld_SYMfvm:,,d_SYMale_f 
msEMSsssSLAB_ACCOUNTpuVEGyEGyEARN_!AULT_t *n!nere=m inssg->ofNOMEMfn		mory_slo_pagen3fn	 o*wrad_AULTac : _pfoini_(M_M 		rea	 nemory_slo_pageifotnt(lcharge _ops.own r =,module)
	AULT_p	iops.own r =,module)
	AULT_t *niops.own r =,module)
*wrad_miscMhegister(&*datgevM_M 		rea	 @pfn,r_errt"
	u:lmisclge *kv tmgister urn  d\n")fn		mory_slo_unBOgfn	 o*wrigisteresyscorv opsO&k_gesyscorv ops)ifotnt(lpagemp _ops.stMI,_in tomap_stMI,_in;otnt(lpagemp _ops.stMI,_slo tomap_stMI,_sloifotfad_AULTini_pdebug(M_M 		rea	 @pfn,r_errt"
	u:ltrmotelgebugf
  iles_urn  d\n")fn		mory_slo_ungebugf
fn	 o*wrad_AULTvfiomops_ini_(M_M WARNif (redeed_pa	return
slo_ungebugf
:t ririgisteresyscorv opsO&k_gesyscorv ops)if	miscMcetmgister(&*datgevM_Mslo_unBOg:etnt(lac : _pfodeini_(M_Mslo_page:etnmemn!nere	de

	oydAULT_t *n!nere=_tirt_pagen3:t ririgisterereboa __fast(ad cleaTreboa __fast(ad)fn	fauhpe	rmove_ot		u__f  los(vm,HP_AP_lotsSTARTING=_tirt_pagen2:tirt_pagen1:
	AULTack ohargwm_merit iup(M_Mslo_pagen0a:
	pagenfaue a Tva_O
	ps_hargwm_meotuse d{)
slo_pagen0:
	AULTirqfVooxio(M_Mslo_irqfV:
	AULTack ooxio(M_Mslo_urn y =
		*writabtrLT;
	r = __copy_to_useiyEG
;ruif (kvm_ioxio(vad_{
		
tebugf
		rmove_yn	0;sivvgnt(lgebugf
	dir);		miscMcetmgister(&*datgevM_Mtnmemn!nere	de

	oydAULT_t *n!nere=_ttnt(lac : _pfodeini_(M_M ririgisteresyscorv opsO&k_gesyscorv ops)if	ririgisterereboa __fast(ad cleaTreboa __fast(ad)fn	fauhpe	rmove_ot		u__f  los(vm,HP_AP_lotsSTARTING=_ttgn_eneroypu(hargwm_medisuse __ftablpuVEGy,n_ran;AULTack ohargwm_merit iup(M_M	AULTack ooxio(M_M	AULTirqfVooxio(M_M	pagenfaue a Tva_O
	ps_hargwm_meotuse d{)
	AULTvfiomops_oxio(M_MtrLT;
	r = __copy_to_useoxio{; uBhty_bitmay_m_workadpahBOL_Gis_dext {rn

	n = mapp;
	uurn

	n = l a Mk
	n = *parunct many(slo!(vmletse_gini_pdt pt 	AULT_p	ahBOL_Gfn_h (mBOL_Gfnif	rfn_ptr_LLmvm_; 	_atoerr;
}ssWBh
EGyEoid *dat_m_workadpahBOL_ ead_guis_dext{
		
ad(&curTmslini_pis_dext sizaltabotem o
		herBh
ckfof (mslparunc (mBOL_, aoo
 * wkvhavd to lf  logntopylocythNVAw kvm_sizneem maieyondlini_i*pizy_se__memoryBhty_bitmay_m_workadpahBOL_Gis_dext *ini_pis_dext = is_dext;rn

	n = mapp;
	u = ini_pis_dext->lea)
	AULT_p	ahBOL_Gfn_h (mBOL_Gfn = ini_pis_dext->(mBOL_Gfnif	rfn_ptr_LLmvm_ = ini_pis_dext->mvm_; 	_atoerr;

	errad_A(mBOL_Gparktected.
);
oe; *(mBOL_Gparktected.
)_siznever suppos d  o 	}
	reton  offsemoryWARNif (erra!KVmM_M 		reerr	 nemory_ini_pisvmlete;

	errad_cgrouptattnerol a Matmtini_pis_dext->parunc, ected.
);
o		reerr	t(slomap_errt"%s:_cgrouptattnerol a Matmlurn  d net	Werra%d\n"p
msE__func__,Werr)fn		mory_ini_pisvmlete;
	 o*wse_Luser_n*kvdected.
, l a Mn*kvdini_pis_dext->parunc));m
ini_pisvmlete: 	_ai_pis_dext->errad_err;
	isvmlete(&_ai_pis_dext->ini_pdt p);
o	ni_pis_dext = ck);

re		reerr	 ne	}
	reterr;

	e; Wai_ to be)wok,  up ignehetspawn r ief wriproc_ecNVA.cufau*(mBOL_Gparkm (edelolt, !*(mBOL_Gshoul_Gstop(M	 neerrad_(mBOL_Gfnd = (ukvm_)if
e	}
	reterr;
 0;oid *datULTtrmote_workadpahBOL_ uct kvm_vcpu *vcpAULT_p	ahBOL_Gfn_h (mBOL_Gfn 
msEMrfn_ptr_LLmvm_,y_kvm_rchar *nasl,
msEM

	n = l a Mk
	n = **(mBOL_Gptr{
		
pny(slok_ge_m_workadpahBOL_Gis_dext 	ni_pis_dext = {}urn

	n = l a Mk
	n = *ahBOL_

t *(mBOL_Gptra= ck);

		ni_pis_dext.keaaKVAUL)
s	ni_pis_dext.parunc ;
is func)
s	ni_pis_dext.(mBOL_Gfn = (mBOL_Gfnif		ni_pis_dext.mvm_ = mvm_; 	_ai_pisvmlettald&_ai_pis_dext.ini_pdt p);

	(mBOL_ad_A(mBOL_Grun(k_ge_m_workadpahBOL_pu&iai_pis_dext,
MEMy0)r "%s-%d", nasl,@l a MpKV_nrtected.
));
o		reISurn OthBOL_er on
		mems_gfnrn OthBOL_e;

	e; A(mBOL_Grun_siznever suppos d  o 	}
	retck);emoryWARNif ((mBOL_ad=uVEGyEGy*wwai__o_h_isvmlettald&_ai_pis_dext.ini_pdt p);

	lt, !_ai_pis_dext.err	 ne*(mBOL_Gptra= ahBOL_

t hva;
}
iai_pis_dext.err;
 0