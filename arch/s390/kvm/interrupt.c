/*
 * handling kvm guest interrupts
 *
 * Copyright IBM Corp. 2008, 2015
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2 only)
 * as published by the Free Software Foundation.
 *
 *    Author(s): Carsten Otte <cotte@de.ibm.com>
 */

#include <linux/interrupt.h>
#include <linux/kvm_host.h>
#include <linux/hrtimer.h>
#include <linux/mmu_context.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/bitmap.h>
#include <linux/vmalloc.h>
#include <asm/asm-offsets.h>
#include <asm/dis.h>
#include <linux/uaccess.h>
#include <asm/sclp.h>
#include <asm/isc.h>
#include <asm/gmap.h>
#include <asm/switch_to.h>
#include <asm/nmi.h>
#include "kvm-s390.h"
#include "gaccess.h"
#include "trace-s390.h"

#define PFAULT_INIT 0x0600
#define PFAULT_DONE 0x0680
#define VIRTIO_PARAM 0x0d00

/* handle external calls via sigp interpretation facility */
static int sca_ext_call_pending(struct kvm_vcpu *vcpu, int *src_id)
{
	int c, scn;

	if (!(atomic_read(&vcpu->arch.sie_block->cpuflags) & CPUSTAT_ECALL_PEND))
		return 0;

	BUG_ON(!kvm_s390_use_sca_entries());
	read_lock(&vcpu->kvm->arch.sca_lock);
	if (vcpu->kvm->arch.use_esca) {
		struct esca_block *sca = vcpu->kvm->arch.sca;
		union esca_sigp_ctrl sigp_ctrl =
			sca->cpu[vcpu->vcpu_id].sigp_ctrl;

		c = sigp_ctrl.c;
		scn = sigp_ctrl.scn;
	} else {
		struct bsca_block *sca = vcpu->kvm->arch.sca;
		union bsca_sigp_ctrl sigp_ctrl =
			sca->cpu[vcpu->vcpu_id].sigp_ctrl;

		c = sigp_ctrl.c;
		scn = sigp_ctrl.scn;
	}
	read_unlock(&vcpu->kvm->arch.sca_lock);

	if (src_id)
		*src_id = scn;

	return c;
}

static int sca_inject_ext_call(struct kvm_vcpu *vcpu, int src_id)
{
	int expect, rc;

	BUG_ON(!kvm_s390_use_sca_entries());
	read_lock(&vcpu->kvm->arch.sca_lock);
	if (vcpu->kvm->arch.use_esca) {
		struct esca_block *sca = vcpu->kvm->arch.sca;
		union esca_sigp_ctrl *sigp_ctrl =
			&(sca->cpu[vcpu->vcpu_id].sigp_ctrl);
		union esca_sigp_ctrl new_val = {0}, old_val = *sigp_ctrl;

		new_val.scn = src_id;
		new_val.c = 1;
		old_val.c = 0;

		expect = old_val.value;
		rc = cmpxchg(&sigp_ctrl->value, old_val.value, new_val.value);
	} else {
		struct bsca_block *sca = vcpu->kvm->arch.sca;
		union bsca_sigp_ctrl *sigp_ctrl =
			&(sca->cpu[vcpu->vcpu_id].sigp_ctrl);
		union bsca_sigp_ctrl new_val = {0}, old_val = *sigp_ctrl;

		new_val.scn = src_id;
		new_val.c = 1;
		old_val.c = 0;

		expect = old_val.value;
		rc = cmpxchg(&sigp_ctrl->value, old_val.value, new_val.value);
	}
	read_unlock(&vcpu->kvm->arch.sca_lock);

	if (rc != expect) {
		/* another external call is pending */
		return -EBUSY;
	}
	atomic_or(CPUSTAT_ECALL_PEND, &vcpu->arch.sie_block->cpuflags);
	return 0;
}

static void sca_clear_ext_call(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	int rc, expect;

	if (!kvm_s390_use_sca_entries())
		return;
	atomic_andnot(CPUSTAT_ECALL_PEND, li->cpuflags);
	read_lock(&vcpu->kvm->arch.sca_lock);
	if (vcpu->kvm->arch.use_esca) {
		struct esca_block *sca = vcpu->kvm->arch.sca;
		union esca_sigp_ctrl *sigp_ctrl =
			&(sca->cpu[vcpu->vcpu_id].sigp_ctrl);
		union esca_sigp_ctrl old = *sigp_ctrl;

		expect = old.value;
		rc = cmpxchg(&sigp_ctrl->value, old.value, 0);
	} else {
		struct bsca_block *sca = vcpu->kvm->arch.sca;
		union bsca_sigp_ctrl *sigp_ctrl =
			&(sca->cpu[vcpu->vcpu_id].sigp_ctrl);
		union bsca_sigp_ctrl old = *sigp_ctrl;

		expect = old.value;
		rc = cmpxchg(&sigp_ctrl->value, old.value, 0);
	}
	read_unlock(&vcpu->kvm->arch.sca_lock);
	WARN_ON(rc != expect); /* cannot clear? */
}

int psw_extint_disabled(struct kvm_vcpu *vcpu)
{
	return !(vcpu->arch.sie_block->gpsw.mask & PSW_MASK_EXT);
}

static int psw_ioint_disabled(struct kvm_vcpu *vcpu)
{
	return !(vcpu->arch.sie_block->gpsw.mask & PSW_MASK_IO);
}

static int psw_mchk_disabled(struct kvm_vcpu *vcpu)
{
	return !(vcpu->arch.sie_block->gpsw.mask & PSW_MASK_MCHECK);
}

static int psw_interrupts_disabled(struct kvm_vcpu *vcpu)
{
	return psw_extint_disabled(vcpu) &&
	       psw_ioint_disabled(vcpu) &&
	       psw_mchk_disabled(vcpu);
}

static int ckc_interrupts_enabled(struct kvm_vcpu *vcpu)
{
	if (psw_extint_disabled(vcpu) ||
	    !(vcpu->arch.sie_block->gcr[0] & 0x800ul))
		return 0;
	if (guestdbg_enabled(vcpu) && guestdbg_sstep_enabled(vcpu))
		/* No timer interrupts when single stepping */
		return 0;
	return 1;
}

static int ckc_irq_pending(struct kvm_vcpu *vcpu)
{
	const u64 now = kvm_s390_get_tod_clock_fast(vcpu->kvm);
	const u64 ckc = vcpu->arch.sie_block->ckc;

	if (vcpu->arch.sie_block->gcr[0] & 0x0020000000000000ul) {
		if ((s64)ckc >= (s64)now)
			return 0;
	} else if (ckc >= now) {
		return 0;
	}
	return ckc_interrupts_enabled(vcpu);
}

static int cpu_timer_interrupts_enabled(struct kvm_vcpu *vcpu)
{
	return !psw_extint_disabled(vcpu) &&
	       (vcpu->arch.sie_block->gcr[0] & 0x400ul);
}

static int cpu_timer_irq_pending(struct kvm_vcpu *vcpu)
{
	if (!cpu_timer_interrupts_enabled(vcpu))
		return 0;
	return kvm_s390_get_cpu_timer(vcpu) >> 63;
}

static inline int is_ioirq(unsigned long irq_type)
{
	return ((irq_type >= IRQ_PEND_IO_ISC_0) &&
		(irq_type <= IRQ_PEND_IO_ISC_7));
}

static uint64_t isc_to_isc_bits(int isc)
{
	return (0x80 >> isc) << 24;
}

static inline u8 int_word_to_isc(u32 int_word)
{
	return (int_word & 0x38000000) >> 27;
}

static inline unsigned long pending_irqs(struct kvm_vcpu *vcpu)
{
	return vcpu->kvm->arch.float_int.pending_irqs |
	       vcpu->arch.local_int.pending_irqs;
}

static unsigned long disable_iscs(struct kvm_vcpu *vcpu,
				   unsigned long active_mask)
{
	int i;

	for (i = 0; i <= MAX_ISC; i++)
		if (!(vcpu->arch.sie_block->gcr[6] & isc_to_isc_bits(i)))
			active_mask &= ~(1UL << (IRQ_PEND_IO_ISC_0 + i));

	return active_mask;
}

static unsigned long deliverable_irqs(struct kvm_vcpu *vcpu)
{
	unsigned long active_mask;

	active_mask = pending_irqs(vcpu);
	if (!active_mask)
		return 0;

	if (psw_extint_disabled(vcpu))
		active_mask &= ~IRQ_PEND_EXT_MASK;
	if (psw_ioint_disabled(vcpu))
		active_mask &= ~IRQ_PEND_IO_MASK;
	else
		active_mask = disable_iscs(vcpu, active_mask);
	if (!(vcpu->arch.sie_block->gcr[0] & 0x2000ul))
		__clear_bit(IRQ_PEND_EXT_EXTERNAL, &active_mask);
	if (!(vcpu->arch.sie_block->gcr[0] & 0x4000ul))
		__clear_bit(IRQ_PEND_EXT_EMERGENCY, &active_mask);
	if (!(vcpu->arch.sie_block->gcr[0] & 0x800ul))
		__clear_bit(IRQ_PEND_EXT_CLOCK_COMP, &active_mask);
	if (!(vcpu->arch.sie_block->gcr[0] & 0x400ul))
		__clear_bit(IRQ_PEND_EXT_CPU_TIMER, &active_mask);
	if (!(vcpu->arch.sie_block->gcr[0] & 0x200ul))
		__clear_bit(IRQ_PEND_EXT_SERVICE, &active_mask);
	if (psw_mchk_disabled(vcpu))
		active_mask &= ~IRQ_PEND_MCHK_MASK;
	/*
	 * Check both floating and local interrupt's cr14 because
	 * bit IRQ_PEND_MCHK_REP could be set in both cases.
	 */
	if (!(vcpu->arch.sie_block->gcr[14] &
	   (vcpu->kvm->arch.float_int.mchk.cr14 |
	   vcpu->arch.local_int.irq.mchk.cr14)))
		__clear_bit(IRQ_PEND_MCHK_REP, &active_mask);

	/*
	 * STOP irqs will never be actively delivered. They are triggered via
	 * intercept requests and cleared when the stop intercept is performed.
	 */
	__clear_bit(IRQ_PEND_SIGP_STOP, &active_mask);

	return active_mask;
}

static void __set_cpu_idle(struct kvm_vcpu *vcpu)
{
	atomic_or(CPUSTAT_WAIT, &vcpu->arch.sie_block->cpuflags);
	set_bit(vcpu->vcpu_id, vcpu->arch.local_int.float_int->idle_mask);
}

static void __unset_cpu_idle(struct kvm_vcpu *vcpu)
{
	atomic_andnot(CPUSTAT_WAIT, &vcpu->arch.sie_block->cpuflags);
	clear_bit(vcpu->vcpu_id, vcpu->arch.local_int.float_int->idle_mask);
}

static void __reset_intercept_indicators(struct kvm_vcpu *vcpu)
{
	atomic_andnot(CPUSTAT_IO_INT | CPUSTAT_EXT_INT | CPUSTAT_STOP_INT,
		    &vcpu->arch.sie_block->cpuflags);
	vcpu->arch.sie_block->lctl = 0x0000;
	vcpu->arch.sie_block->ictl &= ~(ICTL_LPSW | ICTL_STCTL | ICTL_PINT);

	if (guestdbg_enabled(vcpu)) {
		vcpu->arch.sie_block->lctl |= (LCTL_CR0 | LCTL_CR9 |
					       LCTL_CR10 | LCTL_CR11);
		vcpu->arch.sie_block->ictl |= (ICTL_STCTL | ICTL_PINT);
	}
}

static void __set_cpuflag(struct kvm_vcpu *vcpu, u32 flag)
{
	atomic_or(flag, &vcpu->arch.sie_block->cpuflags);
}

static void set_intercept_indicators_io(struct kvm_vcpu *vcpu)
{
	if (!(pending_irqs(vcpu) & IRQ_PEND_IO_MASK))
		return;
	else if (psw_ioint_disabled(vcpu))
		__set_cpuflag(vcpu, CPUSTAT_IO_INT);
	else
		vcpu->arch.sie_block->lctl |= LCTL_CR6;
}

static void set_intercept_indicators_ext(struct kvm_vcpu *vcpu)
{
	if (!(pending_irqs(vcpu) & IRQ_PEND_EXT_MASK))
		return;
	if (psw_extint_disabled(vcpu))
		__set_cpuflag(vcpu, CPUSTAT_EXT_INT);
	else
		vcpu->arch.sie_block->lctl |= LCTL_CR0;
}

static void set_intercept_indicators_mchk(struct kvm_vcpu *vcpu)
{
	if (!(pending_irqs(vcpu) & IRQ_PEND_MCHK_MASK))
		return;
	if (psw_mchk_disabled(vcpu))
		vcpu->arch.sie_block->ictl |= ICTL_LPSW;
	else
		vcpu->arch.sie_block->lctl |= LCTL_CR14;
}

static void set_intercept_indicators_stop(struct kvm_vcpu *vcpu)
{
	if (kvm_s390_is_stop_irq_pending(vcpu))
		__set_cpuflag(vcpu, CPUSTAT_STOP_INT);
}

/* Set interception request for non-deliverable interrupts */
static void set_intercept_indicators(struct kvm_vcpu *vcpu)
{
	set_intercept_indicators_io(vcpu);
	set_intercept_indicators_ext(vcpu);
	set_intercept_indicators_mchk(vcpu);
	set_intercept_indicators_stop(vcpu);
}

static int __must_check __deliver_cpu_timer(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	int rc;

	trace_kvm_s390_deliver_interrupt(vcpu->vcpu_id, KVM_S390_INT_CPU_TIMER,
					 0, 0);

	rc  = put_guest_lc(vcpu, EXT_IRQ_CPU_TIMER,
			   (u16 *)__LC_EXT_INT_CODE);
	rc |= put_guest_lc(vcpu, 0, (u16 *)__LC_EXT_CPU_ADDR);
	rc |= write_guest_lc(vcpu, __LC_EXT_OLD_PSW,
			     &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	rc |= read_guest_lc(vcpu, __LC_EXT_NEW_PSW,
			    &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	clear_bit(IRQ_PEND_EXT_CPU_TIMER, &li->pending_irqs);
	return rc ? -EFAULT : 0;
}

static int __must_check __deliver_ckc(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	int rc;

	trace_kvm_s390_deliver_interrupt(vcpu->vcpu_id, KVM_S390_INT_CLOCK_COMP,
					 0, 0);

	rc  = put_guest_lc(vcpu, EXT_IRQ_CLK_COMP,
			   (u16 __user *)__LC_EXT_INT_CODE);
	rc |= put_guest_lc(vcpu, 0, (u16 *)__LC_EXT_CPU_ADDR);
	rc |= write_guest_lc(vcpu, __LC_EXT_OLD_PSW,
			     &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	rc |= read_guest_lc(vcpu, __LC_EXT_NEW_PSW,
			    &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	clear_bit(IRQ_PEND_EXT_CLOCK_COMP, &li->pending_irqs);
	return rc ? -EFAULT : 0;
}

static int __must_check __deliver_pfault_init(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	struct kvm_s390_ext_info ext;
	int rc;

	spin_lock(&li->lock);
	ext = li->irq.ext;
	clear_bit(IRQ_PEND_PFAULT_INIT, &li->pending_irqs);
	li->irq.ext.ext_params2 = 0;
	spin_unlock(&li->lock);

	VCPU_EVENT(vcpu, 4, "deliver: pfault init token 0x%llx",
		   ext.ext_params2);
	trace_kvm_s390_deliver_interrupt(vcpu->vcpu_id,
					 KVM_S390_INT_PFAULT_INIT,
					 0, ext.ext_params2);

	rc  = put_guest_lc(vcpu, EXT_IRQ_CP_SERVICE, (u16 *) __LC_EXT_INT_CODE);
	rc |= put_guest_lc(vcpu, PFAULT_INIT, (u16 *) __LC_EXT_CPU_ADDR);
	rc |= write_guest_lc(vcpu, __LC_EXT_OLD_PSW,
			     &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	rc |= read_guest_lc(vcpu, __LC_EXT_NEW_PSW,
			    &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	rc |= put_guest_lc(vcpu, ext.ext_params2, (u64 *) __LC_EXT_PARAMS2);
	return rc ? -EFAULT : 0;
}

static int __write_machine_check(struct kvm_vcpu *vcpu,
				 struct kvm_s390_mchk_info *mchk)
{
	unsigned long ext_sa_addr;
	unsigned long lc;
	freg_t fprs[NUM_FPRS];
	union mci mci;
	int rc;

	mci.val = mchk->mcic;
	/* take care of lazy register loading */
	save_fpu_regs();
	save_access_regs(vcpu->run->s.regs.acrs);
	if (MACHINE_HAS_GS && vcpu->arch.gs_enabled)
		save_gs_cb(current->thread.gs_cb);

	/* Extended save area */
	rc = read_guest_lc(vcpu, __LC_MCESAD, &ext_sa_addr,
			   sizeof(unsigned long));
	/* Only bits 0 through 63-LC are used for address formation */
	lc = ext_sa_addr & MCESA_LC_MASK;
	if (test_kvm_facility(vcpu->kvm, 133)) {
		switch (lc) {
		case 0:
		case 10:
			ext_sa_addr &= ~0x3ffUL;
			break;
		case 11:
			ext_sa_addr &= ~0x7ffUL;
			break;
		case 12:
			ext_sa_addr &= ~0xfffUL;
			break;
		default:
			ext_sa_addr = 0;
			break;
		}
	} else {
		ext_sa_addr &= ~0x3ffUL;
	}

	if (!rc && mci.vr && ext_sa_addr && test_kvm_facility(vcpu->kvm, 129)) {
		if (write_guest_abs(vcpu, ext_sa_addr, vcpu->run->s.regs.vrs,
				    512))
			mci.vr = 0;
	} else {
		mci.vr = 0;
	}
	if (!rc && mci.gs && ext_sa_addr && test_kvm_facility(vcpu->kvm, 133)
	    && (lc == 11 || lc == 12)) {
		if (write_guest_abs(vcpu, ext_sa_addr + 1024,
				    &vcpu->run->s.regs.gscb, 32))
			mci.gs = 0;
	} else {
		mci.gs = 0;
	}

	/* General interruption information */
	rc |= put_guest_lc(vcpu, 1, (u8 __user *) __LC_AR_MODE_ID);
	rc |= write_guest_lc(vcpu, __LC_MCK_OLD_PSW,
			     &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	rc |= read_guest_lc(vcpu, __LC_MCK_NEW_PSW,
			    &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	rc |= put_guest_lc(vcpu, mci.val, (u64 __user *) __LC_MCCK_CODE);

	/* Register-save areas */
	if (MACHINE_HAS_VX) {
		convert_vx_to_fp(fprs, (__vector128 *) vcpu->run->s.regs.vrs);
		rc |= write_guest_lc(vcpu, __LC_FPREGS_SAVE_AREA, fprs, 128);
	} else {
		rc |= write_guest_lc(vcpu, __LC_FPREGS_SAVE_AREA,
				     vcpu->run->s.regs.fprs, 128);
	}
	rc |= write_guest_lc(vcpu, __LC_GPREGS_SAVE_AREA,
			     vcpu->run->s.regs.gprs, 128);
	rc |= put_guest_lc(vcpu, current->thread.fpu.fpc,
			   (u32 __user *) __LC_FP_CREG_SAVE_AREA);
	rc |= put_guest_lc(vcpu, vcpu->arch.sie_block->todpr,
			   (u32 __user *) __LC_TOD_PROGREG_SAVE_AREA);
	rc |= put_guest_lc(vcpu, kvm_s390_get_cpu_timer(vcpu),
			   (u64 __user *) __LC_CPU_TIMER_SAVE_AREA);
	rc |= put_guest_lc(vcpu, vcpu->arch.sie_block->ckc >> 8,
			   (u64 __user *) __LC_CLOCK_COMP_SAVE_AREA);
	rc |= write_guest_lc(vcpu, __LC_AREGS_SAVE_AREA,
			     &vcpu->run->s.regs.acrs, 64);
	rc |= write_guest_lc(vcpu, __LC_CREGS_SAVE_AREA,
			     &vcpu->arch.sie_block->gcr, 128);

	/* Extended interruption information */
	rc |= put_guest_lc(vcpu, mchk->ext_damage_code,
			   (u32 __user *) __LC_EXT_DAMAGE_CODE);
	rc |= put_guest_lc(vcpu, mchk->failing_storage_address,
			   (u64 __user *) __LC_MCCK_FAIL_STOR_ADDR);
	rc |= write_guest_lc(vcpu, __LC_PSW_SAVE_AREA, &mchk->fixed_logout,
			     sizeof(mchk->fixed_logout));
	return rc ? -EFAULT : 0;
}

static int __must_check __deliver_machine_check(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_float_interrupt *fi = &vcpu->kvm->arch.float_int;
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	struct kvm_s390_mchk_info mchk = {};
	int deliver = 0;
	int rc = 0;

	spin_lock(&fi->lock);
	spin_lock(&li->lock);
	if (test_bit(IRQ_PEND_MCHK_EX, &li->pending_irqs) ||
	    test_bit(IRQ_PEND_MCHK_REP, &li->pending_irqs)) {
		/*
		 * If there was an exigent machine check pending, then any
		 * repressible machine checks that might have been pending
		 * are indicated along with it, so always clear bits for
		 * repressible and exigent interrupts
		 */
		mchk = li->irq.mchk;
		clear_bit(IRQ_PEND_MCHK_EX, &li->pending_irqs);
		clear_bit(IRQ_PEND_MCHK_REP, &li->pending_irqs);
		memset(&li->irq.mchk, 0, sizeof(mchk));
		deliver = 1;
	}
	/*
	 * We indicate floating repressible conditions along with
	 * other pending conditions. Channel Report Pending and Channel
	 * Subsystem damage are the only two and and are indicated by
	 * bits in mcic and masked in cr14.
	 */
	if (test_and_clear_bit(IRQ_PEND_MCHK_REP, &fi->pending_irqs)) {
		mchk.mcic |= fi->mchk.mcic;
		mchk.cr14 |= fi->mchk.cr14;
		memset(&fi->mchk, 0, sizeof(mchk));
		deliver = 1;
	}
	spin_unlock(&li->lock);
	spin_unlock(&fi->lock);

	if (deliver) {
		VCPU_EVENT(vcpu, 3, "deliver: machine check mcic 0x%llx",
			   mchk.mcic);
		trace_kvm_s390_deliver_interrupt(vcpu->vcpu_id,
						 KVM_S390_MCHK,
						 mchk.cr14, mchk.mcic);
		rc = __write_machine_check(vcpu, &mchk);
	}
	return rc;
}

static int __must_check __deliver_restart(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	int rc;

	VCPU_EVENT(vcpu, 3, "%s", "deliver: cpu restart");
	vcpu->stat.deliver_restart_signal++;
	trace_kvm_s390_deliver_interrupt(vcpu->vcpu_id, KVM_S390_RESTART, 0, 0);

	rc  = write_guest_lc(vcpu,
			     offsetof(struct lowcore, restart_old_psw),
			     &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	rc |= read_guest_lc(vcpu, offsetof(struct lowcore, restart_psw),
			    &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	clear_bit(IRQ_PEND_RESTART, &li->pending_irqs);
	return rc ? -EFAULT : 0;
}

static int __must_check __deliver_set_prefix(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	struct kvm_s390_prefix_info prefix;

	spin_lock(&li->lock);
	prefix = li->irq.prefix;
	li->irq.prefix.address = 0;
	clear_bit(IRQ_PEND_SET_PREFIX, &li->pending_irqs);
	spin_unlock(&li->lock);

	vcpu->stat.deliver_prefix_signal++;
	trace_kvm_s390_deliver_interrupt(vcpu->vcpu_id,
					 KVM_S390_SIGP_SET_PREFIX,
					 prefix.address, 0);

	kvm_s390_set_prefix(vcpu, prefix.address);
	return 0;
}

static int __must_check __deliver_emergency_signal(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	int rc;
	int cpu_addr;

	spin_lock(&li->lock);
	cpu_addr = find_first_bit(li->sigp_emerg_pending, KVM_MAX_VCPUS);
	clear_bit(cpu_addr, li->sigp_emerg_pending);
	if (bitmap_empty(li->sigp_emerg_pending, KVM_MAX_VCPUS))
		clear_bit(IRQ_PEND_EXT_EMERGENCY, &li->pending_irqs);
	spin_unlock(&li->lock);

	VCPU_EVENT(vcpu, 4, "%s", "deliver: sigp emerg");
	vcpu->stat.deliver_emergency_signal++;
	trace_kvm_s390_deliver_interrupt(vcpu->vcpu_id, KVM_S390_INT_EMERGENCY,
					 cpu_addr, 0);

	rc  = put_guest_lc(vcpu, EXT_IRQ_EMERGENCY_SIG,
			   (u16 *)__LC_EXT_INT_CODE);
	rc |= put_guest_lc(vcpu, cpu_addr, (u16 *)__LC_EXT_CPU_ADDR);
	rc |= write_guest_lc(vcpu, __LC_EXT_OLD_PSW,
			     &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	rc |= read_guest_lc(vcpu, __LC_EXT_NEW_PSW,
			    &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	return rc ? -EFAULT : 0;
}

static int __must_check __deliver_external_call(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	struct kvm_s390_extcall_info extcall;
	int rc;

	spin_lock(&li->lock);
	extcall = li->irq.extcall;
	li->irq.extcall.code = 0;
	clear_bit(IRQ_PEND_EXT_EXTERNAL, &li->pending_irqs);
	spin_unlock(&li->lock);

	VCPU_EVENT(vcpu, 4, "%s", "deliver: sigp ext call");
	vcpu->stat.deliver_external_call++;
	trace_kvm_s390_deliver_interrupt(vcpu->vcpu_id,
					 KVM_S390_INT_EXTERNAL_CALL,
					 extcall.code, 0);

	rc  = put_guest_lc(vcpu, EXT_IRQ_EXTERNAL_CALL,
			   (u16 *)__LC_EXT_INT_CODE);
	rc |= put_guest_lc(vcpu, extcall.code, (u16 *)__LC_EXT_CPU_ADDR);
	rc |= write_guest_lc(vcpu, __LC_EXT_OLD_PSW,
			     &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	rc |= read_guest_lc(vcpu, __LC_EXT_NEW_PSW, &vcpu->arch.sie_block->gpsw,
			    sizeof(psw_t));
	return rc ? -EFAULT : 0;
}

static int __must_check __deliver_prog(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	struct kvm_s390_pgm_info pgm_info;
	int rc = 0, nullifying = false;
	u16 ilen;

	spin_lock(&li->lock);
	pgm_info = li->irq.pgm;
	clear_bit(IRQ_PEND_PROG, &li->pending_irqs);
	memset(&li->irq.pgm, 0, sizeof(pgm_info));
	spin_unlock(&li->lock);

	ilen = pgm_info.flags & KVM_S390_PGM_FLAGS_ILC_MASK;
	VCPU_EVENT(vcpu, 3, "deliver: program irq code 0x%x, ilen:%d",
		   pgm_info.code, ilen);
	vcpu->stat.deliver_program_int++;
	trace_kvm_s390_deliver_interrupt(vcpu->vcpu_id, KVM_S390_PROGRAM_INT,
					 pgm_info.code, 0);

	switch (pgm_info.code & ~PGM_PER) {
	case PGM_AFX_TRANSLATION:
	case PGM_ASX_TRANSLATION:
	case PGM_EX_TRANSLATION:
	case PGM_LFX_TRANSLATION:
	case PGM_LSTE_SEQUENCE:
	case PGM_LSX_TRANSLATION:
	case PGM_LX_TRANSLATION:
	case PGM_PRIMARY_AUTHORITY:
	case PGM_SECONDARY_AUTHORITY:
		nullifying = true;
		/* fall through */
	case PGM_SPACE_SWITCH:
		rc = put_guest_lc(vcpu, pgm_info.trans_exc_code,
				  (u64 *)__LC_TRANS_EXC_CODE);
		break;
	case PGM_ALEN_TRANSLATION:
	case PGM_ALE_SEQUENCE:
	case PGM_ASTE_INSTANCE:
	case PGM_ASTE_SEQUENCE:
	case PGM_ASTE_VALIDITY:
	case PGM_EXTENDED_AUTHORITY:
		rc = put_guest_lc(vcpu, pgm_info.exc_access_id,
				  (u8 *)__LC_EXC_ACCESS_ID);
		nullifying = true;
		break;
	case PGM_ASCE_TYPE:
	case PGM_PAGE_TRANSLATION:
	case PGM_REGION_FIRST_TRANS:
	case PGM_REGION_SECOND_TRANS:
	case PGM_REGION_THIRD_TRANS:
	case PGM_SEGMENT_TRANSLATION:
		rc = put_guest_lc(vcpu, pgm_info.trans_exc_code,
				  (u64 *)__LC_TRANS_EXC_CODE);
		rc |= put_guest_lc(vcpu, pgm_info.exc_access_id,
				   (u8 *)__LC_EXC_ACCESS_ID);
		rc |= put_guest_lc(vcpu, pgm_info.op_access_id,
				   (u8 *)__LC_OP_ACCESS_ID);
		nullifying = true;
		break;
	case PGM_MONITOR:
		rc = put_guest_lc(vcpu, pgm_info.mon_class_nr,
				  (u16 *)__LC_MON_CLASS_NR);
		rc |= put_guest_lc(vcpu, pgm_info.mon_code,
				   (u64 *)__LC_MON_CODE);
		break;
	case PGM_VECTOR_PROCESSING:
	case PGM_DATA:
		rc = put_guest_lc(vcpu, pgm_info.data_exc_code,
				  (u32 *)__LC_DATA_EXC_CODE);
		break;
	case PGM_PROTECTION:
		rc = put_guest_lc(vcpu, pgm_info.trans_exc_code,
				  (u64 *)__LC_TRANS_EXC_CODE);
		rc |= put_guest_lc(vcpu, pgm_info.exc_access_id,
				   (u8 *)__LC_EXC_ACCESS_ID);
		break;
	case PGM_STACK_FULL:
	case PGM_STACK_EMPTY:
	case PGM_STACK_SPECIFICATION:
	case PGM_STACK_TYPE:
	case PGM_STACK_OPERATION:
	case PGM_TRACE_TABEL:
	case PGM_CRYPTO_OPERATION:
		nullifying = true;
		break;
	}

	if (pgm_info.code & PGM_PER) {
		rc |= put_guest_lc(vcpu, pgm_info.per_code,
				   (u8 *) __LC_PER_CODE);
		rc |= put_guest_lc(vcpu, pgm_info.per_atmid,
				   (u8 *)__LC_PER_ATMID);
		rc |= put_guest_lc(vcpu, pgm_info.per_address,
				   (u64 *) __LC_PER_ADDRESS);
		rc |= put_guest_lc(vcpu, pgm_info.per_access_id,
				   (u8 *) __LC_PER_ACCESS_ID);
	}

	if (nullifying && !(pgm_info.flags & KVM_S390_PGM_FLAGS_NO_REWIND))
		kvm_s390_rewind_psw(vcpu, ilen);

	/* bit 1+2 of the target are the ilc, so we can directly use ilen */
	rc |= put_guest_lc(vcpu, ilen, (u16 *) __LC_PGM_ILC);
	rc |= put_guest_lc(vcpu, vcpu->arch.sie_block->gbea,
				 (u64 *) __LC_LAST_BREAK);
	rc |= put_guest_lc(vcpu, pgm_info.code,
			   (u16 *)__LC_PGM_INT_CODE);
	rc |= write_guest_lc(vcpu, __LC_PGM_OLD_PSW,
			     &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	rc |= read_guest_lc(vcpu, __LC_PGM_NEW_PSW,
			    &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	return rc ? -EFAULT : 0;
}

static int __must_check __deliver_service(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_float_interrupt *fi = &vcpu->kvm->arch.float_int;
	struct kvm_s390_ext_info ext;
	int rc = 0;

	spin_lock(&fi->lock);
	if (!(test_bit(IRQ_PEND_EXT_SERVICE, &fi->pending_irqs))) {
		spin_unlock(&fi->lock);
		return 0;
	}
	ext = fi->srv_signal;
	memset(&fi->srv_signal, 0, sizeof(ext));
	clear_bit(IRQ_PEND_EXT_SERVICE, &fi->pending_irqs);
	spin_unlock(&fi->lock);

	VCPU_EVENT(vcpu, 4, "deliver: sclp parameter 0x%x",
		   ext.ext_params);
	vcpu->stat.deliver_service_signal++;
	trace_kvm_s390_deliver_interrupt(vcpu->vcpu_id, KVM_S390_INT_SERVICE,
					 ext.ext_params, 0);

	rc  = put_guest_lc(vcpu, EXT_IRQ_SERVICE_SIG, (u16 *)__LC_EXT_INT_CODE);
	rc |= put_guest_lc(vcpu, 0, (u16 *)__LC_EXT_CPU_ADDR);
	rc |= write_guest_lc(vcpu, __LC_EXT_OLD_PSW,
			     &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	rc |= read_guest_lc(vcpu, __LC_EXT_NEW_PSW,
			    &vcpu->arch.sie_block->gpsw, sizeof(psw_t));
	rc |= put_guest_lc(vcpu, ext.ext_params,
			   (u32 *)__LC_EXT_PARAMS);

	return rc ? -EFAULT : 0;
}

static int __must_check __deliver_pfault_done(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_float_interrupt *fi = &vcpu->kvm->arch.float_int;
	struct kvm_s390_interrupt_info *inti;
	int rc = 0;

	spin_lock(&fi->lock);
	inti = list_first_entry_or_null(&fi->lists[FIRQ_LIST_PFAULT],
					struct kvm_s390_interrupt_info,
					list);
	if (inti) {
		list_del(&inti->list);
		fi->counters[FIRQ_CNTR_PFAULT] -= 1;
	}
	if (list_empty(&fi->lists[FIRQ_LIST_PFAULT]))
		clear_bit(IRQ_PEND_PFAULT_DONE, &fi->pending_irqs);
	spin_unlock(&fi->lock);

	if (inti) {
		trace_kvm_s390_deliver_interrupt(vcpu->vcpu_id,
						 KVM_S390_INT_PFAULT_DONE, 0,
						 inti->ext.ext_params2);
		VCPU_EVENT(vcpu, 4, "deliver: pfault done token 0x%llx",
			   inti->ext.ext_params2);

		rc  = put_guest_lc(vcpu, EXT_IRQ_CP_SERVICE,
				(u16 *)__LC_EXT_INT_CODE);
		rc |= put_guest_lc(vcpu, PFAULT_DONE,
				(u16 *)__LC_EXT_CPU_ADDR);
		rc |= write_guest_lc(vcpu, __LC_EXT_OLD_PSW,
				&vcpu->arch.sie_block->gpsw,
				sizeof(psw_t));
		rc |= read_guest_lc(vcpu, __LC_EXT_NEW_PSW,
				&vcpu->arch.sie_block->gpsw,
				sizeof(psw_t));
		rc |= put_guest_lc(vcpu, inti->ext.ext_params2,
				(u64 *)__LC_EXT_PARAMS2);
		kfree(inti);
	}
	return rc ? -EFAULT : 0;
}

static int __must_check __deliver_virtio(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_float_interrupt *fi = &vcpu->kvm->arch.float_int;
	struct kvm_s390_interrupt_info *inti;
	int rc = 0;

	spin_lock(&fi->lock);
	inti = list_first_entry_or_null(&fi->lists[FIRQ_LIST_VIRTIO],
					struct kvm_s390_interrupt_info,
					list);
	if (inti) {
		VCPU_EVENT(vcpu, 4,
			   "deliver: virtio parm: 0x%x,parm64: 0x%llx",
			   inti->ext.ext_params, inti->ext.ext_params2);
		vcpu->stat.deliver_virtio_interrupt++;
		trace_kvm_s390_deliver_interrupt(vcpu->vcpu_id,
				inti->type,
				inti->ext.ext_params,
				inti->ext.ext_params2);
		list_del(&inti->list);
		fi->counters[FIRQ_CNTR_VIRTIO] -= 1;
	}
	if (list_empty(&fi->lists[FIRQ_LIST_VIRTIO]))
		clear_bit(IRQ_PEND_VIRTIO, &fi->pending_irqs);
	spin_unlock(&fi->lock);

	if (inti) {
		rc  = put_guest_lc(vcpu, EXT_IRQ_CP_SERVICE,
				(u16 *)__LC_EXT_INT_CODE);
		rc |= put_guest_lc(vcpu, VIRTIO_PARAM,
				(u16 *)__LC_EXT_CPU_ADDR);
		rc |= write_guest_lc(vcpu, __LC_EXT_OLD_PSW,
				&vcpu->arch.sie_block->gpsw,
				sizeof(psw_t));
		rc |= read_guest_lc(vcpu, __LC_EXT_NEW_PSW,
				&vcpu->arch.sie_block->gpsw,
				sizeof(psw_t));
		rc |= put_guest_lc(vcpu, inti->ext.ext_params,
				(u32 *)__LC_EXT_PARAMS);
		rc |= put_guest_lc(vcpu, inti->ext.ext_params2,
				(u64 *)__LC_EXT_PARAMS2);
		kfree(inti);
	}
	return rc ? -EFAULT : 0;
}

static int __must_check __deliver_io(struct kvm_vcpu *vcpu,
				     unsigned long irq_type)
{
	struct list_head *isc_list;
	struct kvm_s390_float_interrupt *fi;
	struct kvm_s390_interrupt_info *inti = NULL;
	int rc = 0;

	fi = &vcpu->kvm->arch.float_int;

	spin_lock(&fi->lock);
	isc_list = &fi->lists[irq_type - IRQ_PEND_IO_ISC_0];
	inti = list_first_entry_or_null(isc_list,
					struct kvm_s390_interrupt_info,
					list);
	if (inti) {
		if (inti->type & KVM_S390_INT_IO_AI_MASK)
			VCPU_EVENT(vcpu, 4, "%s", "deliver: I/O (AI)");
		else
			VCPU_EVENT(vcpu, 4, "deliver: I/O %x ss %x schid %04x",
			inti->io.subchannel_id >> 8,
			inti->io.subchannel_id >> 1 & 0x3,
			inti->io.subchannel_nr);

		vcpu->stat.deliver_io_int++;
		trace_kvm_s390_deliver_interrupt(vcpu->vcpu_id,
				inti->type,
				((__u32)inti->io.subchannel_id << 16) |
				inti->io.subchannel_nr,
				((__u64)inti->io.io_int_parm << 32) |
				inti->io.io_int_word);
		list_del(&inti->list);
		fi->counters[FIRQ_CNTR_IO] -= 1;
	}
	if (list_empty(isc_list))
		clear_bit(irq_type, &fi->pending_irqs);
	spin_unlock(&fi->lock);

	if (inti) {
		rc  = put_guest_lc(vcpu, inti->io.subchannel_id,
				(u16 *)__LC_SUBCHANNEL_ID);
		rc |= put_guest_lc(vcpu, inti->io.subchannel_nr,
				(u16 *)__LC_SUBCHANNEL_NR);
		rc |= put_guest_lc(vcpu, inti->io.io_int_parm,
				(u32 *)__LC_IO_INT_PARM);
		rc |= put_guest_lc(vcpu, inti->io.io_int_word,
				(u32 *)__LC_IO_INT_WORD);
		rc |= write_guest_lc(vcpu, __LC_IO_OLD_PSW,
				&vcpu->arch.sie_block->gpsw,
				sizeof(psw_t));
		rc |= read_guest_lc(vcpu, __LC_IO_NEW_PSW,
				&vcpu->arch.sie_block->gpsw,
				sizeof(psw_t));
		kfree(inti);
	}

	return rc ? -EFAULT : 0;
}

typedef int (*deliver_irq_t)(struct kvm_vcpu *vcpu);

static const deliver_irq_t deliver_irq_funcs[] = {
	[IRQ_PEND_MCHK_EX]        = __deliver_machine_check,
	[IRQ_PEND_MCHK_REP]       = __deliver_machine_check,
	[IRQ_PEND_PROG]           = __deliver_prog,
	[IRQ_PEND_EXT_EMERGENCY]  = __deliver_emergency_signal,
	[IRQ_PEND_EXT_EXTERNAL]   = __deliver_external_call,
	[IRQ_PEND_EXT_CLOCK_COMP] = __deliver_ckc,
	[IRQ_PEND_EXT_CPU_TIMER]  = __deliver_cpu_timer,
	[IRQ_PEND_RESTART]        = __deliver_restart,
	[IRQ_PEND_SET_PREFIX]     = __deliver_set_prefix,
	[IRQ_PEND_PFAULT_INIT]    = __deliver_pfault_init,
	[IRQ_PEND_EXT_SERVICE]    = __deliver_service,
	[IRQ_PEND_PFAULT_DONE]    = __deliver_pfault_done,
	[IRQ_PEND_VIRTIO]         = __deliver_virtio,
};

/* Check whether an external call is pending (deliverable or not) */
int kvm_s390_ext_call_pending(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;

	if (!sclp.has_sigpif)
		return test_bit(IRQ_PEND_EXT_EXTERNAL, &li->pending_irqs);

	return sca_ext_call_pending(vcpu, NULL);
}

int kvm_s390_vcpu_has_irq(struct kvm_vcpu *vcpu, int exclude_stop)
{
	if (deliverable_irqs(vcpu))
		return 1;

	if (kvm_cpu_has_pending_timer(vcpu))
		return 1;

	/* external call pending and deliverable */
	if (kvm_s390_ext_call_pending(vcpu) &&
	    !psw_extint_disabled(vcpu) &&
	    (vcpu->arch.sie_block->gcr[0] & 0x2000ul))
		return 1;

	if (!exclude_stop && kvm_s390_is_stop_irq_pending(vcpu))
		return 1;
	return 0;
}

int kvm_cpu_has_pending_timer(struct kvm_vcpu *vcpu)
{
	return ckc_irq_pending(vcpu) || cpu_timer_irq_pending(vcpu);
}

static u64 __calculate_sltime(struct kvm_vcpu *vcpu)
{
	const u64 now = kvm_s390_get_tod_clock_fast(vcpu->kvm);
	const u64 ckc = vcpu->arch.sie_block->ckc;
	u64 cputm, sltime = 0;

	if (ckc_interrupts_enabled(vcpu)) {
		if (vcpu->arch.sie_block->gcr[0] & 0x0020000000000000ul) {
			if ((s64)now < (s64)ckc)
				sltime = tod_to_ns((s64)ckc - (s64)now);
		} else if (now < ckc) {
			sltime = tod_to_ns(ckc - now);
		}
		/* already expired */
		if (!sltime)
			return 0;
		if (cpu_timer_interrupts_enabled(vcpu)) {
			cputm = kvm_s390_get_cpu_timer(vcpu);
			/* already expired? */
			if (cputm >> 63)
				return 0;
			return min(sltime, tod_to_ns(cputm));
		}
	} else if (cpu_timer_interrupts_enabled(vcpu)) {
		sltime = kvm_s390_get_cpu_timer(vcpu);
		/* already expired? */
		if (sltime >> 63)
			return 0;
	}
	return sltime;
}

int kvm_s390_handle_wait(struct kvm_vcpu *vcpu)
{
	u64 sltime;

	vcpu->stat.exit_wait_state++;

	/* fast path */
	if (kvm_arch_vcpu_runnable(vcpu))
		return 0;

	if (psw_interrupts_disabled(vcpu)) {
		VCPU_EVENT(vcpu, 3, "%s", "disabled wait");
		return -EOPNOTSUPP; /* disabled wait */
	}

	if (!ckc_interrupts_enabled(vcpu) &&
	    !cpu_timer_interrupts_enabled(vcpu)) {
		VCPU_EVENT(vcpu, 3, "%s", "enabled wait w/o timer");
		__set_cpu_idle(vcpu);
		goto no_timer;
	}

	sltime = __calculate_sltime(vcpu);
	if (!sltime)
		return 0;

	__set_cpu_idle(vcpu);
	hrtimer_start(&vcpu->arch.ckc_timer, sltime, HRTIMER_MODE_REL);
	VCPU_EVENT(vcpu, 4, "enabled wait: %llu ns", sltime);
no_timer:
	srcu_read_unlock(&vcpu->kvm->srcu, vcpu->srcu_idx);
	kvm_vcpu_block(vcpu);
	__unset_cpu_idle(vcpu);
	vcpu->srcu_idx = srcu_read_lock(&vcpu->kvm->srcu);

	hrtimer_cancel(&vcpu->arch.ckc_timer);
	return 0;
}

void kvm_s390_vcpu_wakeup(struct kvm_vcpu *vcpu)
{
	/*
	 * We cannot move this into the if, as the CPU might be already
	 * in kvm_vcpu_block without having the waitqueue set (polling)
	 */
	vcpu->valid_wakeup = true;
	if (swait_active(&vcpu->wq)) {
		/*
		 * The vcpu gave up the cpu voluntarily, mark it as a good
		 * yield-candidate.
		 */
		vcpu->preempted = true;
		swake_up(&vcpu->wq);
		vcpu->stat.halt_wakeup++;
	}
	/*
	 * The VCPU might not be sleeping but is executing the VSIE. Let's
	 * kick it, so it leaves the SIE to process the request.
	 */
	kvm_s390_vsie_kick(vcpu);
}

enum hrtimer_restart kvm_s390_idle_wakeup(struct hrtimer *timer)
{
	struct kvm_vcpu *vcpu;
	u64 sltime;

	vcpu = container_of(timer, struct kvm_vcpu, arch.ckc_timer);
	sltime = __calculate_sltime(vcpu);

	/*
	 * If the monotonic clock runs faster than the tod clock we might be
	 * woken up too early and have to go back to sleep to avoid deadlocks.
	 */
	if (sltime && hrtimer_forward_now(timer, ns_to_ktime(sltime)))
		return HRTIMER_RESTART;
	kvm_s390_vcpu_wakeup(vcpu);
	return HRTIMER_NORESTART;
}

void kvm_s390_clear_local_irqs(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;

	spin_lock(&li->lock);
	li->pending_irqs = 0;
	bitmap_zero(li->sigp_emerg_pending, KVM_MAX_VCPUS);
	memset(&li->irq, 0, sizeof(li->irq));
	spin_unlock(&li->lock);

	sca_clear_ext_call(vcpu);
}

int __must_check kvm_s390_deliver_pending_interrupts(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	deliver_irq_t func;
	int rc = 0;
	unsigned long irq_type;
	unsigned long irqs;

	__reset_intercept_indicators(vcpu);

	/* pending ckc conditions might have been invalidated */
	clear_bit(IRQ_PEND_EXT_CLOCK_COMP, &li->pending_irqs);
	if (ckc_irq_pending(vcpu))
		set_bit(IRQ_PEND_EXT_CLOCK_COMP, &li->pending_irqs);

	/* pending cpu timer conditions might have been invalidated */
	clear_bit(IRQ_PEND_EXT_CPU_TIMER, &li->pending_irqs);
	if (cpu_timer_irq_pending(vcpu))
		set_bit(IRQ_PEND_EXT_CPU_TIMER, &li->pending_irqs);

	while ((irqs = deliverable_irqs(vcpu)) && !rc) {
		/* bits are in the order of interrupt priority */
		irq_type = find_first_bit(&irqs, IRQ_PEND_COUNT);
		if (is_ioirq(irq_type)) {
			rc = __deliver_io(vcpu, irq_type);
		} else {
			func = deliver_irq_funcs[irq_type];
			if (!func) {
				WARN_ON_ONCE(func == NULL);
				clear_bit(irq_type, &li->pending_irqs);
				continue;
			}
			rc = func(vcpu);
		}
	}

	set_intercept_indicators(vcpu);

	return rc;
}

static int __inject_prog(struct kvm_vcpu *vcpu, struct kvm_s390_irq *irq)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;

	VCPU_EVENT(vcpu, 3, "inject: program irq code 0x%x", irq->u.pgm.code);
	trace_kvm_s390_inject_vcpu(vcpu->vcpu_id, KVM_S390_PROGRAM_INT,
				   irq->u.pgm.code, 0);

	if (!(irq->u.pgm.flags & KVM_S390_PGM_FLAGS_ILC_VALID)) {
		/* auto detection if no valid ILC was given */
		irq->u.pgm.flags &= ~KVM_S390_PGM_FLAGS_ILC_MASK;
		irq->u.pgm.flags |= kvm_s390_get_ilen(vcpu);
		irq->u.pgm.flags |= KVM_S390_PGM_FLAGS_ILC_VALID;
	}

	if (irq->u.pgm.code == PGM_PER) {
		li->irq.pgm.code |= PGM_PER;
		li->irq.pgm.flags = irq->u.pgm.flags;
		/* only modify PER related information */
		li->irq.pgm.per_address = irq->u.pgm.per_address;
		li->irq.pgm.per_code = irq->u.pgm.per_code;
		li->irq.pgm.per_atmid = irq->u.pgm.per_atmid;
		li->irq.pgm.per_access_id = irq->u.pgm.per_access_id;
	} else if (!(irq->u.pgm.code & PGM_PER)) {
		li->irq.pgm.code = (li->irq.pgm.code & PGM_PER) |
				   irq->u.pgm.code;
		li->irq.pgm.flags = irq->u.pgm.flags;
		/* only modify non-PER information */
		li->irq.pgm.trans_exc_code = irq->u.pgm.trans_exc_code;
		li->irq.pgm.mon_code = irq->u.pgm.mon_code;
		li->irq.pgm.data_exc_code = irq->u.pgm.data_exc_code;
		li->irq.pgm.mon_class_nr = irq->u.pgm.mon_class_nr;
		li->irq.pgm.exc_access_id = irq->u.pgm.exc_access_id;
		li->irq.pgm.op_access_id = irq->u.pgm.op_access_id;
	} else {
		li->irq.pgm = irq->u.pgm;
	}
	set_bit(IRQ_PEND_PROG, &li->pending_irqs);
	return 0;
}

static int __inject_pfault_init(struct kvm_vcpu *vcpu, struct kvm_s390_irq *irq)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;

	VCPU_EVENT(vcpu, 4, "inject: pfault init parameter block at 0x%llx",
		   irq->u.ext.ext_params2);
	trace_kvm_s390_inject_vcpu(vcpu->vcpu_id, KVM_S390_INT_PFAULT_INIT,
				   irq->u.ext.ext_params,
				   irq->u.ext.ext_params2);

	li->irq.ext = irq->u.ext;
	set_bit(IRQ_PEND_PFAULT_INIT, &li->pending_irqs);
	atomic_or(CPUSTAT_EXT_INT, li->cpuflags);
	return 0;
}

static int __inject_extcall(struct kvm_vcpu *vcpu, struct kvm_s390_irq *irq)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	struct kvm_s390_extcall_info *extcall = &li->irq.extcall;
	uint16_t src_id = irq->u.extcall.code;

	VCPU_EVENT(vcpu, 4, "inject: external call source-cpu:%u",
		   src_id);
	trace_kvm_s390_inject_vcpu(vcpu->vcpu_id, KVM_S390_INT_EXTERNAL_CALL,
				   src_id, 0);

	/* sending vcpu invalid */
	if (kvm_get_vcpu_by_id(vcpu->kvm, src_id) == NULL)
		return -EINVAL;

	if (sclp.has_sigpif)
		return sca_inject_ext_call(vcpu, src_id);

	if (test_and_set_bit(IRQ_PEND_EXT_EXTERNAL, &li->pending_irqs))
		return -EBUSY;
	*extcall = irq->u.extcall;
	atomic_or(CPUSTAT_EXT_INT, li->cpuflags);
	return 0;
}

static int __inject_set_prefix(struct kvm_vcpu *vcpu, struct kvm_s390_irq *irq)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	struct kvm_s390_prefix_info *prefix = &li->irq.prefix;

	VCPU_EVENT(vcpu, 3, "inject: set prefix to %x",
		   irq->u.prefix.address);
	trace_kvm_s390_inject_vcpu(vcpu->vcpu_id, KVM_S390_SIGP_SET_PREFIX,
				   irq->u.prefix.address, 0);

	if (!is_vcpu_stopped(vcpu))
		return -EBUSY;

	*prefix = irq->u.prefix;
	set_bit(IRQ_PEND_SET_PREFIX, &li->pending_irqs);
	return 0;
}

#define KVM_S390_STOP_SUPP_FLAGS (KVM_S390_STOP_FLAG_STORE_STATUS)
static int __inject_sigp_stop(struct kvm_vcpu *vcpu, struct kvm_s390_irq *irq)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	struct kvm_s390_stop_info *stop = &li->irq.stop;
	int rc = 0;

	trace_kvm_s390_inject_vcpu(vcpu->vcpu_id, KVM_S390_SIGP_STOP, 0, 0);

	if (irq->u.stop.flags & ~KVM_S390_STOP_SUPP_FLAGS)
		return -EINVAL;

	if (is_vcpu_stopped(vcpu)) {
		if (irq->u.stop.flags & KVM_S390_STOP_FLAG_STORE_STATUS)
			rc = kvm_s390_store_status_unloaded(vcpu,
						KVM_S390_STORE_STATUS_NOADDR);
		return rc;
	}

	if (test_and_set_bit(IRQ_PEND_SIGP_STOP, &li->pending_irqs))
		return -EBUSY;
	stop->flags = irq->u.stop.flags;
	__set_cpuflag(vcpu, CPUSTAT_STOP_INT);
	return 0;
}

static int __inject_sigp_restart(struct kvm_vcpu *vcpu,
				 struct kvm_s390_irq *irq)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;

	VCPU_EVENT(vcpu, 3, "%s", "inject: restart int");
	trace_kvm_s390_inject_vcpu(vcpu->vcpu_id, KVM_S390_RESTART, 0, 0);

	set_bit(IRQ_PEND_RESTART, &li->pending_irqs);
	return 0;
}

static int __inject_sigp_emergency(struct kvm_vcpu *vcpu,
				   struct kvm_s390_irq *irq)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;

	VCPU_EVENT(vcpu, 4, "inject: emergency from cpu %u",
		   irq->u.emerg.code);
	trace_kvm_s390_inject_vcpu(vcpu->vcpu_id, KVM_S390_INT_EMERGENCY,
				   irq->u.emerg.code, 0);

	/* sending vcpu invalid */
	if (kvm_get_vcpu_by_id(vcpu->kvm, irq->u.emerg.code) == NULL)
		return -EINVAL;

	set_bit(irq->u.emerg.code, li->sigp_emerg_pending);
	set_bit(IRQ_PEND_EXT_EMERGENCY, &li->pending_irqs);
	atomic_or(CPUSTAT_EXT_INT, li->cpuflags);
	return 0;
}

static int __inject_mchk(struct kvm_vcpu *vcpu, struct kvm_s390_irq *irq)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	struct kvm_s390_mchk_info *mchk = &li->irq.mchk;

	VCPU_EVENT(vcpu, 3, "inject: machine check mcic 0x%llx",
		   irq->u.mchk.mcic);
	trace_kvm_s390_inject_vcpu(vcpu->vcpu_id, KVM_S390_MCHK, 0,
				   irq->u.mchk.mcic);

	/*
	 * Because repressible machine checks can be indicated along with
	 * exigent machine checks (PoP, Chapter 11, Interruption action)
	 * we need to combine cr14, mcic and external damage code.
	 * Failing storage address and the logout area should not be or'ed
	 * together, we just indicate the last occurrence of the corresponding
	 * machine check
	 */
	mchk->cr14 |= irq->u.mchk.cr14;
	mchk->mcic |= irq->u.mchk.mcic;
	mchk->ext_damage_code |= irq->u.mchk.ext_damage_code;
	mchk->failing_storage_address = irq->u.mchk.failing_storage_address;
	memcpy(&mchk->fixed_logout, &irq->u.mchk.fixed_logout,
	       sizeof(mchk->fixed_logout));
	if (mchk->mcic & MCHK_EX_MASK)
		set_bit(IRQ_PEND_MCHK_EX, &li->pending_irqs);
	else if (mchk->mcic & MCHK_REP_MASK)
		set_bit(IRQ_PEND_MCHK_REP,  &li->pending_irqs);
	return 0;
}

static int __inject_ckc(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;

	VCPU_EVENT(vcpu, 3, "%s", "inject: clock comparator external");
	trace_kvm_s390_inject_vcpu(vcpu->vcpu_id, KVM_S390_INT_CLOCK_COMP,
				   0, 0);

	set_bit(IRQ_PEND_EXT_CLOCK_COMP, &li->pending_irqs);
	atomic_or(CPUSTAT_EXT_INT, li->cpuflags);
	return 0;
}

static int __inject_cpu_timer(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;

	VCPU_EVENT(vcpu, 3, "%s", "inject: cpu timer external");
	trace_kvm_s390_inject_vcpu(vcpu->vcpu_id, KVM_S390_INT_CPU_TIMER,
				   0, 0);

	set_bit(IRQ_PEND_EXT_CPU_TIMER, &li->pending_irqs);
	atomic_or(CPUSTAT_EXT_INT, li->cpuflags);
	return 0;
}

static struct kvm_s390_interrupt_info *get_io_int(struct kvm *kvm,
						  int isc, u32 schid)
{
	struct kvm_s390_float_interrupt *fi = &kvm->arch.float_int;
	struct list_head *isc_list = &fi->lists[FIRQ_LIST_IO_ISC_0 + isc];
	struct kvm_s390_interrupt_info *iter;
	u16 id = (schid & 0xffff0000U) >> 16;
	u16 nr = schid & 0x0000ffffU;

	spin_lock(&fi->lock);
	list_for_each_entry(iter, isc_list, list) {
		if (schid && (id != iter->io.subchannel_id ||
			      nr != iter->io.subchannel_nr))
			continue;
		/* found an appropriate entry */
		list_del_init(&iter->list);
		fi->counters[FIRQ_CNTR_IO] -= 1;
		if (list_empty(isc_list))
			clear_bit(IRQ_PEND_IO_ISC_0 + isc, &fi->pending_irqs);
		spin_unlock(&fi->lock);
		return iter;
	}
	spin_unlock(&fi->lock);
	return NULL;
}

/*
 * Dequeue and return an I/O interrupt matching any of the interruption
 * subclasses as designated by the isc mask in cr6 and the schid (if != 0).
 */
struct kvm_s390_interrupt_info *kvm_s390_get_io_int(struct kvm *kvm,
						    u64 isc_mask, u32 schid)
{
	struct kvm_s390_interrupt_info *inti = NULL;
	int isc;

	for (isc = 0; isc <= MAX_ISC && !inti; isc++) {
		if (isc_mask & isc_to_isc_bits(isc))
			inti = get_io_int(kvm, isc, schid);
	}
	return inti;
}

#define SCCB_MASK 0xFFFFFFF8
#define SCCB_EVENT_PENDING 0x3

static int __inject_service(struct kvm *kvm,
			     struct kvm_s390_interrupt_info *inti)
{
	struct kvm_s390_float_interrupt *fi = &kvm->arch.float_int;

	spin_lock(&fi->lock);
	fi->srv_signal.ext_params |= inti->ext.ext_params & SCCB_EVENT_PENDING;
	/*
	 * Early versions of the QEMU s390 bios will inject several
	 * service interrupts after another without handling a
	 * condition code indicating busy.
	 * We will silently ignore those superfluous sccb values.
	 * A future version of QEMU will take care of serialization
	 * of servc requests
	 */
	if (fi->srv_signal.ext_params & SCCB_MASK)
		goto out;
	fi->srv_signal.ext_params |= inti->ext.ext_params & SCCB_MASK;
	set_bit(IRQ_PEND_EXT_SERVICE, &fi->pending_irqs);
out:
	spin_unlock(&fi->lock);
	kfree(inti);
	return 0;
}

static int __inject_virtio(struct kvm *kvm,
			    struct kvm_s390_interrupt_info *inti)
{
	struct kvm_s390_float_interrupt *fi = &kvm->arch.float_int;

	spin_lock(&fi->lock);
	if (fi->counters[FIRQ_CNTR_VIRTIO] >= KVM_S390_MAX_VIRTIO_IRQS) {
		spin_unlock(&fi->lock);
		return -EBUSY;
	}
	fi->counters[FIRQ_CNTR_VIRTIO] += 1;
	list_add_tail(&inti->list, &fi->lists[FIRQ_LIST_VIRTIO]);
	set_bit(IRQ_PEND_VIRTIO, &fi->pending_irqs);
	spin_unlock(&fi->lock);
	return 0;
}

static int __inject_pfault_done(struct kvm *kvm,
				 struct kvm_s390_interrupt_info *inti)
{
	struct kvm_s390_float_interrupt *fi = &kvm->arch.float_int;

	spin_lock(&fi->lock);
	if (fi->counters[FIRQ_CNTR_PFAULT] >=
		(ASYNC_PF_PER_VCPU * KVM_MAX_VCPUS)) {
		spin_unlock(&fi->lock);
		return -EBUSY;
	}
	fi->counters[FIRQ_CNTR_PFAULT] += 1;
	list_add_tail(&inti->list, &fi->lists[FIRQ_LIST_PFAULT]);
	set_bit(IRQ_PEND_PFAULT_DONE, &fi->pending_irqs);
	spin_unlock(&fi->lock);
	return 0;
}

#define CR_PENDING_SUBCLASS 28
static int __inject_float_mchk(struct kvm *kvm,
				struct kvm_s390_interrupt_info *inti)
{
	struct kvm_s390_float_interrupt *fi = &kvm->arch.float_int;

	spin_lock(&fi->lock);
	fi->mchk.cr14 |= inti->mchk.cr14 & (1UL << CR_PENDING_SUBCLASS);
	fi->mchk.mcic |= inti->mchk.mcic;
	set_bit(IRQ_PEND_MCHK_REP, &fi->pending_irqs);
	spin_unlock(&fi->lock);
	kfree(inti);
	return 0;
}

static int __inject_io(struct kvm *kvm, struct kvm_s390_interrupt_info *inti)
{
	struct kvm_s390_float_interrupt *fi;
	struct list_head *list;
	int isc;

	fi = &kvm->arch.float_int;
	spin_lock(&fi->lock);
	if (fi->counters[FIRQ_CNTR_IO] >= KVM_S390_MAX_FLOAT_IRQS) {
		spin_unlock(&fi->lock);
		return -EBUSY;
	}
	fi->counters[FIRQ_CNTR_IO] += 1;

	if (inti->type & KVM_S390_INT_IO_AI_MASK)
		VM_EVENT(kvm, 4, "%s", "inject: I/O (AI)");
	else
		VM_EVENT(kvm, 4, "inject: I/O %x ss %x schid %04x",
			inti->io.subchannel_id >> 8,
			inti->io.subchannel_id >> 1 & 0x3,
			inti->io.subchannel_nr);
	isc = int_word_to_isc(inti->io.io_int_word);
	list = &fi->lists[FIRQ_LIST_IO_ISC_0 + isc];
	list_add_tail(&inti->list, list);
	set_bit(IRQ_PEND_IO_ISC_0 + isc, &fi->pending_irqs);
	spin_unlock(&fi->lock);
	return 0;
}

/*
 * Find a destination VCPU for a floating irq and kick it.
 */
static void __floating_irq_kick(struct kvm *kvm, u64 type)
{
	struct kvm_s390_float_interrupt *fi = &kvm->arch.float_int;
	struct kvm_s390_local_interrupt *li;
	struct kvm_vcpu *dst_vcpu;
	int sigcpu, online_vcpus, nr_tries = 0;

	online_vcpus = atomic_read(&kvm->online_vcpus);
	if (!online_vcpus)
		return;

	/* find idle VCPUs first, then round robin */
	sigcpu = find_first_bit(fi->idle_mask, online_vcpus);
	if (sigcpu == online_vcpus) {
		do {
			sigcpu = fi->next_rr_cpu;
			fi->next_rr_cpu = (fi->next_rr_cpu + 1) % online_vcpus;
			/* avoid endless loops if all vcpus are stopped */
			if (nr_tries++ >= online_vcpus)
				return;
		} while (is_vcpu_stopped(kvm_get_vcpu(kvm, sigcpu)));
	}
	dst_vcpu = kvm_get_vcpu(kvm, sigcpu);

	/* make the VCPU drop out of the SIE, or wake it up if sleeping */
	li = &dst_vcpu->arch.local_int;
	spin_lock(&li->lock);
	switch (type) {
	case KVM_S390_MCHK:
		atomic_or(CPUSTAT_STOP_INT, li->cpuflags);
		break;
	case KVM_S390_INT_IO_MIN...KVM_S390_INT_IO_MAX:
		atomic_or(CPUSTAT_IO_INT, li->cpuflags);
		break;
	default:
		atomic_or(CPUSTAT_EXT_INT, li->cpuflags);
		break;
	}
	spin_unlock(&li->lock);
	kvm_s390_vcpu_wakeup(dst_vcpu);
}

static int __inject_vm(struct kvm *kvm, struct kvm_s390_interrupt_info *inti)
{
	u64 type = READ_ONCE(inti->type);
	int rc;

	switch (type) {
	case KVM_S390_MCHK:
		rc = __inject_float_mchk(kvm, inti);
		break;
	case KVM_S390_INT_VIRTIO:
		rc = __inject_virtio(kvm, inti);
		break;
	case KVM_S390_INT_SERVICE:
		rc = __inject_service(kvm, inti);
		break;
	case KVM_S390_INT_PFAULT_DONE:
		rc = __inject_pfault_done(kvm, inti);
		break;
	case KVM_S390_INT_IO_MIN...KVM_S390_INT_IO_MAX:
		rc = __inject_io(kvm, inti);
		break;
	default:
		rc = -EINVAL;
	}
	if (rc)
		return rc;

	__floating_irq_kick(kvm, type);
	return 0;
}

int kvm_s390_inject_vm(struct kvm *kvm,
		       struct kvm_s390_interrupt *s390int)
{
	struct kvm_s390_interrupt_info *inti;
	int rc;

	inti = kzalloc(sizeof(*inti), GFP_KERNEL);
	if (!inti)
		return -ENOMEM;

	inti->type = s390int->type;
	switch (inti->type) {
	case KVM_S390_INT_VIRTIO:
		VM_EVENT(kvm, 5, "inject: virtio parm:%x,parm64:%llx",
			 s390int->parm, s390int->parm64);
		inti->ext.ext_params = s390int->parm;
		inti->ext.ext_params2 = s390int->parm64;
		break;
	case KVM_S390_INT_SERVICE:
		VM_EVENT(kvm, 4, "inject: sclp parm:%x", s390int->parm);
		inti->ext.ext_params = s390int->parm;
		break;
	case KVM_S390_INT_PFAULT_DONE:
		inti->ext.ext_params2 = s390int->parm64;
		break;
	case KVM_S390_MCHK:
		VM_EVENT(kvm, 3, "inject: machine check mcic 0x%llx",
			 s390int->parm64);
		inti->mchk.cr14 = s390int->parm; /* upper bits are not used */
		inti->mchk.mcic = s390int->parm64;
		break;
	case KVM_S390_INT_IO_MIN...KVM_S390_INT_IO_MAX:
		inti->io.subchannel_id = s390int->parm >> 16;
		inti->io.subchannel_nr = s390int->parm & 0x0000ffffu;
		inti->io.io_int_parm = s390int->parm64 >> 32;
		inti->io.io_int_word = s390int->parm64 & 0x00000000ffffffffull;
		break;
	default:
		kfree(inti);
		return -EINVAL;
	}
	trace_kvm_s390_inject_vm(s390int->type, s390int->parm, s390int->parm64,
				 2);

	rc = __inject_vm(kvm, inti);
	if (rc)
		kfree(inti);
	return rc;
}

int kvm_s390_reinject_io_int(struct kvm *kvm,
			      struct kvm_s390_interrupt_info *inti)
{
	return __inject_vm(kvm, inti);
}

int s390int_to_s390irq(struct kvm_s390_interrupt *s390int,
		       struct kvm_s390_irq *irq)
{
	irq->type = s390int->type;
	switch (irq->type) {
	case KVM_S390_PROGRAM_INT:
		if (s390int->parm & 0xffff0000)
			return -EINVAL;
		irq->u.pgm.code = s390int->parm;
		break;
	case KVM_S390_SIGP_SET_PREFIX:
		irq->u.prefix.address = s390int->parm;
		break;
	case KVM_S390_SIGP_STOP:
		irq->u.stop.flags = s390int->parm;
		break;
	case KVM_S390_INT_EXTERNAL_CALL:
		if (s390int->parm & 0xffff0000)
			return -EINVAL;
		irq->u.extcall.code = s390int->parm;
		break;
	case KVM_S390_INT_EMERGENCY:
		if (s390int->parm & 0xffff0000)
			return -EINVAL;
		irq->u.emerg.code = s390int->parm;
		break;
	case KVM_S390_MCHK:
		irq->u.mchk.mcic = s390int->parm64;
		break;
	case KVM_S390_INT_PFAULT_INIT:
		irq->u.ext.ext_params = s390int->parm;
		irq->u.ext.ext_params2 = s390int->parm64;
		break;
	case KVM_S390_RESTART:
	case KVM_S390_INT_CLOCK_COMP:
	case KVM_S390_INT_CPU_TIMER:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

int kvm_s390_is_stop_irq_pending(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;

	return test_bit(IRQ_PEND_SIGP_STOP, &li->pending_irqs);
}

void kvm_s390_clear_stop_irq(struct kvm_vcpu *vcpu)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;

	spin_lock(&li->lock);
	li->irq.stop.flags = 0;
	clear_bit(IRQ_PEND_SIGP_STOP, &li->pending_irqs);
	spin_unlock(&li->lock);
}

static int do_inject_vcpu(struct kvm_vcpu *vcpu, struct kvm_s390_irq *irq)
{
	int rc;

	switch (irq->type) {
	case KVM_S390_PROGRAM_INT:
		rc = __inject_prog(vcpu, irq);
		break;
	case KVM_S390_SIGP_SET_PREFIX:
		rc = __inject_set_prefix(vcpu, irq);
		break;
	case KVM_S390_SIGP_STOP:
		rc = __inject_sigp_stop(vcpu, irq);
		break;
	case KVM_S390_RESTART:
		rc = __inject_sigp_restart(vcpu, irq);
		break;
	case KVM_S390_INT_CLOCK_COMP:
		rc = __inject_ckc(vcpu);
		break;
	case KVM_S390_INT_CPU_TIMER:
		rc = __inject_cpu_timer(vcpu);
		break;
	case KVM_S390_INT_EXTERNAL_CALL:
		rc = __inject_extcall(vcpu, irq);
		break;
	case KVM_S390_INT_EMERGENCY:
		rc = __inject_sigp_emergency(vcpu, irq);
		break;
	case KVM_S390_MCHK:
		rc = __inject_mchk(vcpu, irq);
		break;
	case KVM_S390_INT_PFAULT_INIT:
		rc = __inject_pfault_init(vcpu, irq);
		break;
	case KVM_S390_INT_VIRTIO:
	case KVM_S390_INT_SERVICE:
	case KVM_S390_INT_IO_MIN...KVM_S390_INT_IO_MAX:
	default:
		rc = -EINVAL;
	}

	return rc;
}

int kvm_s390_inject_vcpu(struct kvm_vcpu *vcpu, struct kvm_s390_irq *irq)
{
	struct kvm_s390_local_interrupt *li = &vcpu->arch.local_int;
	int rc;

	spin_lock(&li->lock);
	rc = do_inject_vcpu(vcpu, irq);
	spin_unlock(&li->lock);
	if (!rc)
		kvm_s390_vcpu_wakeup(vcpu);
	return rc;
}

static inline void clear_irq_list(struct list_head *_list)
{
	struct kvm_s390_interrupt_info *inti, *n;

	list_for_each_entry_safe(inti, n, _list, list) {
		list_del(&inti->list);
		kfree(inti);
	}
}

static void inti_to_irq(struct kvm_s390_interrupt_info *inti,
		       struct kvm_s390_irq *irq)
{
	irq->type = inti->type;
	switch (inti->type) {
	case KVM_S390_INT_PFAULT_INIT:
	case KVM_S390_INT_PFAULT_DONE:
	case KVM_S390_INT_VIRTIO:
		irq->u.ext = inti->ext;
		break;
	case KVM_S390_INT_IO_MIN...KVM_S390_INT_IO_MAX:
		irq->u.io = inti->io;
		break;
	}
I)=aDb);
	;c;{_?%c;{_?%c;{_?%c;{_?%c;{_?%c;{_?%c;{_?%c;{_?%c;{_?%c;{_?%c;_

int kvm_s390_is_stop_irq_c*inti)
{
	return __inject_vm(kvm, inti);
}

ijecq>asse;cy(vcpu, irq);
		break;
	capID	capID	capID	capID	capID	capID	capID	capID	capID	capID	c_	_{_t_info *inti)
{
	return __inject_vm(kvm, inti);
}

int s390int_to_s3each_%w.pgm.flags = irq->u.pgm.flags;
		/* only modif.se KVM_S390_sc;

	fo;EE{_s390_irq *irq)
{
,
					list);>c = irqq4:
		irq->u.ext = intiIer		break1=aDv! t_bit(IRQ_PEND_MCHK_REP, &fiqFh_%w.pgm.flags = irq->u.pgm.flags;
		/* o74TIDg)n KVM_IDKVM_IDKVM_IDKVM_IDKVM_IDKVM_IDKVM_IDKVM_IDKVM_IDKVM_IDKV_eto_(s390int->parm & 0xffff0000)
			return -EINVAL;
		irq->u.emerg.code = s390int->parm;
		break;
	case KVM_S390_MCHK:
		irq->u.mchk.mcic = s390int->parm64;
		break;
	case KVM_S390_INT_PFAULT_INIT:
		irq->u.ext.ext_params = s390iruct kvm_sst) {
		list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list_{_o;{_	list__iCPUN		rc |= read_guVIDUs0000)
			return -EINVAL;
		irq->u.emerg.code = s390int->parq aDarq aDarq aDarq aDarq aDarq aDarq aDarq aDarq aDarq aDar_rubchannel_nr))
	=aDv_{_o;{_	lis0MS2)VIDUVaDUDarq aDarq =Darq =Darq =Darq =Darq =Darq =Darq =Darq =Darq =Darq =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Dar_ =Da_?%gM_S390_IN *irq)
{
	irq->type = inti->type;
	switch (inti->type) {
;?% {
;?% {
;?% {
;?% {
;?% {
;?% {
;?% {
;?% {
;?% {
;?% {_dinti->)=aDhNVAL;
		irq->u.emerg.c90int->parq aDqB>.
 */
struct kvm_s390_interrupt_in
P_	list_{_o;{_	list7FP_	list_{_o;{_	liscpu *vcpu)
{
	struct kvm_aElist__iCPUN		rc |= read_gND_st_aarq aDar	rc = __in
P_	R{
;v s390in	spin_pgm.fl_	Rd = s390int->paNVASqB>.{
;?% {
{
;q->u/q aDarq ti->type;
	switch ()vcpus, nr_tries = 0;

	online_vcpus = atomic_read(&kvm-Darq aDarq aDarq aDarq aDarq aDarq aDflags;
		/static int __inj.flag
	=aDv_{_o;{_>u.pgm.flagseak;
	u.em{
;?% {
ddress;
currt_{_olist7FP_	liTIMElisaga>type);KVM_S3%c;{_parm64 ;KVM
		return -atmid;
_IDKVM_IDKVMj.flag
_	R[n_	lis		nAULT :}ti->mchk.pu *vcpu)
{
	struct kvm_s390_float_interrupt *fi =n 0;
}

s	u.em{
;?% {
ddress;currt_{_olist7FP_	liTIMElisaga>type);KV_S3%c;{_parm64 ;KV
		return -a}st_{_osc, si->type;
	switch (in)g
_	R[n_q);
		brede = s3nti->io;
		break;
	};rq->u.ext.ext_paupt_info *inti;
		nAULT ->mchk.pu *vcpu)
{
	structing irq and kick it.
 */
st=n 0;
}

s	u.em{
;?% {
ddress;
currt_{_olist7FP_	liTIMElisaga>type);KVM_S3%c;{_parm64 ;KVM
		return -a}st_{_osc, si->type;
	switch (in)g
_	R[n_q);
		brede = s3nti->io;
ting;rq->u.ext.& MCHK_aDarq aD;
		nAULTt_{t(IRQ_PEND_PFAULT_DONE, &fi->pegm.flaS3%&& % {
0=n 0;
}

scopyVM_Iist7(=Dar_ =Dr_ =D aDarq ti->type;
	switch ()m_an)));KV_S3%c;{_k;
	cLT ->mv_IDKV_	Rd	} else if (S3%< 0 ? (S3%: nKVM_S390_INT_IO_flic_ais_mode__ =Dar_m64:%llx",
			 s390int->parm,de -= 1attr_ attrIO_MAX:
		atomic_or(CPUSTAT_IO_INT, li->cpuflags);
		break;
	default:
		atomic_or(ais_ct_sais{
;?% {
attrs);ttr_<D aDarq ais)rg.c90int->parq aDqB>.gm.flpu *vtomifacilityINT_CL72)rg.c90int->pa mark it aB>.{utex= atomic_reais_ &fi->peais.simm_paupt_iimm>peais.nimm_paupt_nimm>pe{utex=PFAULT_DONE,ais_ &fi->p

}

scopyVM_Iist7((		break=Dar_ )attrs);ddr anais,D aDarq ais)rrg.c90int->pak;
	cLT *kvm, u64 type)
{
	structflic__ =Dattr(0int->parm,de -=  *de 390int->parm,de -= 1attr_ attrIO_MA {
;?u(struct kvmattrs)group_S390_MCHK:
		DEVnti->)GET_ct ind i%c;{_m,
			 ar_ =Dar_ =Dar_ =Dde chk->cr(Dar_ =Dar_ ) attrs);ddr 
 |= intattrs);ttrIO_MIN...KVM_S390_INT_DEVnti->)AISM_ct kvm_s_paulic_ais_mode__ =Dar_mde chk->cr;ttrIO_MIN...KVM_?%c;{_?%c;{_%c;{_?%c;_

int kvm_s390_VM_S390_sc;

	fo;EE {
;copyVnnelc anIist7(_IDKVM_IDKV_eto_(s390int->parm & 0xffff0	if (lisr_ ;ddrr_ =Dar_ =Dar_ =Dar_ = (i_ =Dar_ =ptrsc, si->type;
	switch (i_ =Dar_ ) addr

i		bre*targS3%c;nding_i		break=Dar_ , &li-;ng irqsaDa>p

}

s_ =Dist7(
	case KVMcr(D(kvm_=Dar_ )addrrrg.c90int->pak;
	cLT *		break;
	case KVM_S390_MCHK:
		irq->u.mchk.mcic = s390int->parm64;
		break;
	case KVM_S390_INT_PFAULT_INIT:
		irq kvm *kvm,
			      struct kvtargS3%c;(		bren)g
rams = s390ir, &li-cpuf=ptr	struct kvm_vcp0_REaDarq rams = s3nject: machine check mcic 0x%llx",
			 s390int->parm64);
		intitargS3%c;(		bren)g
rams =	list_, &li-cpuf=ptr	str	list_,vcp0_REaDarq rams =iocase KVM_S390_INT_PFAULT_INIT:
	castargS3%c;(		bren)g
rams =q aD;
		, &li-cpuf=ptr	strq aD;
		,vcp0_REaDarq rams =q aDaO_MIN...KVM_?%c;{_?%c;{_M_S390_INT_CPU_TIM

}

scopyVc anIist7(targS3,X, &li-,D aDarrg.c90int->pak;
	cLT *kvm, u64 type)
{
	structensilen =Dar_ =Dar_ (0int->parm,de -=  *de 3
ord_to_isc(intide -= 1attr_ attrIO_MAX:
		atomic_or(C(s390int->parm & 0xf>pending_irqs)r{
;?% {
;?%q->u.tattrs);ttr{
;?% {
;?% %D aDarq ti->type;
	switch ()m!emerg.c90int->parq aDqB	struct kv;?% {
;?% {_dinti->)=aDhNVAL;
rg.c90int->parq aDqB>.ect_pfa;?% {=D aDarq ti->type;
	switch ()=n 0;
}_parm = s390int->parm64 >> 32;
		inti->io.iio_int_word =  s390int->parm64 & {_%c;copyVnnelc anIist7(j.flagattrs);ddrio.iio_inrddress;)
			return -EIkvm_s390_VM-a}st_rrm64;
		break;
de chk->creturn -EIo_inrddress;)
			return -EIkvm_s390_VM-a}st_;?% -=D aDarq ti->type;
	switch ()VM-aattrs);ddronli aDarq ti->type;
	switch ()VM-nt kvm_s390_VM_S390_sc;
ti->typswitcho_ad, KVM__io_int(ad, KVMDar_ =Dar_ =Dar_ =Dect_prog
;?% d		return  >= ock);
nt->paO_ADAPTERcode);
	tracnding_i;
	traclags);
		bad, KVMo;{d] type)
{
	structreging_iint(ad, KVMDar_ =Dar_ ,de -=  *de 3
ord000)
			return -EIde -= 1attr_ attrIO_MAX:
		atswitcho_ad, KVM__ad, KVM inti->type;
	switcho_ad, KVM_ad, KVM>parm;M

}

scopyVc anIist7(&ad, KVM>parm3
ord000(		break=Dar_ )attrs);ddr a aDarq ad, KVM>parm)rrg.c90int->pak;
	cLT * 3, "ad, KVM>parm. >= ock);
nt->paO_ADAPTERco *inttimerde chk->s);
		bad, KVMo;ad, KVM>parm. >]m!em;
	mcrg.c90int->parq aDqB>.ad, KVM_m = s390int->parm6ad, KVM 32;
		inti->io.io_intad, KVM 
  s390int->parm64 &  = srq aDaHEAD(&ad, KVM =qap	rc = nit_rwsem(&ad, KVM =qap	_ &fi->pea -ENOMde = ad, KVM =n excpgp_rest	ad, KVM =int->ad, KVM>parm. >st	ad, KVM =icpu_sad, KVM>parm. e_vc	ad, KVM =qaskm >> _sad, KVM>parm.qaskm >>vc	ad, KVM =qaskM_MAXfatruvc	ad, KVM =swap _sad, KVM>parm.swapvc	ad, KVM =sup3, "%s", "c;(ad, KVM>parm.>type; &
cpu->kvm, irqADAPTER->u.eRESSIBLEVM_?% chk->s);
		bad, KVMo;ad, KVM =in] _sad, KVMLT *kvm, u64 typep_irq_c*inti)qask(ad, KVMDar_ =Dar_ =Dar_ =Dect_prog
;?% d=Drool qaskM_IO_MA {
;?eefault:
		aswitcho_ad, KVM__ad, KVMm,
			    ad, KVMDk->creli->irq.sttad, KVM		ir!ad, KVM =qaskm >>rg.c90int->parq aDqB	_S3%c;ad, KVM =qaskM_vc	ad, KVM =qaskM_MAXqaskM_vc	lse if (S3 type)
{
	structtomic_or(ad, KVM>xcpDar_ =Dar_ =Dar_ =Dect_prog
;?% d=D_deli ;ddrr_ =Dar_ =Daswitcho_ad, KVM__ad, KVMm,
			    ad, KVMDk->creli->Dar_ =Daswitc(&ir>mcic &a{
	structet->irq.sttad, KVM		ir!addrrg.c90int->parq aDqB>.map _s= s390int->parm6map 32;
		inti->io.io_intmap l,
	[IS3%c;{_parm64 ;K
		return -}&  = srq aDaHEAD(&map)n KVM_IDKmap)nding (;ddro= addr

imap)n;ddro= g(&ir*vcpuvm_s(k->s);
		bg(&i, ;ddrio.it kvmap)n;ddro=c;{_k;
	c l,
	[IS3%c;{_k;
	cLT K
		return -}& IS3%c;_ =Dist7_pagesPU_EVEmap)n;ddr, 1, 1, &map)npage_S390_INTS3%< 0ck);
		return -BUG_ONNTS3%!em1_S39down_pfaul(&ad, KVM =qap	_ &fi->pe% {
at-ENOMinint int-= ad, KVM =n excpg)%< k);
nt->pADAPTER-MAPSDflags;
		/ VCPU dropmap)n KVM anad, KVM =qap	rc =	?% {
;?% {390_inject_ND_Epage(map)npage_S39[IS3%c;{_NT_CPU_TIMEup_pfaul(&ad, KVM =qap	_ &fi->pt(IRQ_0_INTS3PFAULT_INImap vc	lse if (S3 type)
{
	structtomic_or(ad, KVM>unxcpDar_ =Dar_ =Dar_ =Dect_prog
;?% d=D_deli ;ddrr_ =Dar_ =Daswitcho_ad, KVM__ad, KVMm,
			    ad, KVMDk->creli->Dar_ =Daswitc(&ir>mcic &a{,e*tm{
	strucoat_in*/
	}

	if tad, KVM		ir!addrrg.c90int->parq aDqB>.down_pfaul(&ad, KVM =qap	_ &fi->peND_MCHK_REP, &fiqFh_%w.pg&a{,etm{ anad, KVM =qap	truct kvm *kvm,
map)nding (;ddro== addrddress;oat_in*/VENT_ea -ENOMdec= ad, KVM =n excpg)ENT_e;
		/* o74map)n KVM_IDKt_ND_Epage(map)npage_S39[ULT_INImap vc	MIN...KVM_:}ti->mup_pfaul(&ad, KVM =qap	_ &fi->pc	lse if oat_in? 0 :>parq aDqBt_{_o;{_	list_{_oKVM_roy ad, KVM	list_{_o;{_	list_{_o;
;?% {
Dar_ =Daswitc(&ir>mcic &a{,e*tm{
	m-Darq aDarq aDarqk);
nt->paO_ADAPTERcarq aDflags	if tlags);
		bad, KVMo;{] *fi = &kvm->archND_MCHK_REP, &fiqFh_%w.pg&a{,etm{ 
 |= in&lags);
		bad, KVMo;{] =qap	truct kvm *ke;
		/* o74map)n KVM_IDKt_ND_Epage(map)npage_S39[ULT_INImap vc	M}FAULT_INIlags);
		bad, KVMo;{] ;M_IDKVM_IDKVM_pe) {->cpuint(ad, KVMDar_ =Dar_ ,de -=  *de 3
ord000)
to_isc(intide -= 1attr_ attrIO_MAX:
		atomic_or(C(t(ad, KVMnt qYNC_fault:
		aswitcho_ad, KVM__ad, KVM
	structet->irq.stcopyVc anIist7(&NC_,0(		break=Dar_ )attrs);ddr a aDarq NC_)rrg.c90int->pak;
	cLT *ad, KVMm,
			    ad, KVMDde chk->crNC_.eli->Do_intad, KVM 
  s390int->NT_CPU_T		break;NC_. KVM_S390_MCHK:
		irq->uOpADAPTER-MASK:39[IS3%c;q_c*inti)qask(ad, KVMDde chk->crNC_.elcrNC_.qaskn -EIo_inrS3%> 0));KV_S3%c;0ject: machine check mcic 0xOpADAPTER-MAP:39[IS3%c;q_c*inti)ad, KVM>xcpDde chk->crNC_.elcrNC_.;ddrio.ii: machine check mcic 0xOpADAPTER-UNMAP:39[IS3%c;q_c*inti)ad, KVM>unxcpDde chk->crNC_.elcrNC_.;ddrio.ii: machin?%c;{_?%c;{_M_%c;{_?%c;_

int kvm_s390_S3 type)
{
	structrubchan_IDKVM_IDKVM_IDK			 s390int->parm,de -= 1attr_ attrIO	if (psw_intertatic int=ype) ;
	spi24parm;->kvm)csi->pepe);s & SCCB_{
;?% {
attrs)>type;
  s390int->NT_CPU_T% {
attrs);ttr_!li aDarq t_inte;
  s390int->NT_CPU_T% {
copyVc anIist7(&t_int,0(		break=Dar_ ) attrs);ddr a aDarq t_inte;rg.c90int->pak;
	cLTULT_INIlagoto out;
	fi->srvkt kvm_se KVM_St_inte;;bit(IRQ_PENDist7FP_	lissct kturnst_chIMER_R;
		iu.pguret_cpuompa(irq-a) {-stULT] >ne
				   ie superfluous 	breT_EXTERNALng_inturn Hs effecal_i_EXTstructubch;->kcpu timekvm, u64 type)
{
	struct{->cpuiais_modem64:%llx",
			 s390int->parm,de -= 1attr_ attrIO_MAX:
		atomic_or(CPUSTAT_IO_INT, li->cpuflags);
		break;
	default:
		atomic_or(ais_t qYNC_fau {
;?% {
;?% >.gm.flpu *vtomifacilityINT_CL72)rg.c90int->pa mark it aB>.q.stcopyVc anIist7(&NC_,0(		break=Dar_ )attrs);ddr a aDarq NC_)rrg.c90int->pak;
	cLT *o_inrSq. e_%> k);
	kfrg.c90int->parq aDqB>..code = s390int{->cpuiais_modemrSq. e_fff0	if (li KVM_MAXimm_& AIS
{
	stMASKmrSq. e_)) ?ff0	if (li KVM_MAnimm_& AIS
{
	stMASKmrSq. e_)) ?ff0	if (li K2 :>>kvm, irqAIS
{
	stSINGLE :ff0	if (li K>kvm, irqAIS
{
	stTATUrNC_.qailing>.{utex= atomic_reais_ &fi->pe		break;NC_.qailiS390_MCHK:
		irq->AIS
{
	stTAT:ff0M_MAXimm_&= ~AIS
{
	stMASKmrSq. e_);ff0M_MAnimm_&= ~AIS
{
	stMASKmrSq. e_);ff0: machine check mcic 0AIS
{
	stSINGLE:ff0M_MAXimm_|= AIS
{
	stMASKmrSq. e_);ff0M_MAnimm_&= ~AIS
{
	stMASKmrSq. e_);ff0: machin?%c;{_?%c;{_M_%c;{_?%c;_

inte{utex=PFAULT_DONE,ais_ &fi->p

lse if (S3 type)
{
	structtomic_or(
		breaaDKVM_IDKVM_IDK			 s3
ord_to_isc(switcho_ad, KVM__ad, KVMIO_MAX:
		atomic_or(CPUSTAT_IO_INT, li->cpuflags);
		break;
	default:
		atomic_or(_IO_INT, lESTART:cpum *k.de = s3nti->io;
		brIO(1hk.mcip_re3
or.INVAL;
03
or.INVA64"c;(ad, KVM =icpuspi27) |ype8ERGENCY3
o}fau {
;?% {
;?% >.gm.flpu *vtomifacilityINT_CL72)		ir!ad, KVM =sup3, "%s", rg.c90int->= s390int->parm;
		NT_CL&ESTART:ing>.{utex= atomic_reais_ &fi->pe>online_nimm_& AIS
{
	stMASKmad, KVM =icp)=n 0;
.code = s390intaDKV_sup3, "%edmad, KVM =id, ad, KVM =icp)LT K
		return -}&
[IS3%c;q_c*inti)->parm;
		NT_CL&ESTART:ingegm.flaS3%&& VM_MAXimm_& AIS
{
	stMASKmad, KVM =icp)==n 0;
ine_nimm_|= AIS
{
	stMASKmad, KVM =icp)LT K.code = s390int{->cpuiais_modemad, KVM =icp 
 |= int(li K>kvm, irqAIS
{
	stSINGLE,90int-}&t(IRQ_{utex=PFAULT_DONE,ais_ &fi->p
lse if (S3 type)
{
	structulic_
		breaaDKVM_IDKVM_IDK			 s390int->parm,de -= 1attr_ attrIO_MADect_prog
;?% du.tattrs);ttr{
ult:
		aswitcho_ad, KVM__ad, KVMm,
			    ad, KVMDk->creli->irq.sttad, KVMrg.c90int->parq aDqB>.90int->= s390int->parm;aDKVMk->cr;d, KVMrKVM_S390_INT_IO_flic_ais_mode_s =Dar_m64:%llx",
			 s390int->parm,de -= 1attr_ attrIO_MAX:
		atomic_or(CPUSTAT_IO_INT, li->cpuflags);
		break;
	default:
		atomic_or(ais_ct_sais{
;?% {
lpu *vtomifacilityINT_CL72)rg.c90int->pa mark it aB>.q.stcopyVc anIist7(&ais,D(		break=Dar_ )attrs);ddr a aDarq ais)rrg.c90int->pak;
	cLT *{utex= atomic_reais_ &fi->peM_MAXimm_= ais.simm>peM_MAnimm_= ais.nimm>pe{utex=PFAULT_DONE,ais_ &fi->p

kvm, u64 type)
{
	structflic_s =Dattr(0int->parm,de -=  *de 390int->parm,de -= 1attr_ attrIO_MA {
;? 
	return rc;
}

;?% {
Dar_ =Da_vm(kvm, inti)u(struct kvmattrs)group_S390_MCHK:
		DEVnti->)ENQUEUE%c;{_%c;ensilen =Dar_ =Dar_ (de 39;ttrIO_MIN...KVM_S390_INT_DEVnti->)CLEARind i%c;{	list_{_o;{_	list_{_o;{_	lde chk->IO_MIN...KVM_S390_INT_DEVnti->)APF)ENABLE:ff0?% chk->s);
		bgmap)np
	casetime;

	*/VENT_N...KVM_S390_INT_DEVnti->)APF)DISABLE_WA= inti?% chk->s);
		bgmap)np
	casetime;

	*/0jectcpu->arcMrc =suase K async 
	casS390_PGM_Fvcpum_s390whenu->arc;{_	lst_checksilens. Slist_don't__iCPUN		worryu->arcab	brevm_slockst_cworkt7F>sigp_emersynchronaDa_ up (&?% chk->s) up too;{	lisHK_REP, &m(kvmi, , intide chk->I39[ULlis;{_	liasync_pf_ock)le_s39_silenfree(inti);
	}
}

static voDEVnti->)ADAPTER-REGIST
		list	*/reging_iint(ad, KVMDde 39;ttrIO_MIN...KVM_S390_INT_DEVnti->)ADAPTER-MODIFY	list	*/{->cpuint(ad, KVMDde 39;ttrIO_MIN...KVM_S390_INT_DEVnti->)CLEARin	fi->	list	*/rubchan_IDKVMde chk->cr;ttrIO_MIN...KVM_S390_INT_DEVnti->)AISM	list	*/{->cpuiais_modemde chk->cr;ttrIO_MIN...KVM_S390_INT_DEVnti->)AIRQ
		JECTkvm_s_paulic_->parm;aDKVMde chk->cr;ttrIO_MIN...KVM_S390_INT_DEVnti->)AISM_ct kvm_s_paulic_ais_mode_s =Dar_mde chk->cr;ttrIO_MIN...KVM_?%c;{_?%c;{_%c;{_?%c;_

int kvm_s390_VM_S390_sc;

	ctflic_archattr(0int->parm,de -=  *de 3
ord000)
to_isc(intide -= 1attr_ attrIO_MAXuct kvmattrs)group_S390_MCHK:
		DEVnti->)GET_ct ind i%c;_MCHK:
		DEVnti->)ENQUEUE%c;S390_INT_DEVnti->)CLEARind i%c;S390_INT_DEVnti->)APF)ENABLE:ffS390_INT_DEVnti->)APF)DISABLE_WA= intstatic voDEVnti->)ADAPTER-REGIST
		liS390_INT_DEVnti->)ADAPTER-MODIFY	liS390_INT_DEVnti->)CLEARin	fi->	liS390_INT_DEVnti->)AISM	liS390_INT_DEVnti->)AIRQ
		JECTkvmS390_INT_DEVnti->)AISM_ct kvm_svm, u64 tTIMER:
		rc-ENXIOVM_S390_sc;

	ctflic_c...te(0int->parm,de -=  *de 39s & INT_IO_MA% {
lde ;
  s390int->NT_CPU_T% {
?% chk->s);
		bflic;
  s390int->NT_CPU_T?% chk->s);
		bflicrn _ev;

kvm, u64 type)
{
	st		breflic_KVM_roy(0int->parm,de -=  *de IO_MA?% chk->s);
		bflicrn nding_iLT_INIde Ich (typnlock(_or(CPUSTAT_S= &krollar_(flic;>srv_signal.extde -= 1 __i	lisHlic_ __ipum *.nam= s3"k->sHlicffff._ =Dattr_paulic__ =Dattrfff.s =Dattr_paulic_s =Dattrfff.archattr_paulic_archattrfff.c...te_paulic_c...tefff.KVM_roy_paulic_KVM_roy,=Dar_ =Dar_ ject_prog(stru			  ndvcpu)_deli ;ddr, ject_prog(strucpu_nr=Drool swapIO_MADect_prog(strucpuu(stcpu_pacpu_nr +;(addr % PAGstSIZE)m_a8>p

kvm, u6swap ? (cpu_^ (BITS{
		sLONG - 1==n:ucpuu(_S390_sc;
ti->typswitc(&ir>mcic 			 (&ir>mci(lt:
		aswitcho_ad, KVM__ad, KVM 
 |= inteli ;ddrr_ =Dar_ =Daswitc(&ir>mcic &a{
	irq.sttad, KVMrg.c90int->nding_G 0x3

static int __in&a{,enad, KVM =qap	truct kvm *kvm,
map)nding (;ddro== addrd-EIkvm_s390&a{
	sIMER:
		rcusy.
	 * 90_sc;

	ctad, KVM>paer_accesMde =fo *inti)
{
	struct kv
ti->typswitcho_ad, KVM__ad, KVM 
 |= 			return -EINVAL;ad, KVM>pat__ad, KVM>patIO_MADect_prog(strucpuu(_unlocummaFh_%S3,Xidx{
Dar_ =Daswitc(&ir>mcic parm;Mi		bre*&a{
	irqmcic,
			 (&ir>mci(ad, KVM tad, KVM>pat =indv;ddrio.it kv!parm)
  s390int-VENTmap _sp_CPU_TIMER,(parm)npage_S39cpu_pa			  ndvcpu)parm)n;ddr, ad, KVM>pat =indvoff%S3,Xad, KVM =swap->pe	ake it  it, map vc	idx0_REup nt __= atomik->s) up too;maFkEpage_dirtyINT_CLparm)nding (;ddro>> PAGstSHIFT->pe	akepage_dirty= atomparm)npage_S39qmcic,
			 (&ir>mci(ad, KVM tad, KVM>pat =cummaFh_;ddrio.it kv!parm)le VCPup nt __=PFAULT_Dk->s) up ,Xidxn -EINVAL;
		1

inte{ap _sp_CPU_TIMER,(parm)npage_S39cpu_pa			  ndvcpu)parm)n;ddr, ad, KVM>pat =cummaFh_off%S3,
ord00ad, KVM =swap->pe	ummaFh_%S3_pa	set_bit(irq->u.e it, map vc	maFkEpage_dirtyINT_CLparm)nding (;ddro>> PAGstSHIFT->pe	akepage_dirty= atomparm)npage_S39Pup nt __=PFAULT_Dk->s) up ,Xidxn -Ekvm, u6summaFh_%S3_? 0 :>1ch (type) {< 0 -e KVM->parmrogduliTIMerrore) {= 0 -ecoalesced,6summaFh paer_accevm_s390_iacal_ie) {> 0 -e->parmrog_IO_INT, >cpuflags);
unloc =Dad, KVM>pat(0int->parm,kt7s390irq_rou_ =Da&fi->loe3
ord00064:%llx",
			 s39
;?% KV_s &li-equeu
;?%q-vel3
ord000rool NT_PFif (kvIO_MA {
;?eefault:
		aswitcho_ad, KVM__ad, KVM __injeWe'nterT, l_IO_Ing rog_Icheck0->1_Fvcpum_s39.VCPU * KV!q-vel)vm_svm, u64 tTad, KVMm,
			    ad, KVMDk->cre)n;d, KVM.ad, KVM>pli->Do_intad, KVM 
  s390int-1

idown_t __inad, KVM =qap	_ &fi->pe_S3%c;ad, KVM>paer_accesMde =k->cr;d, KVM,ene)n;d, KVM->peup_t __inad, KVM =qap	_ &fi->pe 3, "rS3%> 0)ree(iad, KVM =qaskM_ l,
	[IS3%c;= s390int->parm;aDKVMk->cr;d, KVMrKVEIo_inrS3%.emerg.c[IS3%c;1
	sIMER:
		rc(S3 typeype) {I
	retuheck
		if (s390inthIMER_Rding ->cpuf_o;{_	list_{_ot->parm;

		if (_390in Because repressible machine ch0064:%llxmcck_vovm_iler>mcic &cck_parm)
_MAX:
		atomic_or(C(s390int->parm _ =Dar_ =Dar_ =Dar_ =DaAT_Sti->ty =Dar_ =Dar_ =Damchk->mcic & MCeturns390mci0mci;arm6eli pgm.cod0;         rm;
		break;
	case KVM_S390_SIGD	capID	c_mci.va);

&cck_parm90_flo>pe 3, mci.sM 
  _0 + iscMCruptR14-RECOVERYfi->d_tail(& 3, mci.dg 
  _0 + iscMCruptR14-DEGRADfi->d_tail(& 3, mci.w 
  _0 + iscMCruptR14-WARNfi->d_tail(M_S390;

&ci.int?g
rams.& MCH:r(CPU.trq aD;
	rch.local_i= T(vcpu, 3, "%s", "

&cck_parm90_flo>pernal");
	trace_kvm_s39

&cck_parm90cpu->vcpu_id, KVM_S390_INT_CPU_TIMER,
				   0, 0)&cck_parm90IRQ_PEND_EXT_CPU_TIMER, &l 3, mci.fi-l,
	[/ {I
	retuheck_or(CPUST
		if (s390int_SIGP_SET.de = s3nti->io;
ting;rq-parm64;
		break;
/
	mchk->cr&M_IDKVM_I90_inject_/ {I
	retuheck
		if (s390inthIMspecifi390ressibSIGP_rq.de = s3nti->io;
ting;rq-parm6q_c*inti)->parm;
(kvm, inti&h ()VM-nt	WARNfON390intrc) typep_irq_c*i		 rou_ =Da&fi->>u.ext.ext_params2 = s30int->parm,kt7s390irq_rou_ =Da&fi->loe3
ord00(psw_i0int->parm,irq_rou_ =Da&fi->loueIO_MA {
;?eefaMAXuct kvmuese KVM_S390_MCHK:
		IRQ
ROUTk(&fit->pADAPTERkvm_ese%S3_pac =Dad, KVM>pat;vm_ese;d, KVM.cummaFh_;ddr_paueseu.;d, KVM.cummaFh_;ddr;vm_ese;d, KVM.indv;ddr_paueseu.;d, KVM.indv;ddr;vm_ese;d, KVM.cummaFh_off%S3_paueseu.;d, KVM.cummaFh_off%S3;vm_ese;d, KVM.indvoff%S3_paueseu.;d, KVM.indvoff%S3;vm_ese;d, KVM.ad, KVM>pl_paueseu.;d, KVM.ad, KVM>pl;
	[IS3%c;0ject: machin?%c;{_?%c;{_M_%c;{_?%c;_

int kvm_s390_S3 typep_irq_c*i		 msi(0int->parm,kt7s390irq_rou_ =Da&fi->loe3064:%llx",
			 s3IGP_SE% KV_s &li-equeu
;?%q-vel30rool NT_PFif (kvIO_MAs390int->NT_CPU_ypep_irq_c*inti)i		  KV_st.te(0int->parm,kvm, inti);
		break=Dar_ ;{_	t.teeu
;?%q-ar_ =Dar_ =Dar_ =Dar_ SK)
		set_bit(IRQ_PEND_MCHK_EX, &li->pending_irqs);
	else if (->type = inrqs)r{
;?% {
;?%RQ_PE_	R{
;vms390in	spin_pgm.fl_	Rd = s390int->parm64 & 0.stcopyVc anIist7((		bren)gr_ =D;{_	t.teeu	spi l,
	[I%c;{_k;
	cLT K
		retur_T_IN

int kt(IRQ_PDon't_s390wac =tst_check_IO_INT, lEt.te	   0,AD_O(struct kvm_s390_i_IO_INT, s
				   pu time	_{_t_info *inti)
{
	rePENDINkick it.
 */
stl,
	[I%c;{_cpu = fK
		retur_PFAULT

int kDarq  {
;?% n{< q->u/q aDarq *_	Rd	 n aDflagsI%c;__inject_vm(kvm, inti
_	R[n_	lis	o_inrd
	t_{_o;{_	lis
tur_PFAULTRQ_PEND_PFAULT_D*inti)
{
	rtur_T_IN:>mv_IDKV_	Rd	} else if ( type)
{
	st		bred */
	i->pendKVM_IDKVM_IDKV_eto_SK)
		set_bit(IRQ_PG_SUBCLASS 28
static int _ (inti-G_SUBCLASDect_prog(stru KV_INT_IO_MAXuct kvm_vc_ KVM_S390_MCHKm *kvm,
						  :90_MCHKm *kvm,
						REint;

	spide = s3nti->io;
ting;rq->u.ext.& MCHK_EP_MASK)
		setMIN...KVM_S390__EVENT(kvmROGnt;

	spide = s3nti->io;
_irq *irq)
;rq->u.ext.pgmHK_EP_MASK)pgmetMIN...KVM_S390__EVENT(kvm_S390_SIGP_STOP:
		de = s3nti->io;
		brm_S390_SIGP;rq->u.ext.ext_paEP_MASK) s390iruct kvm_sst) e;
	mchk->fail_irq(st_STOP:
		de = s3nti->io;
		bro_irq(struct ;rq->u.ext.extfo *_paEP_MASK) s3fo *90iruct kvm_sst) e;
	mchk->faiterrupt_info *P:
		de = s3nti->io;
		brterrupt_in90iruct kvm_sst) e;
	mchk->faitist) {
		lisP:
		de = s3nti->io;
		brtist) {
	90iruct kvm_sst) e;
	mchk-h.local_int;

	spide = s3nti->io;
h.local_i;rq->u.ext.s390_paEP_MASK)s39090iruct kvm_sst) e;
	mchk-atic inline
	spide = s3nti->io;
atic in90iruct kvm_sst) e;
	mchk-hq(struct kvm_vcpu de = s3nti->io;
h.locaq(struct ;rq->u.ext.p	rc =HK_EP_MASK)p	rc =90iruct kvm_}_ypep_irq_c*inti)g		  KV_st.te(0int->parm,kvm, inti);
ak=ar_ =Dar_ r_ =D;;?%q-ar_ =Dunloccn;MADect_prog(stru |= irq->u.mchk.ex[BITS{TOsLONGS(ock(&fi->lock)isc_to_isc_bits(isc)SK)
		set_bit(IRQ_PEND_MCHK_EX, &li->pending_iDect_prog(struck it.
 */
sar_ =Dar_ =Dar_ =DaAT_Sti->tyDect_prog(stru KV_INT_% {
;?%CHK;ddr;vm
;?% {
;?% me	_{_t_info *inti)
{
	re_	list__iCPUN		INkick it.
 */
s;&li->pendi |= irq->u.mchk.ex390_INT |= irq->u.mchk.ex3lags);
	return 0 |= irq->u.mchk.ex)}

int s390int_to_s3each_%w. kDartic inirq->u.e KV_INT_390ck it.
 */
s, e;
	mchk-rq aDDflags.code = ti-G