/*
 * linux/arch/arm/common/okl4_hyp_gic.c
 *
 * Copyright (c) 2012-2018 General Dynamics
 * Copyright (c) 2014 Open Kernel Labs, Inc.
 *
 * Based on:
 *   linux/arch/arm/common/gic.c
 *   Copyright (C) 2002 ARM Limited, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This is a paravirtualised version of the ARM GIC driver, which uses
 * OKL4 system calls rather than register accesses to communicate with
 * the virtual GIC. The basic structure is almost identical to that
 * of the native GIC driver.
 *
 * Interrupt architecture for the GIC:
 *
 * o There is one Interrupt Distributor, which receives interrupts
 *   from system devices and sends them to the Interrupt Controllers.
 *
 * o There is one CPU Interface per CPU, which sends interrupts sent
 *   by the Distributor, and interrupts generated locally, to the
 *   associated CPU. The base address of the CPU interface is usually
 *   aliased so that the same address points to different chips depending
 *   on the CPU it is accessed from.
 *
 * Note that IRQs 0-31 are special - they are local to each CPU.
 * As such, the enable set/clear, pending set/clear and active bit
 * registers are banked per-cpu for these sources.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/version.h>

#include <microvisor/microvisor.h>

#include <asm/irq.h>
#include <asm/exception.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
#include <asm/mach/irq.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,2,0)
#include "irqchip.h"
#else
#include <linux/irqchip.h>
#endif

struct vgic_chip_data {
	struct irq_domain *domain;
	unsigned int vgic_irqs;
};

static struct vgic_chip_data vgic_data __read_mostly;

/*
 * Supported arch specific GIC irq extension.
 * Default make them NULL.
 */
struct irq_chip vgic_arch_extn = {
	.irq_eoi	= NULL,
	.irq_mask	= NULL,
	.irq_unmask	= NULL,
	.irq_retrigger	= NULL,
	.irq_set_type	= NULL,
	.irq_set_wake	= NULL,
};

static DEFINE_RAW_SPINLOCK(irq_controller_lock);

static inline unsigned int vgic_irq(struct irq_data *d)
{
	return d->hwirq;
}

static void __gic_eoi_irq(unsigned int irq, unsigned int source)
{
	okl4_error_t err;

	raw_spin_lock(&irq_controller_lock);
	err = _okl4_sys_interrupt_eoi(irq, source);
	raw_spin_unlock(&irq_controller_lock);
	WARN_ON(err != OKL4_OK);
}

static void vgic_eoi_irq(struct irq_data *d)
{
	__gic_eoi_irq(vgic_irq(d), 0UL);
}

static void vgic_mask_irq(struct irq_data *d)
{
	okl4_error_t err;

	raw_spin_lock(&irq_controller_lock);
	if (vgic_arch_extn.irq_mask)
		vgic_arch_extn.irq_mask(d);

	err = _okl4_sys_interrupt_mask(vgic_irq(d));
	raw_spin_unlock(&irq_controller_lock);
        if (err != OKL4_OK) {
            printk(KERN_WARNING "%s: Cannot mask irq %d\n",
                    d->chip->name, vgic_irq(d));
        }
}

static void vgic_unmask_irq(struct irq_data *d)
{
	okl4_error_t err;

	raw_spin_lock(&irq_controller_lock);
	if (vgic_arch_extn.irq_unmask)
		vgic_arch_extn.irq_unmask(d);
	err = _okl4_sys_interrupt_unmask(vgic_irq(d));
	raw_spin_unlock(&irq_controller_lock);
        if (err != OKL4_OK) {
            printk(KERN_WARNING "%s: Cannot unmask irq %d\n",
                    d->chip->name, vgic_irq(d));
        }
}

static int vgic_set_type(struct irq_data *d, unsigned int type)
{
	unsigned int gicirq = vgic_irq(d);
	okl4_gicd_icfgr_t icfgr = 0;
	okl4_error_t err;

	/* Interrupt configuration for SGIs can't be changed */
	if (gicirq < 16)
		return -EINVAL;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		okl4_gicd_icfgr_setedge(&icfgr, 1);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		okl4_gicd_icfgr_setedge(&icfgr, 0);
		break;
	default:
		return -EINVAL;
	}

	raw_spin_lock(&irq_controller_lock);

	if (vgic_arch_extn.irq_set_type)
		vgic_arch_extn.irq_set_type(d, type);

	/*
	 * As recommended by the spec, disable the interrupt before changing
	 * the configuration
	 */
	if (!irqd_irq_disabled(d)) {
		err = _okl4_sys_interrupt_mask(gicirq);
		if (err)
			goto fail;
	}

	err = _okl4_sys_interrupt_set_config(gicirq, icfgr);
	if (err)
		goto fail;

	if (!irqd_irq_disabled(d)) {
		err = _okl4_sys_interrupt_unmask(gicirq);
		if (err)
			goto fail;
	}

	raw_spin_unlock(&irq_controller_lock);
	return 0;

fail:
	raw_spin_unlock(&irq_controller_lock);
	return -EINVAL;
}

static int vgic_retrigger(struct irq_data *d)
{
	if (vgic_arch_extn.irq_retrigger)
		return vgic_arch_extn.irq_retrigger(d);

	/* the genirq layer expects 0 if we can't retrigger in hardware */
	return 0;
}

#ifdef CONFIG_SMP
static int vgic_set_affinity(struct irq_data *d, const struct cpumask *mask_val,
		bool force)
{
	unsigned gicirq = vgic_irq(d);
	unsigned targets = cpumask_bits(mask_val)[0];

	targets &= cpumask_bits(cpu_online_mask)[0] & 0xff;

	if (_okl4_sys_interrupt_set_targets(gicirq, targets) != OKL4_OK)
		return -EINVAL;

	return 0;
}
#endif

asmlinkage void __exception_irq_entry vgic_handle_irq(struct pt_regs *regs)
{
	struct vgic_chip_data *gic = &vgic_data;
	struct _okl4_sys_interrupt_ack_return result;
	u32 irqnr, source;

	do {
		result = _okl4_sys_interrupt_ack();

		irqnr = result.irq;
		source = result.source;

		if (likely(irqnr >= 16 && irqnr < 1020)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,0)
			handle_domain_irq(gic->domain, irqnr, regs);
#else
			irqnr = irq_find_mapping(gic->domain, irqnr);
			handle_IRQ(irqnr, regs);
#endif
			continue;
		}
		if (irqnr < 16) {
			__gic_eoi_irq(irqnr, source);
#ifdef CONFIG_SMP
			handle_IPI(irqnr, regs);
#endif
			continue;
		}
		break;
	} while (1);
}

#ifdef CONFIG_OKL4_FAKE_IRQ_SET_WAKE
int virq_set_wake(struct irq_data *data, unsigned int on)
{
       return 0;
}
#endif

static struct irq_chip okl4_hyp_gic_chip = {
	.name			= "OKL4_HYP_GIC",
	.irq_mask		= vgic_mask_irq,
	.irq_unmask		= vgic_unmask_irq,
	.irq_eoi		= vgic_eoi_irq,
	.irq_set_type		= vgic_set_type,
	.irq_retrigger		= vgic_retrigger,
#ifdef CONFIG_SMP
	.irq_set_affinity	= vgic_set_affinity,
#endif
#ifdef CONFIG_OKL4_FAKE_IRQ_SET_WAKE
	.irq_set_wake = virq_set_wake,
#endif
};

/* FIXME: Jira ticket SDK-4547 - johnc */
static void __init vgic_dist_init(struct vgic_chip_data *gic)
{
	int i;

	_okl4_sys_interrupt_dist_enable(true);

	/* disable all global interrupts */
	for (i = 32; i < gic->vgic_irqs; i++) {
		_okl4_sys_interrupt_mask(i);
		_okl4_sys_interrupt_set_config(i, 0);
	}
}

static void __init vgic_cpu_init(struct vgic_chip_data *gic)
{
	int i;

	/* Ensure all SGIs are enabled. */
	for (i = 0; i < 16; i++)
		_okl4_sys_interrupt_unmask(i);

	/* Ensure all PPIs are disabled. */
	for (i = 16; i < 31; i++)
		_okl4_sys_interrupt_mask(i);

	/* Enable interrupt delivery. */
	_okl4_sys_interrupt_set_priority_mask(0xff);
	_okl4_sys_interrupt_set_control(true);
}

static int vgic_irq_domain_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hw)
{
	if (hw < 32) {
		irq_set_percpu_devid(irq);
		irq_set_chip_and_handler(irq, &okl4_hyp_gic_chip,
					 handle_percpu_devid_irq);
		irq_set_status_flags(irq, IRQ_NOAUTOEN);
	} else {
		irq_set_chip_and_handler(irq, &okl4_hyp_gic_chip,
					 handle_fasteoi_irq);
		irq_set_probe(irq);
	}
	irq_set_chip_data(irq, d->host_data);
	return 0;
}

static int vgic_irq_domain_xlate(struct irq_domain *d,
		struct device_node *controller,
		const u32 *intspec, unsigned int intsize,
		unsigned long *out_hwirq, unsigned int *out_type)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,3,0)
	if (d->of_node != controller)
		return -EINVAL;
#endif
	if (intsize < 3)
		return -EINVAL;

	/* Get the interrupt number and add 16 to skip over SGIs */
	*out_hwirq = intspec[1] + 16;

	/* For SPIs, we need to add 16 more to get the GIC irq ID number */
	if (!intspec[0])
		*out_hwirq += 16;

	*out_type = intspec[2] & IRQ_TYPE_SENSE_MASK;

	/*
	 * The OKL4 GIC binding uses hwirq 1023 as a placeholder for absent
	 * interrupts.
	 *
	 * Ideally we would just leave them out of the tree altogether, but some
	 * bindings don't allow that. In particular, we need to do this to
	 * convince drivers/clocksource/arm_arch_timer.c to use the virtual
	 * timer, when the physical timer isn't available.
	 */
	if (*out_hwirq == 1023)
		return -ENXIO;

	return 0;
}

static const struct irq_domain_ops vgic_irq_domain_ops = {
	.map = vgic_irq_domain_map,
	.xlate = vgic_irq_domain_xlate,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,8,0)
static int vgic_starting_cpu(unsigned int cpu)
{
	vgic_cpu_init(&vgic_data);
	return 0;
}
#else
#ifdef CONFIG_SMP
static int __init vgic_secondary_init(struct notifier_block *nfb,
					unsigned long action, void *hcpu)
{
	if (action == CPU_STARTING)
		vgic_cpu_init(&vgic_data);
	return NOTIFY_OK;
}

/*
 * Notifier for enabling the GIC CPU interface. Set an arbitrarily high
 * priority because the GIC needs to be up before the ARM generic timers.
 */
static struct notifier_block __initdata vgic_cpu_notifier = {
	.notifier_call = vgic_secondary_init,
	.priority = 100,
};
#endif
#endif

#ifdef CONFIG_SMP
static void vgic_raise_softirq(const struct cpumask *mask, unsigned int irq)
{
	int cpu;
	unsigned long map = 0;
	okl4_error_t err;
	okl4_gicd_sgir_t sgir = 0;

	for_each_cpu(cpu, mask)
		map |= 1 << cpu;

	okl4_gicd_sgir_setsgiintid(&sgir, irq);
	okl4_gicd_sgir_setcputargetlist(&sgir, map);
	err = _okl4_sys_interrupt_raise(sgir);
	BUG_ON(err != OKL4_OK);
}
#endif

/* dist_base and cpu_base are unused because we are using syscalls */
void __init vgic_init(struct device_node *node)
{
	irq_hw_number_t hwirq_base = 16;
	struct vgic_chip_data *gic = &vgic_data;
	int vgic_irqs, irq_base;

	/*
	 * Find out how many interrupts are supported.
	 * The GIC only supports up to 1020 interrupt sources.
	 */
	vgic_irqs = _okl4_sys_interrupt_limits().itnumber;
	vgic_irqs = (vgic_irqs + 1) * 32;
	if (vgic_irqs > 1020)
		vgic_irqs = 1020;
	gic->vgic_irqs = vgic_irqs;

	vgic_irqs -= hwirq_base; /* calculate # of irqs to allocate */
	irq_base = irq_alloc_descs(-1, 16, vgic_irqs, numa_node_id());
	if (irq_base < 0) {
		WARN(1, "Cannot allocate irq_descs\n");
		return;
	}

	gic->domain = irq_domain_add_legacy(node, vgic_irqs, irq_base,
			hwirq_base, &vgic_irq_domain_ops, gic);
	if (WARN_ON(!gic->domain))
		return;

#ifdef CONFIG_SMP
	set_smp_cross_call(vgic_raise_softirq);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,8,0)
	cpuhp_setup_state_nocalls(CPUHP_AP_IRQ_OKL4_HYP_GIC_STARTING,
			"AP_IRQ_OKL4_HYP_GIC_STARTING",
			vgic_starting_cpu, NULL);
#else
#ifdef CONFIG_SMP
	register_cpu_notifier(&vgic_cpu_notifier);
#endif
#endif

	set_handle_irq(vgic_handle_irq);

	okl4_hyp_gic_chip.flags |= vgic_arch_extn.flags;
	vgic_dist_init(gic);
	vgic_cpu_init(gic);
}

#ifdef CONFIG_OF
static int vgic_cnt __initdata;

static int __init
vgic_of_init(struct device_node *node, struct device_node *parent)
{
	if (WARN_ON(!node))
		return -ENODEV;

	if (WARN_ON(vgic_cnt))
		return -ENODEV;

	if (WARN_ON(parent))
		return -ENODEV;

	vgic_init(node);

	vgic_cnt++;

	return 0;
}
IRQCHIP_DECLARE(okl4_hyp_gic, "okl,microvisor-vgic", vgic_of_init);

#endif
