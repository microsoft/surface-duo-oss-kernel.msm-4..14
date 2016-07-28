/*
 * Copyright 2016 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>
#include <linux/slab.h>

/*
 * Each stm takes 0x10 Bytes register space
 */
#define STM_CR		0x00
#define STM_CNT		0x04
#define STM_CH(n)	(0x10 * (n + 1))

#define STM_CR_CPS	(0x0 << 8)
#define STM_CR_FRZ	(0x1 << 1)
#define STM_CR_TEN	(0x1 << 0)

#define STM_CCR		0x00
#define STM_CIR		0x04
#define STM_CMP		0x08

#define STM_CCR_CEN	(0x1 << 0)
#define STM_CIR_CIF	(0x1 << 0)

#define MAX_CPUS_NR	(NR_CPUS)
#define MASTER_CORE	0
#define TIMER_NAME	"FSL stm timer"

struct stm_timer {
	void __iomem *timer_base;
	void __iomem *clkevt_base;
	int irq;
	int cpu;
	struct clk *stm_clk;
	unsigned long cycle_per_jiffy;
	struct clock_event_device clockevent_stm;
	struct irqaction stm_timer_irq;
	unsigned long delta;
};

static struct stm_timer *stm[MAX_CPUS_NR];
static int stm_allocated;

static inline void stm_timer_enable(int cpu)
{
	/* enable the stm module */
	__raw_writel(STM_CR_CPS | STM_CR_FRZ | STM_CR_TEN,
			stm[cpu]->timer_base + STM_CR);

	/* enable clockevent channel */
	__raw_writel(STM_CCR_CEN, stm[cpu]->clkevt_base + STM_CCR);
}

static inline void stm_timer_disable(int cpu)
{
	/**
	 * The counter is shared between channels and will continue to
	 * be incremented. If STM_CMP value is too small, the next event can
	 * be lost if we don't disable the entire module.
	 * Disabling the entire module, makes STM not suitable as clocksource.
	 */
	__raw_writel(0, stm[cpu]->timer_base + STM_CR);
	__raw_writel(0, stm[cpu]->clkevt_base + STM_CCR);
}

static u32 get_counter(int cpu)
{
	return __raw_readl(stm[cpu]->timer_base + STM_CNT);
}

static inline void stm_irq_acknowledge(int cpu)
{
	u32 val;

	/* clear the interrupt */
	__raw_writel(STM_CIR_CIF, stm[cpu]->clkevt_base + STM_CIR);

	/* update STM_CMP value using the counter value */
	val = get_counter(cpu) + stm[cpu]->delta;
	__raw_writel(val, stm[cpu]->clkevt_base + STM_CMP);
}

static u64 stm_read_sched_clock(void)
{
	return __raw_readl(stm[MASTER_CORE]->timer_base + STM_CNT);
}

static int __init stm_clocksource_init(unsigned long rate)
{
	sched_clock_register(stm_read_sched_clock, 32, rate);
	clocksource_mmio_init(stm[MASTER_CORE]->timer_base + STM_CNT,
			     "fsl-stm", rate,
			     CONFIG_STM_CLKSRC_RATE, 32,
			     clocksource_mmio_readl_up);

	return 0;
}

static int stm_set_next_event(unsigned long delta,
				struct clock_event_device *unused)
{
	int cpu;
	u32 val;

	cpu = get_cpu();
	stm_timer_disable(cpu);

	stm[cpu]->delta = delta;

	val = get_counter(cpu) + delta;
	__raw_writel(val, stm[cpu]->clkevt_base + STM_CMP);

	stm_timer_enable(cpu);
	put_cpu();

	return 0;
}

static void stm_set_mode(enum clock_event_mode mode,
				struct clock_event_device *evt)
{
	int cpu;

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		cpu = get_cpu();
		put_cpu(); /* TODO: fix next call for preemption */
		stm_set_next_event(stm[cpu]->cycle_per_jiffy, evt);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_UNUSED:
		stm_timer_disable(get_cpu());
		put_cpu();
		break;
	default:
		break;
	}
}

static irqreturn_t stm_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	int cpu = get_cpu();

	put_cpu();

	stm_irq_acknowledge(cpu);

	/*
	 * stm hardware doesn't support oneshot, it will generate an interrupt
	 * and start the counter again so software need to disable the timer
	 * to stop the counter loop in ONESHOT mode.
	 */
	if (likely(evt->mode == CLOCK_EVT_MODE_ONESHOT))
		stm_timer_disable(cpu);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static int __init stm_clockevent_init(int cpu, unsigned long rate, int irq)
{
	__raw_writel(0, stm[cpu]->clkevt_base + STM_CCR);

	stm[cpu]->clockevent_stm.name = TIMER_NAME;
	stm[cpu]->clockevent_stm.features = CLOCK_EVT_FEAT_PERIODIC |
					    CLOCK_EVT_FEAT_ONESHOT;
	stm[cpu]->clockevent_stm.set_mode = stm_set_mode;
	stm[cpu]->clockevent_stm.set_next_event = stm_set_next_event;
	stm[cpu]->clockevent_stm.rating = CONFIG_STM_CLKEVT_RATE;
	stm[cpu]->clockevent_stm.cpumask = cpumask_of(cpu);
	stm[cpu]->clockevent_stm.irq = irq;

	stm[cpu]->stm_timer_irq.name = TIMER_NAME;
	stm[cpu]->stm_timer_irq.flags = IRQF_TIMER | IRQF_IRQPOLL;
	stm[cpu]->stm_timer_irq.handler = stm_timer_interrupt;
	stm[cpu]->stm_timer_irq.dev_id = &stm[cpu]->clockevent_stm;

	BUG_ON(setup_irq(irq, &stm[cpu]->stm_timer_irq));

	if (cpu == MASTER_CORE)
		BUG_ON(irq_set_affinity(irq,  cpumask_of(cpu)));

	clockevents_config_and_register(&stm[cpu]->clockevent_stm, rate, 1,
					0xffffffff);

	if (cpu == MASTER_CORE)
		__raw_writel(STM_CIR_CIF, stm[cpu]->clkevt_base + STM_CIR);

	return 0;
}

static int stm_timer_cpu_notify(struct notifier_block *self,
	unsigned long action, void *hcpu)
{
	int mcpu;

	/*
	* Grab cpu pointer in each case to avoid
	* spurious preemptible warnings
	*/
	int cpu = (long)hcpu;

	if (stm[cpu] == NULL)
		return NOTIFY_OK;

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_ONLINE:
		BUG_ON(irq_set_affinity(stm[cpu]->irq,
			    cpumask_of(cpu)));
		__raw_writel(STM_CIR_CIF,
			     stm[cpu]->clkevt_base + STM_CIR);
	break;
	case CPU_STARTING:
		mcpu = get_cpu();
		put_cpu();
		stm_clockevent_init(mcpu,
			    stm[mcpu]->cycle_per_jiffy * (HZ),
			    stm[mcpu]->irq);
	break;
	case CPU_DYING:
		stm_timer_disable(get_cpu());
		put_cpu();
	    break;
	}

	return NOTIFY_OK;
}

static struct notifier_block stm_timer_cpu_nb = {
	.notifier_call = stm_timer_cpu_notify,
};

void __init stm_timer_init(struct device_node *np)
{
	void __iomem *timer_base;
	unsigned long clk_rate;
	int cpu;

	of_property_read_u32(np, "cpu", &cpu);
	if (cpu < 0 || cpu >= MAX_CPUS_NR) {
		pr_err("fsl-stm: please specify a cpu number between 0 and %d.\n",
				MAX_CPUS_NR-1);
		return;
	}

	stm[cpu] = kzalloc(sizeof(struct stm_timer), GFP_KERNEL);
	if (stm[cpu] == NULL) {
		pr_err("fsl-stm: impossible to allocate memory\n");
		return;
	}

	stm_allocated++;
	stm[cpu]->cpu = cpu;

	timer_base = of_iomap(np, 0);
	BUG_ON(!timer_base);

	stm[cpu]->timer_base = timer_base;

	/* use channel 0 as clockevent */
	stm[cpu]->clkevt_base = timer_base + STM_CH(0);

	stm[cpu]->irq = irq_of_parse_and_map(np, 0);
	BUG_ON(stm[cpu]->irq <= 0);

	stm[cpu]->stm_clk = of_clk_get(np, 0);
	BUG_ON(IS_ERR(stm[cpu]->stm_clk));

	BUG_ON(clk_prepare_enable(stm[cpu]->stm_clk));

	clk_rate = clk_get_rate(stm[cpu]->stm_clk);
	stm[cpu]->cycle_per_jiffy = clk_rate / (HZ);

	if (stm_allocated == 1)
		BUG_ON(register_cpu_notifier(&stm_timer_cpu_nb));

	if (cpu == MASTER_CORE) {
		BUG_ON(stm_clocksource_init(clk_rate));
		stm_clockevent_init(cpu, clk_rate, stm[cpu]->irq);
	}

	/* reset counter value */
	__raw_writel(0, timer_base + STM_CNT);
	/* enable the stm module */
	__raw_writel(STM_CR_CPS | STM_CR_FRZ | STM_CR_TEN,
					timer_base + STM_CR);
}
CLOCKSOURCE_OF_DECLARE(s32v234, "fsl,s32v234-stm", stm_timer_init);
