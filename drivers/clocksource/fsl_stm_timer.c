/*
 * Copyright 2016 Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
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

#define MASTER_CPU	0
#define STM_TIMER_NAME	"FSL stm timer"

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
	struct list_head list;
};

static LIST_HEAD(stms_list);
static struct stm_timer *clocksource;
static bool registered;

static inline struct stm_timer *stm_timer_from_evt(
		struct clock_event_device *evt)
{
	return container_of(evt, struct stm_timer, clockevent_stm);
}

static inline void stm_timer_enable(struct stm_timer *stm)
{
	/* enable the stm module */
	__raw_writel(STM_CR_CPS | STM_CR_FRZ | STM_CR_TEN,
			stm->timer_base + STM_CR);

	/* enable clockevent channel */
	__raw_writel(STM_CCR_CEN, stm->clkevt_base + STM_CCR);
}

static inline void stm_timer_disable(struct stm_timer *stm)
{
	/**
	 * The counter is shared between channels and will continue to
	 * be incremented. If STM_CMP value is too small, the next event can
	 * be lost if we don't disable the entire module.
	 * Disabling the entire module, makes STM not suitable as clocksource.
	 */
	__raw_writel(0, stm->timer_base + STM_CR);
	__raw_writel(0, stm->clkevt_base + STM_CCR);
}

static u32 get_counter(struct stm_timer *stm)
{
	return __raw_readl(stm->timer_base + STM_CNT);
}

static inline void stm_irq_acknowledge(struct stm_timer *stm)
{
	u32 val;

	/* clear the interrupt */
	__raw_writel(STM_CIR_CIF, stm->clkevt_base + STM_CIR);

	/* update STM_CMP value using the counter value */
	val = get_counter(stm) + stm->delta;
	__raw_writel(val, stm->clkevt_base + STM_CMP);
}

static u64 stm_read_sched_clock(void)
{
	return __raw_readl(clocksource->timer_base + STM_CNT);
}

static int __init stm_clocksource_init(struct stm_timer *stm,
						unsigned long rate)
{
	clocksource = stm;
	sched_clock_register(stm_read_sched_clock, 32, rate);
	clocksource_mmio_init(clocksource->timer_base + STM_CNT,
			     "fsl-stm", rate,
			     CONFIG_STM_CLKSRC_RATE, 32,
			     clocksource_mmio_readl_up);

	return 0;
}

static int stm_set_next_event(unsigned long delta,
				struct clock_event_device *evt)
{
	u32 val;
	struct stm_timer *stm = stm_timer_from_evt(evt);

	stm_timer_disable(stm);

	stm->delta = delta;

	val = get_counter(stm) + delta;
	__raw_writel(val, stm->clkevt_base + STM_CMP);

	stm_timer_enable(stm);

	return 0;
}

static void stm_set_mode(enum clock_event_mode mode,
				struct clock_event_device *evt)
{
	struct stm_timer *stm = stm_timer_from_evt(evt);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		stm_set_next_event(stm->cycle_per_jiffy, evt);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_UNUSED:
		stm_timer_disable(stm);
		break;
	default:
		break;
	}
}

static irqreturn_t stm_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	struct stm_timer *stm = stm_timer_from_evt(evt);

	stm_irq_acknowledge(stm);

	/*
	 * stm hardware doesn't support oneshot, it will generate an interrupt
	 * and start the counter again so software need to disable the timer
	 * to stop the counter loop in ONESHOT mode.
	 */
	if (likely(evt->mode == CLOCK_EVT_MODE_ONESHOT))
		stm_timer_disable(stm);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static int stm_clockevent_init(struct stm_timer *stm, unsigned long rate,
								int irq)
{
	__raw_writel(0, stm->clkevt_base + STM_CCR);

	stm->clockevent_stm.name = STM_TIMER_NAME;
	stm->clockevent_stm.features = CLOCK_EVT_FEAT_PERIODIC |
					    CLOCK_EVT_FEAT_ONESHOT;
	stm->clockevent_stm.set_mode = stm_set_mode;
	stm->clockevent_stm.set_next_event = stm_set_next_event;
	stm->clockevent_stm.rating = CONFIG_STM_CLKEVT_RATE;
	stm->clockevent_stm.cpumask = cpumask_of(stm->cpu);
	stm->clockevent_stm.irq = irq;

	stm->stm_timer_irq.name = STM_TIMER_NAME;
	stm->stm_timer_irq.flags = IRQF_TIMER | IRQF_IRQPOLL;
	stm->stm_timer_irq.handler = stm_timer_interrupt;
	stm->stm_timer_irq.dev_id = &stm->clockevent_stm;

	BUG_ON(setup_irq(irq, &stm->stm_timer_irq));

	if (stm->cpu == MASTER_CPU)
		BUG_ON(irq_set_affinity(irq,  cpumask_of(stm->cpu)));

	clockevents_config_and_register(&stm->clockevent_stm, rate, 1,
					0xffffffff);

	if (stm->cpu == MASTER_CPU)
		__raw_writel(STM_CIR_CIF, stm->clkevt_base + STM_CIR);

	return 0;
}

static int stm_timer_cpu_notify(struct notifier_block *self,
	unsigned long action, void *hcpu)
{
	struct stm_timer *stm;
	int cpu = (long)hcpu;

	list_for_each_entry(stm, &stms_list, list)
		if (stm->cpu == cpu)
			break;

	if (stm == NULL || stm->cpu != cpu)
		return NOTIFY_OK;

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_ONLINE:
		BUG_ON(irq_set_affinity(stm->irq,
			    cpumask_of(stm->cpu)));
		__raw_writel(STM_CIR_CIF,
			     stm->clkevt_base + STM_CIR);
	break;
	case CPU_STARTING:
		stm_clockevent_init(stm,
			    stm->cycle_per_jiffy * (HZ),
			    stm->irq);
	break;
	case CPU_DYING:
		stm_timer_disable(stm);
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
	struct stm_timer *stm;

	of_property_read_u32(np, "cpu", &cpu);
	if (cpu < 0 || cpu >= num_possible_cpus()) {
		pr_err("%s: please specify a cpu number between 0 and %d.\n",
				STM_TIMER_NAME, num_possible_cpus() - 1);
		return;
	}

	stm = kzalloc(sizeof(struct stm_timer), GFP_KERNEL);
	if (stm == NULL) {
		pr_err("%s: impossible to allocate memory\n", STM_TIMER_NAME);
		return;
	}

	list_add_tail(&stm->list, &stms_list);

	stm->cpu = cpu;

	timer_base = of_iomap(np, 0);
	BUG_ON(!timer_base);

	stm->timer_base = timer_base;

	/* use channel 0 as clockevent */
	stm->clkevt_base = timer_base + STM_CH(0);

	stm->irq = irq_of_parse_and_map(np, 0);
	BUG_ON(stm->irq <= 0);

	stm->stm_clk = of_clk_get(np, 0);
	BUG_ON(IS_ERR(stm->stm_clk));

	BUG_ON(clk_prepare_enable(stm->stm_clk));

	clk_rate = clk_get_rate(stm->stm_clk);
	stm->cycle_per_jiffy = clk_rate / (HZ);

	if (registered == false) {
		BUG_ON(register_cpu_notifier(&stm_timer_cpu_nb));
		registered = true;
	}

	if (cpu == MASTER_CPU) {
		BUG_ON(stm_clocksource_init(stm, clk_rate));
		stm_clockevent_init(stm, clk_rate, stm->irq);
	}

	/* reset counter value */
	__raw_writel(0, timer_base + STM_CNT);
	/* enable the stm module */
	__raw_writel(STM_CR_CPS | STM_CR_FRZ | STM_CR_TEN,
					timer_base + STM_CR);
}
CLOCKSOURCE_OF_DECLARE(s32v234, "fsl,s32v234-stm", stm_timer_init);
CLOCKSOURCE_OF_DECLARE(s32gen1, "fsl,s32gen1-stm", stm_timer_init);
