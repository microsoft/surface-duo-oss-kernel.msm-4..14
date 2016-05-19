/*
 * Copyright 2012-2016 Freescale Semiconductor, Inc.
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
 * Each pit takes 0x10 Bytes register space
 */
#define PITMCR		0x00
#define PIT0_OFFSET	0x100
#define PITn_OFFSET(n)	(PIT0_OFFSET + 0x10 * (n))
#define PITLDVAL	0x00
#define PITCVAL		0x04
#define PITTCTRL	0x08
#define PITTFLG		0x0c

#define PITMCR_MDIS	(0x1 << 1)

#define PITTCTRL_TEN	(0x1 << 0)
#define PITTCTRL_TIE	(0x1 << 1)
#define PITCTRL_CHN	(0x1 << 2)

#define PITTFLG_TIF	0x1

#if !defined(CONFIG_PIT_CLKSRC_RATE)
#define CONFIG_PIT_CLKSRC_RATE	300
#endif

#if !defined(CONFIG_PIT_CLKEVT_RATE)
#define CONFIG_PIT_CLKEVT_RATE	300
#endif

#if defined(CONFIG_SOC_S32V234)
#define PIT_NR	2
#else
#define PIT_NR	1
#endif

#define MASTER_CORE 0
#define TIMER_NAME "FSL pit timer"

struct pit_timer {
	void __iomem *clksrc_base;
	void __iomem *clkevt_base;
	int irq;
	int core;
	struct clk *pit_clk;
	unsigned long cycle_per_jiffy;
	struct clock_event_device clockevent_pit;
	struct irqaction pit_timer_irq;
};


static struct pit_timer *pit[PIT_NR];

static inline void pit_timer_enable(int cpu)
{
	__raw_writel(PITTCTRL_TEN | PITTCTRL_TIE,
		     pit[cpu]->clkevt_base + PITTCTRL);
}

static inline void pit_timer_disable(int cpu)
{
	__raw_writel(0, pit[cpu]->clkevt_base + PITTCTRL);
}

static inline void pit_irq_acknowledge(int cpu)
{
	__raw_writel(PITTFLG_TIF, pit[cpu]->clkevt_base + PITTFLG);
}

static u64 pit_read_sched_clock(void)
{
	u64 val = ~__raw_readl(pit[get_cpu()]->clksrc_base + PITCVAL);

	put_cpu();

	return val;
}

static int __init pit_clocksource_init(int cpu, unsigned long rate)
{
	__raw_writel(0,  pit[cpu]->clksrc_base + PITTCTRL);
	__raw_writel(0xFFFFFFFF,  pit[cpu]->clksrc_base + PITLDVAL);
	__raw_writel(PITTCTRL_TEN,  pit[cpu]->clksrc_base + PITTCTRL);

	sched_clock_register(pit_read_sched_clock, 32, rate);
	clocksource_mmio_init(pit[cpu]->clksrc_base + PITCVAL,
			     "vf-pit", rate,
			     CONFIG_PIT_CLKSRC_RATE, 32,
			     clocksource_mmio_readl_down);

	return 0;
}

static int pit_set_next_event(unsigned long delta,
				struct clock_event_device *unused)
{
	int cpu;
	/*
	 * set a new value to PITLDVAL register will not restart the timer,
	 * to abort the current cycle and start a timer period with the new
	 * value, the timer must be disabled and enabled again.
	 * and the PITLAVAL should be set to delta minus one according to pit
	 * hardware requirement.
	 */
	cpu = get_cpu();
	pit_timer_disable(cpu);
	__raw_writel(delta - 1,  pit[cpu]->clkevt_base + PITLDVAL);
	pit_timer_enable(cpu);
	put_cpu();

	return 0;
}

static void pit_set_mode(enum clock_event_mode mode,
				struct clock_event_device *evt)
{
	int cpu;

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		cpu = get_cpu();
		put_cpu(); /* TODO: fix next call for preemption */
		pit_set_next_event(pit[cpu]->cycle_per_jiffy, evt);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_UNUSED:
		pit_timer_disable(get_cpu());
		put_cpu();
		break;
	default:
		break;
	}
}

static irqreturn_t pit_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	int cpu = get_cpu();

	put_cpu();

	pit_irq_acknowledge(cpu);

	/*
	 * pit hardware doesn't support oneshot, it will generate an interrupt
	 * and reload the counter value from PITLDVAL when PITCVAL reach zero,
	 * and start the counter again. So software need to disable the timer
	 * to stop the counter loop in ONESHOT mode.
	 */
	if (likely(evt->mode == CLOCK_EVT_MODE_ONESHOT))
		pit_timer_disable(cpu);


	evt->event_handler(evt);


	return IRQ_HANDLED;
}

static int __init pit_clockevent_init(int cpu, unsigned long rate, int irq)
{

	__raw_writel(0, pit[cpu]->clkevt_base + PITTCTRL);

	pit[cpu]->clockevent_pit.name = TIMER_NAME;
	pit[cpu]->clockevent_pit.features = CLOCK_EVT_FEAT_PERIODIC |
					    CLOCK_EVT_FEAT_ONESHOT;
	pit[cpu]->clockevent_pit.set_mode = pit_set_mode;
	pit[cpu]->clockevent_pit.set_next_event = pit_set_next_event;
	pit[cpu]->clockevent_pit.rating = CONFIG_PIT_CLKEVT_RATE;
	pit[cpu]->clockevent_pit.cpumask = cpumask_of(cpu);
	pit[cpu]->clockevent_pit.irq = irq;

	pit[cpu]->pit_timer_irq.name = TIMER_NAME;
	pit[cpu]->pit_timer_irq.flags = IRQF_TIMER | IRQF_IRQPOLL;
	pit[cpu]->pit_timer_irq.handler = pit_timer_interrupt;
	pit[cpu]->pit_timer_irq.dev_id = &pit[cpu]->clockevent_pit;

	BUG_ON(setup_irq(irq, &pit[cpu]->pit_timer_irq));

	if (cpu == MASTER_CORE)
		BUG_ON(irq_set_affinity(irq,  cpumask_of(cpu)));

	/*
	 * The value for the LDVAL register trigger is calculated as:
	 * LDVAL trigger = (period / clock period) - 1
	 * The pit is a 32-bit down count timer, when the conter value
	 * reaches 0, it will generate an interrupt, thus the minimal
	 * LDVAL trigger value is 1. And then the min_delta is
	 * minimal LDVAL trigger value + 1, and the max_delta is full 32-bit.
	 */
	clockevents_config_and_register(&pit[cpu]->clockevent_pit, rate, 2,
					0xffffffff);

	if (cpu == MASTER_CORE)
		__raw_writel(PITTFLG_TIF, pit[cpu]->clkevt_base + PITTFLG);

	return 0;
}

static int pit_timer_cpu_notify(struct notifier_block *self,
	unsigned long action, void *hcpu)
{
	int mcpu;

	/*
	* Grab cpu pointer in each case to avoid
	* spurious preemptible warnings
	*/
	int cpu = (long)hcpu;

	if (cpu > PIT_NR)
		return NOTIFY_OK;

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_ONLINE:
		BUG_ON(irq_set_affinity(pit[cpu]->irq,
			    cpumask_of(cpu)));
		__raw_writel(PITTFLG_TIF,
			     pit[cpu]->clkevt_base + PITTFLG);
	break;
	case CPU_STARTING:
		mcpu = get_cpu();
		put_cpu();
		pit_clockevent_init(mcpu,
			    pit[mcpu]->cycle_per_jiffy * (HZ),
			    pit[mcpu]->irq);
	break;
	case CPU_DYING:
		pit_timer_disable(get_cpu());
		put_cpu();
	    break;
	}

	return NOTIFY_OK;
}

static struct notifier_block pit_timer_cpu_nb = {
	.notifier_call = pit_timer_cpu_notify,
};

static void __init pit_timer_init(struct device_node *np)
{
	void __iomem *timer_base;
	unsigned long clk_rate;
	int core;

	of_property_read_u32(np, "core", &core);

	if (core > PIT_NR) {
		pr_err("Please specify a correct core number.\n");
		return;
	}

	pit[core] = kzalloc(sizeof(struct pit_timer), GFP_KERNEL);
	if (pit[core] == NULL) {
		pr_err("Impossible to allocate memory\n");
		return;
	}

	pit[core]->core = core;

	timer_base = of_iomap(np, 0);
	BUG_ON(!timer_base);

	/*
	 * PIT0 and PIT1 can be chained to build a 64-bit timer,
	 * so choose PIT2 as clocksource, PIT3 as clockevent device,
	 * and leave PIT0 and PIT1 unused for anyone else who needs them.
	 */
	pit[core]->clksrc_base = timer_base + PITn_OFFSET(2);
	pit[core]->clkevt_base = timer_base + PITn_OFFSET(3);

	pit[core]->irq = irq_of_parse_and_map(np, 0);
	BUG_ON(pit[core]->irq <= 0);

	pit[core]->pit_clk = of_clk_get(np, 0);
	BUG_ON(IS_ERR(pit[core]->pit_clk));

	BUG_ON(clk_prepare_enable(pit[core]->pit_clk));

	clk_rate = clk_get_rate(pit[core]->pit_clk);
	pit[core]->cycle_per_jiffy = clk_rate / (HZ);

	/* enable the pit module */
	__raw_writel(~PITMCR_MDIS, timer_base + PITMCR);

	if (core == MASTER_CORE) {
		BUG_ON(register_cpu_notifier(&pit_timer_cpu_nb));
		BUG_ON(pit_clocksource_init(core, clk_rate));
		pit_clockevent_init(core, clk_rate, pit[core]->irq);
	}
}
CLOCKSOURCE_OF_DECLARE(s32v234, "fsl,s32v234-pit", pit_timer_init);
CLOCKSOURCE_OF_DECLARE(vf610, "fsl,vf610-pit", pit_timer_init);
