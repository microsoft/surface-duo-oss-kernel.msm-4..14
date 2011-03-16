#ifndef _LTTNG_TRACE_IRQ_H
#define _LTTNG_TRACE_IRQ_H

#include <linux/kdebug.h>

/*
 * action can be NULL if not available.
 */
DECLARE_TRACE(irq_entry,
	TP_PROTO(unsigned int id, struct pt_regs *regs,
			struct irqaction *action),
		TP_ARGS(id, regs, action));
DECLARE_TRACE(irq_exit,
	TP_PROTO(irqreturn_t retval),
		TP_ARGS(retval));

DECLARE_TRACE(irq_tasklet_low_entry,
	TP_PROTO(struct tasklet_struct *t),
		TP_ARGS(t));
DECLARE_TRACE(irq_tasklet_low_exit,
	TP_PROTO(struct tasklet_struct *t),
		TP_ARGS(t));
DECLARE_TRACE(irq_tasklet_high_entry,
	TP_PROTO(struct tasklet_struct *t),
		TP_ARGS(t));
DECLARE_TRACE(irq_tasklet_high_exit,
	TP_PROTO(struct tasklet_struct *t),
		TP_ARGS(t));

#if 0
DECLARE_TRACE(irq_softirq_entry,
	TP_PROTO(struct softirq_action *h, struct softirq_action *softirq_vec),
		TP_ARGS(h, softirq_vec));
DECLARE_TRACE(irq_softirq_exit,
	TP_PROTO(struct softirq_action *h, struct softirq_action *softirq_vec),
		TP_ARGS(h, softirq_vec));
#endif
DECLARE_TRACE(irq_softirq_raise,
	TP_PROTO(unsigned int nr),
		TP_ARGS(nr));

#endif
