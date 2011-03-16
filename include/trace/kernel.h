#ifndef _TRACE_KERNEL_H
#define _TRACE_KERNEL_H

#include <linux/tracepoint.h>

DECLARE_TRACE(kernel_printk,
	TP_PROTO(unsigned long retaddr),
		TP_ARGS(retaddr));
DECLARE_TRACE(kernel_vprintk,
	TP_PROTO(unsigned long retaddr, char *buf, int len),
		TP_ARGS(retaddr, buf, len));
DECLARE_TRACE(kernel_module_free,
	TP_PROTO(struct module *mod),
		TP_ARGS(mod));
DECLARE_TRACE(kernel_module_load,
	TP_PROTO(struct module *mod),
		TP_ARGS(mod));

#endif
