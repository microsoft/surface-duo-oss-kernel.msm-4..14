/*
 * max34417.h
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#if !defined(_TRACE_MAX34417_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MAX34417_H

#undef TRACE_SYSTEM
#define TRACE_SYSTEM max34417
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE max34417

#include <linux/tracepoint.h>

TRACE_EVENT(max34417_read,
	TP_PROTO(u32 channel,
		char* label,
		u64 power_uw,
		u16 voltage_mv),
	TP_ARGS(channel,
		label,
		power_uw,
		voltage_mv),
	TP_STRUCT__entry(
		__field(u32, channel)
		__string(ch_label, label)
		__field(u64, power_uw)
		__field(u16, voltage_mv)
	),
	TP_fast_assign(
		__entry->channel = channel;
		__assign_str(ch_label, label);
		__entry->power_uw = power_uw;
		__entry->voltage_mv = voltage_mv;
	),
	TP_printk("ch[%d]:%s P:%llu (uW) V:%d (mV)",
		__entry->channel, __get_str(ch_label), __entry->power_uw, __entry->voltage_mv)
);

#endif /* _TRACE_MAX34417_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
