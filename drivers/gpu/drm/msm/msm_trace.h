/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#if !defined(__MSM_TRACE_H__) || defined(TRACE_HEADER_MULTI_READ)
#define __MSM_TRACE_H__

#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/tracepoint.h>

#include <drm/drmP.h>
#include "msm_drv.h"
#include "msm_gpu.h"

#undef TRACE_SYSTEM
#define TRACE_SYSTEM msm
#define TRACE_SYSTEM_STRING __stringify(TRACE_SYSTEM)
#define TRACE_INCLUDE_FILE msm_trace


TRACE_EVENT(msm_flip_request,
		TP_PROTO(int id, struct drm_gem_object *obj),

		TP_ARGS(id, obj),

		TP_STRUCT__entry(
				__field(int, id)
				__field(struct drm_gem_object *, obj)
		),

		TP_fast_assign(
				__entry->id = id;
				__entry->obj = obj;
		),

		TP_printk("id=%d, obj=%p", __entry->id, __entry->obj)
);

TRACE_EVENT(msm_flip_complete,
		TP_PROTO(int id),

		TP_ARGS(id),

		TP_STRUCT__entry(
				__field(int, id)
		),

		TP_fast_assign(
				__entry->id = id;
		),

		TP_printk("id=%d", __entry->id)
);

TRACE_EVENT(msm_fence_wait_request,
		TP_PROTO(uint32_t fence),

		TP_ARGS(fence),

		TP_STRUCT__entry(
				__field(uint32_t, fence)
		),

		TP_fast_assign(
				__entry->fence = fence;
		),

		TP_printk("fence=%u", __entry->fence)
);

TRACE_EVENT(msm_fence_wait_complete,
		TP_PROTO(uint32_t fence, int ret),

		TP_ARGS(fence, ret),

		TP_STRUCT__entry(
				__field(uint32_t, fence)
				__field(int, ret)
		),

		TP_fast_assign(
				__entry->fence = fence;
				__entry->ret = ret;
		),

		TP_printk("fence=%u, ret=%d", __entry->fence, __entry->ret)
);

TRACE_EVENT(msm_gpu_request,
		TP_PROTO(uint32_t fence),

		TP_ARGS(fence),

		TP_STRUCT__entry(
				__field(uint32_t, fence)
		),

		TP_fast_assign(
				__entry->fence = fence;
		),

		TP_printk("fence=%u", __entry->fence)
);

TRACE_EVENT(msm_gpu_complete,
		TP_PROTO(uint32_t fence),

		TP_ARGS(fence),

		TP_STRUCT__entry(
				__field(uint32_t, fence)
		),

		TP_fast_assign(
				__entry->fence = fence;
		),

		TP_printk("fence=%u", __entry->fence)
);

#endif /* __MSM_TRACE_H__ */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#include <trace/define_trace.h>
