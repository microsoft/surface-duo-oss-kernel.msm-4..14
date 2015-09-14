/*
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015 Linaro Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/clk.h>

#include "msm_vidc_internal.h"
#include "msm_vidc_common.h"
#include "msm_vidc_debug.h"

enum load_quirks {
	LOAD_NO_QUIRKS			= 0 << 0,
	LOAD_IGNORE_TURBO_LOAD		= 1 << 0,
	LOAD_IGNORE_THUMBNAIL_LOAD	= 1 << 1,
};

static inline bool is_turbo_session(struct vidc_inst *inst)
{
	return !!(inst->flags & VIDC_TURBO);
}

static inline bool is_thumbnail_session(struct vidc_inst *inst)
{
	return !!(inst->flags & VIDC_THUMBNAIL);
}

static inline bool is_nominal_session(struct vidc_inst *inst)
{
	return !!(inst->flags & VIDC_NOMINAL);
}

static u32 get_mbs_per_sec(struct vidc_inst *inst)
{
	int mbs;
	u32 w = inst->width;
	u32 h = inst->height;

	mbs = (ALIGN(w, 16) / 16) * (ALIGN(h, 16) / 16);

	return mbs * inst->fps;
}

static u32 get_inst_load(struct vidc_inst *inst, enum load_quirks quirks)
{
	u32 load;

	if (!(inst->state >= INST_OPEN && inst->state < INST_STOP))
		return 0;

	load = get_mbs_per_sec(inst);

	if (is_thumbnail_session(inst) && quirks & LOAD_IGNORE_THUMBNAIL_LOAD)
		load = 0;

	if (is_turbo_session(inst) && !(quirks & LOAD_IGNORE_TURBO_LOAD))
		load = inst->core->res.max_load;

	return load;
}

static u32 get_load(struct vidc_core *core, enum session_type type,
		    enum load_quirks quirks)
{
	struct vidc_inst *inst = NULL;
	u32 mbs_per_sec = 0;

	mutex_lock(&core->lock);
	list_for_each_entry(inst, &core->instances, list) {
		if (inst->session_type != type)
			continue;

		mbs_per_sec += get_inst_load(inst, quirks);
	}
	mutex_unlock(&core->lock);

	return mbs_per_sec;
}

static int scale_clocks_load(struct vidc_core *core, u32 mbs_per_sec)
{
	struct clock_info *ci = &core->res.clock_set.clock_tbl[0];
	const struct load_freq_table *table = ci->load_freq_tbl;
	unsigned long freq = table[0].freq;
	int num_rows = ci->count;
	int ret, i;

	if (!mbs_per_sec && num_rows > 1) {
		freq = table[num_rows - 1].freq;
		goto set_freq;
	}

	for (i = 0; i < num_rows; i++) {
		if (mbs_per_sec > table[i].load)
			break;
		freq = table[i].freq;
	}

set_freq:

	ret = clk_set_rate(ci->clk, freq);
	if (ret) {
		dprintk(VIDC_ERR, "Failed to set clock rate %lu (%d)\n",
			freq, ret);
		return ret;
	}

	return 0;
}

static u32 scale_clocks(struct vidc_core *core)
{
	u32 mbs_per_sec = get_load(core, VIDC_ENCODER, LOAD_NO_QUIRKS) +
			  get_load(core, VIDC_DECODER, LOAD_NO_QUIRKS);

	return scale_clocks_load(core, mbs_per_sec);
}

void msm_comm_scale_clocks(struct vidc_core *core)
{
	if (scale_clocks(core)) {
		dprintk(VIDC_WARN, "Failed to scale clocks. "
			"Performance might be impacted\n");
	}
}

int msm_comm_check_overloaded(struct vidc_core *core)
{
	enum load_quirks quirks;
	u32 mbs_per_sec;

	quirks = LOAD_IGNORE_TURBO_LOAD | LOAD_IGNORE_THUMBNAIL_LOAD;

	mbs_per_sec = get_load(core, VIDC_DECODER, quirks) +
		      get_load(core, VIDC_ENCODER, quirks);

	if (mbs_per_sec > core->res.max_load) {
		dprintk(VIDC_ERR, "%s: HW is overloaded, needed: %d max: %d\n",
			__func__, mbs_per_sec, core->res.max_load);
		return -EBUSY;
	}

	return 0;
}
