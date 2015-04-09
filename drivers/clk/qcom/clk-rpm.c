/*
 * Copyright (c) 2015, Linaro Limited
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <soc/qcom/rpm-smd.h>
#include "clk-rpm.h"

#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/clk-provider.h>

#include <soc/qcom/rpm-smd.h>

#include "clk-rpm.h"

#define __clk_rpmrs_set_rate(r, value, ctx) \
	((r)->rpmrs_data->set_rate_fn((r), (value), (ctx)))

#define clk_rpmrs_set_rate_sleep(r, value) \
	__clk_rpmrs_set_rate((r), (value), (r)->rpmrs_data->ctx_sleep_id)

#define clk_rpmrs_set_rate_active(r, value) \
	__clk_rpmrs_set_rate((r), (value), (r)->rpmrs_data->ctx_active_id)

static int clk_rpmrs_set_rate_smd(struct clk_rpm *r, u32 value, u32 context)
{
	struct msm_rpm_kvp kvp = {
		.key = r->rpm_key,
		.data = (void *)&value,
		.length = sizeof(value),
	};

	return msm_rpm_send_message(context, r->rpm_res_type, r->rpm_clk_id,
			&kvp, 1);
}

struct clk_rpmrs_data {
	int (*set_rate_fn)(struct clk_rpm *r, u32 value, u32 context);
	int (*get_rate_fn)(struct clk_rpm *r);
	int ctx_active_id;
	int ctx_sleep_id;
};

struct clk_rpmrs_data clk_rpmrs_data_smd = {
	.set_rate_fn = clk_rpmrs_set_rate_smd,
	.ctx_active_id = MSM_RPM_CTX_ACTIVE_SET,
	.ctx_sleep_id = MSM_RPM_CTX_SLEEP_SET,
};

static DEFINE_MUTEX(rpm_clock_lock);

static void to_active_sleep_khz(struct clk_rpm *r, unsigned long rate,
				unsigned long *active_khz,
				unsigned long *sleep_khz)
{
	/* Convert the rate (hz) to khz */
	*active_khz = DIV_ROUND_UP(rate, 1000);

	/*
	 * Active-only clocks don't care what the rate is during sleep. So,
	 * they vote for zero.
	 */
	if (r->active_only)
		*sleep_khz = 0;
	else
		*sleep_khz = *active_khz;
}

static int clk_rpm_prepare(struct clk_hw *hw)
{
	struct clk_rpm *r = to_clk_rpm(hw);
	u32 value;
	int rc = 0;
	unsigned long this_khz, this_sleep_khz;
	unsigned long peer_khz = 0, peer_sleep_khz = 0;
	struct clk_rpm *peer = r->peer;

	mutex_lock(&rpm_clock_lock);

	to_active_sleep_khz(r, r->rate, &this_khz, &this_sleep_khz);

	/* Don't send requests to the RPM if the rate has not been set. */
	if (this_khz == 0)
		goto out;

	/* Take peer clock's rate into account only if it's enabled. */
	if (peer->enabled)
		to_active_sleep_khz(peer, peer->rate,
				    &peer_khz, &peer_sleep_khz);

	value = max(this_khz, peer_khz);
	if (r->branch)
		value = !!value;

	rc = clk_rpmrs_set_rate_active(r, value);
	if (rc)
		goto out;

	value = max(this_sleep_khz, peer_sleep_khz);
	if (r->branch)
		value = !!value;

	rc = clk_rpmrs_set_rate_sleep(r, value);
	if (rc) {
		/* Undo the active set vote and restore it to peer_khz */
		value = peer_khz;
		rc = clk_rpmrs_set_rate_active(r, value);
	}

out:
	if (!rc)
		r->enabled = true;

	mutex_unlock(&rpm_clock_lock);

	return rc;
}

static void clk_rpm_unprepare(struct clk_hw *hw)
{
	struct clk_rpm *r = to_clk_rpm(hw);

	mutex_lock(&rpm_clock_lock);

	if (r->rate) {
		u32 value;
		struct clk_rpm *peer = r->peer;
		unsigned long peer_khz = 0, peer_sleep_khz = 0;
		int rc;

		/* Take peer clock's rate into account only if it's enabled. */
		if (peer->enabled)
			to_active_sleep_khz(peer, peer->rate,
					    &peer_khz, &peer_sleep_khz);

		value = r->branch ? !!peer_khz : peer_khz;
		rc = clk_rpmrs_set_rate_active(r, value);
		if (rc)
			goto out;

		value = r->branch ? !!peer_sleep_khz : peer_sleep_khz;
		rc = clk_rpmrs_set_rate_sleep(r, value);
	}
	r->enabled = false;
out:
	mutex_unlock(&rpm_clock_lock);
}

static int clk_rpm_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct clk_rpm *r = to_clk_rpm(hw);
	unsigned long this_khz, this_sleep_khz;
	int rc = 0;

	mutex_lock(&rpm_clock_lock);

	if (r->enabled) {
		u32 value;
		struct clk_rpm *peer = r->peer;
		unsigned long peer_khz = 0, peer_sleep_khz = 0;

		to_active_sleep_khz(r, rate, &this_khz, &this_sleep_khz);

		/* Take peer clock's rate into account only if it's enabled. */
		if (peer->enabled)
			to_active_sleep_khz(peer, peer->rate,
					    &peer_khz, &peer_sleep_khz);

		value = max(this_khz, peer_khz);
		rc = clk_rpmrs_set_rate_active(r, value);
		if (rc)
			goto out;

		value = max(this_sleep_khz, peer_sleep_khz);
		rc = clk_rpmrs_set_rate_sleep(r, value);
	}

	r->rate = rate;
out:
	mutex_unlock(&rpm_clock_lock);

	return rc;
}

static int clk_rpm_branch_set_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long parent_rate)
{
	struct clk_rpm *r = to_clk_rpm(hw);

	if (rate == r->rate)
		return 0;

	return -EPERM;
}

static long clk_rpm_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *parent_rate)
{
	return rate;
}

static unsigned long clk_rpm_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct clk_rpm *r = to_clk_rpm(hw);

	if (r->rpmrs_data->get_rate_fn)
		return r->rpmrs_data->get_rate_fn(r);

	return r->rate;
}

int clk_rpm_enable_scaling(void)
{
	int rc, value = 0x1;
	struct msm_rpm_kvp kvp = {
		.key = RPM_SMD_KEY_ENABLE,
		.data = (void *)&value,
		.length = sizeof(value),
	};

	rc = msm_rpm_send_message_noirq(MSM_RPM_CTX_SLEEP_SET,
					RPM_MISC_CLK_TYPE,
					RPM_SCALING_ENABLE_ID, &kvp, 1);
	if (rc < 0) {
		if (rc != -EPROBE_DEFER)
			pr_err("RPM clock scaling (sleep set) did not enable!\n");
		return rc;
	}

	rc = msm_rpm_send_message_noirq(MSM_RPM_CTX_ACTIVE_SET,
					RPM_MISC_CLK_TYPE,
					RPM_SCALING_ENABLE_ID, &kvp, 1);
	if (rc < 0) {
		if (rc != -EPROBE_DEFER)
			pr_err("RPM clock scaling (active set) did not enable!\n");
		return rc;
	}

	pr_debug("%s: Enabled RPM scaling\n", __func__);
	return 0;
}

const struct clk_ops clk_rpm_ops = {
	.prepare = clk_rpm_prepare,
	.unprepare = clk_rpm_unprepare,
	.set_rate = clk_rpm_set_rate,
	.round_rate = clk_rpm_round_rate,
	.recalc_rate = clk_rpm_recalc_rate,
};
EXPORT_SYMBOL_GPL(clk_rpm_ops);

const struct clk_ops clk_rpm_branch_ops = {
	.prepare = clk_rpm_prepare,
	.unprepare = clk_rpm_unprepare,
	.set_rate = clk_rpm_branch_set_rate,
	.round_rate = clk_rpm_round_rate,
	.recalc_rate = clk_rpm_recalc_rate,
};
EXPORT_SYMBOL_GPL(clk_rpm_branch_ops);
