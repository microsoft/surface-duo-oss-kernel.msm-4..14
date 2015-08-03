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

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/soc/qcom/smd-rpm.h>

#include "clk-smd-rpm.h"

#define to_clk_smd_rpm(_hw) container_of(_hw, struct clk_smd_rpm, hw)

static DEFINE_MUTEX(rpm_clk_lock);

static int clk_smd_rpm_set_rate_active(struct clk_smd_rpm *r,
				       unsigned long value)
{
	struct clk_smd_rpm_req req = {
		.key = r->rpm_key,
		.nbytes = sizeof(u32),
		.value = DIV_ROUND_UP(value, 1000), /* RPM expects kHz */
	};

	return qcom_rpm_smd_write(r->rpm, QCOM_SMD_RPM_ACTIVE_STATE,
				  r->rpm_res_type, r->rpm_clk_id, &req,
				  sizeof(req));
}

static int clk_smd_rpm_set_rate_sleep(struct clk_smd_rpm *r,
				      unsigned long value)
{
	struct clk_smd_rpm_req req = {
		.key = r->rpm_key,
		.nbytes = sizeof(u32),
		.value = DIV_ROUND_UP(value, 1000), /* RPM expects kHz */
	};

	return qcom_rpm_smd_write(r->rpm, QCOM_SMD_RPM_SLEEP_STATE,
				  r->rpm_res_type, r->rpm_clk_id, &req,
				  sizeof(req));
}

static void to_active_sleep(struct clk_smd_rpm *r, unsigned long rate,
			    unsigned long *active, unsigned long *sleep)
{
	*active = rate;

	/*
	 * Active-only clocks don't care what the rate is during sleep. So,
	 * they vote for zero.
	 */
	if (r->active_only)
		*sleep = 0;
	else
		*sleep = *active;
}

static int clk_smd_rpm_prepare(struct clk_hw *hw)
{
	struct clk_smd_rpm *r = to_clk_smd_rpm(hw);
	struct clk_smd_rpm *peer = r->peer;
	unsigned long this_rate = 0, this_sleep_rate = 0;
	unsigned long peer_rate = 0, peer_sleep_rate = 0;
	unsigned long active_rate, sleep_rate;
	int ret = 0;

	mutex_lock(&rpm_clk_lock);

	/* Don't send requests to the RPM if the rate has not been set. */
	if (!r->rate)
		goto out;

	to_active_sleep(r, r->rate, &this_rate, &this_sleep_rate);

	/* Take peer clock's rate into account only if it's enabled. */
	if (peer->enabled)
		to_active_sleep(peer, peer->rate,
				&peer_rate, &peer_sleep_rate);

	active_rate = max(this_rate, peer_rate);

	if (r->branch)
		active_rate = !!active_rate;

	ret = clk_smd_rpm_set_rate_active(r, active_rate);
	if (ret)
		goto out;

	sleep_rate = max(this_sleep_rate, peer_sleep_rate);
	if (r->branch)
		sleep_rate = !!sleep_rate;

	ret = clk_smd_rpm_set_rate_sleep(r, sleep_rate);
	if (ret)
		/* Undo the active set vote and restore it */
		ret = clk_smd_rpm_set_rate_active(r, peer_rate);

out:
	if (!ret)
		r->enabled = true;

	mutex_unlock(&rpm_clk_lock);

	return ret;
}

static void clk_smd_rpm_unprepare(struct clk_hw *hw)
{
	struct clk_smd_rpm *r = to_clk_smd_rpm(hw);

	mutex_lock(&rpm_clk_lock);

	if (r->rate) {
		struct clk_smd_rpm *peer = r->peer;
		unsigned long peer_rate = 0, peer_sleep_rate = 0;
		unsigned long active_rate, sleep_rate;
		int ret;

		/* Take peer clock's rate into account only if it's enabled. */
		if (peer->enabled)
			to_active_sleep(peer, peer->rate, &peer_rate,
					&peer_sleep_rate);

		active_rate = r->branch ? !!peer_rate : peer_rate;
		ret = clk_smd_rpm_set_rate_active(r, active_rate);
		if (ret)
			goto out;

		sleep_rate = r->branch ? !!peer_sleep_rate : peer_sleep_rate;
		ret = clk_smd_rpm_set_rate_sleep(r, sleep_rate);
 		if (ret)
			goto out;
	}
	r->enabled = false;

out:
	mutex_unlock(&rpm_clk_lock);
}

static int clk_smd_rpm_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct clk_smd_rpm *r = to_clk_smd_rpm(hw);
	int ret = 0;

	mutex_lock(&rpm_clk_lock);

	if (r->enabled) {
		struct clk_smd_rpm *peer = r->peer;
		unsigned long active_rate, sleep_rate;
		unsigned long this_rate = 0, this_sleep_rate = 0;
		unsigned long peer_rate = 0, peer_sleep_rate = 0;

		to_active_sleep(r, rate, &this_rate, &this_sleep_rate);

		/* Take peer clock's rate into account only if it's enabled. */
		if (peer->enabled)
			to_active_sleep(peer, peer->rate,
					&peer_rate, &peer_sleep_rate);

		active_rate = max(this_rate, peer_rate);
		ret = clk_smd_rpm_set_rate_active(r, active_rate);
		if (ret)
			goto out;

		sleep_rate = max(this_sleep_rate, peer_sleep_rate);
		ret = clk_smd_rpm_set_rate_sleep(r, sleep_rate);
		if (ret)
			goto out;
	}
	r->rate = rate;
out:
	mutex_unlock(&rpm_clk_lock);

	return ret;
}

static long clk_smd_rpm_round_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long *parent_rate)
{

	/*
 	 * RPM handles rate rounding and we don't have a way to
	 * know what the rate will be, so just return whatever
	 * rate is requested.
	 */
	return rate;
}

static unsigned long clk_smd_rpm_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct clk_smd_rpm *r = to_clk_smd_rpm(hw);

	/*
 	 * RPM handles rate rounding and we don't have a way to
	 * know what the rate will be, so just return whatever
	 * rate was set.
	 */
	return r->rate;
}

int clk_smd_rpm_enable_scaling(struct qcom_smd_rpm *rpm)
{
	int ret;
	struct clk_smd_rpm_req req = {
		.key = QCOM_RPM_SMD_KEY_ENABLE,
		.nbytes = sizeof(u32),
		.value = 1,
	};

	ret = qcom_rpm_smd_write(rpm, QCOM_SMD_RPM_SLEEP_STATE,
				QCOM_SMD_RPM_MISC_CLK,
				QCOM_RPM_SCALING_ENABLE_ID, &req, sizeof(req));
	if (ret) {
		pr_err("RPM clock scaling (sleep set) not enabled!\n");
		return ret;
	}

	ret = qcom_rpm_smd_write(rpm, QCOM_SMD_RPM_ACTIVE_STATE,
				 QCOM_SMD_RPM_MISC_CLK,
				 QCOM_RPM_SCALING_ENABLE_ID, &req, sizeof(req));
	if (ret) {
		pr_err("RPM clock scaling (active set) not enabled!\n");
		return ret;
	}

	pr_debug("%s: RPM clock scaling is enabled\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(clk_smd_rpm_enable_scaling);

const struct clk_ops clk_smd_rpm_ops = {
	.prepare = clk_smd_rpm_prepare,
	.unprepare = clk_smd_rpm_unprepare,
	.set_rate = clk_smd_rpm_set_rate,
	.round_rate = clk_smd_rpm_round_rate,
	.recalc_rate = clk_smd_rpm_recalc_rate,
};
EXPORT_SYMBOL_GPL(clk_smd_rpm_ops);

const struct clk_ops clk_smd_rpm_branch_ops = {
	.prepare = clk_smd_rpm_prepare,
	.unprepare = clk_smd_rpm_unprepare,
	.round_rate = clk_smd_rpm_round_rate,
	.recalc_rate = clk_smd_rpm_recalc_rate,
};
EXPORT_SYMBOL_GPL(clk_smd_rpm_branch_ops);
