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
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "clk-rpm.h"

static int clk_rpm_set_rate_active(struct clk_rpm *r, unsigned long value)
{
	struct rpm_clk_req req = {
		.key = QCOM_RPM_SMD_KEY_RATE,
		.nbytes = sizeof(u32),
		.value = value,
	};

	return qcom_rpm_smd_write(r->rpm, QCOM_SMD_RPM_ACTIVE_STATE,
				  r->rpm_res_type, r->rpm_clk_id, &req,
				  sizeof(req));
}


static int clk_rpm_prepare(struct clk_hw *hw)
{
	struct clk_rpm *r = to_clk_rpm(hw);
	struct clk_rpm *peer = r->peer;
	unsigned long this_khz, peer_khz = 0;
	u32 value;
	int rc = 0;

	/* Convert to KHz */
	this_khz = DIV_ROUND_UP(r->rate, 1000);

	/* Don't send requests to the RPM if the rate has not been set. */
	if (this_khz == 0)
		goto out;

	/* Take peer clock's rate into account only if it's enabled. */
	if (peer->enabled)
		peer_khz = DIV_ROUND_UP(peer->rate, 1000);

	value = max(this_khz, peer_khz);
	if (r->branch)
		value = !!value;

	rc = clk_rpm_set_rate_active(r, value);
	if (rc)
		goto out;

out:
	if (!rc)
		r->enabled = true;

	return rc;
}

static void clk_rpm_unprepare(struct clk_hw *hw)
{
	struct clk_rpm *r = to_clk_rpm(hw);

	if (r->rate) {
		struct clk_rpm *peer = r->peer;
		unsigned long peer_khz = 0;
		u32 value;
		int rc;

		/* Take peer clock's rate into account only if it's enabled. */
		if (peer->enabled)
			peer_khz = DIV_ROUND_UP(peer->rate, 1000);

		value = r->branch ? !!peer_khz : peer_khz;
		rc = clk_rpm_set_rate_active(r, value);
		if (rc)
			return;
	}
	r->enabled = false;
}

static int clk_rpm_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct clk_rpm *r = to_clk_rpm(hw);
	unsigned long this_khz;
	int rc = 0;

	if (r->enabled) {
		u32 value;
		struct clk_rpm *peer = r->peer;
		unsigned long peer_khz = 0;

		this_khz = DIV_ROUND_UP(rate, 1000);

		/* Take peer clock's rate into account only if it's enabled. */
		if (peer->enabled)
			peer_khz = DIV_ROUND_UP(peer->rate, 1000);

		value = max(this_khz, peer_khz);
		rc = clk_rpm_set_rate_active(r, value);
		if (rc)
			goto out;
	}

	r->rate = rate;
out:
	return rc;
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

	return r->rate;
}

int clk_rpm_enable_scaling(struct qcom_smd_rpm *rpm)
{
	int rc;
	struct rpm_clk_req req = {
		.key = QCOM_RPM_SMD_KEY_ENABLE,
		.nbytes = sizeof(u32),
		.value = 1,
	};

	rc = qcom_rpm_smd_write(rpm, QCOM_SMD_RPM_ACTIVE_STATE,
				QCOM_SMD_RPM_MISC_CLK,
				QCOM_RPM_SCALING_ENABLE_ID, &req, sizeof(req));
	if (rc < 0) {
		if (rc != -EPROBE_DEFER)
			pr_err("RPM clock scaling (active set) not enabled!\n");
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
	.round_rate = clk_rpm_round_rate,
	.recalc_rate = clk_rpm_recalc_rate,
};
EXPORT_SYMBOL_GPL(clk_rpm_branch_ops);
