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

#ifndef __QCOM_CLK_RPM_H__
#define __QCOM_CLK_RPM_H__

#include <linux/clk-provider.h>
#include <linux/mfd/qcom-smd-rpm.h>

#define QCOM_RPM_KEY_SOFTWARE_ENABLE			0x6e657773
#define QCOM_RPM_KEY_PIN_CTRL_CLK_BUFFER_ENABLE_KEY	0x62636370
#define QCOM_RPM_SMD_KEY_RATE				0x007a484b
#define QCOM_RPM_SMD_KEY_ENABLE				0x62616e45
#define QCOM_RPM_SMD_KEY_STATE				0x54415453
#define QCOM_RPM_SCALING_ENABLE_ID			0x2

struct clk_rpm {
	const int rpm_res_type;
	const int rpm_key;
	const int rpm_clk_id;
	const int rpm_status_id;
	const bool active_only;
	bool enabled;
	bool branch;
	struct clk_rpm *peer;
	struct clk_hw hw;
	unsigned long rate;
	struct qcom_smd_rpm *rpm;
};

struct rpm_clk_req {
	u32 key;
	u32 nbytes;
	u32 value;
};

extern const struct clk_ops clk_rpm_ops;
extern const struct clk_ops clk_rpm_branch_ops;
int clk_rpm_enable_scaling(struct qcom_smd_rpm *rpm);

#define to_clk_rpm(_hw) container_of(_hw, struct clk_rpm, hw)

#define __DEFINE_CLK_RPM(_name, active, type, r_id, stat_id, dep, key) \
	static struct clk_rpm active; \
	static struct clk_rpm _name = { \
		.rpm_res_type = (type), \
		.rpm_clk_id = (r_id), \
		.rpm_status_id = (stat_id), \
		.rpm_key = (key), \
		.peer = &active, \
		.rate = INT_MAX, \
		.hw.init = &(struct clk_init_data){ \
			.ops = &clk_rpm_ops, \
			.name = #_name, \
			.flags = CLK_IS_ROOT, \
		}, \
	}; \
	static struct clk_rpm active = { \
		.rpm_res_type = (type), \
		.rpm_clk_id = (r_id), \
		.rpm_status_id = (stat_id), \
		.rpm_key = (key), \
		.peer = &_name, \
		.active_only = true, \
		.rate = INT_MAX, \
		.hw.init = &(struct clk_init_data){ \
			.ops = &clk_rpm_ops, \
			.name = #active, \
			.flags = CLK_IS_ROOT, \
		}, \
	};

#define __DEFINE_CLK_RPM_BRANCH(_name, active, type, r_id, stat_id, r, \
				key) \
	static struct clk_rpm active; \
	static struct clk_rpm _name = { \
		.rpm_res_type = (type), \
		.rpm_clk_id = (r_id), \
		.rpm_status_id = (stat_id), \
		.rpm_key = (key), \
		.peer = &active, \
		.branch = true, \
		.rate = (r), \
		.hw.init = &(struct clk_init_data){ \
			.ops = &clk_rpm_branch_ops, \
			.name = #_name, \
			.flags = CLK_IS_ROOT, \
		}, \
	}; \
	static struct clk_rpm active = { \
		.rpm_res_type = (type), \
		.rpm_clk_id = (r_id), \
		.rpm_status_id = (stat_id), \
		.rpm_key = (key), \
		.peer = &_name, \
		.active_only = true, \
		.branch = true, \
		.rate = (r), \
		.hw.init = &(struct clk_init_data){ \
			.ops = &clk_rpm_branch_ops, \
			.name = #active, \
			.flags = CLK_IS_ROOT, \
		}, \
	};

#define DEFINE_CLK_RPM_SMD(_name, active, type, r_id, dep) \
		__DEFINE_CLK_RPM(_name, active, type, r_id, 0, dep, \
		QCOM_RPM_SMD_KEY_RATE)

#define DEFINE_CLK_RPM_SMD_BRANCH(_name, active, type, r_id, r) \
		__DEFINE_CLK_RPM_BRANCH(_name, active, type, r_id, 0, r, \
		QCOM_RPM_SMD_KEY_ENABLE)

#define DEFINE_CLK_RPM_SMD_QDSS(_name, active, type, r_id) \
		__DEFINE_CLK_RPM(_name, active, type, r_id, \
		0, 0, QCOM_RPM_SMD_KEY_STATE)

#define DEFINE_CLK_RPM_SMD_XO_BUFFER(_name, active, r_id) \
		__DEFINE_CLK_RPM_BRANCH(_name, active, QCOM_SMD_RPM_CLK_BUF_A, \
		r_id, 0, 1000, QCOM_RPM_KEY_SOFTWARE_ENABLE)

#define DEFINE_CLK_RPM_SMD_XO_BUFFER_PINCTRL(_name, active, r_id) \
		__DEFINE_CLK_RPM_BRANCH(_name, active, QCOM_SMD_RPM_CLK_BUF_A, \
		r_id, 0, 1000, QCOM_RPM_KEY_PIN_CTRL_CLK_BUFFER_ENABLE_KEY)

#endif
