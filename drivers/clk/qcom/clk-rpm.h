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

#define RPM_MISC_CLK_TYPE			0x306b6c63
#define RPM_BUS_CLK_TYPE			0x316b6c63
#define RPM_MEM_CLK_TYPE			0x326b6c63
#define RPM_CLK_BUFFER_A_REQ			0x616B6C63
#define RPM_KEY_SOFTWARE_ENABLE			0x6E657773
#define RPM_KEY_PIN_CTRL_CLK_BUFFER_ENABLE_KEY	0x62636370
#define RPM_SMD_KEY_RATE			0x007A484B
#define RPM_SMD_KEY_ENABLE			0x62616e45
#define RPM_SMD_KEY_STATE			0x54415453
#define RPM_SCALING_ENABLE_ID			0x2

struct clk_rpm {
	const int rpm_res_type;
	const int rpm_key;
	const int rpm_clk_id;
	const int rpm_status_id;
	const bool active_only;
	bool enabled;
	bool branch;
	struct clk_rpmrs_data *rpmrs_data;
	struct clk_rpm *peer;
	struct clk_hw hw;
	unsigned long rate;
};

extern const struct clk_ops clk_rpm_ops;
extern const struct clk_ops clk_rpm_branch_ops;
extern struct clk_rpmrs_data clk_rpmrs_data_smd;
int clk_rpm_enable_scaling(void);

#define to_clk_rpm(_hw) container_of(_hw, struct clk_rpm, hw)

#define __DEFINE_CLK_RPM(_name, active, type, r_id, stat_id, dep, key, \
			 rpmrsdata) \
	static struct clk_rpm active; \
	static struct clk_rpm _name = { \
		.rpm_res_type = (type), \
		.rpm_clk_id = (r_id), \
		.rpm_status_id = (stat_id), \
		.rpm_key = (key), \
		.peer = &active, \
		.rpmrs_data = (rpmrsdata),\
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
		.rpmrs_data = (rpmrsdata),\
		.rate = INT_MAX, \
		.hw.init = &(struct clk_init_data){ \
			.ops = &clk_rpm_ops, \
			.name = #active, \
			.flags = CLK_IS_ROOT, \
		}, \
	};

#define __DEFINE_CLK_RPM_BRANCH(_name, active, type, r_id, stat_id, r, \
				key, rpmrsdata) \
	static struct clk_rpm active; \
	static struct clk_rpm _name = { \
		.rpm_res_type = (type), \
		.rpm_clk_id = (r_id), \
		.rpm_status_id = (stat_id), \
		.rpm_key = (key), \
		.peer = &active, \
		.branch = true, \
		.rpmrs_data = (rpmrsdata),\
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
		.rpmrs_data = (rpmrsdata),\
		.rate = (r), \
		.hw.init = &(struct clk_init_data){ \
			.ops = &clk_rpm_branch_ops, \
			.name = #active, \
			.flags = CLK_IS_ROOT, \
		}, \
	};

#define DEFINE_CLK_RPM_SMD(_name, active, type, r_id, dep) \
		__DEFINE_CLK_RPM(_name, active, type, r_id, 0, dep, \
		RPM_SMD_KEY_RATE, &clk_rpmrs_data_smd)

#define DEFINE_CLK_RPM_SMD_BRANCH(_name, active, type, r_id, r) \
		__DEFINE_CLK_RPM_BRANCH(_name, active, type, r_id, 0, r, \
		RPM_SMD_KEY_ENABLE, &clk_rpmrs_data_smd)

#define DEFINE_CLK_RPM_SMD_QDSS(_name, active, type, r_id) \
		__DEFINE_CLK_RPM(_name, active, type, r_id, \
		0, 0, RPM_SMD_KEY_STATE, &clk_rpmrs_data_smd)
/*
 * The RPM XO buffer clock management code aggregates votes for pin-control mode
 * and software mode separately. Software-enable has higher priority over pin-
 * control, and if the software-mode aggregation results in a 'disable', the
 * buffer will be left in pin-control mode if a pin-control vote is in place.
 */
#define DEFINE_CLK_RPM_SMD_XO_BUFFER(_name, active, r_id) \
		__DEFINE_CLK_RPM_BRANCH(_name, active, RPM_CLK_BUFFER_A_REQ, \
		r_id, 0, 1000, RPM_KEY_SOFTWARE_ENABLE, &clk_rpmrs_data_smd)

#define DEFINE_CLK_RPM_SMD_XO_BUFFER_PINCTRL(_name, active, r_id) \
		__DEFINE_CLK_RPM_BRANCH(_name, active, RPM_CLK_BUFFER_A_REQ, \
		r_id, 0, 1000, RPM_KEY_PIN_CTRL_CLK_BUFFER_ENABLE_KEY, \
		&clk_rpmrs_data_smd)

#endif
