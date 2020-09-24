/* Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __FSM_TTI_INTR_IF__
#define __FSM_TTI_INTR_IF__

#include <linux/types.h>
#include <linux/ioctl.h>

#define FSM_TTI_MONOTONIC_CLOCK_MASK	1000000000
/* ioctl command */
#define FSM_TTI_IOCTL_BASE			't'
#define FSM_TTI_IOCTL_INITIAL_SFN_SLOT_INFO	\
		_IOW(FSM_TTI_IOCTL_BASE, 1, struct fsm_tti_mmap_info)

#define FSM_TTI_MAX_SFN_NUM		1024
#define FSM_TTI_MAX_SFN_MOD_FACTOR	0x3FF /* equivalent to MOD 1024 */

union sfn_slot_info
{
	uint32_t sfn_slot;
	struct {
		uint32_t sfn:16;
		uint32_t slot:16;
	};
};

struct fsm_tti_internal_stats {
	union sfn_slot_info	initial_sfn_slot;
	unsigned long long	current_tti_count;
	unsigned long long	first_tti_recv_time;
	unsigned long long	current_tti_recv_time;
	unsigned long long	sfn_slot_seeding_time;
};

struct fsm_tti_mmap_info {
	/* sfn information range 0-1023 and slot information range 0-79.
	 * In sfn_slot variable, MSB 16 bit will hold the sfn information
	 * and LSB 16 bit will hold slot information.
	 */
	union sfn_slot_info	sfn_slot_info;

	/* Timestamp Information */
	unsigned long long	intr_recv_count;
	unsigned long long	abs_recv_time;

	/* max slot number */
	unsigned short		max_slot;
};

#endif /* __FSM_TTI_INTR_IF__ */

