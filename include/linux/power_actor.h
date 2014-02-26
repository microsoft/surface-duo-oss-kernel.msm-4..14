/*
 * Copyright (C) 2014 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __POWER_ACTOR_H__
#define __POWER_ACTOR_H__

#include <linux/list.h>
#include <linux/mutex.h>

struct power_actor;

/**
 * struct power_actor_ops - callbacks for power actors
 * @get_req_power:	return the current requested power in milliwatts
 * @get_max_power:	return the max power that the device can currently
 *			consume in milliwatts
 * @set_power:		configure the device to consume a certain power in
 *			milliwatts
 */
struct power_actor_ops {
	u32 (*get_req_power)(struct power_actor *,
			struct thermal_zone_device *);
	u32 (*get_max_power)(struct power_actor *,
			struct thermal_zone_device *);
	int (*set_power)(struct power_actor *, struct thermal_zone_device *,
			u32);
};

/**
 * struct power_actor - structure for a power actor
 * @ops:	callbacks for the power actor
 * @data:	a private pointer for type-specific data
 * @actor_node:	node in actor_list
 */
struct power_actor {
	struct power_actor_ops *ops;
	void *data;
	struct list_head actor_node;
};

struct power_actor *power_actor_register(struct power_actor_ops *ops,
					void *privdata);
void power_actor_unregister(struct power_actor *actor);

extern struct list_head actor_list;

#endif /* __POWER_ACTOR_H__ */
