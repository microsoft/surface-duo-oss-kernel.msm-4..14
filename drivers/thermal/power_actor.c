/*
 * Basic interface for power actors
 *
 * Copyright (C) 2014 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "Power actor: " fmt

#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/power_actor.h>
#include <linux/rculist.h>
#include <linux/slab.h>

LIST_HEAD(actor_list);
static DEFINE_MUTEX(actor_list_lock);
static int actor_id = 0;

/**
 * power_actor_register() - Register an actor in the power actor API
 * @weight:	weight of the actor as an 8-bit fixed point
 * @ops:	&struct power_actor_ops for this actor
 * @privdata:	pointer to private data related to the actor
 *
 * Return: The &struct power_actor * on success, ERR_PTR() on failure
 */
struct power_actor *power_actor_register(u32 weight,
					struct power_actor_ops *ops,
					void *privdata)
{
	struct power_actor *actor;
	char actor_name[32];

	if (!ops->get_req_power || !ops->get_max_power || !ops->set_power)
		return ERR_PTR(-EINVAL);

	actor = kzalloc(sizeof(*actor), GFP_KERNEL);
	if (!actor)
		return ERR_PTR(-ENOMEM);

	actor->weight = weight;
	actor->ops = ops;
	actor->data = privdata;

	snprintf(actor_name, 16, "actor%d", actor_id++);
	actor->debugfs_file = debugfs_create_u32(actor_name,
						S_IRUGO | S_IWUSR,
						power_allocator_d,
						&actor->weight);

	mutex_lock(&actor_list_lock);
	list_add_rcu(&actor->actor_node, &actor_list);
	mutex_unlock(&actor_list_lock);

	return actor;
}
EXPORT_SYMBOL(power_actor_register);

/**
 * power_actor_unregister() - Unregister an actor
 * @actor:	the actor to unregister
 */
void power_actor_unregister(struct power_actor *actor)
{
	mutex_lock(&actor_list_lock);
	list_del_rcu(&actor->actor_node);
	mutex_unlock(&actor_list_lock);
	synchronize_rcu();

	kfree(actor);
}
EXPORT_SYMBOL(power_actor_unregister);
