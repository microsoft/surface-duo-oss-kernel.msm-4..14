/*
 * A power allocator to manage temperature
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

#define pr_fmt(fmt) "Power allocator: " fmt

#include <linux/power_actor.h>
#include <linux/rculist.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#define CREATE_TRACE_POINTS
#include <trace/events/thermal_power_allocator.h>

#include "thermal_core.h"

#define FRAC_BITS 10
#define int_to_frac(x) ((x) << FRAC_BITS)
#define frac_to_int(x) ((x) >> FRAC_BITS)

/**
 * mul_frac() - multiply two fixed-point numbers
 * @x:	first multiplicand
 * @y:	second multiplicand
 *
 * Return: the result of multiplying two fixed-point numbers.  The
 * result is also a fixed-point number.
 */
static inline s64 mul_frac(s64 x, s64 y)
{
	return (x * y) >> FRAC_BITS;
}

enum power_allocator_trip_levels {
	TRIP_SWITCH_ON = 0,	/* Switch on PID controller */
	TRIP_MAX_DESIRED_TEMPERATURE, /* Temperature we are controlling for */

	THERMAL_TRIP_NUM,
};

/**
 * struct power_allocator_params - parameters for the power allocator governor
 * @k_po:	Proportional parameter of the PID controller when overshooting
 *		(i.e., when temperature is below the target)
 * @k_pi:	Proportional parameter of the PID controller when undershooting
 * @k_i:	Integral parameter of the PID controller
 * @k_d:	Derivative parameter of the PID controller
 * @integral_cutoff:	threshold below which the error is no longer accumulated
			in the PID controller
 * @err_integral:	accumulated error in the PID controller.
 * @prev_err:	error in the previous iteration of the PID controller.
 *		Used to calculate the derivative term.
 */
struct power_allocator_params {
	s32 k_po;
	s32 k_pu;
	s32 k_i;
	s32 k_d;
	s32 integral_cutoff;
	s32 err_integral;
	s32 prev_err;
};

/**
 * pid_controller() - PID controller
 * @tz:	thermal zone we are operating in
 * @current_temp:	the current temperature in millicelsius
 * @control_temp:	the target temperature in millicelsius
 * @max_allocatable_power:	maximum allocatable power for this thermal zone
 *
 * This PID controller increases the available power budget so that the
 * temperature of the thermal zone gets as close as possible to
 * @control_temp and limits the power if it exceeds it.  k_po is the
 * proportional term when we are overshooting, k_pu is the
 * proportional term when we are undershooting.  integral_cutoff is a
 * threshold below which we stop accumulating the error.  The
 * accumulated error is only valid if the requested power will make
 * the system warmer.  If the system is mostly idle, there's no point
 * in accumulating positive error.
 *
 * Return: The power budget for the next period.
 */
static u32 pid_controller(struct thermal_zone_device *tz,
			unsigned long current_temp, unsigned long control_temp,
			u32 max_allocatable_power)
{
	s64 p, i, d, power_range;
	s32 err, max_power_frac;
	struct power_allocator_params *params = tz->governor_data;

	max_power_frac = int_to_frac(max_allocatable_power);

	err = ((s32)control_temp - (s32)current_temp);
	err = int_to_frac(err);

	/* Calculate the proportional term */
	p = mul_frac(err < 0 ? params->k_po : params->k_pu, err);

	/*
	 * Calculate the integral term
	 *
	 * if the error s less than cut off allow integration (but
	 * the integral is limited to max power)
	 */
	i = mul_frac(params->k_i, params->err_integral);

	/*
	 * err is below the integral_cutoff and
	 * -max_allocatable_power < err_integral < max_allocatable_power
	 */
	if (err < int_to_frac(params->integral_cutoff)) {
		s64 tmpi = i + mul_frac(params->k_i, err);

		if ((-max_power_frac < tmpi) && (tmpi <= max_power_frac)) {
			i = tmpi;
			params->err_integral += err;
		}
	}

	/*
	 * Calculate the derivative term
	 *
	 * We do err - prev_err, so with a positive k_d, a decreasing
	 * error (i.e. driving closer to the line) results in less
	 * power being applied, slowing down the controller)
	 */
	d = mul_frac(params->k_d, err - params->prev_err);
	params->prev_err = err;

	power_range = p + i + d;

	/* feed-forward the known sustainable dissipatable power */
	power_range = tz->tzp->sustainable_power + frac_to_int(power_range);

	power_range = clamp(power_range, (s64)0, (s64)max_allocatable_power);

	trace_thermal_power_allocator_pid(frac_to_int(err),
					frac_to_int(params->err_integral),
					frac_to_int(p), frac_to_int(i),
					frac_to_int(d), power_range);

	return power_range;
}

/**
 * divvy_up_power() - divvy the allocated power between the actors
 * @req_power:	each actor's requested power
 * @max_power:	each actor's maximum available power
 * @num_actors:	size of the @req_power, @max_power and @granted_power's array
 * @total_req_power: sum of @req_power
 * @power_range:	total allocated power
 * @granted_power:	ouput array: each actor's granted power
 *
 * This function divides the total allocated power (@power_range)
 * fairly between the actors.  It first tries to give each actor a
 * share of the @power_range according to how much power it requested
 * compared to the rest of the actors.  For example, if only one actor
 * requests power, then it receives all the @power_range.  If
 * three actors each requests 1mW, each receives a third of the
 * @power_range.
 *
 * If any actor received more than their maximum power, then that
 * surplus is re-divvied among the actors based on how far they are
 * from their respective maximums.
 *
 * Granted power for each actor is written to @granted_power, which
 * should've been allocated by the calling function.
 */
static void divvy_up_power(u32 *req_power, u32 *max_power, int num_actors,
			u32 total_req_power, u32 power_range,
			u32 *granted_power)
{
	u32 extra_power, capped_extra_power, extra_actor_power[num_actors];
	int i;

	if (!total_req_power) {
		/*
		 * Nobody requested anything, so just give everybody
		 * the maximum power
		 */
		for (i = 0; i < num_actors; i++)
			granted_power[i] = max_power[i];

		return;
	}

	capped_extra_power = 0;
	extra_power = 0;
	for (i = 0; i < num_actors; i++) {
		u64 req_range = req_power[i] * power_range;

		granted_power[i] = div_u64(req_range, total_req_power);

		if (granted_power[i] > max_power[i]) {
			extra_power += granted_power[i] - max_power[i];
			granted_power[i] = max_power[i];
		}

		extra_actor_power[i] = max_power[i] - granted_power[i];
		capped_extra_power += extra_actor_power[i];
	}

	if (!extra_power)
		return;

	/*
	 * Re-divvy the reclaimed extra among actors based on
	 * how far they are from the max
	 */
	extra_power = min(extra_power, capped_extra_power);
	if (capped_extra_power > 0)
		for (i = 0; i < num_actors; i++)
			granted_power[i] += (extra_actor_power[i] *
					extra_power) / capped_extra_power;
}

static int allocate_power(struct thermal_zone_device *tz,
			unsigned long current_temp, unsigned long control_temp)
{
	struct power_actor *actor;
	u32 *req_power, *max_power, *granted_power;
	u32 total_req_power, max_allocatable_power;
	u32 total_granted_power, power_range;
	int i, num_actors, ret = 0;

	mutex_lock(&tz->lock);
	rcu_read_lock();

	num_actors = 0;
	list_for_each_entry_rcu(actor, &actor_list, actor_node)
		num_actors++;

	req_power = devm_kcalloc(&tz->device, num_actors, sizeof(*req_power),
				GFP_KERNEL);
	if (!req_power) {
		ret = -ENOMEM;
		goto unlock;
	}

	max_power = devm_kcalloc(&tz->device, num_actors, sizeof(*max_power),
				GFP_KERNEL);
	if (!max_power) {
		ret = -ENOMEM;
		goto free_req_power;
	}

	granted_power = devm_kcalloc(&tz->device, num_actors,
				sizeof(*granted_power), GFP_KERNEL);
	if (!granted_power) {
		ret = -ENOMEM;
		goto free_max_power;
	}

	i = 0;
	total_req_power = 0;
	max_allocatable_power = 0;

	list_for_each_entry_rcu(actor, &actor_list, actor_node) {
		req_power[i] = actor->ops->get_req_power(actor, tz);
		req_power[i] = frac_to_int(actor->weight * req_power[i]);
		total_req_power += req_power[i];

		max_power[i] = actor->ops->get_max_power(actor, tz);
		max_allocatable_power += max_power[i];

		i++;
	}

	power_range = pid_controller(tz, current_temp, control_temp,
				max_allocatable_power);

	divvy_up_power(req_power, max_power, num_actors, total_req_power,
		power_range, granted_power);

	total_granted_power = 0;
	i = 0;
	list_for_each_entry_rcu(actor, &actor_list, actor_node) {
		actor->ops->set_power(actor, tz, granted_power[i]);
		total_granted_power += granted_power[i];

		i++;
	}

	trace_thermal_power_allocator(req_power, total_req_power, granted_power,
				total_granted_power, num_actors, power_range,
				max_allocatable_power, current_temp,
				(s32)control_temp - (s32)current_temp);

	devm_kfree(&tz->device, granted_power);
free_max_power:
	devm_kfree(&tz->device, max_power);
free_req_power:
	devm_kfree(&tz->device, req_power);
unlock:
	rcu_read_unlock();
	mutex_unlock(&tz->lock);

	return ret;
}

static int check_trips(struct thermal_zone_device *tz)
{
	int ret;
	enum thermal_trip_type type;

	if (tz->trips < THERMAL_TRIP_NUM)
		return -EINVAL;

	ret = tz->ops->get_trip_type(tz, TRIP_SWITCH_ON, &type);
	if (ret)
		return ret;

	if ((type != THERMAL_TRIP_PASSIVE) && (type != THERMAL_TRIP_ACTIVE))
		return -EINVAL;

	ret = tz->ops->get_trip_type(tz, TRIP_MAX_DESIRED_TEMPERATURE, &type);
	if (ret)
		return ret;

	if ((type != THERMAL_TRIP_PASSIVE) && (type != THERMAL_TRIP_ACTIVE))
		return -EINVAL;

	return ret;
}

static void reset_pid_controller(struct power_allocator_params *params)
{
	params->err_integral = 0;
	params->prev_err = 0;
}

static void allow_maximum_power(struct thermal_zone_device *tz)
{
	struct power_actor *actor;

	rcu_read_lock();

	list_for_each_entry_rcu(actor, &actor_list, actor_node) {
		u32 max_power = actor->ops->get_max_power(actor, tz);

		actor->ops->set_power(actor, tz, max_power);
	}

	rcu_read_unlock();
}

#define debugfs_entry(name) \
	dentry_f = debugfs_create_u32( #name, S_IWUSR | S_IRUGO, power_allocator_d, &params->name); \
	if (IS_ERR_OR_NULL(dentry_f)) {					\
		pr_warn("Unable to create debugfsfile: " #name "\n"); \
		return; \
	}

static void create_debugfs_pi_params(struct power_allocator_params *params)
{
	struct dentry *dentry_f;

	debugfs_entry(k_po);
	debugfs_entry(k_pu);
	debugfs_entry(k_i);
	debugfs_entry(k_d);
	debugfs_entry(integral_cutoff);
	debugfs_entry(err_integral);
	debugfs_entry(prev_err);
}

#undef debugfs_entry

/**
 * power_allocator_bind() - bind the power_allocator governor to a thermal zone
 * @tz:	thermal zone to bind it to
 *
 * Check that the thermal zone is valid for this governor, that is, it
 * has two thermal trips.  If so, initialize the PID controller
 * parameters and bind it to the thermal zone.
 *
 * Return: 0 on success, -EINVAL if the trips were invalid or -ENOMEM
 * if we ran out of memory.
 */
static int power_allocator_bind(struct thermal_zone_device *tz)
{
	int ret;
	struct power_allocator_params *params;
	unsigned long switch_on_temp, control_temp;
	u32 temperature_threshold;

	ret = check_trips(tz);
	if (ret) {
		dev_err(&tz->device,
			"thermal zone %s has the wrong number of trips for this governor\n",
			tz->type);
		return ret;
	}

	if (!tz->tzp || !tz->tzp->sustainable_power) {
		dev_err(&tz->device,
			"power_allocator: missing sustainable_power\n");
		return -EINVAL;
	}

	params = devm_kzalloc(&tz->device, sizeof(*params), GFP_KERNEL);
	if (!params)
		return -ENOMEM;

	ret = tz->ops->get_trip_temp(tz, TRIP_SWITCH_ON, &switch_on_temp);
	if (ret)
		goto free;

	ret = tz->ops->get_trip_temp(tz, TRIP_MAX_DESIRED_TEMPERATURE,
				&control_temp);
	if (ret)
		goto free;

	temperature_threshold = control_temp - switch_on_temp;

	params->k_po = int_to_frac(tz->tzp->sustainable_power) /
		temperature_threshold;
	params->k_pu = int_to_frac(2 * tz->tzp->sustainable_power) /
		temperature_threshold;
	params->k_i = int_to_frac(10) / 1000;
	params->k_d = int_to_frac(0);
	params->integral_cutoff = 0;

	reset_pid_controller(params);

	create_debugfs_pi_params(params);

	tz->governor_data = params;

	return 0;

free:
	devm_kfree(&tz->device, params);
	return ret;
}

static void power_allocator_unbind(struct thermal_zone_device *tz)
{
	dev_dbg(&tz->device, "Unbinding from thermal zone %d\n", tz->id);
	devm_kfree(&tz->device, tz->governor_data);
	tz->governor_data = NULL;
}

static int power_allocator_throttle(struct thermal_zone_device *tz, int trip)
{
	int ret;
	unsigned long switch_on_temp, control_temp, current_temp;
	struct power_allocator_params *params = tz->governor_data;

	/*
	 * We get called for every trip point but we only need to do
	 * our calculations once
	 */
	if (trip != TRIP_MAX_DESIRED_TEMPERATURE)
		return 0;

	ret = thermal_zone_get_temp(tz, &current_temp);
	if (ret) {
		dev_warn(&tz->device, "Failed to get temperature: %d\n", ret);
		return ret;
	}

	ret = tz->ops->get_trip_temp(tz, TRIP_SWITCH_ON, &switch_on_temp);
	if (ret) {
		dev_warn(&tz->device,
			"Failed to get switch on temperature: %d\n", ret);
		return ret;
	}

	if (current_temp < switch_on_temp) {
		tz->passive = 0;
		reset_pid_controller(params);
		allow_maximum_power(tz);
		return 0;
	}

	tz->passive = 1;

	ret = tz->ops->get_trip_temp(tz, TRIP_MAX_DESIRED_TEMPERATURE,
				&control_temp);
	if (ret) {
		dev_warn(&tz->device,
			"Failed to get the maximum desired temperature: %d\n",
			ret);
		return ret;
	}

	return allocate_power(tz, current_temp, control_temp);
}

struct dentry *power_allocator_d;
static void create_debugfs_interface(void)
{
	power_allocator_d = debugfs_create_dir("power_allocator", NULL);
	if (IS_ERR_OR_NULL(power_allocator_d)) {
		pr_info("unable to create debugfs directory\n");
		return;
	}
}

static struct thermal_governor thermal_gov_power_allocator = {
	.name		= "power_allocator",
	.bind_to_tz	= power_allocator_bind,
	.unbind_from_tz	= power_allocator_unbind,
	.throttle	= power_allocator_throttle,
};

int thermal_gov_power_allocator_register(void)
{
	create_debugfs_interface();

	return thermal_register_governor(&thermal_gov_power_allocator);
}

void thermal_gov_power_allocator_unregister(void)
{
	thermal_unregister_governor(&thermal_gov_power_allocator);
}
