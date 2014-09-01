/*
 * A basic cpu power_actor
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

#define pr_fmt(fmt) "CPU actor: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/cpu_cooling.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/pm_opp.h>
#include <linux/power_actor.h>
#include <linux/printk.h>
#include <linux/slab.h>

#include <trace/events/thermal_power_allocator.h>

/**
 * struct power_table - frequency to power conversion
 * @frequency:	frequency in KHz
 * @power:	power in mW
 *
 * This structure is built when the cooling device registers and helps
 * in translating frequency to power and viceversa.
 */
struct power_table {
	u32 frequency;
	u32 power;
};

/**
 * struct cpu_actor - information for each cpu actor
 * @cpumask:	cpus covered by this actor
 * @last_load:	load measured by the latest call to cpu_get_req_power()
 * @time_in_idle:	previous reading of the absolute time that this cpu was
 *			idle
 * @time_in_idle_timestamp: wall time of the last invocation of
 *			    get_cpu_idle_time_us()
 * @dyn_power_table:	array of struct power_table for frequency to power
 *			conversion
 * @dyn_power_table_entries: number of entries in the @dyn_power_table array
 * @cdev:	cpufreq cooling device associated with this actor
 * @plat_get_static_power:	callback to calculate the static power
 */
struct cpu_actor {
	cpumask_t cpumask;
	u32 last_load;
	u64 time_in_idle[NR_CPUS];
	u64 time_in_idle_timestamp[NR_CPUS];
	struct power_table *dyn_power_table;
	int dyn_power_table_entries;
	struct thermal_cooling_device *cdev;
	get_static_t plat_get_static_power;
};

static u32 cpu_freq_to_power(struct cpu_actor *cpu_actor, u32 freq)
{
	int i;
	struct power_table *pt = cpu_actor->dyn_power_table;

	for (i = 1; i < cpu_actor->dyn_power_table_entries; i++)
		if (freq < pt[i].frequency)
			break;

	return pt[i - 1].power;
}

static u32 cpu_power_to_freq(struct cpu_actor *cpu_actor, u32 power)
{
	int i;
	struct power_table *pt = cpu_actor->dyn_power_table;

	for (i = 1; i < cpu_actor->dyn_power_table_entries; i++)
		if (power < pt[i].power)
			break;

	return pt[i - 1].frequency;
}

/**
 * get_load() - get load for a cpu since last updated
 * @cpu_actor:	&struct cpu_actor for this actor
 * @cpu:	cpu number
 *
 * Return: The average load of cpu @cpu in percentage since this
 * function was last called.
 */
static u32 get_load(struct cpu_actor *cpu_actor, int cpu)
{
	u32 load;
	u64 now, now_idle, delta_time, delta_idle;

	now_idle = get_cpu_idle_time(cpu, &now, 0);
	delta_idle = now_idle - cpu_actor->time_in_idle[cpu];
	delta_time = now - cpu_actor->time_in_idle_timestamp[cpu];

	if (delta_time <= delta_idle)
		load = 0;
	else
		load = div64_u64(100 * (delta_time - delta_idle), delta_time);

	cpu_actor->time_in_idle[cpu] = now_idle;
	cpu_actor->time_in_idle_timestamp[cpu] = now;

	return load;
}

/**
 * get_static_power() - calculate the static power consumed by the cpus
 * @cpu_actor:	&struct cpu_actor for this cpu
 * @tz:		&struct thermal_zone_device closest to the cpu
 * @freq:	frequency in KHz
 *
 * Calculate the static power consumed by the cpus described by
 * @cpu_actor running at frequency @freq.  This function relies on a
 * platform specific function that should have been provided when the
 * actor was registered.  If it wasn't, the static power is assumed to
 * be negligible.
 *
 * Return: The static power consumed by the cpus.  It returns 0 on
 * error or if there is no plat_get_static_power().
 */
static u32 get_static_power(struct cpu_actor *cpu_actor,
			struct thermal_zone_device *tz, unsigned long freq)
{
	struct device *cpu_dev;
	struct dev_pm_opp *opp;
	unsigned long voltage, temperature;
	cpumask_t *cpumask = &cpu_actor->cpumask;
	unsigned long freq_hz = freq * 1000;

	if (!cpu_actor->plat_get_static_power)
		return 0;

	if (freq == 0)
		return 0;

	cpu_dev = get_cpu_device(cpumask_any(cpumask));

	rcu_read_lock();

	opp = dev_pm_opp_find_freq_exact(cpu_dev, freq_hz, true);
	voltage = dev_pm_opp_get_voltage(opp);

	rcu_read_unlock();

	if (voltage == 0) {
		dev_warn_ratelimited(cpu_dev,
				"Failed to get voltage for frequency %lu: %ld\n",
				freq_hz, IS_ERR(opp) ? PTR_ERR(opp) : 0);
		return 0;
	}

	/*
	 * We can call thermal_zone_get_temp() here because this
	 * function may be called with tz->lock held.  Use the latest
	 * read temperature instead.
	 */
	temperature = (unsigned long)tz->temperature;

	return cpu_actor->plat_get_static_power(cpumask, voltage, temperature);
}

/**
 * get_dynamic_power() - calculate the dynamic power
 * @cpu_actor:	cpu_actor pointer
 * @freq:	current frequency
 *
 * Return: the dynamic power consumed by the cpus described by
 * @cpu_actor.
 */
static u32 get_dynamic_power(struct cpu_actor *cpu_actor, unsigned long freq)
{
	u32 power, raw_cpu_power;

	raw_cpu_power = cpu_freq_to_power(cpu_actor, freq);
	power = (raw_cpu_power * cpu_actor->last_load) / 100;

	return power;
}

/**
 * cpu_get_req_power() - get the current power
 * @actor:	power actor pointer
 * @tz:		&thermal_zone_device closest to the CPU
 *
 * Callback for the power actor to return the current power
 * consumption in milliwatts.
 */
static u32 cpu_get_req_power(struct power_actor *actor,
			struct thermal_zone_device *tz)
{
	int cpu;
	u32 static_power, dynamic_power, total_load = 0;
	unsigned long freq;
	struct cpu_actor *cpu_actor = actor->data;

	freq = cpufreq_quick_get(cpumask_any(&cpu_actor->cpumask));

	for_each_cpu(cpu, &cpu_actor->cpumask) {
		u32 load;

		if (cpu_online(cpu))
			load = get_load(cpu_actor, cpu);
		else
			load = 0;

		total_load += load;
	}

	cpu_actor->last_load = total_load;

	static_power = get_static_power(cpu_actor, tz, freq);
	dynamic_power = get_dynamic_power(cpu_actor, freq);

	return static_power + dynamic_power;
}

/**
 * cpu_get_max_power() - get the maximum power that the cpu could currently consume
 * @actor:	power actor pointer
 * @tz:		&thermal_zone_device closest to the CPU
 *
 * Callback for the power actor to return the maximum power
 * consumption in milliwatts that the cpu could currently consume.
 * The static power depends on temperature so the maximum power will
 * vary over time.
 */
static u32 cpu_get_max_power(struct power_actor *actor,
			struct thermal_zone_device *tz)
{
	u32 max_static_power, max_dyn_power;
	cpumask_t cpumask;
	unsigned int max_freq, last_entry, num_cpus;
	struct cpu_actor *cpu_actor = actor->data;

	cpumask_and(&cpumask, &cpu_actor->cpumask, cpu_online_mask);
	max_freq = cpufreq_quick_get_max(cpumask_any(&cpumask));
	max_static_power = get_static_power(cpu_actor, tz, max_freq);

	last_entry = cpu_actor->dyn_power_table_entries - 1;
	num_cpus = cpumask_weight(&cpumask);
	max_dyn_power = cpu_actor->dyn_power_table[last_entry].power * num_cpus;

	return max_static_power + max_dyn_power;
}

/**
 * cpu_set_power() - set cpufreq cooling device to consume a certain power
 * @actor: power actor pointer
 * @tz:		&thermal_zone_device closest to the CPU
 * @power: the power in milliwatts that should be set
 *
 * Callback for the power actor to configure the power consumption of
 * the CPU to be @power milliwatts at most.  This function assumes
 * that the load will remain constant.  The power is translated into a
 * cooling state that the cpu cooling device then sets.
 *
 * Return: 0 on success, -EINVAL if it couldn't convert the frequency
 * to a cpufreq cooling device state.
 */
static int cpu_set_power(struct power_actor *actor,
			struct thermal_zone_device *tz, u32 power)
{
	unsigned int cpu, cur_freq, target_freq;
	unsigned long cdev_state;
	u32 normalised_power, last_load;
	s32 dyn_power;
	struct thermal_cooling_device *cdev;
	struct cpu_actor *cpu_actor = actor->data;

	cdev = cpu_actor->cdev;
	cpu = cpumask_any_and(&cpu_actor->cpumask, cpu_online_mask);
	if (cpu >= nr_cpu_ids)
		return -EINVAL;

	cur_freq = cpufreq_quick_get(cpu);

	dyn_power = power - get_static_power(cpu_actor, tz, cur_freq);
	dyn_power = dyn_power > 0 ? dyn_power : 0;
	last_load = cpu_actor->last_load ? cpu_actor->last_load : 1;
	normalised_power = (dyn_power * 100) / last_load;
	target_freq = cpu_power_to_freq(cpu_actor, normalised_power);

	cdev_state = cpufreq_cooling_get_level(cpu, target_freq);
	if (cdev_state == THERMAL_CSTATE_INVALID) {
		pr_err("Failed to convert %dKHz for cpu %d into a cdev state\n",
			target_freq, cpu);
		return -EINVAL;
	}

	trace_thermal_power_actor_cpu_limit(&cpu_actor->cpumask, target_freq,
					cdev_state, power);

	return cdev->ops->set_cur_state(cdev, cdev_state);
}

static struct power_actor_ops cpu_actor_ops = {
	.get_req_power = cpu_get_req_power,
	.get_max_power = cpu_get_max_power,
	.set_power = cpu_set_power,
};

/**
 * build_dyn_power_table() - create a dynamic power to frequency table
 * @cpu_actor:	the cpu_actor in which to store the table
 * @capacitance: dynamic power coefficient for these cpus
 *
 * Build a dynamic power to frequency table for this cpu and store it
 * in @cpu_actor.  This table will be used in cpu_power_to_freq() and
 * cpu_freq_to_power() to convert between power and frequency
 * efficiently.  Power is stored in mW, frequency in KHz.  The
 * resulting table is in ascending order.
 *
 * Return: 0 on success, -E* on error.
 */
static int build_dyn_power_table(struct cpu_actor *cpu_actor, u32 capacitance)
{
	struct power_table *power_table;
	struct dev_pm_opp *opp;
	struct device *dev = NULL;
	int num_opps, cpu, i, ret = 0;
	unsigned long freq;

	num_opps = 0;

	rcu_read_lock();

	for_each_cpu(cpu, &cpu_actor->cpumask) {
		dev = get_cpu_device(cpu);
		if (!dev)
			continue;

		num_opps = dev_pm_opp_get_opp_count(dev);
		if (num_opps > 0) {
			break;
		} else if (num_opps < 0) {
			ret = num_opps;
			goto unlock;
		}
	}

	if (num_opps == 0) {
		ret = -EINVAL;
		goto unlock;
	}

	power_table = kcalloc(num_opps, sizeof(*power_table), GFP_KERNEL);

	i = 0;
	for (freq = 0;
	     opp = dev_pm_opp_find_freq_ceil(dev, &freq), !IS_ERR(opp);
	     freq++) {
		u32 freq_mhz, voltage_mv;
		u64 power;

		freq_mhz = freq / 1000000;
		voltage_mv = dev_pm_opp_get_voltage(opp) / 1000;

		/*
		 * Do the multiplication with MHz and millivolt so as
		 * to not overflow.
		 */
		power = (u64)capacitance * freq_mhz * voltage_mv * voltage_mv;
		do_div(power, 1000000000);

		/* frequency is stored in power_table in KHz */
		power_table[i].frequency = freq / 1000;
		power_table[i].power = power;

		i++;
	}

	if (i == 0) {
		ret = PTR_ERR(opp);
		goto unlock;
	}

	cpu_actor->dyn_power_table = power_table;
	cpu_actor->dyn_power_table_entries = i;

unlock:
	rcu_read_unlock();
	return ret;
}

/**
 * power_cpu_actor_register() - register a cpu_actor within the power actor API
 * @np:		DT node for the cpus.
 * @cpu:	one of the cpus covered by this power_actor
 * @capacitance:	dynamic power coefficient for these cpus
 * @weight:	weight of the cpu actor as an 8-bit fixed point
 * @plat_static_func:	function to calculate the static power consumed by these
 *			cpus (optional)
 *
 * Create a cpufreq cooling device for @cpu (and its related cpus) and
 * register it with the power actor API using a simple cpu power
 * model.  If @np is not NULL, the cpufreq cooling device is
 * registered with of_cpufreq_cooling_register(), otherwise
 * cpufreq_cooling_register() is used.  The cpus must have registered
 * their OPPs in the OPP library.
 *
 * An optional @plat_static_func may be provided to calculate the
 * static power consumed by these cpus.  If the platform's static
 * power consumption is unknown or negligible, make it NULL.
 *
 * The actor registered should be freed using
 * power_cpu_actor_unregister() when it's no longer needed.
 *
 * Return: The power_actor created on success or the corresponding
 * ERR_PTR() on failure.
 */
struct power_actor *
power_cpu_actor_register(struct device_node *np, unsigned int cpu,
			u32 capacitance, u32 weight,
			get_static_t plat_static_func)
{
	int ret;
	struct thermal_cooling_device *cdev;
	struct power_actor *actor, *err_ret;
	struct cpu_actor *cpu_actor;
	struct cpufreq_policy *cpufreq_pol;

	cpu_actor = kzalloc(sizeof(*cpu_actor), GFP_KERNEL);
	if (!cpu_actor)
		return ERR_PTR(-ENOMEM);

	cpufreq_pol = cpufreq_cpu_get(cpu);
	if (!cpufreq_pol) {
		err_ret = ERR_PTR(-EINVAL);
		goto kfree_cpu_actor;
	}

	cpumask_copy(&cpu_actor->cpumask, cpufreq_pol->related_cpus);

	cpufreq_cpu_put(cpufreq_pol);

	if (!np)
		cdev = cpufreq_cooling_register(&cpu_actor->cpumask);
	else
		cdev = of_cpufreq_cooling_register(np, &cpu_actor->cpumask);

	if (!cdev) {
		err_ret = ERR_PTR(PTR_ERR(cdev));
		goto kfree_cpu_actor;
	}

	cpu_actor->cdev = cdev;
	cpu_actor->plat_get_static_power = plat_static_func;

	ret = build_dyn_power_table(cpu_actor, capacitance);
	if (ret) {
		err_ret = ERR_PTR(ret);
		goto cdev_unregister;
	}

	actor = power_actor_register(weight, &cpu_actor_ops, cpu_actor);
	if (IS_ERR(actor)) {
		err_ret = actor;
		goto kfree_dyn_power_table;
	}

	return actor;

kfree_dyn_power_table:
	kfree(cpu_actor->dyn_power_table);
cdev_unregister:
	cpufreq_cooling_unregister(cdev);
kfree_cpu_actor:
	kfree(cpu_actor);

	return err_ret;
}

/**
 * power_cpu_actor_unregister() - Unregister a power cpu actor
 * @actor:	the actor to unregister
 */
void power_cpu_actor_unregister(struct power_actor *actor)
{
	struct cpu_actor *cpu_actor = actor->data;

	kfree(cpu_actor->dyn_power_table);
	kfree(cpu_actor);
	power_actor_unregister(actor);
}
