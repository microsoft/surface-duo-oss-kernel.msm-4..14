/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Copyright (C) 2014 Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * The OPP code in function set_target() is reused from
 * drivers/cpufreq/omap-cpufreq.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/cpufreq-dt.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/thermal.h>

struct private_data {
	struct device *cpu_dev;
	struct regulator *cpu_reg;
	struct thermal_cooling_device *cdev;
	unsigned int voltage_tolerance; /* in percentage */
	struct notifier_block opp_nb;
	struct mutex lock;
	unsigned long opp_freq;
};

static struct freq_attr *cpufreq_dt_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,   /* Extra space for boost-attr if required */
	NULL,
};

static int opp_notifier(struct notifier_block *nb, unsigned long event,
			void *data)
{
	struct dev_pm_opp *opp = data;
	struct private_data *priv = container_of(nb, struct private_data,
						 opp_nb);
	struct device *cpu_dev = priv->cpu_dev;
	struct regulator *cpu_reg = priv->cpu_reg;
	unsigned long volt, tol, freq;
	int ret = 0;

	switch (event) {
		case OPP_EVENT_ADJUST_VOLTAGE:
			volt = dev_pm_opp_get_voltage(opp);
			freq = dev_pm_opp_get_freq(opp);
			tol = volt * priv->voltage_tolerance / 100;

			mutex_lock(&priv->lock);
			if (freq == priv->opp_freq)
				ret = regulator_set_voltage_tol(cpu_reg, volt,
								tol);
			mutex_unlock(&priv->lock);
			if (ret) {
				dev_err(cpu_dev,
					"failed to scale voltage up: %d\n",
					ret);
				return ret;
			}
			break;
		default:
			break;
	}

	return 0;
}

static int set_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct dev_pm_opp *opp;
	struct cpufreq_frequency_table *freq_table = policy->freq_table;
	struct clk *cpu_clk = policy->clk;
	struct clk *l2_clk = policy->l2_clk;
	struct private_data *priv = policy->driver_data;
	struct device *cpu_dev = priv->cpu_dev;
	struct regulator *cpu_reg = priv->cpu_reg;
	unsigned long volt = 0, volt_old = 0, tol = 0;
	unsigned int old_freq, new_freq, l2_freq;
	unsigned long new_l2_freq = 0;
	long freq_Hz, freq_exact;
	unsigned long opp_freq = 0;
	int ret;

	freq_Hz = clk_round_rate(cpu_clk, freq_table[index].frequency * 1000);
	if (freq_Hz <= 0)
		freq_Hz = freq_table[index].frequency * 1000;

	freq_exact = freq_Hz;
	new_freq = freq_Hz / 1000;
	old_freq = clk_get_rate(cpu_clk) / 1000;

	mutex_lock(&priv->lock);
	if (!IS_ERR(cpu_reg)) {

		rcu_read_lock();
		opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq_Hz);
		if (IS_ERR(opp)) {
			rcu_read_unlock();
			dev_err(cpu_dev, "failed to find OPP for %ld\n",
				freq_Hz);
			ret = PTR_ERR(opp);
			goto out;
		}
		volt = dev_pm_opp_get_voltage(opp);
		opp_freq = dev_pm_opp_get_freq(opp);
		rcu_read_unlock();
		tol = volt * priv->voltage_tolerance / 100;
		volt_old = regulator_get_voltage(cpu_reg);
		dev_dbg(cpu_dev, "Found OPP: %ld kHz, %ld uV\n",
			opp_freq / 1000, volt);
	}

	dev_dbg(cpu_dev, "%u MHz, %ld mV --> %u MHz, %ld mV\n",
		old_freq / 1000, (volt_old > 0) ? volt_old / 1000 : -1,
		new_freq / 1000, volt ? volt / 1000 : -1);

	/* scaling up?  scale voltage before frequency */
	if (!IS_ERR(cpu_reg) && new_freq > old_freq) {
		ret = regulator_set_voltage_tol(cpu_reg, volt, tol);
		if (ret) {
			dev_err(cpu_dev, "failed to scale voltage up: %d\n",
				ret);
			goto out;
		}
	}

	ret = clk_set_rate(cpu_clk, freq_exact);
	if (ret) {
		dev_err(cpu_dev, "failed to set clock rate: %d\n", ret);
		if (!IS_ERR(cpu_reg) && volt_old > 0)
			regulator_set_voltage_tol(cpu_reg, volt_old, tol);
		goto out;
	}

	if (!IS_ERR(l2_clk) && policy->l2_rate[0] && policy->l2_rate[1] &&
	    policy->l2_rate[2]) {
		static unsigned long krait_l2[CONFIG_NR_CPUS] = { };
		int cpu, ret = 0;

		if (freq_exact >= policy->l2_rate[2])
			new_l2_freq = policy->l2_rate[2];
		else if (freq_exact >= policy->l2_rate[1])
			new_l2_freq = policy->l2_rate[1];
		else
			new_l2_freq = policy->l2_rate[0];

		krait_l2[policy->cpu] = new_l2_freq;
		for_each_present_cpu(cpu)
			new_l2_freq = max(new_l2_freq, krait_l2[cpu]);

		l2_freq = clk_get_rate(l2_clk);

		if (l2_freq != new_l2_freq) {
			/* scale l2 with the core */
			ret = clk_set_rate(l2_clk, new_l2_freq);
		}
	}

	/* scaling down?  scale voltage after frequency */
	if (!IS_ERR(cpu_reg) && new_freq < old_freq) {
		ret = regulator_set_voltage_tol(cpu_reg, volt, tol);
		if (ret) {
			dev_err(cpu_dev, "failed to scale voltage down: %d\n",
				ret);
			clk_set_rate(cpu_clk, old_freq * 1000);
			goto out;
		}
	}

	priv->opp_freq = opp_freq;

out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int allocate_resources(int cpu, struct device **cdev,
			      struct regulator **creg, struct clk **cclk,
			      struct clk **l2)
{
	struct device *cpu_dev;
	struct regulator *cpu_reg;
	struct clk *cpu_clk, *l2_clk = NULL;
	int ret = 0;
	char *reg_cpu0 = "cpu0", *reg_cpu = "cpu", *reg;

	cpu_dev = get_cpu_device(cpu);
	if (!cpu_dev) {
		pr_err("failed to get cpu%d device\n", cpu);
		return -ENODEV;
	}

	/* Try "cpu0" for older DTs */
	if (!cpu)
		reg = reg_cpu0;
	else
		reg = reg_cpu;

try_again:
	cpu_reg = regulator_get_optional(cpu_dev, reg);
	if (IS_ERR(cpu_reg)) {
		/*
		 * If cpu's regulator supply node is present, but regulator is
		 * not yet registered, we should try defering probe.
		 */
		if (PTR_ERR(cpu_reg) == -EPROBE_DEFER) {
			dev_dbg(cpu_dev, "cpu%d regulator not ready, retry\n",
				cpu);
			return -EPROBE_DEFER;
		}

		/* Try with "cpu-supply" */
		if (reg == reg_cpu0) {
			reg = reg_cpu;
			goto try_again;
		}

		dev_dbg(cpu_dev, "no regulator for cpu%d: %ld\n",
			cpu, PTR_ERR(cpu_reg));
	}

	cpu_clk = clk_get(cpu_dev, NULL);
	if (IS_ERR(cpu_clk)) {
		/* put regulator */
		if (!IS_ERR(cpu_reg))
			regulator_put(cpu_reg);

		ret = PTR_ERR(cpu_clk);

		/*
		 * If cpu's clk node is present, but clock is not yet
		 * registered, we should try defering probe.
		 */
		if (ret == -EPROBE_DEFER)
			dev_dbg(cpu_dev, "cpu%d clock not ready, retry\n", cpu);
		else
			dev_err(cpu_dev, "failed to get cpu%d clock: %d\n", cpu,
				ret);
	} else {
		*cdev = cpu_dev;
		*creg = cpu_reg;
		*cclk = cpu_clk;

		l2_clk = clk_get(cpu_dev, "l2");
		if (!IS_ERR(l2_clk))
			*l2 = l2_clk;
	}

	return ret;
}

static int cpufreq_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *freq_table;
	struct device_node *np;
	struct device_node *l2_np;
	struct private_data *priv;
	struct device *cpu_dev;
	struct regulator *cpu_reg;
	struct dev_pm_opp *suspend_opp;
	struct clk *cpu_clk, *l2_clk;
	unsigned long min_uV = ~0, max_uV = 0;
	unsigned int transition_latency;
	bool need_update = false;
	int ret;
	struct srcu_notifier_head *opp_srcu_head;

	ret = allocate_resources(policy->cpu, &cpu_dev, &cpu_reg, &cpu_clk,
				 &l2_clk);
	if (ret) {
		pr_err("%s: Failed to allocate resources: %d\n", __func__, ret);
		return ret;
	}

	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		dev_err(cpu_dev, "failed to find cpu%d node\n", policy->cpu);
		ret = -ENOENT;
		goto out_put_reg_clk;
	}

	/* Get OPP-sharing information from "operating-points-v2" bindings */
	ret = dev_pm_opp_of_get_sharing_cpus(cpu_dev, policy->cpus);
	if (ret) {
		/*
		 * operating-points-v2 not supported, fallback to old method of
		 * finding shared-OPPs for backward compatibility.
		 */
		if (ret == -ENOENT)
			need_update = true;
		else
			goto out_node_put;
	}

	/*
	 * Initialize OPP tables for all policy->cpus. They will be shared by
	 * all CPUs which have marked their CPUs shared with OPP bindings.
	 *
	 * For platforms not using operating-points-v2 bindings, we do this
	 * before updating policy->cpus. Otherwise, we will end up creating
	 * duplicate OPPs for policy->cpus.
	 *
	 * OPPs might be populated at runtime, don't check for error here
	 */
	dev_pm_opp_of_cpumask_add_table(policy->cpus);

	/*
	 * But we need OPP table to function so if it is not there let's
	 * give platform code chance to provide it for us.
	 */
	ret = dev_pm_opp_get_opp_count(cpu_dev);
	if (ret <= 0) {
		pr_debug("OPP table is not ready, deferring probe\n");
		ret = -EPROBE_DEFER;
		goto out_free_opp;
	}

	if (need_update) {
		struct cpufreq_dt_platform_data *pd = cpufreq_get_driver_data();

		if (!pd || !pd->independent_clocks)
			cpumask_setall(policy->cpus);

		/*
		 * OPP tables are initialized only for policy->cpu, do it for
		 * others as well.
		 */
		ret = dev_pm_opp_set_sharing_cpus(cpu_dev, policy->cpus);
		if (ret)
			dev_err(cpu_dev, "%s: failed to mark OPPs as shared: %d\n",
				__func__, ret);

		of_property_read_u32(np, "clock-latency", &transition_latency);
	} else {
		transition_latency = dev_pm_opp_get_max_clock_latency(cpu_dev);
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto out_free_opp;
	}

	mutex_init(&priv->lock);

	rcu_read_lock();
	opp_srcu_head = dev_pm_opp_get_notifier(cpu_dev);
	if (IS_ERR(opp_srcu_head)) {
		ret = PTR_ERR(opp_srcu_head);
		rcu_read_unlock();
		goto out_free_priv;
	}

	priv->opp_nb.notifier_call = opp_notifier;
	ret = srcu_notifier_chain_register(opp_srcu_head, &priv->opp_nb);
	rcu_read_unlock();
	if (ret)
		goto out_free_priv;

	of_property_read_u32(np, "voltage-tolerance", &priv->voltage_tolerance);

	if (!transition_latency)
		transition_latency = CPUFREQ_ETERNAL;

	if (!IS_ERR(cpu_reg)) {
		unsigned long opp_freq = 0;

		/*
		 * Disable any OPPs where the connected regulator isn't able to
		 * provide the specified voltage and record minimum and maximum
		 * voltage levels.
		 */
		while (1) {
			struct dev_pm_opp *opp;
			unsigned long opp_uV, tol_uV;

			rcu_read_lock();
			opp = dev_pm_opp_find_freq_ceil(cpu_dev, &opp_freq);
			if (IS_ERR(opp)) {
				rcu_read_unlock();
				break;
			}
			opp_uV = dev_pm_opp_get_voltage(opp);
			rcu_read_unlock();

			tol_uV = opp_uV * priv->voltage_tolerance / 100;
			if (regulator_is_supported_voltage(cpu_reg,
							   opp_uV - tol_uV,
							   opp_uV + tol_uV)) {
				if (opp_uV < min_uV)
					min_uV = opp_uV;
				if (opp_uV > max_uV)
					max_uV = opp_uV;
			} else {
				dev_pm_opp_disable(cpu_dev, opp_freq);
			}

			opp_freq++;
		}

		ret = regulator_set_voltage_time(cpu_reg, min_uV, max_uV);
		if (ret > 0)
			transition_latency += ret * 1000;
	}

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		pr_err("failed to init cpufreq table: %d\n", ret);
		goto out_unregister_nb;
	}

	priv->cpu_dev = cpu_dev;
	priv->cpu_reg = cpu_reg;
	policy->driver_data = priv;

	policy->clk = cpu_clk;

	rcu_read_lock();
	suspend_opp = dev_pm_opp_get_suspend_opp(cpu_dev);
	if (suspend_opp)
		policy->suspend_freq = dev_pm_opp_get_freq(suspend_opp) / 1000;
	rcu_read_unlock();
	policy->l2_clk = l2_clk;

	l2_np = of_find_node_by_name(NULL, "qcom,l2");
	if (l2_np)
		of_property_read_u32_array(l2_np, "qcom,l2-rates", policy->l2_rate, 3);

	ret = cpufreq_table_validate_and_show(policy, freq_table);
	if (ret) {
		dev_err(cpu_dev, "%s: invalid frequency table: %d\n", __func__,
			ret);
		goto out_free_cpufreq_table;
	}

	/* Support turbo/boost mode */
	if (policy_has_boost_freq(policy)) {
		/* This gets disabled by core on driver unregister */
		ret = cpufreq_enable_boost_support();
		if (ret)
			goto out_free_cpufreq_table;
		cpufreq_dt_attr[1] = &cpufreq_freq_attr_scaling_boost_freqs;
	}

	policy->cpuinfo.transition_latency = transition_latency;

	of_node_put(np);

	return 0;

out_free_cpufreq_table:
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);
out_unregister_nb:
	srcu_notifier_chain_unregister(opp_srcu_head, &priv->opp_nb);
out_free_priv:
	kfree(priv);
out_free_opp:
	dev_pm_opp_of_cpumask_remove_table(policy->cpus);
out_node_put:
	of_node_put(np);
out_put_reg_clk:
	clk_put(cpu_clk);
	if (!IS_ERR(cpu_reg))
		regulator_put(cpu_reg);

	return ret;
}

static int cpufreq_exit(struct cpufreq_policy *policy)
{
	struct private_data *priv = policy->driver_data;

	cpufreq_cooling_unregister(priv->cdev);
	dev_pm_opp_free_cpufreq_table(priv->cpu_dev, &policy->freq_table);
	dev_pm_opp_of_cpumask_remove_table(policy->related_cpus);
	clk_put(policy->clk);
	if (!IS_ERR(priv->cpu_reg))
		regulator_put(priv->cpu_reg);
	kfree(priv);

	return 0;
}

static void cpufreq_ready(struct cpufreq_policy *policy)
{
	struct private_data *priv = policy->driver_data;
	struct device_node *np = of_node_get(priv->cpu_dev->of_node);

	if (WARN_ON(!np))
		return;

	/*
	 * For now, just loading the cooling device;
	 * thermal DT code takes care of matching them.
	 */
	if (of_find_property(np, "#cooling-cells", NULL)) {
		priv->cdev = of_cpufreq_cooling_register(np,
							 policy->related_cpus);
		if (IS_ERR(priv->cdev)) {
			dev_err(priv->cpu_dev,
				"running cpufreq without cooling device: %ld\n",
				PTR_ERR(priv->cdev));

			priv->cdev = NULL;
		}
	}

	of_node_put(np);
}

static struct cpufreq_driver dt_cpufreq_driver = {
	.flags = CPUFREQ_STICKY | CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = set_target,
	.get = cpufreq_generic_get,
	.init = cpufreq_init,
	.exit = cpufreq_exit,
	.ready = cpufreq_ready,
	.name = "cpufreq-dt",
	.attr = cpufreq_dt_attr,
	.suspend = cpufreq_generic_suspend,
};

static int dt_cpufreq_probe(struct platform_device *pdev)
{
	struct device *cpu_dev;
	struct regulator *cpu_reg;
	struct clk *cpu_clk, *l2_clk;
	int ret;

	/*
	 * All per-cluster (CPUs sharing clock/voltages) initialization is done
	 * from ->init(). In probe(), we just need to make sure that clk and
	 * regulators are available. Else defer probe and retry.
	 *
	 * FIXME: Is checking this only for CPU0 sufficient ?
	 */
	ret = allocate_resources(0, &cpu_dev, &cpu_reg, &cpu_clk, &l2_clk);
	if (ret)
		return ret;

	clk_put(cpu_clk);
	if (!IS_ERR(cpu_reg))
		regulator_put(cpu_reg);

	dt_cpufreq_driver.driver_data = dev_get_platdata(&pdev->dev);

	ret = cpufreq_register_driver(&dt_cpufreq_driver);
	if (ret)
		dev_err(cpu_dev, "failed register driver: %d\n", ret);

	return ret;
}

static int dt_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&dt_cpufreq_driver);
	return 0;
}

static struct platform_driver dt_cpufreq_platdrv = {
	.driver = {
		.name	= "cpufreq-dt",
	},
	.probe		= dt_cpufreq_probe,
	.remove		= dt_cpufreq_remove,
};
module_platform_driver(dt_cpufreq_platdrv);

MODULE_ALIAS("platform:cpufreq-dt");
MODULE_AUTHOR("Viresh Kumar <viresh.kumar@linaro.org>");
MODULE_AUTHOR("Shawn Guo <shawn.guo@linaro.org>");
MODULE_DESCRIPTION("Generic cpufreq driver");
MODULE_LICENSE("GPL");
