/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/pm_clock.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include "gdsc.h"

#define PWR_ON_MASK		BIT(31)
#define EN_REST_WAIT_MASK	GENMASK_ULL(23, 20)
#define EN_FEW_WAIT_MASK	GENMASK_ULL(19, 16)
#define CLK_DIS_WAIT_MASK	GENMASK_ULL(15, 12)
#define SW_OVERRIDE_MASK	BIT(2)
#define HW_CONTROL_MASK		BIT(1)
#define SW_COLLAPSE_MASK	BIT(0)

/* Wait 2^n CXO cycles between all states. Here, n=2 (4 cycles). */
#define EN_REST_WAIT_VAL	(0x2 << 20)
#define EN_FEW_WAIT_VAL		(0x8 << 16)
#define CLK_DIS_WAIT_VAL	(0x2 << 12)

#define RETAIN_MEM		BIT(14)
#define RETAIN_PERIPH		BIT(13)

#define TIMEOUT_US		100

#define domain_to_gdsc(domain) container_of(domain, struct gdsc, pd)

static int gdsc_is_enabled(struct gdsc *sc)
{
	u32 val;
	int ret;

	ret = regmap_read(sc->regmap, sc->gdscr, &val);
	if (ret)
		return ret;

	return !!(val & PWR_ON_MASK);
}

static int gdsc_toggle_logic(struct gdsc *sc, bool en)
{
	int ret;
	u32 val = en ? 0 : SW_COLLAPSE_MASK;
	u32 check = en ? PWR_ON_MASK : 0;
	unsigned long timeout;

	ret = regmap_update_bits(sc->regmap, sc->gdscr, SW_COLLAPSE_MASK, val);
	if (ret)
		return ret;

	timeout = jiffies + usecs_to_jiffies(TIMEOUT_US);
	do {
		ret = regmap_read(sc->regmap, sc->gdscr, &val);
		if (ret)
			return ret;

		if ((val & PWR_ON_MASK) == check)
			return 0;
	} while (time_before(jiffies, timeout));

	ret = regmap_read(sc->regmap, sc->gdscr, &val);
	if (ret)
		return ret;

	if ((val & PWR_ON_MASK) == check)
		return 0;

	return -ETIMEDOUT;
}

static inline int gdsc_deassert_reset(struct gdsc *sc)
{
	int i;

	for (i = 0; i < sc->reset_count; i++)
		sc->rcdev->ops->deassert(sc->rcdev, sc->resets[i]);
	return 0;
}

static inline int gdsc_assert_reset(struct gdsc *sc)
{
	int i;

	for (i = 0; i < sc->reset_count; i++)
		sc->rcdev->ops->assert(sc->rcdev, sc->resets[i]);
	return 0;
}

static inline void gdsc_force_mem_on(struct gdsc *sc)
{
	int i;
	u32 mask = RETAIN_MEM | RETAIN_PERIPH;

	for (i = 0; i < sc->cxc_count; i++)
		regmap_update_bits(sc->regmap, sc->cxcs[i], mask, mask);
}

static inline void gdsc_clear_mem_on(struct gdsc *sc)
{
	int i;
	u32 mask = RETAIN_MEM | RETAIN_PERIPH;

	for (i = 0; i < sc->cxc_count; i++)
		regmap_update_bits(sc->regmap, sc->cxcs[i], mask, 0);
}

static int gdsc_enable(struct generic_pm_domain *domain)
{
	struct gdsc *sc = domain_to_gdsc(domain);
	int ret;

	if (sc->pwrsts == PWRSTS_ON)
		return gdsc_deassert_reset(sc);

	if (sc->root_clk)
		clk_prepare_enable(sc->root_clk);

	ret = gdsc_toggle_logic(sc, true);
	if (ret)
		return ret;

	if (sc->pwrsts & PWRSTS_OFF)
		gdsc_force_mem_on(sc);

	/*
	 * If clocks to this power domain were already on, they will take an
	 * additional 4 clock cycles to re-enable after the power domain is
	 * enabled. Delay to account for this. A delay is also needed to ensure
	 * clocks are not enabled within 400ns of enabling power to the
	 * memories.
	 */
	udelay(1);

	return 0;
}

static int gdsc_disable(struct generic_pm_domain *domain)
{
	int ret;
	struct gdsc *sc = domain_to_gdsc(domain);

	if (sc->pwrsts == PWRSTS_ON)
		return gdsc_assert_reset(sc);

	if (sc->pwrsts & PWRSTS_OFF)
		gdsc_clear_mem_on(sc);

	ret = gdsc_toggle_logic(sc, false);

	if (sc->root_clk)
		clk_disable_unprepare(sc->root_clk);

	return ret;
}

static inline bool match(unsigned int id, unsigned int *ids, unsigned int count)
{
	int i;

	for (i = 0; i < count; i++)
		if (id == ids[i])
			return true;
	return false;
}

static int gdsc_attach(struct generic_pm_domain *domain, struct device *dev)
{
	int ret, i = 0, j = 0;
	struct gdsc *sc = domain_to_gdsc(domain);
	struct of_phandle_args clkspec;
	struct device_node *np = dev->of_node;

	if (!sc->clock_count)
		return 0;

	ret = pm_clk_create(dev);
	if (ret) {
		dev_dbg(dev, "pm_clk_create failed %d\n", ret);
		return ret;
	}

	sc->clks = devm_kcalloc(dev, sc->clock_count, sizeof(sc->clks),
				       GFP_KERNEL);
	if (!sc->clks)
		return -ENOMEM;

	while (!of_parse_phandle_with_args(np, "clocks", "#clock-cells", i,
					   &clkspec)) {
		if (match(clkspec.args[0], sc->clocks, sc->clock_count)) {
			sc->clks[j] = of_clk_get_from_provider(&clkspec);
			pm_clk_add_clk(dev, sc->clks[j]);
			j++;
		} else if (clkspec.args[0] == sc->root_clock)
			sc->root_clk = of_clk_get_from_provider(&clkspec);
		i++;
	}
	return 0;
};

static void gdsc_detach(struct generic_pm_domain *domain, struct device *dev)
{
	struct gdsc *sc = domain_to_gdsc(domain);

	if (!sc->clock_count)
		return;

	pm_clk_destroy(dev);
};

static int gdsc_init(struct gdsc *sc)
{
	u32 mask, val;
	int on, ret;

	/*
	 * Disable HW trigger: collapse/restore occur based on registers writes.
	 * Disable SW override: Use hardware state-machine for sequencing.
	 * Configure wait time between states.
	 */
	mask = HW_CONTROL_MASK | SW_OVERRIDE_MASK |
	       EN_REST_WAIT_MASK | EN_FEW_WAIT_MASK | CLK_DIS_WAIT_MASK;
	val = EN_REST_WAIT_VAL | EN_FEW_WAIT_VAL | CLK_DIS_WAIT_VAL;
	ret = regmap_update_bits(sc->regmap, sc->gdscr, mask, val);
	if (ret)
		return ret;

	/* Force gdsc ON if only ON state is supported */
	if (sc->pwrsts == PWRSTS_ON) {
		ret = gdsc_toggle_logic(sc, true);
		if (ret)
			return ret;
	}

	on = gdsc_is_enabled(sc);
	if (on < 0)
		return on;

	if (on || (sc->pwrsts & PWRSTS_RET))
		gdsc_force_mem_on(sc);
	else
		gdsc_clear_mem_on(sc);

	sc->pd.power_off = gdsc_disable;
	sc->pd.power_on = gdsc_enable;
	sc->pd.attach_dev = gdsc_attach;
	sc->pd.detach_dev = gdsc_detach;
	sc->pd.flags = GENPD_FLAG_PM_CLK;
	pm_genpd_init(&sc->pd, NULL, !on);

	return 0;
}

int gdsc_register(struct device *dev, struct gdsc **scs, size_t num,
		  struct reset_controller_dev *rcdev, struct regmap *regmap)
{
	int i, ret;
	struct genpd_onecell_data *data;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->domains = devm_kcalloc(dev, num, sizeof(*data->domains),
				     GFP_KERNEL);
	if (!data->domains)
		return -ENOMEM;

	data->num_domains = num;
	for (i = 0; i < num; i++) {
		if (!scs[i])
			continue;
		scs[i]->regmap = regmap;
		scs[i]->rcdev = rcdev;
		ret = gdsc_init(scs[i]);
		if (ret)
			return ret;
		data->domains[i] = &scs[i]->pd;
	}

	return of_genpd_add_provider_onecell(dev->of_node, data);
}

void gdsc_unregister(struct device *dev)
{
	of_genpd_del_provider(dev->of_node);
}

#ifndef CONFIG_PM
static void enable_clock(struct of_phandle_args *clkspec)
{
	struct clk *clk;

	clk = of_clk_get_from_provider(clkspec);
	if (!IS_ERR(clk)) {
		clk_prepare_enable(clk);
		clk_put(clk);
	}
}

static void disable_clock(struct of_phandle_args *clkspec)
{
	struct clk *clk;

	clk = of_clk_get_from_provider(clkspec);
	if (!IS_ERR(clk)) {
		clk_disable_unprepare(clk);
		clk_put(clk);
	}
}

static int clk_notify(struct notifier_block *nb, unsigned long action,
		      void *data)
{
	int sz, i = 0;
	struct device *dev = data;
	struct gdsc_notifier_block *gdsc_nb;
	struct of_phandle_args clkspec;
	struct device_node *np = dev->of_node;

	if (!of_find_property(dev->of_node, "power-domains", &sz))
		return 0;

	gdsc_nb = container_of(nb, struct gdsc_notifier_block, nb);

	if (!gdsc_nb->clock_count)
		return 0;

	switch (action) {
	case BUS_NOTIFY_BIND_DRIVER:
		while (!of_parse_phandle_with_args(np, "clocks", "#clock-cells",
						   i, &clkspec)) {
			if (match(clkspec.args[0], gdsc_nb->clocks,
				  gdsc_nb->clock_count))
				enable_clock(&clkspec);
			i++;
		}
		break;
	case BUS_NOTIFY_UNBOUND_DRIVER:
		while (!of_parse_phandle_with_args(np, "clocks", "#clock-cells",
						   i, &clkspec)) {
			if (match(clkspec.args[0], gdsc_nb->clocks,
				  gdsc_nb->clock_count))
				disable_clock(&clkspec);
			i++;
		}
		break;
	}
	return 0;
}

void qcom_pm_add_notifier(struct gdsc_notifier_block *gdsc_nb)
{
	if (!gdsc_nb)
		return;

	gdsc_nb->nb.notifier_call = clk_notify,
	bus_register_notifier(&platform_bus_type, &gdsc_nb->nb);
}
#endif
