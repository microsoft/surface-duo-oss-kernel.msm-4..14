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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/pm_clock.h>
#include <linux/slab.h>
#include "gdsc.h"

#define PWR_ON_MASK		BIT(31)
#define EN_REST_WAIT_MASK	GENMASK(23, 20)
#define EN_FEW_WAIT_MASK	GENMASK(19, 16)
#define CLK_DIS_WAIT_MASK	GENMASK(15, 12)
#define SW_OVERRIDE_MASK	BIT(2)
#define HW_CONTROL_MASK		BIT(1)
#define SW_COLLAPSE_MASK	BIT(0)

/* Wait 2^n CXO cycles between all states. Here, n=2 (4 cycles). */
#define EN_REST_WAIT_VAL	(0x2 << 20)
#define EN_FEW_WAIT_VAL		(0x8 << 16)
#define CLK_DIS_WAIT_VAL	(0x2 << 12)

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

	ret =  regmap_read(sc->regmap, sc->gdscr, &val);
	if (ret)
		return ret;

	if ((val & PWR_ON_MASK) == check)
		return 0;

	return -ETIMEDOUT;
}

static int gdsc_enable(struct generic_pm_domain *domain)
{
	struct gdsc *sc = domain_to_gdsc(domain);
	int ret;

	if (sc->root_clk)
		clk_prepare_enable(sc->root_clk);

	ret = gdsc_toggle_logic(sc, true);
	if (ret)
		return ret;
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

	ret = gdsc_toggle_logic(sc, false);

	if (sc->root_clk)
		clk_disable_unprepare(sc->root_clk);

	return ret;
}

static int gdsc_attach(struct generic_pm_domain *domain, struct device *dev)
{
	int ret;
	struct gdsc *sc = domain_to_gdsc(domain);
	char **con_id;

	if (!sc->con_ids[0])
		return 0;

	ret = pm_clk_create(dev);
	if (ret) {
		dev_err(dev, "pm_clk_create failed %d\n", ret);
		return ret;
	}

	for (con_id = sc->con_ids; *con_id; con_id++) {
		ret = pm_clk_add(dev, *con_id);
		if (ret) {
			dev_err(dev, "pm_clk_add failed %d\n", ret);
			goto fail;
		}
	}

	if (sc->root_con_id) {
		sc->root_clk = clk_get(dev, sc->root_con_id);
		if (IS_ERR(sc->root_clk)) {
			dev_err(dev, "failed to get root clock\n");
			return PTR_ERR(sc->root_clk);
		}
	}

	return 0;
fail:
	pm_clk_destroy(dev);
	return ret;
};

static void gdsc_detach(struct generic_pm_domain *domain, struct device *dev)
{
	struct gdsc *sc = domain_to_gdsc(domain);

	if (!sc->con_ids[0])
		return;

	pm_clk_destroy(dev);
	return;
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

	on = gdsc_is_enabled(sc);
	if (on < 0)
		return on;

	sc->pd.power_off = gdsc_disable;
	sc->pd.power_on = gdsc_enable;
	sc->pd.attach_dev = gdsc_attach;
	sc->pd.detach_dev = gdsc_detach;
	sc->pd.flags = GENPD_FLAG_PM_CLK;
	pm_genpd_init(&sc->pd, NULL, !on);

	return 0;
}

int gdsc_register(struct device *dev, struct gdsc **scs, size_t num,
		  struct regmap *regmap)
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
