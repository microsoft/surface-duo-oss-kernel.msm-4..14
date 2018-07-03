/*
 * Simple driver to enable clocks not claimed by a device driver.
 *
 * Copyright 2017 Cog Systems Pty Ltd
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#if defined(CONFIG_ARCH_MSM) || defined (CONFIG_ARCH_QCOM)
#include <linux/clk/msm-clk-provider.h>
#else
#include <linux/clk-provider.h>
#endif
#include <linux/of.h>
#include <linux/list.h>

struct okl4_clock {
	struct list_head list;
	struct clk *clk;
};

static LIST_HEAD(okl4_late_clocks);

#if defined(CONFIG_ARCH_MSM) || defined (CONFIG_ARCH_QCOM)
#define __clk_get_name clk_name
#endif

int __init okl4_prepare_clocks(void)
{
	struct device_node *np;
	struct clk *c;
	unsigned int i = 0;

	for_each_compatible_node(np, NULL, "okl,enable-clocks") {
		if (!of_device_is_available(np))
			continue;

		for (c = of_clk_get(np, i); IS_ERR(c) == 0;
				c = of_clk_get(np, ++i)) {
			int ret = clk_prepare_enable(c);

			if (ret) {
				struct okl4_clock *clk;

				/*
				 * This clock can't be enabled now, so add it to
				 * the late clock list so that we can attempt to
				 * enable it later.
				 */
				printk(KERN_INFO "okl4_prepare_clocks: cannot "
						"enable %s during early init, "
						"delaying until late init\n",
						__clk_get_name(c));
				clk = kzalloc(sizeof(*clk), GFP_KERNEL);
				if (WARN_ON(!clk))
					continue;
				clk->clk = c;
				list_add_tail(&clk->list, &okl4_late_clocks);
			}
		}
		i = 0;
	}

	return 0;
}
early_initcall(okl4_prepare_clocks);

int __init okl4_prepare_clocks_late(void)
{
	struct okl4_clock *clk;

	list_for_each_entry(clk, &okl4_late_clocks, list) {
		int ret = clk_prepare_enable(clk->clk);

		if (ret)
			printk(KERN_WARNING "okl4_prepare_clocks_late: cannot "
					"enable %s\n",__clk_get_name(clk->clk));
	}

	return 0;
}
late_initcall(okl4_prepare_clocks_late);
