/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (C) 2013 ARM Limited
 * Copyright (C) 2015 Cog Systems Pty Ltd
 *
 */

#define pr_fmt(fmt) "psci: " fmt

#include <linux/acpi.h>
#include <linux/init.h>
#include <linux/of.h>
#include <microvisor/microvisor.h>
#include <asm/system_misc.h>

typedef int (*of_initcall_t)(const struct device_node *);

static okl4_kcap_t okl4_reset_virq = OKL4_KCAP_INVALID;

static void cell_sys_reset(enum reboot_mode reboot_mode, const char *cmd)
{
	if (okl4_reset_virq != OKL4_KCAP_INVALID)
		_okl4_sys_vinterrupt_raise(okl4_reset_virq, 0);
}

static int cell_reset_0_1_init(struct device_node *np)
{
	u32 val;

	if (!of_property_read_u32(np, "reset_interrupt_line", &val)) {
		okl4_reset_virq = val;
		printk(KERN_INFO "%s: Found reset_virq, cap = %d\n", __func__,
			okl4_reset_virq);
	} else {
		printk(KERN_WARNING "missing property in cell_reset node\n");
	}

	arm_pm_restart = cell_sys_reset;

	of_node_put(np);
	return 0;
}

static const struct of_device_id cell_reset_of_match[] __initconst = {
	{ .compatible = "okl,cell_reset-0.1", .data = cell_reset_0_1_init},
	{},
};

int __init okl4_pm_init(void)
{
	struct device_node *np;
	const struct of_device_id *matched_np;
	of_initcall_t init_fn;

	np = of_find_matching_node_and_match(NULL, cell_reset_of_match, &matched_np);
	if (!np)
		return -ENODEV;

	init_fn = (of_initcall_t)matched_np->data;
	return init_fn(np);
}
