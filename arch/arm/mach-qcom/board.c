/* Copyright (c) 2010-2014 The Linux Foundation. All rights reserved.
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

#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/delay.h>

#include <asm/mach/arch.h>

static const char * const qcom_dt_match[] __initconst = {
	"qcom,apq8064",
	"qcom,apq8074-dragonboard",
	"qcom,apq8084",
	"qcom,ipq8062",
	"qcom,ipq8064",
	"qcom,msm8660-surf",
	"qcom,msm8960-cdp",
	NULL
};

static void __init qcom_late_init(void)
{
	struct device_node *node;
	int reset_gpio, ret;

	for_each_compatible_node(node, NULL, "atheros,ath6kl") {
		of_node_put(node);

		reset_gpio = of_get_named_gpio(node, "reset-gpio", 0);
		if (reset_gpio < 0)
			return;

		ret = gpio_request_one(reset_gpio,
					GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "reset");
		if (ret)
			return;
		
		udelay(100);
		gpio_set_value(reset_gpio, 0);
		udelay(100);
		gpio_set_value(reset_gpio, 1);
	}
}

DT_MACHINE_START(QCOM_DT, "Qualcomm (Flattened Device Tree)")
	.init_late	= qcom_late_init,
	.dt_compat	= qcom_dt_match,
MACHINE_END
