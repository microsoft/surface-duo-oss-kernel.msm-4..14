/*
 *  Copyright (C) 2014 Linaro Ltd
 *
 * Author: Ulf Hansson <ulf.hansson@linaro.org>
 *
 * License terms: GNU General Public License (GPL) version 2
 *
 *  Simple MMC power sequence management
 */
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>

#include <linux/mmc/host.h>

#include "pwrseq.h"

struct mmc_pwrseq_simple {
	struct mmc_pwrseq pwrseq;
	bool clk_enabled;
	struct clk *ext_clk;
	int nr_gpios;
	struct gpio_desc *reset_gpios[0];
};

#define to_pwrseq_simple(p) container_of(p, struct mmc_pwrseq_simple, pwrseq)

static void mmc_pwrseq_simple_set_gpios_value(struct mmc_pwrseq_simple *pwrseq,
					      int value)
{
	int i;

	for (i = 0; i < pwrseq->nr_gpios; i++)
		if (!IS_ERR(pwrseq->reset_gpios[i]))
			gpiod_set_value_cansleep(pwrseq->reset_gpios[i], value);
}

static void mmc_pwrseq_simple_pre_power_on(struct mmc_host *host)
{
	struct mmc_pwrseq_simple *pwrseq = to_pwrseq_simple(host->pwrseq);

	if (!IS_ERR(pwrseq->ext_clk) && !pwrseq->clk_enabled) {
		clk_prepare_enable(pwrseq->ext_clk);
		pwrseq->clk_enabled = true;
	}

	mmc_pwrseq_simple_set_gpios_value(pwrseq, 1);
}

static void mmc_pwrseq_simple_post_power_on(struct mmc_host *host)
{
	struct mmc_pwrseq_simple *pwrseq = to_pwrseq_simple(host->pwrseq);

	mmc_pwrseq_simple_set_gpios_value(pwrseq, 0);
}

static void mmc_pwrseq_simple_power_off(struct mmc_host *host)
{
	struct mmc_pwrseq_simple *pwrseq = to_pwrseq_simple(host->pwrseq);

	mmc_pwrseq_simple_set_gpios_value(pwrseq, 1);

	if (!IS_ERR(pwrseq->ext_clk) && pwrseq->clk_enabled) {
		clk_disable_unprepare(pwrseq->ext_clk);
		pwrseq->clk_enabled = false;
	}
}

static void mmc_pwrseq_simple_free(struct mmc_host *host)
{
	struct mmc_pwrseq_simple *pwrseq = to_pwrseq_simple(host->pwrseq);
	int i;

	for (i = 0; i < pwrseq->nr_gpios; i++)
		if (!IS_ERR(pwrseq->reset_gpios[i]))
			gpiod_put(pwrseq->reset_gpios[i]);

	if (!IS_ERR(pwrseq->ext_clk))
		clk_put(pwrseq->ext_clk);

}

int mmc_pwrseq_simple_alloc(struct mmc_host *host)
{
	struct mmc_pwrseq_simple *pwrseq = to_pwrseq_simple(host->pwrseq);
	struct device *dev = host->pwrseq->dev;
	int i, ret = 0;

	pwrseq->ext_clk = clk_get(dev, "ext_clock");
	if (IS_ERR(pwrseq->ext_clk) &&
	    PTR_ERR(pwrseq->ext_clk) != -ENOENT) {
		return PTR_ERR(pwrseq->ext_clk);
	
	}

	for (i = 0; i < pwrseq->nr_gpios; i++) {
		pwrseq->reset_gpios[i] = gpiod_get_index(dev, "reset", i,
							 GPIOD_OUT_HIGH);
		if (IS_ERR(pwrseq->reset_gpios[i]) &&
		    PTR_ERR(pwrseq->reset_gpios[i]) != -ENOENT &&
		    PTR_ERR(pwrseq->reset_gpios[i]) != -ENOSYS) {
			ret = PTR_ERR(pwrseq->reset_gpios[i]);

			while (i--)
				gpiod_put(pwrseq->reset_gpios[i]);

			if (!IS_ERR(pwrseq->ext_clk))
				clk_put(pwrseq->ext_clk);

			return -EINVAL;
		}
	}


	return 0;
}

static struct mmc_pwrseq_ops mmc_pwrseq_simple_ops = {
	.alloc = mmc_pwrseq_simple_alloc,
	.pre_power_on = mmc_pwrseq_simple_pre_power_on,
	.post_power_on = mmc_pwrseq_simple_post_power_on,
	.power_off = mmc_pwrseq_simple_power_off,
	.free = mmc_pwrseq_simple_free,
};

static const struct of_device_id mmc_pwrseq_simple_of_match[] = {
	{ .compatible = "mmc-pwrseq-simple",},
	{/* sentinel */},
};
MODULE_DEVICE_TABLE(of, mmc_pwrseq_simple_of_match);

static int mmc_pwrseq_simple_probe(struct platform_device *pdev)
{
	struct mmc_pwrseq_simple *spwrseq;
	struct device *dev = &pdev->dev;
	int nr_gpios;

	nr_gpios = of_gpio_named_count(dev->of_node, "reset-gpios");
	if (nr_gpios < 0)
		nr_gpios = 0;

	spwrseq = devm_kzalloc(dev, sizeof(struct mmc_pwrseq_simple) + nr_gpios *
			 sizeof(struct gpio_desc *), GFP_KERNEL);
	if (!spwrseq)
		return -ENOMEM;

	spwrseq->pwrseq.dev = dev;
	spwrseq->nr_gpios = nr_gpios;
	spwrseq->pwrseq.ops = &mmc_pwrseq_simple_ops;

	platform_set_drvdata(pdev, spwrseq);

	return mmc_pwrseq_register(&spwrseq->pwrseq);
}

static int mmc_pwrseq_simple_remove(struct platform_device *pdev)
{
	struct mmc_pwrseq_simple *spwrseq = platform_get_drvdata(pdev);

	return mmc_pwrseq_unregister(&spwrseq->pwrseq);
}

static struct platform_driver mmc_pwrseq_simple_driver = {
	.probe = mmc_pwrseq_simple_probe,
	.remove = mmc_pwrseq_simple_remove,
	.driver = {
		.name = "pwrseq_simple",
		.of_match_table = mmc_pwrseq_simple_of_match,
	},
};

module_platform_driver(mmc_pwrseq_simple_driver);
MODULE_LICENSE("GPL v2");
