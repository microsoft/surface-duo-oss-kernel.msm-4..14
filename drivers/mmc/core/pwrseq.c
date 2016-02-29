/*
 *  Copyright (C) 2014 Linaro Ltd
 *
 * Author: Ulf Hansson <ulf.hansson@linaro.org>
 *
 * License terms: GNU General Public License (GPL) version 2
 *
 *  MMC power sequence management
 */
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>

#include <linux/mmc/host.h>

#include "pwrseq.h"

static DEFINE_MUTEX(pwrseq_list_mutex);
static LIST_HEAD(pwrseq_list);

static struct mmc_pwrseq *of_find_mmc_pwrseq(struct mmc_host *host)
{
	struct device_node *np;
	struct mmc_pwrseq *p, *pwrseq = NULL;

	np = of_parse_phandle(host->parent->of_node, "mmc-pwrseq", 0);
	if (!np)
		return NULL;

	mutex_lock(&pwrseq_list_mutex);
	list_for_each_entry(p, &pwrseq_list, list) {
		if (p->dev->of_node == np) {
			pwrseq = p;
			break;
		}
	}

	of_node_put(np);
	mutex_unlock(&pwrseq_list_mutex);

	return pwrseq ? : ERR_PTR(-EPROBE_DEFER);
}

int mmc_pwrseq_alloc(struct mmc_host *host)
{
	struct mmc_pwrseq *pwrseq;
	int ret = 0;

	pwrseq = of_find_mmc_pwrseq(host);

	if (IS_ERR_OR_NULL(pwrseq))
		return PTR_ERR(pwrseq);

	if (pwrseq->ops && pwrseq->ops->alloc) {
		host->pwrseq = pwrseq;
		ret = pwrseq->ops->alloc(host);

		if (IS_ERR_VALUE(ret)) {
			host->pwrseq = NULL;
			goto err;
		}
		try_module_get(pwrseq->owner);
	}

	dev_info(host->parent, "allocated mmc-pwrseq\n");

err:
	return ret;
}

void mmc_pwrseq_pre_power_on(struct mmc_host *host)
{
	struct mmc_pwrseq *pwrseq = host->pwrseq;

	if (pwrseq && pwrseq->ops->pre_power_on)
		pwrseq->ops->pre_power_on(host);
}

void mmc_pwrseq_post_power_on(struct mmc_host *host)
{
	struct mmc_pwrseq *pwrseq = host->pwrseq;

	if (pwrseq && pwrseq->ops->post_power_on)
		pwrseq->ops->post_power_on(host);
}

void mmc_pwrseq_power_off(struct mmc_host *host)
{
	struct mmc_pwrseq *pwrseq = host->pwrseq;

	if (pwrseq && pwrseq->ops->power_off)
		pwrseq->ops->power_off(host);
}

void mmc_pwrseq_free(struct mmc_host *host)
{
	struct mmc_pwrseq *pwrseq = host->pwrseq;

	if (pwrseq) {
		if (pwrseq->ops->free)
			pwrseq->ops->free(host);
		module_put(pwrseq->owner);

		host->pwrseq = NULL;
	}

}

int mmc_pwrseq_register(struct mmc_pwrseq *pwrseq)
{
	if (!pwrseq || !pwrseq->ops || !pwrseq->dev)
		return -EINVAL;

	mutex_lock(&pwrseq_list_mutex);
	list_add(&pwrseq->list, &pwrseq_list);
	mutex_unlock(&pwrseq_list_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(mmc_pwrseq_register);

void mmc_pwrseq_unregister(struct mmc_pwrseq *pwrseq)
{
	if (pwrseq) {
		mutex_lock(&pwrseq_list_mutex);
		list_del(&pwrseq->list);
		mutex_unlock(&pwrseq_list_mutex);
	}
}
EXPORT_SYMBOL_GPL(mmc_pwrseq_unregister);
