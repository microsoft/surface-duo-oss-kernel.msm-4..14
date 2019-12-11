// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 Linaro Limited, All rights reserved.
 * Author: Mike Leach <mike.leach@linaro.org>
 */

#include <linux/coresight.h>

#include "coresight-cti.h"

/* basic attributes */
static ssize_t enable_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	int enable_req;
	bool enabled, powered;
	struct cti_drvdata *drvdata = dev_get_drvdata(dev->parent);
	ssize_t size = 0;

	enable_req = atomic_read(&drvdata->config.enable_req_count);
	spin_lock(&drvdata->spinlock);
	powered = drvdata->config.hw_powered;
	enabled = drvdata->config.hw_enabled;
	spin_unlock(&drvdata->spinlock);

	if (powered) {
		size = scnprintf(buf, PAGE_SIZE, "cti %s; powered;\n",
				 enabled ? "enabled" : "disabled");
	} else {
		size = scnprintf(buf, PAGE_SIZE, "cti %s; unpowered;\n",
				 enable_req ? "enable req" : "disabled");
	}
	return size;
}

static ssize_t enable_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t size)
{
	int ret = 0;
	unsigned long val;
	struct cti_drvdata *drvdata = dev_get_drvdata(dev->parent);

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if (val)
		ret = cti_enable(drvdata->csdev);
	else
		ret = cti_disable(drvdata->csdev);
	if (ret)
		return ret;
	return size;
}
static DEVICE_ATTR_RW(enable);

/* attribute and group sysfs tables. */
static struct attribute *coresight_cti_attrs[] = {
	&dev_attr_enable.attr,
	NULL,
};

static const struct attribute_group coresight_cti_group = {
	.attrs = coresight_cti_attrs,
};

const struct attribute_group *coresight_cti_groups[] = {
	&coresight_cti_group,
	NULL,
};
