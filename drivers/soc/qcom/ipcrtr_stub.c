/*
 * Copyright (c) 2014, Sony Mobile Communications AB.
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/soc/qcom/smd.h>

static int ipcrtr_stub_callback(struct qcom_smd_device *qsdev,
				const void *data,
				 size_t count)
{
	print_hex_dump(KERN_DEBUG, "IPCRTR <<<: ", DUMP_PREFIX_OFFSET, 16, 1, data, count, true);

	return 0;
}

static int ipcrtr_stub_probe(struct qcom_smd_device *sdev)
{
	dev_err(&sdev->dev, "ipcrtr initialized\n");

	return 0;
}

static const struct of_device_id ipcrtr_stub_of_match[] = {
	{ .compatible = "qcom,ipcrtr" },
	{}
};
MODULE_DEVICE_TABLE(of, ipcrtr_stub_of_match);

static struct qcom_smd_driver ipcrtr_stub_driver = {
	.probe = ipcrtr_stub_probe,
	.callback = ipcrtr_stub_callback,
	.driver  = {
		.name  = "ipcrtr_stub",
		.owner = THIS_MODULE,
		.of_match_table = ipcrtr_stub_of_match,
	},
};

module_qcom_smd_driver(ipcrtr_stub_driver);

MODULE_AUTHOR("Bjorn Andersson <bjorn.andersson@sonymobile.com>");
MODULE_DESCRIPTION("Qualcomm IPCRTR stub driver");
MODULE_LICENSE("GPLv2");

